"""B03-R4D task-space end-effector tracking warm-start smoke runner.

本脚本把 R4C 的 joint-space state tracking 推进到 task-space 末端轨迹
tracking：状态仍是 `x=[q1,q2,dq1,dq2]`，控制仍是 `u=[tau1,tau2]`，
但 cost 改成末端位置误差 `||p_ee(q)-p_ref||`。

当前边界：
- 不修改 B02 controller / benchmark / regression；
- 不进入 task-space Jacobian、QP 或 WBC；
- iLQR-lite 对 task-space cost 使用有限差分二次近似，只用于短 horizon smoke。
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
import logging
from pathlib import Path
import sys
from typing import Any

import numpy as np
import yaml

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import seaborn as sns


SCRIPT_PATH = Path(__file__).resolve()
SIMULATOR_ROOT = SCRIPT_PATH.parents[1]
PROJECT_ROOT = SIMULATOR_ROOT.parent
REPO_ROOT = PROJECT_ROOT.parent.parent

if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

try:
    from projects.B_mujoco_mpc_study.simulator.envs.two_link_env import TwoLinkEnv
except ModuleNotFoundError as exc:
    if exc.name != "mujoco":
        raise
    TwoLinkEnv = Any  # type: ignore[misc, assignment]
    TWO_LINK_ENV_IMPORT_ERROR: ModuleNotFoundError | None = exc
else:
    TWO_LINK_ENV_IMPORT_ERROR = None

from projects.B_mujoco_mpc_study.simulator.planners.ilqg_solver import ILQGLiteSolver
from projects.B_mujoco_mpc_study.simulator.planners.mpc_solver_interface import (
    BaseMPCSolver,
    MPCProblem,
    MPCSolution,
)
from projects.B_mujoco_mpc_study.simulator.planners.sampling_mpc_solvers import (
    CEMShootingSolver,
    MPPILiteSolver,
)
from projects.B_mujoco_mpc_study.simulator.adapters.b02_to_b03_adapter import (
    B02AdapterConfig,
    rollout_cost_candidates,
)
from projects.B_mujoco_mpc_study.simulator.scripts import run_B03_ilqr_lite_two_link_smoke as r4c
from projects.B_mujoco_mpc_study.simulator.scripts import run_B03_sampling_to_ilqr_warm_start_smoke as r4c2b


LOGGER = logging.getLogger(__name__)

TASK_NAME = "B03_task_space_warm_start_smoke"
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "configs" / "B03_mpc_solver_ladder.yaml"
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "outputs" / "runs" / TASK_NAME


@dataclass(frozen=True)
class TaskSpaceRolloutResult:
    """给 sampling solver 消费的 task-space 批量 rollout 结果。"""

    costs: np.ndarray
    predicted_states: np.ndarray
    predicted_ee_positions: np.ndarray


@dataclass(frozen=True)
class SolverRunResult:
    """一次 solver 调用及其对应 problem。"""

    label: str
    problem: MPCProblem
    solution: MPCSolution


@dataclass(frozen=True)
class TaskSpaceComparisonResult:
    """R4D 三路末端 tracking 对照结果。"""

    sampling: SolverRunResult
    zero_init_ilqr: SolverRunResult
    warm_start_ilqr: SolverRunResult


def _as_finite_array(value: Any, name: str) -> np.ndarray:
    """把输入转成有限 float 数组，错误信息保留变量名。"""
    array = np.asarray(value, dtype=float)
    if not np.isfinite(array).all():
        raise ValueError(f"{name} must contain only finite values, got shape={array.shape}")
    return array


def _normalize_sampling_solver_family(sampling_solver_family: str) -> str:
    """统一 sampling solver 名称，兼容 CLI 简写。"""
    family = sampling_solver_family.strip().lower()
    aliases = {
        "cem": "cem",
        "cem_shooting": "cem",
        "mppi": "mppi_lite",
        "mppi_lite": "mppi_lite",
    }
    if family not in aliases:
        raise ValueError("sampling_solver_family must be one of: cem, mppi_lite")
    return aliases[family]


def build_sampling_solver(sampling_solver_family: str) -> BaseMPCSolver:
    """根据名称创建 sampling-family solver。"""
    family = _normalize_sampling_solver_family(sampling_solver_family)
    if family == "cem":
        return CEMShootingSolver()
    if family == "mppi_lite":
        return MPPILiteSolver()
    raise ValueError(f"Unsupported sampling solver family: {sampling_solver_family}")


def compute_end_effector_xy(env: Any, x: np.ndarray) -> np.ndarray:
    """在给定 two-link 状态上读取末端二维位置 `p_ee(q)`。

    输入是 `x=[q1,q2,dq1,dq2]`；输出是 shape=(2,) 的 `[x_ee,y_ee]`。
    这个 helper 会保存/恢复 env 状态，避免 cost 查询污染真实 rollout。
    """
    state = _as_finite_array(x, "x")
    r4c.validate_state_shape(state)
    get_ee_fn = getattr(env, "get_end_effector_position", None)
    if not callable(get_ee_fn):
        raise NotImplementedError("R4D needs env.get_end_effector_position() to compute p_ee(q).")

    snapshot = r4c.save_two_link_env_state(env)
    try:
        r4c.set_two_link_state(env, state)
        ee_xy = _as_finite_array(get_ee_fn(), "end_effector_position")
        if ee_xy.shape != (2,):
            raise ValueError(f"end effector xy must have shape (2,), got {ee_xy.shape}")
        return ee_xy.astype(float, copy=True)
    finally:
        r4c.restore_two_link_env_state(env, snapshot)


def build_task_space_reference(
    *,
    start_xy: np.ndarray,
    horizon: int,
    dt: float,
    config: dict[str, Any],
) -> np.ndarray:
    """构造最小末端二维参考轨迹 `p_ref[k]`。

    第一版支持 hold / line / circle 三种轻量 reference。`dt` 目前只用于
    明确时间尺度检查，后续可以扩展成基于真实时间的轨迹生成。
    """
    start = _as_finite_array(start_xy, "start_xy")
    if start.shape != (2,):
        raise ValueError(f"start_xy must have shape (2,), got {start.shape}")
    if horizon <= 0:
        raise ValueError(f"horizon must be positive, got {horizon}")
    if dt <= 0.0:
        raise ValueError(f"dt must be positive, got {dt}")

    task_config = dict(config.get("task_space", {}))
    reference_type = str(task_config.get("reference_type", "circle")).strip().lower()
    progress = np.arange(horizon + 1, dtype=float) / max(float(horizon), 1.0)

    if reference_type == "hold":
        return np.repeat(start[None, :], horizon + 1, axis=0)
    if reference_type == "line":
        offset = np.asarray(task_config.get("line_offset", [0.05, 0.02]), dtype=float)
        if offset.shape != (2,):
            raise ValueError(f"line_offset must have shape (2,), got {offset.shape}")
        return start[None, :] + progress[:, None] * offset[None, :]
    if reference_type == "circle":
        radius = float(task_config.get("reference_radius", 0.04))
        cycles = float(task_config.get("reference_cycles", 0.25))
        if radius < 0.0:
            raise ValueError(f"reference_radius must be non-negative, got {radius}")
        theta = 2.0 * np.pi * cycles * progress
        reference = np.zeros((horizon + 1, 2), dtype=float)
        reference[:, 0] = start[0] + radius * (np.cos(theta) - 1.0)
        reference[:, 1] = start[1] + radius * np.sin(theta)
        return reference

    raise ValueError("task_space.reference_type must be one of: hold, line, circle")


def _normalize_cost_profile(cost_profile: str) -> str:
    """统一 task-space cost profile 名称。"""
    profile = cost_profile.strip().lower()
    aliases = {
        "task_space": "task_space",
        "r4d": "task_space",
        "legacy": "legacy_ladder",
        "legacy_ladder": "legacy_ladder",
        "b03_ladder": "legacy_ladder",
    }
    if profile not in aliases:
        raise ValueError("cost_profile must be 'task_space' or 'legacy_ladder'")
    return aliases[profile]


def _task_space_weights(config: dict[str, Any], cost_profile: str = "task_space") -> tuple[np.ndarray, np.ndarray, float, float, float, bool]:
    """从 config 读取末端误差、速度和控制权重。

    `task_space` 是 R4D 当前口径，保留 0.5 * quadratic form。
    `legacy_ladder` 复用旧 B03 adapter 的 cost 口径，不额外乘 0.5，
    terminal 项也只惩罚末端位置误差，不重复惩罚速度。
    """
    profile = _normalize_cost_profile(cost_profile)
    if profile == "legacy_ladder":
        cost_config = dict(config.get("cost", {}))
        ee_weight = float(cost_config.get("ee_weight", 80.0))
        terminal_ee_weight = float(cost_config.get("terminal_weight", 10.0))
        velocity_weight = float(cost_config.get("dq_weight", 0.1))
        control_weight = float(cost_config.get("torque_weight", 0.002))
        quadratic_scale = 1.0
        include_terminal_velocity = False
    else:
        task_config = dict(config.get("task_space", {}))
        ee_weight = float(task_config.get("ee_weight", 80.0))
        terminal_ee_weight = float(task_config.get("terminal_ee_weight", 120.0))
        velocity_weight = float(task_config.get("velocity_weight", 0.2))
        control_weight = float(task_config.get("control_weight", 0.05))
        quadratic_scale = 0.5
        include_terminal_velocity = True
    if min(ee_weight, terminal_ee_weight, velocity_weight, control_weight) < 0.0:
        raise ValueError("task-space weights must be non-negative")
    return (
        np.eye(2, dtype=float) * ee_weight,
        np.eye(2, dtype=float) * terminal_ee_weight,
        velocity_weight,
        control_weight,
        quadratic_scale,
        include_terminal_velocity,
    )


def build_task_space_cost_functions(
    *,
    env: Any,
    target_ee_positions: np.ndarray,
    config: dict[str, Any],
    stage_cost_timing: str = "pre_step",
    cost_profile: str = "task_space",
) -> tuple[Any, Any, Any]:
    """构造 task-space stage/terminal cost 和 end_effector_fn。"""
    targets = _as_finite_array(target_ee_positions, "target_ee_positions")
    if targets.ndim != 2 or targets.shape[1] != 2:
        raise ValueError(f"target_ee_positions must have shape (H+1, 2), got {targets.shape}")
    timing = stage_cost_timing.strip().lower()
    if timing not in ("pre_step", "post_step"):
        raise ValueError("stage_cost_timing must be 'pre_step' or 'post_step'")
    Q_ee, Q_terminal_ee, velocity_weight, control_weight, quadratic_scale, include_terminal_velocity = _task_space_weights(
        config,
        cost_profile=cost_profile,
    )
    terminal_target_index = -2 if timing == "post_step" else -1

    def end_effector_fn(x: np.ndarray) -> np.ndarray:
        return compute_end_effector_xy(env, x)

    def stage_cost_fn(x: np.ndarray, u: np.ndarray, t: int) -> float:
        x_arr = _as_finite_array(x, "x")
        u_arr = _as_finite_array(u, "u")
        r4c.validate_state_shape(x_arr)
        r4c.validate_control_shape(u_arr)
        if t < 0 or t >= targets.shape[0] - 1:
            raise ValueError(f"stage index t out of range: {t}")
        ee_error = end_effector_fn(x_arr) - targets[t]
        dq = x_arr[2:]
        return float(
            quadratic_scale
            * (
                ee_error.T @ Q_ee @ ee_error
                + velocity_weight * dq.T @ dq
                + control_weight * u_arr.T @ u_arr
            )
        )

    def terminal_cost_fn(x: np.ndarray) -> float:
        x_arr = _as_finite_array(x, "x_terminal")
        r4c.validate_state_shape(x_arr)
        ee_error = end_effector_fn(x_arr) - targets[terminal_target_index]
        dq = x_arr[2:]
        terminal_cost = quadratic_scale * float(ee_error.T @ Q_terminal_ee @ ee_error)
        if include_terminal_velocity:
            terminal_cost += quadratic_scale * float(velocity_weight * dq.T @ dq)
        return float(terminal_cost)

    return stage_cost_fn, terminal_cost_fn, end_effector_fn


def evaluate_task_space_rollouts(candidate_controls: np.ndarray, problem: MPCProblem) -> TaskSpaceRolloutResult:
    """批量评估 task-space tracking 候选控制序列。"""
    controls = _as_finite_array(candidate_controls, "candidate_controls")
    if controls.ndim != 3:
        raise ValueError(f"candidate_controls must have shape (N, H, control_dim), got {controls.shape}")
    num_candidates, horizon, control_dim = controls.shape
    if horizon != int(problem.horizon) or control_dim != int(problem.control_dim):
        raise ValueError(
            "candidate_controls shape must match problem horizon/control_dim: "
            f"controls={controls.shape}, problem=({problem.horizon}, {problem.control_dim})"
        )

    dynamics_fn = problem.dynamics_fn or problem.metadata.get("dynamics_fn")
    stage_cost_fn = problem.metadata.get("stage_cost_fn")
    terminal_cost_fn = problem.metadata.get("terminal_cost_fn")
    end_effector_fn = problem.metadata.get("end_effector_fn")
    stage_cost_timing = str(problem.metadata.get("stage_cost_timing", "pre_step")).strip().lower()
    if not callable(dynamics_fn) or not callable(stage_cost_fn) or not callable(terminal_cost_fn) or not callable(end_effector_fn):
        raise ValueError("task-space problem requires dynamics_fn, stage_cost_fn, terminal_cost_fn, and end_effector_fn.")
    if stage_cost_timing not in ("pre_step", "post_step"):
        raise ValueError("stage_cost_timing must be 'pre_step' or 'post_step'")

    current_state = _as_finite_array(problem.current_state, "problem.current_state")
    r4c.validate_state_shape(current_state)
    predicted_states = np.zeros((num_candidates, horizon + 1, current_state.size), dtype=float)
    predicted_ee_positions = np.zeros((num_candidates, horizon + 1, 2), dtype=float)
    costs = np.zeros(num_candidates, dtype=float)

    for candidate_index in range(num_candidates):
        states = predicted_states[candidate_index]
        ee_positions = predicted_ee_positions[candidate_index]
        states[0] = current_state
        ee_positions[0] = _as_finite_array(end_effector_fn(states[0]), "ee_positions[0]")
        total_cost = 0.0
        for step_index in range(horizon):
            x_t = states[step_index]
            u_t = controls[candidate_index, step_index]
            x_next = _as_finite_array(dynamics_fn(x_t, u_t), f"dynamics_fn candidate={candidate_index} step={step_index}")
            r4c.validate_state_shape(x_next)
            states[step_index + 1] = x_next
            ee_positions[step_index + 1] = _as_finite_array(end_effector_fn(x_next), f"ee_positions candidate={candidate_index} step={step_index + 1}")
            if stage_cost_timing == "post_step":
                total_cost += float(stage_cost_fn(x_next, u_t, step_index))
            else:
                total_cost += float(stage_cost_fn(x_t, u_t, step_index))
        total_cost += float(terminal_cost_fn(states[-1]))
        costs[candidate_index] = total_cost

    return TaskSpaceRolloutResult(costs=costs, predicted_states=predicted_states, predicted_ee_positions=predicted_ee_positions)


def _make_legacy_ladder_rollout_cost_fn(
    *,
    env: Any,
    config: dict[str, Any],
    horizon: int,
    control_dim: int,
    solver_config: dict[str, Any],
) -> Any:
    """复用旧 B03 adapter 的 B02 rollout cost 后端。

    这个后端用于同口径复现实验：cost 使用旧 `cost.*` 权重，目标序列只取
    `H` 个执行后目标点，terminal 也对齐到第 `H` 步而不是 `H+1` 步。
    """
    cost_config = dict(config.get("cost", {}))
    adapter_config = B02AdapterConfig(
        horizon=int(horizon),
        dt=float(dict(config.get("simulation", {})).get("dt", 0.01)),
        control_dim=int(control_dim),
        torque_limit=float(solver_config.get("torque_limit", dict(config.get("sampling", {})).get("torque_limit", 2.0))),
        ee_weight=float(cost_config.get("ee_weight", 80.0)),
        dq_weight=float(cost_config.get("dq_weight", 0.1)),
        torque_weight=float(cost_config.get("torque_weight", 0.002)),
        terminal_weight=float(cost_config.get("terminal_weight", 10.0)),
        record_predicted_states=True,
        record_predicted_ee_positions=True,
    )

    def legacy_ladder_rollout_cost_fn(candidate_controls: np.ndarray, problem: MPCProblem) -> Any:
        targets = _as_finite_array(problem.target_horizon, "problem.target_horizon")
        if targets.shape != (problem.horizon + 1, 2):
            raise ValueError(
                "legacy_ladder rollout expects problem.target_horizon shape "
                f"{(problem.horizon + 1, 2)}, got {targets.shape}"
            )
        adapter_problem = MPCProblem(
            current_state=np.asarray(problem.current_state, dtype=float),
            target_horizon=targets[: problem.horizon],
            horizon=int(problem.horizon),
            control_dim=int(problem.control_dim),
            dt=float(problem.dt),
            cost_config={
                "ee_weight": adapter_config.ee_weight,
                "dq_weight": adapter_config.dq_weight,
                "torque_weight": adapter_config.torque_weight,
                "terminal_weight": adapter_config.terminal_weight,
            },
            solver_config=dict(problem.solver_config),
            rollout_cost_fn=None,
            dynamics_fn=problem.dynamics_fn,
            metadata=dict(problem.metadata),
        )
        return rollout_cost_candidates(candidate_controls, adapter_problem, env, adapter_config)

    return legacy_ladder_rollout_cost_fn


def _resolve_horizon(config: dict[str, Any], horizon: int | None) -> int:
    """从显式参数或 config 中解析短 horizon smoke 长度。"""
    if horizon is not None:
        resolved = int(horizon)
    elif "horizon" in dict(config.get("task_space", {})):
        resolved = int(dict(config.get("task_space", {}))["horizon"])
    else:
        resolved = min(int(dict(config.get("ilqg_lite", {})).get("horizon", 12)), 12)
    if resolved <= 0:
        raise ValueError(f"horizon must be positive, got {resolved}")
    return resolved


def build_task_space_tracking_problem(
    *,
    env: Any,
    config: dict[str, Any],
    horizon: int | None = None,
    solver_family: str = "ilqg_lite",
    previous_solution: MPCSolution | None = None,
    sampling_overrides: dict[str, Any] | None = None,
    target_ee_positions: np.ndarray | None = None,
    direct_initial_controls: np.ndarray | None = None,
    warm_start_source: str | None = None,
    warm_start_mode: str | None = None,
    stage_cost_timing: str = "pre_step",
    cost_profile: str = "task_space",
) -> MPCProblem:
    """构造最小 task-space end-effector tracking MPCProblem。"""
    resolved_horizon = _resolve_horizon(config, horizon)
    normalized_cost_profile = _normalize_cost_profile(cost_profile)
    current_state = r4c.get_two_link_state(env)
    r4c.validate_state_shape(current_state)
    dynamics_fn = r4c.build_two_link_dynamics_fn(env)
    dt = float(dict(config.get("simulation", {})).get("dt", getattr(env, "dt", 0.01)))
    if dt <= 0.0:
        raise ValueError(f"dt must be positive, got {dt}")

    if target_ee_positions is None:
        start_xy = compute_end_effector_xy(env, current_state)
        resolved_targets = build_task_space_reference(
            start_xy=start_xy,
            horizon=resolved_horizon,
            dt=dt,
            config=config,
        )
    else:
        resolved_targets = _as_finite_array(target_ee_positions, "target_ee_positions")
        if resolved_targets.shape != (resolved_horizon + 1, 2):
            raise ValueError(
                "target_ee_positions must have shape (horizon + 1, 2), "
                f"got {resolved_targets.shape} for horizon={resolved_horizon}"
            )
    stage_cost_fn, terminal_cost_fn, end_effector_fn = build_task_space_cost_functions(
        env=env,
        target_ee_positions=resolved_targets,
        config=config,
        stage_cost_timing=stage_cost_timing,
        cost_profile=cost_profile,
    )

    family = _normalize_sampling_solver_family(solver_family) if solver_family != "ilqg_lite" else "ilqg_lite"
    if family == "ilqg_lite":
        if direct_initial_controls is not None:
            # 同一控制步内 sampling -> iLQR 时，sampling 控制序列还没有被执行，
            # 因此这里必须原样作为 iLQR 初值，不能做 receding-horizon 左移。
            warm_start_controls = _as_finite_array(direct_initial_controls, "direct_initial_controls")
            if warm_start_controls.shape != (resolved_horizon, 2):
                raise ValueError(
                    "direct_initial_controls must have shape (horizon, 2), "
                    f"got {warm_start_controls.shape} for horizon={resolved_horizon}"
                )
            resolved_warm_start_mode = warm_start_mode or "direct_initial_controls"
            resolved_warm_start_source = warm_start_source or "direct_initial_controls"
        else:
            warm_start_controls = r4c.build_warm_start_controls_from_previous_solution(
                previous_solution=previous_solution,
                horizon=resolved_horizon,
                control_dim=2,
            )
            resolved_warm_start_mode = "previous_solution_shifted" if warm_start_controls is not None else "zeros"
            resolved_warm_start_source = getattr(previous_solution, "solver_name", "previous_solution") if warm_start_controls is not None else "zeros"
        warm_start_used = warm_start_controls is not None
        initial_controls = warm_start_controls if warm_start_used else np.zeros((resolved_horizon, 2), dtype=float)
        solver_config = dict(config.get("ilqg_lite", {}))
        solver_config.setdefault("horizon", resolved_horizon)
    else:
        warm_start_used = False
        initial_controls = np.zeros((resolved_horizon, 2), dtype=float)
        resolved_warm_start_mode = "zeros"
        resolved_warm_start_source = "zeros"
        solver_config = r4c2b.build_sampling_solver_config(
            config=config,
            sampling_solver_family=family,
            horizon=resolved_horizon,
            control_dim=2,
            sampling_overrides=sampling_overrides,
        )

    metadata = {
        "task_name": "B03-R4D_task_space_tracking",
        "tracking_space": "task_space_xy",
        "state_convention": "x=[q1,q2,dq1,dq2]",
        "control_convention": "u=[tau1,tau2]",
        "target_ee_positions": resolved_targets,
        "initial_controls": initial_controls,
        "dynamics_fn": dynamics_fn,
        "stage_cost_fn": stage_cost_fn,
        "terminal_cost_fn": terminal_cost_fn,
        "end_effector_fn": end_effector_fn,
        "stage_cost_timing": stage_cost_timing,
        "cost_profile": normalized_cost_profile,
        "warm_start_used": bool(warm_start_used),
        "warm_start_source": resolved_warm_start_source,
        "warm_start_mode": resolved_warm_start_mode,
        "sampling_solver_family": None if family == "ilqg_lite" else family,
    }
    if family == "ilqg_lite":
        rollout_cost_fn = None
        rollout_cost_backend = "none"
    elif (
        normalized_cost_profile == "legacy_ladder"
        and stage_cost_timing.strip().lower() == "post_step"
        and callable(getattr(env, "rollout", None))
        and callable(getattr(env, "set_state", None))
        and callable(getattr(env, "get_end_effector_position", None))
    ):
        rollout_cost_fn = _make_legacy_ladder_rollout_cost_fn(
            env=env,
            config=config,
            horizon=resolved_horizon,
            control_dim=2,
            solver_config=solver_config,
        )
        rollout_cost_backend = "b02_adapter_legacy_ladder"
    else:
        rollout_cost_fn = evaluate_task_space_rollouts
        rollout_cost_backend = "r4d_task_space"
    metadata["rollout_cost_backend"] = rollout_cost_backend
    cost_config = dict(config.get("cost", {})) if normalized_cost_profile == "legacy_ladder" else dict(config.get("task_space", {}))

    return MPCProblem(
        current_state=np.asarray(current_state, dtype=float).copy(),
        target_horizon=resolved_targets,
        horizon=resolved_horizon,
        control_dim=2,
        dt=dt,
        cost_config=cost_config,
        solver_config=solver_config,
        rollout_cost_fn=rollout_cost_fn,
        dynamics_fn=dynamics_fn,
        metadata=metadata,
    )


def run_task_space_warm_start_comparison(
    *,
    env: Any,
    config: dict[str, Any],
    horizon: int | None = None,
    sampling_solver_family: str = "cem",
    sampling_overrides: dict[str, Any] | None = None,
) -> TaskSpaceComparisonResult:
    """运行 sampling、zero-init iLQR、sampling-warm-start iLQR 三路末端 tracking 对照。"""
    family = _normalize_sampling_solver_family(sampling_solver_family)
    initial_snapshot = r4c.save_two_link_env_state(env)
    try:
        r4c.restore_two_link_env_state(env, initial_snapshot)
        sampling_problem = build_task_space_tracking_problem(
            env=env,
            config=config,
            horizon=horizon,
            solver_family=family,
            sampling_overrides=sampling_overrides,
        )
        sampling_solution = build_sampling_solver(family).solve(sampling_problem)

        r4c.restore_two_link_env_state(env, initial_snapshot)
        zero_problem = build_task_space_tracking_problem(
            env=env,
            config=config,
            horizon=horizon,
            solver_family="ilqg_lite",
            previous_solution=None,
        )
        zero_solution = ILQGLiteSolver().solve(zero_problem)

        r4c.restore_two_link_env_state(env, initial_snapshot)
        warm_problem = build_task_space_tracking_problem(
            env=env,
            config=config,
            horizon=horizon,
            solver_family="ilqg_lite",
            previous_solution=sampling_solution,
        )
        warm_solution = ILQGLiteSolver().solve(warm_problem, previous_solution=sampling_solution)
    finally:
        r4c.restore_two_link_env_state(env, initial_snapshot)

    return TaskSpaceComparisonResult(
        sampling=SolverRunResult(label=f"sampling_{family}", problem=sampling_problem, solution=sampling_solution),
        zero_init_ilqr=SolverRunResult(label="zero_init_ilqr", problem=zero_problem, solution=zero_solution),
        warm_start_ilqr=SolverRunResult(label="sampling_warm_start_ilqr", problem=warm_problem, solution=warm_solution),
    )


def ensure_output_dirs(output_dir: Path) -> dict[str, Path]:
    """创建 R4D 输出目录。"""
    root = Path(output_dir)
    output_paths = {
        "cache": root / "outputs" / "cache",
        "figures": root / "outputs" / "figures",
        "reports": root / "outputs" / "reports",
        "metrics": root / "outputs" / "metrics",
    }
    for path in output_paths.values():
        path.mkdir(parents=True, exist_ok=True)
    return output_paths


def build_output_paths(output_dir: Path | None) -> dict[str, Path]:
    """规划 R4D CSV / NPZ / PNG / Markdown 输出路径。"""
    run_dir = output_dir or DEFAULT_OUTPUT_ROOT / datetime.now().strftime("%Y%m%d_%H%M%S")
    base = ensure_output_dirs(run_dir)
    return {
        "run_dir": run_dir,
        **base,
        "metrics_csv": run_dir / "outputs" / "metrics" / "B03_R4D_task_space_metrics.csv",
        "cache_npz": run_dir / "outputs" / "cache" / "B03_R4D_task_space_cache.npz",
        "report_md": run_dir / "outputs" / "reports" / "B03_R4D_task_space_report.md",
        "ee_trajectory_figure": run_dir / "outputs" / "figures" / "B03_R4D_ee_trajectory_xy.png",
        "ee_error_figure": run_dir / "outputs" / "figures" / "B03_R4D_ee_tracking_error.png",
        "control_figure": run_dir / "outputs" / "figures" / "B03_R4D_control_sequence.png",
        "cost_runtime_figure": run_dir / "outputs" / "figures" / "B03_R4D_cost_runtime_comparison.png",
    }


def _relative_output_path(path: Path, run_dir: Path) -> str:
    """把输出路径转成相对 run_dir 的文本。"""
    try:
        return path.relative_to(run_dir).as_posix()
    except ValueError:
        return path.as_posix()


def _safe_predicted_ee(solution: MPCSolution, label: str) -> np.ndarray:
    """读取 solver 输出中的 predicted_ee_positions。"""
    if solution.predicted_ee_positions is None:
        raise ValueError(f"{label} solution is missing predicted_ee_positions")
    ee = _as_finite_array(solution.predicted_ee_positions, f"{label}.predicted_ee_positions")
    if ee.ndim != 2 or ee.shape[1] != 2:
        raise ValueError(f"{label}.predicted_ee_positions must have shape (H+1, 2), got {ee.shape}")
    return ee


def _metrics_row(run: SolverRunResult) -> dict[str, Any]:
    """把一次 solver run 转成 CSV/report 行。"""
    target = _as_finite_array(run.problem.metadata["target_ee_positions"], "target_ee_positions")
    predicted_ee = _safe_predicted_ee(run.solution, run.label)
    errors = np.linalg.norm(predicted_ee - target, axis=1)
    controls = _as_finite_array(run.solution.predicted_controls, f"{run.label}.predicted_controls")
    initial_controls = _as_finite_array(run.problem.metadata.get("initial_controls"), f"{run.label}.initial_controls")
    stats = run.solution.solver_stats
    cost_history = list(run.solution.metadata.get("cost_history", [run.solution.best_cost]))
    return {
        "label": run.label,
        "solver_name": run.solution.solver_name,
        "best_cost": float(run.solution.best_cost),
        "initial_cost": float(cost_history[0]),
        "final_cost": float(cost_history[-1]),
        "runtime_ms": float(stats.runtime_ms),
        "num_rollouts": int(stats.num_rollouts),
        "num_iterations": int(stats.num_iterations),
        "final_ee_error": float(errors[-1]),
        "mean_ee_error": float(np.mean(errors)),
        "max_ee_error": float(np.max(errors)),
        "control_energy": float(np.sum(controls * controls)),
        "initial_control_energy": float(np.sum(initial_controls * initial_controls)),
        "warm_start_used": bool(run.problem.metadata.get("warm_start_used", False)),
        "warm_start_source": str(run.problem.metadata.get("warm_start_source", "zeros")),
        "horizon": int(run.problem.horizon),
        "dt": float(run.problem.dt),
    }


def _configure_axis(ax: Any) -> None:
    """统一 R4D 图的坐标轴样式。"""
    ax.grid(False)
    ax.tick_params(axis="both", which="both", length=0, labelcolor="dimgrey")
    ax.patch.set_edgecolor("lightgrey")
    ax.patch.set_linewidth(0.8)


def plot_ee_trajectory(comparison: TaskSpaceComparisonResult, output_path: Path) -> None:
    """绘制末端 XY 轨迹：target vs 三路 solver。"""
    sns.set_theme(font_scale=1.0, style="whitegrid", font="DejaVu Sans")
    target = _as_finite_array(comparison.zero_init_ilqr.problem.metadata["target_ee_positions"], "target_ee_positions")
    series = [
        ("target", target, "dimgrey", "-", 2.2),
        ("sampling", _safe_predicted_ee(comparison.sampling.solution, "sampling"), "#72a5b4", ":", 1.8),
        ("zero-init iLQR", _safe_predicted_ee(comparison.zero_init_ilqr.solution, "zero"), "#4575b4", "--", 1.8),
        ("warm-start iLQR", _safe_predicted_ee(comparison.warm_start_ilqr.solution, "warm"), "#d73027", "-", 1.8),
    ]
    fig, ax = plt.subplots(figsize=(7.4, 6.0), dpi=150)
    for label, values, color, linestyle, linewidth in series:
        ax.plot(values[:, 0], values[:, 1], label=label, color=color, linestyle=linestyle, linewidth=linewidth)
        ax.scatter(values[0, 0], values[0, 1], color=color, s=28, edgecolors="white", linewidths=0.7, zorder=4)
        ax.scatter(values[-1, 0], values[-1, 1], color=color, s=42, marker="s", edgecolors="white", linewidths=0.7, zorder=4)
    ax.set_title("B03-R4D End-Effector XY Tracking", loc="left", fontsize=13, color="dimgrey")
    ax.set_xlabel("x_ee", fontsize=10, color="dimgrey")
    ax.set_ylabel("y_ee", fontsize=10, color="dimgrey")
    ax.axis("equal")
    _configure_axis(ax)
    ax.legend(frameon=True, facecolor="white", framealpha=0.8, edgecolor="lightgrey", labelcolor="dimgrey", fontsize=8)
    sns.despine(left=True, bottom=True)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_ee_error(comparison: TaskSpaceComparisonResult, output_path: Path) -> None:
    """绘制每一步末端 tracking error。"""
    sns.set_theme(font_scale=1.0, style="whitegrid", font="DejaVu Sans")
    target = _as_finite_array(comparison.zero_init_ilqr.problem.metadata["target_ee_positions"], "target_ee_positions")
    runs = [comparison.sampling, comparison.zero_init_ilqr, comparison.warm_start_ilqr]
    styles = {
        "sampling_cem": ("#72a5b4", ":"),
        "sampling_mppi_lite": ("#72a5b4", ":"),
        "zero_init_ilqr": ("#4575b4", "--"),
        "sampling_warm_start_ilqr": ("#d73027", "-"),
    }
    fig, ax = plt.subplots(figsize=(8.8, 4.8), dpi=150)
    steps = np.arange(target.shape[0])
    for run in runs:
        ee = _safe_predicted_ee(run.solution, run.label)
        errors = np.linalg.norm(ee - target, axis=1)
        color, linestyle = styles.get(run.label, ("#446485", "-"))
        ax.plot(steps, errors, label=run.label, color=color, linestyle=linestyle, linewidth=1.9)
        ax.text(steps[-1], errors[-1], f" {errors[-1]:.2e}", color="dimgrey", fontsize=8, va="center")
    ax.set_title("B03-R4D End-Effector Tracking Error", loc="left", fontsize=13, color="dimgrey")
    ax.set_xlabel("step", fontsize=10, color="dimgrey")
    ax.set_ylabel("||p_ee - p_ref||", fontsize=10, color="dimgrey")
    _configure_axis(ax)
    ax.legend(frameon=True, facecolor="white", framealpha=0.8, edgecolor="lightgrey", labelcolor="dimgrey", fontsize=8)
    sns.despine(left=True, bottom=True)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_control_sequence(comparison: TaskSpaceComparisonResult, output_path: Path) -> None:
    """绘制 tau1/tau2 控制序列。"""
    sns.set_theme(font_scale=1.0, style="whitegrid", font="DejaVu Sans")
    runs = [comparison.sampling, comparison.zero_init_ilqr, comparison.warm_start_ilqr]
    fig, axes = plt.subplots(1, 2, figsize=(12.0, 4.8), dpi=150, sharex=True)
    styles = {
        "sampling_cem": ("#72a5b4", ":"),
        "sampling_mppi_lite": ("#72a5b4", ":"),
        "zero_init_ilqr": ("#4575b4", "--"),
        "sampling_warm_start_ilqr": ("#d73027", "-"),
    }
    for control_index, ax in enumerate(axes):
        ax.axhline(0.0, color="lightgrey", linewidth=0.8, zorder=1)
        for run in runs:
            controls = _as_finite_array(run.solution.predicted_controls, f"{run.label}.predicted_controls")
            color, linestyle = styles.get(run.label, ("#446485", "-"))
            ax.step(np.arange(controls.shape[0]), controls[:, control_index], where="post", label=run.label, color=color, linestyle=linestyle, linewidth=1.8)
        ax.set_title(r"$\bf{(" + chr(ord("a") + control_index) + r")}$" + f"  tau{control_index + 1}", loc="left", fontsize=11, color="dimgrey")
        ax.set_xlabel("control step", fontsize=10, color="dimgrey")
        if control_index == 0:
            ax.set_ylabel("torque", fontsize=10, color="dimgrey")
        _configure_axis(ax)
    axes[0].legend(frameon=True, facecolor="white", framealpha=0.8, edgecolor="lightgrey", labelcolor="dimgrey", fontsize=8, loc="upper right")
    fig.suptitle("B03-R4D Control Sequence", fontsize=14, color="dimgrey", y=0.99)
    sns.despine(left=True, bottom=True)
    fig.tight_layout(rect=(0.02, 0.02, 0.99, 0.94))
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_cost_runtime(rows: list[dict[str, Any]], output_path: Path) -> None:
    """绘制 R4D solver cost/runtime/error 指标对比。"""
    sns.set_theme(font_scale=1.0, style="whitegrid", font="DejaVu Sans")
    labels = [str(row["label"]).replace("_", "\n") for row in rows]
    colors = ["#72a5b4", "#4575b4", "#d73027"][: len(rows)]
    metric_specs = [
        ("best_cost", "Best cost", ".2e"),
        ("runtime_ms", "Runtime (ms)", ".1f"),
        ("final_ee_error", "Final EE error", ".2e"),
        ("mean_ee_error", "Mean EE error", ".2e"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(12.0, 7.2), dpi=150)
    for axis_index, (ax, (key, title, fmt)) in enumerate(zip(axes.ravel(), metric_specs)):
        values = np.asarray([float(row[key]) for row in rows], dtype=float)
        x_positions = np.arange(values.size)
        ax.bar(x_positions, values, color=colors, alpha=0.82, width=0.62)
        top = max(float(np.max(values)), 1.0e-12)
        for x_pos, value in zip(x_positions, values):
            ax.text(x_pos, max(float(value), top * 0.02), f"{value:{fmt}}", ha="center", va="bottom", fontsize=8, color="dimgrey", weight="semibold")
        ax.set_xticks(x_positions)
        ax.set_xticklabels(labels, fontsize=8)
        ax.set_title(r"$\bf{(" + chr(ord("a") + axis_index) + r")}$" + f"  {title}", loc="left", fontsize=11, color="dimgrey")
        _configure_axis(ax)
    fig.suptitle("B03-R4D Cost / Runtime / Error Comparison", fontsize=14, color="dimgrey", y=0.99)
    sns.despine(left=True, bottom=True)
    fig.tight_layout(rect=(0.02, 0.02, 0.99, 0.94))
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def write_task_space_outputs(
    *,
    comparison: TaskSpaceComparisonResult,
    output_paths: dict[str, Path],
) -> None:
    """写入 R4D CSV / NPZ / PNG / Markdown report。"""
    runs = [comparison.sampling, comparison.zero_init_ilqr, comparison.warm_start_ilqr]
    rows = [_metrics_row(run) for run in runs]

    with output_paths["metrics_csv"].open("w", encoding="utf-8", newline="") as metrics_file:
        writer = csv.DictWriter(metrics_file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    target = _as_finite_array(comparison.zero_init_ilqr.problem.metadata["target_ee_positions"], "target_ee_positions")
    np.savez(
        output_paths["cache_npz"],
        target_ee_positions=target,
        sampling_predicted_states=np.asarray(comparison.sampling.solution.predicted_states, dtype=float),
        sampling_predicted_controls=np.asarray(comparison.sampling.solution.predicted_controls, dtype=float),
        sampling_predicted_ee_positions=_safe_predicted_ee(comparison.sampling.solution, "sampling"),
        zero_init_ilqr_predicted_states=np.asarray(comparison.zero_init_ilqr.solution.predicted_states, dtype=float),
        zero_init_ilqr_predicted_controls=np.asarray(comparison.zero_init_ilqr.solution.predicted_controls, dtype=float),
        zero_init_ilqr_predicted_ee_positions=_safe_predicted_ee(comparison.zero_init_ilqr.solution, "zero"),
        warm_start_ilqr_predicted_states=np.asarray(comparison.warm_start_ilqr.solution.predicted_states, dtype=float),
        warm_start_ilqr_predicted_controls=np.asarray(comparison.warm_start_ilqr.solution.predicted_controls, dtype=float),
        warm_start_ilqr_initial_controls=np.asarray(comparison.warm_start_ilqr.problem.metadata["initial_controls"], dtype=float),
        warm_start_ilqr_predicted_ee_positions=_safe_predicted_ee(comparison.warm_start_ilqr.solution, "warm"),
    )

    plot_ee_trajectory(comparison, output_paths["ee_trajectory_figure"])
    plot_ee_error(comparison, output_paths["ee_error_figure"])
    plot_control_sequence(comparison, output_paths["control_figure"])
    plot_cost_runtime(rows, output_paths["cost_runtime_figure"])

    run_dir = output_paths["run_dir"]
    report_lines = [
        "# B03-R4D Task-Space Warm-Start Smoke Report",
        "",
        "## Summary",
        "",
        "This smoke run tracks a two-link end-effector XY reference with sampling, zero-init iLQR-lite, and sampling-warm-start iLQR-lite.",
        "",
        "## Metrics",
        "",
        "| label | solver | warm_start | best_cost | runtime_ms | final_ee_error | mean_ee_error | max_ee_error | iterations | rollouts |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        report_lines.append(
            "| {label} | {solver_name} | {warm_start_used} | {best_cost:.6g} | {runtime_ms:.6g} | "
            "{final_ee_error:.6g} | {mean_ee_error:.6g} | {max_ee_error:.6g} | {num_iterations} | {num_rollouts} |".format(**row)
        )
    report_lines.extend(
        [
            "",
            "## Output Files",
            "",
            f"- metrics_csv: `{_relative_output_path(output_paths['metrics_csv'], run_dir)}`",
            f"- cache_npz: `{_relative_output_path(output_paths['cache_npz'], run_dir)}`",
            f"- ee_trajectory_figure: `{_relative_output_path(output_paths['ee_trajectory_figure'], run_dir)}`",
            f"- ee_error_figure: `{_relative_output_path(output_paths['ee_error_figure'], run_dir)}`",
            f"- control_figure: `{_relative_output_path(output_paths['control_figure'], run_dir)}`",
            f"- cost_runtime_figure: `{_relative_output_path(output_paths['cost_runtime_figure'], run_dir)}`",
            "",
            "## Current Scope",
            "",
            "- Tracks task-space XY position `p_ee(q)=[x_ee,y_ee]`.",
            "- Uses finite-difference cost quadratization for iLQR-lite task-space cost.",
            "- Does not modify B02 controller / benchmark / regression.",
            "- Does not implement task-space Jacobian, QP, WBC, or humanoid control.",
        ]
    )
    output_paths["report_md"].write_text("\n".join(report_lines), encoding="utf-8")

    LOGGER.info("R4D metrics written: %s", output_paths["metrics_csv"])
    LOGGER.info("R4D cache written: %s", output_paths["cache_npz"])
    LOGGER.info("R4D figures written: %s", output_paths["figures"])
    LOGGER.info("R4D report written: %s", output_paths["report_md"])


def load_yaml_config(config_path: Path) -> dict[str, Any]:
    """读取 YAML 配置。"""
    if not config_path.exists():
        raise FileNotFoundError(f"找不到配置文件: {config_path}")
    with config_path.open("r", encoding="utf-8") as config_file:
        loaded = yaml.safe_load(config_file) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"配置文件必须是 YAML mapping，当前类型为: {type(loaded)!r}")
    return loaded


def setup_two_link_env(config: dict[str, Any]) -> Any:
    """按配置创建 two-link env。"""
    if TWO_LINK_ENV_IMPORT_ERROR is not None:
        raise TWO_LINK_ENV_IMPORT_ERROR
    simulation_config = dict(config.get("simulation", {}))
    model_path = Path(simulation_config.get("model_path", r4c.DEFAULT_TWO_LINK_MODEL_PATH))
    if not model_path.is_absolute():
        model_path = (PROJECT_ROOT / model_path).resolve()
    if not model_path.exists():
        model_path = r4c.DEFAULT_TWO_LINK_MODEL_PATH
    dt = float(simulation_config.get("dt", 0.01))
    end_effector_site = str(simulation_config.get("end_effector_site", "ee_site"))
    env = TwoLinkEnv(model_path=model_path, dt=dt, end_effector_site=end_effector_site)
    initial_q = simulation_config.get("initial_q", [0.3, 0.4])
    initial_dq = simulation_config.get("initial_dq", [0.0, 0.0])
    env.reset(q=(float(initial_q[0]), float(initial_q[1])), dq=(float(initial_dq[0]), float(initial_dq[1])))
    LOGGER.info("Two-link task-space env: model=%s dt=%s site=%s", model_path, dt, end_effector_site)
    return env


def build_arg_parser() -> argparse.ArgumentParser:
    """创建命令行参数。"""
    parser = argparse.ArgumentParser(description="B03-R4D task-space warm-start smoke.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH, help="YAML 配置路径。")
    parser.add_argument("--output-dir", type=Path, default=None, help="输出根目录。")
    parser.add_argument("--sampling-solver", choices=("cem", "mppi_lite"), default="cem", help="sampling warm-start 来源。")
    parser.add_argument("--horizon", type=int, default=None, help="覆盖 horizon。")
    parser.add_argument("--num-candidates", type=int, default=None, help="覆盖 sampling 候选数。")
    parser.add_argument("--sampling-iterations", type=int, default=None, help="覆盖 CEM/MPPI 迭代数。")
    parser.add_argument("--sampling-std", type=float, default=None, help="覆盖 sampling std。")
    parser.add_argument("--torque-limit", type=float, default=None, help="覆盖 torque limit。")
    parser.add_argument("--seed", type=int, default=None, help="覆盖 sampling seed。")
    parser.add_argument("--log-level", choices=("INFO", "DEBUG"), default="INFO", help="日志等级。")
    return parser


def _sampling_overrides_from_args(args: argparse.Namespace) -> dict[str, Any]:
    """把 CLI 覆盖项转成 sampling solver_config。"""
    overrides: dict[str, Any] = {}
    if args.num_candidates is not None:
        overrides["num_candidates"] = int(args.num_candidates)
    if args.sampling_iterations is not None:
        overrides["num_iterations"] = int(args.sampling_iterations)
    if args.sampling_std is not None:
        overrides["sampling_std"] = float(args.sampling_std)
        overrides["initial_std"] = float(args.sampling_std)
        overrides["noise_std"] = float(args.sampling_std)
    if args.torque_limit is not None:
        overrides["torque_limit"] = float(args.torque_limit)
    if args.seed is not None:
        overrides["seed"] = int(args.seed)
    return overrides


def configure_logging(log_level: str) -> None:
    """配置日志。"""
    logging.basicConfig(level=getattr(logging, log_level), format="%(levelname)s %(name)s: %(message)s")


def main() -> int:
    """命令行入口。"""
    args = build_arg_parser().parse_args()
    configure_logging(args.log_level)
    config = load_yaml_config(args.config)
    env = setup_two_link_env(config)
    comparison = run_task_space_warm_start_comparison(
        env=env,
        config=config,
        horizon=args.horizon,
        sampling_solver_family=args.sampling_solver,
        sampling_overrides=_sampling_overrides_from_args(args),
    )
    output_paths = build_output_paths(args.output_dir)
    write_task_space_outputs(comparison=comparison, output_paths=output_paths)

    for run in [comparison.sampling, comparison.zero_init_ilqr, comparison.warm_start_ilqr]:
        row = _metrics_row(run)
        LOGGER.info(
            "%s: best_cost=%.6g final_ee_error=%.6g mean_ee_error=%.6g runtime_ms=%.3f warm_start=%s source=%s",
            row["label"],
            row["best_cost"],
            row["final_ee_error"],
            row["mean_ee_error"],
            row["runtime_ms"],
            row["warm_start_used"],
            row["warm_start_source"],
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
