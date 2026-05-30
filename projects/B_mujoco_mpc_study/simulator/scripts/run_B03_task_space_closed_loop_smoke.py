"""B03-R4D-6 closed-loop task-space MPC rollout smoke.

R4D-1~5 验证的是一次 open-loop solve；本脚本验证真正的 MPC 闭环结构：
每个控制步读取真实状态、生成未来末端参考轨迹、求解 MPCProblem、只执行
`first_control`，然后记录真实 `actual_ee` 相对 `target_ee` 的误差。

轨迹来源优先复用项目已有定义：
- `b03_lissajous`: 使用 B03 solver ladder 的 lissajous 轨迹；
- `b02_figure8` / `b02_circle` / `b02_sinusoidal`: 使用 B02 tracking 配置风格。
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
import logging
import math
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

from projects.B_mujoco_mpc_study.simulator.planners.ilqg_solver import ILQGLiteSolver
from projects.B_mujoco_mpc_study.simulator.planners.mpc_solver_interface import BaseMPCSolver, MPCSolution
from projects.B_mujoco_mpc_study.simulator.planners.sampling_mpc_solvers import (
    CEMShootingSolver,
    MPPILiteSolver,
    RandomShootingSolver,
    WarmStartSamplingSolver,
)
from projects.B_mujoco_mpc_study.simulator.scripts import run_B03_ilqr_lite_two_link_smoke as r4c
from projects.B_mujoco_mpc_study.simulator.scripts import run_B03_task_space_warm_start_smoke as r4d


LOGGER = logging.getLogger(__name__)

TASK_NAME = "B03_task_space_closed_loop_smoke"
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "configs" / "B03_mpc_solver_ladder.yaml"
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "outputs" / "runs" / TASK_NAME
DEFAULT_SMOKE_NUM_STEPS = 8
DEFAULT_SMOKE_HORIZON = 4
DEFAULT_SMOKE_NUM_CANDIDATES = 8
DEFAULT_SMOKE_SAMPLING_ITERATIONS = 1
DEFAULT_SMOKE_SAMPLING_STD = 0.1
DEFAULT_SMOKE_TORQUE_LIMIT = 1.0
DEFAULT_SMOKE_SEED = 4
TWO_LINK_LENGTH_1 = 0.45
TWO_LINK_LENGTH_2 = 0.35


@dataclass(frozen=True)
class ClosedLoopResult:
    """R4D-6 闭环 rollout 结果。"""

    target_source: str
    solver_family: str
    step_rows: np.ndarray
    actual_states: np.ndarray
    actual_ee_positions: np.ndarray
    target_ee_positions: np.ndarray
    executed_controls: np.ndarray
    predicted_ee_positions: np.ndarray
    summary_metrics: dict[str, float]


def _as_finite_array(value: Any, name: str) -> np.ndarray:
    """把输入转成有限 float 数组。"""
    array = np.asarray(value, dtype=float)
    if not np.isfinite(array).all():
        raise ValueError(f"{name} must contain only finite values, got shape={array.shape}")
    return array


def _as_pair(value: Any, name: str) -> tuple[float, float]:
    """读取二维配置。"""
    array = _as_finite_array(value, name)
    if array.shape != (2,):
        raise ValueError(f"{name} must have shape (2,), got {array.shape}")
    return float(array[0]), float(array[1])


def _normalize_solver_family(solver_family: str) -> str:
    """统一 closed-loop solver 名称。"""
    family = solver_family.strip().lower()
    aliases = {
        "cem": "cem",
        "random": "random_shooting",
        "random_shooting": "random_shooting",
        "warm_start_sampling": "warm_start_sampling",
        "mppi": "mppi_lite",
        "mppi_lite": "mppi_lite",
        "ilqg": "ilqg_lite",
        "ilqg_lite": "ilqg_lite",
        "sampling_warm_start_ilqg": "sampling_warm_start_ilqg",
        "sampling_warm_start_ilqr": "sampling_warm_start_ilqg",
    }
    if family not in aliases:
        raise ValueError(
            "solver_family must be one of: random_shooting, warm_start_sampling, "
            "cem, mppi_lite, ilqg_lite, sampling_warm_start_ilqg"
        )
    return aliases[family]


def build_solver(solver_family: str) -> BaseMPCSolver:
    """创建闭环每一步使用的 solver。"""
    family = _normalize_solver_family(solver_family)
    if family == "random_shooting":
        return RandomShootingSolver()
    if family == "warm_start_sampling":
        return WarmStartSamplingSolver()
    if family == "cem":
        return CEMShootingSolver()
    if family == "mppi_lite":
        return MPPILiteSolver()
    if family in ("ilqg_lite", "sampling_warm_start_ilqg"):
        return ILQGLiteSolver()
    raise ValueError(f"Unsupported solver_family: {solver_family}")


def _target_config(config: dict[str, Any], target_source: str) -> dict[str, Any]:
    """按 source 解析已有 B02/B03 轨迹配置。"""
    source = target_source.strip().lower()
    if source.startswith("b03"):
        target = dict(config.get("target", {}))
        return {
            "type": target.get("type", "lissajous"),
            "center": _as_pair(target.get("center", [0.45, 0.10]), "target.center"),
            "radius": float(target.get("radius", 0.08)),
            "frequency": float(target.get("frequency", 0.2)),
            "radius_x": target.get("radius_x"),
            "radius_y": target.get("radius_y"),
            "frequency_x": target.get("frequency_x"),
            "frequency_y": target.get("frequency_y"),
            "phase_offset": float(target.get("phase_offset", 0.0)),
        }

    b02_target = dict(config.get("target_trajectory", config.get("target", {})))
    source_type = source.replace("b02_", "")
    return {
        "type": source_type if source.startswith("b02_") else b02_target.get("type", "figure8"),
        "center": _as_pair(b02_target.get("center", [0.58, 0.28]), "target_trajectory.center"),
        "radius": float(b02_target.get("radius", 0.08)),
        "angular_speed": float(b02_target.get("angular_speed", 0.6)),
        "x_amplitude": b02_target.get("x_amplitude"),
        "y_amplitude": b02_target.get("y_amplitude"),
        "fixed_target": b02_target.get("fixed_target"),
        "custom_trajectory": b02_target.get("custom_trajectory"),
    }


def _sample_custom_target(current_time: float, dt: float, step_offset: int, custom_trajectory: list[tuple[float, float]]) -> tuple[float, float]:
    """复用 B02 custom trajectory 的采样约定。"""
    if not custom_trajectory:
        raise ValueError("custom_trajectory must not be empty")
    target_time = current_time + step_offset * dt
    index = min(len(custom_trajectory) - 1, max(0, int(round(target_time / dt))))
    return custom_trajectory[index]


def build_closed_loop_target_horizon(
    *,
    config: dict[str, Any],
    target_source: str,
    current_time: float,
    horizon: int,
    dt: float,
    start_offset_steps: int = 0,
) -> np.ndarray:
    """复用已有 B02/B03 轨迹配置，生成闭环当前时刻的 target horizon。

    输出 shape 是 `(H+1, 2)`，第 0 项是当前控制步的即时目标，后面 H 项
    是 MPC 预测窗口内的未来目标。
    """
    if horizon <= 0:
        raise ValueError(f"horizon must be positive, got {horizon}")
    if dt <= 0.0:
        raise ValueError(f"dt must be positive, got {dt}")
    if start_offset_steps < 0:
        raise ValueError(f"start_offset_steps must be non-negative, got {start_offset_steps}")
    source = target_source.strip().lower()
    target = _target_config(config, source)
    center = target["center"]
    cx, cy = float(center[0]), float(center[1])
    points: list[tuple[float, float]] = []

    for step_offset in range(horizon + 1):
        target_time = current_time + (step_offset + start_offset_steps) * dt
        if source.startswith("b03"):
            target_type = str(target["type"]).lower()
            radius = float(target["radius"])
            frequency = float(target["frequency"])
            if target_type == "circle":
                phase = 2.0 * math.pi * frequency * target_time
                points.append((cx + radius * math.cos(phase), cy + radius * math.sin(phase)))
            elif target_type == "fixed":
                points.append((cx, cy))
            elif target_type == "figure8":
                rx = radius if target["radius_x"] is None else float(target["radius_x"])
                ry = radius if target["radius_y"] is None else float(target["radius_y"])
                fx = frequency if target["frequency_x"] is None else float(target["frequency_x"])
                fy = 2.0 * frequency if target["frequency_y"] is None else float(target["frequency_y"])
                points.append((cx + rx * math.sin(2.0 * math.pi * fx * target_time), cy + ry * math.sin(2.0 * math.pi * fy * target_time)))
            elif target_type == "lissajous":
                rx = radius if target["radius_x"] is None else float(target["radius_x"])
                ry = radius if target["radius_y"] is None else float(target["radius_y"])
                fx = frequency if target["frequency_x"] is None else float(target["frequency_x"])
                fy = 1.5 * frequency if target["frequency_y"] is None else float(target["frequency_y"])
                phase_offset = float(target["phase_offset"])
                points.append((cx + rx * math.sin(2.0 * math.pi * fx * target_time), cy + ry * math.sin(2.0 * math.pi * fy * target_time + phase_offset)))
            else:
                raise ValueError(f"Unsupported B03 target type: {target_type}")
        else:
            target_type = str(target["type"]).lower()
            phase = float(target["angular_speed"]) * target_time
            radius = float(target["radius"])
            if target_type == "fixed":
                fixed = center if target["fixed_target"] is None else _as_pair(target["fixed_target"], "fixed_target")
                points.append((float(fixed[0]), float(fixed[1])))
            elif target_type == "circle":
                points.append((cx + radius * math.cos(phase), cy + radius * math.sin(phase)))
            elif target_type == "figure8":
                rx = radius if target["x_amplitude"] is None else float(target["x_amplitude"])
                ry = radius if target["y_amplitude"] is None else float(target["y_amplitude"])
                points.append((cx + rx * math.sin(phase), cy + ry * math.sin(2.0 * phase)))
            elif target_type == "sinusoidal":
                rx = radius if target["x_amplitude"] is None else float(target["x_amplitude"])
                ry = radius if target["y_amplitude"] is None else float(target["y_amplitude"])
                points.append((cx + rx * math.sin(phase), cy + ry * math.cos(phase)))
            elif target_type == "custom":
                custom_raw = target["custom_trajectory"]
                if custom_raw is None:
                    raise ValueError("custom target_source requires custom_trajectory")
                custom = [_as_pair(point, "custom_trajectory point") for point in custom_raw]
                points.append(_sample_custom_target(current_time, dt, step_offset, custom))
            else:
                raise ValueError(f"Unsupported B02 target type: {target_type}")

    return np.asarray(points, dtype=float)


def _resolve_horizon(config: dict[str, Any], horizon: int | None) -> int:
    """解析 closed-loop MPC horizon。"""
    if horizon is not None:
        resolved = int(horizon)
    else:
        # 这是 smoke runner，不默认继承完整 ladder 的 horizon=16/32。
        # 用户需要更重实验时显式传 --horizon。
        resolved = DEFAULT_SMOKE_HORIZON
    if resolved <= 0:
        raise ValueError(f"horizon must be positive, got {resolved}")
    return resolved


def _resolve_num_steps(config: dict[str, Any], num_steps: int | None) -> int:
    """解析 closed-loop 仿真步数。"""
    if num_steps is not None:
        resolved = int(num_steps)
    else:
        # 不带参数运行时必须快速返回，避免误用 B03_mpc_solver_ladder.yaml 的 num_steps=1000。
        resolved = DEFAULT_SMOKE_NUM_STEPS
    if resolved <= 0:
        raise ValueError(f"num_steps must be positive, got {resolved}")
    return resolved


def _solution_predicted_ee(solution: MPCSolution, horizon: int) -> np.ndarray:
    """读取 solution 的 predicted EE trajectory。"""
    if solution.predicted_ee_positions is None:
        raise ValueError("closed-loop solution must include predicted_ee_positions")
    predicted = _as_finite_array(solution.predicted_ee_positions, "predicted_ee_positions")
    if predicted.shape != (horizon + 1, 2):
        raise ValueError(f"predicted_ee_positions must have shape {(horizon + 1, 2)}, got {predicted.shape}")
    return predicted


def solve_two_link_ik_for_xy(
    target_xy: np.ndarray,
    *,
    link1: float = TWO_LINK_LENGTH_1,
    link2: float = TWO_LINK_LENGTH_2,
    elbow: str = "down",
) -> np.ndarray:
    """用平面二连杆解析 IK，把末端放到 target_xy。

    输入是二维末端目标 `[x, y]`；输出是关节角 `[q1, q2]`。这里使用
    B02_two_link.xml 的几何长度 `link1=0.45, link2=0.35`。如果目标超出
    工作空间，直接抛出清晰错误，避免静默把初始状态放到错误位置。
    """
    target = _as_finite_array(target_xy, "target_xy")
    if target.shape != (2,):
        raise ValueError(f"target_xy must have shape (2,), got {target.shape}")
    if link1 <= 0.0 or link2 <= 0.0:
        raise ValueError(f"link lengths must be positive, got link1={link1}, link2={link2}")

    x, y = float(target[0]), float(target[1])
    radius_sq = x * x + y * y
    radius = math.sqrt(radius_sq)
    min_radius = abs(link1 - link2)
    max_radius = link1 + link2
    tolerance = 1.0e-9
    if radius < min_radius - tolerance or radius > max_radius + tolerance:
        raise ValueError(
            "target_xy is outside the two-link workspace: "
            f"target={target.tolist()}, radius={radius:.6g}, reachable=[{min_radius:.6g}, {max_radius:.6g}]"
        )

    cos_q2 = (radius_sq - link1 * link1 - link2 * link2) / (2.0 * link1 * link2)
    cos_q2 = float(np.clip(cos_q2, -1.0, 1.0))
    sin_q2_abs = math.sqrt(max(0.0, 1.0 - cos_q2 * cos_q2))
    if elbow.strip().lower() in ("up", "negative"):
        sin_q2 = -sin_q2_abs
    else:
        sin_q2 = sin_q2_abs

    q2 = math.atan2(sin_q2, cos_q2)
    q1 = math.atan2(y, x) - math.atan2(link2 * sin_q2, link1 + link2 * cos_q2)
    return np.asarray([q1, q2], dtype=float)


def align_env_initial_ee_to_target_t0(*, env: Any, target_t0: np.ndarray) -> np.ndarray:
    """把闭环初始姿态重置到 target_t0 对应的二连杆 IK 解。

    输出是写入环境的 `[q1, q2]`。速度统一置零，因为这一步只是设置
    起跑姿态，避免初始速度额外引入 tracking 偏差。
    """
    q = solve_two_link_ik_for_xy(target_t0)
    reset_fn = getattr(env, "reset", None)
    if callable(reset_fn):
        reset_fn(q=(float(q[0]), float(q[1])), dq=(0.0, 0.0))
    else:
        r4c.set_two_link_state(env, np.asarray([q[0], q[1], 0.0, 0.0], dtype=float))
    return q.copy()


def _initial_state_from_config(config: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    """读取旧 B03 ladder 风格的初始关节状态。"""
    initial_q = _as_finite_array(config.get("initial_q", (0.3, 0.4)), "initial_q")
    initial_dq = _as_finite_array(config.get("initial_dq", (0.0, 0.0)), "initial_dq")
    if initial_q.shape != (2,):
        raise ValueError(f"initial_q must have shape (2,), got {initial_q.shape}")
    if initial_dq.shape != (2,):
        raise ValueError(f"initial_dq must have shape (2,), got {initial_dq.shape}")
    return initial_q.astype(float).copy(), initial_dq.astype(float).copy()


def reset_closed_loop_initial_state(
    *,
    env: Any,
    config: dict[str, Any],
    mode: str,
    target_t0: np.ndarray,
) -> np.ndarray:
    """设置闭环起跑状态，支持当前 IK 对齐和旧 ladder 配置口径。

    `ik_align` 用解析 IK 把末端放到目标起点；`config` 复用旧 B03 ladder
    默认初值 `q=[0.3, 0.4]`、`dq=[0, 0]`，用于同口径复现实验。
    """
    normalized = mode.strip().lower()
    if normalized in ("ik", "ik_align", "target_t0"):
        return align_env_initial_ee_to_target_t0(env=env, target_t0=target_t0)
    if normalized in ("config", "legacy", "legacy_ladder"):
        initial_q, initial_dq = _initial_state_from_config(config)
        reset_fn = getattr(env, "reset", None)
        if callable(reset_fn):
            reset_fn(q=(float(initial_q[0]), float(initial_q[1])), dq=(float(initial_dq[0]), float(initial_dq[1])))
        else:
            r4c.set_two_link_state(env, np.asarray([initial_q[0], initial_q[1], initial_dq[0], initial_dq[1]], dtype=float))
        return initial_q.copy()
    raise ValueError("initial_state_mode must be 'ik_align' or 'config'")


def _normalize_sampling_previous_solution_mode(mode: str) -> str:
    """统一 closed-loop sampling warm-start 口径。"""
    normalized = mode.strip().lower()
    aliases = {
        "warm_start_only": "warm_start_only",
        "warm-start-only": "warm_start_only",
        "current": "warm_start_only",
        "all": "all_sampling",
        "all_sampling": "all_sampling",
        "legacy": "all_sampling",
        "legacy_ladder": "all_sampling",
    }
    if normalized not in aliases:
        raise ValueError("sampling_previous_solution_mode must be 'warm_start_only' or 'all_sampling'")
    return aliases[normalized]


def run_closed_loop_task_space_rollout(
    *,
    env: Any,
    config: dict[str, Any],
    target_source: str = "b03_lissajous",
    solver_family: str = "cem",
    num_steps: int | None = None,
    horizon: int | None = None,
    sampling_overrides: dict[str, Any] | None = None,
    target_start_offset_steps: int = 0,
    initial_state_mode: str = "ik_align",
    stage_cost_timing: str = "pre_step",
    cost_profile: str = "task_space",
    sampling_previous_solution_mode: str = "warm_start_only",
) -> ClosedLoopResult:
    """运行 R4D-6 closed-loop task-space MPC rollout。"""
    family = _normalize_solver_family(solver_family)
    previous_mode = _normalize_sampling_previous_solution_mode(sampling_previous_solution_mode)
    resolved_steps = _resolve_num_steps(config, num_steps)
    resolved_horizon = _resolve_horizon(config, horizon)
    dt = float(dict(config.get("simulation", {})).get("dt", getattr(env, "dt", 0.01)))
    if dt <= 0.0:
        raise ValueError(f"dt must be positive, got {dt}")

    initial_target_t0 = build_closed_loop_target_horizon(
        config=config,
        target_source=target_source,
        current_time=0.0,
        horizon=resolved_horizon,
        dt=dt,
        start_offset_steps=target_start_offset_steps,
    )[0]
    initial_q = reset_closed_loop_initial_state(
        env=env,
        config=config,
        mode=initial_state_mode,
        target_t0=initial_target_t0,
    )
    LOGGER.info(
        "R4D-6 initial state mode=%s target0=%s q=%s",
        initial_state_mode,
        initial_target_t0.tolist(),
        initial_q.tolist(),
    )

    solver = build_solver(family)
    sampling_warm_start_solver = CEMShootingSolver() if family == "sampling_warm_start_ilqg" else None
    previous_solution: MPCSolution | None = None
    actual_states: list[np.ndarray] = []
    actual_ee_positions: list[np.ndarray] = []
    target_ee_positions: list[np.ndarray] = []
    executed_controls: list[np.ndarray] = []
    predicted_ee_positions: list[np.ndarray] = []
    step_rows: list[dict[str, Any]] = []

    for step_index in range(resolved_steps):
        current_time = float(step_index) * dt
        LOGGER.info(
            "R4D-6 closed-loop step %s/%s: target=%s solver=%s horizon=%s",
            step_index + 1,
            resolved_steps,
            target_source,
            family,
            resolved_horizon,
        )
        target_horizon = build_closed_loop_target_horizon(
            config=config,
            target_source=target_source,
            current_time=current_time,
            horizon=resolved_horizon,
            dt=dt,
            start_offset_steps=target_start_offset_steps,
        )
        sampling_solution: MPCSolution | None = None
        sampling_runtime_ms = 0.0
        sampling_num_rollouts = 0.0
        sampling_num_iterations = 0.0
        sampling_best_cost = float("nan")
        ilqg_runtime_ms = 0.0
        ilqg_num_rollouts = 0.0
        ilqg_num_iterations = 0.0
        warm_start_used = False
        warm_start_source = "zeros"
        warm_start_mode = "zeros"

        if family == "sampling_warm_start_ilqg":
            if sampling_warm_start_solver is None:
                raise RuntimeError("sampling_warm_start_ilqg requires a sampling solver")
            # 同一控制步内先用 CEM 在相同 state / target horizon 上找 nominal controls。
            # 这条 sampling 解还没有被执行，因此后续 iLQR warm-start 必须原样使用，不能左移。
            sampling_problem = r4d.build_task_space_tracking_problem(
                env=env,
                config=config,
                horizon=resolved_horizon,
                solver_family="cem",
                previous_solution=None,
                sampling_overrides=sampling_overrides,
                target_ee_positions=target_horizon,
                stage_cost_timing=stage_cost_timing,
                cost_profile=cost_profile,
            )
            sampling_solution = sampling_warm_start_solver.solve(sampling_problem, previous_solution=None)
            sampling_controls = _as_finite_array(sampling_solution.predicted_controls, "sampling_solution.predicted_controls")
            if sampling_controls.shape != (resolved_horizon, 2):
                raise ValueError(
                    "sampling_solution.predicted_controls must have shape "
                    f"{(resolved_horizon, 2)}, got {sampling_controls.shape}"
                )
            sampling_runtime_ms = float(sampling_solution.solver_stats.runtime_ms)
            sampling_num_rollouts = float(sampling_solution.solver_stats.num_rollouts)
            sampling_num_iterations = float(sampling_solution.solver_stats.num_iterations)
            sampling_best_cost = float(sampling_solution.best_cost)

            problem = r4d.build_task_space_tracking_problem(
                env=env,
                config=config,
                horizon=resolved_horizon,
                solver_family="ilqg_lite",
                previous_solution=None,
                sampling_overrides=sampling_overrides,
                target_ee_positions=target_horizon,
                direct_initial_controls=sampling_controls,
                warm_start_source=sampling_solution.solver_name,
                warm_start_mode="same_step_sampling_solution",
                stage_cost_timing="pre_step",
                cost_profile=cost_profile,
            )
            solution = solver.solve(problem, previous_solution=None)
            ilqg_runtime_ms = float(solution.solver_stats.runtime_ms)
            ilqg_num_rollouts = float(solution.solver_stats.num_rollouts)
            ilqg_num_iterations = float(solution.solver_stats.num_iterations)
            warm_start_used = bool(problem.metadata.get("warm_start_used", False))
            warm_start_source = str(problem.metadata.get("warm_start_source", "sampling"))
            warm_start_mode = str(problem.metadata.get("warm_start_mode", "same_step_sampling_solution"))
        else:
            problem_family = "cem" if family in ("random_shooting", "warm_start_sampling") else family
            if previous_mode == "all_sampling" and family in ("random_shooting", "warm_start_sampling", "cem", "mppi_lite"):
                use_previous = previous_solution
            else:
                use_previous = previous_solution if family == "warm_start_sampling" else None
            problem = r4d.build_task_space_tracking_problem(
                env=env,
                config=config,
                horizon=resolved_horizon,
                solver_family=problem_family,
                previous_solution=use_previous,
                sampling_overrides=sampling_overrides,
                target_ee_positions=target_horizon,
                stage_cost_timing=stage_cost_timing if family in ("random_shooting", "warm_start_sampling", "cem", "mppi_lite") else "pre_step",
                cost_profile=cost_profile,
            )
            solution = solver.solve(problem, previous_solution=use_previous)
            if family in ("random_shooting", "warm_start_sampling", "cem", "mppi_lite"):
                sampling_runtime_ms = float(solution.solver_stats.runtime_ms)
                sampling_num_rollouts = float(solution.solver_stats.num_rollouts)
                sampling_num_iterations = float(solution.solver_stats.num_iterations)
                sampling_best_cost = float(solution.best_cost)
            if family == "ilqg_lite":
                ilqg_runtime_ms = float(solution.solver_stats.runtime_ms)
                ilqg_num_rollouts = float(solution.solver_stats.num_rollouts)
                ilqg_num_iterations = float(solution.solver_stats.num_iterations)
        first_control = _as_finite_array(solution.first_control, "solution.first_control")
        r4c.validate_control_shape(first_control)

        total_runtime_ms = sampling_runtime_ms + ilqg_runtime_ms
        if family not in ("sampling_warm_start_ilqg", "random_shooting", "warm_start_sampling", "cem", "mppi_lite", "ilqg_lite"):
            total_runtime_ms = float(solution.solver_stats.runtime_ms)
        if family in ("random_shooting", "warm_start_sampling", "cem", "mppi_lite", "ilqg_lite"):
            total_runtime_ms = float(solution.solver_stats.runtime_ms)

        env.step(tuple(float(value) for value in first_control.tolist()))
        applied_control = first_control
        get_last_applied_torque_fn = getattr(env, "get_last_applied_torque", None)
        if callable(get_last_applied_torque_fn):
            applied_control = _as_finite_array(get_last_applied_torque_fn(), "env.get_last_applied_torque()")
            r4c.validate_control_shape(applied_control)
        actual_state = r4c.get_two_link_state(env)
        actual_ee = r4d.compute_end_effector_xy(env, actual_state)
        target_now = target_horizon[0]
        ee_error = float(np.linalg.norm(actual_ee - target_now))

        actual_states.append(actual_state)
        actual_ee_positions.append(actual_ee)
        target_ee_positions.append(target_now)
        executed_controls.append(applied_control)
        predicted_ee_positions.append(_solution_predicted_ee(solution, resolved_horizon))
        step_rows.append(
            {
                "step": float(step_index),
                "time": current_time + float(target_start_offset_steps) * dt,
                "target_x": float(target_now[0]),
                "target_y": float(target_now[1]),
                "actual_x": float(actual_ee[0]),
                "actual_y": float(actual_ee[1]),
                "ee_error": ee_error,
                "tau1": float(applied_control[0]),
                "tau2": float(applied_control[1]),
                "best_cost": float(solution.best_cost),
                "runtime_ms": float(total_runtime_ms),
                "num_rollouts": float(sampling_num_rollouts + ilqg_num_rollouts),
                "num_iterations": float(sampling_num_iterations + ilqg_num_iterations),
                "sampling_best_cost": sampling_best_cost,
                "sampling_runtime_ms": sampling_runtime_ms,
                "sampling_num_rollouts": sampling_num_rollouts,
                "sampling_num_iterations": sampling_num_iterations,
                "ilqg_best_cost": float(solution.best_cost) if family in ("ilqg_lite", "sampling_warm_start_ilqg") else float("nan"),
                "ilqg_runtime_ms": ilqg_runtime_ms,
                "ilqg_num_rollouts": ilqg_num_rollouts,
                "ilqg_num_iterations": ilqg_num_iterations,
                "warm_start_used": str(bool(warm_start_used)),
                "warm_start_source": warm_start_source,
                "warm_start_mode": warm_start_mode,
                "target_start_offset_steps": float(target_start_offset_steps),
                "stage_cost_timing": stage_cost_timing,
                "cost_profile": cost_profile,
                "initial_state_mode": initial_state_mode,
                "sampling_previous_solution_mode": previous_mode,
            }
        )
        LOGGER.info(
            "R4D-6 step %s/%s done: ee_error=%.6g best_cost=%.6g runtime_ms=%.3f",
            step_index + 1,
            resolved_steps,
            ee_error,
            float(solution.best_cost),
            float(total_runtime_ms),
        )
        previous_solution = solution

    actual_ee_array = np.asarray(actual_ee_positions, dtype=float)
    target_ee_array = np.asarray(target_ee_positions, dtype=float)
    controls_array = np.asarray(executed_controls, dtype=float)
    errors = np.linalg.norm(actual_ee_array - target_ee_array, axis=1)
    runtime_values = np.asarray([row["runtime_ms"] for row in step_rows], dtype=float)
    sampling_runtime_values = np.asarray([row["sampling_runtime_ms"] for row in step_rows], dtype=float)
    ilqg_runtime_values = np.asarray([row["ilqg_runtime_ms"] for row in step_rows], dtype=float)
    summary_metrics = {
        "final_ee_error": float(errors[-1]),
        "mean_ee_error": float(np.mean(errors)),
        "max_ee_error": float(np.max(errors)),
        "mean_runtime_ms": float(np.mean(runtime_values)),
        "max_runtime_ms": float(np.max(runtime_values)),
        "mean_sampling_runtime_ms": float(np.mean(sampling_runtime_values)),
        "mean_ilqg_runtime_ms": float(np.mean(ilqg_runtime_values)),
        "mean_abs_torque": float(np.mean(np.abs(controls_array))),
        "max_abs_torque": float(np.max(np.abs(controls_array))),
        "control_energy": float(np.sum(controls_array * controls_array)),
    }

    return ClosedLoopResult(
        target_source=target_source,
        solver_family=family,
        step_rows=np.asarray(step_rows, dtype=object),
        actual_states=np.asarray(actual_states, dtype=float),
        actual_ee_positions=actual_ee_array,
        target_ee_positions=target_ee_array,
        executed_controls=controls_array,
        predicted_ee_positions=np.asarray(predicted_ee_positions, dtype=float),
        summary_metrics=summary_metrics,
    )


def ensure_output_dirs(output_dir: Path) -> dict[str, Path]:
    """创建 R4D-6 输出目录。"""
    root = Path(output_dir)
    paths = {
        "cache": root / "outputs" / "cache",
        "figures": root / "outputs" / "figures",
        "reports": root / "outputs" / "reports",
        "metrics": root / "outputs" / "metrics",
    }
    for path in paths.values():
        path.mkdir(parents=True, exist_ok=True)
    return paths


def build_output_paths(output_dir: Path | None) -> dict[str, Path]:
    """规划 R4D-6 输出文件。"""
    run_dir = output_dir or DEFAULT_OUTPUT_ROOT / datetime.now().strftime("%Y%m%d_%H%M%S")
    base = ensure_output_dirs(run_dir)
    return {
        "run_dir": run_dir,
        **base,
        "step_metrics_csv": run_dir / "outputs" / "metrics" / "B03_R4D6_closed_loop_steps.csv",
        "summary_metrics_csv": run_dir / "outputs" / "metrics" / "B03_R4D6_closed_loop_summary.csv",
        "cache_npz": run_dir / "outputs" / "cache" / "B03_R4D6_closed_loop_cache.npz",
        "report_md": run_dir / "outputs" / "reports" / "B03_R4D6_closed_loop_report.md",
        "ee_trajectory_figure": run_dir / "outputs" / "figures" / "B03_R4D6_closed_loop_ee_xy.png",
        "ee_error_figure": run_dir / "outputs" / "figures" / "B03_R4D6_closed_loop_ee_error.png",
        "control_figure": run_dir / "outputs" / "figures" / "B03_R4D6_closed_loop_controls.png",
        "runtime_figure": run_dir / "outputs" / "figures" / "B03_R4D6_closed_loop_runtime.png",
    }


def _relative_output_path(path: Path, run_dir: Path) -> str:
    """把输出路径转成相对 run_dir 的文本。"""
    try:
        return path.relative_to(run_dir).as_posix()
    except ValueError:
        return path.as_posix()


def _configure_axis(ax: Any) -> None:
    """统一 R4D-6 图样式。"""
    ax.grid(False)
    ax.tick_params(axis="both", which="both", length=0, labelcolor="dimgrey")
    ax.patch.set_edgecolor("lightgrey")
    ax.patch.set_linewidth(0.8)


def plot_closed_loop_ee_trajectory(result: ClosedLoopResult, output_path: Path) -> None:
    """绘制闭环真实末端轨迹和目标轨迹。"""
    sns.set_theme(font_scale=1.0, style="whitegrid", font="DejaVu Sans")
    fig, ax = plt.subplots(figsize=(7.4, 6.0), dpi=150)
    ax.plot(result.target_ee_positions[:, 0], result.target_ee_positions[:, 1], color="dimgrey", linewidth=2.2, label="target")
    ax.plot(result.actual_ee_positions[:, 0], result.actual_ee_positions[:, 1], color="#d73027", linewidth=1.9, label="actual")
    ax.scatter(result.target_ee_positions[0, 0], result.target_ee_positions[0, 1], color="dimgrey", s=30, edgecolors="white", linewidths=0.7)
    ax.scatter(result.actual_ee_positions[-1, 0], result.actual_ee_positions[-1, 1], color="#d73027", s=42, marker="s", edgecolors="white", linewidths=0.7)
    ax.set_title("B03-R4D-6 Closed-Loop End-Effector XY", loc="left", fontsize=13, color="dimgrey")
    ax.set_xlabel("x_ee", fontsize=10, color="dimgrey")
    ax.set_ylabel("y_ee", fontsize=10, color="dimgrey")
    ax.axis("equal")
    _configure_axis(ax)
    ax.legend(frameon=True, facecolor="white", framealpha=0.8, edgecolor="lightgrey", labelcolor="dimgrey", fontsize=8)
    fig.text(0.99, 0.01, f"mean error: {result.summary_metrics['mean_ee_error']:.2e}", ha="right", va="bottom", fontsize=9, color="dimgrey", style="italic")
    sns.despine(left=True, bottom=True)
    fig.tight_layout(rect=(0.02, 0.03, 0.99, 0.98))
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_closed_loop_error(result: ClosedLoopResult, output_path: Path) -> None:
    """绘制闭环每步末端误差。"""
    sns.set_theme(font_scale=1.0, style="whitegrid", font="DejaVu Sans")
    errors = np.linalg.norm(result.actual_ee_positions - result.target_ee_positions, axis=1)
    steps = np.arange(errors.size)
    fig, ax = plt.subplots(figsize=(8.8, 4.8), dpi=150)
    ax.plot(steps, errors, color="#4575b4", linewidth=2.0)
    ax.scatter(steps, errors, color="#4575b4", s=24, edgecolors="white", linewidths=0.7, zorder=4)
    ax.text(steps[-1], errors[-1], f" final {errors[-1]:.2e}", color="dimgrey", fontsize=8, va="center")
    ax.set_title("B03-R4D-6 Closed-Loop EE Error", loc="left", fontsize=13, color="dimgrey")
    ax.set_xlabel("control step", fontsize=10, color="dimgrey")
    ax.set_ylabel("||actual_ee - target_ee||", fontsize=10, color="dimgrey")
    _configure_axis(ax)
    sns.despine(left=True, bottom=True)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_closed_loop_controls(result: ClosedLoopResult, output_path: Path) -> None:
    """绘制闭环实际执行控制。"""
    sns.set_theme(font_scale=1.0, style="whitegrid", font="DejaVu Sans")
    steps = np.arange(result.executed_controls.shape[0])
    fig, axes = plt.subplots(1, 2, figsize=(12.0, 4.8), dpi=150, sharex=True)
    for index, ax in enumerate(axes):
        ax.axhline(0.0, color="lightgrey", linewidth=0.8)
        ax.step(steps, result.executed_controls[:, index], where="post", color="#d73027", linewidth=1.9)
        ax.set_title(r"$\bf{(" + chr(ord("a") + index) + r")}$" + f"  executed tau{index + 1}", loc="left", fontsize=11, color="dimgrey")
        ax.set_xlabel("control step", fontsize=10, color="dimgrey")
        if index == 0:
            ax.set_ylabel("torque", fontsize=10, color="dimgrey")
        _configure_axis(ax)
    fig.suptitle("B03-R4D-6 Executed Controls", fontsize=14, color="dimgrey", y=0.99)
    sns.despine(left=True, bottom=True)
    fig.tight_layout(rect=(0.02, 0.02, 0.99, 0.94))
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_closed_loop_runtime(result: ClosedLoopResult, output_path: Path) -> None:
    """绘制闭环每步 solver runtime。"""
    sns.set_theme(font_scale=1.0, style="whitegrid", font="DejaVu Sans")
    rows = list(result.step_rows)
    runtime_ms = np.asarray([float(row["runtime_ms"]) for row in rows], dtype=float)
    steps = np.arange(runtime_ms.size)
    fig, ax = plt.subplots(figsize=(8.8, 4.8), dpi=150)
    ax.bar(steps, runtime_ms, color="#72a5b4", alpha=0.82, width=0.62)
    ax.axhline(float(np.mean(runtime_ms)), color="#d73027", linewidth=1.4, linestyle="--", label="mean")
    ax.set_title("B03-R4D-6 Per-Step Solver Runtime", loc="left", fontsize=13, color="dimgrey")
    ax.set_xlabel("control step", fontsize=10, color="dimgrey")
    ax.set_ylabel("runtime (ms)", fontsize=10, color="dimgrey")
    _configure_axis(ax)
    ax.legend(frameon=True, facecolor="white", framealpha=0.8, edgecolor="lightgrey", labelcolor="dimgrey", fontsize=8)
    sns.despine(left=True, bottom=True)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def write_closed_loop_outputs(*, result: ClosedLoopResult, output_paths: dict[str, Path]) -> None:
    """写入 R4D-6 闭环 CSV / NPZ / PNG / Markdown report。"""
    rows = list(result.step_rows)
    with output_paths["step_metrics_csv"].open("w", encoding="utf-8", newline="") as step_file:
        fieldnames = list(rows[0].keys())
        writer = csv.DictWriter(step_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    with output_paths["summary_metrics_csv"].open("w", encoding="utf-8", newline="") as summary_file:
        fieldnames = ["target_source", "solver_family", *result.summary_metrics.keys()]
        writer = csv.DictWriter(summary_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerow({"target_source": result.target_source, "solver_family": result.solver_family, **result.summary_metrics})

    np.savez(
        output_paths["cache_npz"],
        actual_states=result.actual_states,
        actual_ee_positions=result.actual_ee_positions,
        target_ee_positions=result.target_ee_positions,
        executed_controls=result.executed_controls,
        predicted_ee_positions=result.predicted_ee_positions,
    )

    plot_closed_loop_ee_trajectory(result, output_paths["ee_trajectory_figure"])
    plot_closed_loop_error(result, output_paths["ee_error_figure"])
    plot_closed_loop_controls(result, output_paths["control_figure"])
    plot_closed_loop_runtime(result, output_paths["runtime_figure"])

    run_dir = output_paths["run_dir"]
    metrics = result.summary_metrics
    report_lines = [
        "# B03-R4D-6 Closed-Loop Task-Space MPC Smoke Report",
        "",
        "## Summary",
        "",
        f"- target_source: `{result.target_source}`",
        f"- solver_family: `{result.solver_family}`",
        f"- steps: {result.actual_ee_positions.shape[0]}",
        f"- final_ee_error: {metrics['final_ee_error']:.6g}",
        f"- mean_ee_error: {metrics['mean_ee_error']:.6g}",
        f"- max_ee_error: {metrics['max_ee_error']:.6g}",
        f"- mean_runtime_ms: {metrics['mean_runtime_ms']:.6g}",
        "",
        "## Output Files",
        "",
        f"- step_metrics_csv: `{_relative_output_path(output_paths['step_metrics_csv'], run_dir)}`",
        f"- summary_metrics_csv: `{_relative_output_path(output_paths['summary_metrics_csv'], run_dir)}`",
        f"- cache_npz: `{_relative_output_path(output_paths['cache_npz'], run_dir)}`",
        f"- ee_trajectory_figure: `{_relative_output_path(output_paths['ee_trajectory_figure'], run_dir)}`",
        f"- ee_error_figure: `{_relative_output_path(output_paths['ee_error_figure'], run_dir)}`",
        f"- control_figure: `{_relative_output_path(output_paths['control_figure'], run_dir)}`",
        f"- runtime_figure: `{_relative_output_path(output_paths['runtime_figure'], run_dir)}`",
        "",
        "## Current Scope",
        "",
        "- Reuses existing B03/B02 task-space reference trajectories.",
        "- Replans at each control step and executes only `solution.first_control`.",
        "- Records actual closed-loop end-effector trajectory after MuJoCo step.",
        "- Does not modify B02 controller / benchmark / regression.",
    ]
    output_paths["report_md"].write_text("\n".join(report_lines), encoding="utf-8")

    LOGGER.info("R4D-6 step metrics written: %s", output_paths["step_metrics_csv"])
    LOGGER.info("R4D-6 summary metrics written: %s", output_paths["summary_metrics_csv"])
    LOGGER.info("R4D-6 figures written: %s", output_paths["figures"])
    LOGGER.info("R4D-6 report written: %s", output_paths["report_md"])


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
    """复用 R4D 环境创建逻辑。"""
    return r4d.setup_two_link_env(config)


def build_arg_parser() -> argparse.ArgumentParser:
    """创建命令行参数。"""
    parser = argparse.ArgumentParser(description="B03-R4D-6 closed-loop task-space MPC smoke.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH, help="YAML 配置路径。")
    parser.add_argument("--output-dir", type=Path, default=None, help="输出根目录。")
    parser.add_argument(
        "--target-source",
        choices=("b03_lissajous", "b03_circle", "b03_figure8", "b02_figure8", "b02_circle", "b02_sinusoidal", "b02_custom"),
        default="b03_lissajous",
        help="复用哪一套已有末端参考轨迹。",
    )
    parser.add_argument(
        "--solver-family",
        choices=("random_shooting", "warm_start_sampling", "cem", "mppi_lite", "ilqg_lite", "sampling_warm_start_ilqg"),
        default="cem",
        help="闭环每步使用的 solver。",
    )
    parser.add_argument("--num-steps", type=int, default=None, help="闭环执行步数。")
    parser.add_argument("--horizon", type=int, default=None, help="MPC horizon。")
    parser.add_argument("--num-candidates", type=int, default=None, help="覆盖 sampling 候选数。")
    parser.add_argument("--sampling-iterations", type=int, default=None, help="覆盖 CEM/MPPI 迭代数。")
    parser.add_argument("--sampling-std", type=float, default=None, help="覆盖 sampling std。")
    parser.add_argument("--smoothing-alpha", type=float, default=None, help="覆盖 CEM/MPPI smoothing_alpha。")
    parser.add_argument("--torque-limit", type=float, default=None, help="覆盖 torque limit。")
    parser.add_argument("--seed", type=int, default=None, help="覆盖 sampling seed。")
    parser.add_argument(
        "--target-start-offset-steps",
        type=int,
        default=0,
        help="目标 horizon 从当前时刻后第几步开始；旧 B03 ladder 口径为 1。",
    )
    parser.add_argument(
        "--initial-state-mode",
        choices=("ik_align", "config"),
        default="ik_align",
        help="初始状态：ik_align 对齐目标起点；config 复用旧 B03 ladder 默认 q=[0.3,0.4]。",
    )
    parser.add_argument(
        "--stage-cost-timing",
        choices=("pre_step", "post_step"),
        default="pre_step",
        help="采样 rollout cost 的 stage 误差时序；旧 B03 adapter 口径为 post_step。",
    )
    parser.add_argument(
        "--cost-profile",
        choices=("task_space", "legacy_ladder"),
        default="task_space",
        help="task-space cost 权重口径；旧 B03 adapter 口径为 legacy_ladder。",
    )
    parser.add_argument(
        "--sampling-previous-solution-mode",
        choices=("warm_start_only", "all_sampling"),
        default="warm_start_only",
        help="上一控制周期解的传递方式；旧 B03 runner 对所有 sampling solver 都传 previous_solution。",
    )
    parser.add_argument("--log-level", choices=("INFO", "DEBUG"), default="INFO", help="日志等级。")
    return parser


def _sampling_overrides_from_args(args: argparse.Namespace) -> dict[str, Any]:
    """把 CLI 覆盖项转成 sampling solver_config。"""
    overrides: dict[str, Any] = {
        "num_candidates": DEFAULT_SMOKE_NUM_CANDIDATES,
        "num_iterations": DEFAULT_SMOKE_SAMPLING_ITERATIONS,
        "sampling_std": DEFAULT_SMOKE_SAMPLING_STD,
        "initial_std": DEFAULT_SMOKE_SAMPLING_STD,
        "noise_std": DEFAULT_SMOKE_SAMPLING_STD,
        "torque_limit": DEFAULT_SMOKE_TORQUE_LIMIT,
        "seed": DEFAULT_SMOKE_SEED,
    }
    if args.num_candidates is not None:
        overrides["num_candidates"] = int(args.num_candidates)
    if args.sampling_iterations is not None:
        overrides["num_iterations"] = int(args.sampling_iterations)
    if args.sampling_std is not None:
        overrides["sampling_std"] = float(args.sampling_std)
        overrides["initial_std"] = float(args.sampling_std)
        overrides["noise_std"] = float(args.sampling_std)
    if args.smoothing_alpha is not None:
        overrides["smoothing_alpha"] = float(args.smoothing_alpha)
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
    result = run_closed_loop_task_space_rollout(
        env=env,
        config=config,
        target_source=args.target_source,
        solver_family=args.solver_family,
        num_steps=args.num_steps,
        horizon=args.horizon,
        sampling_overrides=_sampling_overrides_from_args(args),
        target_start_offset_steps=args.target_start_offset_steps,
        initial_state_mode=args.initial_state_mode,
        stage_cost_timing=args.stage_cost_timing,
        cost_profile=args.cost_profile,
        sampling_previous_solution_mode=args.sampling_previous_solution_mode,
    )
    output_paths = build_output_paths(args.output_dir)
    write_closed_loop_outputs(result=result, output_paths=output_paths)
    LOGGER.info(
        "R4D-6 closed-loop: target=%s solver=%s final_ee_error=%.6g mean_ee_error=%.6g mean_runtime_ms=%.3f",
        result.target_source,
        result.solver_family,
        result.summary_metrics["final_ee_error"],
        result.summary_metrics["mean_ee_error"],
        result.summary_metrics["mean_runtime_ms"],
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
