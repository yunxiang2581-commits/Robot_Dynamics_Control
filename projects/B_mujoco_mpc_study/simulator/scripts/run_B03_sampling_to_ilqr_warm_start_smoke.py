"""B03-R4C-2B sampling-to-iLQR warm-start smoke runner.

本脚本做一件很窄的事：
- 先用 sampling-family solver（默认 CEM，也可切到 MPPI-lite）在同一个
  two-link state-tracking MPCProblem 上生成 `sampling_solution`；
- 再用同一初始状态和同一条 `x_ref` 跑 zero-init iLQR-lite；
- 最后把 `sampling_solution.predicted_controls` 作为 previous_solution，
  交给 R4C-2 已接好的 iLQR-lite warm-start 路径；
- 输出三路结果的 CSV / NPZ / Markdown report。

当前边界：
- 不修改 iLQR-lite 数学核心；
- 不修改 B02 controller / benchmark / regression；
- 不实现 task-space end-effector tracking；
- 只做短 horizon joint-space state tracking smoke。
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, replace
from datetime import datetime
import logging
from pathlib import Path
import sys
from typing import Any

import numpy as np

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
from projects.B_mujoco_mpc_study.simulator.planners.mpc_solver_interface import (
    BaseMPCSolver,
    MPCProblem,
    MPCSolution,
)
from projects.B_mujoco_mpc_study.simulator.planners.sampling_mpc_solvers import (
    CEMShootingSolver,
    MPPILiteSolver,
)
from projects.B_mujoco_mpc_study.simulator.scripts import run_B03_ilqr_lite_two_link_smoke as r4c


LOGGER = logging.getLogger(__name__)

TASK_NAME = "B03_sampling_to_ilqr_warm_start_smoke"
DEFAULT_CONFIG_PATH = r4c.DEFAULT_CONFIG_PATH
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "outputs" / "runs" / TASK_NAME


@dataclass(frozen=True)
class StateTrackingRolloutResult:
    """给 sampling solver 消费的批量 rollout 结果。"""

    costs: np.ndarray
    predicted_states: np.ndarray
    predicted_ee_positions: np.ndarray | None = None


@dataclass(frozen=True)
class SolverRunResult:
    """一次 solver 调用及其对应问题。"""

    label: str
    problem: MPCProblem
    solution: MPCSolution


@dataclass(frozen=True)
class ComparisonResult:
    """R4C-2B 三路对照结果。"""

    sampling: SolverRunResult
    zero_init_ilqr: SolverRunResult
    warm_start_ilqr: SolverRunResult


def _as_finite_array(value: Any, name: str) -> np.ndarray:
    """把输入转成有限 float 数组，错误信息保留变量名。"""
    array = np.asarray(value, dtype=float)
    if not np.isfinite(array).all():
        raise ValueError(f"{name} must contain only finite values, got shape={array.shape}")
    return array


def _tracking_arrays_from_problem(problem: MPCProblem) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """从 R4C state-tracking problem 中取出 x_refs / Q / R / Q_terminal。"""
    metadata = dict(problem.metadata or {})
    cost_config = dict(problem.cost_config or {})

    x_refs = _as_finite_array(metadata.get("x_refs", problem.target_horizon), "x_refs")
    Q = _as_finite_array(metadata.get("Q", cost_config.get("Q")), "Q")
    R = _as_finite_array(metadata.get("R", cost_config.get("R")), "R")
    Q_terminal = _as_finite_array(
        metadata.get("Q_terminal", cost_config.get("Q_terminal")),
        "Q_terminal",
    )

    horizon = int(problem.horizon)
    state_dim = int(np.asarray(problem.current_state, dtype=float).size)
    control_dim = int(problem.control_dim)
    if x_refs.shape != (horizon + 1, state_dim):
        raise ValueError(f"x_refs must have shape {(horizon + 1, state_dim)}, got {x_refs.shape}")
    if Q.shape != (state_dim, state_dim):
        raise ValueError(f"Q must have shape {(state_dim, state_dim)}, got {Q.shape}")
    if R.shape != (control_dim, control_dim):
        raise ValueError(f"R must have shape {(control_dim, control_dim)}, got {R.shape}")
    if Q_terminal.shape != (state_dim, state_dim):
        raise ValueError(
            f"Q_terminal must have shape {(state_dim, state_dim)}, got {Q_terminal.shape}"
        )
    return x_refs, Q, R, Q_terminal


def evaluate_state_tracking_rollouts(
    candidate_controls: np.ndarray,
    problem: MPCProblem,
) -> StateTrackingRolloutResult:
    """用同一个 dynamics_fn 批量评估 sampling 候选控制序列。

    输入：
    - candidate_controls: shape=(N, H, 2)，N 是候选数；
    - problem.current_state: 当前 two-link state，约定 x=[q1,q2,dq1,dq2]；
    - problem.dynamics_fn: R4C-1C 的无副作用 one-step dynamics adapter；
    - problem.metadata/cost_config: state tracking 的 x_refs / Q / R / Q_terminal。

    输出：
    - costs: shape=(N,)；
    - predicted_states: shape=(N, H+1, 4)。
    """
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
    if not callable(dynamics_fn):
        raise ValueError("problem.dynamics_fn is required for state-tracking sampling rollouts.")

    current_state = _as_finite_array(problem.current_state, "problem.current_state")
    if current_state.ndim != 1:
        raise ValueError(f"problem.current_state must be 1D, got {current_state.shape}")

    x_refs, Q, R, Q_terminal = _tracking_arrays_from_problem(problem)
    state_dim = int(current_state.size)
    predicted_states = np.zeros((num_candidates, horizon + 1, state_dim), dtype=float)
    costs = np.zeros(num_candidates, dtype=float)

    for candidate_index in range(num_candidates):
        states = predicted_states[candidate_index]
        states[0] = current_state
        total_cost = 0.0
        for step_index in range(horizon):
            x_t = states[step_index]
            u_t = controls[candidate_index, step_index]
            dx = x_t - x_refs[step_index]
            total_cost += float(0.5 * dx.T @ Q @ dx + 0.5 * u_t.T @ R @ u_t)

            x_next = _as_finite_array(dynamics_fn(x_t, u_t), f"dynamics_fn candidate={candidate_index} step={step_index}")
            if x_next.shape != (state_dim,):
                raise ValueError(f"dynamics_fn must return shape {(state_dim,)}, got {x_next.shape}")
            states[step_index + 1] = x_next

        terminal_dx = states[-1] - x_refs[-1]
        total_cost += float(0.5 * terminal_dx.T @ Q_terminal @ terminal_dx)
        costs[candidate_index] = total_cost

    return StateTrackingRolloutResult(costs=costs, predicted_states=predicted_states)


def _normalize_sampling_solver_family(sampling_solver_family: str) -> str:
    """统一 sampling solver 名称，避免 CLI 和配置中大小写/别名造成分支混乱。"""
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


def build_sampling_solver_config(
    *,
    config: dict[str, Any],
    sampling_solver_family: str,
    horizon: int,
    control_dim: int,
    sampling_overrides: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """构造 CEM/MPPI-lite 的最小 solver_config。

    配置来源顺序：
    1. `sampling` 公共配置；
    2. `cem` 或 `mppi` 专属配置；
    3. 调用者传入的 sampling_overrides。
    """
    family = _normalize_sampling_solver_family(sampling_solver_family)
    sampling_config = dict(config.get("sampling", {}))
    family_config = dict(config.get("cem" if family == "cem" else "mppi", {}))

    num_candidates = int(family_config.get("num_candidates", sampling_config.get("num_candidates", 64)))
    torque_limit = float(family_config.get("torque_limit", sampling_config.get("torque_limit", 2.0)))
    seed = family_config.get("seed", sampling_config.get("seed", 0))

    solver_config: dict[str, Any] = {
        "horizon": int(horizon),
        "control_dim": int(control_dim),
        "num_candidates": num_candidates,
        "torque_limit": torque_limit,
        "seed": None if seed is None else int(seed),
    }

    if family == "cem":
        sampling_std = float(
            family_config.get(
                "initial_std",
                family_config.get("sampling_std", sampling_config.get("sampling_std", 0.5)),
            )
        )
        solver_config.update(
            {
                "num_iterations": int(family_config.get("num_iterations", 2)),
                "elite_ratio": float(family_config.get("elite_ratio", 0.2)),
                "sampling_std": sampling_std,
                "initial_std": sampling_std,
                "min_std": float(family_config.get("min_std", 0.05)),
                "smoothing_alpha": float(family_config.get("smoothing_alpha", 0.0)),
                "early_stop_patience": int(family_config.get("early_stop_patience", 0)),
                "improvement_tolerance": float(family_config.get("improvement_tolerance", 0.0)),
            }
        )
    else:
        noise_std = float(
            family_config.get(
                "noise_std",
                family_config.get("sampling_std", sampling_config.get("sampling_std", 0.5)),
            )
        )
        solver_config.update(
            {
                "num_iterations": int(family_config.get("num_iterations", 2)),
                "temperature": float(family_config.get("temperature", 1.0)),
                "sampling_std": noise_std,
                "noise_std": noise_std,
                "min_std": float(family_config.get("min_std", 0.05)),
                "smoothing_alpha": float(family_config.get("smoothing_alpha", 0.0)),
                "early_stop_patience": int(family_config.get("early_stop_patience", 0)),
                "improvement_tolerance": float(family_config.get("improvement_tolerance", 0.0)),
            }
        )

    if sampling_overrides:
        solver_config.update(dict(sampling_overrides))
    return solver_config


def replace_problem_metadata(problem: MPCProblem, updates: dict[str, Any]) -> MPCProblem:
    """返回一个 metadata 更新后的 MPCProblem，原对象保持不变。"""
    metadata = dict(problem.metadata or {})
    metadata.update(updates)
    return replace(problem, metadata=metadata)


def _build_base_state_tracking_problem(
    *,
    env: Any,
    x_ref: np.ndarray,
    config: dict[str, Any],
    initial_controls: np.ndarray,
) -> MPCProblem:
    """复用 R4C helper 构造基础 state-tracking problem。"""
    x0 = r4c.get_two_link_state(env)
    r4c.validate_state_shape(x0)
    dynamics_fn = r4c.build_two_link_dynamics_fn(env=env)
    return r4c.build_state_tracking_problem(
        env=env,
        dynamics_fn=dynamics_fn,
        x0=x0,
        x_refs=x_ref,
        u_nominal=initial_controls,
        config=config,
    )


def build_sampling_state_tracking_problem(
    *,
    env: Any,
    x_ref: np.ndarray,
    config: dict[str, Any],
    sampling_solver_family: str = "cem",
    sampling_overrides: dict[str, Any] | None = None,
) -> MPCProblem:
    """构造给 CEM/MPPI-lite 使用的 two-link state-tracking MPCProblem。"""
    reference_states = np.asarray(x_ref, dtype=float)
    horizon = int(reference_states.shape[0] - 1)
    initial_controls = np.zeros((horizon, 2), dtype=float)
    base_problem = _build_base_state_tracking_problem(
        env=env,
        x_ref=reference_states,
        config=config,
        initial_controls=initial_controls,
    )
    family = _normalize_sampling_solver_family(sampling_solver_family)
    solver_config = build_sampling_solver_config(
        config=config,
        sampling_solver_family=family,
        horizon=base_problem.horizon,
        control_dim=base_problem.control_dim,
        sampling_overrides=sampling_overrides,
    )
    metadata = {
        **dict(base_problem.metadata or {}),
        "task_name": "B03-R4C-2B_sampling_state_tracking",
        "sampling_solver_family": family,
        "warm_start_used": False,
        "warm_start_source": "zeros",
    }
    return replace(
        base_problem,
        solver_config=solver_config,
        rollout_cost_fn=evaluate_state_tracking_rollouts,
        metadata=metadata,
    )


def build_ilqr_state_tracking_problem(
    *,
    env: Any,
    x_ref: np.ndarray,
    config: dict[str, Any],
    previous_solution: MPCSolution | None = None,
) -> MPCProblem:
    """构造 zero-init 或 sampling-warm-start 的 iLQR-lite problem。"""
    reference_states = np.asarray(x_ref, dtype=float)
    horizon = int(reference_states.shape[0] - 1)
    warm_start_controls = r4c.build_warm_start_controls_from_previous_solution(
        previous_solution=previous_solution,
        horizon=horizon,
        control_dim=2,
    )
    warm_start_used = warm_start_controls is not None
    initial_controls = warm_start_controls if warm_start_used else np.zeros((horizon, 2), dtype=float)
    problem = _build_base_state_tracking_problem(
        env=env,
        x_ref=reference_states,
        config=config,
        initial_controls=initial_controls,
    )
    problem.metadata["warm_start_used"] = bool(warm_start_used)
    problem.metadata["warm_start_source"] = (
        getattr(previous_solution, "solver_name", "previous_solution") if warm_start_used else "zeros"
    )
    return problem


def run_sampling_to_ilqr_comparison(
    *,
    env: Any,
    x_ref: np.ndarray,
    config: dict[str, Any],
    sampling_solver_family: str = "cem",
    sampling_overrides: dict[str, Any] | None = None,
) -> ComparisonResult:
    """运行 sampling、zero-init iLQR、sampling-warm-start iLQR 三路对照。"""
    family = _normalize_sampling_solver_family(sampling_solver_family)
    initial_snapshot = r4c.save_two_link_env_state(env)
    try:
        r4c.restore_two_link_env_state(env, initial_snapshot)
        sampling_problem = build_sampling_state_tracking_problem(
            env=env,
            x_ref=x_ref,
            config=config,
            sampling_solver_family=family,
            sampling_overrides=sampling_overrides,
        )
        sampling_solution = build_sampling_solver(family).solve(sampling_problem)

        r4c.restore_two_link_env_state(env, initial_snapshot)
        zero_problem = build_ilqr_state_tracking_problem(
            env=env,
            x_ref=x_ref,
            config=config,
            previous_solution=None,
        )
        zero_solution = ILQGLiteSolver().solve(zero_problem)

        r4c.restore_two_link_env_state(env, initial_snapshot)
        warm_problem = build_ilqr_state_tracking_problem(
            env=env,
            x_ref=x_ref,
            config=config,
            previous_solution=sampling_solution,
        )
        warm_solution = ILQGLiteSolver().solve(warm_problem, previous_solution=sampling_solution)
    finally:
        r4c.restore_two_link_env_state(env, initial_snapshot)

    return ComparisonResult(
        sampling=SolverRunResult(
            label=f"sampling_{family}",
            problem=sampling_problem,
            solution=sampling_solution,
        ),
        zero_init_ilqr=SolverRunResult(
            label="zero_init_ilqr",
            problem=zero_problem,
            solution=zero_solution,
        ),
        warm_start_ilqr=SolverRunResult(
            label="sampling_warm_start_ilqr",
            problem=warm_problem,
            solution=warm_solution,
        ),
    )


def ensure_output_dirs(output_dir: Path) -> dict[str, Path]:
    """创建 R4C-2B 输出目录。"""
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
    """规划 R4C-2B 的 CSV / NPZ / Markdown 输出路径。"""
    run_dir = output_dir or DEFAULT_OUTPUT_ROOT / datetime.now().strftime("%Y%m%d_%H%M%S")
    base = ensure_output_dirs(run_dir)
    return {
        "run_dir": run_dir,
        **base,
        "comparison_metrics_csv": run_dir / "outputs" / "metrics" / "B03_R4C2B_sampling_to_ilqr_metrics.csv",
        "comparison_cache": run_dir / "outputs" / "cache" / "B03_R4C2B_sampling_to_ilqr_cache.npz",
        "comparison_metrics_figure": run_dir / "outputs" / "figures" / "B03_R4C2B_cost_runtime_comparison.png",
        "trajectory_comparison_figure": run_dir / "outputs" / "figures" / "B03_R4C2B_state_trajectory_comparison.png",
        "control_comparison_figure": run_dir / "outputs" / "figures" / "B03_R4C2B_control_sequence_comparison.png",
        "comparison_report": run_dir / "outputs" / "reports" / "B03_R4C2B_sampling_to_ilqr_report.md",
    }


def _cost_history_from_solution(solution: MPCSolution) -> np.ndarray:
    """抽取可用于 initial/final cost 的 cost 序列。"""
    metadata = dict(solution.metadata or {})
    raw_history = metadata.get("cost_history", metadata.get("iteration_best_costs", [solution.best_cost]))
    history = np.asarray(raw_history, dtype=float).reshape(-1)
    if history.size == 0:
        history = np.asarray([solution.best_cost], dtype=float)
    if not np.isfinite(history).all():
        raise ValueError(f"{solution.solver_name} cost history contains non-finite values.")
    return history


def _final_state_error_norm(run: SolverRunResult) -> float:
    """计算 predicted final state 与 reference final state 的误差范数。"""
    states = run.solution.predicted_states
    if states is None:
        return float("nan")
    states_array = np.asarray(states, dtype=float)
    if states_array.ndim != 2 or states_array.shape[0] == 0:
        return float("nan")
    x_refs = np.asarray(run.problem.metadata.get("x_refs", run.problem.target_horizon), dtype=float)
    if x_refs.ndim != 2 or x_refs.shape[0] == 0 or x_refs.shape[1] != states_array.shape[1]:
        return float("nan")
    return float(np.linalg.norm(states_array[-1] - x_refs[-1]))


def _metrics_row(run: SolverRunResult) -> dict[str, Any]:
    """把一次 solver 结果转成 comparison CSV 的一行。"""
    cost_history = _cost_history_from_solution(run.solution)
    controls = np.asarray(run.solution.predicted_controls, dtype=float)
    return {
        "label": run.label,
        "solver_name": run.solution.solver_name,
        "success": bool(run.solution.solver_stats.success),
        "message": run.solution.solver_stats.message,
        "termination_reason": run.solution.metadata.get("termination_reason", ""),
        "warm_start_used": bool(run.problem.metadata.get("warm_start_used", False)),
        "warm_start_source": str(run.problem.metadata.get("warm_start_source", "zeros")),
        "horizon": int(run.problem.horizon),
        "control_dim": int(run.problem.control_dim),
        "best_cost": float(run.solution.best_cost),
        "initial_cost": float(cost_history[0]),
        "final_cost": float(cost_history[-1]),
        "num_cost_entries": int(cost_history.size),
        "runtime_ms": float(run.solution.solver_stats.runtime_ms),
        "num_rollouts": int(run.solution.solver_stats.num_rollouts),
        "num_iterations": int(run.solution.solver_stats.num_iterations),
        "max_abs_control": float(np.max(np.abs(controls))) if controls.size else 0.0,
        "final_state_error_norm": _final_state_error_norm(run),
    }


def _relative_output_path(path: Path, run_dir: Path) -> str:
    """报告中尽量写相对路径，便于阅读。"""
    try:
        return str(path.relative_to(run_dir)).replace("\\", "/")
    except ValueError:
        return str(path).replace("\\", "/")


def _plot_label(label: str) -> str:
    """把内部 run label 压短，避免图上横轴文字拥挤。"""
    label_map = {
        "sampling_cem": "Sampling\nCEM",
        "sampling_mppi_lite": "Sampling\nMPPI",
        "zero_init_ilqr": "Zero-init\niLQR",
        "sampling_warm_start_ilqr": "Warm-start\niLQR",
    }
    return label_map.get(label, label.replace("_", "\n"))


def _configure_plot_axis(ax: Any) -> None:
    """统一 R4C-2B 图的坐标轴样式。"""
    ax.grid(False)
    ax.tick_params(axis="both", which="both", length=0, labelcolor="dimgrey")
    ax.patch.set_edgecolor("lightgrey")
    ax.patch.set_linewidth(0.8)


def plot_comparison_metrics(rows: list[dict[str, Any]], output_path: Path) -> None:
    """绘制三路 solver 的 cost/runtime/error 指标对比图。"""
    sns.set_theme(font_scale=1.0, style="whitegrid", font="DejaVu Sans")
    labels = [_plot_label(str(row["label"])) for row in rows]
    colors = ["#72a5b4", "#4575b4", "#d73027"][: len(rows)]
    metric_specs = [
        ("best_cost", "Best cost", ".2e"),
        ("initial_cost", "Initial cost", ".2e"),
        ("runtime_ms", "Runtime (ms)", ".1f"),
        ("final_state_error_norm", "Final state error", ".2e"),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(12, 7.2), dpi=150)
    axes_flat = list(axes.ravel())
    for axis_index, (ax, (metric_key, title, fmt)) in enumerate(zip(axes_flat, metric_specs)):
        values = np.asarray([float(row[metric_key]) for row in rows], dtype=float)
        x_positions = np.arange(len(values))
        ax.bar(x_positions, values, color=colors, alpha=0.82, width=0.62, zorder=3)
        positive_values = values[values > 0.0]
        if positive_values.size > 0 and float(np.max(positive_values) / np.min(positive_values)) > 100.0:
            ax.set_yscale("log")
        top = float(np.max(positive_values)) if positive_values.size else 1.0
        for x_pos, value in zip(x_positions, values):
            text_y = value if value > 0.0 else top * 0.02
            ax.text(
                x_pos,
                text_y,
                f"{value:{fmt}}",
                ha="center",
                va="bottom",
                fontsize=8,
                color="dimgrey",
                weight="semibold",
            )
        ax.set_xticks(x_positions)
        ax.set_xticklabels(labels, fontsize=8)
        ax.set_title(r"$\bf{(" + chr(ord("a") + axis_index) + r")}$" + f"  {title}", loc="left", fontsize=11, color="dimgrey")
        _configure_plot_axis(ax)

    warm_row = rows[-1]
    fig.suptitle("B03-R4C-2B Sampling to iLQR Comparison", fontsize=14, color="dimgrey", y=0.99)
    fig.text(
        0.99,
        0.01,
        f"Warm-start source: {warm_row['warm_start_source']} | horizon: {warm_row['horizon']}",
        ha="right",
        va="bottom",
        fontsize=9,
        color="dimgrey",
        style="italic",
    )
    sns.despine(left=True, bottom=True)
    fig.tight_layout(rect=(0.02, 0.04, 0.99, 0.95))
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_state_trajectory_comparison(comparison: ComparisonResult, output_path: Path) -> None:
    """绘制 q1/q2 的 reference、sampling、zero-init iLQR、warm-start iLQR 轨迹。"""
    sns.set_theme(font_scale=1.0, style="whitegrid", font="DejaVu Sans")
    x_refs = np.asarray(comparison.zero_init_ilqr.problem.metadata.get("x_refs"), dtype=float)
    sampling_states = np.asarray(comparison.sampling.solution.predicted_states, dtype=float)
    zero_states = np.asarray(comparison.zero_init_ilqr.solution.predicted_states, dtype=float)
    warm_states = np.asarray(comparison.warm_start_ilqr.solution.predicted_states, dtype=float)
    time_index = np.arange(x_refs.shape[0])

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), dpi=150, sharex=True)
    joint_specs = [(0, "q1"), (1, "q2")]
    for axis_index, (ax, (joint_index, joint_name)) in enumerate(zip(axes, joint_specs)):
        ax.plot(time_index, x_refs[:, joint_index], color="dimgrey", linewidth=2.2, label="reference")
        ax.plot(time_index, sampling_states[:, joint_index], color="#72a5b4", linewidth=1.8, linestyle=":", label="sampling")
        ax.plot(time_index, zero_states[:, joint_index], color="#4575b4", linewidth=1.8, linestyle="--", label="zero-init iLQR")
        ax.plot(time_index, warm_states[:, joint_index], color="#d73027", linewidth=1.8, linestyle="-", label="warm-start iLQR")
        zero_error = float(zero_states[-1, joint_index] - x_refs[-1, joint_index])
        warm_error = float(warm_states[-1, joint_index] - x_refs[-1, joint_index])
        ax.text(
            0.98,
            0.05,
            f"final err: zero {zero_error:+.1e} | warm {warm_error:+.1e}",
            transform=ax.transAxes,
            ha="right",
            va="bottom",
            fontsize=8,
            color="dimgrey",
            style="italic",
        )
        ax.set_title(r"$\bf{(" + chr(ord("a") + axis_index) + r")}$" + f"  {joint_name} state trajectory", loc="left", fontsize=11, color="dimgrey")
        ax.set_xlabel("step", fontsize=10, labelpad=6, color="dimgrey")
        if axis_index == 0:
            ax.set_ylabel("angle (rad)", fontsize=10, labelpad=6, color="dimgrey")
        _configure_plot_axis(ax)

    axes[0].legend(
        frameon=True,
        facecolor="white",
        framealpha=0.8,
        edgecolor="lightgrey",
        labelcolor="dimgrey",
        fontsize=8,
        loc="upper left",
    )
    fig.suptitle("B03-R4C-2B State Trajectory Comparison", fontsize=14, color="dimgrey", y=0.99)
    sns.despine(left=True, bottom=True)
    fig.tight_layout(rect=(0.02, 0.02, 0.99, 0.94))
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_control_sequence_comparison(comparison: ComparisonResult, output_path: Path) -> None:
    """绘制 sampling 控制、shifted warm-start 初值和 iLQR 优化后控制。"""
    sns.set_theme(font_scale=1.0, style="whitegrid", font="DejaVu Sans")
    sampling_controls = np.asarray(comparison.sampling.solution.predicted_controls, dtype=float)
    zero_controls = np.asarray(comparison.zero_init_ilqr.solution.predicted_controls, dtype=float)
    warm_initial_controls = np.asarray(comparison.warm_start_ilqr.problem.metadata.get("initial_controls"), dtype=float)
    warm_controls = np.asarray(comparison.warm_start_ilqr.solution.predicted_controls, dtype=float)
    time_index = np.arange(sampling_controls.shape[0])

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), dpi=150, sharex=True)
    control_specs = [(0, "tau1"), (1, "tau2")]
    for axis_index, (ax, (control_index, control_name)) in enumerate(zip(axes, control_specs)):
        ax.axhline(0.0, color="lightgrey", linewidth=0.8, zorder=1)
        ax.step(time_index, sampling_controls[:, control_index], where="post", color="#72a5b4", linewidth=1.7, linestyle=":", label="sampling controls")
        ax.step(time_index, warm_initial_controls[:, control_index], where="post", color="#fc8d59", linewidth=1.7, linestyle="-.", label="shifted warm init")
        ax.step(time_index, zero_controls[:, control_index], where="post", color="#4575b4", linewidth=1.7, linestyle="--", label="zero-init iLQR")
        ax.step(time_index, warm_controls[:, control_index], where="post", color="#d73027", linewidth=1.7, linestyle="-", label="warm-start iLQR")
        ax.set_title(r"$\bf{(" + chr(ord("a") + axis_index) + r")}$" + f"  {control_name} sequence", loc="left", fontsize=11, color="dimgrey")
        ax.set_xlabel("control step", fontsize=10, labelpad=6, color="dimgrey")
        if axis_index == 0:
            ax.set_ylabel("torque", fontsize=10, labelpad=6, color="dimgrey")
        _configure_plot_axis(ax)

    axes[0].legend(
        frameon=True,
        facecolor="white",
        framealpha=0.8,
        edgecolor="lightgrey",
        labelcolor="dimgrey",
        fontsize=8,
        loc="upper right",
    )
    max_warm_init = float(np.max(np.abs(warm_initial_controls))) if warm_initial_controls.size else 0.0
    max_warm_optimized = float(np.max(np.abs(warm_controls))) if warm_controls.size else 0.0
    fig.suptitle("B03-R4C-2B Control Sequence Comparison", fontsize=14, color="dimgrey", y=0.99)
    fig.text(
        0.99,
        0.01,
        f"Warm init max |u|: {max_warm_init:.2e}; optimized max |u|: {max_warm_optimized:.2e}",
        ha="right",
        va="bottom",
        fontsize=9,
        color="dimgrey",
        style="italic",
    )
    sns.despine(left=True, bottom=True)
    fig.tight_layout(rect=(0.02, 0.04, 0.99, 0.94))
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def write_comparison_outputs(
    *,
    comparison: ComparisonResult,
    output_paths: dict[str, Path],
) -> None:
    """写入 R4C-2B 对照 CSV / NPZ / Markdown report。"""
    runs = [comparison.sampling, comparison.zero_init_ilqr, comparison.warm_start_ilqr]
    rows = [_metrics_row(run) for run in runs]

    metrics_path = output_paths["comparison_metrics_csv"]
    with metrics_path.open("w", encoding="utf-8", newline="") as metrics_file:
        writer = csv.DictWriter(metrics_file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    np.savez(
        output_paths["comparison_cache"],
        x_refs=np.asarray(comparison.zero_init_ilqr.problem.metadata.get("x_refs"), dtype=float),
        sampling_predicted_states=np.asarray(comparison.sampling.solution.predicted_states, dtype=float),
        sampling_predicted_controls=np.asarray(comparison.sampling.solution.predicted_controls, dtype=float),
        zero_init_ilqr_predicted_states=np.asarray(comparison.zero_init_ilqr.solution.predicted_states, dtype=float),
        zero_init_ilqr_predicted_controls=np.asarray(comparison.zero_init_ilqr.solution.predicted_controls, dtype=float),
        zero_init_ilqr_initial_controls=np.asarray(
            comparison.zero_init_ilqr.problem.metadata.get("initial_controls"),
            dtype=float,
        ),
        warm_start_ilqr_predicted_states=np.asarray(comparison.warm_start_ilqr.solution.predicted_states, dtype=float),
        warm_start_ilqr_predicted_controls=np.asarray(comparison.warm_start_ilqr.solution.predicted_controls, dtype=float),
        warm_start_ilqr_initial_controls=np.asarray(
            comparison.warm_start_ilqr.problem.metadata.get("initial_controls"),
            dtype=float,
        ),
    )

    plot_comparison_metrics(rows, output_paths["comparison_metrics_figure"])
    plot_state_trajectory_comparison(comparison, output_paths["trajectory_comparison_figure"])
    plot_control_sequence_comparison(comparison, output_paths["control_comparison_figure"])

    run_dir = output_paths["run_dir"]
    report_path = output_paths["comparison_report"]
    report_lines = [
        "# B03-R4C-2B Sampling to iLQR Warm-Start Smoke Report",
        "",
        "## Summary",
        "",
        "This smoke run first generates a real sampling-family MPCSolution, then compares zero-init iLQR-lite against sampling-warm-start iLQR-lite on the same joint-space state-tracking problem.",
        "",
        "## Metrics",
        "",
        "| label | solver | warm_start | best_cost | initial_cost | final_cost | runtime_ms | rollouts | iterations | final_state_error_norm |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        report_lines.append(
            "| {label} | {solver_name} | {warm_start_used} | {best_cost:.6g} | "
            "{initial_cost:.6g} | {final_cost:.6g} | {runtime_ms:.6g} | "
            "{num_rollouts} | {num_iterations} | {final_state_error_norm:.6g} |".format(**row)
        )
    report_lines.extend(
        [
            "",
            "## Output Files",
            "",
            f"- metrics_csv: `{_relative_output_path(output_paths['comparison_metrics_csv'], run_dir)}`",
            f"- cache_npz: `{_relative_output_path(output_paths['comparison_cache'], run_dir)}`",
            f"- cost_runtime_figure: `{_relative_output_path(output_paths['comparison_metrics_figure'], run_dir)}`",
            f"- state_trajectory_figure: `{_relative_output_path(output_paths['trajectory_comparison_figure'], run_dir)}`",
            f"- control_sequence_figure: `{_relative_output_path(output_paths['control_comparison_figure'], run_dir)}`",
            "",
            "## Current Scope",
            "",
            "- Uses joint-space state tracking: `x=[q1,q2,dq1,dq2]`.",
            "- Uses the same R4C one-step dynamics adapter for sampling rollouts and iLQR-lite rollouts.",
            "- Does not modify the iLQR-lite core solver math.",
            "- Does not modify the B02 controller, B02 benchmark, or B02 regression settings.",
            "- Does not implement task-space end-effector tracking yet.",
            "",
        ]
    )
    report_path.write_text("\n".join(report_lines), encoding="utf-8")

    LOGGER.info("R4C-2B comparison metrics 已写入: %s", metrics_path)
    LOGGER.info("R4C-2B comparison cache 已写入: %s", output_paths["comparison_cache"])
    LOGGER.info("R4C-2B comparison figures 已写入: %s", output_paths["figures"])
    LOGGER.info("R4C-2B comparison report 已写入: %s", report_path)


def build_arg_parser() -> argparse.ArgumentParser:
    """创建 R4C-2B 命令行参数。"""
    parser = argparse.ArgumentParser(description="B03-R4C-2B sampling-to-iLQR warm-start smoke.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH, help="YAML 配置路径。")
    parser.add_argument("--output-dir", type=Path, default=None, help="输出根目录。")
    parser.add_argument(
        "--sampling-solver",
        choices=("cem", "mppi_lite"),
        default="cem",
        help="先运行哪个 sampling-family solver 作为 iLQR warm-start 来源。",
    )
    parser.add_argument("--num-candidates", type=int, default=None, help="覆盖 sampling 候选数。")
    parser.add_argument("--sampling-iterations", type=int, default=None, help="覆盖 CEM/MPPI 迭代数。")
    parser.add_argument("--sampling-std", type=float, default=None, help="覆盖 CEM initial_std / MPPI noise_std。")
    parser.add_argument("--torque-limit", type=float, default=None, help="覆盖 sampling torque_limit。")
    parser.add_argument("--seed", type=int, default=None, help="覆盖 sampling seed。")
    parser.add_argument("--log-level", choices=("INFO", "DEBUG"), default="INFO", help="日志等级。")
    return parser


def _sampling_overrides_from_args(args: argparse.Namespace) -> dict[str, Any]:
    """把 CLI 覆盖项转成 solver_config 字典。"""
    overrides: dict[str, Any] = {}
    if args.num_candidates is not None:
        overrides["num_candidates"] = int(args.num_candidates)
    if args.sampling_iterations is not None:
        overrides["num_iterations"] = int(args.sampling_iterations)
    if args.sampling_std is not None:
        overrides["sampling_std"] = float(args.sampling_std)
        if args.sampling_solver == "cem":
            overrides["initial_std"] = float(args.sampling_std)
        else:
            overrides["noise_std"] = float(args.sampling_std)
    if args.torque_limit is not None:
        overrides["torque_limit"] = float(args.torque_limit)
    if args.seed is not None:
        overrides["seed"] = int(args.seed)
    return overrides


def main(argv: list[str] | None = None) -> None:
    """脚本入口：运行三路对照并写出可复盘文件。"""
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    r4c.configure_logging(args.log_level)

    config = r4c.load_yaml_config(args.config)
    output_paths = build_output_paths(args.output_dir)
    env, x_ref = r4c.setup_environment(args.config)
    comparison = run_sampling_to_ilqr_comparison(
        env=env,
        x_ref=x_ref,
        config=config,
        sampling_solver_family=args.sampling_solver,
        sampling_overrides=_sampling_overrides_from_args(args),
    )
    write_comparison_outputs(comparison=comparison, output_paths=output_paths)

    for run in [comparison.sampling, comparison.zero_init_ilqr, comparison.warm_start_ilqr]:
        row = _metrics_row(run)
        LOGGER.info(
            "%s: best_cost=%.6g final_cost=%.6g runtime_ms=%.3f warm_start=%s source=%s",
            row["label"],
            row["best_cost"],
            row["final_cost"],
            row["runtime_ms"],
            row["warm_start_used"],
            row["warm_start_source"],
        )


if __name__ == "__main__":
    main()
