"""B03 solver comparison visualization utilities."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

try:
    from projects.B_mujoco_mpc_study.simulator.utils.solver_benchmark_logger import SolverStepRow
except ModuleNotFoundError:
    from utils.solver_benchmark_logger import SolverStepRow


def compute_control_smoothness(controls: np.ndarray) -> float:
    """Compute mean first-order control difference."""
    control_array = np.asarray(controls, dtype=float)
    if control_array.ndim != 2 or control_array.shape[0] < 2:
        return 0.0
    diffs = control_array[1:] - control_array[:-1]
    return float(np.mean(np.linalg.norm(diffs, axis=1)))


def compute_trajectory_smoothness(points: np.ndarray) -> float:
    """Compute mean second-order trajectory difference."""
    point_array = np.asarray(points, dtype=float)
    if point_array.ndim != 2 or point_array.shape[0] < 3:
        return 0.0
    second_diff = point_array[2:] - 2.0 * point_array[1:-1] + point_array[:-2]
    return float(np.mean(np.linalg.norm(second_diff, axis=1)))


def _validate_step_rows_by_solver(
    step_rows_by_solver: dict[str, list[SolverStepRow]],
) -> dict[str, list[SolverStepRow]]:
    if not step_rows_by_solver:
        raise ValueError("step_rows_by_solver must not be empty")
    return step_rows_by_solver


def _prepare_output_path(output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)


def _plot_series_by_solver(
    *,
    step_rows_by_solver: dict[str, list[SolverStepRow]],
    output_path: Path,
    title: str,
    ylabel: str,
    value_getter: Any,
) -> Path:
    rows_map = _validate_step_rows_by_solver(step_rows_by_solver)
    _prepare_output_path(output_path)

    plt.figure(figsize=(8, 4.5))
    for solver_name, rows in rows_map.items():
        if not rows:
            continue
        times = [row.time for row in rows]
        values = [float(value_getter(row)) for row in rows]
        plt.plot(times, values, label=solver_name)
    plt.xlabel("time [s]")
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()
    return output_path


def plot_solver_error_comparison(
    step_rows_by_solver: dict[str, list[SolverStepRow]],
    output_path: Path,
) -> Any:
    """Plot current end-effector error curves for all solvers."""
    return _plot_series_by_solver(
        step_rows_by_solver=step_rows_by_solver,
        output_path=output_path,
        title="B03 solver error comparison",
        ylabel="ee error [m]",
        value_getter=lambda row: row.current_ee_error,
    )


def plot_solver_runtime_comparison(
    step_rows_by_solver: dict[str, list[SolverStepRow]],
    output_path: Path,
) -> Any:
    """Plot per-step runtime curves for all solvers."""
    return _plot_series_by_solver(
        step_rows_by_solver=step_rows_by_solver,
        output_path=output_path,
        title="B03 solver runtime comparison",
        ylabel="runtime [ms]",
        value_getter=lambda row: row.runtime_ms,
    )


def plot_solver_cost_comparison(
    step_rows_by_solver: dict[str, list[SolverStepRow]],
    output_path: Path,
) -> Any:
    """Plot per-step best-cost curves for all solvers."""
    return _plot_series_by_solver(
        step_rows_by_solver=step_rows_by_solver,
        output_path=output_path,
        title="B03 solver cost comparison",
        ylabel="best cost",
        value_getter=lambda row: row.best_cost,
    )


def plot_control_smoothness_comparison(
    step_rows_by_solver: dict[str, list[SolverStepRow]],
    output_path: Path,
) -> Any:
    """Plot per-step control smoothness curves for all solvers."""
    return _plot_series_by_solver(
        step_rows_by_solver=step_rows_by_solver,
        output_path=output_path,
        title="B03 control smoothness comparison",
        ylabel="control smoothness",
        value_getter=lambda row: row.control_smoothness,
    )


def plot_trajectory_smoothness_comparison(
    step_rows_by_solver: dict[str, list[SolverStepRow]],
    output_path: Path,
) -> Any:
    """Plot per-step trajectory smoothness curves for all solvers."""
    return _plot_series_by_solver(
        step_rows_by_solver=step_rows_by_solver,
        output_path=output_path,
        title="B03 trajectory smoothness comparison",
        ylabel="trajectory smoothness",
        value_getter=lambda row: row.trajectory_smoothness,
    )


def build_solver_comparison_video_plan(*args: object, **kwargs: object) -> Any:
    """TODO: keep video layout planning separate from solver logic."""
    _ = args, kwargs
    raise NotImplementedError("B03 currently keeps comparison video planning as a TODO skeleton.")
