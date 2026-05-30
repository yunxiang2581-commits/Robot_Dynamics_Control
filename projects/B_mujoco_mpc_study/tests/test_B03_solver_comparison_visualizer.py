from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SIMULATOR_ROOT = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "simulator"
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

from utils.solver_comparison_visualizer import (
    compute_control_smoothness,
    compute_trajectory_smoothness,
    plot_control_smoothness_comparison,
    plot_solver_cost_comparison,
    plot_solver_error_comparison,
    plot_solver_runtime_comparison,
    plot_trajectory_smoothness_comparison,
)
from utils.solver_benchmark_logger import SolverStepRow


def test_compute_control_smoothness() -> None:
    controls = np.array(
        [
            [0.0, 0.0],
            [1.0, 0.0],
            [1.0, 1.0],
        ],
        dtype=float,
    )

    smoothness = compute_control_smoothness(controls)

    expected = (1.0 + 1.0) / 2.0
    assert smoothness == pytest.approx(expected)


def test_compute_trajectory_smoothness() -> None:
    points = np.array(
        [
            [0.0, 0.0],
            [1.0, 0.0],
            [3.0, 0.0],
            [6.0, 0.0],
        ],
        dtype=float,
    )

    smoothness = compute_trajectory_smoothness(points)

    expected = (np.linalg.norm(np.array([1.0, 0.0])) + np.linalg.norm(np.array([1.0, 0.0]))) / 2.0
    assert smoothness == pytest.approx(expected)


def test_short_sequence_edge_cases() -> None:
    assert compute_control_smoothness(np.zeros((0, 2))) == pytest.approx(0.0)
    assert compute_control_smoothness(np.zeros((1, 2))) == pytest.approx(0.0)
    assert compute_trajectory_smoothness(np.zeros((0, 2))) == pytest.approx(0.0)
    assert compute_trajectory_smoothness(np.zeros((2, 2))) == pytest.approx(0.0)


def test_plot_solver_comparison_figures_create_files() -> None:
    tmp_path = REPO_ROOT / "outputs" / "pytest_tmp" / "b03_solver_comparison_visualizer"
    tmp_path.mkdir(parents=True, exist_ok=True)
    step_rows_by_solver = {
        "random_shooting": [
            SolverStepRow(
                step=0,
                time=0.01,
                solver_name="random_shooting",
                final_ee_error=0.20,
                current_ee_error=0.20,
                best_cost=2.0,
                runtime_ms=0.5,
                num_rollouts=8,
                num_iterations=1,
                max_abs_torque=0.1,
                control_smoothness=0.0,
                trajectory_smoothness=0.0,
                success=True,
            ),
            SolverStepRow(
                step=1,
                time=0.02,
                solver_name="random_shooting",
                final_ee_error=0.10,
                current_ee_error=0.10,
                best_cost=1.0,
                runtime_ms=0.6,
                num_rollouts=8,
                num_iterations=1,
                max_abs_torque=0.2,
                control_smoothness=0.1,
                trajectory_smoothness=0.05,
                success=True,
            ),
        ],
        "warm_start_sampling": [
            SolverStepRow(
                step=0,
                time=0.01,
                solver_name="warm_start_sampling",
                final_ee_error=0.18,
                current_ee_error=0.18,
                best_cost=1.8,
                runtime_ms=0.4,
                num_rollouts=8,
                num_iterations=1,
                max_abs_torque=0.1,
                control_smoothness=0.0,
                trajectory_smoothness=0.0,
                success=True,
            ),
            SolverStepRow(
                step=1,
                time=0.02,
                solver_name="warm_start_sampling",
                final_ee_error=0.08,
                current_ee_error=0.08,
                best_cost=0.9,
                runtime_ms=0.5,
                num_rollouts=8,
                num_iterations=1,
                max_abs_torque=0.2,
                control_smoothness=0.08,
                trajectory_smoothness=0.04,
                success=True,
            ),
        ],
    }

    error_path = tmp_path / "error.png"
    runtime_path = tmp_path / "runtime.png"
    cost_path = tmp_path / "cost.png"
    smoothness_path = tmp_path / "smoothness.png"
    trajectory_smoothness_path = tmp_path / "trajectory_smoothness.png"

    plot_solver_error_comparison(step_rows_by_solver, error_path)
    plot_solver_runtime_comparison(step_rows_by_solver, runtime_path)
    plot_solver_cost_comparison(step_rows_by_solver, cost_path)
    plot_control_smoothness_comparison(step_rows_by_solver, smoothness_path)
    plot_trajectory_smoothness_comparison(step_rows_by_solver, trajectory_smoothness_path)

    assert error_path.exists()
    assert runtime_path.exists()
    assert cost_path.exists()
    assert smoothness_path.exists()
    assert trajectory_smoothness_path.exists()
    assert error_path.stat().st_size > 0
    assert runtime_path.stat().st_size > 0
    assert cost_path.stat().st_size > 0
    assert smoothness_path.stat().st_size > 0
    assert trajectory_smoothness_path.stat().st_size > 0
