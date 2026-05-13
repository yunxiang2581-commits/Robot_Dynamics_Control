from __future__ import annotations

from pathlib import Path
import sys
from uuid import uuid4


REPO_ROOT = Path(__file__).resolve().parents[3]
SIMULATOR_ROOT = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "simulator"
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

from utils.solver_benchmark_logger import SolverBenchmarkLogBuffer


def _workspace_tmp_dir() -> Path:
    tmp_dir = REPO_ROOT / ".pytest_tmp" / f"B03_solver_logger_{uuid4().hex}"
    tmp_dir.mkdir(parents=True, exist_ok=True)
    return tmp_dir


def test_append_step() -> None:
    logger = SolverBenchmarkLogBuffer()

    logger.append_step(
        step=0,
        time=0.01,
        solver_name="random_shooting",
        final_ee_error=0.2,
        current_ee_error=0.3,
        best_cost=1.0,
        runtime_ms=2.5,
        num_rollouts=128,
        num_iterations=1,
        max_abs_torque=0.8,
        control_smoothness=0.1,
        trajectory_smoothness=0.05,
        success=True,
    )

    rows = logger.step_rows()
    assert len(rows) == 1
    assert rows[0].solver_name == "random_shooting"
    assert rows[0].runtime_ms == 2.5


def test_append_summary() -> None:
    logger = SolverBenchmarkLogBuffer()

    logger.append_summary(
        solver_name="cem",
        final_ee_error=0.1,
        mean_ee_error=0.2,
        max_ee_error=0.4,
        runtime_per_control_step=3.0,
        max_abs_torque=1.1,
        mean_abs_torque=0.6,
        control_smoothness=0.08,
        trajectory_smoothness=0.03,
        best_cost=0.9,
        cost_std=0.2,
        solver_success_rate=1.0,
    )

    rows = logger.summary_rows()
    assert len(rows) == 1
    assert rows[0].solver_name == "cem"
    assert rows[0].best_cost == 0.9


def test_save_step_csv() -> None:
    logger = SolverBenchmarkLogBuffer()
    logger.append_step(
        step=1,
        time=0.02,
        solver_name="mppi_lite",
        final_ee_error=0.15,
        current_ee_error=0.25,
        best_cost=0.8,
        runtime_ms=4.0,
        num_rollouts=64,
        num_iterations=2,
        max_abs_torque=1.0,
        control_smoothness=0.12,
        trajectory_smoothness=0.04,
        success=True,
    )

    output_path = _workspace_tmp_dir() / "logs" / "steps.csv"
    logger.save_step_csv(output_path)

    lines = output_path.read_text(encoding="utf-8").strip().splitlines()
    assert lines[0].startswith("step,time,solver_name")
    assert len(lines) == 2


def test_save_summary_csv() -> None:
    logger = SolverBenchmarkLogBuffer()
    logger.append_summary(
        solver_name="warm_start_sampling",
        final_ee_error=0.12,
        mean_ee_error=0.18,
        max_ee_error=0.35,
        runtime_per_control_step=2.2,
        max_abs_torque=0.9,
        mean_abs_torque=0.5,
        control_smoothness=0.07,
        trajectory_smoothness=0.02,
        best_cost=0.7,
        cost_std=0.1,
        solver_success_rate=0.95,
    )

    output_path = _workspace_tmp_dir() / "metrics" / "summary.csv"
    logger.save_summary_csv(output_path)

    lines = output_path.read_text(encoding="utf-8").strip().splitlines()
    assert lines[0].startswith("solver_name,final_ee_error")
    assert len(lines) == 2
