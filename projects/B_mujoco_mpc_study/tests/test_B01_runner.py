from __future__ import annotations

from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from projects.B_mujoco_mpc_study.simulator.scripts import run_B01_single_joint_mpc_demo as runner


def test_ensure_output_dirs_creates_parent_directories() -> None:
    base_dir = Path(__file__).resolve().parents[1] / "outputs" / "test_runner_dirs"
    outputs = {
        "log": base_dir / "logs" / "B01_run_log.txt",
        "metrics": base_dir / "metrics" / "B01_metrics.csv",
    }

    runner.ensure_output_dirs(outputs)

    assert outputs["log"].parent.exists()
    assert outputs["metrics"].parent.exists()


def test_compute_summary_metrics_reports_core_tracking_values() -> None:
    metrics = runner.compute_summary_metrics(
        q_history=[0.0, 0.5, 0.8],
        torque_history=[-0.2, 0.3, -0.5],
        runtime_history=[0.01, 0.03],
        q_target=1.0,
    )

    assert metrics["final_error"] == 0.19999999999999996
    assert metrics["mean_tracking_error"] == (1.0 + 0.5 + 0.19999999999999996) / 3
    assert metrics["max_torque"] == 0.5
    assert metrics["runtime_per_control_step"] == 0.02
