from __future__ import annotations

from pathlib import Path
import sys
import uuid


REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from projects.B_mujoco_mpc_study.simulator.utils.plotting import save_b02_tracking_figures


def test_save_b02_tracking_figures_writes_three_expected_files() -> None:
    temp_dir = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "outputs" / "pytest_tmp"
    temp_dir.mkdir(parents=True, exist_ok=True)

    outputs = {
        "xy_target_vs_actual": temp_dir / f"xy_{uuid.uuid4().hex}.png",
        "tracking_error_time": temp_dir / f"error_{uuid.uuid4().hex}.png",
        "control_input_time": temp_dir / f"control_{uuid.uuid4().hex}.png",
    }
    tracking_rows = [
        {
            "time": 0.01,
            "target_x": 0.5,
            "target_y": 0.2,
            "actual_x": 0.48,
            "actual_y": 0.19,
            "error_norm": 0.03,
            "u1": 0.1,
            "u2": -0.1,
        },
        {
            "time": 0.02,
            "target_x": 0.52,
            "target_y": 0.21,
            "actual_x": 0.50,
            "actual_y": 0.20,
            "error_norm": 0.025,
            "u1": 0.08,
            "u2": -0.05,
        },
    ]

    try:
        save_b02_tracking_figures(tracking_rows=tracking_rows, outputs=outputs)
        assert outputs["xy_target_vs_actual"].exists()
        assert outputs["tracking_error_time"].exists()
        assert outputs["control_input_time"].exists()
        assert outputs["xy_target_vs_actual"].stat().st_size > 0
        assert outputs["tracking_error_time"].stat().st_size > 0
        assert outputs["control_input_time"].stat().st_size > 0
    finally:
        for output_path in outputs.values():
            if output_path.exists():
                output_path.unlink()
