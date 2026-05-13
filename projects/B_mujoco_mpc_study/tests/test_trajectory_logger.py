from __future__ import annotations

from pathlib import Path
import sys
import uuid


REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from projects.B_mujoco_mpc_study.simulator.utils.trajectory_logger import TrackingLogBuffer


def test_tracking_log_buffer_appends_rows_and_saves_csv() -> None:
    logger = TrackingLogBuffer()

    logger.append_step(
        step=0,
        time=0.01,
        state=(0.1, 0.2, 0.3, 0.4),
        target_xy=(0.5, 0.6),
        actual_xy=(0.55, 0.65),
        control=(0.7, -0.2),
        mpc_cost=1.23,
    )
    logger.append_step(
        step=1,
        time=0.02,
        state=(0.2, 0.3, 0.4, 0.5),
        target_xy=(0.6, 0.7),
        actual_xy=(0.62, 0.71),
        control=(0.1, 0.0),
        mpc_cost=0.98,
    )

    rows = logger.to_rows()

    assert len(rows) == 2
    assert rows[0].error_norm > 0.0

    temp_dir = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "outputs" / "pytest_tmp"
    temp_dir.mkdir(parents=True, exist_ok=True)
    output_path = temp_dir / f"tracking_{uuid.uuid4().hex}.csv"
    try:
        logger.save_csv(output_path)

        csv_text = output_path.read_text(encoding="utf-8")
        assert "step,time,q1,q2,dq1,dq2,target_x,target_y,actual_x,actual_y,error_norm,u1,u2,mpc_cost" in csv_text
        assert len(csv_text.strip().splitlines()) == 3
    finally:
        if output_path.exists():
            output_path.unlink()
