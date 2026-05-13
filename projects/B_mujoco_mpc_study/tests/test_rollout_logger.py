from __future__ import annotations

from pathlib import Path
import sys
from uuid import uuid4


REPO_ROOT = Path(__file__).resolve().parents[3]
SIMULATOR_ROOT = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "simulator"
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

from utils.rollout_logger import RolloutLogBuffer


def _workspace_tmp_dir() -> Path:
    """使用仓库内临时目录，避免依赖系统 Temp 权限。"""
    tmp_dir = REPO_ROOT / ".pytest_tmp" / f"B03_rollout_logger_{uuid4().hex}"
    tmp_dir.mkdir(parents=True, exist_ok=True)
    return tmp_dir


def test_append_selected_rollout_records_one_row() -> None:
    logger = RolloutLogBuffer()

    logger.append_selected_rollout(
        step=0,
        time=0.01,
        selected_index=2,
        best_cost=1.5,
        mean_cost=3.0,
        first_control=(0.1, -0.2),
        current_actual=(0.4, 0.5),
        current_target=(0.6, 0.7),
    )

    rows = logger.selected_rows()
    assert len(rows) == 1
    assert rows[0].step == 0
    assert rows[0].selected_index == 2
    assert rows[0].first_control_u1 == 0.1
    assert rows[0].first_control_u2 == -0.2


def test_save_selected_rollout_csv() -> None:
    logger = RolloutLogBuffer()
    logger.append_selected_rollout(
        step=1,
        time=0.02,
        selected_index=0,
        best_cost=0.5,
        mean_cost=1.0,
        first_control=(0.0, 0.2),
        current_actual=(0.1, 0.2),
        current_target=(0.3, 0.4),
    )

    output_path = _workspace_tmp_dir() / "logs" / "selected.csv"
    logger.save_selected_rollout_csv(output_path)

    lines = output_path.read_text(encoding="utf-8").strip().splitlines()
    assert lines[0].startswith("step,time,selected_index")
    assert len(lines) == 2


def test_append_candidate_costs_records_all_candidates() -> None:
    logger = RolloutLogBuffer()

    logger.append_candidate_costs(
        step=3,
        costs=[3.0, 1.0, 2.0],
        sorted_indices=[1, 2, 0],
        selected_index=1,
    )

    rows = logger.candidate_cost_rows()
    assert len(rows) == 3
    assert [row.rank for row in rows] == [0, 1, 2]
    assert [row.candidate_index for row in rows] == [1, 2, 0]
    assert rows[0].is_selected is True
    assert rows[1].is_selected is False


def test_save_candidate_costs_csv_has_expected_row_count() -> None:
    logger = RolloutLogBuffer()
    logger.append_candidate_costs(
        step=5,
        costs=[2.5, 0.5],
        sorted_indices=[1, 0],
        selected_index=1,
    )

    output_path = _workspace_tmp_dir() / "logs" / "candidate_costs.csv"
    logger.save_candidate_costs_csv(output_path)

    lines = output_path.read_text(encoding="utf-8").strip().splitlines()
    assert lines[0].startswith("step,candidate_index")
    assert len(lines) == 3
