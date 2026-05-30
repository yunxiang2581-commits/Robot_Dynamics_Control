from __future__ import annotations

from pathlib import Path
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SIMULATOR_ROOT = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "simulator"
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

MODULES_BEFORE_ROLLOUT_VISUALIZER_IMPORT = set(sys.modules)
from utils.rollout_visualizer import build_rollout_marker_plan
MODULES_ADDED_BY_ROLLOUT_VISUALIZER_IMPORT = set(sys.modules) - MODULES_BEFORE_ROLLOUT_VISUALIZER_IMPORT


def test_build_rollout_marker_plan_selects_best_rollout() -> None:
    plan = build_rollout_marker_plan(
        target_xy=[(0.0, 0.0), (1.0, 0.0)],
        actual_xy=[(0.0, 0.1), (0.9, 0.1)],
        candidate_rollouts_xy=[
            [(0.0, 0.1), (0.5, 0.2)],
            [(0.0, 0.1), (1.0, 0.0)],
            [(0.0, 0.1), (-0.5, 0.0)],
        ],
        best_rollout_xy=[(0.0, 0.1), (1.0, 0.0)],
        current_step=1,
        costs=[2.0, 0.25, 5.0],
    )

    assert plan.current_target == (1.0, 0.0)
    assert plan.current_actual == (0.9, 0.1)
    assert plan.selected_index == 1
    assert plan.best_cost == pytest.approx(0.25)
    assert plan.mean_cost == pytest.approx(2.4166666667)
    assert plan.best_rollout == [(0.0, 0.1), (1.0, 0.0)]


def test_build_rollout_marker_plan_limits_candidate_count() -> None:
    plan = build_rollout_marker_plan(
        target_xy=[(0.0, 0.0)],
        actual_xy=[(0.0, 0.0)],
        candidate_rollouts_xy=[
            [(0.0, 0.0), (1.0, 0.0)],
            [(0.0, 0.0), (2.0, 0.0)],
            [(0.0, 0.0), (3.0, 0.0)],
        ],
        best_rollout_xy=[(0.0, 0.0), (1.0, 0.0)],
        current_step=0,
        costs=[1.0, 2.0, 3.0],
        max_rollouts_to_draw=2,
    )

    assert len(plan.candidate_rollouts_to_draw) == 2


def test_rollout_visualizer_does_not_import_mujoco() -> None:
    assert "mujoco" not in MODULES_ADDED_BY_ROLLOUT_VISUALIZER_IMPORT
