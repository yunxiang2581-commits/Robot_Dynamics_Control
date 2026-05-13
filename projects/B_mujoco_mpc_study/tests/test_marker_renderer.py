from __future__ import annotations

from pathlib import Path
import sys

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from projects.B_mujoco_mpc_study.simulator.utils.marker_renderer import (
    build_frame_marker_plan,
    build_scene_marker_payload,
    draw_overlay,
)


def test_build_frame_marker_plan_splits_current_history_future() -> None:
    plan = build_frame_marker_plan(
        step_index=1,
        target_xy=[(0.1, 0.1), (0.2, 0.2), (0.3, 0.3)],
        actual_xy=[(0.11, 0.12), (0.18, 0.19), (0.31, 0.33)],
        times=[0.01, 0.02, 0.03],
        error_norms=[0.01, 0.02, 0.03],
        mpc_costs=[3.0, 2.0, 1.0],
    )

    assert plan.current_target == (0.2, 0.2)
    assert plan.target_history == [(0.1, 0.1), (0.2, 0.2)]
    assert plan.target_future == [(0.2, 0.2), (0.3, 0.3)]
    assert plan.current_actual == (0.18, 0.19)
    assert plan.error_line == ((0.18, 0.19), (0.2, 0.2))
    assert "time" in plan.text_items[0].lower()


def test_draw_overlay_modifies_frame_pixels() -> None:
    frame = np.zeros((120, 160, 3), dtype=np.uint8)
    plan = build_frame_marker_plan(
        step_index=1,
        target_xy=[(0.1, 0.1), (0.2, 0.2), (0.3, 0.3)],
        actual_xy=[(0.11, 0.12), (0.18, 0.19), (0.31, 0.33)],
        times=[0.01, 0.02, 0.03],
        error_norms=[0.01, 0.02, 0.03],
        mpc_costs=[3.0, 2.0, 1.0],
    )

    overlaid = draw_overlay(frame, plan)

    assert overlaid.shape == frame.shape
    assert np.count_nonzero(overlaid) > 0


def test_build_scene_marker_payload_contains_current_history_future_actual_and_error() -> None:
    payload = build_scene_marker_payload(
        step_index=1,
        tracking_rows=[
            {
                "step": 0,
                "target_x": 0.1,
                "target_y": 0.2,
                "actual_x": 0.11,
                "actual_y": 0.19,
            },
            {
                "step": 1,
                "target_x": 0.2,
                "target_y": 0.3,
                "actual_x": 0.18,
                "actual_y": 0.28,
            },
            {
                "step": 2,
                "target_x": 0.3,
                "target_y": 0.4,
                "actual_x": 0.31,
                "actual_y": 0.39,
            },
        ],
    )

    geom_types = [geom.geom_type for geom in payload.geoms]

    assert geom_types.count("sphere") >= 2
    assert geom_types.count("line") >= 3
