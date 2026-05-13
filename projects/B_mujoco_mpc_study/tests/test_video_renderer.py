from __future__ import annotations

from pathlib import Path
import sys
import uuid

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from projects.B_mujoco_mpc_study.simulator.utils.marker_renderer import build_frame_marker_plan
from projects.B_mujoco_mpc_study.simulator.utils.video_renderer import render_marked_video, validate_frame_and_log_lengths


def test_validate_frame_and_log_lengths_accepts_equal_lengths() -> None:
    validate_frame_and_log_lengths(raw_frames=[np.zeros((4, 4, 3), dtype=np.uint8)] * 2, num_rows=2)


def test_validate_frame_and_log_lengths_rejects_mismatch() -> None:
    with pytest.raises(ValueError, match="raw_frames"):
        validate_frame_and_log_lengths(raw_frames=[np.zeros((4, 4, 3), dtype=np.uint8)] * 2, num_rows=3)


def test_render_marked_video_writes_mp4_from_raw_frames_and_rows() -> None:
    temp_dir = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "outputs" / "pytest_tmp"
    temp_dir.mkdir(parents=True, exist_ok=True)
    output_path = temp_dir / f"marked_{uuid.uuid4().hex}.mp4"

    rows = []
    for i in range(2):
        rows.append(
            {
                "step": i,
                "time": 0.01 * (i + 1),
                "target_x": 0.1 + 0.05 * i,
                "target_y": 0.2 + 0.05 * i,
                "actual_x": 0.11 + 0.03 * i,
                "actual_y": 0.19 + 0.04 * i,
                "error_norm": 0.02,
                "mpc_cost": 1.0 + i,
            }
        )

    raw_frames = [np.zeros((120, 160, 3), dtype=np.uint8) for _ in range(2)]

    try:
        render_marked_video(raw_frames=raw_frames, tracking_rows=rows, output_path=output_path, fps=10)
        assert output_path.exists()
        assert output_path.stat().st_size > 0
    finally:
        if output_path.exists():
            output_path.unlink()


def test_render_marked_video_supports_scene_mode_with_callback() -> None:
    temp_dir = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "outputs" / "pytest_tmp"
    temp_dir.mkdir(parents=True, exist_ok=True)
    output_path = temp_dir / f"scene_{uuid.uuid4().hex}.mp4"

    callback_calls: list[tuple[int, object]] = []
    rows = [
        {
            "step": 0,
            "time": 0.01,
            "target_x": 0.1,
            "target_y": 0.2,
            "actual_x": 0.11,
            "actual_y": 0.19,
            "error_norm": 0.02,
            "mpc_cost": 1.0,
            "u1": 0.0,
            "u2": 0.0,
        }
    ]

    def fake_scene_provider(step_index: int, tracking_rows: list[dict[str, float]]) -> object:
        return {"step": step_index, "rows": len(tracking_rows)}

    def fake_scene_frame_renderer(step_index: int, marker_payload: object) -> np.ndarray:
        callback_calls.append((step_index, marker_payload))
        return np.full((64, 64, 3), 80, dtype=np.uint8)

    try:
        render_marked_video(
            raw_frames=[np.zeros((64, 64, 3), dtype=np.uint8)],
            tracking_rows=rows,
            output_path=output_path,
            fps=10,
            render_mode="scene",
            scene_marker_provider=fake_scene_provider,
            scene_frame_renderer=fake_scene_frame_renderer,
        )
        assert output_path.exists()
        assert callback_calls == [(0, {"step": 0, "rows": 1})]
    finally:
        if output_path.exists():
            output_path.unlink()


def test_render_marked_video_supports_hybrid_mode_with_scene_and_overlay() -> None:
    temp_dir = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "outputs" / "pytest_tmp"
    temp_dir.mkdir(parents=True, exist_ok=True)
    output_path = temp_dir / f"hybrid_{uuid.uuid4().hex}.mp4"

    rows = [
        {
            "step": 0,
            "time": 0.01,
            "target_x": 0.1,
            "target_y": 0.2,
            "actual_x": 0.11,
            "actual_y": 0.19,
            "error_norm": 0.02,
            "mpc_cost": 1.0,
            "u1": 0.0,
            "u2": 0.0,
        }
    ]

    def fake_scene_provider(step_index: int, tracking_rows: list[dict[str, float]]) -> object:
        return {"step": step_index}

    def fake_scene_frame_renderer(step_index: int, marker_payload: object) -> np.ndarray:
        _ = marker_payload
        return np.zeros((120, 160, 3), dtype=np.uint8)

    try:
        render_marked_video(
            raw_frames=[np.zeros((120, 160, 3), dtype=np.uint8)],
            tracking_rows=rows,
            output_path=output_path,
            fps=10,
            render_mode="hybrid",
            scene_marker_provider=fake_scene_provider,
            scene_frame_renderer=fake_scene_frame_renderer,
        )
        assert output_path.exists()
        assert output_path.stat().st_size > 0
    finally:
        if output_path.exists():
            output_path.unlink()
