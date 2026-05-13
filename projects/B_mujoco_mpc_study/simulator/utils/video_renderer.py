"""B02 marked video renderer skeleton."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable

import imageio.v2 as imageio

from projects.B_mujoco_mpc_study.simulator.utils.marker_renderer import (
    MarkerStyleConfig,
    build_scene_marker_payload,
    build_frame_marker_plan,
    draw_overlay,
)


def validate_frame_and_log_lengths(raw_frames: list[Any], num_rows: int) -> None:
    """检查 raw frame 与 tracking log 长度一致。

    TODO:
    - 要实现什么：检查 raw frame 数量和 tracking log 行数是否严格一致。
    - 为什么需要：按时序打 marker 的前提是视频帧与日志一一对应。
    - 输入是什么：raw_frames 和 num_rows。
    - 输出是什么：无返回值，若不一致则抛异常。
    - 验证标准：数量相等时通过，不相等时错误信息中应包含双方长度。
    """
    if len(raw_frames) != num_rows:
        raise ValueError(
            f"raw_frames 与 tracking_rows 数量不一致: len(raw_frames)={len(raw_frames)}, num_rows={num_rows}"
        )


def render_marked_video(
    raw_frames: list[Any],
    tracking_rows: list[Any],
    output_path: Path,
    fps: int,
    style: Any | None = None,
    render_mode: str = "overlay",
    scene_marker_provider: Any | None = None,
    scene_frame_renderer: Callable[[int, Any], Any] | None = None,
) -> None:
    """第一版标记视频导出入口。

    TODO:
    - 要实现什么：消费 raw_frames 和 tracking_rows，逐帧生成带 2D overlay 的 marked video。
    - 为什么需要：第一版 B02 的核心验收项是稳定生成按时序标记的 MP4。
    - 输入是什么：raw_frames、tracking_rows、output_path、fps、可选 style 和 scene_marker_provider。
    - 输出是什么：marked MP4 文件。
    - 验证标准：输出视频帧数应与 raw_frames 和 tracking_rows 完全对齐。
    """
    if render_mode not in {"overlay", "scene", "hybrid"}:
        raise ValueError(f"render_mode must be one of overlay/scene/hybrid, got {render_mode}")
    validate_frame_and_log_lengths(raw_frames=raw_frames, num_rows=len(tracking_rows))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    overlay_style = style if style is not None else MarkerStyleConfig()

    target_xy = [(float(row["target_x"]), float(row["target_y"])) for row in tracking_rows]
    actual_xy = [(float(row["actual_x"]), float(row["actual_y"])) for row in tracking_rows]
    times = [float(row["time"]) for row in tracking_rows]
    error_norms = [float(row["error_norm"]) for row in tracking_rows]
    mpc_costs = [float(row["mpc_cost"]) for row in tracking_rows]

    marked_frames: list[Any] = []
    for step_index, frame in enumerate(raw_frames):
        marker_plan = build_frame_marker_plan(
            step_index=step_index,
            target_xy=target_xy,
            actual_xy=actual_xy,
            times=times,
            error_norms=error_norms,
            mpc_costs=mpc_costs,
        )
        scene_payload = None
        if scene_marker_provider is not None:
            scene_payload = scene_marker_provider(step_index, tracking_rows)
        elif render_mode in {"scene", "hybrid"}:
            scene_payload = build_scene_marker_payload(step_index, tracking_rows)

        if render_mode == "overlay":
            output_frame = draw_overlay(frame, marker_plan, overlay_style)
        elif render_mode == "scene":
            if scene_frame_renderer is None:
                raise ValueError("scene 模式需要 scene_frame_renderer。")
            output_frame = scene_frame_renderer(step_index, scene_payload)
        else:
            if scene_frame_renderer is None:
                raise ValueError("hybrid 模式需要 scene_frame_renderer。")
            scene_frame = scene_frame_renderer(step_index, scene_payload)
            output_frame = draw_overlay(scene_frame, marker_plan, overlay_style)

        marked_frames.append(output_frame)

    imageio.mimsave(output_path, marked_frames, fps=fps)
