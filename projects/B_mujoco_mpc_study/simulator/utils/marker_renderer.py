"""B02 frame marker planning and overlay rendering utilities."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np
from PIL import Image, ImageDraw


@dataclass(frozen=True)
class FrameMarkerPlan:
    """第 k 帧应显示的时序 marker 计划。"""

    current_target: tuple[float, float]
    target_history: list[tuple[float, float]]
    target_future: list[tuple[float, float]]
    current_actual: tuple[float, float]
    error_line: tuple[tuple[float, float], tuple[float, float]]
    text_items: list[str]


@dataclass(frozen=True)
class SceneMarkerGeom:
    """MuJoCo scene marker 几何体描述。"""

    geom_type: str
    pos: tuple[float, float, float]
    size: tuple[float, float, float]
    rgba: tuple[float, float, float, float]
    from_pos: tuple[float, float, float] | None = None
    to_pos: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class SceneMarkerPayload:
    """单帧 scene marker 负载。"""

    geoms: list[SceneMarkerGeom]


@dataclass(frozen=True)
class MarkerStyleConfig:
    """第一版 2D overlay 的最小样式配置。"""

    current_target_color: tuple[int, int, int] = (255, 80, 80)
    history_color: tuple[int, int, int] = (255, 180, 60)
    future_color: tuple[int, int, int] = (120, 120, 255)
    actual_color: tuple[int, int, int] = (80, 255, 120)
    error_line_color: tuple[int, int, int] = (255, 255, 255)
    panel_background_color: tuple[int, int, int] = (24, 24, 24)
    panel_border_color: tuple[int, int, int] = (180, 180, 180)
    text_color: tuple[int, int, int] = (255, 255, 255)
    panel_width: int = 220
    panel_height: int = 200
    panel_margin: int = 12


def build_scene_marker_payload(
    step_index: int,
    tracking_rows: list[dict[str, float]],
    z_height: float = 0.0,
) -> SceneMarkerPayload:
    """构造第二版 scene marker 负载。

    输入：
    - `step_index`：当前帧索引
    - `tracking_rows`：统一 tracking log
    - `z_height`：平面 marker 的 z 高度

    输出：
    - `SceneMarkerPayload`

    验证标准：
    - 包含 current target
    - 包含 current actual
    - 包含 error line
    - 包含 target history trail
    - 包含 target future preview
    """
    row = tracking_rows[step_index]
    current_target = (float(row["target_x"]), float(row["target_y"]), float(z_height))
    current_actual = (float(row["actual_x"]), float(row["actual_y"]), float(z_height))

    history_points = [
        (float(item["target_x"]), float(item["target_y"]), float(z_height))
        for item in tracking_rows[: step_index + 1]
    ]
    future_points = [
        (float(item["target_x"]), float(item["target_y"]), float(z_height))
        for item in tracking_rows[step_index:]
    ]

    geoms: list[SceneMarkerGeom] = [
        SceneMarkerGeom(
            geom_type="sphere",
            pos=current_target,
            size=(0.02, 0.02, 0.02),
            rgba=(1.0, 0.3, 0.3, 1.0),
        ),
        SceneMarkerGeom(
            geom_type="sphere",
            pos=current_actual,
            size=(0.02, 0.02, 0.02),
            rgba=(0.3, 1.0, 0.5, 1.0),
        ),
        SceneMarkerGeom(
            geom_type="line",
            pos=current_actual,
            from_pos=current_actual,
            to_pos=current_target,
            size=(3.0, 0.0, 0.0),
            rgba=(1.0, 1.0, 1.0, 1.0),
        ),
    ]

    for from_pos, to_pos in zip(history_points[:-1], history_points[1:]):
        geoms.append(
            SceneMarkerGeom(
                geom_type="line",
                pos=from_pos,
                from_pos=from_pos,
                to_pos=to_pos,
                size=(2.0, 0.0, 0.0),
                rgba=(1.0, 0.71, 0.24, 0.9),
            )
        )

    for from_pos, to_pos in zip(future_points[:-1], future_points[1:]):
        geoms.append(
            SceneMarkerGeom(
                geom_type="line",
                pos=from_pos,
                from_pos=from_pos,
                to_pos=to_pos,
                size=(1.0, 0.0, 0.0),
                rgba=(0.47, 0.47, 1.0, 0.75),
            )
        )

    return SceneMarkerPayload(geoms=geoms)


def build_frame_marker_plan(
    step_index: int,
    target_xy: list[tuple[float, float]],
    actual_xy: list[tuple[float, float]],
    times: list[float],
    error_norms: list[float],
    mpc_costs: list[float],
) -> FrameMarkerPlan:
    """构造第 k 帧 marker 计划。"""
    current_target = target_xy[step_index]
    current_actual = actual_xy[step_index]
    return FrameMarkerPlan(
        current_target=current_target,
        target_history=target_xy[: step_index + 1],
        target_future=target_xy[step_index:],
        current_actual=current_actual,
        error_line=(current_actual, current_target),
        text_items=[
            f"time: {times[step_index]:.3f}s",
            f"error_norm: {error_norms[step_index]:.6f}",
            f"mpc_cost: {mpc_costs[step_index]:.6f}",
        ],
    )


def draw_overlay(frame: Any, marker_plan: FrameMarkerPlan, style: MarkerStyleConfig | None = None) -> Any:
    """绘制第一版 2D overlay。"""
    style = style or MarkerStyleConfig()
    frame_array = np.array(frame, copy=True)
    if frame_array.ndim != 3 or frame_array.shape[2] != 3:
        raise ValueError(f"frame must be HxWx3 RGB, got shape={getattr(frame_array, 'shape', None)}")

    image = Image.fromarray(frame_array.astype(np.uint8), mode="RGB")
    draw = ImageDraw.Draw(image)

    width, height = image.size
    panel_left = max(0, width - style.panel_width - style.panel_margin)
    panel_top = style.panel_margin
    panel_right = width - style.panel_margin
    panel_bottom = min(height - style.panel_margin, panel_top + style.panel_height)

    draw.rectangle(
        [(panel_left, panel_top), (panel_right, panel_bottom)],
        fill=style.panel_background_color,
        outline=style.panel_border_color,
        width=2,
    )

    points = marker_plan.target_history + marker_plan.target_future + [marker_plan.current_actual]
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    if math.isclose(min_x, max_x):
        min_x -= 0.05
        max_x += 0.05
    if math.isclose(min_y, max_y):
        min_y -= 0.05
        max_y += 0.05

    inner_margin = 16
    plot_left = panel_left + inner_margin
    plot_top = panel_top + inner_margin
    plot_right = panel_right - inner_margin
    plot_bottom = panel_bottom - 58

    def map_point(point: tuple[float, float]) -> tuple[float, float]:
        x, y = point
        px = plot_left + (x - min_x) / (max_x - min_x) * max(1.0, plot_right - plot_left)
        py = plot_bottom - (y - min_y) / (max_y - min_y) * max(1.0, plot_bottom - plot_top)
        return float(px), float(py)

    def draw_polyline(points_xy: list[tuple[float, float]], color: tuple[int, int, int], width_px: int) -> None:
        if len(points_xy) < 2:
            return
        mapped = [map_point(point) for point in points_xy]
        draw.line(mapped, fill=color, width=width_px)

    def draw_point(point_xy: tuple[float, float], color: tuple[int, int, int], radius: int) -> None:
        px, py = map_point(point_xy)
        draw.ellipse([(px - radius, py - radius), (px + radius, py + radius)], fill=color)

    draw_polyline(marker_plan.target_history, style.history_color, 3)
    draw_polyline(marker_plan.target_future, style.future_color, 2)

    actual_point, target_point = marker_plan.error_line
    draw.line([map_point(actual_point), map_point(target_point)], fill=style.error_line_color, width=2)
    draw_point(marker_plan.current_target, style.current_target_color, 5)
    draw_point(marker_plan.current_actual, style.actual_color, 5)

    text_y = plot_bottom + 8
    for text_item in marker_plan.text_items:
        draw.text((plot_left, text_y), text_item, fill=style.text_color)
        text_y += 15

    return np.asarray(image)
