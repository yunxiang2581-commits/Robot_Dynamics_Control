"""B03 rollout visualization planning utilities.

本文件只生成“应该画什么”的数据计划，不直接调用 MuJoCo，也不生成 MP4。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class RolloutMarkerPlan:
    """当前帧的 rollout 可视化计划。"""

    current_target: tuple[float, float]
    current_actual: tuple[float, float]
    candidate_rollouts_to_draw: list[list[tuple[float, float]]]
    best_rollout: list[tuple[float, float]]
    selected_index: int
    best_cost: float
    mean_cost: float


def _copy_xy_sequence(points: Sequence[tuple[float, float]]) -> list[tuple[float, float]]:
    return [(float(x), float(y)) for x, y in points]


def build_rollout_marker_plan(
    target_xy: Sequence[tuple[float, float]],
    actual_xy: Sequence[tuple[float, float]],
    candidate_rollouts_xy: Sequence[Sequence[tuple[float, float]]],
    best_rollout_xy: Sequence[tuple[float, float]],
    current_step: int,
    costs: Sequence[float],
    max_rollouts_to_draw: int | None = None,
) -> RolloutMarkerPlan:
    """构造当前 step 的 rollout marker 计划。

    输入：
    - `target_xy`：实际闭环中的目标轨迹历史或完整序列。
    - `actual_xy`：实际闭环末端轨迹历史。
    - `candidate_rollouts_xy`：当前 step 采样得到的候选预测轨迹。
    - `best_rollout_xy`：已被 planner 选中的预测轨迹。
    - `current_step`：当前闭环步号。
    - `costs`：每条候选 rollout 的 cost。
    - `max_rollouts_to_draw`：最多绘制多少条候选 rollout。

    输出：
    - 只包含绘图所需的数据，不依赖 MuJoCo scene 或 viewer。

    验证标准：
    - selected index 等于 cost 最小候选。
    - best/mean cost 与输入 costs 一致。
    - 候选 rollout 数量可被 `max_rollouts_to_draw` 限制。
    """
    if current_step < 0:
        raise ValueError(f"current_step must be non-negative, got {current_step}")
    if current_step >= len(target_xy) or current_step >= len(actual_xy):
        raise ValueError(
            "current_step is outside target_xy or actual_xy: "
            f"current_step={current_step}, len(target_xy)={len(target_xy)}, len(actual_xy)={len(actual_xy)}"
        )

    cost_array = np.asarray(costs, dtype=float)
    if cost_array.ndim != 1 or cost_array.size == 0:
        raise ValueError("costs must be a non-empty 1D sequence")
    if np.isnan(cost_array).any():
        raise ValueError("costs must not contain NaN")
    if len(candidate_rollouts_xy) != cost_array.size:
        raise ValueError(
            "candidate_rollouts_xy and costs must have the same length, "
            f"got {len(candidate_rollouts_xy)} and {cost_array.size}"
        )

    selected_index = int(np.argmin(cost_array))
    sorted_indices = np.argsort(cost_array).astype(int).tolist()
    if max_rollouts_to_draw is not None:
        if max_rollouts_to_draw < 0:
            raise ValueError(f"max_rollouts_to_draw must be non-negative, got {max_rollouts_to_draw}")
        sorted_indices = sorted_indices[: int(max_rollouts_to_draw)]

    return RolloutMarkerPlan(
        current_target=(float(target_xy[current_step][0]), float(target_xy[current_step][1])),
        current_actual=(float(actual_xy[current_step][0]), float(actual_xy[current_step][1])),
        candidate_rollouts_to_draw=[
            _copy_xy_sequence(candidate_rollouts_xy[candidate_index]) for candidate_index in sorted_indices
        ],
        best_rollout=_copy_xy_sequence(best_rollout_xy),
        selected_index=selected_index,
        best_cost=float(cost_array[selected_index]),
        mean_cost=float(np.mean(cost_array)),
    )


def draw_rollout_overlay(*args: object, **kwargs: object) -> object:
    """TODO: 绘制 B03 rollout overlay。

    TODO:
    - 要补什么：把 `RolloutMarkerPlan` 画到视频帧或 Matplotlib figure 上。
    - 为什么需要：候选 rollout、best rollout、actual trajectory 需要在同一张图中区分。
    - 推荐做法：复用 B02 的 overlay/marker 风格，但保持为 utils 后处理，不写进 controller。
    - 输入是什么：图像帧、marker plan、样式配置。
    - 输出是什么：带 rollout 标记的图像帧。
    """
    _ = args, kwargs
    raise NotImplementedError("B03-A only defines the draw_rollout_overlay TODO skeleton.")


def plot_rollout_cost_distribution(*args: object, **kwargs: object) -> object:
    """TODO: 绘制某一步候选 rollout cost 分布。

    TODO:
    - 要补什么：使用 candidate cost log 画直方图或排序曲线。
    - 为什么需要：帮助理解 best rollout 是否明显优于其他候选。
    - 输入是什么：某一步的 candidate costs。
    - 输出是什么：cost distribution figure。
    """
    _ = args, kwargs
    raise NotImplementedError("B03-A only defines the plot_rollout_cost_distribution TODO skeleton.")


def plot_best_cost_time(*args: object, **kwargs: object) -> object:
    """TODO: 绘制 best cost 随闭环时间变化。

    TODO:
    - 要补什么：从 selected rollout log 中读取每一步 best_cost。
    - 为什么需要：观察 receding horizon 是否逐渐改善跟踪。
    - 输入是什么：selected rollout rows。
    - 输出是什么：best cost time figure。
    """
    _ = args, kwargs
    raise NotImplementedError("B03-A only defines the plot_best_cost_time TODO skeleton.")
