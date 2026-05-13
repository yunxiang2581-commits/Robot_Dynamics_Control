"""Plotting utilities for Project B.

当前文件同时承担两类职责：
- 保留 B01 早期教学接口名，继续作为 skeleton
- 提供 B02 已独立出来的最小图表后处理实现
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt


def plot_angle_tracking(time_history: list[float], q_history: list[float], q_target: float, output_path: Path) -> None:
    """规划角度跟踪曲线输出。

    TODO:
    - 要实现什么：绘制 `q(t)` 和 `q_target` 的对比曲线。
    - 为什么需要：B01 必须可视化关节是否真的跟踪目标。
    - 输入是什么：时间序列、角度历史、目标角、输出路径。
    - 输出是什么：`outputs/figures/B01_angle_tracking.png`。
    - 验证标准：图中应包含当前角度曲线、目标线、坐标轴和图例。
    """
    raise NotImplementedError("TODO: 使用 matplotlib 保存 angle tracking figure。")


def plot_angle_error(time_history: list[float], q_history: list[float], q_target: float, output_path: Path) -> None:
    """规划角度误差曲线输出。

    TODO:
    - 要实现什么：绘制 `q(t) - q_target` 随时间变化。
    - 为什么需要：误差曲线能直接显示控制效果是否收敛。
    - 输入是什么：时间序列、角度历史、目标角、输出路径。
    - 输出是什么：`outputs/figures/B01_angle_error.png`。
    - 验证标准：如果控制有效，误差绝对值应整体下降或保持在小范围内。
    """
    raise NotImplementedError("TODO: 使用 matplotlib 保存 angle error figure。")


def plot_torque(time_history: list[float], torque_history: list[float], output_path: Path) -> None:
    """规划力矩曲线输出。

    TODO:
    - 要实现什么：绘制每个控制步执行的 torque。
    - 为什么需要：控制效果必须和控制代价一起看，不能只看误差。
    - 输入是什么：时间序列、力矩历史、输出路径。
    - 输出是什么：`outputs/figures/B01_torque.png`。
    - 验证标准：曲线应在 torque limit 范围内，并标注单位或说明。
    """
    raise NotImplementedError("TODO: 使用 matplotlib 保存 torque figure。")


def plot_best_cost(time_history: list[float], best_cost_history: list[float], output_path: Path) -> None:
    """规划 best cost 曲线输出。

    TODO:
    - 要实现什么：绘制每个 MPC 控制周期选中的 best horizon cost。
    - 为什么需要：best cost 可以显示 planner 对未来轨迹评价的变化。
    - 输入是什么：时间序列、best cost 历史、输出路径。
    - 输出是什么：`outputs/figures/B01_best_cost.png`。
    - 验证标准：曲线应有界、无 NaN。
    """
    raise NotImplementedError("TODO: 使用 matplotlib 保存 best cost figure。")


def save_b02_tracking_figures(tracking_rows: list[dict[str, float]], outputs: dict[str, Path]) -> None:
    """保存 B02 独立后处理图表。

    TODO:
    - 要实现什么：根据 tracking log 生成 target-vs-actual XY、tracking error-time、control input-time 三张图。
    - 为什么需要：B02 的图表应从统一 tracking log 出发，而不是继续散落在 runner 的 history 变量里。
    - 输入是什么：tracking_rows 和输出路径字典。
    - 输出是什么：三张 PNG 图像。
    - 验证标准：三张图都应成功写出，且文件大小大于 0。
    """
    if not tracking_rows:
        raise ValueError("tracking_rows 不能为空，无法生成 B02 图表。")

    time_history = [float(row["time"]) for row in tracking_rows]
    target_x = [float(row["target_x"]) for row in tracking_rows]
    target_y = [float(row["target_y"]) for row in tracking_rows]
    actual_x = [float(row["actual_x"]) for row in tracking_rows]
    actual_y = [float(row["actual_y"]) for row in tracking_rows]
    error_history = [float(row["error_norm"]) for row in tracking_rows]
    u1_history = [float(row["u1"]) for row in tracking_rows]
    u2_history = [float(row["u2"]) for row in tracking_rows]

    outputs["xy_target_vs_actual"].parent.mkdir(parents=True, exist_ok=True)
    plt.figure(figsize=(5, 5))
    plt.plot(target_x, target_y, "--", label="target")
    plt.plot(actual_x, actual_y, label="actual")
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("B02 target vs actual XY")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(outputs["xy_target_vs_actual"])
    plt.close()

    outputs["tracking_error_time"].parent.mkdir(parents=True, exist_ok=True)
    plt.figure(figsize=(8, 4.5))
    plt.plot(time_history, error_history)
    plt.xlabel("time [s]")
    plt.ylabel("tracking error [m]")
    plt.title("B02 tracking error vs time")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(outputs["tracking_error_time"])
    plt.close()

    outputs["control_input_time"].parent.mkdir(parents=True, exist_ok=True)
    plt.figure(figsize=(8, 4.5))
    plt.plot(time_history, u1_history, label="u1")
    plt.plot(time_history, u2_history, label="u2")
    plt.xlabel("time [s]")
    plt.ylabel("control input [Nm]")
    plt.title("B02 control input vs time")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(outputs["control_input_time"])
    plt.close()
