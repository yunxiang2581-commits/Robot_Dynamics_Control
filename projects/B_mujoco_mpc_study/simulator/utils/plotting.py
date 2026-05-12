"""Plotting skeleton for Project B B01.

本文件只规划图像输出接口，不生成 figures。
"""

from __future__ import annotations

from pathlib import Path


def plot_angle_tracking(time_history: list[float], q_history: list[float], q_target: float, output_path: Path) -> None:
    """规划角度跟踪曲线输出。

    TODO:
    - 要实现什么：绘制 `q(t)` 和 `q_target` 的对比曲线。
    - 为什么要实现：B01 必须可视化关节是否真的跟踪目标。
    - 输入是什么：时间序列、角度历史、目标角、输出路径。
    - 输出是什么：`outputs/figures/B01_angle_tracking.png`。
    - 物理意义：展示关节角度随时间如何靠近目标。
    - 数学意义：可视化状态变量 `q_k` 与参考 `q_target`。
    - 验证标准：图中应包含当前角度曲线、目标线、坐标轴和图例。
    """
    raise NotImplementedError("TODO: 使用 matplotlib 保存 angle tracking figure。")


def plot_angle_error(time_history: list[float], q_history: list[float], q_target: float, output_path: Path) -> None:
    """规划角度误差曲线输出。

    TODO:
    - 要实现什么：绘制 `q(t) - q_target` 随时间变化。
    - 为什么要实现：误差曲线能直接显示控制效果是否收敛。
    - 输入是什么：时间序列、角度历史、目标角、输出路径。
    - 输出是什么：`outputs/figures/B01_angle_error.png`。
    - 物理意义：显示关节偏离目标的程度。
    - 数学意义：可视化 residual `r_q = q - q_target`。
    - 验证标准：如果控制有效，误差绝对值应整体下降或保持在小范围内。
    """
    raise NotImplementedError("TODO: 使用 matplotlib 保存 angle error figure。")


def plot_torque(time_history: list[float], torque_history: list[float], output_path: Path) -> None:
    """规划力矩曲线输出。

    TODO:
    - 要实现什么：绘制每个控制步执行的 torque。
    - 为什么要实现：控制效果必须和控制代价一起看，不能只看误差。
    - 输入是什么：时间序列、力矩历史、输出路径。
    - 输出是什么：`outputs/figures/B01_torque.png`。
    - 物理意义：展示 actuator 施加的控制力矩。
    - 数学意义：可视化控制变量 `u_k = tau_k`。
    - 验证标准：曲线应在 torque limit 范围内，并标注单位或说明。
    """
    raise NotImplementedError("TODO: 使用 matplotlib 保存 torque figure。")


def plot_best_cost(time_history: list[float], best_cost_history: list[float], output_path: Path) -> None:
    """规划 best cost 曲线输出。

    TODO:
    - 要实现什么：绘制每个 MPC 控制周期选中的 best horizon cost。
    - 为什么要实现：best cost 可以显示 planner 在每轮规划中对未来轨迹的评价变化。
    - 输入是什么：时间序列、best cost 历史、输出路径。
    - 输出是什么：`outputs/figures/B01_best_cost.png`。
    - 物理意义：展示控制器认为未来行为是否越来越接近目标。
    - 数学意义：可视化 `min_i J_i` 随控制步变化。
    - 验证标准：曲线应有限、无 NaN，并在日志中解释 cost 权重。
    """
    raise NotImplementedError("TODO: 使用 matplotlib 保存 best cost figure。")
