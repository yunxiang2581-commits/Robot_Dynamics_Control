"""Metrics skeleton for Project B B01.

本文件只规划 metrics 接口，不生成 CSV。
"""

from __future__ import annotations

from pathlib import Path


def compute_final_error(q_history: list[float], q_target: float) -> float:
    """计算最终角度误差。

    TODO:
    - 要实现什么：取最后一个关节角 `q_final`，计算 `q_final - q_target` 的绝对值或带符号误差。
    - 为什么要实现：final error 是判断最后是否接近目标的核心指标。
    - 输入是什么：角度历史 `q_history` 和目标角 `q_target`。
    - 输出是什么：最终角度误差。
    - 物理意义：实验结束时关节离目标还有多远。
    - 数学意义：`e_final = q_N - q_target` 或 `abs(q_N - q_target)`。
    - 验证标准：如果最后角度等于目标，误差应为 0。
    """
    raise NotImplementedError("TODO: 计算 final tracking error。")


def compute_mean_tracking_error(q_history: list[float], q_target: float) -> float:
    """计算平均跟踪误差。

    TODO:
    - 要实现什么：对整段轨迹的 `abs(q - q_target)` 求平均。
    - 为什么要实现：仅看 final error 不够，平均误差能反映整个过程的跟踪质量。
    - 输入是什么：角度历史 `q_history` 和目标角 `q_target`。
    - 输出是什么：平均跟踪误差。
    - 物理意义：整个仿真过程中关节偏离目标的平均程度。
    - 数学意义：`mean(abs(q_k - q_target))`。
    - 验证标准：所有角度都等于目标时，平均误差应为 0。
    """
    raise NotImplementedError("TODO: 计算 mean tracking error。")


def compute_max_torque(torque_history: list[float]) -> float:
    """计算最大力矩幅值。

    TODO:
    - 要实现什么：计算执行 torque 历史中的最大绝对值。
    - 为什么要实现：max torque 用于检查控制是否超过限制或过于激进。
    - 输入是什么：力矩历史 `torque_history`。
    - 输出是什么：最大力矩幅值。
    - 物理意义：actuator 在实验中承受的最大控制力矩。
    - 数学意义：`max(abs(tau_k))`。
    - 验证标准：结果不应超过配置的 torque limit。
    """
    raise NotImplementedError("TODO: 计算 max torque。")


def compute_runtime_per_control_step(runtime_history: list[float]) -> float:
    """计算平均每步规划耗时。

    TODO:
    - 要实现什么：对每个控制周期的 planner runtime 求平均。
    - 为什么要实现：MPC 是否能实时运行取决于每步规划耗时。
    - 输入是什么：每步控制耗时 `runtime_history`。
    - 输出是什么：平均 runtime per control step。
    - 物理意义：控制器每次做决策需要多少时间。
    - 数学意义：`mean(runtime_k)`。
    - 验证标准：候选数量增加时，平均耗时通常应增加。
    """
    raise NotImplementedError("TODO: 计算 runtime per control step。")


def save_metrics_csv(metrics: dict[str, float], output_path: Path) -> None:
    """保存 metrics CSV。

    TODO:
    - 要实现什么：把 final error、mean error、max torque、runtime 等指标写入 CSV。
    - 为什么要实现：Project B 要求每个 demo 必须有 metrics CSV，便于复盘和对比。
    - 输入是什么：metrics 字典和输出路径。
    - 输出是什么：`outputs/metrics/B01_metrics.csv` 文件。
    - 物理意义：把仿真实验结果变成可量化证据。
    - 数学意义：保存标量评价指标，方便横向比较不同参数配置。
    - 验证标准：CSV 包含表头，每个指标有明确名称和值。
    """
    raise NotImplementedError("TODO: 写入 metrics CSV。")
