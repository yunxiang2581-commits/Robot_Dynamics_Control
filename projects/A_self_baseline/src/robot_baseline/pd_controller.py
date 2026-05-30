"""Joint-space PD control helpers."""

from __future__ import annotations

from typing import Any


def compute_pd_torque(q: Any, dq: Any, q_des: Any, dq_des: Any, kp: Any, kd: Any) -> Any:
    """Compute joint PD torque.

    TODO(中文):
    - 要实现什么: 根据位置误差和速度误差计算 tau = kp*(q_des-q)+kd*(dq_des-dq)。
    - 输入是什么: 当前 q/dq、期望 q_des/dq_des、增益 kp/kd。
    - 输出是什么: 关节力矩 tau。
    - 求职重要性: PD 是机器人控制实验最小闭环, 能体现稳定性和参数调试能力。
    - 推荐 API: numpy 数组广播和形状检查。
    - 如何验证: 零误差时 torque 为零, 正位置误差产生期望方向力矩。
    """
    raise NotImplementedError("TODO: compute PD torque.")
