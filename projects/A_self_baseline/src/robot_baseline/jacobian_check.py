"""Jacobian computation and finite-difference validation helpers."""

from __future__ import annotations

from typing import Any


def compute_frame_jacobian(model: Any, data: Any, q: Any, frame_name: str) -> Any:
    """Compute a frame Jacobian.

    TODO(中文):
    - 要实现什么: 计算目标 frame 的 6D Jacobian。
    - 输入是什么: model、data、q、frame_name。
    - 输出是什么: 形状通常为 6 x nv 的 Jacobian。
    - 求职重要性: Jacobian 是 IK、速度控制、QP-IK 和 WBC 的核心矩阵。
    - 推荐 API: pin.computeJointJacobians, pin.updateFramePlacements, pin.getFrameJacobian。
    - 如何验证: 检查矩阵维度为 6 x model.nv。
    """
    raise NotImplementedError("TODO: compute frame Jacobian.")


def finite_difference_frame_velocity(model: Any, data: Any, q: Any, dq: Any, frame_name: str, dt: float) -> Any:
    """Estimate frame velocity with finite differences.

    TODO(中文):
    - 要实现什么: 比较 q 和 q+dq*dt 的 frame 位姿变化, 估计速度。
    - 输入是什么: model、data、q、dq、frame_name、dt。
    - 输出是什么: 有限差分得到的线速度/角速度近似。
    - 求职重要性: 能验证 Jacobian 推导和坐标系选择是否正确。
    - 推荐 API: pin.integrate, pin.forwardKinematics, pin.log。
    - 如何验证: 与 J @ dq 比较, 误差应在合理范围。
    """
    raise NotImplementedError("TODO: finite-difference frame velocity.")


def compare_jacobian_with_finite_difference(jacobian: Any, dq: Any, fd_velocity: Any) -> dict[str, float]:
    """Compare analytic Jacobian velocity against finite differences.

    TODO(中文):
    - 要实现什么: 计算 J @ dq 与有限差分速度的误差范数。
    - 输入是什么: jacobian、dq、fd_velocity。
    - 输出是什么: 包含平移/旋转/总误差的 dict。
    - 求职重要性: 展示数值验证能力, 避免只会调用库函数。
    - 推荐 API: numpy.linalg.norm。
    - 如何验证: 使用小 dt 时误差不应异常增大。
    """
    raise NotImplementedError("TODO: compare Jacobian with finite difference.")
