"""QP-based inverse kinematics helpers with constraints."""

from __future__ import annotations

from typing import Any


def build_qp_ik_problem(jacobian: Any, task_velocity: Any, q: Any, limits: dict[str, Any], weights: dict[str, float]) -> dict[str, Any]:
    """Build matrices for a constrained QP-IK problem.

    TODO(中文):
    - 要实现什么: 构造 P、q、A、l、u 等 OSQP 输入矩阵。
    - 输入是什么: Jacobian、任务速度、当前 q、关节限制、权重。
    - 输出是什么: QP 矩阵和变量说明。
    - 求职重要性: QP-IK 是从运动学控制走向约束优化控制的关键。
    - 推荐 API: osqp.OSQP, scipy.sparse.csc_matrix。
    - 如何验证: 检查矩阵维度一致, 上下限没有反向, P 半正定。
    """
    raise NotImplementedError("TODO: build QP-IK problem.")


def solve_qp_ik_step(problem: dict[str, Any]) -> dict[str, Any]:
    """Solve one QP-IK step.

    TODO(中文):
    - 要实现什么: 调用 OSQP 求解 dq 并返回状态。
    - 输入是什么: build_qp_ik_problem 生成的 problem。
    - 输出是什么: dq、solver 状态、目标值、约束违反量。
    - 求职重要性: 会读 solver 状态和约束违反量是工程调试必需能力。
    - 推荐 API: osqp.OSQP.setup, solver.solve。
    - 如何验证: solver status 为 solved, dq 满足关节速度/位置约束。
    """
    raise NotImplementedError("TODO: solve QP-IK step.")
