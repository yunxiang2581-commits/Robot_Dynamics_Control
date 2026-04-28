"""Teaching Mini-WBC QP helpers."""

from __future__ import annotations

from typing import Any


def build_mini_wbc_qp(tasks: list[dict[str, Any]], constraints: dict[str, Any], weights: dict[str, float]) -> dict[str, Any]:
    """Build a teaching Mini-WBC QP.

    TODO(中文):
    - 要实现什么: 组织任务目标、约束和权重, 构造教学版 WBC QP 结构。
    - 输入是什么: task 列表、接触/关节/动力学约束、权重。
    - 输出是什么: QP 矩阵、变量定义和约束说明。
    - 求职重要性: WBC 是腿足控制岗位核心主题, 先掌握 QP 结构再接动力学。
    - 推荐 API: osqp.OSQP, scipy.sparse, 后续 Pinocchio dynamics API。
    - 如何验证: 检查变量维度、任务权重、约束数量和单位一致。
    """
    raise NotImplementedError("TODO: build Mini-WBC QP.")


def solve_mini_wbc_step(problem: dict[str, Any]) -> dict[str, Any]:
    """Solve one teaching Mini-WBC QP step.

    TODO(中文):
    - 要实现什么: 调用 QP solver 得到教学版控制变量。
    - 输入是什么: build_mini_wbc_qp 生成的问题字典。
    - 输出是什么: 求解状态、变量结果、约束检查。
    - 求职重要性: 能解释 solver 输出和失败原因是工程能力的体现。
    - 推荐 API: osqp.OSQP.solve。
    - 如何验证: solver 状态正常, 变量维度与 problem 定义一致。
    """
    raise NotImplementedError("TODO: solve Mini-WBC step.")
