"""B03 SQP / direct multiple shooting NMPC learning skeleton.

核心教学目标：
- 理解 SQP 与 direct multiple shooting 的问题结构；
- 理解为什么 full NMPC 比 sampling MPC 更复杂；
- 仅在简单模型中先搭问题结构，不直接进入 OpenLoong。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from planners.mpc_solver_interface import BaseMPCSolver, MPCProblem, MPCSolution


@dataclass(frozen=True)
class SQPConfig:
    """SQP-MPC 配置骨架。"""

    horizon: int
    max_sqp_iterations: int
    qp_solver: str


@dataclass(frozen=True)
class NLPDecisionLayout:
    """multiple shooting 决策变量布局。"""

    state_slices: list[slice]
    control_slices: list[slice]
    total_size: int


@dataclass(frozen=True)
class DynamicsConstraint:
    """单步动力学等式约束骨架。"""

    step: int
    defect_dim: int


@dataclass(frozen=True)
class MultipleShootingProblem:
    """multiple shooting 问题描述骨架。"""

    horizon: int
    state_dim: int
    control_dim: int
    decision_layout: NLPDecisionLayout


def build_multiple_shooting_decision_layout(
    horizon: int,
    state_dim: int,
    control_dim: int,
) -> NLPDecisionLayout:
    """TODO: 定义 `z = [x0, u0, x1, u1, ..., xH]` 的索引布局。

    TODO:
    - 输入：horizon、状态维度、控制维度。
    - 输出：每个 `x_k` / `u_k` 在一维决策向量里的切片。
    - 为什么需要：后续 pack/unpack、约束和目标函数都依赖统一索引布局。
    """
    _ = horizon, state_dim, control_dim
    raise NotImplementedError("B03-A only defines the build_multiple_shooting_decision_layout TODO skeleton.")


def pack_decision_variables(
    states: np.ndarray,
    controls: np.ndarray,
    layout: NLPDecisionLayout,
) -> np.ndarray:
    """TODO: 把 `X, U` 打包成一维决策向量 `z`。"""
    _ = states, controls, layout
    raise NotImplementedError("B03-A only defines the pack_decision_variables TODO skeleton.")


def unpack_decision_variables(
    decision_vector: np.ndarray,
    horizon: int,
    state_dim: int,
    control_dim: int,
    layout: NLPDecisionLayout,
) -> tuple[np.ndarray, np.ndarray]:
    """TODO: 从一维 `z` 还原 `X, U`。"""
    _ = decision_vector, horizon, state_dim, control_dim, layout
    raise NotImplementedError("B03-A only defines the unpack_decision_variables TODO skeleton.")


def build_dynamics_defects(
    env: Any,
    states: np.ndarray,
    controls: np.ndarray,
    dt: float,
) -> np.ndarray:
    """TODO: 计算 `defects[k] = x_{k+1} - f(x_k, u_k)`。"""
    _ = env, states, controls, dt
    raise NotImplementedError("B03-A only defines the build_dynamics_defects TODO skeleton.")


def build_nmpc_objective(
    states: np.ndarray,
    controls: np.ndarray,
    target_horizon: np.ndarray,
    cost_config: dict[str, Any],
) -> float:
    """TODO: 构造 stage cost + terminal cost 的总目标函数。"""
    _ = states, controls, target_horizon, cost_config
    raise NotImplementedError("B03-A only defines the build_nmpc_objective TODO skeleton.")


class SQPMPCSolver(BaseMPCSolver):
    """SQP-MPC learning skeleton.

    教学说明：
    - SQP 把非线性优化问题在名义轨迹附近近似成一系列 QP 子问题；
    - direct multiple shooting 提供了更显式的状态/控制决策变量与动力学缺陷约束；
    - full NMPC 比 sampling MPC 更复杂，因为不仅要 rollout 和比较 cost，
      还要显式处理约束、局部线性化和优化变量更新。
    """

    solver_name = "sqp_mpc_todo"

    def solve(
        self,
        problem: MPCProblem,
        previous_solution: MPCSolution | None = None,
    ) -> MPCSolution:
        """TODO: 线性化约束、构造 QP 子问题并更新轨迹。"""
        _ = problem, previous_solution
        raise NotImplementedError("B03-A only defines the SQPMPCSolver TODO skeleton.")


class DirectMultipleShootingNMPCSolver(BaseMPCSolver):
    """Direct multiple shooting NMPC learning skeleton."""

    solver_name = "direct_multiple_shooting_todo"

    def solve(
        self,
        problem: MPCProblem,
        previous_solution: MPCSolution | None = None,
    ) -> MPCSolution:
        """TODO: 构造 NLP variables、等式约束、目标函数和 bounds。"""
        _ = problem, previous_solution
        raise NotImplementedError(
            "B03-A only defines the DirectMultipleShootingNMPCSolver TODO skeleton."
        )
