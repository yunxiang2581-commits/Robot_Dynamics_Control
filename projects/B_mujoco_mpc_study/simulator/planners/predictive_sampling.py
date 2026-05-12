"""Predictive sampling planner skeleton for B01.

本文件只保留 MPC planner 的教学型 TODO，不实现完整采样和代价计算。
"""

from __future__ import annotations

from typing import Any
import numpy as np


class PredictiveSamplingPlanner:
    """Predictive Sampling 规划器骨架。

    核心思想：采样多条 torque 序列，用环境 rollout 预测未来，计算 horizon cost，
    选择 cost 最小的序列。
    """

    def __init__(
        self,
        horizon: int,
        num_candidates: int,
        torque_limit: float,
        q_weight: float = 1.0,
        dq_weight: float = 0.1,
        torque_weight: float = 0.001,
        terminal_weight: float = 5.0,
    ) -> None:
        """初始化 planner 参数。

        TODO:
        - 要实现什么：保存 horizon、候选数量、力矩限制和 cost 权重。
        - 为什么要实现：这些参数决定 MPC 看多远、采样多少控制序列、如何评价好坏。
        - 输入是什么：horizon、num_candidates、torque_limit 和各项 cost 权重。
        - 输出是什么：初始化后的 planner 对象。
        - 物理意义：限制力矩大小，避免不真实控制。
        - 数学意义：定义优化问题的搜索空间和目标函数权重。
        - 验证标准：参数应为正数，horizon 和候选数量应为正整数。
        """

        if horizon <= 0:
            raise ValueError(f"horizon 必须是正整数，当前 horizon={horizon}")

        if num_candidates <= 0:
            raise ValueError(f"num_candidates 必须是正整数，当前 num_candidates={num_candidates}")

        if torque_limit <= 0.0:
            raise ValueError(f"torque_limit 必须为正数，当前 torque_limit={torque_limit}")

        if q_weight < 0.0:
            raise ValueError(f"q_weight 不能为负数，当前 q_weight={q_weight}")

        if dq_weight < 0.0:
            raise ValueError(f"dq_weight 不能为负数，当前 dq_weight={dq_weight}")

        if torque_weight < 0.0:
            raise ValueError(f"torque_weight 不能为负数，当前 torque_weight={torque_weight}")

        if terminal_weight < 0.0:
            raise ValueError(f"terminal_weight 不能为负数，当前 terminal_weight={terminal_weight}")

        self.horizon = horizon
        self.num_candidates = num_candidates
        self.torque_limit = torque_limit
        self.q_weight = q_weight
        self.dq_weight = dq_weight
        self.torque_weight = torque_weight
        self.terminal_weight = terminal_weight

    def sample_torque_sequences(self) -> list[list[float]]:
        """采样候选 torque 序列。

        TODO:
        - 要实现什么：生成 `num_candidates` 条长度为 `horizon` 的 torque 序列。
        - 为什么要实现：predictive sampling 需要多个候选未来控制方案进行比较。
        - 输入是什么：使用 planner 内部的 horizon、num_candidates、torque_limit。
        - 输出是什么：二维列表，每一行是一条候选 torque sequence。
        - 物理意义：每条序列表示未来一段时间可能施加的力矩。
        - 数学意义：在控制空间 `u_0:H-1` 中采样候选点。
        - 验证标准：输出形状为 `[num_candidates, horizon]`，每个 torque 都在限制范围内。
        """
        # 采样候选 torque 序列。

        samples = np.random.uniform(
            low=-self.torque_limit,
            high=self.torque_limit,
            size=(self.num_candidates, self.horizon),
        )

        return samples.tolist()

    def _target_at_step(self, q_target: float | list[float], step_index: int) -> float:
        """返回 horizon 中某一步对应的目标角。

        教学说明：
        - 要实现什么：同时支持常数目标 `q_target` 和目标序列 `q_target[k]`。
        - 为什么要实现：smooth/ramp target 下，未来每一步的参考角不同，cost 必须按时间对齐。
        - 输入是什么：目标角标量或目标角序列，以及 horizon 内的步号。
        - 输出是什么：当前步用于计算 residual 的参考角。
        - 物理意义：关节不是瞬间跳到最终目标，而是跟随一条逐渐变化的参考轨迹。
        - 数学意义：把 `q - q*` 扩展为 `q_k - q*_k`。
        - 验证标准：传入标量时行为与旧版本一致；传入序列时每一步使用对应目标。
        """
        if isinstance(q_target, (list, tuple, np.ndarray)):
            if len(q_target) == 0:
                raise ValueError("q_target 序列不能为空。")
            return float(q_target[min(step_index, len(q_target) - 1)])
        return float(q_target)

    def compute_horizon_cost(
        self,
        states: list[tuple[float, float]],
        torque_sequence: list[float],
        q_target: float | list[float],
    ) -> float:
        """计算一条 rollout 的 horizon cost。

        TODO:
        - 要实现什么：根据状态序列、力矩序列和目标角度计算累计 cost。
        - 为什么要实现：planner 需要用同一套指标比较不同 rollout 的好坏。
        - 输入是什么：`states=[(q,dq),...]`，`torque_sequence=[tau,...]`，`q_target`。
        - 输出是什么：标量 cost，越小表示越优。
        - 物理意义：同时惩罚角度偏差、速度过大和力矩过大。
        - 数学意义：实现 `sum w_q(q-q*)^2 + w_dq dq^2 + w_tau tau^2 + terminal cost`。
        - 验证标准：目标误差越大 cost 应越大；力矩越大 torque penalty 应越大。
        """
        if len(states) != len(torque_sequence) + 1:
            raise ValueError(
                "states 长度必须等于 len(torque_sequence) + 1，"
                f"当前 len(states)={len(states)}, len(torque_sequence)={len(torque_sequence)}"
            )
    
        total_cost = 0.0

        for k, tau in enumerate(torque_sequence):
            q, dq = states[k+1]  # 注意 states 比 torque_sequence 长 1，因为 states 包含初始状态 x_0
            target_k = self._target_at_step(q_target, k)
            q_error = q - target_k

            q_cost = self.q_weight * (q_error ** 2)
            v_cost = self.dq_weight * (dq ** 2)
            torque_cost = self.torque_weight * (tau ** 2)

            total_cost += q_cost + v_cost + torque_cost

        # 终端 cost，惩罚最终状态与目标的偏差。q_terminal, _ = states[-1]
        q_terminal, _ = states[-1]
        terminal_target = self._target_at_step(q_target, len(torque_sequence) - 1)
        terminal_error = q_terminal - terminal_target
        terminal_cost = self.terminal_weight * terminal_error**2

        total_cost += terminal_cost
        return total_cost

    def select_best_sequence(
        self,
        candidate_sequences: list[list[float]],
        candidate_costs: list[float],
    ) -> tuple[list[float], float, int]:
        """选择 cost 最小的 torque 序列。

        TODO:
        - 要实现什么：从候选序列和 cost 中找出最小 cost 对应的序列。
        - 为什么要实现：MPC 需要确定当前控制周期的最佳未来控制方案。
        - 输入是什么：候选 torque 序列列表，以及对应 cost 列表。
        - 输出是什么：`best_sequence`、`best_cost`、`best_index`。
        - 物理意义：选择最可能让关节接近目标且控制平滑的力矩方案。
        - 数学意义：执行 `argmin_i J_i`。
        - 验证标准：返回的 cost 应等于 `min(candidate_costs)`。
        """
        if len(candidate_sequences) != len(candidate_costs):
            raise ValueError(
                "candidate_sequences 和 candidate_costs 长度必须相等，"
                f"当前 len(candidate_sequences)={len(candidate_sequences)}, "
                f"len(candidate_costs)={len(candidate_costs)}"
            )

        best_index = int(np.argmin(candidate_costs))
        best_sequence = candidate_sequences[best_index]
        best_cost = candidate_costs[best_index]

        return best_sequence, best_cost, best_index

    def plan(self, env: Any, current_state: tuple[float, float], q_target: float | list[float]) -> dict[str, Any]:
        """执行一次 MPC 规划。

        TODO:
        - 要实现什么：采样 torque 序列，对每条序列 rollout，计算 cost，并选出最优序列。
        - 为什么要实现：这是 B01 的核心 MPC planning step。
        - 输入是什么：环境 `env`、当前状态 `(q,dq)`、目标角 `q_target`。
        - 输出是什么：包含 best sequence、best torque、best cost、candidate costs 等信息的字典。
        - 物理意义：预测多种未来力矩方案对关节运动的影响。
        - 数学意义：近似求解有限时域最优控制问题。
        - 验证标准：`best_torque` 应等于 `best_sequence[0]`，并且所有候选都有 cost。
        """
        # 1. 采样候选 torque 序列。
        candidate_sequences = self.sample_torque_sequences()

        candidate_costs: list[float] = []
        candidate_rollouts: list[list[tuple[float, float]]] = []

        # 2. 对每条候选序列 rollout，计算 cost。
        for torque_sequence in candidate_sequences:
            states = env.rollout(current_state, torque_sequence)
            candidate_rollouts.append(states)

            cost = self.compute_horizon_cost(states, torque_sequence, q_target)
            candidate_costs.append(cost)

        # 3. 选择 cost 最小的序列。
        best_sequence, best_cost, best_index = self.select_best_sequence(candidate_sequences, candidate_costs)

        # 4.执行最优序列的第一个 torque。
        best_torque = best_sequence[0]

        # 5. 返回规划结果。
        return {
            "best_sequence": best_sequence,
            "best_torque": best_torque,
            "best_cost": best_cost,
            "best_index": best_index,
            "candidate_sequences": candidate_sequences,
            "candidate_costs": candidate_costs,
            "candidate_rollouts": candidate_rollouts,
        }
