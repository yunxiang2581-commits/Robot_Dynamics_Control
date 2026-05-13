"""B02 two-link predictive sampling planner skeleton.

本文件只进入 B02 的 task-space MPC 设计，不直接实现完整算法。
"""

from __future__ import annotations

from typing import Any

import numpy as np


class TwoLinkPredictiveSamplingPlanner:
    """二连杆 task-space predictive sampling planner 骨架。"""

    def __init__(
        self,
        horizon: int,
        num_candidates: int,
        torque_limit: float,
        ee_weight: float = 20.0,
        dq_weight: float = 0.1,
        torque_weight: float = 0.002,
        terminal_weight: float = 5.0,
    ) -> None:
        """保存 planner 参数。

        TODO:
        - 要实现什么：保存 horizon、候选数、力矩限制和 cost 权重。
        - 为什么要实现：这些参数定义有限时域搜索空间和评价函数。
        - 输入是什么：MPC horizon、采样数、torque limit、各项 cost 权重。
        - 输出是什么：初始化后的 planner 对象。
        - 物理意义：限制两个 actuator 的控制强度。
        - 数学意义：定义 `argmin J` 的搜索范围和 cost 结构。
        - 验证标准：horizon 和 num_candidates 为正，权重非负。
        """
        if horizon <= 0:
            raise ValueError(f"horizon 必须是正整数，当前 horizon={horizon}")
        if num_candidates <= 0:
            raise ValueError(f"num_candidates 必须是正整数，当前 num_candidates={num_candidates}")
        if torque_limit <= 0.0:
            raise ValueError(f"torque_limit 必须为正数，当前 torque_limit={torque_limit}")
        if ee_weight < 0.0:
            raise ValueError(f"ee_weight 不能为负数，当前 ee_weight={ee_weight}")
        if dq_weight < 0.0:
            raise ValueError(f"dq_weight 不能为负数，当前 dq_weight={dq_weight}")
        if torque_weight < 0.0:
            raise ValueError(f"torque_weight 不能为负数，当前 torque_weight={torque_weight}")
        if terminal_weight < 0.0:
            raise ValueError(f"terminal_weight 不能为负数，当前 terminal_weight={terminal_weight}")

        self.horizon = horizon
        self.num_candidates = num_candidates
        self.torque_limit = torque_limit
        self.ee_weight = ee_weight
        self.dq_weight = dq_weight
        self.torque_weight = torque_weight
        self.terminal_weight = terminal_weight

    def sample_torque_sequences(self) -> list[list[tuple[float, float]]]:
        """采样双关节 torque 序列。

        TODO:
        - 要实现什么：生成 `num_candidates` 条长度为 horizon 的 `[tau1,tau2]` 序列。
        - 为什么要实现：B02 每个候选控制方案有两个关节力矩。
        - 输入是什么：内部 horizon、num_candidates、torque_limit。
        - 输出是什么：形如 `[candidate][k]=(tau1,tau2)` 的列表。
        - 物理意义：每条序列代表未来一段时间两个电机的力矩计划。
        - 数学意义：在二维控制空间 `u=[tau1,tau2]` 中采样。
        - 验证标准：所有力矩在 `[-torque_limit, torque_limit]` 内。
        """
        samples = np.random.uniform(
            low=-self.torque_limit,
            high=self.torque_limit,
            size=(self.num_candidates, self.horizon, 2),
        )

        # 输出保持成 list[list[tuple]]，方便和 env.step((tau1, tau2)) 的接口对齐。
        return [
            [(float(tau1), float(tau2)) for tau1, tau2 in candidate]
            for candidate in samples
        ]

    def _target_at_step(self, target_sequence: list[tuple[float, float]], step_index: int) -> tuple[float, float]:
        """返回 horizon 中某一步的末端位置目标。

        如果目标序列短于 horizon，使用最后一个目标保持不变。这样 demo 可以先给
        短目标序列，后续再扩展成完整轨迹。
        """
        if len(target_sequence) == 0:
            raise ValueError("target_sequence 不能为空。")

        target = target_sequence[min(step_index, len(target_sequence) - 1)]
        if len(target) != 2:
            raise ValueError("target_sequence 中每个目标都必须是 (x_ee, y_ee)。")

        return float(target[0]), float(target[1])

    def compute_horizon_cost(
        self,
        env: Any,
        states: list[tuple[float, float, float, float]],
        torque_sequence: list[tuple[float, float]],
        target_sequence: list[tuple[float, float]],
    ) -> float:
        """计算二连杆 task-space horizon cost。

        TODO:
        - 要实现什么：累计末端位置误差、关节速度惩罚、力矩惩罚和 terminal cost。
        - 为什么要实现：planner 必须用同一套指标比较不同 rollout 的好坏。
        - 输入是什么：环境、状态序列、力矩序列、末端目标序列。
        - 输出是什么：标量 cost，越小越好。
        - 物理意义：希望末端贴近目标轨迹，同时动作不过猛。
        - 数学意义：实现 `sum ||p_ee(q_k)-p_target_k||^2 + ...`。
        - 验证标准：目标误差越大 cost 越大；力矩越大 torque penalty 越大。
        """
        if len(states) != len(torque_sequence) + 1:
            raise ValueError(
                "states 长度必须等于 len(torque_sequence) + 1，"
                f"当前 len(states)={len(states)}, len(torque_sequence)={len(torque_sequence)}"
            )

        real_state = env.get_state()
        total_cost = 0.0

        try:
            for k, torque in enumerate(torque_sequence):
                if len(torque) != 2:
                    raise ValueError("torque_sequence 中每个 torque 都必须是 (tau1, tau2)。")

                # states[0] 是 x0；第 k 个 torque 作用后对应 states[k+1]。
                q1, q2, dq1, dq2 = states[k + 1]
                env.set_state((q1, q2, dq1, dq2))
                x_ee, y_ee = env.get_end_effector_position()
                x_target, y_target = self._target_at_step(target_sequence, k)

                ee_error_x = x_ee - x_target
                ee_error_y = y_ee - y_target
                tau1, tau2 = torque

                ee_cost = self.ee_weight * (ee_error_x**2 + ee_error_y**2)
                velocity_cost = self.dq_weight * (dq1**2 + dq2**2)
                torque_cost = self.torque_weight * (tau1**2 + tau2**2)

                total_cost += ee_cost + velocity_cost + torque_cost

            q1_terminal, q2_terminal, dq1_terminal, dq2_terminal = states[-1]
            env.set_state((q1_terminal, q2_terminal, dq1_terminal, dq2_terminal))
            x_terminal, y_terminal = env.get_end_effector_position()
            x_target_terminal, y_target_terminal = self._target_at_step(
                target_sequence,
                len(torque_sequence) - 1,
            )
            terminal_error_x = x_terminal - x_target_terminal
            terminal_error_y = y_terminal - y_target_terminal
            total_cost += self.terminal_weight * (terminal_error_x**2 + terminal_error_y**2)
        finally:
            # cost 计算只是评价 rollout，不应污染真实闭环环境状态。
            env.set_state(real_state)

        return total_cost

    def select_best_sequence(
        self,
        candidate_sequences: list[list[tuple[float, float]]],
        candidate_costs: list[float],
    ) -> tuple[list[tuple[float, float]], float, int]:
        """选择 cost 最小的 torque 序列。

        TODO:
        - 要实现什么：从所有候选序列中找到 cost 最小者。
        - 为什么要实现：MPC 当前控制周期需要选出最优未来计划。
        - 输入是什么：候选序列列表和对应 cost 列表。
        - 输出是什么：best_sequence、best_cost、best_index。
        - 物理意义：选择最能让末端接近目标且代价较小的动作。
        - 数学意义：执行 `argmin_i J_i`。
        - 验证标准：返回 cost 等于 `min(candidate_costs)`。
        """
        if len(candidate_sequences) != len(candidate_costs):
            raise ValueError(
                "candidate_sequences 和 candidate_costs 长度必须相等，"
                f"当前 len(candidate_sequences)={len(candidate_sequences)}, "
                f"len(candidate_costs)={len(candidate_costs)}"
            )
        if len(candidate_sequences) == 0:
            raise ValueError("candidate_sequences 不能为空。")

        best_index = int(np.argmin(candidate_costs))
        best_sequence = candidate_sequences[best_index]
        best_cost = float(candidate_costs[best_index])

        return best_sequence, best_cost, best_index

    def plan(
        self,
        env: Any,
        current_state: tuple[float, float, float, float],
        target_sequence: list[tuple[float, float]],
    ) -> dict[str, Any]:
        """执行一次二连杆 MPC 规划。

        TODO:
        - 要实现什么：采样 torque 序列、rollout、计算 cost、选出 best sequence。
        - 为什么要实现：这是 B02 的核心 task-space MPC planning step。
        - 输入是什么：环境、当前状态、horizon 内目标末端轨迹。
        - 输出是什么：包含 best torque、best cost、候选 cost 等信息的字典。
        - 物理意义：预测多种双关节动作对末端轨迹的影响。
        - 数学意义：近似求解 task-space 有限时域最优控制问题。
        - 验证标准：`best_torque` 应等于 `best_sequence[0]`。
        """
        candidate_sequences = self.sample_torque_sequences()

        candidate_costs: list[float] = []
        candidate_rollouts: list[list[tuple[float, float, float, float]]] = []

        for torque_sequence in candidate_sequences:
            states = env.rollout(current_state, torque_sequence)
            candidate_rollouts.append(states)

            cost = self.compute_horizon_cost(env, states, torque_sequence, target_sequence)
            candidate_costs.append(cost)

        best_sequence, best_cost, best_index = self.select_best_sequence(
            candidate_sequences,
            candidate_costs,
        )
        best_torque = best_sequence[0]

        return {
            "best_sequence": best_sequence,
            "best_torque": best_torque,
            "best_cost": best_cost,
            "best_index": best_index,
            "candidate_sequences": candidate_sequences,
            "candidate_costs": candidate_costs,
            "candidate_rollouts": candidate_rollouts,
        }
