"""B03 predictive sampling planner learning skeleton.

B03 的学习目标：
- 把 B02 已经能运行的 task-space MPC 拆成更透明的 predictive sampling 数据流。
- 先实现不依赖 MuJoCo 的纯函数：采样候选控制序列、按 cost 排名。
- 将真正的 MuJoCo rollout、B02 residual cost 和 receding horizon 闭环保留为 TODO。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np


@dataclass(frozen=True)
class CandidateBatch:
    """一批候选控制序列。

    `controls` 的 shape 固定为：
    `(num_candidates, horizon, control_dim)`。
    """

    controls: np.ndarray
    num_candidates: int
    horizon: int
    control_dim: int
    sampling_std: float
    torque_limit: float
    seed: int | None


@dataclass(frozen=True)
class RolloutResult:
    """单条候选序列 rollout 后的结果容器。

    B03-A 暂不填充真实 MuJoCo rollout，本 dataclass 先固定接口：
    - states：未来 horizon 内的状态序列
    - ee_positions：未来 horizon 内的末端位置序列
    - controls：本条 rollout 使用的控制序列
    - cost：该 rollout 的 horizon cost，后续由 `evaluate_rollout_cost` 计算
    """

    states: np.ndarray
    ee_positions: np.ndarray
    controls: np.ndarray
    cost: float | None = None


@dataclass(frozen=True)
class PredictiveSamplingPlan:
    """一次 predictive sampling 排名后的摘要。"""

    selected_index: int
    sorted_indices: list[int]
    best_cost: float
    mean_cost: float
    min_cost: float
    max_cost: float


def _require_positive_int(value: int, name: str) -> None:
    if int(value) != value or value <= 0:
        raise ValueError(f"{name} must be a positive integer, got {value}")


def sample_candidate_controls(
    num_candidates: int,
    horizon: int,
    control_dim: int,
    sampling_std: float,
    torque_limit: float,
    seed: int | None = None,
) -> CandidateBatch:
    """采样候选控制序列。

    要实现什么：
    - 生成 `num_candidates` 条控制序列。
    - 每条序列长度为 `horizon`。
    - 每个控制向量维度为 `control_dim`。

    为什么需要：
    - predictive sampling 不直接求解析最优解，而是先提出多种未来控制方案，
      再用 rollout + cost 对它们排序。

    输入：
    - `num_candidates`：候选序列数量。
    - `horizon`：每条序列向未来看的步数。
    - `control_dim`：控制维度；B02 二连杆中通常为 2。
    - `sampling_std`：正态采样标准差。
    - `torque_limit`：控制上限，采样后会 clip 到安全范围。
    - `seed`：随机种子，用于复现实验。

    输出：
    - `CandidateBatch.controls`，shape 为
      `(num_candidates, horizon, control_dim)`。

    验证标准：
    - 输出无 NaN。
    - 所有控制满足 `abs(u) <= torque_limit`。
    - 同一 seed 下结果可复现。
    """

    _require_positive_int(num_candidates, "num_candidates")
    _require_positive_int(horizon, "horizon")
    _require_positive_int(control_dim, "control_dim")
    if sampling_std < 0.0:
        raise ValueError(f"sampling_std must be non-negative, got {sampling_std}")
    if torque_limit <= 0.0:
        raise ValueError(f"torque_limit must be positive, got {torque_limit}")

    rng = np.random.default_rng(seed)
    controls = rng.normal(
        loc=0.0,
        scale=float(sampling_std),
        size=(int(num_candidates), int(horizon), int(control_dim)),
    )
    controls = np.clip(controls, -float(torque_limit), float(torque_limit))

    if np.isnan(controls).any():
        raise ValueError("sampled controls contain NaN")

    return CandidateBatch(
        controls=controls.astype(float, copy=False),
        num_candidates=int(num_candidates),
        horizon=int(horizon),
        control_dim=int(control_dim),
        sampling_std=float(sampling_std),
        torque_limit=float(torque_limit),
        seed=seed,
    )


def rank_rollouts(rollout_costs: Sequence[float]) -> PredictiveSamplingPlan:
    """根据 horizon cost 对候选 rollout 排名。

    要实现什么：
    - 接收每条候选 rollout 的标量 cost。
    - 找到 cost 最小的候选。
    - 返回排序索引和基本统计量。

    为什么需要：
    - MPC 当前时刻只执行最优序列的第一项，因此必须知道哪条序列的
      horizon cost 最低。

    输入：
    - `rollout_costs[i]`：第 i 条候选 rollout 的累计代价。

    输出：
    - `selected_index = argmin(costs)`。
    - `sorted_indices`：按 cost 从小到大排列的候选编号。
    - `best_cost / mean_cost / min_cost / max_cost`。

    验证标准：
    - NaN cost 会被拒绝。
    - `selected_index` 与 `numpy.argmin` 一致。
    """

    costs = np.asarray(rollout_costs, dtype=float)
    if costs.ndim != 1:
        raise ValueError(f"rollout_costs must be a 1D sequence, got shape={costs.shape}")
    if costs.size == 0:
        raise ValueError("rollout_costs must not be empty")
    if np.isnan(costs).any():
        raise ValueError("rollout_costs must not contain NaN")

    sorted_indices = np.argsort(costs).astype(int).tolist()
    selected_index = int(sorted_indices[0])
    best_cost = float(costs[selected_index])

    return PredictiveSamplingPlan(
        selected_index=selected_index,
        sorted_indices=sorted_indices,
        best_cost=best_cost,
        mean_cost=float(np.mean(costs)),
        min_cost=float(np.min(costs)),
        max_cost=float(np.max(costs)),
    )


def rollout_candidate_sequence(
    env: Any,
    current_state: Any,
    control_sequence: np.ndarray,
) -> RolloutResult:
    """TODO: 对单条候选控制序列做 MuJoCo horizon rollout。

    TODO:
    - 要补什么：复制 MuJoCo `data`，从 `current_state` 出发依次施加
      `control_sequence[k]`，记录未来状态和末端位置。
    - 为什么需要：predictive sampling 的核心是“先预测未来，再比较未来”。
    - 推荐 API：未来可参考 B02 env 的 `get_state`、`set_state`、`step`、
      `rollout`、`get_end_effector_position`，必要时用 MuJoCo data copy 避免污染真实环境。
    - 输入是什么：环境、当前状态、一条 shape 为 `(horizon, control_dim)` 的控制序列。
    - 输出是什么：`RolloutResult(states, ee_positions, controls)`。
    - 验证标准：rollout 前后真实环境状态不被改变；输出长度与 horizon 对齐。
    """
    _ = env, current_state, control_sequence
    raise NotImplementedError("B03-A only defines the rollout_candidate_sequence TODO skeleton.")


def evaluate_rollout_cost(
    rollout: RolloutResult,
    target_sequence: np.ndarray,
    ee_weight: float,
    dq_weight: float,
    torque_weight: float,
    terminal_weight: float,
) -> float:
    """TODO: 使用 B02 task-space residual 评价一条 rollout。

    TODO:
    - 要补什么：对 horizon 内每一步累计末端误差、速度、力矩和 terminal error。
    - 为什么需要：cost 是候选 rollout 排名的唯一标准，必须与 B02 task-space tracking
      的 residual 思路保持一致。
    - 推荐公式：
      `cost = ee_weight * ||p_ee - p_target||^2
            + dq_weight * ||dq||^2
            + torque_weight * ||u||^2
            + terminal_weight * ||p_ee_T - p_target_T||^2`
    - 输入是什么：一条 rollout、目标末端轨迹、各 cost 权重。
    - 输出是什么：标量 cost，越小越好。
    - 验证标准：目标误差或控制量变大时 cost 应单调增大。
    """
    _ = rollout, target_sequence, ee_weight, dq_weight, torque_weight, terminal_weight
    raise NotImplementedError("B03-A only defines the evaluate_rollout_cost TODO skeleton.")


def plan_once(
    env: Any,
    current_state: Any,
    target_sequence: np.ndarray,
    num_candidates: int,
    horizon: int,
    control_dim: int,
    sampling_std: float,
    torque_limit: float,
    seed: int | None = None,
) -> dict[str, Any]:
    """TODO: 执行一次 predictive sampling planning step。

    TODO:
    - 要补什么：`sample -> rollout -> evaluate -> rank -> return first control`。
    - 为什么需要：这是 receding horizon control 的一个控制周期。
    - 输入是什么：当前环境状态、目标 horizon、采样参数和力矩限制。
    - 输出是什么：best sequence 的第一项控制、所有候选 cost、best rollout 等。
    - 验证标准：返回的 first control 必须等于 best sequence 的第 0 项。
    """
    _ = (
        env,
        current_state,
        target_sequence,
        num_candidates,
        horizon,
        control_dim,
        sampling_std,
        torque_limit,
        seed,
    )
    raise NotImplementedError("B03-A only defines the plan_once TODO skeleton.")


def plan_receding_horizon(
    env: Any,
    initial_state: Any,
    target_generator: Any,
    num_steps: int,
) -> dict[str, Any]:
    """TODO: 重复执行 receding horizon planning。

    TODO:
    - 要补什么：每一步重新调用 `plan_once`，但只执行 best sequence 的第一项控制。
    - 为什么需要：MPC 不一次性执行整条未来计划，而是每一步根据新状态重新规划，
      这样能应对模型误差和外界扰动。
    - 输入是什么：环境、初始状态、目标生成器、闭环步数。
    - 输出是什么：actual trajectory、selected rollout log、candidate cost log。
    - 验证标准：控制循环步数与 `num_steps` 一致，日志每步都有 selected index 和 cost。
    """
    _ = env, initial_state, target_generator, num_steps
    raise NotImplementedError("B03-A only defines the plan_receding_horizon TODO skeleton.")
