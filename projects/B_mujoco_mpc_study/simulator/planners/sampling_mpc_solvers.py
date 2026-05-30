"""B03 sampling-based MPC solver skeletons.

这里聚焦 solver ladder 的前几层：
- Random Shooting / Predictive Sampling
- Warm-start Predictive Sampling
- CEM-MPC
- MPPI-lite

B03-A 只实现纯函数基础件，其余 solver 类保持函数级 TODO skeleton。
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Sequence

import numpy as np

from planners.mpc_solver_interface import BaseMPCSolver, MPCProblem, MPCSolution, SolverStats


@dataclass(frozen=True)
class CandidateBatch:
    """候选控制序列批次。"""

    controls: np.ndarray
    num_candidates: int
    horizon: int
    control_dim: int
    sampling_std: float
    torque_limit: float
    seed: int | None


@dataclass(frozen=True)
class RolloutRanking:
    """候选 rollout 的排序结果。"""

    selected_index: int
    sorted_indices: list[int]
    best_cost: float
    mean_cost: float
    min_cost: float
    max_cost: float
    cost_std: float


@dataclass(frozen=True)
class SamplingSolverConfig:
    """sampling solver 公共配置。"""

    horizon: int
    num_candidates: int
    control_dim: int
    torque_limit: float
    sampling_std: float
    seed: int | None = None


def _empty_state_prediction(problem: MPCProblem) -> np.ndarray:
    """返回用于占位的空状态预测数组。"""
    state_dim = int(np.asarray(problem.current_state, dtype=float).size)
    return np.zeros((0, state_dim), dtype=float)


def _require_rollout_cost_fn(problem: MPCProblem) -> Any:
    if problem.rollout_cost_fn is None:
        raise ValueError("problem.rollout_cost_fn is required for sampling-based B03 solvers.")
    return problem.rollout_cost_fn


def _extract_rollout_payload(problem: MPCProblem, candidate_controls: np.ndarray) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None]:
    """兼容 `rollout_cost_fn` 返回 costs 或带预测轨迹的结果对象。"""
    rollout_cost_fn = _require_rollout_cost_fn(problem)
    rollout_result = rollout_cost_fn(candidate_controls, problem)

    if isinstance(rollout_result, np.ndarray):
        costs = np.asarray(rollout_result, dtype=float)
        predicted_states = None
        predicted_ee_positions = None
    elif hasattr(rollout_result, "costs"):
        costs = np.asarray(getattr(rollout_result, "costs"), dtype=float)
        predicted_states = getattr(rollout_result, "predicted_states", None)
        predicted_ee_positions = getattr(rollout_result, "predicted_ee_positions", None)
        if predicted_states is not None:
            predicted_states = np.asarray(predicted_states, dtype=float)
        if predicted_ee_positions is not None:
            predicted_ee_positions = np.asarray(predicted_ee_positions, dtype=float)
    else:
        raise TypeError(
            "problem.rollout_cost_fn must return either a cost array or an object with .costs."
        )

    if costs.shape != (candidate_controls.shape[0],):
        raise ValueError(
            "rollout costs shape must be (num_candidates,), "
            f"got {costs.shape} for num_candidates={candidate_controls.shape[0]}"
        )

    return costs, predicted_states, predicted_ee_positions


def _build_sampling_solution(
    *,
    problem: MPCProblem,
    candidate_controls: np.ndarray,
    solver_name: str,
    start_time: float,
    costs: np.ndarray,
    predicted_states: np.ndarray | None,
    predicted_ee_positions: np.ndarray | None,
) -> MPCSolution:
    """把 rollout ranking 与预测轨迹组装成统一 `MPCSolution`。"""
    ranking = rank_rollouts(costs)
    best_controls = candidate_controls[ranking.selected_index]
    best_states = None if predicted_states is None else predicted_states[ranking.selected_index]
    best_ee_positions = None if predicted_ee_positions is None else predicted_ee_positions[ranking.selected_index]

    return MPCSolution(
        first_control=np.asarray(best_controls[0], dtype=float),
        predicted_states=best_states if best_states is not None else _empty_state_prediction(problem),
        predicted_controls=np.asarray(best_controls, dtype=float),
        best_cost=ranking.best_cost,
        solver_name=solver_name,
        solver_stats=SolverStats(
            runtime_ms=(time.perf_counter() - start_time) * 1000.0,
            num_rollouts=int(candidate_controls.shape[0]),
            num_iterations=1,
            success=True,
            message="sampling rollout_cost_fn evaluated successfully",
        ),
        predicted_ee_positions=best_ee_positions,
        selected_index=ranking.selected_index,
        metadata={"sorted_indices": ranking.sorted_indices, "cost_std": ranking.cost_std},
    )


def _resolve_shifted_mean_sequence(
    problem: MPCProblem,
    previous_solution: MPCSolution | None,
) -> np.ndarray | None:
    """从上一拍解中提取 warm-start 均值序列。"""
    if previous_solution is None or previous_solution.predicted_controls is None:
        return None

    previous_controls = np.asarray(previous_solution.predicted_controls, dtype=float)
    if previous_controls.shape != (problem.horizon, problem.control_dim):
        return None

    return shift_control_sequence(previous_controls)


def _build_selected_solution(
    *,
    problem: MPCProblem,
    solver_name: str,
    start_time: float,
    best_controls: np.ndarray,
    best_cost: float,
    predicted_states: np.ndarray | None,
    predicted_ee_positions: np.ndarray | None,
    selected_index: int | None,
    num_rollouts: int,
    num_iterations: int,
    message: str,
    metadata: dict[str, Any] | None = None,
) -> MPCSolution:
    """把已知最优候选直接组装成 `MPCSolution`。"""
    return MPCSolution(
        first_control=np.asarray(best_controls[0], dtype=float),
        predicted_states=(
            np.asarray(predicted_states, dtype=float)
            if predicted_states is not None
            else _empty_state_prediction(problem)
        ),
        predicted_controls=np.asarray(best_controls, dtype=float),
        best_cost=float(best_cost),
        solver_name=solver_name,
        solver_stats=SolverStats(
            runtime_ms=(time.perf_counter() - start_time) * 1000.0,
            num_rollouts=int(num_rollouts),
            num_iterations=int(num_iterations),
            success=True,
            message=message,
        ),
        predicted_ee_positions=(
            None if predicted_ee_positions is None else np.asarray(predicted_ee_positions, dtype=float)
        ),
        selected_index=selected_index,
        metadata=dict(metadata or {}),
    )


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
    mean_sequence: np.ndarray | None = None,
) -> CandidateBatch:
    """采样候选控制序列。

    输入：
    - `num_candidates`：候选数。
    - `horizon`：每条序列长度。
    - `control_dim`：每步控制维度。
    - `sampling_std`：采样噪声标准差。
    - `torque_limit`：控制限幅。
    - `seed`：随机种子。
    - `mean_sequence`：可选的采样中心；若给定，则在其周围加噪声采样。

    输出：
    - `CandidateBatch.controls.shape == (num_candidates, horizon, control_dim)`。

    数学关系：
    - 若无 `mean_sequence`：`u ~ N(0, sigma^2 I)`。
    - 若有 `mean_sequence`：`u ~ mean_sequence + N(0, sigma^2 I)`。
    - 最后执行 `clip(u, -torque_limit, torque_limit)`。

    验证标准：
    - shape 正确。
    - 固定 seed 可复现。
    - 无 NaN。
    - 所有控制绝对值不超过 `torque_limit`。

    常见错误：
    - `mean_sequence` shape 与 `(horizon, control_dim)` 不匹配。
    - 忘记 clip 导致超出 actuator 安全范围。
    """
    _require_positive_int(num_candidates, "num_candidates")
    _require_positive_int(horizon, "horizon")
    _require_positive_int(control_dim, "control_dim")
    if sampling_std < 0.0:
        raise ValueError(f"sampling_std must be non-negative, got {sampling_std}")
    if torque_limit <= 0.0:
        raise ValueError(f"torque_limit must be positive, got {torque_limit}")

    if mean_sequence is None:
        mean = np.zeros((int(horizon), int(control_dim)), dtype=float)
    else:
        mean = np.asarray(mean_sequence, dtype=float)
        if mean.shape != (int(horizon), int(control_dim)):
            raise ValueError(
                "mean_sequence shape must match (horizon, control_dim), "
                f"got {mean.shape} vs {(int(horizon), int(control_dim))}"
            )

    rng = np.random.default_rng(seed)
    noise = rng.normal(
        loc=0.0,
        scale=float(sampling_std),
        size=(int(num_candidates), int(horizon), int(control_dim)),
    )
    controls = mean[None, :, :] + noise
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


def shift_control_sequence(previous_controls: np.ndarray) -> np.ndarray:
    """把上一时刻最优控制序列左移一格，作为 warm start 初值。

    输入：
    - `previous_controls.shape == (horizon, control_dim)`。

    输出：
    - `shifted[:-1] = previous_controls[1:]`
    - `shifted[-1] = previous_controls[-1]`

    验证标准：
    - 输出 shape 与输入一致。

    常见错误：
    - 最后一项错误地补零，导致 warm-start 末端不连续。
    """
    controls = np.asarray(previous_controls, dtype=float)
    if controls.ndim != 2:
        raise ValueError(f"previous_controls must be 2D, got shape={controls.shape}")
    if controls.shape[0] == 0:
        raise ValueError("previous_controls must have at least one time step")

    shifted = np.empty_like(controls)
    shifted[:-1] = controls[1:]
    shifted[-1] = controls[-1]
    return shifted


def rank_rollouts(costs: Sequence[float]) -> RolloutRanking:
    """按 cost 对候选 rollout 排名。

    输入：
    - `costs.shape == (num_candidates,)`。

    输出：
    - `selected_index = argmin(costs)`。
    - `sorted_indices = argsort(costs)`。
    - `best_cost, mean_cost, min_cost, max_cost, cost_std`。

    数学关系：
    - 选择当前控制周期 cost 最小的预测轨迹。

    验证标准：
    - 拒绝 NaN / inf。
    - `selected_index` 与 `numpy.argmin` 一致。

    常见错误：
    - 用未过滤的 NaN / inf 做排序，导致排名不可信。
    """
    cost_array = np.asarray(costs, dtype=float)
    if cost_array.ndim != 1 or cost_array.size == 0:
        raise ValueError("costs must be a non-empty 1D sequence")
    if np.isnan(cost_array).any():
        raise ValueError("costs must not contain NaN")
    if not np.isfinite(cost_array).all():
        raise ValueError("costs must not contain inf")

    sorted_indices = np.argsort(cost_array).astype(int).tolist()
    selected_index = int(sorted_indices[0])
    return RolloutRanking(
        selected_index=selected_index,
        sorted_indices=sorted_indices,
        best_cost=float(cost_array[selected_index]),
        mean_cost=float(np.mean(cost_array)),
        min_cost=float(np.min(cost_array)),
        max_cost=float(np.max(cost_array)),
        cost_std=float(np.std(cost_array)),
    )


def _compute_elite_count(num_candidates: int, elite_ratio: float) -> int:
    """根据比例计算 elite 个数，并保证至少保留 1 条样本。"""
    if not (0.0 < elite_ratio <= 1.0):
        raise ValueError(f"elite_ratio must be in (0, 1], got {elite_ratio}")
    return max(1, int(round(num_candidates * elite_ratio)))


def _update_cem_distribution(
    *,
    elite_controls: np.ndarray,
    mean_sequence: np.ndarray,
    std: float,
    smoothing_alpha: float,
    min_std: float,
    max_std: float | None,
) -> tuple[np.ndarray, float]:
    """根据 elite 样本更新 CEM 的均值和标量标准差。"""
    if not (0.0 <= smoothing_alpha < 1.0):
        raise ValueError(f"smoothing_alpha must be in [0, 1), got {smoothing_alpha}")
    if min_std <= 0.0:
        raise ValueError(f"min_std must be positive, got {min_std}")
    if max_std is not None and max_std < min_std:
        raise ValueError(f"max_std must be >= min_std, got {max_std} < {min_std}")

    elite_mean = np.mean(elite_controls, axis=0)
    centered = elite_controls - elite_mean[None, :, :]
    elite_std = float(np.sqrt(np.mean(centered**2)))

    updated_mean = smoothing_alpha * mean_sequence + (1.0 - smoothing_alpha) * elite_mean
    updated_std = smoothing_alpha * float(std) + (1.0 - smoothing_alpha) * elite_std
    updated_std = max(float(min_std), updated_std)
    if max_std is not None:
        updated_std = min(float(max_std), updated_std)

    return updated_mean, float(updated_std)


def _compute_mppi_weights(costs: np.ndarray, temperature: float) -> tuple[np.ndarray, dict[str, float]]:
    """根据 MPPI 温度参数把 rollout cost 转成归一化权重。

    返回 (weights, diagnostics)。diagnostics 包含 weight_entropy、min_weight、max_weight。
    """
    if temperature <= 0.0:
        raise ValueError(f"temperature must be positive, got {temperature}")

    shifted_costs = np.asarray(costs, dtype=float) - float(np.min(costs))
    logits = -shifted_costs / float(temperature)
    max_logit = float(np.max(logits))
    stable_logits = logits - max_logit
    weights = np.exp(stable_logits)
    weight_sum = float(np.sum(weights))
    if not np.isfinite(weight_sum) or weight_sum <= 0.0:
        raise ValueError("MPPI weights are numerically invalid; check temperature and rollout costs.")
    weights = weights / weight_sum

    entropy = -float(np.sum(weights * np.log(weights + 1e-30)))
    diagnostics = {
        "weight_entropy": entropy,
        "min_weight": float(np.min(weights)),
        "max_weight": float(np.max(weights)),
    }
    return weights, diagnostics


def _update_mppi_distribution(
    *,
    candidate_controls: np.ndarray,
    mean_sequence: np.ndarray,
    weights: np.ndarray,
    std: float,
    smoothing_alpha: float,
    min_std: float,
    max_std: float | None,
) -> tuple[np.ndarray, float]:
    """根据 MPPI 权重更新均值和标量噪声标准差。"""
    if not (0.0 <= smoothing_alpha < 1.0):
        raise ValueError(f"smoothing_alpha must be in [0, 1), got {smoothing_alpha}")
    if min_std <= 0.0:
        raise ValueError(f"min_std must be positive, got {min_std}")
    if max_std is not None and max_std < min_std:
        raise ValueError(f"max_std must be >= min_std, got {max_std} < {min_std}")

    noises = np.asarray(candidate_controls, dtype=float) - mean_sequence[None, :, :]
    weighted_delta = np.tensordot(weights, noises, axes=(0, 0))
    proposal_mean = mean_sequence + weighted_delta

    centered_noises = noises - weighted_delta[None, :, :]
    weighted_variance = np.tensordot(weights, centered_noises**2, axes=(0, 0))
    proposal_std = float(np.sqrt(np.mean(weighted_variance)))

    updated_mean = smoothing_alpha * mean_sequence + (1.0 - smoothing_alpha) * proposal_mean
    updated_std = smoothing_alpha * float(std) + (1.0 - smoothing_alpha) * proposal_std
    updated_std = max(float(min_std), updated_std)
    if max_std is not None:
        updated_std = min(float(max_std), updated_std)

    return updated_mean, float(updated_std)


class RandomShootingSolver(BaseMPCSolver):
    """Random Shooting / Predictive Sampling solver skeleton."""

    solver_name = "random_shooting"

    def solve(
        self,
        problem: MPCProblem,
        previous_solution: MPCSolution | None = None,
    ) -> MPCSolution:
        """执行最基础的 sampling MPC。

        输入：
        - 当前状态、目标 horizon、sampling 配置。

        输出：
        - 采样 candidate controls 后返回最佳 `first_control` 和预测轨迹。

        数学关系：
        - sample -> rollout -> evaluate -> rank -> execute first control

        验证标准：
        - `first_control` 等于最佳控制序列第 0 项。

        常见错误：
        - rollout 污染真实环境状态。
        - 用当前状态误差代替完整 horizon cost。
        """
        _ = previous_solution
        start_time = time.perf_counter()
        num_candidates = int(problem.solver_config["num_candidates"])
        candidate_batch = sample_candidate_controls(
            num_candidates=num_candidates,
            horizon=problem.horizon,
            control_dim=problem.control_dim,
            sampling_std=float(problem.solver_config["sampling_std"]),
            torque_limit=float(problem.solver_config["torque_limit"]),
            seed=problem.solver_config.get("seed"),
        )
        costs, predicted_states, predicted_ee_positions = _extract_rollout_payload(
            problem,
            candidate_batch.controls,
        )
        return _build_sampling_solution(
            problem=problem,
            candidate_controls=candidate_batch.controls,
            solver_name=self.solver_name,
            start_time=start_time,
            costs=costs,
            predicted_states=predicted_states,
            predicted_ee_positions=predicted_ee_positions,
        )


class WarmStartSamplingSolver(BaseMPCSolver):
    """Warm-start predictive sampling solver skeleton."""

    solver_name = "warm_start_sampling"

    def solve(
        self,
        problem: MPCProblem,
        previous_solution: MPCSolution | None = None,
    ) -> MPCSolution:
        """使用上一时刻解做 warm start。

        输入：
        - 当前问题和可选 `previous_solution`。

        输出：
        - 围绕 shifted mean 采样后的最优结果。

        数学关系：
        - `mean_k = shift(previous_best_controls)`
        - `u_i ~ mean_k + noise`

        验证标准：
        - 无 previous solution 时退化为普通 random shooting。

        常见错误：
        - shift 后 shape 错误。
        - 忘记对 shifted mean 周围采样而直接复用旧解。
        """
        start_time = time.perf_counter()
        mean_sequence = _resolve_shifted_mean_sequence(problem, previous_solution)

        num_candidates = int(problem.solver_config["num_candidates"])
        candidate_batch = sample_candidate_controls(
            num_candidates=num_candidates,
            horizon=problem.horizon,
            control_dim=problem.control_dim,
            sampling_std=float(problem.solver_config["sampling_std"]),
            torque_limit=float(problem.solver_config["torque_limit"]),
            seed=problem.solver_config.get("seed"),
            mean_sequence=mean_sequence,
        )
        costs, predicted_states, predicted_ee_positions = _extract_rollout_payload(
            problem,
            candidate_batch.controls,
        )
        return _build_sampling_solution(
            problem=problem,
            candidate_controls=candidate_batch.controls,
            solver_name=self.solver_name,
            start_time=start_time,
            costs=costs,
            predicted_states=predicted_states,
            predicted_ee_positions=predicted_ee_positions,
        )


class CEMShootingSolver(BaseMPCSolver):
    """CEM-MPC solver with warm start, smoothing, and global-best tracking."""

    solver_name = "cem"

    def solve(
        self,
        problem: MPCProblem,
        previous_solution: MPCSolution | None = None,
    ) -> MPCSolution:
        """Run a B03-oriented CEM-R2 solver."""
        start_time = time.perf_counter()

        num_candidates = int(problem.solver_config["num_candidates"])
        horizon = int(problem.horizon)
        control_dim = int(problem.control_dim)
        torque_limit = float(problem.solver_config["torque_limit"])
        num_iterations = int(problem.solver_config.get("num_iterations", 3))
        elite_ratio = float(problem.solver_config.get("elite_ratio", 0.1))
        elite_count = _compute_elite_count(num_candidates, elite_ratio)
        smoothing_alpha = float(problem.solver_config.get("smoothing_alpha", 0.0))
        min_std = float(problem.solver_config.get("min_std", 0.1))
        max_std_raw = problem.solver_config.get("max_std")
        max_std = None if max_std_raw is None else float(max_std_raw)
        std = float(problem.solver_config.get("initial_std", problem.solver_config.get("sampling_std", 1.0)))
        improvement_tol = float(problem.solver_config.get("improvement_tolerance", 0.0))
        patience = int(problem.solver_config.get("early_stop_patience", 0))
        seed = problem.solver_config.get("seed")

        # Reuse the previous best sequence when available; otherwise start from zeros.
        mean_sequence = _resolve_shifted_mean_sequence(problem, previous_solution)
        if mean_sequence is None:
            mean_sequence = np.zeros((horizon, control_dim), dtype=float)

        iteration_best_costs: list[float] = []
        best_controls_global: np.ndarray | None = None
        best_states_global: np.ndarray | None = None
        best_ee_positions_global: np.ndarray | None = None
        best_cost_global = float("inf")
        best_index_global: int | None = None
        no_improvement_count = 0
        actual_iterations = 0

        for iteration in range(num_iterations):
            iteration_seed = None if seed is None else int(seed) + iteration
            candidate_batch = sample_candidate_controls(
                num_candidates=num_candidates,
                horizon=horizon,
                control_dim=control_dim,
                sampling_std=std,
                torque_limit=torque_limit,
                seed=iteration_seed,
                mean_sequence=mean_sequence,
            )
            costs, predicted_states, predicted_ee_positions = _extract_rollout_payload(
                problem,
                candidate_batch.controls,
            )
            ranking = rank_rollouts(costs)
            actual_iterations += 1
            iteration_best_costs.append(ranking.best_cost)

            elite_indices = ranking.sorted_indices[:elite_count]
            elite_controls = candidate_batch.controls[elite_indices]
            mean_sequence, std = _update_cem_distribution(
                elite_controls=elite_controls,
                mean_sequence=mean_sequence,
                std=std,
                smoothing_alpha=smoothing_alpha,
                min_std=min_std,
                max_std=max_std,
            )

            if ranking.best_cost + improvement_tol < best_cost_global:
                best_cost_global = ranking.best_cost
                best_index_global = ranking.selected_index
                best_controls_global = np.asarray(candidate_batch.controls[ranking.selected_index], dtype=float)
                best_states_global = (
                    None if predicted_states is None else np.asarray(predicted_states[ranking.selected_index], dtype=float)
                )
                best_ee_positions_global = (
                    None
                    if predicted_ee_positions is None
                    else np.asarray(predicted_ee_positions[ranking.selected_index], dtype=float)
                )
                no_improvement_count = 0
            else:
                no_improvement_count += 1

            if patience > 0 and no_improvement_count >= patience:
                break

        if best_controls_global is None:
            raise RuntimeError("CEM failed to select a global best control sequence.")

        return _build_selected_solution(
            problem=problem,
            solver_name=self.solver_name,
            start_time=start_time,
            best_controls=best_controls_global,
            best_cost=best_cost_global,
            predicted_states=best_states_global,
            predicted_ee_positions=best_ee_positions_global,
            selected_index=best_index_global,
            num_rollouts=num_candidates * actual_iterations,
            num_iterations=actual_iterations,
            message="CEM rollout_cost_fn evaluated successfully",
            metadata={
                "elite_ratio": elite_ratio,
                "elite_count": elite_count,
                "iteration_best_costs": iteration_best_costs,
                "final_sampling_std": std,
                "final_mean_sequence": mean_sequence,
                "smoothing_alpha": smoothing_alpha,
                "early_stop_patience": patience,
                "improvement_tolerance": improvement_tol,
            },
        )


class MPPILiteSolver(BaseMPCSolver):
    """MPPI-lite solver with warm start, soft weighting, and global-best tracking."""

    solver_name = "mppi_lite"

    def solve(
        self,
        problem: MPCProblem,
        previous_solution: MPCSolution | None = None,
    ) -> MPCSolution:
        """Run a B03-oriented MPPI-R2 solver."""
        start_time = time.perf_counter()

        num_candidates = int(problem.solver_config["num_candidates"])
        horizon = int(problem.horizon)
        control_dim = int(problem.control_dim)
        torque_limit = float(problem.solver_config["torque_limit"])
        num_iterations = int(problem.solver_config.get("num_iterations", 3))
        temperature = float(problem.solver_config.get("temperature", 1.0))
        smoothing_alpha = float(problem.solver_config.get("smoothing_alpha", 0.0))
        min_std = float(problem.solver_config.get("min_std", 0.1))
        max_std_raw = problem.solver_config.get("max_std")
        max_std = None if max_std_raw is None else float(max_std_raw)
        std = float(
            problem.solver_config.get(
                "noise_std",
                problem.solver_config.get("sampling_std", 1.0),
            )
        )
        improvement_tol = float(problem.solver_config.get("improvement_tolerance", 0.0))
        patience = int(problem.solver_config.get("early_stop_patience", 0))
        seed = problem.solver_config.get("seed")

        # Reuse the previous best sequence when available; otherwise start from zeros.
        mean_sequence = _resolve_shifted_mean_sequence(problem, previous_solution)
        if mean_sequence is None:
            mean_sequence = np.zeros((horizon, control_dim), dtype=float)

        iteration_best_costs: list[float] = []
        best_controls_global: np.ndarray | None = None
        best_states_global: np.ndarray | None = None
        best_ee_positions_global: np.ndarray | None = None
        best_cost_global = float("inf")
        best_index_global: int | None = None
        no_improvement_count = 0
        actual_iterations = 0

        weight_diagnostics: dict[str, float] = {}

        for iteration in range(num_iterations):
            iteration_seed = None if seed is None else int(seed) + iteration
            candidate_batch = sample_candidate_controls(
                num_candidates=num_candidates,
                horizon=horizon,
                control_dim=control_dim,
                sampling_std=std,
                torque_limit=torque_limit,
                seed=iteration_seed,
                mean_sequence=mean_sequence,
            )
            costs, predicted_states, predicted_ee_positions = _extract_rollout_payload(
                problem,
                candidate_batch.controls,
            )
            ranking = rank_rollouts(costs)
            weights, weight_diagnostics = _compute_mppi_weights(costs, temperature)
            actual_iterations += 1
            iteration_best_costs.append(ranking.best_cost)

            mean_sequence, std = _update_mppi_distribution(
                candidate_controls=candidate_batch.controls,
                mean_sequence=mean_sequence,
                weights=weights,
                std=std,
                smoothing_alpha=smoothing_alpha,
                min_std=min_std,
                max_std=max_std,
            )

            if ranking.best_cost + improvement_tol < best_cost_global:
                best_cost_global = ranking.best_cost
                best_index_global = ranking.selected_index
                best_controls_global = np.asarray(candidate_batch.controls[ranking.selected_index], dtype=float)
                best_states_global = (
                    None if predicted_states is None else np.asarray(predicted_states[ranking.selected_index], dtype=float)
                )
                best_ee_positions_global = (
                    None
                    if predicted_ee_positions is None
                    else np.asarray(predicted_ee_positions[ranking.selected_index], dtype=float)
                )
                no_improvement_count = 0
            else:
                no_improvement_count += 1

            if patience > 0 and no_improvement_count >= patience:
                break

        if best_controls_global is None:
            raise RuntimeError("MPPI failed to select a global best control sequence.")

        updated_sequence = np.clip(mean_sequence, -torque_limit, torque_limit)
        first_control = np.asarray(best_controls_global[0], dtype=float)

        return MPCSolution(
            first_control=first_control,
            predicted_states=(
                np.asarray(best_states_global, dtype=float)
                if best_states_global is not None
                else _empty_state_prediction(problem)
            ),
            predicted_controls=updated_sequence,
            best_cost=float(best_cost_global),
            solver_name=self.solver_name,
            solver_stats=SolverStats(
                runtime_ms=(time.perf_counter() - start_time) * 1000.0,
                num_rollouts=num_candidates * actual_iterations,
                num_iterations=actual_iterations,
                success=True,
                message="MPPI rollout_cost_fn evaluated successfully",
            ),
            predicted_ee_positions=(
                None if best_ee_positions_global is None else np.asarray(best_ee_positions_global, dtype=float)
            ),
            selected_index=best_index_global,
            metadata={
                "temperature": temperature,
                "weight_entropy": weight_diagnostics.get("weight_entropy", float("nan")),
                "min_weight": weight_diagnostics.get("min_weight", float("nan")),
                "max_weight": weight_diagnostics.get("max_weight", float("nan")),
                "iteration_best_costs": iteration_best_costs,
                "final_noise_std": std,
                "final_mean_sequence": mean_sequence,
                "smoothing_alpha": smoothing_alpha,
                "early_stop_patience": patience,
                "improvement_tolerance": improvement_tol,
            },
        )
