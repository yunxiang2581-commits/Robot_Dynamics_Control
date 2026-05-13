"""B02-to-B03 adapter.

本模块负责把 B02 的二连杆 tracking 任务包装成 B03 solver 可消费的接口：
- B02 提供任务和仿真事实；
- B03 提供 solver 家族；
- adapter 负责把当前状态、目标 horizon、rollout cost 和 baseline controller 接起来。
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Callable

import numpy as np

from planners.mpc_solver_interface import BaseMPCSolver, MPCProblem, MPCSolution, SolverStats


@dataclass(frozen=True)
class B02TrackingSnapshot:
    time: float
    q: np.ndarray
    qvel: np.ndarray
    state: np.ndarray
    target_xy: np.ndarray
    actual_xy: np.ndarray
    error_norm: float


@dataclass(frozen=True)
class B02RolloutCostResult:
    costs: np.ndarray
    predicted_states: np.ndarray | None
    predicted_ee_positions: np.ndarray | None
    candidate_controls: np.ndarray


@dataclass(frozen=True)
class B02AdapterConfig:
    horizon: int
    dt: float
    control_dim: int
    torque_limit: float
    ee_weight: float
    dq_weight: float
    torque_weight: float
    terminal_weight: float
    record_predicted_states: bool
    record_predicted_ee_positions: bool


def _capture_env_state(env: Any) -> dict[str, Any] | None:
    """保存真实 MuJoCo 状态，保证 rollout 不污染闭环环境。"""
    data = getattr(env, "data", None)
    if data is None:
        return None

    return {
        "qpos": np.array(data.qpos, copy=True),
        "qvel": np.array(data.qvel, copy=True),
        "ctrl": np.array(data.ctrl, copy=True),
        "time": float(getattr(data, "time", 0.0)),
        "last_applied_torque": getattr(env, "last_applied_torque", None),
    }


def _restore_env_state(env: Any, saved_state: dict[str, Any] | None) -> None:
    """恢复真实 MuJoCo 状态。"""
    if saved_state is None:
        return

    data = getattr(env, "data", None)
    model = getattr(env, "model", None)
    if data is None:
        return

    data.qpos[:] = saved_state["qpos"]
    data.qvel[:] = saved_state["qvel"]
    data.ctrl[:] = saved_state["ctrl"]
    if hasattr(data, "time"):
        data.time = saved_state["time"]
    if saved_state["last_applied_torque"] is not None:
        env.last_applied_torque = saved_state["last_applied_torque"]

    if model is not None:
        try:
            import mujoco

            mujoco.mj_forward(model, data)
        except Exception:
            pass


def _resolve_site_id(model: Any, site_name: str | None, ee_site_id: int | None) -> int:
    if ee_site_id is not None:
        return int(ee_site_id)
    if site_name is None:
        raise ValueError("Either site_name or ee_site_id must be provided.")

    try:
        import mujoco
    except ImportError as exc:
        raise RuntimeError("extract_b02_tracking_snapshot needs mujoco to resolve site_name.") from exc

    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    if site_id < 0:
        available_names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i)
            for i in range(getattr(model, "nsite", 0))
        ]
        raise ValueError(f"Cannot find site {site_name!r}. Available sites: {available_names}")
    return int(site_id)


def _compute_ee_positions_for_states(env: Any, states: np.ndarray) -> np.ndarray:
    """从状态序列恢复每个时刻的末端位置。"""
    saved_state = _capture_env_state(env)
    ee_positions = np.zeros((states.shape[0], 2), dtype=float)
    try:
        for index, state in enumerate(states):
            env.set_state(tuple(float(value) for value in state.tolist()))
            ee_positions[index] = np.asarray(env.get_end_effector_position(), dtype=float)
    finally:
        _restore_env_state(env, saved_state)
    return ee_positions


def extract_b02_tracking_snapshot(
    model: Any,
    data: Any,
    target_xy: np.ndarray | list[float] | tuple[float, float],
    site_name: str | None = None,
    ee_site_id: int | None = None,
    time_value: float | None = None,
) -> B02TrackingSnapshot:
    """从 B02 当前 model/data 提取 tracking snapshot。

    这个函数只读取当前事实，不修改 MuJoCo 状态。
    """
    target_xy_array = np.asarray(target_xy, dtype=float)
    if target_xy_array.shape != (2,):
        raise ValueError(f"target_xy must have shape (2,), got {target_xy_array.shape}")

    site_id = _resolve_site_id(model=model, site_name=site_name, ee_site_id=ee_site_id)
    if site_id >= getattr(model, "nsite", 0):
        raise ValueError(f"site id out of range: {site_id}")

    q = np.asarray(data.qpos[:2], dtype=float).copy()
    qvel = np.asarray(data.qvel[:2], dtype=float).copy()
    if q.shape != (2,) or qvel.shape != (2,):
        raise ValueError("B02 snapshot expects two-link q/qvel with shape (2,).")

    actual_xy = np.asarray(data.site_xpos[site_id][:2], dtype=float).copy()
    if actual_xy.shape != (2,):
        raise RuntimeError(f"Failed to read site position for site_id={site_id}.")

    state = np.concatenate([q, qvel]).astype(float, copy=False)
    snapshot_time = float(time_value if time_value is not None else getattr(data, "time", 0.0))
    error_norm = float(np.linalg.norm(actual_xy - target_xy_array))

    return B02TrackingSnapshot(
        time=snapshot_time,
        q=q,
        qvel=qvel,
        state=state,
        target_xy=target_xy_array,
        actual_xy=actual_xy,
        error_norm=error_norm,
    )


def build_b03_problem_from_b02_snapshot(
    snapshot: B02TrackingSnapshot,
    target_horizon: np.ndarray,
    adapter_config: B02AdapterConfig,
    solver_config: dict[str, Any] | None = None,
    rollout_cost_fn: Callable[[np.ndarray, MPCProblem], Any] | None = None,
) -> MPCProblem:
    """把 B02 snapshot 转成 B03 可消费的 `MPCProblem`。"""
    target_horizon_array = np.asarray(target_horizon, dtype=float)
    expected_shape = (adapter_config.horizon, 2)
    if target_horizon_array.shape != expected_shape:
        raise ValueError(f"target_horizon must have shape {expected_shape}, got {target_horizon_array.shape}")

    solver_config_dict = dict(solver_config or {})
    solver_config_dict.setdefault("horizon", adapter_config.horizon)
    solver_config_dict.setdefault("control_dim", adapter_config.control_dim)
    solver_config_dict.setdefault("torque_limit", adapter_config.torque_limit)

    return MPCProblem(
        current_state=np.asarray(snapshot.state, dtype=float),
        target_horizon=target_horizon_array,
        horizon=adapter_config.horizon,
        control_dim=adapter_config.control_dim,
        dt=float(adapter_config.dt),
        cost_config={
            "ee_weight": float(adapter_config.ee_weight),
            "dq_weight": float(adapter_config.dq_weight),
            "torque_weight": float(adapter_config.torque_weight),
            "terminal_weight": float(adapter_config.terminal_weight),
        },
        solver_config=solver_config_dict,
        rollout_cost_fn=rollout_cost_fn,
        metadata={
            "snapshot_time": float(snapshot.time),
            "snapshot_error_norm": float(snapshot.error_norm),
            "snapshot_target_xy": np.asarray(snapshot.target_xy, dtype=float),
            "snapshot_actual_xy": np.asarray(snapshot.actual_xy, dtype=float),
        },
    )


def rollout_cost_candidates(
    candidate_controls: np.ndarray,
    problem: MPCProblem,
    env: Any,
    adapter_config: B02AdapterConfig,
) -> B02RolloutCostResult:
    """对 candidate control sequences 做真实 B02 two-link rollout 并计算 cost。

    当前实现直接复用 B02 `TwoLinkEnv.rollout()`、`set_state()` 与
    `get_end_effector_position()`，因此 solver 不需要知道 MuJoCo 细节。
    """
    controls = np.asarray(candidate_controls, dtype=float)
    if controls.ndim != 3:
        raise ValueError(f"candidate_controls must be 3D, got shape={controls.shape}")
    num_candidates, horizon, control_dim = controls.shape
    if horizon != adapter_config.horizon or horizon != problem.horizon:
        raise ValueError(
            "candidate_controls horizon does not match adapter/problem horizon: "
            f"{horizon}, {adapter_config.horizon}, {problem.horizon}"
        )
    if control_dim != adapter_config.control_dim or control_dim != problem.control_dim:
        raise ValueError(
            "candidate_controls control_dim does not match adapter/problem control_dim: "
            f"{control_dim}, {adapter_config.control_dim}, {problem.control_dim}"
        )

    target_horizon = np.asarray(problem.target_horizon, dtype=float)
    if target_horizon.shape != (horizon, 2):
        raise ValueError(f"problem.target_horizon must have shape {(horizon, 2)}, got {target_horizon.shape}")

    if not hasattr(env, "rollout") or not hasattr(env, "set_state") or not hasattr(env, "get_end_effector_position"):
        raise NotImplementedError(
            "rollout_cost_candidates currently expects a B02-style env with rollout/set_state/get_end_effector_position."
        )

    clipped_controls = np.clip(controls, -adapter_config.torque_limit, adapter_config.torque_limit)
    state_dim = int(np.asarray(problem.current_state, dtype=float).size)
    if state_dim < adapter_config.control_dim * 2:
        raise ValueError(
            "current_state is too short to contain q and dq for two-link cost evaluation: "
            f"state_dim={state_dim}, control_dim={adapter_config.control_dim}"
        )

    predicted_states = (
        np.zeros((num_candidates, horizon + 1, state_dim), dtype=float)
        if adapter_config.record_predicted_states
        else None
    )
    predicted_ee_positions = (
        np.zeros((num_candidates, horizon + 1, 2), dtype=float)
        if adapter_config.record_predicted_ee_positions
        else None
    )
    costs = np.zeros((num_candidates,), dtype=float)

    saved_env_state = _capture_env_state(env)
    initial_state = tuple(float(value) for value in np.asarray(problem.current_state, dtype=float).tolist())

    try:
        for candidate_index in range(num_candidates):
            torque_sequence = [
                tuple(float(value) for value in control)
                for control in clipped_controls[candidate_index]
            ]
            states_list = env.rollout(initial_state, torque_sequence)
            states = np.asarray(states_list, dtype=float)
            if states.shape != (horizon + 1, state_dim):
                raise ValueError(
                    "rollout returned unexpected state shape: "
                    f"{states.shape}, expected {(horizon + 1, state_dim)}"
                )

            ee_positions = np.zeros((horizon + 1, 2), dtype=float)
            for step_index, state in enumerate(states):
                env.set_state(tuple(float(value) for value in state.tolist()))
                ee_positions[step_index] = np.asarray(env.get_end_effector_position(), dtype=float)

            if predicted_states is not None:
                predicted_states[candidate_index] = states
            if predicted_ee_positions is not None:
                predicted_ee_positions[candidate_index] = ee_positions

            dq = states[1:, adapter_config.control_dim : adapter_config.control_dim * 2]
            ee_error = ee_positions[1:] - target_horizon
            torque_penalty = clipped_controls[candidate_index]
            terminal_error = ee_positions[-1] - target_horizon[-1]

            costs[candidate_index] = (
                float(adapter_config.ee_weight) * float(np.sum(ee_error**2))
                + float(adapter_config.dq_weight) * float(np.sum(dq**2))
                + float(adapter_config.torque_weight) * float(np.sum(torque_penalty**2))
                + float(adapter_config.terminal_weight) * float(np.sum(terminal_error**2))
            )
    finally:
        _restore_env_state(env, saved_env_state)

    return B02RolloutCostResult(
        costs=costs,
        predicted_states=predicted_states,
        predicted_ee_positions=predicted_ee_positions,
        candidate_controls=clipped_controls,
    )


def make_b02_rollout_cost_fn(
    env: Any,
    adapter_config: B02AdapterConfig,
    evaluator: Callable[[np.ndarray, MPCProblem, Any, B02AdapterConfig], Any] | None = None,
    return_costs_only: bool = False,
) -> Callable[[np.ndarray, MPCProblem], Any]:
    """生成供 B03 solver 调用的 rollout cost callable。

    兼容两种模式：
    - 返回 `B02RolloutCostResult`，供 solver 读取 cost 和预测轨迹；
    - 返回纯 `costs` 数组，兼容只关心排序的旧 solver。
    """

    def rollout_cost_fn(candidate_controls: np.ndarray, problem: MPCProblem) -> Any:
        result = (
            evaluator(candidate_controls, problem, env, adapter_config)
            if evaluator is not None
            else rollout_cost_candidates(
                candidate_controls=candidate_controls,
                problem=problem,
                env=env,
                adapter_config=adapter_config,
            )
        )
        if return_costs_only and hasattr(result, "costs"):
            return np.asarray(result.costs, dtype=float)
        return result

    return rollout_cost_fn


class B02BaselineSolver(BaseMPCSolver):
    """把 B02 baseline controller 包装成 B03 solver 比较对象。"""

    solver_name = "b02_baseline"

    def __init__(self, controller: Any, env: Any) -> None:
        self.controller = controller
        self.env = env

    def solve(
        self,
        problem: MPCProblem,
        previous_solution: MPCSolution | None = None,
    ) -> MPCSolution:
        """调用 B02 controller，但不修改它的原始接口。"""
        _ = previous_solution
        start_time = time.perf_counter()
        target_sequence = [
            (float(target[0]), float(target[1]))
            for target in np.asarray(problem.target_horizon, dtype=float)
        ]
        current_state = tuple(float(value) for value in np.asarray(problem.current_state, dtype=float).tolist())
        first_control = self.controller.compute_control(
            self.env,
            current_state=current_state,
            target_sequence=target_sequence,
        )

        plan = getattr(self.controller, "last_plan", None) or {}
        best_sequence = np.asarray(plan.get("best_sequence", [first_control]), dtype=float)
        if best_sequence.ndim == 1:
            best_sequence = best_sequence[None, :]

        predicted_states = None
        predicted_ee_positions = None
        best_index = plan.get("best_index", 0)
        candidate_rollouts = plan.get("candidate_rollouts")
        if isinstance(candidate_rollouts, list) and candidate_rollouts:
            predicted_states = np.asarray(candidate_rollouts[int(best_index)], dtype=float)
            try:
                predicted_ee_positions = _compute_ee_positions_for_states(self.env, predicted_states)
            except Exception:
                predicted_ee_positions = None

        best_cost = float(plan.get("best_cost", np.nan))
        num_rollouts = len(plan.get("candidate_sequences", [])) if isinstance(plan.get("candidate_sequences"), list) else 0

        return MPCSolution(
            first_control=np.asarray(first_control, dtype=float),
            predicted_states=predicted_states,
            predicted_controls=best_sequence,
            best_cost=best_cost,
            solver_name=self.solver_name,
            solver_stats=SolverStats(
                runtime_ms=(time.perf_counter() - start_time) * 1000.0,
                num_rollouts=num_rollouts,
                num_iterations=1,
                success=True,
                message="Wrapped B02 controller output.",
            ),
            predicted_ee_positions=predicted_ee_positions,
            selected_index=int(best_index) if best_index is not None else None,
            metadata={"wrapped_last_plan": plan},
        )


def wrap_b02_controller_as_solver(controller: Any, env: Any) -> BaseMPCSolver:
    """把 B02 baseline controller 包装成 B03 `BaseMPCSolver`。"""
    return B02BaselineSolver(controller=controller, env=env)
