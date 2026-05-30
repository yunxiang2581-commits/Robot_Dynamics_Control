from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SIMULATOR_ROOT = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "simulator"
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

from planners.ilqg_solver import (
    ILQGConfig,
    ILQGLiteSolver,
    LinearizedDynamics,
    QuadraticCostApproximation,
    backward_pass,
    forward_pass,
    linearize_trajectory_dynamics,
    quadratize_trajectory_cost,
    rollout_nominal_trajectory,
)
from planners.mpc_solver_interface import MPCProblem


def _double_integrator_dynamics(dt: float = 0.1):
    def dynamics_fn(x: np.ndarray, u: np.ndarray) -> np.ndarray:
        return np.array(
            [
                x[0] + dt * x[1],
                x[1] + dt * u[0],
            ],
            dtype=float,
        )

    return dynamics_fn


def _tracking_cost_fn(x_refs: np.ndarray, Q: np.ndarray, R: np.ndarray):
    def cost_fn(x: np.ndarray, u: np.ndarray, t: int) -> float:
        dx = np.asarray(x, dtype=float) - x_refs[t]
        u_arr = np.asarray(u, dtype=float)
        return float(0.5 * dx.T @ Q @ dx + 0.5 * u_arr.T @ R @ u_arr)

    return cost_fn


def test_rollout_nominal_trajectory_shapes() -> None:
    horizon = 4
    controls = np.zeros((horizon, 1), dtype=float)
    x_refs = np.zeros((horizon + 1, 2), dtype=float)
    cost_fn = _tracking_cost_fn(x_refs, np.eye(2), np.eye(1))

    nominal = rollout_nominal_trajectory(
        dynamics_fn=_double_integrator_dynamics(),
        initial_state=np.array([1.0, 0.0], dtype=float),
        controls=controls,
        cost_fn=cost_fn,
    )

    assert nominal.states.shape == (horizon + 1, 2)
    assert nominal.controls.shape == (horizon, 1)
    assert nominal.costs.shape == (horizon,)
    assert np.isfinite(nominal.total_cost)
    np.testing.assert_allclose(nominal.states[0], np.array([1.0, 0.0]))


def test_linearize_trajectory_dynamics_linear_system_values() -> None:
    dt = 0.2
    true_a = np.array([[1.0, dt], [0.0, 1.0]], dtype=float)
    true_b = np.array([[0.0], [dt]], dtype=float)
    dynamics_fn = _double_integrator_dynamics(dt=dt)
    controls = np.zeros((3, 1), dtype=float)
    nominal = rollout_nominal_trajectory(
        dynamics_fn=dynamics_fn,
        initial_state=np.array([0.5, -0.2], dtype=float),
        controls=controls,
        cost_fn=lambda x, u, t: 0.0,
    )

    linearized = linearize_trajectory_dynamics(
        dynamics_fn=dynamics_fn,
        nominal_trajectory=nominal,
        eps=1e-6,
    )

    assert linearized.A.shape == (3, 2, 2)
    assert linearized.B.shape == (3, 2, 1)
    np.testing.assert_allclose(linearized.A[0], true_a, atol=1e-6)
    np.testing.assert_allclose(linearized.B[0], true_b, atol=1e-6)


def test_quadratize_trajectory_cost_shapes() -> None:
    horizon = 5
    states = np.zeros((horizon + 1, 2), dtype=float)
    controls = np.zeros((horizon, 1), dtype=float)
    x_refs = np.ones((horizon + 1, 2), dtype=float)

    quad = quadratize_trajectory_cost(
        states=states,
        controls=controls,
        x_refs=x_refs,
        Q=np.diag([10.0, 1.0]),
        R=np.diag([0.1]),
        Q_terminal=np.diag([20.0, 2.0]),
    )

    assert quad.l_x.shape == (horizon, 2)
    assert quad.l_u.shape == (horizon, 1)
    assert quad.l_xx.shape == (horizon, 2, 2)
    assert quad.l_uu.shape == (horizon, 1, 1)
    assert quad.l_ux.shape == (horizon, 1, 2)
    assert quad.terminal_x.shape == (2,)
    assert quad.terminal_xx.shape == (2, 2)


def test_backward_pass_lqr_shapes() -> None:
    horizon = 4
    linearized = LinearizedDynamics(
        A=np.repeat(np.array([[[1.0, 0.1], [0.0, 1.0]]], dtype=float), horizon, axis=0),
        B=np.repeat(np.array([[[0.0], [0.1]]], dtype=float), horizon, axis=0),
    )
    quad = QuadraticCostApproximation(
        l_x=np.zeros((horizon, 2), dtype=float),
        l_u=np.zeros((horizon, 1), dtype=float),
        l_xx=np.repeat(np.eye(2)[None, :, :], horizon, axis=0),
        l_uu=np.repeat(np.eye(1)[None, :, :], horizon, axis=0),
        l_ux=np.zeros((horizon, 1, 2), dtype=float),
        terminal_x=np.zeros(2, dtype=float),
        terminal_xx=np.eye(2),
    )

    result = backward_pass(linearized, quad, ILQGConfig(horizon=horizon))

    assert result.success is True
    assert result.feedforward_gains.shape == (horizon, 1)
    assert result.feedback_gains.shape == (horizon, 1, 2)
    assert np.isfinite(result.expected_cost_reduction)


def test_forward_pass_decreases_or_returns_finite_cost() -> None:
    horizon = 6
    dynamics_fn = _double_integrator_dynamics(dt=0.1)
    controls = np.zeros((horizon, 1), dtype=float)
    x_refs = np.zeros((horizon + 1, 2), dtype=float)
    cost_fn = _tracking_cost_fn(x_refs, np.eye(2), np.eye(1))
    nominal = rollout_nominal_trajectory(
        dynamics_fn=dynamics_fn,
        initial_state=np.array([1.0, 0.0], dtype=float),
        controls=controls,
        cost_fn=cost_fn,
    )
    linearized = linearize_trajectory_dynamics(dynamics_fn, nominal, eps=1e-6)
    quad = quadratize_trajectory_cost(
        nominal.states,
        nominal.controls,
        x_refs,
        Q=np.eye(2),
        R=np.eye(1),
        Q_terminal=np.eye(2),
    )
    backward = backward_pass(linearized, quad, ILQGConfig(horizon=horizon))

    candidate = forward_pass(
        dynamics_fn=dynamics_fn,
        cost_fn=cost_fn,
        nominal_trajectory=nominal,
        backward_result=backward,
        initial_state=np.array([1.0, 0.0], dtype=float),
        alpha=1.0,
        config=ILQGConfig(horizon=horizon, control_limit=10.0),
    )

    assert candidate.states.shape == (horizon + 1, 2)
    assert candidate.controls.shape == (horizon, 1)
    assert np.isfinite(candidate.total_cost)


def test_ilqg_lite_solver_toy_problem_returns_solution() -> None:
    horizon = 12
    x_refs = np.zeros((horizon + 1, 2), dtype=float)
    x_refs[:, 0] = 1.0
    problem = MPCProblem(
        current_state=np.array([0.0, 0.0], dtype=float),
        target_horizon=x_refs,
        horizon=horizon,
        control_dim=1,
        dt=0.1,
        cost_config={},
        solver_config={
            "max_iterations": 5,
            "control_limit": 10.0,
            "line_search_alphas": [1.0, 0.5, 0.25],
            "regularization": 1e-6,
            "tolerance": 1e-9,
        },
        dynamics_fn=_double_integrator_dynamics(dt=0.1),
        metadata={
            "x_refs": x_refs,
            "Q": np.diag([10.0, 1.0]),
            "R": np.diag([0.1]),
            "Q_terminal": np.diag([20.0, 2.0]),
        },
    )

    solution = ILQGLiteSolver().solve(problem)

    assert solution.first_control.shape == (1,)
    assert solution.predicted_controls.shape == (horizon, 1)
    assert solution.predicted_states.shape == (horizon + 1, 2)
    assert solution.solver_name == "ilqg_lite"
    assert np.isfinite(solution.best_cost)
    assert solution.solver_stats.success is True
    assert solution.metadata["cost_history"]


def test_ilqg_lite_solver_accepts_custom_cost_functions_without_x_refs() -> None:
    """task-space cost 不能总是写成 x_ref/Q/R，需要 solver 接受通用 cost。"""
    horizon = 6
    target_xy = np.array([0.25, 0.05], dtype=float)

    def end_effector_fn(x: np.ndarray) -> np.ndarray:
        x_arr = np.asarray(x, dtype=float)
        return np.array([x_arr[0] + 0.5 * x_arr[1], x_arr[1]], dtype=float)

    def stage_cost_fn(x: np.ndarray, u: np.ndarray, _t: int) -> float:
        ee_error = end_effector_fn(x) - target_xy
        u_arr = np.asarray(u, dtype=float)
        return float(0.5 * ee_error.T @ ee_error + 0.05 * u_arr.T @ u_arr)

    def terminal_cost_fn(x: np.ndarray) -> float:
        ee_error = end_effector_fn(x) - target_xy
        return float(2.0 * ee_error.T @ ee_error)

    problem = MPCProblem(
        current_state=np.array([0.0, 0.0], dtype=float),
        target_horizon=np.zeros((horizon + 1, 2), dtype=float),
        horizon=horizon,
        control_dim=1,
        dt=0.1,
        cost_config={},
        solver_config={
            "max_iterations": 2,
            "control_limit": 5.0,
            "line_search_alphas": [1.0, 0.5],
            "finite_difference_eps": 1.0e-5,
            "regularization": 1.0e-5,
        },
        dynamics_fn=_double_integrator_dynamics(dt=0.1),
        metadata={
            "stage_cost_fn": stage_cost_fn,
            "terminal_cost_fn": terminal_cost_fn,
            "end_effector_fn": end_effector_fn,
            "initial_controls": np.zeros((horizon, 1), dtype=float),
        },
    )

    solution = ILQGLiteSolver().solve(problem)

    assert solution.predicted_states.shape == (horizon + 1, 2)
    assert solution.predicted_controls.shape == (horizon, 1)
    assert solution.predicted_ee_positions is not None
    assert solution.predicted_ee_positions.shape == (horizon + 1, 2)
    assert np.isfinite(solution.best_cost)
    assert solution.metadata["cost_model"] == "custom_finite_difference"


def test_ilqg_lite_solver_requires_dynamics_fn() -> None:
    horizon = 3
    x_refs = np.zeros((horizon + 1, 2), dtype=float)
    problem = MPCProblem(
        current_state=np.zeros(2, dtype=float),
        target_horizon=x_refs,
        horizon=horizon,
        control_dim=1,
        dt=0.1,
        cost_config={},
        solver_config={},
        metadata={"x_refs": x_refs},
    )

    with pytest.raises(ValueError, match="dynamics_fn"):
        ILQGLiteSolver().solve(problem)


def test_ilqg_lite_solver_requires_x_refs() -> None:
    horizon = 3
    problem = MPCProblem(
        current_state=np.zeros(2, dtype=float),
        target_horizon=np.zeros((horizon, 1), dtype=float),
        horizon=horizon,
        control_dim=1,
        dt=0.1,
        cost_config={},
        solver_config={},
        dynamics_fn=_double_integrator_dynamics(dt=0.1),
    )

    with pytest.raises(ValueError, match="x_refs"):
        ILQGLiteSolver().solve(problem)
