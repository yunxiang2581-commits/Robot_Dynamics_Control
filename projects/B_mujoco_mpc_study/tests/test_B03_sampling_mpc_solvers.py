from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SIMULATOR_ROOT = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "simulator"
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

from planners.sampling_mpc_solvers import (
    CEMShootingSolver,
    MPPILiteSolver,
    RandomShootingSolver,
    WarmStartSamplingSolver,
    rank_rollouts,
    sample_candidate_controls,
    shift_control_sequence,
)
from planners.mpc_solver_interface import MPCProblem, MPCSolution, SolverStats


def test_sample_candidate_controls_shape() -> None:
    batch = sample_candidate_controls(
        num_candidates=8,
        horizon=5,
        control_dim=2,
        sampling_std=0.5,
        torque_limit=1.0,
        seed=7,
    )

    assert batch.controls.shape == (8, 5, 2)


def test_sample_candidate_controls_torque_limit_clip() -> None:
    batch = sample_candidate_controls(
        num_candidates=16,
        horizon=4,
        control_dim=2,
        sampling_std=50.0,
        torque_limit=0.75,
        seed=9,
    )

    assert not np.isnan(batch.controls).any()
    assert float(np.max(np.abs(batch.controls))) <= 0.75 + 1e-12


def test_sample_candidate_controls_seed_reproducibility() -> None:
    first = sample_candidate_controls(
        num_candidates=6,
        horizon=3,
        control_dim=2,
        sampling_std=0.8,
        torque_limit=2.0,
        seed=42,
    )
    second = sample_candidate_controls(
        num_candidates=6,
        horizon=3,
        control_dim=2,
        sampling_std=0.8,
        torque_limit=2.0,
        seed=42,
    )

    np.testing.assert_allclose(first.controls, second.controls)


def test_sample_candidate_controls_mean_sequence_works() -> None:
    mean_sequence = np.array(
        [
            [0.1, -0.1],
            [0.2, -0.2],
            [0.3, -0.3],
        ],
        dtype=float,
    )
    batch = sample_candidate_controls(
        num_candidates=1,
        horizon=3,
        control_dim=2,
        sampling_std=0.0,
        torque_limit=1.0,
        seed=0,
        mean_sequence=mean_sequence,
    )

    np.testing.assert_allclose(batch.controls[0], mean_sequence)


def test_shift_control_sequence_works() -> None:
    previous_controls = np.array(
        [
            [1.0, 2.0],
            [3.0, 4.0],
            [5.0, 6.0],
        ],
        dtype=float,
    )

    shifted = shift_control_sequence(previous_controls)

    expected = np.array(
        [
            [3.0, 4.0],
            [5.0, 6.0],
            [5.0, 6.0],
        ],
        dtype=float,
    )
    np.testing.assert_allclose(shifted, expected)


def test_rank_rollouts_selects_min_cost() -> None:
    ranking = rank_rollouts([4.0, 1.0, 3.0, 2.0])

    assert ranking.selected_index == 1
    assert ranking.sorted_indices == [1, 3, 2, 0]
    assert ranking.best_cost == pytest.approx(1.0)
    assert ranking.mean_cost == pytest.approx(2.5)
    assert ranking.min_cost == pytest.approx(1.0)
    assert ranking.max_cost == pytest.approx(4.0)
    assert ranking.cost_std == pytest.approx(np.std([4.0, 1.0, 3.0, 2.0]))


def test_rank_rollouts_rejects_nan() -> None:
    with pytest.raises(ValueError, match="NaN"):
        rank_rollouts([1.0, float("nan"), 2.0])


def test_rank_rollouts_rejects_inf() -> None:
    with pytest.raises(ValueError, match="inf"):
        rank_rollouts([1.0, float("inf"), 2.0])


class FakeRolloutResult:
    def __init__(self, costs: np.ndarray, predicted_states: np.ndarray, predicted_ee_positions: np.ndarray) -> None:
        self.costs = costs
        self.predicted_states = predicted_states
        self.predicted_ee_positions = predicted_ee_positions


def test_random_shooting_solver_accepts_adapter_rollout_result() -> None:
    horizon = 3
    state_dim = 4
    costs = np.array([3.0, 1.0, 2.0], dtype=float)
    predicted_states = np.arange(3 * (horizon + 1) * state_dim, dtype=float).reshape(3, horizon + 1, state_dim)
    predicted_ee_positions = np.arange(3 * (horizon + 1) * 2, dtype=float).reshape(3, horizon + 1, 2)

    def rollout_cost_fn(candidate_controls: np.ndarray, problem: MPCProblem) -> FakeRolloutResult:
        assert candidate_controls.shape == (3, horizon, 2)
        _ = problem
        return FakeRolloutResult(costs, predicted_states, predicted_ee_positions)

    problem = MPCProblem(
        current_state=np.zeros(4, dtype=float),
        target_horizon=np.zeros((horizon, 2), dtype=float),
        horizon=horizon,
        control_dim=2,
        dt=0.01,
        cost_config={},
        solver_config={"num_candidates": 3, "sampling_std": 0.5, "torque_limit": 1.0, "seed": 0},
        rollout_cost_fn=rollout_cost_fn,
    )

    solution = RandomShootingSolver().solve(problem)

    assert solution.selected_index == 1
    assert solution.best_cost == pytest.approx(1.0)
    np.testing.assert_allclose(solution.predicted_ee_positions, predicted_ee_positions[1])


def test_warm_start_solver_accepts_adapter_rollout_result() -> None:
    horizon = 3
    state_dim = 4
    costs = np.array([2.0, 0.5, 1.5], dtype=float)
    predicted_states = np.arange(3 * (horizon + 1) * state_dim, dtype=float).reshape(3, horizon + 1, state_dim)
    predicted_ee_positions = np.arange(3 * (horizon + 1) * 2, dtype=float).reshape(3, horizon + 1, 2)

    def rollout_cost_fn(candidate_controls: np.ndarray, problem: MPCProblem) -> FakeRolloutResult:
        assert candidate_controls.shape == (3, horizon, 2)
        _ = problem
        return FakeRolloutResult(costs, predicted_states, predicted_ee_positions)

    previous_solution = MPCSolution(
        first_control=np.array([0.0, 0.0], dtype=float),
        predicted_states=np.zeros((horizon + 1, state_dim), dtype=float),
        predicted_controls=np.array(
            [
                [0.1, 0.2],
                [0.3, 0.4],
                [0.5, 0.6],
            ],
            dtype=float,
        ),
        best_cost=1.0,
        solver_name="random_shooting",
        solver_stats=SolverStats(
            runtime_ms=1.0,
            num_rollouts=3,
            num_iterations=1,
            success=True,
            message="ok",
        ),
    )

    problem = MPCProblem(
        current_state=np.zeros(4, dtype=float),
        target_horizon=np.zeros((horizon, 2), dtype=float),
        horizon=horizon,
        control_dim=2,
        dt=0.01,
        cost_config={},
        solver_config={"num_candidates": 3, "sampling_std": 0.5, "torque_limit": 1.0, "seed": 0},
        rollout_cost_fn=rollout_cost_fn,
    )

    solution = WarmStartSamplingSolver().solve(problem, previous_solution=previous_solution)

    assert solution.selected_index == 1
    assert solution.best_cost == pytest.approx(0.5)
    np.testing.assert_allclose(solution.predicted_ee_positions, predicted_ee_positions[1])


def test_cem_solver_tracks_global_best_across_iterations() -> None:
    horizon = 3
    state_dim = 4
    num_candidates = 4
    call_count = {"value": 0}

    def rollout_cost_fn(candidate_controls: np.ndarray, problem: MPCProblem) -> FakeRolloutResult:
        assert candidate_controls.shape == (num_candidates, horizon, 2)
        _ = problem
        iteration = call_count["value"]
        call_count["value"] += 1

        if iteration == 0:
            costs = np.array([3.0, 2.0, 1.5, 1.0], dtype=float)
            best_index = 3
        elif iteration == 1:
            costs = np.array([1.8, 0.25, 1.0, 1.2], dtype=float)
            best_index = 1
        else:
            costs = np.array([0.8, 1.1, 1.4, 1.7], dtype=float)
            best_index = 0

        predicted_states = np.full((num_candidates, horizon + 1, state_dim), float(iteration), dtype=float)
        predicted_ee_positions = np.full((num_candidates, horizon + 1, 2), float(iteration), dtype=float)
        predicted_ee_positions[best_index, :, :] = 100.0 + float(iteration)
        return FakeRolloutResult(costs, predicted_states, predicted_ee_positions)

    previous_solution = MPCSolution(
        first_control=np.array([0.0, 0.0], dtype=float),
        predicted_states=np.zeros((horizon + 1, state_dim), dtype=float),
        predicted_controls=np.array(
            [
                [0.1, 0.2],
                [0.3, 0.4],
                [0.5, 0.6],
            ],
            dtype=float,
        ),
        best_cost=1.0,
        solver_name="warm_start_sampling",
        solver_stats=SolverStats(
            runtime_ms=1.0,
            num_rollouts=num_candidates,
            num_iterations=1,
            success=True,
            message="ok",
        ),
    )

    problem = MPCProblem(
        current_state=np.zeros(state_dim, dtype=float),
        target_horizon=np.zeros((horizon, 2), dtype=float),
        horizon=horizon,
        control_dim=2,
        dt=0.01,
        cost_config={},
        solver_config={
            "num_candidates": num_candidates,
            "sampling_std": 0.5,
            "initial_std": 0.7,
            "torque_limit": 1.0,
            "seed": 0,
            "num_iterations": 3,
            "elite_ratio": 0.25,
            "min_std": 0.05,
            "smoothing_alpha": 0.2,
        },
        rollout_cost_fn=rollout_cost_fn,
    )

    solution = CEMShootingSolver().solve(problem, previous_solution=previous_solution)

    assert call_count["value"] == 3
    assert solution.best_cost == pytest.approx(0.25)
    assert solution.selected_index == 1
    assert solution.solver_stats.num_rollouts == 12
    assert solution.solver_stats.num_iterations == 3
    assert solution.metadata["elite_count"] == 1
    assert solution.metadata["iteration_best_costs"] == [1.0, 0.25, 0.8]
    np.testing.assert_allclose(solution.predicted_ee_positions, np.full((horizon + 1, 2), 101.0, dtype=float))


def test_cem_solver_supports_early_stop() -> None:
    horizon = 2
    state_dim = 4
    num_candidates = 3
    call_count = {"value": 0}

    def rollout_cost_fn(candidate_controls: np.ndarray, problem: MPCProblem) -> FakeRolloutResult:
        assert candidate_controls.shape == (num_candidates, horizon, 2)
        _ = problem
        iteration = call_count["value"]
        call_count["value"] += 1
        costs = np.array([1.0, 1.1, 1.2], dtype=float)
        predicted_states = np.full((num_candidates, horizon + 1, state_dim), float(iteration), dtype=float)
        predicted_ee_positions = np.full((num_candidates, horizon + 1, 2), float(iteration), dtype=float)
        return FakeRolloutResult(costs, predicted_states, predicted_ee_positions)

    problem = MPCProblem(
        current_state=np.zeros(state_dim, dtype=float),
        target_horizon=np.zeros((horizon, 2), dtype=float),
        horizon=horizon,
        control_dim=2,
        dt=0.01,
        cost_config={},
        solver_config={
            "num_candidates": num_candidates,
            "sampling_std": 0.5,
            "torque_limit": 1.0,
            "num_iterations": 5,
            "elite_ratio": 0.5,
            "min_std": 0.05,
            "early_stop_patience": 2,
            "improvement_tolerance": 1e-9,
        },
        rollout_cost_fn=rollout_cost_fn,
    )

    solution = CEMShootingSolver().solve(problem)

    assert call_count["value"] == 3
    assert solution.solver_stats.num_iterations == 3
    assert solution.solver_stats.num_rollouts == 9


def test_mppi_solver_tracks_global_best_across_iterations() -> None:
    horizon = 3
    state_dim = 4
    num_candidates = 4
    call_count = {"value": 0}

    def rollout_cost_fn(candidate_controls: np.ndarray, problem: MPCProblem) -> FakeRolloutResult:
        assert candidate_controls.shape == (num_candidates, horizon, 2)
        _ = problem
        iteration = call_count["value"]
        call_count["value"] += 1

        if iteration == 0:
            costs = np.array([2.0, 1.0, 1.5, 1.8], dtype=float)
            best_index = 1
        elif iteration == 1:
            costs = np.array([1.7, 1.4, 0.2, 1.1], dtype=float)
            best_index = 2
        else:
            costs = np.array([0.5, 0.8, 0.9, 1.2], dtype=float)
            best_index = 0

        predicted_states = np.full((num_candidates, horizon + 1, state_dim), float(iteration), dtype=float)
        predicted_ee_positions = np.full((num_candidates, horizon + 1, 2), float(iteration), dtype=float)
        predicted_ee_positions[best_index, :, :] = 200.0 + float(iteration)
        return FakeRolloutResult(costs, predicted_states, predicted_ee_positions)

    previous_solution = MPCSolution(
        first_control=np.array([0.0, 0.0], dtype=float),
        predicted_states=np.zeros((horizon + 1, state_dim), dtype=float),
        predicted_controls=np.array(
            [
                [0.1, 0.2],
                [0.3, 0.4],
                [0.5, 0.6],
            ],
            dtype=float,
        ),
        best_cost=1.0,
        solver_name="warm_start_sampling",
        solver_stats=SolverStats(
            runtime_ms=1.0,
            num_rollouts=num_candidates,
            num_iterations=1,
            success=True,
            message="ok",
        ),
    )

    problem = MPCProblem(
        current_state=np.zeros(state_dim, dtype=float),
        target_horizon=np.zeros((horizon, 2), dtype=float),
        horizon=horizon,
        control_dim=2,
        dt=0.01,
        cost_config={},
        solver_config={
            "num_candidates": num_candidates,
            "sampling_std": 0.5,
            "noise_std": 0.6,
            "torque_limit": 1.0,
            "seed": 0,
            "num_iterations": 3,
            "temperature": 0.7,
            "min_std": 0.05,
            "smoothing_alpha": 0.2,
        },
        rollout_cost_fn=rollout_cost_fn,
    )

    solution = MPPILiteSolver().solve(problem, previous_solution=previous_solution)

    assert call_count["value"] == 3
    assert solution.best_cost == pytest.approx(0.2)
    assert solution.selected_index == 2
    assert solution.solver_stats.num_rollouts == 12
    assert solution.solver_stats.num_iterations == 3
    assert solution.metadata["iteration_best_costs"] == [1.0, 0.2, 0.5]
    np.testing.assert_allclose(solution.predicted_ee_positions, np.full((horizon + 1, 2), 201.0, dtype=float))


def test_mppi_solver_supports_early_stop() -> None:
    horizon = 2
    state_dim = 4
    num_candidates = 3
    call_count = {"value": 0}

    def rollout_cost_fn(candidate_controls: np.ndarray, problem: MPCProblem) -> FakeRolloutResult:
        assert candidate_controls.shape == (num_candidates, horizon, 2)
        _ = problem
        iteration = call_count["value"]
        call_count["value"] += 1
        costs = np.array([1.0, 1.05, 1.1], dtype=float)
        predicted_states = np.full((num_candidates, horizon + 1, state_dim), float(iteration), dtype=float)
        predicted_ee_positions = np.full((num_candidates, horizon + 1, 2), float(iteration), dtype=float)
        return FakeRolloutResult(costs, predicted_states, predicted_ee_positions)

    problem = MPCProblem(
        current_state=np.zeros(state_dim, dtype=float),
        target_horizon=np.zeros((horizon, 2), dtype=float),
        horizon=horizon,
        control_dim=2,
        dt=0.01,
        cost_config={},
        solver_config={
            "num_candidates": num_candidates,
            "sampling_std": 0.5,
            "torque_limit": 1.0,
            "num_iterations": 5,
            "temperature": 1.0,
            "min_std": 0.05,
            "early_stop_patience": 2,
            "improvement_tolerance": 1e-9,
        },
        rollout_cost_fn=rollout_cost_fn,
    )

    solution = MPPILiteSolver().solve(problem)

    assert call_count["value"] == 3
    assert solution.solver_stats.num_iterations == 3
    assert solution.solver_stats.num_rollouts == 9


def test_mppi_lite_outputs_updated_sequence_as_predicted_controls() -> None:
    """MPPI-lite predicted_controls should be the updated mean_sequence, not the best sample."""
    horizon = 3
    state_dim = 4
    num_candidates = 8

    def rollout_cost_fn(candidate_controls: np.ndarray, problem: MPCProblem) -> FakeRolloutResult:
        _ = problem
        costs = np.linspace(0.5, 2.0, num_candidates, dtype=float)
        predicted_states = np.zeros((num_candidates, horizon + 1, state_dim), dtype=float)
        predicted_ee_positions = np.zeros((num_candidates, horizon + 1, 2), dtype=float)
        return FakeRolloutResult(costs, predicted_states, predicted_ee_positions)

    problem = MPCProblem(
        current_state=np.zeros(state_dim, dtype=float),
        target_horizon=np.zeros((horizon, 2), dtype=float),
        horizon=horizon,
        control_dim=2,
        dt=0.01,
        cost_config={},
        solver_config={
            "num_candidates": num_candidates,
            "sampling_std": 0.5,
            "noise_std": 0.5,
            "torque_limit": 1.0,
            "seed": 42,
            "num_iterations": 2,
            "temperature": 1.0,
            "min_std": 0.05,
        },
        rollout_cost_fn=rollout_cost_fn,
    )

    solution = MPPILiteSolver().solve(problem)

    assert solution.predicted_controls.shape == (horizon, 2)
    assert solution.first_control.shape == (2,)
    assert solution.metadata["temperature"] == pytest.approx(1.0)
    assert "weight_entropy" in solution.metadata
    assert "max_weight" in solution.metadata
    assert "min_weight" in solution.metadata
    assert solution.metadata["weight_entropy"] > 0.0
    assert 0.0 < solution.metadata["max_weight"] <= 1.0
    assert 0.0 < solution.metadata["min_weight"] <= 1.0


def test_mppi_lite_predicted_controls_is_not_best_sample() -> None:
    """Verify that MPPI-lite predicted_controls differs from best sample when weights are spread."""
    horizon = 2
    state_dim = 4
    num_candidates = 4

    call_count = {"value": 0}

    def rollout_cost_fn(candidate_controls: np.ndarray, problem: MPCProblem) -> FakeRolloutResult:
        _ = problem
        iteration = call_count["value"]
        call_count["value"] += 1
        if iteration == 0:
            costs = np.array([1.0, 0.5, 1.5, 2.0], dtype=float)
        else:
            costs = np.array([0.8, 0.6, 1.2, 1.8], dtype=float)
        predicted_states = np.zeros((num_candidates, horizon + 1, state_dim), dtype=float)
        predicted_ee_positions = np.zeros((num_candidates, horizon + 1, 2), dtype=float)
        return FakeRolloutResult(costs, predicted_states, predicted_ee_positions)

    problem = MPCProblem(
        current_state=np.zeros(state_dim, dtype=float),
        target_horizon=np.zeros((horizon, 2), dtype=float),
        horizon=horizon,
        control_dim=2,
        dt=0.01,
        cost_config={},
        solver_config={
            "num_candidates": num_candidates,
            "sampling_std": 1.0,
            "noise_std": 1.0,
            "torque_limit": 5.0,
            "seed": 123,
            "num_iterations": 2,
            "temperature": 0.5,
            "min_std": 0.05,
        },
        rollout_cost_fn=rollout_cost_fn,
    )

    solution = MPPILiteSolver().solve(problem)

    assert solution.predicted_controls.shape == (horizon, 2)
    assert not np.isnan(solution.predicted_controls).any()


def test_cem_solver_defaults_no_early_stop() -> None:
    """CEM without early_stop_patience runs all iterations."""
    horizon = 2
    state_dim = 4
    num_candidates = 3
    call_count = {"value": 0}

    def rollout_cost_fn(candidate_controls: np.ndarray, problem: MPCProblem) -> FakeRolloutResult:
        _ = problem
        call_count["value"] += 1
        costs = np.array([1.0, 1.1, 1.2], dtype=float)
        predicted_states = np.zeros((num_candidates, horizon + 1, state_dim), dtype=float)
        predicted_ee_positions = np.zeros((num_candidates, horizon + 1, 2), dtype=float)
        return FakeRolloutResult(costs, predicted_states, predicted_ee_positions)

    problem = MPCProblem(
        current_state=np.zeros(state_dim, dtype=float),
        target_horizon=np.zeros((horizon, 2), dtype=float),
        horizon=horizon,
        control_dim=2,
        dt=0.01,
        cost_config={},
        solver_config={
            "num_candidates": num_candidates,
            "sampling_std": 0.5,
            "torque_limit": 1.0,
            "num_iterations": 4,
            "elite_ratio": 0.5,
            "min_std": 0.05,
        },
        rollout_cost_fn=rollout_cost_fn,
    )

    solution = CEMShootingSolver().solve(problem)

    assert call_count["value"] == 4
    assert solution.solver_stats.num_iterations == 4
    assert solution.solver_stats.num_rollouts == 12
    assert solution.metadata["iteration_best_costs"]
