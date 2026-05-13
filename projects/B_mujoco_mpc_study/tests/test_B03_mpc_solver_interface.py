from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SIMULATOR_ROOT = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "simulator"
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

from planners.mpc_solver_interface import BaseMPCSolver, MPCProblem, MPCSolution, SolverStats


def test_mpc_problem_dataclass_can_be_constructed() -> None:
    problem = MPCProblem(
        current_state=np.array([0.0, 0.0]),
        target_horizon=np.zeros((4, 2)),
        horizon=4,
        control_dim=2,
        dt=0.01,
        cost_config={"ee_weight": 80.0},
        solver_config={"name": "random_shooting"},
    )

    assert problem.horizon == 4
    assert problem.control_dim == 2
    assert problem.rollout_cost_fn is None
    assert problem.metadata == {}


def test_mpc_solution_dataclass_can_be_constructed() -> None:
    solution = MPCSolution(
        first_control=np.array([0.1, -0.1]),
        predicted_states=np.zeros((5, 4)),
        predicted_controls=np.zeros((4, 2)),
        best_cost=1.23,
        solver_name="random_shooting",
        solver_stats=SolverStats(
            runtime_ms=2.5,
            num_rollouts=64,
            num_iterations=1,
            success=True,
            message="ok",
        ),
    )

    assert solution.solver_name == "random_shooting"
    assert solution.best_cost == pytest.approx(1.23)
    assert solution.selected_index is None
    assert solution.predicted_ee_positions is None


def test_solver_stats_dataclass_can_be_constructed() -> None:
    stats = SolverStats(
        runtime_ms=3.5,
        num_rollouts=128,
        num_iterations=2,
        success=False,
        message="todo",
    )

    assert stats.runtime_ms == pytest.approx(3.5)
    assert stats.success is False


def test_base_mpc_solver_solve_raises_not_implemented() -> None:
    solver = BaseMPCSolver()
    problem = MPCProblem(
        current_state=np.array([0.0]),
        target_horizon=np.zeros((2, 1)),
        horizon=2,
        control_dim=1,
        dt=0.01,
        cost_config={},
        solver_config={},
    )

    with pytest.raises(NotImplementedError):
        solver.solve(problem)
