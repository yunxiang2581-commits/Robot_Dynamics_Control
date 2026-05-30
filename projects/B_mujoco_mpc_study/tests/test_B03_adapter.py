from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SIMULATOR_ROOT = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "simulator"
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

from adapters.b02_to_b03_adapter import (
    B02AdapterConfig,
    B02TrackingSnapshot,
    build_b03_problem_from_b02_snapshot,
    make_b02_rollout_cost_fn,
    rollout_cost_candidates,
    wrap_b02_controller_as_solver,
)
from planners.mpc_solver_interface import MPCSolution, SolverStats


class FakeBaselineController:
    def __init__(self) -> None:
        self.last_plan: dict[str, object] | None = None

    def compute_control(
        self,
        env: object,
        current_state: tuple[float, float, float, float],
        target_sequence: list[tuple[float, float]],
    ) -> tuple[float, float]:
        _ = env, current_state, target_sequence
        self.last_plan = {
            "best_sequence": [(0.1, -0.2), (0.2, -0.1)],
            "best_index": 0,
            "best_cost": 1.5,
            "candidate_sequences": [[(0.1, -0.2), (0.2, -0.1)]],
            "candidate_rollouts": [[(0.1, 0.2, 0.0, 0.0), (0.2, 0.3, 0.0, 0.0), (0.3, 0.4, 0.0, 0.0)]],
        }
        return 0.1, -0.2


def _make_snapshot() -> B02TrackingSnapshot:
    return B02TrackingSnapshot(
        time=0.0,
        q=np.array([0.1, 0.2], dtype=float),
        qvel=np.array([0.0, 0.0], dtype=float),
        state=np.array([0.1, 0.2, 0.0, 0.0], dtype=float),
        target_xy=np.array([0.4, 0.2], dtype=float),
        actual_xy=np.array([0.35, 0.18], dtype=float),
        error_norm=float(np.linalg.norm(np.array([0.35, 0.18]) - np.array([0.4, 0.2]))),
    )


def _make_adapter_config() -> B02AdapterConfig:
    return B02AdapterConfig(
        horizon=3,
        dt=0.01,
        control_dim=2,
        torque_limit=10.0,
        ee_weight=80.0,
        dq_weight=0.1,
        torque_weight=0.002,
        terminal_weight=10.0,
        record_predicted_states=True,
        record_predicted_ee_positions=True,
    )


def test_build_b03_problem_from_b02_snapshot_shapes() -> None:
    snapshot = _make_snapshot()
    adapter_config = _make_adapter_config()
    target_horizon = np.array(
        [
            [0.4, 0.2],
            [0.41, 0.21],
            [0.42, 0.22],
        ],
        dtype=float,
    )

    problem = build_b03_problem_from_b02_snapshot(
        snapshot=snapshot,
        target_horizon=target_horizon,
        adapter_config=adapter_config,
    )

    assert problem.current_state.shape == (4,)
    assert problem.target_horizon.shape == (3, 2)
    assert "ee_weight" in problem.cost_config
    assert "torque_weight" in problem.cost_config


def test_make_b02_rollout_cost_fn_fake_result() -> None:
    snapshot = _make_snapshot()
    adapter_config = _make_adapter_config()
    target_horizon = np.zeros((3, 2), dtype=float)

    def fake_evaluator(
        candidate_controls: np.ndarray,
        problem: object,
        env: object,
        cfg: B02AdapterConfig,
    ) -> np.ndarray:
        _ = problem, env, cfg
        return np.sum(candidate_controls**2, axis=(1, 2))

    rollout_cost_fn = make_b02_rollout_cost_fn(
        env=object(),
        adapter_config=adapter_config,
        evaluator=fake_evaluator,
    )
    problem = build_b03_problem_from_b02_snapshot(
        snapshot=snapshot,
        target_horizon=target_horizon,
        adapter_config=adapter_config,
        rollout_cost_fn=rollout_cost_fn,
    )
    candidate_controls = np.ones((4, 3, 2), dtype=float)

    costs = rollout_cost_fn(candidate_controls, problem)

    assert isinstance(costs, np.ndarray)
    assert costs.shape == (4,)
    assert not np.isnan(costs).any()


def test_rollout_cost_candidates_rejects_bad_shape() -> None:
    snapshot = _make_snapshot()
    adapter_config = _make_adapter_config()
    problem = build_b03_problem_from_b02_snapshot(
        snapshot=snapshot,
        target_horizon=np.zeros((3, 2), dtype=float),
        adapter_config=adapter_config,
    )

    with pytest.raises(ValueError):
        rollout_cost_candidates(
            candidate_controls=np.zeros((3, 2), dtype=float),
            problem=problem,
            env=object(),
            adapter_config=adapter_config,
        )

    with pytest.raises(ValueError):
        rollout_cost_candidates(
            candidate_controls=np.zeros((2, 3, 3), dtype=float),
            problem=problem,
            env=object(),
            adapter_config=adapter_config,
        )


def test_b02_baseline_solver_wrapper_returns_solution() -> None:
    controller = FakeBaselineController()
    solver = wrap_b02_controller_as_solver(controller=controller, env=object())
    problem = build_b03_problem_from_b02_snapshot(
        snapshot=_make_snapshot(),
        target_horizon=np.array(
            [
                [0.4, 0.2],
                [0.41, 0.21],
                [0.42, 0.22],
            ],
            dtype=float,
        ),
        adapter_config=_make_adapter_config(),
    )

    solution = solver.solve(problem)

    assert isinstance(solution, MPCSolution)
    assert solution.solver_name == "b02_baseline"
    np.testing.assert_allclose(solution.first_control, np.array([0.1, -0.2]))
    assert solution.best_cost == pytest.approx(1.5)
