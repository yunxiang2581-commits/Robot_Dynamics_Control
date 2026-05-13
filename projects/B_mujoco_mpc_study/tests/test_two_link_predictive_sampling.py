from __future__ import annotations

from pathlib import Path
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SIMULATOR_ROOT = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "simulator"
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

from planners.two_link_predictive_sampling import TwoLinkPredictiveSamplingPlanner


class FakeTwoLinkEnv:
    """测试用的极简 task-space 环境。

    这里把末端位置简化成 `(q1, q2)`，这样可以专注验证 planner 的 cost 逻辑，
    不需要启动真实 MuJoCo。
    """

    def __init__(self) -> None:
        self.state = (9.0, 9.0, 0.0, 0.0)

    def get_state(self) -> tuple[float, float, float, float]:
        return self.state

    def set_state(self, state: tuple[float, float, float, float]) -> None:
        self.state = state

    def get_end_effector_position(self) -> tuple[float, float]:
        q1, q2, _, _ = self.state
        return q1, q2

    def rollout(
        self,
        initial_state: tuple[float, float, float, float],
        torque_sequence: list[tuple[float, float]],
    ) -> list[tuple[float, float, float, float]]:
        tau1, tau2 = torque_sequence[0]
        return [
            initial_state,
            (initial_state[0] + tau1, initial_state[1] + tau2, 0.0, 0.0),
        ]


def test_sample_torque_sequences_returns_two_joint_sequences_in_limit() -> None:
    planner = TwoLinkPredictiveSamplingPlanner(horizon=3, num_candidates=5, torque_limit=2.0)

    sequences = planner.sample_torque_sequences()

    assert len(sequences) == 5
    assert all(len(sequence) == 3 for sequence in sequences)
    assert all(len(torque) == 2 for sequence in sequences for torque in sequence)
    assert all(-2.0 <= value <= 2.0 for sequence in sequences for torque in sequence for value in torque)


def test_compute_horizon_cost_uses_ee_error_velocity_torque_and_terminal_terms() -> None:
    planner = TwoLinkPredictiveSamplingPlanner(
        horizon=1,
        num_candidates=1,
        torque_limit=2.0,
        ee_weight=20.0,
        dq_weight=0.1,
        torque_weight=0.002,
        terminal_weight=5.0,
    )
    env = FakeTwoLinkEnv()
    real_state = env.get_state()

    cost = planner.compute_horizon_cost(
        env=env,
        states=[(0.0, 0.0, 0.0, 0.0), (1.0, 0.0, 0.5, -0.5)],
        torque_sequence=[(2.0, -1.0)],
        target_sequence=[(1.0, 0.0)],
    )

    assert cost == pytest.approx(0.06)
    assert env.get_state() == real_state


def test_select_best_sequence_returns_min_cost_entry() -> None:
    planner = TwoLinkPredictiveSamplingPlanner(horizon=1, num_candidates=3, torque_limit=2.0)

    best_sequence, best_cost, best_index = planner.select_best_sequence(
        candidate_sequences=[
            [(0.0, 0.0)],
            [(1.0, -1.0)],
            [(-1.0, 1.0)],
        ],
        candidate_costs=[3.0, 1.5, 2.0],
    )

    assert best_sequence == [(1.0, -1.0)]
    assert best_cost == 1.5
    assert best_index == 1


def test_plan_rolls_out_candidates_and_returns_first_best_torque() -> None:
    planner = TwoLinkPredictiveSamplingPlanner(
        horizon=1,
        num_candidates=2,
        torque_limit=2.0,
        ee_weight=1.0,
        dq_weight=0.0,
        torque_weight=0.0,
        terminal_weight=0.0,
    )
    planner.sample_torque_sequences = lambda: [[(0.0, 0.0)], [(1.0, 0.0)]]  # type: ignore[method-assign]
    env = FakeTwoLinkEnv()

    result = planner.plan(
        env=env,
        current_state=(0.0, 0.0, 0.0, 0.0),
        target_sequence=[(1.0, 0.0)],
    )

    assert result["best_sequence"] == [(1.0, 0.0)]
    assert result["best_torque"] == (1.0, 0.0)
    assert result["best_index"] == 1
    assert len(result["candidate_costs"]) == 2
    assert len(result["candidate_rollouts"]) == 2
