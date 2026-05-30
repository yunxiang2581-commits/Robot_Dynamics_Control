from __future__ import annotations

from pathlib import Path
import csv
import sys

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SIMULATOR_ROOT = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "simulator"
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

from projects.B_mujoco_mpc_study.simulator.scripts import (  # noqa: E402
    run_B03_task_space_warm_start_smoke as runner,
)


class FakeTaskSpaceEnv:
    def __init__(self) -> None:
        self.data = type("FakeData", (), {})()
        self.data.qpos = np.array([0.2, 0.3], dtype=float)
        self.data.qvel = np.array([0.0, 0.0], dtype=float)
        self.data.ctrl = np.array([0.0, 0.0], dtype=float)
        self.data.time = 0.0
        self.model = object()
        self.dt = 0.05
        self.last_applied_torque = (0.0, 0.0)

    def forward(self) -> None:
        return None

    def get_state(self) -> tuple[float, float, float, float]:
        return (
            float(self.data.qpos[0]),
            float(self.data.qpos[1]),
            float(self.data.qvel[0]),
            float(self.data.qvel[1]),
        )

    def set_state(self, state: tuple[float, float, float, float]) -> None:
        self.data.qpos[:] = [float(state[0]), float(state[1])]
        self.data.qvel[:] = [float(state[2]), float(state[3])]
        self.data.ctrl[:] = 0.0

    def step(self, torque: tuple[float, float]) -> tuple[float, float, float, float]:
        u = np.asarray(torque, dtype=float)
        self.data.ctrl[:] = u
        self.data.qpos[:] = self.data.qpos + self.dt * self.data.qvel
        self.data.qvel[:] = self.data.qvel + self.dt * u
        self.data.time += self.dt
        self.last_applied_torque = (float(u[0]), float(u[1]))
        return self.get_state()

    def get_end_effector_position(self) -> tuple[float, float]:
        q1, q2 = self.data.qpos
        return float(np.cos(q1) + 0.5 * np.cos(q1 + q2)), float(np.sin(q1) + 0.5 * np.sin(q1 + q2))


class FakeB02RolloutEnv(FakeTaskSpaceEnv):
    def rollout(self, initial_state: tuple[float, float, float, float], torque_sequence: list[tuple[float, float]]) -> list[tuple[float, float, float, float]]:
        saved_state = self.get_state()
        self.set_state(initial_state)
        states = [self.get_state()]
        for torque in torque_sequence:
            states.append(self.step(torque))
        self.set_state(saved_state)
        return states


def _config() -> dict[str, object]:
    return {
        "simulation": {"dt": 0.05},
        "task_space": {
            "ee_weight": 80.0,
            "terminal_ee_weight": 120.0,
            "velocity_weight": 0.2,
            "control_weight": 0.05,
            "reference_type": "circle",
            "reference_radius": 0.03,
            "reference_cycles": 0.25,
        },
        "ilqg_lite": {
            "max_iterations": 1,
            "control_limit": 1.0,
            "line_search_alphas": [1.0, 0.5],
            "finite_difference_eps": 1.0e-5,
            "regularization": 1.0e-5,
        },
        "sampling": {"num_candidates": 5, "sampling_std": 0.1, "torque_limit": 1.0, "seed": 2},
        "cem": {"num_candidates": 5, "num_iterations": 1, "initial_std": 0.1, "torque_limit": 1.0, "seed": 2},
    }


def test_build_task_space_tracking_problem_evaluates_ee_rollout() -> None:
    env = FakeTaskSpaceEnv()
    horizon = 4
    problem = runner.build_task_space_tracking_problem(
        env=env,
        config=_config(),
        horizon=horizon,
        solver_family="cem",
        sampling_overrides={"num_candidates": 3, "num_iterations": 1, "sampling_std": 0.1, "torque_limit": 1.0, "seed": 1},
    )

    assert problem.target_horizon.shape == (horizon + 1, 2)
    assert problem.metadata["task_name"] == "B03-R4D_task_space_tracking"
    assert problem.metadata["tracking_space"] == "task_space_xy"
    assert callable(problem.rollout_cost_fn)
    assert callable(problem.metadata["stage_cost_fn"])
    assert callable(problem.metadata["terminal_cost_fn"])

    candidate_controls = np.zeros((2, horizon, 2), dtype=float)
    candidate_controls[1, :, 0] = 0.2
    rollout = runner.evaluate_task_space_rollouts(candidate_controls, problem)

    assert rollout.costs.shape == (2,)
    assert rollout.predicted_states.shape == (2, horizon + 1, 4)
    assert rollout.predicted_ee_positions.shape == (2, horizon + 1, 2)
    assert np.isfinite(rollout.costs).all()


def test_build_task_space_tracking_problem_accepts_external_target_horizon() -> None:
    env = FakeTaskSpaceEnv()
    horizon = 4
    external_targets = np.array(
        [
            [0.40, 0.10],
            [0.42, 0.12],
            [0.44, 0.08],
            [0.46, 0.11],
            [0.48, 0.09],
        ],
        dtype=float,
    )

    problem = runner.build_task_space_tracking_problem(
        env=env,
        config=_config(),
        horizon=horizon,
        solver_family="cem",
        target_ee_positions=external_targets,
        sampling_overrides={"num_candidates": 3, "num_iterations": 1, "sampling_std": 0.1, "torque_limit": 1.0, "seed": 1},
    )

    np.testing.assert_allclose(problem.target_horizon, external_targets)
    np.testing.assert_allclose(problem.metadata["target_ee_positions"], external_targets)


def test_post_step_rollout_terminal_cost_uses_last_executed_target() -> None:
    env = FakeTaskSpaceEnv()
    horizon = 2
    state = np.asarray(env.get_state(), dtype=float)
    actual_xy = runner.compute_end_effector_xy(env, state)
    external_targets = np.asarray(
        [
            actual_xy,
            actual_xy,
            actual_xy + np.asarray([1.0, 0.0], dtype=float),
        ],
        dtype=float,
    )

    problem = runner.build_task_space_tracking_problem(
        env=env,
        config=_config(),
        horizon=horizon,
        solver_family="cem",
        target_ee_positions=external_targets,
        sampling_overrides={"num_candidates": 1, "num_iterations": 1, "sampling_std": 0.0, "torque_limit": 1.0, "seed": 1},
        stage_cost_timing="post_step",
    )
    rollout = runner.evaluate_task_space_rollouts(np.zeros((1, horizon, 2), dtype=float), problem)

    assert rollout.costs[0] == pytest.approx(0.0)


def test_legacy_ladder_cost_profile_uses_b03_weights_without_extra_terminal_velocity() -> None:
    env = FakeTaskSpaceEnv()
    config = _config()
    config["task_space"] = {
        "ee_weight": 1000.0,
        "terminal_ee_weight": 1000.0,
        "velocity_weight": 1000.0,
        "control_weight": 1000.0,
    }
    config["cost"] = {
        "ee_weight": 2.0,
        "dq_weight": 3.0,
        "torque_weight": 5.0,
        "terminal_weight": 7.0,
    }
    state = np.asarray(env.get_state(), dtype=float)
    state[2:] = np.asarray([0.1, -0.2], dtype=float)
    actual_xy = runner.compute_end_effector_xy(env, state)
    targets = np.repeat(actual_xy[None, :], 3, axis=0)

    stage_cost_fn, terminal_cost_fn, _ = runner.build_task_space_cost_functions(
        env=env,
        target_ee_positions=targets,
        config=config,
        cost_profile="legacy_ladder",
    )
    control = np.asarray([0.3, -0.4], dtype=float)

    expected_stage = 3.0 * float(np.sum(state[2:] ** 2)) + 5.0 * float(np.sum(control**2))
    assert stage_cost_fn(state, control, 0) == pytest.approx(expected_stage)
    assert terminal_cost_fn(state) == pytest.approx(0.0)


def test_legacy_ladder_sampling_problem_uses_b02_adapter_rollout_backend() -> None:
    env = FakeB02RolloutEnv()
    horizon = 2
    state = np.asarray(env.get_state(), dtype=float)
    actual_xy = runner.compute_end_effector_xy(env, state)
    targets = np.repeat(actual_xy[None, :], horizon + 1, axis=0)

    problem = runner.build_task_space_tracking_problem(
        env=env,
        config=_config(),
        horizon=horizon,
        solver_family="cem",
        target_ee_positions=targets,
        sampling_overrides={"num_candidates": 1, "num_iterations": 1, "sampling_std": 0.0, "torque_limit": 1.0, "seed": 1},
        stage_cost_timing="post_step",
        cost_profile="legacy_ladder",
    )
    rollout = problem.rollout_cost_fn(np.zeros((1, horizon, 2), dtype=float), problem)

    assert problem.metadata["rollout_cost_backend"] == "b02_adapter_legacy_ladder"
    assert rollout.costs[0] == pytest.approx(0.0)


def test_run_task_space_warm_start_comparison_outputs_ee_metrics(tmp_path: Path) -> None:
    env = FakeTaskSpaceEnv()
    output_paths = runner.build_output_paths(tmp_path / "r4d_smoke")

    comparison = runner.run_task_space_warm_start_comparison(
        env=env,
        config=_config(),
        horizon=4,
        sampling_solver_family="cem",
        sampling_overrides={"num_candidates": 5, "num_iterations": 1, "sampling_std": 0.1, "torque_limit": 1.0, "seed": 3},
    )
    runner.write_task_space_outputs(comparison=comparison, output_paths=output_paths)

    assert comparison.sampling.solution.predicted_ee_positions is not None
    assert comparison.zero_init_ilqr.solution.predicted_ee_positions is not None
    assert comparison.warm_start_ilqr.solution.predicted_ee_positions is not None
    assert comparison.warm_start_ilqr.problem.metadata["warm_start_used"] is True
    assert comparison.warm_start_ilqr.problem.metadata["warm_start_source"] == "cem"

    assert output_paths["metrics_csv"].is_file()
    assert output_paths["cache_npz"].is_file()
    assert output_paths["report_md"].is_file()
    assert output_paths["ee_trajectory_figure"].is_file()
    assert output_paths["ee_error_figure"].is_file()
    assert output_paths["control_figure"].is_file()

    with output_paths["metrics_csv"].open("r", encoding="utf-8") as metrics_file:
        rows = list(csv.DictReader(metrics_file))
    assert [row["label"] for row in rows] == ["sampling_cem", "zero_init_ilqr", "sampling_warm_start_ilqr"]
    assert "final_ee_error" in rows[0]
    assert "mean_ee_error" in rows[0]
    assert "max_ee_error" in rows[0]

    cached = np.load(output_paths["cache_npz"])
    assert cached["target_ee_positions"].shape == (5, 2)
    assert cached["sampling_predicted_ee_positions"].shape == (5, 2)
    assert cached["warm_start_ilqr_predicted_ee_positions"].shape == (5, 2)

    report_text = output_paths["report_md"].read_text(encoding="utf-8")
    assert "B03-R4D Task-Space Warm-Start Smoke Report" in report_text
    assert "B03_R4D_ee_trajectory_xy.png" in report_text
    assert "B03_R4D_ee_tracking_error.png" in report_text
    assert "B03_R4D_control_sequence.png" in report_text
