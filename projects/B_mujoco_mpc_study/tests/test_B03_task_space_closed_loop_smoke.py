from __future__ import annotations

from pathlib import Path
import csv
import sys

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[3]
SIMULATOR_ROOT = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "simulator"
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

from projects.B_mujoco_mpc_study.simulator.scripts import (  # noqa: E402
    run_B03_task_space_closed_loop_smoke as closed_loop,
)
from projects.B_mujoco_mpc_study.simulator.planners.mpc_solver_interface import (  # noqa: E402
    MPCProblem,
    MPCSolution,
    SolverStats,
)
from projects.B_mujoco_mpc_study.tests.test_B03_task_space_warm_start_smoke import (  # noqa: E402
    FakeTaskSpaceEnv,
)


def _config() -> dict[str, object]:
    return {
        "simulation": {"dt": 0.05, "num_steps": 6},
        "target": {
            "type": "lissajous",
            "center": [0.45, 0.10],
            "radius": 0.08,
            "frequency": 0.2,
            "radius_x": 0.10,
            "radius_y": 0.06,
            "frequency_x": 0.30,
            "frequency_y": 0.50,
            "phase_offset": 1.047,
        },
        "task_space": {
            "ee_weight": 80.0,
            "terminal_ee_weight": 120.0,
            "velocity_weight": 0.2,
            "control_weight": 0.05,
        },
        "ilqg_lite": {
            "max_iterations": 1,
            "control_limit": 1.0,
            "line_search_alphas": [1.0, 0.5],
            "finite_difference_eps": 1.0e-5,
            "regularization": 1.0e-5,
        },
        "sampling": {"horizon": 4, "num_candidates": 5, "sampling_std": 0.1, "torque_limit": 1.0, "seed": 2},
        "cem": {"num_candidates": 5, "num_iterations": 1, "initial_std": 0.1, "torque_limit": 1.0, "seed": 2},
    }


def test_build_target_horizon_uses_existing_b03_lissajous_config() -> None:
    config = _config()

    horizon_targets = closed_loop.build_closed_loop_target_horizon(
        config=config,
        target_source="b03_lissajous",
        current_time=0.0,
        horizon=4,
        dt=0.05,
    )

    assert horizon_targets.shape == (5, 2)
    assert not np.allclose(horizon_targets[0], horizon_targets[-1])


def test_build_target_horizon_can_start_from_next_control_time() -> None:
    config = _config()

    current_timing_targets = closed_loop.build_closed_loop_target_horizon(
        config=config,
        target_source="b03_lissajous",
        current_time=0.0,
        horizon=4,
        dt=0.05,
        start_offset_steps=0,
    )
    legacy_timing_targets = closed_loop.build_closed_loop_target_horizon(
        config=config,
        target_source="b03_lissajous",
        current_time=0.0,
        horizon=4,
        dt=0.05,
        start_offset_steps=1,
    )

    np.testing.assert_allclose(legacy_timing_targets[0], current_timing_targets[1])


def test_default_cli_uses_lightweight_smoke_values_instead_of_full_ladder_config() -> None:
    heavy_config = _config()
    heavy_config["simulation"] = {"dt": 0.05, "num_steps": 1000}
    heavy_config["sampling"] = {"horizon": 16, "num_candidates": 256, "sampling_std": 2.0, "torque_limit": 10.0, "seed": 2}
    heavy_config["cem"] = {"num_candidates": 256, "num_iterations": 3, "initial_std": 2.0, "torque_limit": 10.0, "seed": 2}
    args = closed_loop.build_arg_parser().parse_args([])

    assert closed_loop._resolve_num_steps(heavy_config, args.num_steps) == closed_loop.DEFAULT_SMOKE_NUM_STEPS
    assert closed_loop._resolve_horizon(heavy_config, args.horizon) == closed_loop.DEFAULT_SMOKE_HORIZON
    overrides = closed_loop._sampling_overrides_from_args(args)
    assert overrides["num_candidates"] == closed_loop.DEFAULT_SMOKE_NUM_CANDIDATES
    assert overrides["num_iterations"] == closed_loop.DEFAULT_SMOKE_SAMPLING_ITERATIONS


def test_sampling_overrides_can_set_legacy_cem_smoothing_alpha() -> None:
    args = closed_loop.build_arg_parser().parse_args(["--smoothing-alpha", "0.2"])

    overrides = closed_loop._sampling_overrides_from_args(args)

    assert overrides["smoothing_alpha"] == 0.2


def test_align_env_initial_ee_to_target_t0_sets_initial_q() -> None:
    config = _config()

    class B02GeometryFakeEnv(FakeTaskSpaceEnv):
        def get_end_effector_position(self) -> tuple[float, float]:
            q1, q2 = self.data.qpos
            return (
                float(0.45 * np.cos(q1) + 0.35 * np.cos(q1 + q2)),
                float(0.45 * np.sin(q1) + 0.35 * np.sin(q1 + q2)),
            )

    env = B02GeometryFakeEnv()
    target_t0 = closed_loop.build_closed_loop_target_horizon(
        config=config,
        target_source="b03_lissajous",
        current_time=0.0,
        horizon=4,
        dt=0.05,
    )[0]

    aligned_q = closed_loop.align_env_initial_ee_to_target_t0(env=env, target_t0=target_t0)
    actual_ee = np.asarray(env.get_end_effector_position(), dtype=float)

    assert aligned_q.shape == (2,)
    np.testing.assert_allclose(actual_ee, target_t0, atol=1.0e-9)
    np.testing.assert_allclose(env.data.qvel, np.zeros(2), atol=1.0e-12)


def test_reset_closed_loop_initial_state_supports_legacy_config_mode() -> None:
    env = FakeTaskSpaceEnv()
    config = _config()

    q = closed_loop.reset_closed_loop_initial_state(
        env=env,
        config=config,
        mode="config",
        target_t0=np.asarray([0.45, 0.10], dtype=float),
    )

    np.testing.assert_allclose(q, np.asarray([0.3, 0.4], dtype=float))
    np.testing.assert_allclose(env.data.qpos, np.asarray([0.3, 0.4], dtype=float))
    np.testing.assert_allclose(env.data.qvel, np.zeros(2), atol=1.0e-12)


def test_run_closed_loop_rollout_records_actual_target_and_controls(tmp_path: Path) -> None:
    env = FakeTaskSpaceEnv()
    output_paths = closed_loop.build_output_paths(tmp_path / "r4d6_closed_loop")

    result = closed_loop.run_closed_loop_task_space_rollout(
        env=env,
        config=_config(),
        target_source="b03_lissajous",
        solver_family="cem",
        num_steps=5,
        horizon=4,
        sampling_overrides={"num_candidates": 5, "num_iterations": 1, "sampling_std": 0.1, "torque_limit": 1.0, "seed": 4},
    )
    closed_loop.write_closed_loop_outputs(result=result, output_paths=output_paths)

    assert result.step_rows.shape[0] == 5
    assert result.actual_ee_positions.shape == (5, 2)
    assert result.target_ee_positions.shape == (5, 2)
    assert result.executed_controls.shape == (5, 2)
    assert result.predicted_ee_positions.shape == (5, 5, 2)
    assert np.isfinite(result.summary_metrics["mean_ee_error"])

    assert output_paths["step_metrics_csv"].is_file()
    assert output_paths["summary_metrics_csv"].is_file()
    assert output_paths["cache_npz"].is_file()
    assert output_paths["report_md"].is_file()
    assert output_paths["ee_trajectory_figure"].is_file()
    assert output_paths["ee_error_figure"].is_file()
    assert output_paths["control_figure"].is_file()
    assert output_paths["runtime_figure"].is_file()

    with output_paths["step_metrics_csv"].open("r", encoding="utf-8") as step_file:
        rows = list(csv.DictReader(step_file))
    assert len(rows) == 5
    assert "actual_x" in rows[0]
    assert "target_x" in rows[0]
    assert "ee_error" in rows[0]
    assert "runtime_ms" in rows[0]

    cached = np.load(output_paths["cache_npz"])
    assert cached["actual_ee_positions"].shape == (5, 2)
    assert cached["target_ee_positions"].shape == (5, 2)
    assert cached["predicted_ee_positions"].shape == (5, 5, 2)

    report_text = output_paths["report_md"].read_text(encoding="utf-8")
    assert "B03-R4D-6 Closed-Loop Task-Space MPC Smoke Report" in report_text
    assert "b03_lissajous" in report_text
    assert "B03_R4D6_closed_loop_ee_xy.png" in report_text


def test_run_closed_loop_rollout_supports_sampling_ladder_solvers() -> None:
    for solver_family in ("random_shooting", "warm_start_sampling", "mppi_lite"):
        env = FakeTaskSpaceEnv()
        result = closed_loop.run_closed_loop_task_space_rollout(
            env=env,
            config=_config(),
            target_source="b03_lissajous",
            solver_family=solver_family,
            num_steps=2,
            horizon=4,
            sampling_overrides={"num_candidates": 4, "num_iterations": 1, "sampling_std": 0.1, "torque_limit": 1.0, "seed": 4},
        )

        assert result.step_rows.shape[0] == 2
        assert result.solver_family == solver_family
        assert float(result.step_rows[0]["sampling_runtime_ms"]) >= 0.0
        assert float(result.step_rows[0]["sampling_num_rollouts"]) > 0.0


def test_closed_loop_can_reuse_previous_solution_for_all_sampling_solvers(monkeypatch) -> None:
    env = FakeTaskSpaceEnv()
    horizon = 4
    previous_was_none: list[bool] = []

    class FakeCEMSolver:
        solver_name = "cem"

        def solve(self, problem: MPCProblem, previous_solution: MPCSolution | None = None) -> MPCSolution:
            previous_was_none.append(previous_solution is None)
            assert problem.rollout_cost_fn is not None
            controls = np.zeros((horizon, 2), dtype=float)
            controls[:, 0] = 0.01 * len(previous_was_none)
            return MPCSolution(
                first_control=controls[0],
                predicted_states=np.zeros((horizon + 1, 4), dtype=float),
                predicted_controls=controls,
                best_cost=float(len(previous_was_none)),
                solver_name="cem",
                solver_stats=SolverStats(runtime_ms=1.0, num_rollouts=3, num_iterations=1, success=True),
                predicted_ee_positions=np.asarray(problem.target_horizon, dtype=float),
            )

    monkeypatch.setattr(closed_loop, "CEMShootingSolver", FakeCEMSolver)

    result = closed_loop.run_closed_loop_task_space_rollout(
        env=env,
        config=_config(),
        target_source="b03_lissajous",
        solver_family="cem",
        num_steps=2,
        horizon=horizon,
        sampling_overrides={"num_candidates": 3, "num_iterations": 1, "sampling_std": 0.1, "torque_limit": 1.0, "seed": 4},
        sampling_previous_solution_mode="all_sampling",
    )

    assert previous_was_none == [True, False]
    assert result.step_rows.shape[0] == 2


def test_closed_loop_records_applied_torque_after_env_clipping(monkeypatch) -> None:
    horizon = 4

    class ClippedTorqueEnv(FakeTaskSpaceEnv):
        def step(self, torque: tuple[float, float]) -> tuple[float, float, float, float]:
            clipped = np.clip(np.asarray(torque, dtype=float), -0.05, 0.05)
            return super().step((float(clipped[0]), float(clipped[1])))

        def get_last_applied_torque(self) -> tuple[float, float]:
            return self.last_applied_torque

    class FakeCEMSolver:
        solver_name = "cem"

        def solve(self, problem: MPCProblem, previous_solution: MPCSolution | None = None) -> MPCSolution:
            _ = previous_solution
            controls = np.zeros((horizon, 2), dtype=float)
            controls[0] = np.asarray([0.20, -0.20], dtype=float)
            return MPCSolution(
                first_control=controls[0],
                predicted_states=np.zeros((horizon + 1, 4), dtype=float),
                predicted_controls=controls,
                best_cost=1.0,
                solver_name="cem",
                solver_stats=SolverStats(runtime_ms=1.0, num_rollouts=3, num_iterations=1, success=True),
                predicted_ee_positions=np.asarray(problem.target_horizon, dtype=float),
            )

    monkeypatch.setattr(closed_loop, "CEMShootingSolver", FakeCEMSolver)

    result = closed_loop.run_closed_loop_task_space_rollout(
        env=ClippedTorqueEnv(),
        config=_config(),
        target_source="b03_lissajous",
        solver_family="cem",
        num_steps=1,
        horizon=horizon,
        sampling_overrides={"num_candidates": 3, "num_iterations": 1, "sampling_std": 0.1, "torque_limit": 1.0, "seed": 4},
    )

    np.testing.assert_allclose(result.executed_controls[0], np.asarray([0.05, -0.05], dtype=float))
    assert float(result.step_rows[0]["tau1"]) == 0.05
    assert float(result.step_rows[0]["tau2"]) == -0.05


def test_sampling_warm_start_ilqg_uses_current_sampling_solution_without_shift(monkeypatch) -> None:
    env = FakeTaskSpaceEnv()
    horizon = 4
    sampling_controls = np.array(
        [
            [0.20, -0.10],
            [0.30, -0.20],
            [0.40, -0.30],
            [0.50, -0.40],
        ],
        dtype=float,
    )
    calls: list[str] = []

    class FakeCEMSolver:
        solver_name = "cem"

        def solve(self, problem: MPCProblem, previous_solution: MPCSolution | None = None) -> MPCSolution:
            calls.append("sampling")
            assert previous_solution is None
            assert problem.rollout_cost_fn is not None
            return MPCSolution(
                first_control=sampling_controls[0],
                predicted_states=np.zeros((horizon + 1, 4), dtype=float),
                predicted_controls=sampling_controls,
                best_cost=12.0,
                solver_name="cem",
                solver_stats=SolverStats(runtime_ms=1.5, num_rollouts=7, num_iterations=1, success=True),
                predicted_ee_positions=np.asarray(problem.target_horizon, dtype=float),
            )

    class FakeILQGSolver:
        solver_name = "ilqg_lite"

        def solve(self, problem: MPCProblem, previous_solution: MPCSolution | None = None) -> MPCSolution:
            calls.append("ilqg")
            assert previous_solution is None
            assert problem.rollout_cost_fn is None
            assert problem.metadata["warm_start_used"] is True
            assert problem.metadata["warm_start_source"] == "cem"
            assert problem.metadata["warm_start_mode"] == "same_step_sampling_solution"
            np.testing.assert_allclose(problem.metadata["initial_controls"], sampling_controls)
            return MPCSolution(
                first_control=sampling_controls[0],
                predicted_states=np.zeros((horizon + 1, 4), dtype=float),
                predicted_controls=np.asarray(problem.metadata["initial_controls"], dtype=float),
                best_cost=3.0,
                solver_name="ilqg_lite",
                solver_stats=SolverStats(runtime_ms=2.5, num_rollouts=2, num_iterations=1, success=True),
                predicted_ee_positions=np.asarray(problem.target_horizon, dtype=float),
            )

    monkeypatch.setattr(closed_loop, "CEMShootingSolver", FakeCEMSolver)
    monkeypatch.setattr(closed_loop, "ILQGLiteSolver", FakeILQGSolver)

    result = closed_loop.run_closed_loop_task_space_rollout(
        env=env,
        config=_config(),
        target_source="b03_lissajous",
        solver_family="sampling_warm_start_ilqg",
        num_steps=1,
        horizon=horizon,
        sampling_overrides={"num_candidates": 7, "num_iterations": 1, "sampling_std": 0.1, "torque_limit": 1.0, "seed": 4},
    )

    assert calls == ["sampling", "ilqg"]
    assert result.solver_family == "sampling_warm_start_ilqg"
    assert float(result.step_rows[0]["sampling_runtime_ms"]) == 1.5
    assert float(result.step_rows[0]["ilqg_runtime_ms"]) == 2.5
    assert float(result.step_rows[0]["runtime_ms"]) == 4.0
    assert result.step_rows[0]["warm_start_source"] == "cem"
