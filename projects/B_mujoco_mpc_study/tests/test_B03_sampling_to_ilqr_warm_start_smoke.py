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

from projects.B_mujoco_mpc_study.simulator.planners.mpc_solver_interface import (  # noqa: E402
    MPCProblem,
    MPCSolution,
    SolverStats,
)
from projects.B_mujoco_mpc_study.simulator.planners.sampling_mpc_solvers import (  # noqa: E402
    shift_control_sequence,
)
from projects.B_mujoco_mpc_study.simulator.scripts import (  # noqa: E402
    run_B03_sampling_to_ilqr_warm_start_smoke as runner,
)


class FakeStepEnv:
    def __init__(self) -> None:
        self.data = type("FakeData", (), {})()
        self.data.qpos = np.array([0.0, 0.0], dtype=float)
        self.data.qvel = np.array([0.0, 0.0], dtype=float)
        self.data.ctrl = np.array([0.0, 0.0], dtype=float)
        self.data.time = 0.0
        self.model = object()
        self.dt = 0.05
        self.last_applied_torque = (0.0, 0.0)

    def forward(self) -> None:
        return None

    def step(self, torque: tuple[float, float]) -> tuple[float, float, float, float]:
        u = np.asarray(torque, dtype=float)
        self.data.ctrl[:] = u
        self.data.qpos[:] = self.data.qpos + self.dt * self.data.qvel
        self.data.qvel[:] = self.data.qvel + self.dt * u
        self.data.time += self.dt
        self.last_applied_torque = (float(u[0]), float(u[1]))
        return tuple(np.concatenate([self.data.qpos, self.data.qvel]))


def _simple_problem() -> MPCProblem:
    horizon = 3
    x_refs = np.zeros((horizon + 1, 4), dtype=float)
    x_refs[:, 0] = 0.05
    Q = np.diag([10.0, 10.0, 1.0, 1.0])
    R = np.diag([0.1, 0.1])
    Q_terminal = np.diag([20.0, 20.0, 2.0, 2.0])

    def dynamics_fn(x: np.ndarray, u: np.ndarray) -> np.ndarray:
        x_arr = np.asarray(x, dtype=float)
        u_arr = np.asarray(u, dtype=float)
        dt = 0.05
        return np.array(
            [
                x_arr[0] + dt * x_arr[2],
                x_arr[1] + dt * x_arr[3],
                x_arr[2] + dt * u_arr[0],
                x_arr[3] + dt * u_arr[1],
            ],
            dtype=float,
        )

    return MPCProblem(
        current_state=np.zeros(4, dtype=float),
        target_horizon=x_refs,
        horizon=horizon,
        control_dim=2,
        dt=0.05,
        cost_config={"Q": Q, "R": R, "Q_terminal": Q_terminal},
        solver_config={"num_candidates": 4, "sampling_std": 0.1, "torque_limit": 1.0, "seed": 0},
        dynamics_fn=dynamics_fn,
        metadata={
            "x_refs": x_refs,
            "Q": Q,
            "R": R,
            "Q_terminal": Q_terminal,
            "initial_controls": np.zeros((horizon, 2), dtype=float),
        },
    )


def _fake_solution(
    *,
    solver_name: str,
    predicted_controls: np.ndarray,
    predicted_states: np.ndarray,
    best_cost: float,
) -> MPCSolution:
    return MPCSolution(
        first_control=np.asarray(predicted_controls[0], dtype=float),
        predicted_states=np.asarray(predicted_states, dtype=float),
        predicted_controls=np.asarray(predicted_controls, dtype=float),
        best_cost=float(best_cost),
        solver_name=solver_name,
        solver_stats=SolverStats(
            runtime_ms=1.5,
            num_rollouts=7,
            num_iterations=2,
            success=True,
            message="fake solution",
        ),
        metadata={"cost_history": [best_cost * 2.0, best_cost], "termination_reason": "fake"},
    )


def test_state_tracking_rollout_cost_fn_returns_costs_and_predicted_states() -> None:
    problem = _simple_problem()
    candidate_controls = np.zeros((2, problem.horizon, problem.control_dim), dtype=float)
    candidate_controls[1, :, 0] = 0.2

    rollout = runner.evaluate_state_tracking_rollouts(candidate_controls, problem)

    assert rollout.costs.shape == (2,)
    assert rollout.predicted_states.shape == (2, problem.horizon + 1, 4)
    assert np.isfinite(rollout.costs).all()
    expected_initial_states = np.repeat(problem.current_state[None, :], 2, axis=0)
    np.testing.assert_allclose(rollout.predicted_states[:, 0, :], expected_initial_states)


def test_build_sampling_problem_adds_rollout_cost_fn_and_solver_config() -> None:
    env = FakeStepEnv()
    x_ref = np.zeros((4, 4), dtype=float)
    x_ref[:, 0] = 0.03
    config = {
        "simulation": {"dt": 0.05},
        "cost": {
            "Q": [10.0, 10.0, 1.0, 1.0],
            "R": [0.1, 0.1],
            "Q_terminal": [20.0, 20.0, 2.0, 2.0],
        },
    }

    problem = runner.build_sampling_state_tracking_problem(
        env=env,
        x_ref=x_ref,
        config=config,
        sampling_solver_family="cem",
        sampling_overrides={"num_candidates": 5, "num_iterations": 1, "sampling_std": 0.2, "torque_limit": 1.5, "seed": 3},
    )

    assert problem.rollout_cost_fn is runner.evaluate_state_tracking_rollouts
    assert problem.solver_config["num_candidates"] == 5
    assert problem.solver_config["num_iterations"] == 1
    assert problem.solver_config["sampling_std"] == pytest.approx(0.2)
    assert problem.solver_config["torque_limit"] == pytest.approx(1.5)
    assert problem.metadata["warm_start_used"] is False
    assert problem.metadata["warm_start_source"] == "zeros"


def test_run_sampling_to_ilqr_comparison_uses_sampling_solution_as_warm_start() -> None:
    env = FakeStepEnv()
    x_ref = np.zeros((4, 4), dtype=float)
    x_ref[:, 0] = 0.04
    config = {
        "simulation": {"dt": 0.05},
        "ilqg_lite": {
            "max_iterations": 1,
            "control_limit": 1.0,
            "line_search_alphas": [1.0, 0.5],
            "finite_difference_eps": 1.0e-6,
        },
        "cost": {
            "Q": [10.0, 10.0, 1.0, 1.0],
            "R": [0.1, 0.1],
            "Q_terminal": [20.0, 20.0, 2.0, 2.0],
        },
    }

    comparison = runner.run_sampling_to_ilqr_comparison(
        env=env,
        x_ref=x_ref,
        config=config,
        sampling_solver_family="cem",
        sampling_overrides={"num_candidates": 5, "num_iterations": 1, "sampling_std": 0.2, "torque_limit": 1.0, "seed": 1},
    )

    assert comparison.sampling.solution.solver_name == "cem"
    assert comparison.zero_init_ilqr.problem.metadata["warm_start_used"] is False
    assert comparison.warm_start_ilqr.problem.metadata["warm_start_used"] is True
    assert comparison.warm_start_ilqr.problem.metadata["warm_start_source"] == "cem"
    expected_initial_controls = shift_control_sequence(comparison.sampling.solution.predicted_controls)
    np.testing.assert_allclose(
        comparison.warm_start_ilqr.problem.metadata["initial_controls"],
        expected_initial_controls,
    )


def test_write_comparison_outputs_writes_csv_npz_and_report(tmp_path: Path) -> None:
    problem = _simple_problem()
    output_paths = runner.build_output_paths(tmp_path / "r4c_2b_outputs")
    predicted_states = np.zeros((problem.horizon + 1, 4), dtype=float)
    controls = np.zeros((problem.horizon, 2), dtype=float)

    sampling = runner.SolverRunResult(
        label="sampling_cem",
        problem=problem,
        solution=_fake_solution(
            solver_name="cem",
            predicted_controls=controls + 0.1,
            predicted_states=predicted_states,
            best_cost=3.0,
        ),
    )
    zero = runner.SolverRunResult(
        label="zero_init_ilqr",
        problem=problem,
        solution=_fake_solution(
            solver_name="ilqg_lite",
            predicted_controls=controls,
            predicted_states=predicted_states,
            best_cost=2.0,
        ),
    )
    warm_problem = runner.replace_problem_metadata(
        problem,
        {"warm_start_used": True, "warm_start_source": "cem", "initial_controls": controls + 0.1},
    )
    warm = runner.SolverRunResult(
        label="sampling_warm_start_ilqr",
        problem=warm_problem,
        solution=_fake_solution(
            solver_name="ilqg_lite",
            predicted_controls=controls + 0.05,
            predicted_states=predicted_states,
            best_cost=1.0,
        ),
    )
    comparison = runner.ComparisonResult(sampling=sampling, zero_init_ilqr=zero, warm_start_ilqr=warm)

    runner.write_comparison_outputs(comparison=comparison, output_paths=output_paths)

    assert output_paths["comparison_metrics_csv"].is_file()
    assert output_paths["comparison_cache"].is_file()
    assert output_paths["comparison_report"].is_file()
    assert output_paths["comparison_metrics_figure"].is_file()
    assert output_paths["trajectory_comparison_figure"].is_file()
    assert output_paths["control_comparison_figure"].is_file()
    assert output_paths["comparison_metrics_figure"].stat().st_size > 0
    assert output_paths["trajectory_comparison_figure"].stat().st_size > 0
    assert output_paths["control_comparison_figure"].stat().st_size > 0

    with output_paths["comparison_metrics_csv"].open("r", encoding="utf-8") as metrics_file:
        rows = list(csv.DictReader(metrics_file))
    assert [row["label"] for row in rows] == ["sampling_cem", "zero_init_ilqr", "sampling_warm_start_ilqr"]
    assert "best_cost" in rows[0]
    assert "initial_cost" in rows[0]
    assert "final_cost" in rows[0]
    assert "runtime_ms" in rows[0]
    assert "num_rollouts" in rows[0]
    assert "num_iterations" in rows[0]
    assert "final_state_error_norm" in rows[0]
    assert rows[2]["warm_start_used"] == "True"
    assert rows[2]["warm_start_source"] == "cem"

    cached = np.load(output_paths["comparison_cache"])
    assert cached["sampling_predicted_controls"].shape == (problem.horizon, 2)
    assert cached["zero_init_ilqr_predicted_controls"].shape == (problem.horizon, 2)
    assert cached["warm_start_ilqr_initial_controls"].shape == (problem.horizon, 2)

    report_text = output_paths["comparison_report"].read_text(encoding="utf-8")
    assert "# B03-R4C-2B Sampling to iLQR Warm-Start Smoke Report" in report_text
    assert "sampling_cem" in report_text
    assert "zero_init_ilqr" in report_text
    assert "sampling_warm_start_ilqr" in report_text
    assert "B03_R4C2B_cost_runtime_comparison.png" in report_text
    assert "B03_R4C2B_state_trajectory_comparison.png" in report_text
    assert "B03_R4C2B_control_sequence_comparison.png" in report_text
