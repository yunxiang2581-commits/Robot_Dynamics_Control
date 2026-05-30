from __future__ import annotations

from pathlib import Path
import sys
import csv

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from projects.B_mujoco_mpc_study.simulator.scripts import (
    run_B03_ilqr_lite_two_link_smoke as runner,
)
from projects.B_mujoco_mpc_study.simulator.planners.ilqg_solver import ILQGLiteSolver
from projects.B_mujoco_mpc_study.simulator.planners.mpc_solver_interface import (
    MPCProblem,
    MPCSolution,
    SolverStats,
)


def _fake_previous_solution(predicted_controls: np.ndarray) -> MPCSolution:
    return MPCSolution(
        first_control=np.asarray(predicted_controls[0], dtype=float),
        predicted_states=None,
        predicted_controls=np.asarray(predicted_controls, dtype=float),
        best_cost=1.0,
        solver_name="fake_sampling_solver",
        solver_stats=SolverStats(
            runtime_ms=0.0,
            num_rollouts=0,
            num_iterations=0,
            success=True,
            message="fake warm start source",
        ),
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
        return tuple(runner.get_two_link_state(self))


class RecordingSolver:
    def __init__(self) -> None:
        self.problem: MPCProblem | None = None
        self.previous_solution: MPCSolution | None = None

    def solve(
        self,
        problem: MPCProblem,
        previous_solution: MPCSolution | None = None,
    ) -> MPCSolution:
        self.problem = problem
        self.previous_solution = previous_solution
        controls = np.asarray(problem.metadata["initial_controls"], dtype=float)
        states = np.repeat(np.asarray(problem.current_state, dtype=float)[None, :], problem.horizon + 1, axis=0)
        return MPCSolution(
            first_control=controls[0],
            predicted_states=states,
            predicted_controls=controls,
            best_cost=0.0,
            solver_name="recording_solver",
            solver_stats=SolverStats(
                runtime_ms=0.0,
                num_rollouts=0,
                num_iterations=0,
                success=True,
                message="recorded",
            ),
            metadata={"cost_history": [0.0], "termination_reason": "recorded"},
        )


def test_two_link_smoke_skeleton_imports() -> None:
    assert runner.TASK_NAME == "B03_ilqr_lite_two_link_smoke"
    assert callable(runner.main)


def test_shape_validation_accepts_expected_shapes() -> None:
    x0 = np.zeros(4, dtype=float)
    u0 = np.zeros(2, dtype=float)
    x_refs = np.zeros((6, 4), dtype=float)
    u_nominal = np.zeros((5, 2), dtype=float)

    runner.validate_state_shape(x0)
    runner.validate_control_shape(u0)
    runner.validate_nominal_trajectory_shapes(x_refs, u_nominal)


def test_shape_validation_rejects_bad_shapes() -> None:
    with pytest.raises(ValueError):
        runner.validate_state_shape(np.zeros((1, 4), dtype=float))

    with pytest.raises(ValueError):
        runner.validate_control_shape(np.zeros(3, dtype=float))

    with pytest.raises(ValueError):
        runner.validate_nominal_trajectory_shapes(
            np.zeros((5, 4), dtype=float),
            np.zeros((5, 2), dtype=float),
        )


def test_ensure_output_dirs_creates_expected_subdirs() -> None:
    output_root = REPO_ROOT / "outputs" / "pytest_tmp" / "B03_R4C1A_skeleton"
    output_paths = runner.ensure_output_dirs(output_root)

    assert output_paths["cache"].is_dir()
    assert output_paths["figures"].is_dir()
    assert output_paths["reports"].is_dir()
    assert output_paths["metrics"].is_dir()
    assert output_paths["cache"] == output_root / "outputs" / "cache"
    assert output_paths["figures"] == output_root / "outputs" / "figures"
    assert output_paths["reports"] == output_root / "outputs" / "reports"
    assert output_paths["metrics"] == output_root / "outputs" / "metrics"


def test_build_two_link_dynamics_fn_rejects_env_without_step_api() -> None:
    with pytest.raises(NotImplementedError, match="step"):
        runner.build_two_link_dynamics_fn(env=object())


def test_build_warm_start_controls_from_previous_solution_shifts_controls() -> None:
    previous_controls = np.array(
        [
            [1.0, 2.0],
            [3.0, 4.0],
            [5.0, 6.0],
        ],
        dtype=float,
    )
    previous_solution = _fake_previous_solution(previous_controls)

    warm_start = runner.build_warm_start_controls_from_previous_solution(
        previous_solution=previous_solution,
        horizon=3,
        control_dim=2,
    )

    np.testing.assert_allclose(
        warm_start,
        np.array(
            [
                [3.0, 4.0],
                [5.0, 6.0],
                [5.0, 6.0],
            ],
            dtype=float,
        ),
    )


def test_build_warm_start_controls_returns_none_for_missing_previous_solution() -> None:
    assert runner.build_warm_start_controls_from_previous_solution(None, horizon=3, control_dim=2) is None


def test_run_smoke_simulation_passes_previous_solution_warm_start_to_solver() -> None:
    env = FakeStepEnv()
    x_ref = np.zeros((4, 4), dtype=float)
    previous_controls = np.array(
        [
            [1.0, 2.0],
            [3.0, 4.0],
            [5.0, 6.0],
        ],
        dtype=float,
    )
    previous_solution = _fake_previous_solution(previous_controls)
    solver = RecordingSolver()

    solution = runner.run_smoke_simulation(
        env=env,
        solver=solver,
        x_ref=x_ref,
        config={"simulation": {"dt": 0.05}},
        previous_solution=previous_solution,
    )

    expected_warm_start = np.array(
        [
            [3.0, 4.0],
            [5.0, 6.0],
            [5.0, 6.0],
        ],
        dtype=float,
    )
    assert solver.previous_solution is previous_solution
    assert solver.problem is not None
    np.testing.assert_allclose(solver.problem.metadata["initial_controls"], expected_warm_start)
    np.testing.assert_allclose(solution.predicted_controls, expected_warm_start)


def test_build_state_tracking_problem_returns_minimal_mpc_problem() -> None:
    x0 = np.zeros(4, dtype=float)
    x_refs = np.zeros((6, 4), dtype=float)
    u_nominal = np.ones((5, 2), dtype=float) * 0.25
    dynamics_fn = lambda x, u: np.asarray(x, dtype=float)
    config = {
        "simulation": {"dt": 0.02},
        "ilqg_lite": {
            "max_iterations": 3,
            "control_limit": 8.0,
            "line_search_alphas": [1.0, 0.5],
        },
        "cost": {
            "Q": [10.0, 10.0, 1.0, 1.0],
            "R": [0.1, 0.2],
            "Q_terminal": [20.0, 20.0, 2.0, 2.0],
        },
    }

    problem = runner.build_state_tracking_problem(
        env=object(),
        dynamics_fn=dynamics_fn,
        x0=x0,
        x_refs=x_refs,
        u_nominal=u_nominal,
        config=config,
    )

    assert problem.current_state.shape == (4,)
    assert problem.target_horizon.shape == (6, 4)
    assert problem.horizon == 5
    assert problem.control_dim == 2
    assert problem.dt == pytest.approx(0.02)
    assert problem.dynamics_fn is dynamics_fn
    assert problem.solver_config["max_iterations"] == 3
    assert problem.solver_config["control_limit"] == pytest.approx(8.0)
    np.testing.assert_allclose(problem.metadata["x_refs"], x_refs)
    np.testing.assert_allclose(problem.metadata["initial_controls"], u_nominal)
    np.testing.assert_allclose(problem.metadata["Q"], np.diag([10.0, 10.0, 1.0, 1.0]))
    np.testing.assert_allclose(problem.metadata["R"], np.diag([0.1, 0.2]))
    np.testing.assert_allclose(problem.metadata["Q_terminal"], np.diag([20.0, 20.0, 2.0, 2.0]))


def test_state_tracking_problem_can_be_consumed_by_ilqg_solver() -> None:
    x0 = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)
    horizon = 4
    x_refs = np.zeros((horizon + 1, 4), dtype=float)
    x_refs[:, 0] = 0.05
    u_nominal = np.zeros((horizon, 2), dtype=float)

    def simple_two_link_dynamics(x: np.ndarray, u: np.ndarray) -> np.ndarray:
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

    problem = runner.build_state_tracking_problem(
        env=object(),
        dynamics_fn=simple_two_link_dynamics,
        x0=x0,
        x_refs=x_refs,
        u_nominal=u_nominal,
        config={
            "simulation": {"dt": 0.05},
            "ilqg_lite": {
                "max_iterations": 2,
                "control_limit": 2.0,
                "line_search_alphas": [1.0, 0.5],
                "finite_difference_eps": 1.0e-6,
            },
            "cost": {
                "Q": [10.0, 10.0, 1.0, 1.0],
                "R": [0.1, 0.1],
                "Q_terminal": [20.0, 20.0, 2.0, 2.0],
            },
        },
    )

    solution = ILQGLiteSolver().solve(problem)

    assert solution.first_control.shape == (2,)
    assert solution.predicted_controls.shape == (horizon, 2)
    assert solution.predicted_states is not None
    assert solution.predicted_states.shape == (horizon + 1, 4)
    assert np.isfinite(solution.best_cost)


def test_write_smoke_outputs_writes_cost_history_and_metrics(tmp_path: Path) -> None:
    x0 = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)
    horizon = 3
    x_refs = np.zeros((horizon + 1, 4), dtype=float)
    x_refs[:, 0] = 0.02
    u_nominal = np.zeros((horizon, 2), dtype=float)

    def simple_two_link_dynamics(x: np.ndarray, u: np.ndarray) -> np.ndarray:
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

    problem = runner.build_state_tracking_problem(
        env=object(),
        dynamics_fn=simple_two_link_dynamics,
        x0=x0,
        x_refs=x_refs,
        u_nominal=u_nominal,
        config={
            "simulation": {"dt": 0.05},
            "ilqg_lite": {"max_iterations": 2, "control_limit": 2.0},
            "cost": {
                "Q": [10.0, 10.0, 1.0, 1.0],
                "R": [0.1, 0.1],
                "Q_terminal": [20.0, 20.0, 2.0, 2.0],
            },
        },
    )
    solution = ILQGLiteSolver().solve(problem)
    output_paths = runner.build_output_paths(tmp_path / "r4c_1e_outputs")

    runner.write_smoke_outputs(solution=solution, problem=problem, output_paths=output_paths)

    assert output_paths["cost_history_csv"].is_file()
    assert output_paths["metrics_csv"].is_file()
    assert output_paths["state_cache"].is_file()

    with output_paths["cost_history_csv"].open("r", encoding="utf-8") as cost_file:
        cost_rows = list(csv.DictReader(cost_file))
    assert len(cost_rows) == len(solution.metadata["cost_history"])
    assert set(cost_rows[0]) == {"iteration", "cost"}

    with output_paths["metrics_csv"].open("r", encoding="utf-8") as metrics_file:
        metrics_rows = list(csv.DictReader(metrics_file))
    assert len(metrics_rows) == 1
    metrics = metrics_rows[0]
    assert metrics["solver_name"] == "ilqg_lite"
    assert int(metrics["horizon"]) == horizon
    assert int(metrics["state_dim"]) == 4
    assert int(metrics["control_dim"]) == 2
    assert metrics["success"] in {"True", "False"}
    assert "warm_start_used" in metrics
    assert "warm_start_source" in metrics

    cached = np.load(output_paths["state_cache"])
    assert cached["predicted_states"].shape == (horizon + 1, 4)
    assert cached["predicted_controls"].shape == (horizon, 2)
    assert cached["x_refs"].shape == (horizon + 1, 4)
    assert "initial_controls" in cached


def test_plot_smoke_figures_writes_png_files(tmp_path: Path) -> None:
    x0 = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)
    horizon = 4
    x_refs = np.zeros((horizon + 1, 4), dtype=float)
    x_refs[:, 0] = np.linspace(0.0, 0.05, horizon + 1)
    u_nominal = np.zeros((horizon, 2), dtype=float)

    def simple_two_link_dynamics(x: np.ndarray, u: np.ndarray) -> np.ndarray:
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

    problem = runner.build_state_tracking_problem(
        env=object(),
        dynamics_fn=simple_two_link_dynamics,
        x0=x0,
        x_refs=x_refs,
        u_nominal=u_nominal,
        config={
            "simulation": {"dt": 0.05},
            "ilqg_lite": {"max_iterations": 2, "control_limit": 2.0},
            "cost": {
                "Q": [10.0, 10.0, 1.0, 1.0],
                "R": [0.1, 0.1],
                "Q_terminal": [20.0, 20.0, 2.0, 2.0],
            },
        },
    )
    solution = ILQGLiteSolver().solve(problem)
    output_paths = runner.build_output_paths(tmp_path / "r4c_1f_figures")

    runner.plot_trajectory(solution=solution, x_ref=x_refs, output_dir=output_paths["figures"])
    runner.plot_cost_history(solution=solution, output_dir=output_paths["figures"])

    assert output_paths["trajectory_figure"].is_file()
    assert output_paths["trajectory_figure"].stat().st_size > 0
    assert output_paths["cost_figure"].is_file()
    assert output_paths["cost_figure"].stat().st_size > 0


def test_write_smoke_report_summarizes_outputs(tmp_path: Path) -> None:
    x0 = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)
    horizon = 3
    x_refs = np.zeros((horizon + 1, 4), dtype=float)
    x_refs[:, 0] = 0.02
    u_nominal = np.zeros((horizon, 2), dtype=float)

    def simple_two_link_dynamics(x: np.ndarray, u: np.ndarray) -> np.ndarray:
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

    problem = runner.build_state_tracking_problem(
        env=object(),
        dynamics_fn=simple_two_link_dynamics,
        x0=x0,
        x_refs=x_refs,
        u_nominal=u_nominal,
        config={
            "simulation": {"dt": 0.05},
            "ilqg_lite": {"max_iterations": 2, "control_limit": 2.0},
            "cost": {
                "Q": [10.0, 10.0, 1.0, 1.0],
                "R": [0.1, 0.1],
                "Q_terminal": [20.0, 20.0, 2.0, 2.0],
            },
        },
    )
    solution = ILQGLiteSolver().solve(problem)
    output_paths = runner.build_output_paths(tmp_path / "r4c_1g_report")
    runner.write_smoke_outputs(solution=solution, problem=problem, output_paths=output_paths)
    runner.plot_trajectory(solution=solution, x_ref=x_refs, output_dir=output_paths["figures"])
    runner.plot_cost_history(solution=solution, output_dir=output_paths["figures"])

    runner.write_smoke_report(solution=solution, problem=problem, output_paths=output_paths)

    assert output_paths["smoke_report"].is_file()
    report_text = output_paths["smoke_report"].read_text(encoding="utf-8")
    assert "# B03-R4C iLQR Two-Link Smoke Report" in report_text
    assert "success" in report_text
    assert "horizon" in report_text
    assert "best_cost" in report_text
    assert "B03_R4C_ilqr_two_link_smoke_metrics.csv" in report_text
    assert "B03_R4C_two_link_state_trajectory_todo.png" in report_text
    assert "B03_R4C_ilqr_cost_history_todo.png" in report_text
