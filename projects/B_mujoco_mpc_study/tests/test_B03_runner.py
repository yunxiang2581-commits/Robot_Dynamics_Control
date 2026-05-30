from __future__ import annotations

from pathlib import Path
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from projects.B_mujoco_mpc_study.simulator.scripts import run_B03_mpc_solver_ladder_demo as runner
from projects.B_mujoco_mpc_study.simulator.planners.mpc_solver_interface import MPCProblem, MPCSolution, SolverStats


def test_build_solver_outputs_groups_paths_by_solver_name() -> None:
    run_dir = Path("demo_run")
    outputs = runner.build_solver_outputs(run_dir, "random_shooting")

    assert outputs["solver_dir"] == run_dir / "solvers" / "random_shooting"
    assert outputs["step_log"].parent == run_dir / "solvers" / "random_shooting" / "logs"
    assert outputs["summary_csv"].parent == run_dir / "solvers" / "random_shooting" / "metrics"
    assert outputs["error_figure"].parent == run_dir / "solvers" / "random_shooting" / "figures"
    assert outputs["raw_video"].parent == run_dir / "solvers" / "random_shooting" / "videos"
    assert outputs["raw_video"].name == "B03_random_shooting_raw.mp4"
    assert outputs["marked_video"].name == "B03_random_shooting_marked.mp4"
    assert outputs["tracking_log"].name == "B03_random_shooting_tracking_log.csv"
    assert outputs["metrics_csv"].name == "B03_random_shooting_metrics.csv"
    assert outputs["run_log"].name == "B03_random_shooting_run_log.txt"
    assert outputs["error_figure"].name == "B03_random_shooting_error_curve.png"
    assert outputs["ee_trajectory_xy"].name == "B03_random_shooting_ee_trajectory_xy.png"
    assert outputs["ee_tracking_error"].name == "B03_random_shooting_ee_tracking_error.png"
    assert outputs["joint_torque"].name == "B03_random_shooting_joint_torque.png"


def test_run_b03_solver_ladder_skeleton_returns_solver_grouped_outputs(monkeypatch: pytest.MonkeyPatch) -> None:
    class FakeEnv:
        def __init__(self, *_: object, **__: object) -> None:
            self.state = (0.1, 0.2, 0.0, 0.0)
            self.last_applied_torque = (0.0, 0.0)
            self.frame_value = 0

        def reset(self, q: tuple[float, float], dq: tuple[float, float]) -> tuple[float, float, float, float]:
            self.state = (float(q[0]), float(q[1]), float(dq[0]), float(dq[1]))
            return self.state

        def get_state(self) -> tuple[float, float, float, float]:
            return self.state

        def step(self, torque: tuple[float, float] | list[float]) -> tuple[float, float, float, float]:
            tau1 = float(torque[0])
            tau2 = float(torque[1])
            self.last_applied_torque = (tau1, tau2)
            self.state = (
                self.state[0] + 0.01 * tau1,
                self.state[1] + 0.01 * tau2,
                0.1 * tau1,
                0.1 * tau2,
            )
            return self.state

        def get_last_applied_torque(self) -> tuple[float, float]:
            return self.last_applied_torque

        def get_end_effector_position(self) -> tuple[float, float]:
            q1, q2, _, _ = self.state
            return q1 + q2, q1 - q2

        def render_frame(self) -> runner.np.ndarray:
            self.frame_value += 1
            return runner.np.full((8, 8, 3), self.frame_value, dtype=runner.np.uint8)

        def render_frame_with_scene_geoms(self, scene_geoms: list[dict[str, object]]) -> runner.np.ndarray:
            _ = scene_geoms
            return self.render_frame()

    class FakeSolver:
        def __init__(self, solver_name: str) -> None:
            self.solver_name = solver_name

        def solve(
            self,
            problem: MPCProblem,
            previous_solution: MPCSolution | None = None,
        ) -> MPCSolution:
            _ = previous_solution
            control = {
                "random_shooting": (0.1, 0.0),
                "warm_start_sampling": (0.0, 0.2),
            }[self.solver_name]
            return MPCSolution(
                first_control=runner.np.asarray(control, dtype=float),
                predicted_states=runner.np.zeros((problem.horizon + 1, 4), dtype=float),
                predicted_controls=runner.np.tile(runner.np.asarray(control, dtype=float), (problem.horizon, 1)),
                best_cost=1.0 if self.solver_name == "random_shooting" else 0.8,
                solver_name=self.solver_name,
                solver_stats=SolverStats(
                    runtime_ms=0.5,
                    num_rollouts=4,
                    num_iterations=1,
                    success=True,
                    message="ok",
                ),
                predicted_ee_positions=runner.np.zeros((problem.horizon + 1, 2), dtype=float),
                selected_index=0,
                metadata={"cost_std": 0.1},
            )

    monkeypatch.setattr(runner, "load_b03_config", lambda path: {"loaded_from": str(path)})
    monkeypatch.setattr(
        runner,
        "resolve_b03_runtime_config",
        lambda config, args: {
            "model_family": "two_link",
            "num_steps": 2,
            "dt": 0.01,
            "seed": 0,
            "horizon": 3,
            "control_dim": 2,
            "torque_limit": 1.0,
            "target_type": "circle",
            "target_center": (0.4, 0.2),
            "target_radius": 0.05,
                "target_frequency": 0.2,
                "enabled_solvers": ["random_shooting", "warm_start_sampling"],
                "export_video": True,
                "save_figures": True,
                "save_metrics": True,
                "cost": {"ee_weight": 1.0, "dq_weight": 0.1, "torque_weight": 0.01, "terminal_weight": 1.0},
                "solver_configs": {
                "random_shooting": {"num_candidates": 4, "sampling_std": 0.3, "torque_limit": 1.0, "seed": 0},
                "warm_start_sampling": {"num_candidates": 4, "sampling_std": 0.3, "torque_limit": 1.0, "seed": 0},
            },
        },
    )
    monkeypatch.setattr(runner, "create_b03_two_link_env", lambda runtime_config: FakeEnv())
    monkeypatch.setattr(
        runner,
        "build_b03_solver",
        lambda solver_name, runtime_config, env: FakeSolver(solver_name),
    )
    monkeypatch.setattr(
        runner,
        "build_target_sequence",
        lambda **kwargs: [(0.4, 0.2)] * kwargs["horizon"],
    )

    written_paths: list[Path] = []

    def fake_save_video(frames: list[object], output_path: Path, fps: int) -> None:
        assert frames
        assert fps > 0
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_bytes(b"fake_video")
        written_paths.append(output_path)

    def fake_render_marked_video(
        raw_frames: list[object],
        tracking_rows: list[object],
        output_path: Path,
        fps: int,
        **kwargs: object,
    ) -> None:
        _ = kwargs
        assert raw_frames
        assert tracking_rows
        assert fps > 0
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_bytes(b"fake_marked_video")
        written_paths.append(output_path)

    monkeypatch.setattr(runner, "save_video", fake_save_video)
    monkeypatch.setattr(runner, "render_marked_video", fake_render_marked_video)

    args = runner.build_arg_parser().parse_args(
        [
            "--config",
            str(runner.DEFAULT_CONFIG_PATH),
            "--run-id",
            "pytest_b03_runner",
            "--num-steps",
            "2",
            "--horizon",
            "3",
            "--num-candidates",
            "4",
            "--solvers",
            "random_shooting",
            "warm_start_sampling",
            "--save-figures",
            "--save-metrics",
            "--export-video",
            "--no-show-viewer",
        ]
    )

    result = runner.run_b03_solver_ladder_skeleton(args)

    assert result["status"].startswith("B03 solver ladder comparison skeleton completed")
    assert set(result["solver_results"].keys()) == {"random_shooting", "warm_start_sampling"}
    assert result["solver_outputs"]["random_shooting"]["solver_dir"].name == "random_shooting"
    assert result["solver_outputs"]["warm_start_sampling"]["solver_dir"].name == "warm_start_sampling"
    assert len(result["solver_results"]["random_shooting"]["step_rows"]) == 2
    assert len(result["solver_results"]["warm_start_sampling"]["step_rows"]) == 2
    assert len(result["summary_rows"]) == 2
    assert result["comparison_outputs"]["error_curve"].exists()
    assert result["comparison_outputs"]["runtime_curve"].exists()
    assert result["comparison_outputs"]["cost_curve"].exists()
    assert result["comparison_outputs"]["control_smoothness_curve"].exists()
    assert result["comparison_outputs"]["trajectory_smoothness_curve"].exists()
    assert result["solver_outputs"]["random_shooting"]["tracking_log"].exists()
    assert result["solver_outputs"]["random_shooting"]["metrics_csv"].exists()
    assert result["solver_outputs"]["random_shooting"]["run_log"].exists()
    assert result["solver_outputs"]["random_shooting"]["raw_video"].exists()
    assert result["solver_outputs"]["random_shooting"]["marked_video"].exists()
    assert result["solver_outputs"]["random_shooting"]["error_figure"].exists()
    assert result["solver_outputs"]["random_shooting"]["ee_trajectory_xy"].exists()
    assert result["solver_outputs"]["random_shooting"]["ee_tracking_error"].exists()
    assert result["solver_outputs"]["random_shooting"]["joint_torque"].exists()
    assert result["solver_outputs"]["warm_start_sampling"]["tracking_log"].exists()
    assert result["solver_outputs"]["warm_start_sampling"]["metrics_csv"].exists()
    assert result["solver_outputs"]["warm_start_sampling"]["run_log"].exists()
    assert result["solver_outputs"]["warm_start_sampling"]["raw_video"].exists()
    assert result["solver_outputs"]["warm_start_sampling"]["marked_video"].exists()
    assert result["solver_outputs"]["warm_start_sampling"]["error_figure"].exists()
    assert result["solver_outputs"]["warm_start_sampling"]["ee_trajectory_xy"].exists()
    assert result["solver_outputs"]["warm_start_sampling"]["ee_tracking_error"].exists()
    assert result["solver_outputs"]["warm_start_sampling"]["joint_torque"].exists()
    assert any(path.name == "B03_random_shooting_raw.mp4" for path in written_paths)
    assert any(path.name == "B03_random_shooting_marked.mp4" for path in written_paths)
