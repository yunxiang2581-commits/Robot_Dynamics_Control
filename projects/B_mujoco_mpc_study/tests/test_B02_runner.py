from __future__ import annotations

from pathlib import Path
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from projects.B_mujoco_mpc_study.simulator.scripts import run_B02_two_link_mpc_tracking_demo as runner


def test_build_target_sequence_creates_circle_points() -> None:
    targets = runner.build_target_sequence(
        target_type="circle",
        current_time=0.0,
        horizon=3,
        dt=0.1,
        center=(0.5, 0.2),
        radius=0.1,
        angular_speed=0.0,
    )

    assert targets == [(0.6, 0.2), (0.6, 0.2), (0.6, 0.2)]


def test_build_target_sequence_creates_figure8_points() -> None:
    targets = runner.build_target_sequence(
        target_type="figure8",
        current_time=0.0,
        horizon=2,
        dt=0.25,
        center=(0.45, 0.2),
        radius=0.08,
        angular_speed=1.0,
        x_amplitude=0.2,
        y_amplitude=0.1,
    )

    assert targets[0] == pytest.approx(
        (
            0.45 + 0.2 * runner.math.sin(0.25),
            0.2 + 0.1 * runner.math.sin(0.5),
        )
    )
    assert targets[1] == pytest.approx(
        (
            0.45 + 0.2 * runner.math.sin(0.5),
            0.2 + 0.1 * runner.math.sin(1.0),
        )
    )


def test_build_target_sequence_creates_fixed_points() -> None:
    targets = runner.build_target_sequence(
        target_type="fixed",
        current_time=0.0,
        horizon=3,
        dt=0.1,
        center=(0.5, 0.2),
        radius=0.1,
        angular_speed=1.0,
        fixed_target=(0.42, 0.18),
    )

    assert targets == [(0.42, 0.18), (0.42, 0.18), (0.42, 0.18)]


def test_build_target_sequence_creates_sinusoidal_points() -> None:
    targets = runner.build_target_sequence(
        target_type="sinusoidal",
        current_time=0.0,
        horizon=2,
        dt=0.25,
        center=(0.45, 0.2),
        radius=0.08,
        angular_speed=1.0,
        x_amplitude=0.2,
        y_amplitude=0.1,
    )

    assert targets[0] == pytest.approx(
        (
            0.45 + 0.2 * runner.math.sin(0.25),
            0.2 + 0.1 * runner.math.cos(0.25),
        )
    )
    assert targets[1] == pytest.approx(
        (
            0.45 + 0.2 * runner.math.sin(0.5),
            0.2 + 0.1 * runner.math.cos(0.5),
        )
    )


def test_build_target_sequence_uses_custom_points_by_step_index() -> None:
    targets = runner.build_target_sequence(
        target_type="custom",
        current_time=0.1,
        horizon=3,
        dt=0.1,
        center=(0.0, 0.0),
        radius=0.0,
        angular_speed=0.0,
        custom_trajectory=[(0.1, 0.1), (0.2, 0.2), (0.3, 0.3)],
    )

    assert targets == [(0.2, 0.2), (0.3, 0.3), (0.3, 0.3)]


def test_compute_summary_metrics_reports_ee_tracking_values() -> None:
    metrics = runner.compute_summary_metrics(
        ee_history=[(0.5, 0.2), (0.6, 0.2)],
        target_history=[(0.6, 0.2), (0.6, 0.2)],
        commanded_torque_history=[(0.8, -0.3), (-0.4, 0.1)],
        applied_torque_history=[(0.2, -0.3), (-0.4, 0.1)],
        runtime_history=[0.01, 0.03],
    )

    assert metrics["final_ee_error"] == 0.0
    assert metrics["mean_ee_error"] == pytest.approx(0.05)
    assert metrics["max_abs_torque"] == 0.4
    assert metrics["max_abs_applied_torque"] == 0.4
    assert metrics["max_abs_command_torque"] == 0.8
    assert metrics["runtime_per_control_step"] == 0.02


def test_build_run_outputs_includes_tracking_log_and_marked_video() -> None:
    run_dir = Path("demo_run")
    outputs = runner.build_run_outputs(run_dir)

    assert outputs["tracking_log"].name == "B02_tracking_log.csv"
    assert outputs["marked_video"].name == "B02_two_link_mpc_tracking_marked.mp4"


def test_build_run_outputs_includes_independent_postprocess_figures() -> None:
    outputs = runner.build_run_outputs(Path("demo_run"))

    assert outputs["xy_target_vs_actual"].name == "B02_xy_target_vs_actual.png"
    assert outputs["tracking_error_time"].name == "B02_tracking_error_time.png"
    assert outputs["control_input_time"].name == "B02_control_input_time.png"


def test_resolve_run_args_supports_lightweight_regression_config() -> None:
    regression_config = (
        REPO_ROOT
        / "projects"
        / "B_mujoco_mpc_study"
        / "configs"
        / "B02_two_link_mpc_regression.yaml"
    )

    args = runner.resolve_run_args(
        runner.build_arg_parser().parse_args(
            [
                "--config",
                str(regression_config),
                "--run-id",
                "pytest_b02_regression_config",
                "--no-show-viewer",
            ]
        )
    )

    assert args.num_steps == 100
    assert args.horizon == 8
    assert args.num_candidates == 64
    assert args.export_video is True
    assert args.save_figures is True
    assert args.save_metrics is True
    assert args.show_viewer is False


def test_build_run_outputs_keeps_regression_and_benchmark_on_same_artifact_schema() -> None:
    outputs = runner.build_run_outputs(Path("regression_run"))

    assert outputs["video"].name == "B02_two_link_mpc_tracking_demo.mp4"
    assert outputs["marked_video"].name == "B02_two_link_mpc_tracking_marked.mp4"
    assert outputs["tracking_log"].name == "B02_tracking_log.csv"
    assert outputs["metrics"].name == "B02_metrics.csv"


def test_run_closed_loop_rejects_torque_limit_above_actuator_limit(monkeypatch: pytest.MonkeyPatch) -> None:
    class FakeEnv:
        def __init__(self, model_path: str, dt: float, end_effector_site: str) -> None:
            self.model = type("FakeModel", (), {"actuator_ctrlrange": [(-10.0, 10.0), (-10.0, 10.0)]})()

        def reset(self, q: tuple[float, float], dq: tuple[float, float]) -> tuple[float, float, float, float]:
            return q[0], q[1], dq[0], dq[1]

    class FakePlanner:
        def __init__(self, **_: object) -> None:
            pass

    class FakeController:
        def __init__(self, planner: object) -> None:
            self.planner = planner

    monkeypatch.setattr(
        runner,
        "import_b02_components",
        lambda: (FakeEnv, FakePlanner, FakeController),
    )

    args = runner.resolve_run_args(
        runner.build_arg_parser().parse_args(
            [
                "--run-id",
                "pytest_b02_invalid_torque_limit",
                "--num-steps",
                "1",
                "--horizon",
                "1",
                "--num-candidates",
                "1",
                "--torque-limit",
                "20",
                "--no-export-video",
                "--no-save-figures",
                "--no-save-metrics",
                "--no-show-viewer",
            ]
        )
    )

    with pytest.raises(ValueError, match="torque_limit"):
        runner.run_closed_loop(args)


def test_run_closed_loop_exports_marked_video_when_video_enabled(monkeypatch: pytest.MonkeyPatch) -> None:
    class FakeEnv:
        def __init__(self, model_path: str, dt: float, end_effector_site: str) -> None:
            self.model = type("FakeModel", (), {"actuator_ctrlrange": [(-10.0, 10.0), (-10.0, 10.0)]})()

        def reset(self, q: tuple[float, float], dq: tuple[float, float]) -> tuple[float, float, float, float]:
            return q[0], q[1], dq[0], dq[1]

        def step(self, torque: tuple[float, float]) -> tuple[float, float, float, float]:
            return 0.1, 0.2, 0.0, 0.0

        def get_last_applied_torque(self) -> tuple[float, float]:
            return 0.1, -0.1

        def get_end_effector_position(self) -> tuple[float, float]:
            return 0.5, 0.25

        def render_frame(self) -> object:
            return object()

    class FakePlanner:
        def __init__(self, **_: object) -> None:
            pass

    class FakeController:
        def __init__(self, planner: object) -> None:
            self.planner = planner
            self.last_plan = {"best_cost": 1.0}

        def compute_control(
            self,
            env: object,
            current_state: tuple[float, float, float, float],
            target_sequence: list[tuple[float, float]],
        ) -> tuple[float, float]:
            return 0.1, -0.1

    marked_calls: dict[str, object] = {}

    monkeypatch.setattr(
        runner,
        "import_b02_components",
        lambda: (FakeEnv, FakePlanner, FakeController),
    )
    monkeypatch.setattr(runner, "save_video", lambda frames, output_path, fps: None)
    monkeypatch.setattr(
        runner,
        "render_marked_video",
        lambda raw_frames, tracking_rows, output_path, fps: marked_calls.update(
            {
                "num_frames": len(raw_frames),
                "num_rows": len(tracking_rows),
                "output_name": output_path.name,
                "fps": fps,
            }
        ),
    )

    args = runner.resolve_run_args(
        runner.build_arg_parser().parse_args(
            [
                "--run-id",
                "pytest_b02_marked_video",
                "--num-steps",
                "2",
                "--horizon",
                "1",
                "--num-candidates",
                "1",
                "--export-video",
                "--no-save-figures",
                "--no-save-metrics",
                "--no-show-viewer",
            ]
        )
    )

    runner.run_closed_loop(args)

    assert marked_calls["num_frames"] == 2
    assert marked_calls["num_rows"] == 2
    assert marked_calls["output_name"] == "B02_two_link_mpc_tracking_marked.mp4"


def test_run_closed_loop_uses_independent_plotting_when_figures_enabled(monkeypatch: pytest.MonkeyPatch) -> None:
    class FakeEnv:
        def __init__(self, model_path: str, dt: float, end_effector_site: str) -> None:
            self.model = type("FakeModel", (), {"actuator_ctrlrange": [(-10.0, 10.0), (-10.0, 10.0)]})()

        def reset(self, q: tuple[float, float], dq: tuple[float, float]) -> tuple[float, float, float, float]:
            return q[0], q[1], dq[0], dq[1]

        def step(self, torque: tuple[float, float]) -> tuple[float, float, float, float]:
            return 0.1, 0.2, 0.0, 0.0

        def get_last_applied_torque(self) -> tuple[float, float]:
            return 0.1, -0.1

        def get_end_effector_position(self) -> tuple[float, float]:
            return 0.5, 0.25

    class FakePlanner:
        def __init__(self, **_: object) -> None:
            pass

    class FakeController:
        def __init__(self, planner: object) -> None:
            self.planner = planner
            self.last_plan = {"best_cost": 1.0}

        def compute_control(
            self,
            env: object,
            current_state: tuple[float, float, float, float],
            target_sequence: list[tuple[float, float]],
        ) -> tuple[float, float]:
            return 0.1, -0.1

    plotting_calls: dict[str, object] = {}

    monkeypatch.setattr(
        runner,
        "import_b02_components",
        lambda: (FakeEnv, FakePlanner, FakeController),
    )
    monkeypatch.setattr(
        runner,
        "save_b02_tracking_figures",
        lambda tracking_rows, outputs: plotting_calls.update(
            {
                "num_rows": len(tracking_rows),
                "xy_name": outputs["xy_target_vs_actual"].name,
                "error_name": outputs["tracking_error_time"].name,
                "control_name": outputs["control_input_time"].name,
            }
        ),
    )

    args = runner.resolve_run_args(
        runner.build_arg_parser().parse_args(
            [
                "--run-id",
                "pytest_b02_independent_plotting",
                "--num-steps",
                "2",
                "--horizon",
                "1",
                "--num-candidates",
                "1",
                "--save-figures",
                "--no-save-metrics",
                "--no-export-video",
                "--no-show-viewer",
            ]
        )
    )

    runner.run_closed_loop(args)

    assert plotting_calls["num_rows"] == 2
    assert plotting_calls["xy_name"] == "B02_xy_target_vs_actual.png"
    assert plotting_calls["error_name"] == "B02_tracking_error_time.png"
    assert plotting_calls["control_name"] == "B02_control_input_time.png"


def test_run_closed_loop_passes_hybrid_mode_to_marked_video_renderer(monkeypatch: pytest.MonkeyPatch) -> None:
    class FakeEnv:
        def __init__(self, model_path: str, dt: float, end_effector_site: str) -> None:
            self.model = type("FakeModel", (), {"actuator_ctrlrange": [(-10.0, 10.0), (-10.0, 10.0)]})()

        def reset(self, q: tuple[float, float], dq: tuple[float, float]) -> tuple[float, float, float, float]:
            return q[0], q[1], dq[0], dq[1]

        def step(self, torque: tuple[float, float]) -> tuple[float, float, float, float]:
            return 0.1, 0.2, 0.0, 0.0

        def get_last_applied_torque(self) -> tuple[float, float]:
            return 0.1, -0.1

        def get_end_effector_position(self) -> tuple[float, float]:
            return 0.5, 0.25

        def render_frame(self) -> object:
            return object()

        def render_frame_with_scene_geoms(self, scene_geoms: list[dict[str, object]]) -> object:
            return {"scene_geoms": len(scene_geoms)}

    class FakePlanner:
        def __init__(self, **_: object) -> None:
            pass

    class FakeController:
        def __init__(self, planner: object) -> None:
            self.planner = planner
            self.last_plan = {"best_cost": 1.0}

        def compute_control(
            self,
            env: object,
            current_state: tuple[float, float, float, float],
            target_sequence: list[tuple[float, float]],
        ) -> tuple[float, float]:
            return 0.1, -0.1

    marked_calls: dict[str, object] = {}

    monkeypatch.setattr(
        runner,
        "import_b02_components",
        lambda: (FakeEnv, FakePlanner, FakeController),
    )
    monkeypatch.setattr(runner, "save_video", lambda frames, output_path, fps: None)
    monkeypatch.setattr(
        runner,
        "render_marked_video",
        lambda raw_frames, tracking_rows, output_path, fps, **kwargs: marked_calls.update(
            {
                "render_mode": kwargs.get("render_mode"),
                "scene_marker_provider": kwargs.get("scene_marker_provider") is not None,
                "scene_frame_renderer": kwargs.get("scene_frame_renderer") is not None,
                "output_name": output_path.name,
            }
        ),
    )

    args = runner.resolve_run_args(
        runner.build_arg_parser().parse_args(
            [
                "--run-id",
                "pytest_b02_hybrid_mode",
                "--num-steps",
                "2",
                "--horizon",
                "1",
                "--num-candidates",
                "1",
                "--export-video",
                "--no-save-figures",
                "--no-save-metrics",
                "--no-show-viewer",
                "--render-mode",
                "hybrid",
            ]
        )
    )

    runner.run_closed_loop(args)

    assert marked_calls["render_mode"] == "hybrid"
    assert marked_calls["scene_marker_provider"] is True
    assert marked_calls["scene_frame_renderer"] is True
    assert marked_calls["output_name"] == "B02_two_link_mpc_tracking_marked.mp4"


def test_run_closed_loop_smoke_writes_log_and_metrics() -> None:
    pytest.importorskip("mujoco")

    output_root = Path(__file__).resolve().parents[1] / "outputs" / "test_runner_dirs"
    args = runner.resolve_run_args(
        runner.build_arg_parser().parse_args(
            [
                "--run-id",
                "pytest_b02_smoke",
                "--output-root",
                str(output_root),
                "--num-steps",
                "2",
                "--horizon",
                "1",
                "--num-candidates",
                "2",
                "--no-export-video",
                "--no-save-figures",
                "--save-metrics",
                "--no-show-viewer",
            ]
        )
    )

    result = runner.run_closed_loop(args)
    outputs = runner.build_run_outputs(args.run_dir)

    assert len(result["time_history"]) == 2
    assert outputs["log"].exists()
    assert outputs["metrics"].exists()
    assert outputs["tracking_log"].exists()
    assert "final_ee_error" in outputs["metrics"].read_text(encoding="utf-8")
