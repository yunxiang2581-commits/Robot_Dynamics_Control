"""Run B03 MPC solver ladder demo skeleton.

This runner keeps B03 in a learning-oriented shape:
- reuse the B02 two-link task/env/target/cost through the adapter
- let each solver run alone or run all enabled solvers sequentially
- save outputs grouped by solver name
- keep visualization, logging, and video export outside solver/controller code
- avoid long MuJoCo simulations in the current stage
"""

from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime
import logging
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np
import yaml


SCRIPT_PATH = Path(__file__).resolve()
SIMULATOR_ROOT = SCRIPT_PATH.parents[1]
PROJECT_ROOT = SIMULATOR_ROOT.parent
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "configs" / "B03_mpc_solver_ladder.yaml"
TASK_NAME = "B03_mpc_solver_ladder_demo"
DEFAULT_TWO_LINK_MODEL_PATH = SIMULATOR_ROOT / "models" / "B02_two_link.xml"
DEFAULT_END_EFFECTOR_SITE = "ee_site"

REPO_ROOT = PROJECT_ROOT.parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

from projects.B_mujoco_mpc_study.simulator.adapters.b02_to_b03_adapter import (
    B02AdapterConfig,
    B02TrackingSnapshot,
    build_b03_problem_from_b02_snapshot,
    make_b02_rollout_cost_fn,
    wrap_b02_controller_as_solver,
)
from projects.B_mujoco_mpc_study.simulator.planners.mpc_solver_interface import MPCSolution
from projects.B_mujoco_mpc_study.simulator.planners.sampling_mpc_solvers import (
    CEMShootingSolver,
    MPPILiteSolver,
    RandomShootingSolver,
    WarmStartSamplingSolver,
)
from projects.B_mujoco_mpc_study.simulator.utils.plotting import save_b02_tracking_figures
from projects.B_mujoco_mpc_study.simulator.utils.solver_benchmark_logger import (
    SolverBenchmarkLogBuffer,
    SolverSummaryRow,
    save_comparison_metrics_csv,
)
from projects.B_mujoco_mpc_study.simulator.utils.solver_comparison_visualizer import (
    compute_control_smoothness,
    compute_trajectory_smoothness,
    plot_control_smoothness_comparison,
    plot_solver_cost_comparison,
    plot_solver_error_comparison,
    plot_solver_runtime_comparison,
    plot_trajectory_smoothness_comparison,
)
from projects.B_mujoco_mpc_study.simulator.utils.video_renderer import render_marked_video


def build_arg_parser() -> argparse.ArgumentParser:
    """Create the B03 CLI."""
    parser = argparse.ArgumentParser(description="Run B03 MPC solver ladder demo skeleton.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH, help="B03 YAML config path.")
    parser.add_argument("--run-id", type=str, default=None, help="Optional run id.")
    parser.add_argument("--model-family", type=str, default=None, help="Model family override.")
    parser.add_argument("--solvers", nargs="*", default=None, help="Enabled solver names.")
    parser.add_argument("--num-steps", type=int, default=None, help="Override closed-loop step count.")
    parser.add_argument("--horizon", type=int, default=None, help="Override horizon.")
    parser.add_argument("--num-candidates", type=int, default=None, help="Override candidate count.")
    parser.add_argument("--seed", type=int, default=None, help="Override random seed.")
    parser.add_argument("--export-video", action=argparse.BooleanOptionalAction, default=None, help="Export MP4 outputs.")
    parser.add_argument("--save-figures", action=argparse.BooleanOptionalAction, default=None, help="Save PNG figures.")
    parser.add_argument("--save-metrics", action=argparse.BooleanOptionalAction, default=None, help="Save CSV/TXT metrics.")
    parser.add_argument("--no-show-viewer", action="store_true", help="Keep viewer disabled.")
    parser.add_argument("--log-level", type=str, default="INFO", help="Python logging level.")
    return parser


def build_run_dir(run_id: str | None) -> Path:
    """Build the B03 run directory."""
    actual_run_id = run_id or datetime.now().strftime("%Y%m%d_%H%M%S")
    return PROJECT_ROOT / "outputs" / "runs" / TASK_NAME / actual_run_id


def build_run_outputs(run_dir: Path) -> dict[str, Path]:
    """Build top-level B03 output paths."""
    return {
        "run_dir": run_dir,
        "videos": run_dir / "videos",
        "figures": run_dir / "figures",
        "metrics": run_dir / "metrics",
        "logs": run_dir / "logs",
        "summary_csv": run_dir / "metrics" / "B03_solver_summary.csv",
        "run_log": run_dir / "logs" / "B03_solver_run_log.txt",
        "error_curve": run_dir / "figures" / "B03_solver_error_comparison.png",
        "runtime_curve": run_dir / "figures" / "B03_solver_runtime_comparison.png",
        "cost_curve": run_dir / "figures" / "B03_solver_cost_comparison.png",
        "control_smoothness_curve": run_dir / "figures" / "B03_control_smoothness_comparison.png",
        "trajectory_smoothness_curve": run_dir / "figures" / "B03_trajectory_smoothness_comparison.png",
        "comparison_metrics_csv": run_dir / "metrics" / "B03_solver_comparison_metrics.csv",
    }


def build_solver_outputs(run_dir: Path, solver_name: str) -> dict[str, Path]:
    """Build per-solver grouped outputs."""
    solver_dir = run_dir / "solvers" / solver_name
    outputs = {
        "solver_dir": solver_dir,
        "logs_dir": solver_dir / "logs",
        "metrics_dir": solver_dir / "metrics",
        "figures_dir": solver_dir / "figures",
        "videos_dir": solver_dir / "videos",
        "step_log": solver_dir / "logs" / f"B03_{solver_name}_step_log.csv",
        "tracking_log": solver_dir / "logs" / f"B03_{solver_name}_tracking_log.csv",
        "run_log": solver_dir / "logs" / f"B03_{solver_name}_run_log.txt",
        "summary_csv": solver_dir / "metrics" / f"B03_{solver_name}_summary.csv",
        "metrics_csv": solver_dir / "metrics" / f"B03_{solver_name}_metrics.csv",
        "raw_video": solver_dir / "videos" / f"B03_{solver_name}_raw.mp4",
        "marked_video": solver_dir / "videos" / f"B03_{solver_name}_marked.mp4",
        "xy_target_vs_actual": solver_dir / "figures" / f"B03_{solver_name}_xy_target_vs_actual.png",
        "tracking_error_time": solver_dir / "figures" / f"B03_{solver_name}_tracking_error_time.png",
        "control_input_time": solver_dir / "figures" / f"B03_{solver_name}_control_input_time.png",
        "ee_trajectory_xy": solver_dir / "figures" / f"B03_{solver_name}_ee_trajectory_xy.png",
        "ee_tracking_error": solver_dir / "figures" / f"B03_{solver_name}_ee_tracking_error.png",
        "joint_torque": solver_dir / "figures" / f"B03_{solver_name}_joint_torque.png",
        "error_figure": solver_dir / "figures" / f"B03_{solver_name}_error_curve.png",
    }
    outputs["logs"] = outputs["step_log"]
    outputs["metrics"] = outputs["summary_csv"]
    outputs["figures"] = outputs["error_figure"]
    outputs["videos"] = outputs["raw_video"]
    return outputs


def ensure_output_dirs(outputs: dict[str, Path]) -> None:
    """Create output directories or parent directories."""
    for key, output_path in outputs.items():
        if key.endswith("_dir"):
            output_path.mkdir(parents=True, exist_ok=True)
        elif output_path.suffix:
            output_path.parent.mkdir(parents=True, exist_ok=True)
        else:
            output_path.mkdir(parents=True, exist_ok=True)


def load_b03_config(config_path: Path) -> dict[str, Any]:
    """Load the B03 YAML config."""
    with config_path.open("r", encoding="utf-8") as config_file:
        loaded = yaml.safe_load(config_file) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"B03 config must be a mapping, got {type(loaded)!r}")
    return loaded


def _as_pair(value: Any, name: str) -> tuple[float, float]:
    if len(value) != 2:
        raise ValueError(f"{name} must contain exactly 2 values, got {value}")
    return float(value[0]), float(value[1])


def resolve_b03_runtime_config(config: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    """Merge YAML and CLI into a runtime config."""
    simulation = config.get("simulation", {})
    sampling = config.get("sampling", {})
    target = config.get("target", {})
    solvers = config.get("solvers", {})
    cost = config.get("cost", {})
    visualization = config.get("visualization", {})
    cem = config.get("cem", {})
    mppi = config.get("mppi", {})

    enabled_solvers = list(solvers.get("enabled", []))
    if args.solvers:
        enabled_solvers = list(args.solvers)

    runtime_config = {
        "model_family": args.model_family or simulation.get("model_family", "two_link"),
        "num_steps": int(args.num_steps if args.num_steps is not None else simulation.get("num_steps", 10)),
        "dt": float(simulation.get("dt", 0.01)),
        "seed": int(args.seed if args.seed is not None else simulation.get("seed", sampling.get("seed", 0))),
        "horizon": int(args.horizon if args.horizon is not None else sampling.get("horizon", 16)),
        "num_candidates": int(args.num_candidates if args.num_candidates is not None else sampling.get("num_candidates", 64)),
        "control_dim": int(sampling.get("control_dim", 2)),
        "torque_limit": float(sampling.get("torque_limit", 2.0)),
        "sampling_std": float(sampling.get("sampling_std", 0.5)),
        "target_type": target.get("type", "circle"),
        "target_center": _as_pair(target.get("center", (0.45, 0.10)), "target.center"),
        "target_radius": float(target.get("radius", 0.08)),
        "target_frequency": float(target.get("frequency", 0.2)),
        "target_radius_x": float(target["radius_x"]) if "radius_x" in target else None,
        "target_radius_y": float(target["radius_y"]) if "radius_y" in target else None,
        "target_frequency_x": float(target["frequency_x"]) if "frequency_x" in target else None,
        "target_frequency_y": float(target["frequency_y"]) if "frequency_y" in target else None,
        "target_phase_offset": float(target.get("phase_offset", 0.0)),
        "enabled_solvers": [
            name
            for name in enabled_solvers
            if name in {"b02_baseline", "random_shooting", "warm_start_sampling", "cem", "mppi_lite"}
        ],
        "export_video": bool(args.export_video if args.export_video is not None else visualization.get("export_video", False)),
        "save_figures": bool(args.save_figures if args.save_figures is not None else False),
        "save_metrics": bool(args.save_metrics if args.save_metrics is not None else True),
        "show_viewer": True,
        "initial_q": (0.3, 0.4),
        "initial_dq": (0.0, 0.0),
        "model_path": DEFAULT_TWO_LINK_MODEL_PATH,
        "end_effector_site": DEFAULT_END_EFFECTOR_SITE,
        "cost": {
            "ee_weight": float(cost.get("ee_weight", 80.0)),
            "dq_weight": float(cost.get("dq_weight", 0.1)),
            "torque_weight": float(cost.get("torque_weight", 0.002)),
            "terminal_weight": float(cost.get("terminal_weight", 10.0)),
            "smoothness_weight": float(cost.get("smoothness_weight", 0.01)),
        },
    }

    runtime_config["solver_configs"] = {
        "random_shooting": {
            "num_candidates": runtime_config["num_candidates"],
            "sampling_std": runtime_config["sampling_std"],
            "torque_limit": runtime_config["torque_limit"],
            "seed": runtime_config["seed"],
        },
        "warm_start_sampling": {
            "num_candidates": runtime_config["num_candidates"],
            "sampling_std": runtime_config["sampling_std"],
            "torque_limit": runtime_config["torque_limit"],
            "seed": runtime_config["seed"],
        },
        "cem": {
            "num_candidates": int(cem.get("num_candidates", runtime_config["num_candidates"])),
            "sampling_std": float(cem.get("initial_std", runtime_config["sampling_std"])),
            "initial_std": float(cem.get("initial_std", runtime_config["sampling_std"])),
            "torque_limit": float(cem.get("torque_limit", runtime_config["torque_limit"])),
            "seed": runtime_config["seed"],
            "num_iterations": int(cem.get("num_iterations", 3)),
            "elite_ratio": float(cem.get("elite_ratio", 0.1)),
            "min_std": float(cem.get("min_std", 0.1)),
            "smoothing_alpha": float(cem.get("smoothing_alpha", 0.2)),
        },
        "mppi_lite": {
            "num_candidates": int(mppi.get("num_candidates", runtime_config["num_candidates"])),
            "sampling_std": float(mppi.get("noise_std", runtime_config["sampling_std"])),
            "noise_std": float(mppi.get("noise_std", runtime_config["sampling_std"])),
            "torque_limit": float(mppi.get("torque_limit", runtime_config["torque_limit"])),
            "seed": runtime_config["seed"],
            "num_iterations": int(mppi.get("num_iterations", 3)),
            "temperature": float(mppi.get("temperature", 1.0)),
            "min_std": float(mppi.get("min_std", 0.1)),
            "smoothing_alpha": float(mppi.get("smoothing_alpha", 0.2)),
        },
    }
    return runtime_config


def build_target_sequence(
    *,
    target_type: str,
    current_time: float,
    horizon: int,
    dt: float,
    center: tuple[float, float],
    radius: float,
    frequency: float,
    radius_x: float | None = None,
    radius_y: float | None = None,
    frequency_x: float | None = None,
    frequency_y: float | None = None,
    phase_offset: float = 0.0,
) -> list[tuple[float, float]]:
    """Build the current target horizon."""
    cx, cy = center
    targets: list[tuple[float, float]] = []
    for step_offset in range(horizon):
        target_time = current_time + (step_offset + 1) * dt
        phase = 2.0 * math.pi * frequency * target_time
        if target_type == "circle":
            targets.append((cx + radius * math.cos(phase), cy + radius * math.sin(phase)))
        elif target_type == "fixed":
            targets.append((cx, cy))
        elif target_type == "figure8":
            rx = radius_x if radius_x is not None else radius
            ry = radius_y if radius_y is not None else radius
            fx = frequency_x if frequency_x is not None else frequency
            fy = frequency_y if frequency_y is not None else 2.0 * frequency
            phase_x = 2.0 * math.pi * fx * target_time
            phase_y = 2.0 * math.pi * fy * target_time
            targets.append((cx + rx * math.sin(phase_x), cy + ry * math.sin(phase_y)))
        elif target_type == "lissajous":
            rx = radius_x if radius_x is not None else radius
            ry = radius_y if radius_y is not None else radius
            fx = frequency_x if frequency_x is not None else frequency
            fy = frequency_y if frequency_y is not None else frequency * 1.5
            phase_x = 2.0 * math.pi * fx * target_time
            phase_y = 2.0 * math.pi * fy * target_time + phase_offset
            targets.append((cx + rx * math.sin(phase_x), cy + ry * math.sin(phase_y)))
        else:
            raise ValueError(f"Unsupported B03 target_type: {target_type}")
    return targets


def create_b03_two_link_env(runtime_config: dict[str, Any]) -> Any:
    """Create the B02 two-link environment used by B03."""
    from projects.B_mujoco_mpc_study.simulator.envs.two_link_env import TwoLinkEnv

    return TwoLinkEnv(
        model_path=runtime_config["model_path"],
        dt=runtime_config["dt"],
        end_effector_site=runtime_config["end_effector_site"],
    )


def build_adapter_config(runtime_config: dict[str, Any]) -> B02AdapterConfig:
    """Convert runtime config to adapter config."""
    return B02AdapterConfig(
        horizon=int(runtime_config["horizon"]),
        dt=float(runtime_config["dt"]),
        control_dim=int(runtime_config["control_dim"]),
        torque_limit=float(runtime_config["torque_limit"]),
        ee_weight=float(runtime_config["cost"]["ee_weight"]),
        dq_weight=float(runtime_config["cost"]["dq_weight"]),
        torque_weight=float(runtime_config["cost"]["torque_weight"]),
        terminal_weight=float(runtime_config["cost"]["terminal_weight"]),
        record_predicted_states=True,
        record_predicted_ee_positions=True,
    )


def _initial_state_from_runtime_config(runtime_config: dict[str, Any]) -> tuple[tuple[float, float], tuple[float, float]]:
    """Read the initial state from runtime config."""
    initial_q = runtime_config.get("initial_q", (0.3, 0.4))
    initial_dq = runtime_config.get("initial_dq", (0.0, 0.0))
    return (float(initial_q[0]), float(initial_q[1])), (float(initial_dq[0]), float(initial_dq[1]))


def extract_snapshot_from_env(
    env: Any,
    target_xy: tuple[float, float],
    time_value: float,
) -> B02TrackingSnapshot:
    """Extract a B02 snapshot from the current environment state."""
    state = np.asarray(env.get_state(), dtype=float)
    actual_xy = np.asarray(env.get_end_effector_position(), dtype=float)
    target_array = np.asarray(target_xy, dtype=float)
    return B02TrackingSnapshot(
        time=float(time_value),
        q=state[:2].copy(),
        qvel=state[2:4].copy(),
        state=state.copy(),
        target_xy=target_array,
        actual_xy=actual_xy.copy(),
        error_norm=float(np.linalg.norm(actual_xy - target_array)),
    )


def create_b02_baseline_solver(runtime_config: dict[str, Any], env: Any) -> Any:
    """Wrap the B02 baseline controller as a B03 solver."""
    from projects.B_mujoco_mpc_study.simulator.controllers.two_link_mpc_controller import TwoLinkMPCController
    from projects.B_mujoco_mpc_study.simulator.planners.two_link_predictive_sampling import TwoLinkPredictiveSamplingPlanner

    planner = TwoLinkPredictiveSamplingPlanner(
        horizon=int(runtime_config["horizon"]),
        num_candidates=int(runtime_config["num_candidates"]),
        torque_limit=float(runtime_config["torque_limit"]),
        ee_weight=float(runtime_config["cost"]["ee_weight"]),
        dq_weight=float(runtime_config["cost"]["dq_weight"]),
        torque_weight=float(runtime_config["cost"]["torque_weight"]),
        terminal_weight=float(runtime_config["cost"]["terminal_weight"]),
    )
    controller = TwoLinkMPCController(planner=planner)
    return wrap_b02_controller_as_solver(controller=controller, env=env)


def build_b03_solver(solver_name: str, runtime_config: dict[str, Any], env: Any) -> Any:
    """Instantiate a currently supported B03 solver."""
    _ = runtime_config, env
    if solver_name == "b02_baseline":
        return create_b02_baseline_solver(runtime_config, env)
    if solver_name == "random_shooting":
        return RandomShootingSolver()
    if solver_name == "warm_start_sampling":
        return WarmStartSamplingSolver()
    if solver_name == "cem":
        return CEMShootingSolver()
    if solver_name == "mppi_lite":
        return MPPILiteSolver()
    raise ValueError(f"Unsupported or not-yet-implemented B03 solver: {solver_name}")


def _compute_summary_row(solver_name: str, step_rows: list[Any]) -> SolverSummaryRow:
    """Build a summary row from step rows."""
    if not step_rows:
        return SolverSummaryRow(
            solver_name=solver_name,
            final_ee_error=float("nan"),
            mean_ee_error=float("nan"),
            max_ee_error=float("nan"),
            runtime_per_control_step=float("nan"),
            max_abs_torque=float("nan"),
            mean_abs_torque=float("nan"),
            control_smoothness=0.0,
            trajectory_smoothness=0.0,
            best_cost=float("nan"),
            cost_std=float("nan"),
            solver_success_rate=0.0,
        )

    current_errors = np.asarray([row.current_ee_error for row in step_rows], dtype=float)
    runtimes = np.asarray([row.runtime_ms for row in step_rows], dtype=float)
    torques = np.asarray([row.max_abs_torque for row in step_rows], dtype=float)
    best_costs = np.asarray([row.best_cost for row in step_rows], dtype=float)
    success_rate = float(np.mean([1.0 if row.success else 0.0 for row in step_rows]))
    return SolverSummaryRow(
        solver_name=solver_name,
        final_ee_error=float(current_errors[-1]),
        mean_ee_error=float(np.mean(current_errors)),
        max_ee_error=float(np.max(current_errors)),
        runtime_per_control_step=float(np.mean(runtimes)),
        max_abs_torque=float(np.max(torques)),
        mean_abs_torque=float(np.mean(torques)),
        control_smoothness=float(step_rows[-1].control_smoothness),
        trajectory_smoothness=float(step_rows[-1].trajectory_smoothness),
        best_cost=float(np.min(best_costs)),
        cost_std=float(np.std(best_costs)),
        solver_success_rate=success_rate,
    )


def save_metrics_csv(metrics: dict[str, float], output_path: Path) -> None:
    """Save metrics as a two-column CSV."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    lines = ["metric,value"]
    for name, value in metrics.items():
        lines.append(f"{name},{value}")
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def save_video(frames: list[Any], output_path: Path, fps: int) -> None:
    """Save raw MP4 output."""
    if not frames:
        raise ValueError("frames must not be empty")
    try:
        import imageio.v2 as imageio
    except ImportError as exc:
        raise RuntimeError("Exporting MP4 requires imageio") from exc

    output_path.parent.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(output_path, frames, fps=fps)


def _build_tracking_rows(
    *,
    time_history: list[float],
    target_history: list[np.ndarray],
    actual_history: list[np.ndarray],
    control_history: list[np.ndarray],
    best_cost_history: list[float],
) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    for step_idx, (time_value, target_xy, actual_xy, control, best_cost) in enumerate(
        zip(time_history, target_history, actual_history, control_history, best_cost_history)
    ):
        error_norm = float(np.linalg.norm(np.asarray(actual_xy, dtype=float) - np.asarray(target_xy, dtype=float)))
        rows.append(
            {
                "step": step_idx,
                "time": float(time_value),
                "target_x": float(target_xy[0]),
                "target_y": float(target_xy[1]),
                "actual_x": float(actual_xy[0]),
                "actual_y": float(actual_xy[1]),
                "error_norm": error_norm,
                "u1": float(control[0]),
                "u2": float(control[1]),
                "mpc_cost": float(best_cost),
            }
        )
    return rows


def _save_tracking_log(tracking_rows: list[dict[str, float]], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    header = [
        "step",
        "time",
        "target_x",
        "target_y",
        "actual_x",
        "actual_y",
        "error_norm",
        "u1",
        "u2",
        "mpc_cost",
    ]
    lines = [",".join(header)]
    for row in tracking_rows:
        lines.append(",".join(str(row[column]) for column in header))
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_solver_run_log(
    output_path: Path,
    *,
    solver_name: str,
    runtime_config: dict[str, Any],
    summary_row: SolverSummaryRow,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        f"task_name: {TASK_NAME}",
        f"solver_name: {solver_name}",
        f"model_family: {runtime_config['model_family']}",
        f"num_steps: {runtime_config['num_steps']}",
        f"horizon: {runtime_config['horizon']}",
        f"target_type: {runtime_config['target_type']}",
        f"export_video: {runtime_config['export_video']}",
        f"save_figures: {runtime_config['save_figures']}",
        f"save_metrics: {runtime_config['save_metrics']}",
        "",
        "summary:",
        f"- final_ee_error: {summary_row.final_ee_error}",
        f"- mean_ee_error: {summary_row.mean_ee_error}",
        f"- max_ee_error: {summary_row.max_ee_error}",
        f"- runtime_per_control_step: {summary_row.runtime_per_control_step}",
        f"- best_cost: {summary_row.best_cost}",
        f"- control_smoothness: {summary_row.control_smoothness}",
        f"- trajectory_smoothness: {summary_row.trajectory_smoothness}",
    ]
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_run_log(
    output_path: Path,
    runtime_config: dict[str, Any],
    summary_rows: list[SolverSummaryRow],
) -> None:
    """Write a short top-level run log."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        f"task_name: {TASK_NAME}",
        f"model_family: {runtime_config['model_family']}",
        f"num_steps: {runtime_config['num_steps']}",
        f"horizon: {runtime_config['horizon']}",
        f"enabled_solvers: {runtime_config['enabled_solvers']}",
        "",
        "solver_summaries:",
    ]
    for row in summary_rows:
        lines.append(
            f"- {row.solver_name}: final_ee_error={row.final_ee_error:.6f}, "
            f"mean_ee_error={row.mean_ee_error:.6f}, runtime_ms={row.runtime_per_control_step:.6f}"
        )
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _build_solver_metrics(summary_row: SolverSummaryRow) -> dict[str, float]:
    return {
        "final_ee_error": summary_row.final_ee_error,
        "mean_ee_error": summary_row.mean_ee_error,
        "max_ee_error": summary_row.max_ee_error,
        "runtime_per_control_step": summary_row.runtime_per_control_step,
        "max_abs_torque": summary_row.max_abs_torque,
        "mean_abs_torque": summary_row.mean_abs_torque,
        "control_smoothness": summary_row.control_smoothness,
        "trajectory_smoothness": summary_row.trajectory_smoothness,
        "best_cost": summary_row.best_cost,
        "cost_std": summary_row.cost_std,
        "solver_success_rate": summary_row.solver_success_rate,
    }


def _save_solver_figures(
    solver_outputs: dict[str, Path],
    tracking_rows: list[dict[str, float]],
) -> None:
    save_b02_tracking_figures(
        tracking_rows=tracking_rows,
        outputs={
            "xy_target_vs_actual": solver_outputs["xy_target_vs_actual"],
            "tracking_error_time": solver_outputs["tracking_error_time"],
            "control_input_time": solver_outputs["control_input_time"],
            "ee_trajectory_xy": solver_outputs["ee_trajectory_xy"],
            "ee_tracking_error": solver_outputs["ee_tracking_error"],
            "joint_torque": solver_outputs["joint_torque"],
        },
    )


def _save_per_solver_error_figure(
    *,
    solver_name: str,
    step_rows: list[Any],
    output_path: Path,
) -> None:
    plot_solver_error_comparison({solver_name: step_rows}, output_path)


_Z_SCENE = 0.01


def _build_scene_geoms(
    *,
    target_xy: tuple[float, float],
    actual_xy: tuple[float, float],
    target_history: list[np.ndarray],
    ee_history: list[np.ndarray],
) -> list[dict[str, Any]]:
    """Build scene geoms with actual EE trajectory trail."""
    geoms: list[dict[str, Any]] = []
    tx, ty = float(target_xy[0]), float(target_xy[1])
    ax, ay = float(actual_xy[0]), float(actual_xy[1])
    z = _Z_SCENE

    geoms.append({
        "geom_type": "sphere",
        "pos": (tx, ty, z),
        "size": (0.015, 0.015, 0.015),
        "rgba": (1.0, 0.3, 0.3, 1.0),
    })
    geoms.append({
        "geom_type": "sphere",
        "pos": (ax, ay, z),
        "size": (0.015, 0.015, 0.015),
        "rgba": (0.3, 1.0, 0.5, 1.0),
    })
    geoms.append({
        "geom_type": "line",
        "pos": (ax, ay, z),
        "from_pos": (ax, ay, z),
        "to_pos": (tx, ty, z),
        "size": (2.0, 0.0, 0.0),
        "rgba": (1.0, 1.0, 1.0, 0.6),
    })

    for i in range(1, len(target_history)):
        p0 = (float(target_history[i - 1][0]), float(target_history[i - 1][1]), z)
        p1 = (float(target_history[i][0]), float(target_history[i][1]), z)
        geoms.append({
            "geom_type": "line",
            "pos": p0,
            "from_pos": p0,
            "to_pos": p1,
            "size": (1.5, 0.0, 0.0),
            "rgba": (1.0, 0.71, 0.24, 0.9),
        })

    for i in range(1, len(ee_history)):
        p0 = (float(ee_history[i - 1][0]), float(ee_history[i - 1][1]), z)
        p1 = (float(ee_history[i][0]), float(ee_history[i][1]), z)
        geoms.append({
            "geom_type": "line",
            "pos": p0,
            "from_pos": p0,
            "to_pos": p1,
            "size": (2.5, 0.0, 0.0),
            "rgba": (0.3, 1.0, 0.5, 0.9),
        })

    return geoms


def _inject_viewer_scene_geoms(
    *,
    viewer: Any,
    target_xy: Any,
    actual_xy: Any,
    target_history: list[Any],
    ee_history: list[Any],
) -> None:
    """Inject trajectory geoms into the live viewer's user scene."""
    import mujoco

    scn = viewer.user_scn
    scn.ngeom = 0
    z = _Z_SCENE

    def _add_sphere(pos: tuple[float, float, float], size: float, rgba: tuple[float, float, float, float]) -> None:
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            g, mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([size, size, size], dtype=np.float64),
            np.array(pos, dtype=np.float64),
            np.eye(3, dtype=np.float64).reshape(-1),
            np.array(rgba, dtype=np.float32),
        )
        scn.ngeom += 1

    def _add_line(p0: tuple[float, float, float], p1: tuple[float, float, float], width: float, rgba: tuple[float, float, float, float]) -> None:
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            g, mujoco.mjtGeom.mjGEOM_LINE,
            np.array([width, 0, 0], dtype=np.float64),
            np.array(p0, dtype=np.float64),
            np.eye(3, dtype=np.float64).reshape(-1),
            np.array(rgba, dtype=np.float32),
        )
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_LINE, width, np.array(p0, dtype=np.float64), np.array(p1, dtype=np.float64))
        scn.ngeom += 1

    # Target marker (red) and actual marker (green)
    tx, ty = float(target_xy[0]), float(target_xy[1])
    ax, ay = float(actual_xy[0]), float(actual_xy[1])
    _add_sphere((tx, ty, z), 0.015, (1.0, 0.3, 0.3, 1.0))
    _add_sphere((ax, ay, z), 0.015, (0.3, 1.0, 0.5, 1.0))

    # Error line (white)
    _add_line((ax, ay, z), (tx, ty, z), 2.0, (1.0, 1.0, 1.0, 0.6))

    # Target trajectory trail (yellow)
    for i in range(1, len(target_history)):
        p0 = (float(target_history[i - 1][0]), float(target_history[i - 1][1]), z)
        p1 = (float(target_history[i][0]), float(target_history[i][1]), z)
        _add_line(p0, p1, 1.5, (1.0, 0.71, 0.24, 0.9))

    # Actual EE trajectory trail (green)
    for i in range(1, len(ee_history)):
        p0 = (float(ee_history[i - 1][0]), float(ee_history[i - 1][1]), z)
        p1 = (float(ee_history[i][0]), float(ee_history[i][1]), z)
        _add_line(p0, p1, 2.5, (0.3, 1.0, 0.5, 0.9))


def run_single_solver_comparison(
    *,
    solver_name: str,
    runtime_config: dict[str, Any],
    run_dir: Path,
    show_viewer: bool = False,
) -> dict[str, Any]:
    """Run one small closed-loop comparison loop for a single solver."""
    env = create_b03_two_link_env(runtime_config)
    solver = build_b03_solver(solver_name, runtime_config, env)
    adapter_config = build_adapter_config(runtime_config)

    viewer = None
    if show_viewer:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(env.model, env.data)
        # Fix camera to match env's fixed camera
        viewer.cam.fixedcamid = env._camera.fixedcamid
        viewer.cam.type = env._camera.type
        viewer.cam.lookat[:] = env._camera.lookat
        viewer.cam.distance = env._camera.distance
        viewer.cam.azimuth = env._camera.azimuth
        viewer.cam.elevation = env._camera.elevation
    rollout_cost_fn = make_b02_rollout_cost_fn(env=env, adapter_config=adapter_config)

    initial_q, initial_dq = _initial_state_from_runtime_config(runtime_config)
    env.reset(q=initial_q, dq=initial_dq)
    solver_outputs = build_solver_outputs(run_dir, solver_name)
    ensure_output_dirs(solver_outputs)

    benchmark_logger = SolverBenchmarkLogBuffer()
    previous_solution: MPCSolution | None = None
    control_history: list[np.ndarray] = []
    target_history: list[np.ndarray] = []
    ee_history: list[np.ndarray] = []
    best_cost_history: list[float] = []
    time_history: list[float] = []
    raw_frames: list[Any] = []

    for step_idx in range(int(runtime_config["num_steps"])):
        current_time = step_idx * float(runtime_config["dt"])
        target_horizon = build_target_sequence(
            target_type=runtime_config["target_type"],
            current_time=current_time,
            horizon=int(runtime_config["horizon"]),
            dt=float(runtime_config["dt"]),
            center=runtime_config["target_center"],
            radius=float(runtime_config["target_radius"]),
            frequency=float(runtime_config["target_frequency"]),
            radius_x=runtime_config.get("target_radius_x"),
            radius_y=runtime_config.get("target_radius_y"),
            frequency_x=runtime_config.get("target_frequency_x"),
            frequency_y=runtime_config.get("target_frequency_y"),
            phase_offset=float(runtime_config.get("target_phase_offset", 0.0)),
        )
        snapshot = extract_snapshot_from_env(
            env=env,
            target_xy=target_horizon[0],
            time_value=current_time,
        )
        problem = build_b03_problem_from_b02_snapshot(
            snapshot=snapshot,
            target_horizon=np.asarray(target_horizon, dtype=float),
            adapter_config=adapter_config,
            solver_config=deepcopy(runtime_config["solver_configs"].get(solver_name, {})),
            rollout_cost_fn=rollout_cost_fn,
        )

        solution = solver.solve(problem, previous_solution=previous_solution)
        previous_solution = solution

        env.step(tuple(float(value) for value in solution.first_control.tolist()))
        applied_torque = np.asarray(env.get_last_applied_torque(), dtype=float)
        actual_xy = np.asarray(env.get_end_effector_position(), dtype=float)
        target_xy = np.asarray(target_horizon[0], dtype=float)
        current_error = float(np.linalg.norm(actual_xy - target_xy))
        record_time = current_time + float(runtime_config["dt"])

        control_history.append(applied_torque)
        ee_history.append(actual_xy)
        target_history.append(target_xy)

        # Update viewer with trajectory geoms
        if viewer is not None:
            _inject_viewer_scene_geoms(
                viewer=viewer,
                target_xy=target_xy,
                actual_xy=actual_xy,
                target_history=target_history,
                ee_history=ee_history,
            )
            viewer.sync()
        best_cost_history.append(float(solution.best_cost))
        time_history.append(record_time)

        benchmark_logger.append_step(
            step=step_idx,
            time=record_time,
            solver_name=solver_name,
            final_ee_error=current_error,
            current_ee_error=current_error,
            best_cost=float(solution.best_cost),
            runtime_ms=float(solution.solver_stats.runtime_ms),
            num_rollouts=int(solution.solver_stats.num_rollouts),
            num_iterations=int(solution.solver_stats.num_iterations),
            max_abs_torque=float(np.max(np.abs(applied_torque))),
            control_smoothness=compute_control_smoothness(np.asarray(control_history, dtype=float)),
            trajectory_smoothness=compute_trajectory_smoothness(np.asarray(ee_history, dtype=float)),
            success=bool(solution.solver_stats.success),
        )

        if bool(runtime_config.get("export_video", False)) and hasattr(env, "render_frame"):
            scene_geoms = _build_scene_geoms(
                target_xy=target_horizon[0],
                actual_xy=actual_xy,
                target_history=target_history,
                ee_history=ee_history,
            )
            raw_frames.append(env.render_frame_with_scene_geoms(scene_geoms))

    if viewer is not None:
        viewer.close()

    summary_row = _compute_summary_row(solver_name, benchmark_logger.step_rows())
    benchmark_logger.append_summary(
        solver_name=summary_row.solver_name,
        final_ee_error=summary_row.final_ee_error,
        mean_ee_error=summary_row.mean_ee_error,
        max_ee_error=summary_row.max_ee_error,
        runtime_per_control_step=summary_row.runtime_per_control_step,
        max_abs_torque=summary_row.max_abs_torque,
        mean_abs_torque=summary_row.mean_abs_torque,
        control_smoothness=summary_row.control_smoothness,
        trajectory_smoothness=summary_row.trajectory_smoothness,
        best_cost=summary_row.best_cost,
        cost_std=summary_row.cost_std,
        solver_success_rate=summary_row.solver_success_rate,
    )

    tracking_rows = _build_tracking_rows(
        time_history=time_history,
        target_history=target_history,
        actual_history=ee_history,
        control_history=control_history,
        best_cost_history=best_cost_history,
    )
    metrics = _build_solver_metrics(summary_row)

    benchmark_logger.save_step_csv(solver_outputs["step_log"])
    benchmark_logger.save_summary_csv(solver_outputs["summary_csv"])
    _save_tracking_log(tracking_rows, solver_outputs["tracking_log"])
    _write_solver_run_log(
        solver_outputs["run_log"],
        solver_name=solver_name,
        runtime_config=runtime_config,
        summary_row=summary_row,
    )
    if bool(runtime_config.get("save_metrics", True)):
        save_metrics_csv(metrics, solver_outputs["metrics_csv"])
    if bool(runtime_config.get("save_figures", False)) and tracking_rows:
        _save_solver_figures(solver_outputs, tracking_rows)
        _save_per_solver_error_figure(
            solver_name=solver_name,
            step_rows=benchmark_logger.step_rows(),
            output_path=solver_outputs["error_figure"],
        )
    if bool(runtime_config.get("export_video", False)) and raw_frames:
        fps = max(1, int(round(1.0 / float(runtime_config["dt"]))))
        save_video(raw_frames, solver_outputs["raw_video"], fps=fps)
        render_marked_video(
            raw_frames=raw_frames,
            tracking_rows=tracking_rows,
            output_path=solver_outputs["marked_video"],
            fps=fps,
        )

    return {
        "solver_name": solver_name,
        "solver_outputs": solver_outputs,
        "step_rows": benchmark_logger.step_rows(),
        "summary_row": summary_row,
        "last_solution": previous_solution,
        "tracking_rows": tracking_rows,
        "metrics": metrics,
    }


def _save_cross_solver_comparison_figures(
    outputs: dict[str, Path],
    solver_results: dict[str, Any],
) -> dict[str, Path]:
    step_rows_by_solver = {
        solver_name: result["step_rows"]
        for solver_name, result in solver_results.items()
        if result["step_rows"]
    }
    if not step_rows_by_solver:
        return {}

    plot_solver_error_comparison(step_rows_by_solver, outputs["error_curve"])
    plot_solver_runtime_comparison(step_rows_by_solver, outputs["runtime_curve"])
    plot_solver_cost_comparison(step_rows_by_solver, outputs["cost_curve"])
    plot_control_smoothness_comparison(step_rows_by_solver, outputs["control_smoothness_curve"])
    plot_trajectory_smoothness_comparison(step_rows_by_solver, outputs["trajectory_smoothness_curve"])
    return {
        "error_curve": outputs["error_curve"],
        "runtime_curve": outputs["runtime_curve"],
        "cost_curve": outputs["cost_curve"],
        "control_smoothness_curve": outputs["control_smoothness_curve"],
        "trajectory_smoothness_curve": outputs["trajectory_smoothness_curve"],
    }


def run_b03_solver_ladder_skeleton(args: argparse.Namespace) -> dict[str, Any]:
    """Run the B03 solver-ladder comparison skeleton."""
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))

    if not args.config.exists():
        raise FileNotFoundError(f"Cannot find B03 config: {args.config}")

    config = load_b03_config(args.config)
    runtime_config = resolve_b03_runtime_config(config, args)
    run_dir = build_run_dir(args.run_id)
    outputs = build_run_outputs(run_dir)
    ensure_output_dirs(outputs)

    solver_results: dict[str, Any] = {}
    solver_outputs: dict[str, Any] = {}
    summary_rows: list[SolverSummaryRow] = []

    logging.info("B03 comparison skeleton prepared outputs at %s", run_dir)
    logging.info("Enabled solvers: %s", runtime_config["enabled_solvers"])
    logging.info("This runner keeps the loop intentionally small; no long MuJoCo simulation is run.")

    show_viewer = not args.no_show_viewer

    for solver_name in runtime_config["enabled_solvers"]:
        logging.info("Running B03 comparison skeleton for solver=%s", solver_name)
        single_result = run_single_solver_comparison(
            solver_name=solver_name,
            runtime_config=runtime_config,
            run_dir=run_dir,
            show_viewer=show_viewer,
        )
        solver_results[solver_name] = single_result
        solver_outputs[solver_name] = single_result["solver_outputs"]
        summary_rows.append(single_result["summary_row"])

    global_summary_logger = SolverBenchmarkLogBuffer()
    for row in summary_rows:
        global_summary_logger.append_summary(
            solver_name=row.solver_name,
            final_ee_error=row.final_ee_error,
            mean_ee_error=row.mean_ee_error,
            max_ee_error=row.max_ee_error,
            runtime_per_control_step=row.runtime_per_control_step,
            max_abs_torque=row.max_abs_torque,
            mean_abs_torque=row.mean_abs_torque,
            control_smoothness=row.control_smoothness,
            trajectory_smoothness=row.trajectory_smoothness,
            best_cost=row.best_cost,
            cost_std=row.cost_std,
            solver_success_rate=row.solver_success_rate,
        )
    global_summary_logger.save_summary_csv(outputs["summary_csv"])
    _write_run_log(outputs["run_log"], runtime_config, summary_rows)
    comparison_outputs = _save_cross_solver_comparison_figures(outputs, solver_results)

    step_rows_by_solver = {
        name: result["step_rows"]
        for name, result in solver_results.items()
        if result["step_rows"]
    }
    solver_configs_for_metrics = {
        name: runtime_config["solver_configs"].get(name, {})
        for name in solver_results
    }
    solver_configs_for_metrics = {
        name: {**cfg, "horizon": runtime_config["horizon"]}
        for name, cfg in solver_configs_for_metrics.items()
    }
    save_comparison_metrics_csv(
        step_rows_by_solver,
        outputs["comparison_metrics_csv"],
        solver_configs=solver_configs_for_metrics,
    )

    return {
        "task_name": TASK_NAME,
        "config": args.config,
        "runtime_config": runtime_config,
        "run_dir": run_dir,
        "outputs": outputs,
        "solver_outputs": solver_outputs,
        "solver_results": solver_results,
        "comparison_outputs": comparison_outputs,
        "summary_rows": summary_rows,
        "enabled_solvers": runtime_config["enabled_solvers"],
        "status": "B03 solver ladder comparison skeleton completed; no long MuJoCo simulation was run.",
    }


def main(argv: list[str] | None = None) -> None:
    """B03 runner entry point."""
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    result = run_b03_solver_ladder_skeleton(args)

    print(result["status"])
    print(f"run_dir: {result['run_dir']}")
    print(f"solvers: {result['enabled_solvers']}")


if __name__ == "__main__":
    main()
