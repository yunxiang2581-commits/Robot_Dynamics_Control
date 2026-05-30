"""B03-R3T Sampling Solver Automated Tuning Runner.

Small-scale parameter sweep for CEM, MPPI-lite and Warm-start Sampling.
Reuses existing B03 solver / adapter / logger logic.

Also supports --recommended-benchmark mode for running the four recommended
sampling solver configs and generating comparison figures + report.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime
import logging
from pathlib import Path
import sys
from typing import Any

import numpy as np
import yaml


SCRIPT_PATH = Path(__file__).resolve()
SIMULATOR_ROOT = SCRIPT_PATH.parents[1]
PROJECT_ROOT = SIMULATOR_ROOT.parent
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "configs" / "B03_sampling_tuning.yaml"
DEFAULT_RECOMMENDED_CONFIG_PATH = PROJECT_ROOT / "configs" / "B03_sampling_recommended_benchmark.yaml"
TASK_NAME = "B03_sampling_solver_tuning"
RECOMMENDED_TASK_NAME = "B03_sampling_recommended_benchmark"
DEFAULT_TWO_LINK_MODEL_PATH = SIMULATOR_ROOT / "models" / "B02_two_link.xml"
DEFAULT_END_EFFECTOR_SITE = "ee_site"

REPO_ROOT = PROJECT_ROOT.parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

from projects.B_mujoco_mpc_study.simulator.scripts.run_B03_mpc_solver_ladder_demo import (
    build_adapter_config,
    build_b03_solver,
    build_run_dir,
    build_target_sequence,
    create_b03_two_link_env,
    extract_snapshot_from_env,
    _build_solver_metrics,
    _build_tracking_rows,
    _compute_summary_row,
    _initial_state_from_runtime_config,
    _inject_viewer_scene_geoms,
    ensure_output_dirs,
    load_b03_config,
    save_metrics_csv,
    _as_pair,
)
from projects.B_mujoco_mpc_study.simulator.adapters.b02_to_b03_adapter import (
    build_b03_problem_from_b02_snapshot,
    make_b02_rollout_cost_fn,
)
from projects.B_mujoco_mpc_study.simulator.planners.mpc_solver_interface import MPCSolution
from projects.B_mujoco_mpc_study.simulator.utils.solver_benchmark_logger import SolverBenchmarkLogBuffer
from projects.B_mujoco_mpc_study.simulator.utils.solver_comparison_visualizer import (
    compute_control_smoothness,
    compute_trajectory_smoothness,
)
from projects.B_mujoco_mpc_study.simulator.utils.sampling_tuning_reporter import (
    compute_tuning_score,
    plot_cem_parameter_sweep,
    plot_mppi_temperature_sweep,
    plot_tuning_score_comparison,
    plot_warm_start_smoothness_sweep,
    save_sweep_metrics_csv,
    select_best_variants,
    write_recommended_solver_report,
    write_tuning_report,
)
from projects.B_mujoco_mpc_study.simulator.utils.solver_comparison_visualizer import (
    plot_control_smoothness_comparison,
    plot_solver_error_comparison,
    plot_solver_runtime_comparison,
    plot_trajectory_smoothness_comparison,
)
from projects.B_mujoco_mpc_study.simulator.utils.solver_benchmark_logger import (
    build_comparison_metrics_rows,
    save_comparison_metrics_csv,
)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="B03-R3T sampling solver automated tuning.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH, help="Tuning YAML config path.")
    parser.add_argument("--run-id", type=str, default=None, help="Optional run id.")
    parser.add_argument("--solvers", nargs="*", default=None, help="Limit to specific solver families.")
    parser.add_argument("--max-variants", type=int, default=None, help="Max number of variants to run.")
    parser.add_argument("--num-steps", type=int, default=None, help="Override closed-loop step count.")
    parser.add_argument("--horizon", type=int, default=None, help="Override horizon.")
    parser.add_argument("--seed", type=int, default=None, help="Override random seed.")
    parser.add_argument("--save-figures", action=argparse.BooleanOptionalAction, default=None, help="Save PNG figures.")
    parser.add_argument("--save-report", action=argparse.BooleanOptionalAction, default=None, help="Save markdown report.")
    parser.add_argument("--show-viewer", action="store_true", default=False, help="Show live MuJoCo viewer window.")
    parser.add_argument("--recommended-benchmark", action="store_true", default=False, help="Run recommended benchmark with the four recommended solver configs.")
    parser.add_argument("--save-recommended-report", action=argparse.BooleanOptionalAction, default=None, help="Save recommended solver report markdown.")
    parser.add_argument("--log-level", type=str, default="INFO", help="Python logging level.")
    return parser


def _build_variant_list(
    sweep_config: dict[str, Any],
    enabled_solvers: list[str] | None,
) -> list[dict[str, Any]]:
    """Build the full list of variant configs from sweep section."""
    variants: list[dict[str, Any]] = []
    for solver_family in sweep_config.get("enabled_solvers", []):
        if enabled_solvers and solver_family not in enabled_solvers:
            continue
        family_variants = sweep_config.get(solver_family, [])
        for variant in family_variants:
            variants.append({
                "solver_family": solver_family,
                "variant_name": variant.get("name", solver_family),
                "solver_config": {k: v for k, v in variant.items() if k != "name"},
            })
    return variants


def _build_variant_runtime_config(
    base_runtime_config: dict[str, Any],
    variant: dict[str, Any],
) -> dict[str, Any]:
    """Build a runtime_config for a specific variant."""
    rc = deepcopy(base_runtime_config)
    solver_family = variant["solver_family"]
    variant_cfg = variant["solver_config"]

    # Merge variant params into the solver config
    if solver_family in rc["solver_configs"]:
        rc["solver_configs"][solver_family] = {
            **rc["solver_configs"][solver_family],
            **variant_cfg,
        }
    else:
        rc["solver_configs"][solver_family] = dict(variant_cfg)

    # For warm_start_sampling, pass through torque_rate_weight via cost config
    if "torque_rate_weight" in variant_cfg:
        rc["cost"]["torque_rate_weight"] = float(variant_cfg["torque_rate_weight"])

    return rc


def _run_variant(
    *,
    variant_name: str,
    solver_family: str,
    runtime_config: dict[str, Any],
    run_dir: Path,
    show_viewer: bool = False,
) -> dict[str, Any]:
    """Run a single variant and return its metrics."""
    env = create_b03_two_link_env(runtime_config)
    solver = build_b03_solver(solver_family, runtime_config, env)
    adapter_config = build_adapter_config(runtime_config)
    rollout_cost_fn = make_b02_rollout_cost_fn(env=env, adapter_config=adapter_config)

    initial_q, initial_dq = _initial_state_from_runtime_config(runtime_config)
    env.reset(q=initial_q, dq=initial_dq)

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

    benchmark_logger = SolverBenchmarkLogBuffer()
    previous_solution: MPCSolution | None = None
    control_history: list[np.ndarray] = []
    target_history: list[np.ndarray] = []
    ee_history: list[np.ndarray] = []
    best_cost_history: list[float] = []
    time_history: list[float] = []

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
            solver_config=deepcopy(runtime_config["solver_configs"].get(solver_family, {})),
            rollout_cost_fn=rollout_cost_fn,
        )

        solution = solver.solve(problem, previous_solution=previous_solution)
        previous_solution = solution

        env.step(tuple(float(value) for value in solution.first_control.tolist()))
        applied_torque = np.asarray(env.get_last_applied_torque(), dtype=float)
        actual_xy = np.asarray(env.get_end_effector_position(), dtype=float)
        target_xy = np.asarray(target_horizon[0], dtype=float)
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
            solver_name=variant_name,
            final_ee_error=float(np.linalg.norm(actual_xy - target_xy)),
            current_ee_error=float(np.linalg.norm(actual_xy - target_xy)),
            best_cost=float(solution.best_cost),
            runtime_ms=float(solution.solver_stats.runtime_ms),
            num_rollouts=int(solution.solver_stats.num_rollouts),
            num_iterations=int(solution.solver_stats.num_iterations),
            max_abs_torque=float(np.max(np.abs(applied_torque))),
            control_smoothness=compute_control_smoothness(np.asarray(control_history, dtype=float)),
            trajectory_smoothness=compute_trajectory_smoothness(np.asarray(ee_history, dtype=float)),
            success=bool(solution.solver_stats.success),
        )

    if viewer is not None:
        viewer.close()

    step_rows = benchmark_logger.step_rows()
    summary_row = _compute_summary_row(variant_name, step_rows)

    # Extract weight diagnostics from MPPI metadata
    weight_entropy = float("nan")
    max_weight = float("nan")
    if previous_solution is not None and hasattr(previous_solution, "metadata"):
        weight_entropy = float(previous_solution.metadata.get("weight_entropy", float("nan")))
        max_weight = float(previous_solution.metadata.get("max_weight", float("nan")))

    return {
        "variant_name": variant_name,
        "solver_family": solver_family,
        "summary_row": summary_row,
        "step_rows": step_rows,
        "weight_entropy": weight_entropy,
        "max_weight": max_weight,
        "solver_config": runtime_config["solver_configs"].get(solver_family, {}),
    }


def _build_variant_metrics_row(variant_result: dict[str, Any]) -> dict[str, Any]:
    """Build a flat metrics dict from a variant result."""
    sr = variant_result["summary_row"]
    return {
        "variant_name": variant_result["variant_name"],
        "solver_family": variant_result["solver_family"],
        "mean_ee_error": sr.mean_ee_error,
        "max_ee_error": sr.max_ee_error,
        "mean_runtime_ms": sr.runtime_per_control_step,
        "max_runtime_ms": sr.runtime_per_control_step,  # approx
        "control_smoothness": sr.control_smoothness,
        "trajectory_smoothness": sr.trajectory_smoothness,
        "success_rate": sr.solver_success_rate,
        "weight_entropy": variant_result.get("weight_entropy", float("nan")),
        "max_weight": variant_result.get("max_weight", float("nan")),
    }


def _write_run_log(
    output_path: Path,
    *,
    config: dict[str, Any],
    scored_rows: list[dict[str, Any]],
    best_variants: dict[str, Any],
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        f"task_name: {TASK_NAME}",
        f"num_variants: {len(scored_rows)}",
        f"feasible: {sum(1 for r in scored_rows if r.get('is_feasible', False))}",
        "",
        "best_variants:",
    ]
    for key, bv in best_variants.items():
        if bv is None:
            lines.append(f"- {key}: none")
        else:
            lines.append(f"- {key}: {bv.get('variant_name', '?')} (score={bv.get('score', float('nan')):.4f})")
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_tuning(args: argparse.Namespace) -> dict[str, Any]:
    """Main tuning entry point."""
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))

    if not args.config.exists():
        raise FileNotFoundError(f"Cannot find tuning config: {args.config}")

    with args.config.open("r", encoding="utf-8") as f:
        tuning_config = yaml.safe_load(f) or {}

    base_config_path = PROJECT_ROOT / tuning_config.get("base", {}).get("source_config", "").replace("projects/B_mujoco_mpc_study/", "")
    if not base_config_path.exists():
        # Try relative to project root
        base_config_path = PROJECT_ROOT / ".." / tuning_config.get("base", {}).get("source_config", "")
        base_config_path = base_config_path.resolve()

    if not base_config_path.exists():
        # Fallback to default B03 config
        base_config_path = PROJECT_ROOT / "configs" / "B03_mpc_solver_ladder.yaml"

    base_config = load_b03_config(base_config_path)

    # Build a minimal runtime config from base
    simulation = base_config.get("simulation", {})
    sampling = base_config.get("sampling", {})
    target = tuning_config.get("target", base_config.get("target", {}))
    cost = base_config.get("cost", {})
    cem = base_config.get("cem", {})
    mppi = base_config.get("mppi", {})

    num_steps = int(args.num_steps if args.num_steps is not None else tuning_config.get("simulation", {}).get("num_steps", 100))
    horizon = int(args.horizon if args.horizon is not None else tuning_config.get("simulation", {}).get("horizon", 16))
    seed = int(args.seed if args.seed is not None else tuning_config.get("base", {}).get("seed", 0))

    base_runtime_config: dict[str, Any] = {
        "model_family": simulation.get("model_family", "two_link"),
        "num_steps": num_steps,
        "dt": float(simulation.get("dt", 0.01)),
        "seed": seed,
        "horizon": horizon,
        "num_candidates": int(sampling.get("num_candidates", 256)),
        "control_dim": int(sampling.get("control_dim", 2)),
        "torque_limit": float(sampling.get("torque_limit", 10.0)),
        "sampling_std": float(sampling.get("sampling_std", 2.0)),
        "target_type": target.get("type", "circle"),
        "target_center": _as_pair(target.get("center", (0.45, 0.10)), "target.center"),
        "target_radius": float(target.get("radius", 0.08)),
        "target_frequency": float(target.get("frequency", 0.2)),
        "export_video": False,
        "save_figures": bool(args.save_figures if args.save_figures is not None else False),
        "save_metrics": True,
        "show_viewer": bool(args.show_viewer),
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
        "solver_configs": {
            "random_shooting": {
                "num_candidates": int(sampling.get("num_candidates", 256)),
                "sampling_std": float(sampling.get("sampling_std", 2.0)),
                "torque_limit": float(sampling.get("torque_limit", 10.0)),
                "seed": seed,
            },
            "warm_start_sampling": {
                "num_candidates": int(sampling.get("num_candidates", 256)),
                "sampling_std": float(sampling.get("sampling_std", 2.0)),
                "torque_limit": float(sampling.get("torque_limit", 10.0)),
                "seed": seed,
            },
            "cem": {
                "num_candidates": int(cem.get("num_candidates", 256)),
                "sampling_std": float(cem.get("initial_std", 2.0)),
                "initial_std": float(cem.get("initial_std", 2.0)),
                "torque_limit": float(cem.get("torque_limit", 10.0)),
                "seed": seed,
                "num_iterations": int(cem.get("num_iterations", 3)),
                "elite_ratio": float(cem.get("elite_ratio", 0.1)),
                "min_std": float(cem.get("min_std", 0.1)),
                "smoothing_alpha": float(cem.get("smoothing_alpha", 0.2)),
            },
            "mppi_lite": {
                "num_candidates": int(mppi.get("num_candidates", 256)),
                "sampling_std": float(mppi.get("noise_std", 2.0)),
                "noise_std": float(mppi.get("noise_std", 2.0)),
                "torque_limit": float(mppi.get("torque_limit", 10.0)),
                "seed": seed,
                "num_iterations": int(mppi.get("num_iterations", 3)),
                "temperature": float(mppi.get("temperature", 1.0)),
                "min_std": float(mppi.get("min_std", 0.1)),
                "smoothing_alpha": float(mppi.get("smoothing_alpha", 0.2)),
            },
        },
    }

    # Build variant list
    sweep_config = tuning_config.get("sweep", {})
    enabled_solvers = list(args.solvers) if args.solvers else None
    all_variants = _build_variant_list(sweep_config, enabled_solvers)

    if args.max_variants is not None and args.max_variants > 0:
        all_variants = all_variants[: args.max_variants]

    # Output dirs
    actual_run_id = args.run_id or datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = PROJECT_ROOT / "outputs" / "runs" / TASK_NAME / actual_run_id
    outputs = {
        "run_dir": run_dir,
        "metrics": run_dir / "metrics",
        "figures": run_dir / "figures",
        "reports": run_dir / "reports",
        "logs": run_dir / "logs",
        "sweep_csv": run_dir / "metrics" / "B03_parameter_sweep_metrics.csv",
        "run_log": run_dir / "logs" / "B03_sampling_tuning_run_log.txt",
        "cem_figure": run_dir / "figures" / "B03_cem_parameter_sweep.png",
        "mppi_figure": run_dir / "figures" / "B03_mppi_temperature_sweep.png",
        "ws_figure": run_dir / "figures" / "B03_warm_start_smoothness_sweep.png",
        "score_figure": run_dir / "figures" / "B03_tuning_score_comparison.png",
        "report_md": run_dir / "reports" / "B03_sampling_parameter_tuning_report.md",
    }
    for v in outputs.values():
        if isinstance(v, Path):
            if v.suffix:
                v.parent.mkdir(parents=True, exist_ok=True)
            else:
                v.mkdir(parents=True, exist_ok=True)

    logging.info("B03-R3T tuning prepared %d variants at %s", len(all_variants), run_dir)

    # Run each variant
    variant_results: list[dict[str, Any]] = []
    for idx, variant in enumerate(all_variants):
        logging.info(
            "Running variant %d/%d: %s (%s)",
            idx + 1, len(all_variants),
            variant["variant_name"], variant["solver_family"],
        )
        variant_runtime = _build_variant_runtime_config(base_runtime_config, variant)
        result = _run_variant(
            variant_name=variant["variant_name"],
            solver_family=variant["solver_family"],
            runtime_config=variant_runtime,
            run_dir=run_dir,
            show_viewer=bool(base_runtime_config.get("show_viewer", False)),
        )
        variant_results.append(result)

    # Build metrics rows
    metrics_rows = [_build_variant_metrics_row(r) for r in variant_results]

    # Score
    weights = tuning_config.get("selection", {}).get("objective", {})
    constraints = tuning_config.get("selection", {}).get("constraints", {})

    scored_rows: list[dict[str, Any]] = []
    for row in metrics_rows:
        score, is_feasible, reason = compute_tuning_score(row, weights, constraints)
        scored_rows.append({**row, "score": score, "is_feasible": is_feasible, "feasibility_reason": reason})

    # Save sweep metrics CSV
    save_sweep_metrics_csv(scored_rows, outputs["sweep_csv"])
    logging.info("Saved sweep metrics to %s", outputs["sweep_csv"])

    # Select best variants
    best_variants = select_best_variants(scored_rows)

    # Generate plots
    save_figs = bool(args.save_figures if args.save_figures is not None else True)
    if save_figs:
        cem_rows = [r for r in scored_rows if r.get("solver_family") == "cem"]
        mppi_rows = [r for r in scored_rows if r.get("solver_family") == "mppi_lite"]
        ws_rows = [r for r in scored_rows if r.get("solver_family") == "warm_start_sampling"]

        if cem_rows:
            plot_cem_parameter_sweep(cem_rows, outputs["cem_figure"])
        if mppi_rows:
            plot_mppi_temperature_sweep(mppi_rows, outputs["mppi_figure"])
        if ws_rows:
            plot_warm_start_smoothness_sweep(ws_rows, outputs["ws_figure"])
        plot_tuning_score_comparison(scored_rows, outputs["score_figure"])
        logging.info("Saved tuning figures to %s", outputs["figures"])

    # Generate report
    save_rpt = bool(args.save_report if args.save_report is not None else True)
    if save_rpt:
        write_tuning_report(scored_rows, best_variants, weights, constraints, outputs["report_md"])
        logging.info("Saved tuning report to %s", outputs["report_md"])

    # Run log
    _write_run_log(
        outputs["run_log"],
        config=tuning_config,
        scored_rows=scored_rows,
        best_variants=best_variants,
    )

    # Print summary
    bv = best_variants.get("best_overall")
    status = "B03-R3T tuning completed."
    if bv is not None:
        status += f" Best overall: {bv.get('variant_name', '?')} (score={bv.get('score', float('nan')):.4f})"
    print(status)
    print(f"run_dir: {run_dir}")
    print(f"variants_run: {len(all_variants)}")
    feasible_count = sum(1 for r in scored_rows if r.get("is_feasible", False))
    print(f"feasible: {feasible_count}/{len(scored_rows)}")

    return {
        "task_name": TASK_NAME,
        "run_dir": run_dir,
        "outputs": outputs,
        "scored_rows": scored_rows,
        "best_variants": best_variants,
        "status": status,
    }


# ---------------------------------------------------------------------------
# Recommended benchmark
# ---------------------------------------------------------------------------

_SOLVER_NAME_TO_FAMILY: dict[str, str] = {
    "random_shooting": "random_shooting",
    "warm_start_fast": "warm_start_sampling",
    "cem_default": "cem",
    "mppi_smooth": "mppi_lite",
}


def run_recommended_benchmark(args: argparse.Namespace) -> dict[str, Any]:
    """Run the four recommended sampling solver configs as a short benchmark."""
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))

    config_path = args.config if args.config != DEFAULT_CONFIG_PATH else DEFAULT_RECOMMENDED_CONFIG_PATH
    if not config_path.exists():
        raise FileNotFoundError(f"Cannot find recommended benchmark config: {config_path}")

    with config_path.open("r", encoding="utf-8") as f:
        bench_config = yaml.safe_load(f) or {}

    simulation = bench_config.get("simulation", {})
    target = bench_config.get("target", {})
    configs_section = bench_config.get("configs", {})

    num_steps = int(args.num_steps if args.num_steps is not None else simulation.get("num_steps", 100))
    horizon = int(args.horizon if args.horizon is not None else simulation.get("horizon", 16))
    seed = int(args.seed if args.seed is not None else simulation.get("seed", 0))
    dt = float(simulation.get("dt", 0.01))

    enabled_solvers = bench_config.get("solvers", {}).get("enabled", [])
    if args.solvers:
        enabled_solvers = [s for s in enabled_solvers if s in args.solvers]

    actual_run_id = args.run_id or datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = PROJECT_ROOT / "outputs" / "runs" / RECOMMENDED_TASK_NAME / actual_run_id
    figures_dir = run_dir / "figures"
    metrics_dir = run_dir / "metrics"
    reports_dir = run_dir / "reports"
    logs_dir = run_dir / "logs"
    for d in (figures_dir, metrics_dir, reports_dir, logs_dir):
        d.mkdir(parents=True, exist_ok=True)

    base_runtime_config: dict[str, Any] = {
        "model_family": "two_link",
        "num_steps": num_steps,
        "dt": dt,
        "seed": seed,
        "horizon": horizon,
        "num_candidates": 128,
        "control_dim": 2,
        "torque_limit": 10.0,
        "sampling_std": 2.0,
        "target_type": target.get("type", "circle"),
        "target_center": _as_pair(target.get("center", (0.45, 0.10)), "target.center"),
        "target_radius": float(target.get("radius", 0.08)),
        "target_frequency": float(target.get("frequency", 0.2)),
        "export_video": False,
        "save_figures": False,
        "save_metrics": True,
        "show_viewer": bool(args.show_viewer),
        "initial_q": (0.3, 0.4),
        "initial_dq": (0.0, 0.0),
        "model_path": DEFAULT_TWO_LINK_MODEL_PATH,
        "end_effector_site": DEFAULT_END_EFFECTOR_SITE,
        "cost": {
            "ee_weight": 80.0,
            "dq_weight": 0.1,
            "torque_weight": 0.002,
            "terminal_weight": 10.0,
            "smoothness_weight": 0.01,
        },
        "solver_configs": {},
    }

    logging.info("Recommended benchmark: %d solvers at %s", len(enabled_solvers), run_dir)

    all_variant_results: list[dict[str, Any]] = []
    step_rows_by_solver: dict[str, list] = {}

    for solver_name in enabled_solvers:
        solver_family = _SOLVER_NAME_TO_FAMILY.get(solver_name)
        if solver_family is None:
            logging.warning("Unknown solver name '%s', skipping.", solver_name)
            continue

        solver_cfg = dict(configs_section.get(solver_name, {}))

        # Build a variant-like config
        variant = {
            "solver_family": solver_family,
            "variant_name": solver_name,
            "solver_config": solver_cfg,
        }

        variant_runtime = _build_variant_runtime_config(base_runtime_config, variant)
        # For warm_start_fast, pass torque_rate_weight through cost
        if "torque_rate_weight" in solver_cfg:
            variant_runtime["cost"]["torque_rate_weight"] = float(solver_cfg["torque_rate_weight"])

        logging.info("Running solver: %s (family=%s)", solver_name, solver_family)
        result = _run_variant(
            variant_name=solver_name,
            solver_family=solver_family,
            runtime_config=variant_runtime,
            run_dir=run_dir,
            show_viewer=bool(args.show_viewer),
        )
        all_variant_results.append(result)
        step_rows_by_solver[solver_name] = result["step_rows"]

    # Save comparison metrics CSV
    comparison_rows = build_comparison_metrics_rows(step_rows_by_solver)
    comparison_csv_path = metrics_dir / "B03_recommended_comparison_metrics.csv"
    save_comparison_metrics_csv(comparison_rows, comparison_csv_path)
    logging.info("Saved comparison metrics to %s", comparison_csv_path)

    # Save comparison figures
    save_figs = bool(args.save_figures if args.save_figures is not None else True)
    if save_figs and step_rows_by_solver:
        plot_solver_error_comparison(
            step_rows_by_solver,
            figures_dir / "B03_recommended_error_comparison.png",
        )
        plot_solver_runtime_comparison(
            step_rows_by_solver,
            figures_dir / "B03_recommended_runtime_comparison.png",
        )
        plot_control_smoothness_comparison(
            step_rows_by_solver,
            figures_dir / "B03_recommended_control_smoothness.png",
        )
        plot_trajectory_smoothness_comparison(
            step_rows_by_solver,
            figures_dir / "B03_recommended_trajectory_smoothness.png",
        )
        logging.info("Saved recommended figures to %s", figures_dir)

    # Build metrics rows for report
    metrics_rows = [_build_variant_metrics_row(r) for r in all_variant_results]

    # Save recommended report
    save_rpt = bool(args.save_recommended_report if args.save_recommended_report is not None else True)
    if save_rpt:
        report_path = reports_dir / "B03_recommended_sampling_solver_report.md"
        write_recommended_solver_report(
            metrics_rows=metrics_rows,
            output_path=report_path,
            num_steps=num_steps,
            horizon=horizon,
        )
        logging.info("Saved recommended report to %s", report_path)

    # Run log
    run_log_path = logs_dir / "B03_recommended_benchmark_run_log.txt"
    lines = [
        f"task_name: {RECOMMENDED_TASK_NAME}",
        f"num_solvers: {len(all_variant_results)}",
        f"num_steps: {num_steps}",
        f"horizon: {horizon}",
        f"seed: {seed}",
        "",
    ]
    for r in all_variant_results:
        sr = r["summary_row"]
        lines.append(f"- {r['variant_name']}: mean_ee_error={sr.mean_ee_error:.6f}, runtime={sr.runtime_per_control_step:.1f}ms")
    run_log_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(f"Recommended benchmark completed. run_dir: {run_dir}")
    print(f"solvers_run: {len(all_variant_results)}")
    for r in all_variant_results:
        sr = r["summary_row"]
        print(f"  {r['variant_name']}: error={sr.mean_ee_error:.6f}, runtime={sr.runtime_per_control_step:.1f}ms")

    return {
        "task_name": RECOMMENDED_TASK_NAME,
        "run_dir": run_dir,
        "variant_results": all_variant_results,
    }


def main(argv: list[str] | None = None) -> None:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    if args.recommended_benchmark:
        run_recommended_benchmark(args)
    else:
        run_tuning(args)


if __name__ == "__main__":
    main()
