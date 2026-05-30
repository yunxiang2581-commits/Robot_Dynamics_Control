"""B03-R3T sampling solver automated tuning reporter.

Pure functions for scoring, selecting and plotting parameter sweep results.
Also includes recommended solver report generation.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


# ---------------------------------------------------------------------------
# Scoring
# ---------------------------------------------------------------------------

def compute_tuning_score(
    row: dict[str, Any],
    weights: dict[str, float],
    constraints: dict[str, float],
) -> tuple[float, bool, str]:
    """Compute a weighted tuning score for a single variant.

    Returns (score, is_feasible, reason).
    Score is lower-is-better.
    """
    mean_ee_error = float(row.get("mean_ee_error", float("nan")))
    mean_runtime_ms = float(row.get("mean_runtime_ms", float("nan")))
    control_smoothness = float(row.get("control_smoothness", float("nan")))
    trajectory_smoothness = float(row.get("trajectory_smoothness", float("nan")))
    success_rate = float(row.get("success_rate", float("nan")))

    # Feasibility checks
    max_runtime = float(constraints.get("max_mean_runtime_ms", 500.0))
    max_error = float(constraints.get("max_mean_ee_error", 0.05))
    min_success = float(constraints.get("min_success_rate", 0.95))

    if not math.isfinite(success_rate) or success_rate < min_success:
        return float("inf"), False, f"success_rate={success_rate:.3f} < {min_success}"
    if not math.isfinite(mean_runtime_ms) or mean_runtime_ms > max_runtime:
        return float("inf"), False, f"mean_runtime_ms={mean_runtime_ms:.1f} > {max_runtime}"
    if not math.isfinite(mean_ee_error) or mean_ee_error > max_error:
        return float("inf"), False, f"mean_ee_error={mean_ee_error:.6f} > {max_error}"

    # Normalised score components (each ~ 0..1 range)
    w_error = float(weights.get("mean_ee_error_weight", 1.0))
    w_runtime = float(weights.get("runtime_weight", 0.2))
    w_ctrl = float(weights.get("control_smoothness_weight", 0.5))
    w_traj = float(weights.get("trajectory_smoothness_weight", 0.5))

    error_term = w_error * mean_ee_error / max_error if max_error > 0 else 0.0
    runtime_term = w_runtime * mean_runtime_ms / max_runtime if max_runtime > 0 else 0.0
    ctrl_term = w_ctrl * control_smoothness if math.isfinite(control_smoothness) else 0.0
    traj_term = w_traj * trajectory_smoothness if math.isfinite(trajectory_smoothness) else 0.0

    score = error_term + runtime_term + ctrl_term + traj_term
    return float(score), True, "feasible"


# ---------------------------------------------------------------------------
# Best-variant selection
# ---------------------------------------------------------------------------

def _family_from_variant_name(variant_name: str) -> str:
    """Extract solver family prefix from variant name."""
    if variant_name.startswith("cem"):
        return "cem"
    if variant_name.startswith("mppi"):
        return "mppi_lite"
    if variant_name.startswith("warm_start"):
        return "warm_start_sampling"
    return variant_name


def select_best_variants(
    scored_rows: list[dict[str, Any]],
) -> dict[str, dict[str, Any] | None]:
    """Select best variant overall and per solver family.

    Input: list of dicts each containing at least 'variant_name', 'solver_family', 'score', 'is_feasible'.
    """
    feasible = [r for r in scored_rows if r.get("is_feasible", False)]
    if not feasible:
        return {"best_overall": None, "best_cem": None, "best_mppi": None, "best_warm_start": None}

    best_overall = min(feasible, key=lambda r: float(r.get("score", float("inf"))))

    def _best_of(prefix: str) -> dict[str, Any] | None:
        candidates = [r for r in feasible if _family_from_variant_name(str(r.get("variant_name", ""))) == prefix]
        if not candidates:
            return None
        return min(candidates, key=lambda r: float(r.get("score", float("inf"))))

    return {
        "best_overall": best_overall,
        "best_cem": _best_of("cem"),
        "best_mppi": _best_of("mppi_lite"),
        "best_warm_start": _best_of("warm_start_sampling"),
    }


# ---------------------------------------------------------------------------
# Plotting helpers
# ---------------------------------------------------------------------------

def _save_figure(fig: Any, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=120, bbox_inches="tight")
    plt.close(fig)


def _bar_group(
    ax: Any,
    labels: list[str],
    values: list[float],
    ylabel: str,
    title: str,
    color: str = "steelblue",
) -> None:
    x = np.arange(len(labels))
    ax.bar(x, values, color=color, alpha=0.85)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=30, ha="right", fontsize=8)
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=10)
    ax.grid(axis="y", alpha=0.3)


# ---------------------------------------------------------------------------
# CEM sweep plot
# ---------------------------------------------------------------------------

def plot_cem_parameter_sweep(
    cem_rows: list[dict[str, Any]],
    output_path: Path,
) -> None:
    """Bar chart comparing CEM variants on key metrics."""
    if not cem_rows:
        return

    labels = [str(r.get("variant_name", r.get("solver_name", "?"))) for r in cem_rows]
    errors = [float(r.get("mean_ee_error", 0.0)) for r in cem_rows]
    runtimes = [float(r.get("mean_runtime_ms", 0.0)) for r in cem_rows]
    ctrl = [float(r.get("control_smoothness", 0.0)) for r in cem_rows]
    traj = [float(r.get("trajectory_smoothness", 0.0)) for r in cem_rows]

    fig, axes = plt.subplots(2, 2, figsize=(10, 7))
    _bar_group(axes[0, 0], labels, errors, "mean_ee_error", "CEM - EE Error")
    _bar_group(axes[0, 1], labels, runtimes, "mean_runtime_ms", "CEM - Runtime (ms)")
    _bar_group(axes[1, 0], labels, ctrl, "control_smoothness", "CEM - Control Smoothness")
    _bar_group(axes[1, 1], labels, traj, "trajectory_smoothness", "CEM - Trajectory Smoothness")
    fig.suptitle("CEM Parameter Sweep", fontsize=12, fontweight="bold")
    fig.tight_layout()
    _save_figure(fig, output_path)


# ---------------------------------------------------------------------------
# MPPI temperature sweep plot
# ---------------------------------------------------------------------------

def plot_mppi_temperature_sweep(
    mppi_rows: list[dict[str, Any]],
    output_path: Path,
) -> None:
    """Bar chart comparing MPPI-lite temperature variants."""
    if not mppi_rows:
        return

    labels = [str(r.get("variant_name", r.get("solver_name", "?"))) for r in mppi_rows]
    errors = [float(r.get("mean_ee_error", 0.0)) for r in mppi_rows]
    ctrl = [float(r.get("control_smoothness", 0.0)) for r in mppi_rows]
    traj = [float(r.get("trajectory_smoothness", 0.0)) for r in mppi_rows]
    entropy = [float(r.get("weight_entropy", 0.0)) for r in mppi_rows]
    max_w = [float(r.get("max_weight", 0.0)) for r in mppi_rows]

    fig, axes = plt.subplots(2, 3, figsize=(13, 7))
    _bar_group(axes[0, 0], labels, errors, "mean_ee_error", "MPPI - EE Error")
    _bar_group(axes[0, 1], labels, ctrl, "control_smoothness", "MPPI - Control Smoothness")
    _bar_group(axes[0, 2], labels, traj, "trajectory_smoothness", "MPPI - Trajectory Smoothness")
    _bar_group(axes[1, 0], labels, entropy, "weight_entropy", "MPPI - Weight Entropy", color="darkorange")
    _bar_group(axes[1, 1], labels, max_w, "max_weight", "MPPI - Max Weight", color="seagreen")
    axes[1, 2].axis("off")
    fig.suptitle("MPPI-lite Temperature Sweep", fontsize=12, fontweight="bold")
    fig.tight_layout()
    _save_figure(fig, output_path)


# ---------------------------------------------------------------------------
# Warm-start smoothness sweep plot
# ---------------------------------------------------------------------------

def plot_warm_start_smoothness_sweep(
    ws_rows: list[dict[str, Any]],
    output_path: Path,
) -> None:
    """Bar chart comparing warm-start smoothness variants."""
    if not ws_rows:
        return

    labels = [str(r.get("variant_name", r.get("solver_name", "?"))) for r in ws_rows]
    errors = [float(r.get("mean_ee_error", 0.0)) for r in ws_rows]
    ctrl = [float(r.get("control_smoothness", 0.0)) for r in ws_rows]
    traj = [float(r.get("trajectory_smoothness", 0.0)) for r in ws_rows]

    fig, axes = plt.subplots(1, 3, figsize=(12, 4))
    _bar_group(axes[0], labels, errors, "mean_ee_error", "Warm-start - EE Error")
    _bar_group(axes[1], labels, ctrl, "control_smoothness", "Warm-start - Control Smoothness")
    _bar_group(axes[2], labels, traj, "trajectory_smoothness", "Warm-start - Trajectory Smoothness")
    fig.suptitle("Warm-start Smoothness Sweep", fontsize=12, fontweight="bold")
    fig.tight_layout()
    _save_figure(fig, output_path)


# ---------------------------------------------------------------------------
# Tuning score comparison plot
# ---------------------------------------------------------------------------

def plot_tuning_score_comparison(
    scored_rows: list[dict[str, Any]],
    output_path: Path,
) -> None:
    """Bar chart of tuning scores for all feasible variants."""
    feasible = [r for r in scored_rows if r.get("is_feasible", False)]
    if not feasible:
        return

    feasible_sorted = sorted(feasible, key=lambda r: float(r.get("score", float("inf"))))
    labels = [str(r.get("variant_name", "?")) for r in feasible_sorted]
    scores = [float(r.get("score", 0.0)) for r in feasible_sorted]

    fig, ax = plt.subplots(figsize=(10, 5))
    colors = ["seagreen" if _family_from_variant_name(l) == "cem"
              else "steelblue" if _family_from_variant_name(l) == "mppi_lite"
              else "darkorange"
              for l in labels]
    _bar_group(ax, labels, scores, "tuning_score (lower is better)", "Tuning Score Comparison", color="steelblue")
    ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=8)
    fig.suptitle("All Variants - Tuning Score (lower is better)", fontsize=12, fontweight="bold")
    fig.tight_layout()
    _save_figure(fig, output_path)


# ---------------------------------------------------------------------------
# Tuning report writer
# ---------------------------------------------------------------------------

def write_tuning_report(
    scored_rows: list[dict[str, Any]],
    best_variants: dict[str, dict[str, Any] | None],
    weights: dict[str, float],
    constraints: dict[str, float],
    output_path: Path,
) -> None:
    """Write a markdown tuning report."""
    output_path.parent.mkdir(parents=True, exist_ok=True)

    lines: list[str] = []
    lines.append("# B03-R3T Sampling Solver Parameter Tuning Report\n")
    lines.append("## 1. Tuning Objective\n")
    lines.append("Automated small-scale parameter sweep for CEM, MPPI-lite and Warm-start Sampling.\n")
    lines.append(f"- Error weight: {weights.get('mean_ee_error_weight', 1.0)}")
    lines.append(f"- Runtime weight: {weights.get('runtime_weight', 0.2)}")
    lines.append(f"- Control smoothness weight: {weights.get('control_smoothness_weight', 0.5)}")
    lines.append(f"- Trajectory smoothness weight: {weights.get('trajectory_smoothness_weight', 0.5)}")
    lines.append("")
    lines.append("## 2. Constraints\n")
    lines.append(f"- max_mean_runtime_ms: {constraints.get('max_mean_runtime_ms', 500.0)}")
    lines.append(f"- max_mean_ee_error: {constraints.get('max_mean_ee_error', 0.05)}")
    lines.append(f"- min_success_rate: {constraints.get('min_success_rate', 0.95)}")
    lines.append("")

    # Variant table
    lines.append("## 3. All Variants\n")
    lines.append("| variant | solver_family | mean_ee_error | mean_runtime_ms | ctrl_smooth | traj_smooth | score | feasible |")
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- |")
    for r in scored_rows:
        name = r.get("variant_name", "?")
        family = r.get("solver_family", "?")
        err = r.get("mean_ee_error", float("nan"))
        rt = r.get("mean_runtime_ms", float("nan"))
        cs = r.get("control_smoothness", float("nan"))
        ts = r.get("trajectory_smoothness", float("nan"))
        sc = r.get("score", float("inf"))
        feas = "yes" if r.get("is_feasible", False) else "no"
        err_s = f"{err:.6f}" if math.isfinite(float(err)) else "NaN"
        rt_s = f"{rt:.1f}" if math.isfinite(float(rt)) else "NaN"
        cs_s = f"{cs:.6f}" if math.isfinite(float(cs)) else "NaN"
        ts_s = f"{ts:.6f}" if math.isfinite(float(ts)) else "NaN"
        sc_s = f"{sc:.4f}" if math.isfinite(float(sc)) else "inf"
        lines.append(f"| {name} | {family} | {err_s} | {rt_s} | {cs_s} | {ts_s} | {sc_s} | {feas} |")
    lines.append("")

    # Best variants
    lines.append("## 4. Best Variants\n")
    for key, label in [("best_overall", "Overall"), ("best_cem", "CEM"), ("best_mppi", "MPPI-lite"), ("best_warm_start", "Warm-start")]:
        bv = best_variants.get(key)
        if bv is None:
            lines.append(f"- **{label}**: no feasible variant found")
        else:
            lines.append(f"- **{label}**: {bv.get('variant_name', '?')} (score={bv.get('score', float('nan')):.4f})")
    lines.append("")

    # Per-family conclusions
    lines.append("## 5. CEM Conclusions\n")
    cem_rows = [r for r in scored_rows if _family_from_variant_name(str(r.get("variant_name", ""))) == "cem"]
    cem_feasible = [r for r in cem_rows if r.get("is_feasible", False)]
    if cem_feasible:
        best_cem = min(cem_feasible, key=lambda r: float(r.get("score", float("inf"))))
        lines.append(f"Best CEM variant: **{best_cem.get('variant_name', '?')}**")
        lines.append(f"- elite_ratio and num_candidates trade off search quality vs runtime.")
        lines.append(f"- Early stopping can reduce runtime without sacrificing accuracy.")
    else:
        lines.append("No feasible CEM variant found. Consider relaxing constraints.")
    lines.append("")

    lines.append("## 6. MPPI-lite Conclusions\n")
    mppi_rows = [r for r in scored_rows if _family_from_variant_name(str(r.get("variant_name", ""))) == "mppi_lite"]
    mppi_feasible = [r for r in mppi_rows if r.get("is_feasible", False)]
    if mppi_feasible:
        best_mppi = min(mppi_feasible, key=lambda r: float(r.get("score", float("inf"))))
        lines.append(f"Best MPPI-lite variant: **{best_mppi.get('variant_name', '?')}**")
        lines.append(f"- Temperature controls weight concentration; lower temp = sharper weights.")
        lines.append(f"- Check weight_entropy and max_weight for diagnostics.")
    else:
        lines.append("No feasible MPPI-lite variant found. Consider relaxing constraints.")
    lines.append("")

    lines.append("## 7. Warm-start Conclusions\n")
    ws_rows = [r for r in scored_rows if _family_from_variant_name(str(r.get("variant_name", ""))) == "warm_start_sampling"]
    ws_feasible = [r for r in ws_rows if r.get("is_feasible", False)]
    if ws_feasible:
        best_ws = min(ws_feasible, key=lambda r: float(r.get("score", float("inf"))))
        lines.append(f"Best Warm-start variant: **{best_ws.get('variant_name', '?')}**")
        lines.append(f"- torque_rate_weight controls control smoothness penalty.")
    else:
        lines.append("No feasible Warm-start variant found. Consider relaxing constraints.")
    lines.append("")

    # Recommended defaults
    lines.append("## 8. Recommended Default Parameters\n")
    bv = best_variants.get("best_overall")
    if bv is not None:
        lines.append(f"Overall best: **{bv.get('variant_name', '?')}**")
        lines.append("")
        lines.append("Use this variant's parameters as the default for future B03 benchmark runs.")
    else:
        lines.append("No feasible overall best found. Review constraints and re-run.")
    lines.append("")

    lines.append("## 9. Current Limitations\n")
    lines.append("- Small-scale sweep only (short num_steps, limited variants).")
    lines.append("- Does not replace formal benchmark.")
    lines.append("- Does not include iLQG / SQP / NMPC tuning.")
    lines.append("- Single seed; seed sensitivity not tested.")
    lines.append("")

    lines.append("## 10. Next Steps\n")
    lines.append("- Re-run B03 benchmark with recommended parameters.")
    lines.append("- Or proceed to B03-R4: mini iLQR / iLQG learning skeleton.")
    lines.append("")

    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


# ---------------------------------------------------------------------------
# Recommended solver report
# ---------------------------------------------------------------------------

_SOLVER_ROLES: dict[str, dict[str, str]] = {
    "random_shooting": {
        "label": "Random Shooting",
        "role": "baseline",
        "description": "Level 1 solver. Samples control sequences from scratch each step. Serves as a baseline reference for sample efficiency and tracking quality.",
    },
    "warm_start_fast": {
        "label": "Warm-start Sampling",
        "role": "fast",
        "description": "Level 2 solver. Shifts previous best sequence as warm-start mean. Low runtime, suitable for fast-iteration scenarios.",
    },
    "cem_default": {
        "label": "CEM-MPC",
        "role": "balanced",
        "description": "Level 3 solver. Cross-entropy method with elite-based distribution updates. Best balance of tracking accuracy, runtime, and control quality.",
    },
    "mppi_smooth": {
        "label": "MPPI-lite",
        "role": "smooth",
        "description": "Level 4 solver. Cost-weighted soft averaging with temperature control. Smoothest control and trajectory output.",
    },
}


def write_recommended_solver_report(
    *,
    metrics_rows: list[dict[str, Any]],
    output_path: Path,
    num_steps: int = 100,
    horizon: int = 16,
) -> None:
    """Write a recommended solver benchmark report in markdown."""
    output_path.parent.mkdir(parents=True, exist_ok=True)

    lines: list[str] = []
    lines.append("# B03 Recommended Sampling Solver Benchmark Report\n")

    # 1. Purpose
    lines.append("## 1. Benchmark Purpose\n")
    lines.append("This short benchmark validates the recommended parameter configs for the four B03 sampling-family solvers.")
    lines.append("It is not a final long benchmark. Its purpose is to fix default configs before proceeding to B03-R4 (mini iLQR / iLQG).\n")
    lines.append(f"- Simulation steps: {num_steps}")
    lines.append(f"- Horizon: {horizon}")
    lines.append(f"- Target: circle trajectory (center=[0.45, 0.10], radius=0.08, freq=0.2)")
    lines.append("")

    # 2. Solver roles
    lines.append("## 2. Solver Roles\n")
    lines.append("| Solver Config | Family | Role | Description |")
    lines.append("| --- | --- | --- | --- |")
    for row in metrics_rows:
        name = str(row.get("variant_name", "?"))
        role_info = _SOLVER_ROLES.get(name, {"label": name, "role": "unknown", "description": ""})
        lines.append(f"| {name} | {role_info['label']} | {role_info['role']} | {role_info['description']} |")
    lines.append("")

    # 3. Metrics table
    lines.append("## 3. Metrics Summary\n")
    lines.append("| solver | mean_ee_error | max_ee_error | mean_runtime_ms | ctrl_smooth | traj_smooth | success_rate |")
    lines.append("| --- | --- | --- | --- | --- | --- | --- |")
    for row in metrics_rows:
        name = row.get("variant_name", "?")
        err = row.get("mean_ee_error", float("nan"))
        max_err = row.get("max_ee_error", float("nan"))
        rt = row.get("mean_runtime_ms", float("nan"))
        cs = row.get("control_smoothness", float("nan"))
        ts = row.get("trajectory_smoothness", float("nan"))
        sr = row.get("success_rate", float("nan"))
        err_s = f"{err:.6f}" if math.isfinite(float(err)) else "NaN"
        max_err_s = f"{max_err:.6f}" if math.isfinite(float(max_err)) else "NaN"
        rt_s = f"{rt:.1f}" if math.isfinite(float(rt)) else "NaN"
        cs_s = f"{cs:.6f}" if math.isfinite(float(cs)) else "NaN"
        ts_s = f"{ts:.6f}" if math.isfinite(float(ts)) else "NaN"
        sr_s = f"{sr:.3f}" if math.isfinite(float(sr)) else "NaN"
        lines.append(f"| {name} | {err_s} | {max_err_s} | {rt_s} | {cs_s} | {ts_s} | {sr_s} |")
    lines.append("")

    # 4. Tracking error conclusions
    lines.append("## 4. Tracking Error Conclusions\n")
    valid_errors = [(r.get("variant_name", "?"), float(r.get("mean_ee_error", float("nan")))) for r in metrics_rows]
    valid_errors = [(n, e) for n, e in valid_errors if math.isfinite(e)]
    if valid_errors:
        best_error = min(valid_errors, key=lambda x: x[1])
        worst_error = max(valid_errors, key=lambda x: x[1])
        lines.append(f"- Best tracking error: **{best_error[0]}** (mean_ee_error={best_error[1]:.6f})")
        lines.append(f"- Worst tracking error: **{worst_error[0]}** (mean_ee_error={worst_error[1]:.6f})")
        if best_error[0] == "cem_default":
            lines.append("- CEM achieves the best tracking accuracy due to iterative elite-based distribution refinement.")
        elif best_error[0] == "mppi_smooth":
            lines.append("- MPPI-lite achieves competitive tracking with soft cost-weighted averaging.")
    else:
        lines.append("- No valid tracking error data available.")
    lines.append("")

    # 5. Runtime conclusions
    lines.append("## 5. Runtime Conclusions\n")
    valid_runtimes = [(r.get("variant_name", "?"), float(r.get("mean_runtime_ms", float("nan")))) for r in metrics_rows]
    valid_runtimes = [(n, rt) for n, rt in valid_runtimes if math.isfinite(rt)]
    if valid_runtimes:
        fastest = min(valid_runtimes, key=lambda x: x[1])
        slowest = max(valid_runtimes, key=lambda x: x[1])
        lines.append(f"- Fastest solver: **{fastest[0]}** (mean_runtime_ms={fastest[1]:.1f})")
        lines.append(f"- Slowest solver: **{slowest[0]}** (mean_runtime_ms={slowest[1]:.1f})")
        if fastest[0] == "warm_start_fast":
            lines.append("- Warm-start is fastest because it reuses the previous solution as search center, requiring fewer samples.")
    else:
        lines.append("- No valid runtime data available.")
    lines.append("")

    # 6. Control smoothness conclusions
    lines.append("## 6. Control Smoothness Conclusions\n")
    valid_ctrl = [(r.get("variant_name", "?"), float(r.get("control_smoothness", float("nan")))) for r in metrics_rows]
    valid_ctrl = [(n, c) for n, c in valid_ctrl if math.isfinite(c)]
    if valid_ctrl:
        smoothest_ctrl = min(valid_ctrl, key=lambda x: x[1])
        roughest_ctrl = max(valid_ctrl, key=lambda x: x[1])
        lines.append(f"- Smoothest control: **{smoothest_ctrl[0]}** (control_smoothness={smoothest_ctrl[1]:.6f})")
        lines.append(f"- Roughest control: **{roughest_ctrl[0]}** (control_smoothness={roughest_ctrl[1]:.6f})")
        if smoothest_ctrl[0] == "mppi_smooth":
            lines.append("- MPPI-lite produces the smoothest control because soft cost-weighted averaging naturally penalizes abrupt changes.")
    else:
        lines.append("- No valid control smoothness data available.")
    lines.append("")

    # 7. Trajectory smoothness conclusions
    lines.append("## 7. Trajectory Smoothness Conclusions\n")
    valid_traj = [(r.get("variant_name", "?"), float(r.get("trajectory_smoothness", float("nan")))) for r in metrics_rows]
    valid_traj = [(n, t) for n, t in valid_traj if math.isfinite(t)]
    if valid_traj:
        smoothest_traj = min(valid_traj, key=lambda x: x[1])
        roughest_traj = max(valid_traj, key=lambda x: x[1])
        lines.append(f"- Smoothest trajectory: **{smoothest_traj[0]}** (trajectory_smoothness={smoothest_traj[1]:.6f})")
        lines.append(f"- Roughest trajectory: **{roughest_traj[0]}** (trajectory_smoothness={roughest_traj[1]:.6f})")
    else:
        lines.append("- No valid trajectory smoothness data available.")
    lines.append("")

    # 8. Recommended defaults
    lines.append("## 8. Recommended Default Configs\n")
    lines.append("| Use Case | Recommended Solver | Rationale |")
    lines.append("| --- | --- | --- |")
    lines.append("| General default | cem_default | Best balance of accuracy, runtime, and control quality |")
    lines.append("| Smoothness priority | mppi_smooth | Soft averaging produces smoothest control and trajectory |")
    lines.append("| Fast iteration | warm_start_fast | Lowest runtime, reuses previous solution |")
    lines.append("| Baseline reference | random_shooting | Simplest solver, no warm-start or distribution updates |")
    lines.append("")

    # 9. Current limitations
    lines.append("## 9. Current Limitations\n")
    lines.append("- Short benchmark only (100 steps, not the final long 1000-step benchmark).")
    lines.append("- Single seed; seed sensitivity not tested.")
    lines.append("- Does not include iLQG / SQP / NMPC solvers.")
    lines.append("- Circle target only; no lissajous or figure-8 targets tested.")
    lines.append("")

    # 10. Next steps
    lines.append("## 10. Next Steps\n")
    lines.append("- B03-R4: mini iLQR / iLQG learning skeleton.")
    lines.append("- Long benchmark with lissajous target after iLQG is implemented.")
    lines.append("- Seed sensitivity analysis for recommended configs.")
    lines.append("")

    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


# ---------------------------------------------------------------------------
# Sweep metrics CSV
# ---------------------------------------------------------------------------

SWEEP_METRICS_COLUMNS = [
    "variant_name",
    "solver_family",
    "mean_ee_error",
    "max_ee_error",
    "mean_runtime_ms",
    "max_runtime_ms",
    "control_smoothness",
    "trajectory_smoothness",
    "success_rate",
    "weight_entropy",
    "max_weight",
    "score",
    "is_feasible",
    "feasibility_reason",
]


def save_sweep_metrics_csv(
    scored_rows: list[dict[str, Any]],
    output_path: Path,
) -> None:
    """Save parameter sweep metrics CSV."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=SWEEP_METRICS_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        for row in scored_rows:
            writer.writerow(row)
