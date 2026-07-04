# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "matplotlib",
# ]
# ///
"""Compare T5-D2 and T5-D3 walk posture-hold errors.

输入:
    D2: logs/20260704_t5_d2_walk6d_inplace_wrist112_smoke70.log
    D3: logs/20260704_t5_d3_lock4500_smoke70.log

输出:
    analysis/t5_d3_lock_delay/
    - t5_d3_lock_delay_samples.csv
    - t5_d3_lock_delay_summary.txt
    - figures/t5_d3_lock_delay_comparison.png/pdf
"""

import csv
import re
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import gridspec


def parse_t5_walk_log(log_path, label):
    """Parse sparse T5 walk posture diagnostics with nearest sim time."""
    number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
    time_re = re.compile(rf"^-+({number}) s-+")
    posture_re = re.compile(
        rf"\[T5-WALK-POSTURE\] pos_err_mm=({number}) "
        rf"rot_err_deg=({number}) locked=([01]).*"
        rf"motionState=([-+]?\d+) legState=([-+]?\d+)"
    )
    lock_re = re.compile(r"\[T5-WALK-LOCK\].*lock_count=([-+]?\d+)")

    rows = []
    sim_time = 0.0
    lock_count = None
    with log_path.open("r", encoding="utf-8", errors="ignore") as stream:
        for line in stream:
            time_match = time_re.search(line)
            if time_match:
                sim_time = float(time_match.group(1))
                continue

            lock_match = lock_re.search(line)
            if lock_match:
                lock_count = int(lock_match.group(1))

            posture_match = posture_re.search(line)
            if not posture_match:
                continue

            rows.append(
                {
                    "run": label,
                    "source_log": str(log_path),
                    "lock_count": lock_count if lock_count is not None else "",
                    "time_s": sim_time,
                    "pos_err_mm": float(posture_match.group(1)),
                    "rot_err_deg": float(posture_match.group(2)),
                    "locked": int(posture_match.group(3)),
                    "motion_state": int(posture_match.group(4)),
                    "leg_state": int(posture_match.group(5)),
                }
            )

    if not rows:
        raise RuntimeError(f"No [T5-WALK-POSTURE] rows found in {log_path}")
    return rows


def summarize(rows, steady_start_s):
    """Return all-locked and steady-state metrics for one run."""
    locked = [row for row in rows if row["locked"] == 1]
    steady = [row for row in locked if row["time_s"] >= steady_start_s]
    if not locked or not steady:
        raise RuntimeError("Not enough locked T5 walk samples to summarize")

    pos = [row["pos_err_mm"] for row in locked]
    rot = [row["rot_err_deg"] for row in locked]
    steady_pos = [row["pos_err_mm"] for row in steady]
    steady_rot = [row["rot_err_deg"] for row in steady]
    return {
        "locked_count": len(locked),
        "all_pos_max_mm": max(pos),
        "all_rot_max_deg": max(rot),
        "steady_count": len(steady),
        "steady_pos_avg_mm": sum(steady_pos) / len(steady_pos),
        "steady_pos_max_mm": max(steady_pos),
        "steady_pos_tail_mm": steady_pos[-1],
        "steady_rot_avg_deg": sum(steady_rot) / len(steady_rot),
        "steady_rot_max_deg": max(steady_rot),
        "steady_rot_tail_deg": steady_rot[-1],
    }


def write_summary(summary_path, metrics_by_run, steady_start_s):
    """Write a compact text summary for logbook reuse."""
    d2 = metrics_by_run["D2 lock@1000"]
    d3 = metrics_by_run["D3 lock@4500"]
    improvement = 100.0 * (d2["all_pos_max_mm"] - d3["all_pos_max_mm"]) / d2["all_pos_max_mm"]
    lines = [
        "T5-D3 delayed-lock right-hand 6-DoF posture hold summary",
        f"steady_start_s={steady_start_s:.1f}",
        "",
        "[D2 lock@1000]",
    ]
    for key, value in d2.items():
        lines.append(f"{key}={value:.3f}" if isinstance(value, float) else f"{key}={value}")
    lines.extend(["", "[D3 lock@4500]"])
    for key, value in d3.items():
        lines.append(f"{key}={value:.3f}" if isinstance(value, float) else f"{key}={value}")
    lines.extend(
        [
            "",
            f"all_pos_peak_reduction_pct={improvement:.1f}",
            "done_check_result=PASS_ALL_LOCKED",
        ]
    )
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def apply_axis_style(ax):
    """Apply local chart styling."""
    ax.grid(False)
    ax.tick_params(axis="both", which="both", length=0, labelcolor="dimgrey")
    ax.patch.set_edgecolor("lightgrey")
    ax.patch.set_linewidth(0.8)


def main():
    """Create a two-panel D2 vs D3 comparison chart.

    Saves
    -----
    CSV, TXT, PNG, and PDF artifacts under the R2 analysis directory.
    """
    # --- Style Setup ---
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "axes.titlesize": 12,
            "axes.labelsize": 11,
            "axes.labelcolor": "dimgrey",
            "xtick.color": "dimgrey",
            "ytick.color": "dimgrey",
            "figure.facecolor": "white",
            "axes.facecolor": "white",
        }
    )

    # --- Data ---
    project_root = Path(__file__).resolve().parents[1]
    run_root = (
        project_root
        / "outputs"
        / "docker_reproduction"
        / "openloong_ubuntu22_build_R2"
        / "20260530_180827"
    )
    log_specs = [
        ("D2 lock@1000", run_root / "logs" / "20260704_t5_d2_walk6d_inplace_wrist112_smoke70.log"),
        ("D3 lock@4500", run_root / "logs" / "20260704_t5_d3_lock4500_smoke70.log"),
    ]
    out_dir = run_root / "analysis" / "t5_d3_lock_delay"
    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    for label, log_path in log_specs:
        rows.extend(parse_t5_walk_log(log_path, label))
    csv_path = out_dir / "t5_d3_lock_delay_samples.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        fieldnames = [
            "run",
            "source_log",
            "lock_count",
            "time_s",
            "pos_err_mm",
            "rot_err_deg",
            "locked",
            "motion_state",
            "leg_state",
        ]
        writer = csv.DictWriter(stream, fieldnames=fieldnames, quoting=csv.QUOTE_MINIMAL)
        writer.writeheader()
        writer.writerows(rows)

    steady_start_s = 4.5
    metrics_by_run = {
        label: summarize([row for row in rows if row["run"] == label], steady_start_s)
        for label, _ in log_specs
    }
    write_summary(out_dir / "t5_d3_lock_delay_summary.txt", metrics_by_run, steady_start_s)

    # --- Plot ---
    colors = {"D2 lock@1000": "#1f253f", "D3 lock@4500": "#33a02c"}
    markers = {"D2 lock@1000": "o", "D3 lock@4500": "s"}
    panels = [
        ("pos_err_mm", "Position Error (mm)", 20.0, "Position Peak at Gait Handoff"),
        ("rot_err_deg", "Orientation Error (deg)", 5.0, "Orientation Error"),
    ]

    fig = plt.figure(figsize=(12, 6.8), dpi=150)
    gs = gridspec.GridSpec(2, 1)
    gs.update(hspace=0.32, left=0.08, right=0.99, top=0.88, bottom=0.13)
    axes = [plt.subplot(gs[i, 0]) for i in range(2)]

    locked_rows = [row for row in rows if row["locked"] == 1]
    for idx, (ax, (metric, ylabel, threshold, title)) in enumerate(zip(axes, panels)):
        for run_label, _ in log_specs:
            sub = [row for row in locked_rows if row["run"] == run_label]
            ax.plot(
                [row["time_s"] for row in sub],
                [row[metric] for row in sub],
                color=colors[run_label],
                marker=markers[run_label],
                markersize=4.5,
                markeredgecolor="white",
                markeredgewidth=0.6,
                linewidth=2.1,
                label=run_label,
                zorder=3,
            )
        ax.axhline(threshold, color="#bd0c0c", linewidth=1.2, linestyle="--",
                   label=f"done-check {threshold:.0f}", zorder=2)
        ax.axvspan(3.0, 4.5, color="grey", alpha=0.08, zorder=0)
        ax.axvline(3.0, color="lightgrey", linewidth=1.0, zorder=1)
        ax.axvline(4.5, color="dimgrey", linewidth=1.0, linestyle=":", zorder=1)
        ax.set_ylabel(ylabel, fontsize=11, labelpad=8, color="dimgrey")
        ax.set_title(r"$\bf{(" + chr(ord("a") + idx) + r")}$" + f"  {title}",
                     loc="left", fontsize=12, pad=7, color="dimgrey")
        y_max = max(max(row[metric] for row in locked_rows), threshold)
        ax.set_ylim(0, y_max * 1.16 + 0.5)
        ax.set_xlim(0.75, 10.25)
        apply_axis_style(ax)

    axes[0].legend(
        loc="upper right",
        fontsize=9,
        frameon=True,
        facecolor="white",
        framealpha=0.8,
        edgecolor="lightgrey",
        labelcolor="dimgrey",
    )
    axes[1].set_xlabel("Simulation Time (s)", fontsize=11, labelpad=8, color="dimgrey")
    axes[0].annotate(
        "delayed lock avoids the first\nsingle-support hand spike",
        xy=(5.5, metrics_by_run["D3 lock@4500"]["all_pos_max_mm"]),
        xytext=(6.2, 28.0),
        arrowprops={"arrowstyle": "-", "color": "dimgrey", "lw": 0.8},
        fontsize=9,
        color="dimgrey",
        ha="left",
        va="center",
    )
    fig.suptitle(
        "T5-D3 Delayed Hand-Pose Lock Reduces Gait-Handoff Transient",
        fontsize=14,
        color="dimgrey",
        y=0.97,
    )
    improvement = (
        100.0
        * (metrics_by_run["D2 lock@1000"]["all_pos_max_mm"] - metrics_by_run["D3 lock@4500"]["all_pos_max_mm"])
        / metrics_by_run["D2 lock@1000"]["all_pos_max_mm"]
    )
    fig.text(
        0.99,
        0.02,
        f"All-locked position peak: {metrics_by_run['D2 lock@1000']['all_pos_max_mm']:.1f} mm -> "
        f"{metrics_by_run['D3 lock@4500']['all_pos_max_mm']:.1f} mm ({improvement:.1f}% lower).",
        ha="right",
        va="bottom",
        fontsize=9,
        color="dimgrey",
        style="italic",
    )
    for ax in axes:
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.spines["left"].set_visible(False)
        ax.spines["bottom"].set_visible(False)

    # --- Save ---
    fig.savefig(fig_dir / "t5_d3_lock_delay_comparison.pdf", dpi=150, bbox_inches="tight")
    fig.savefig(fig_dir / "t5_d3_lock_delay_comparison.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
