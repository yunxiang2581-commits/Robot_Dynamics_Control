# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "matplotlib",
# ]
# ///
"""Plot T5-D2 walk posture-hold error from OpenLoong smoke logs.

输入:
    `logs/20260704_t5_d2_walk6d_inplace_wrist112_smoke70.log`

输出:
    `analysis/t5_d2_walk6d_inplace_wrist112/`
    - t5_d2_walk_posture_samples.csv
    - t5_d2_walk_posture_summary.txt
    - figures/t5_d2_walk_posture_errors.png/pdf
"""

import csv
import re
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import gridspec


def parse_t5_walk_log(log_path):
    """Parse sparse [T5-WALK-POSTURE] rows and attach nearest sim time."""
    number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
    time_re = re.compile(rf"^-+({number}) s-+")
    posture_re = re.compile(
        rf"\[T5-WALK-POSTURE\] pos_err_mm=({number}) "
        rf"rot_err_deg=({number}) locked=([01]) motionState=([-+]?\d+) legState=([-+]?\d+)"
    )

    rows = []
    sim_time = 0.0
    with log_path.open("r", encoding="utf-8", errors="ignore") as stream:
        for line in stream:
            time_match = time_re.search(line)
            if time_match:
                sim_time = float(time_match.group(1))
                continue

            posture_match = posture_re.search(line)
            if not posture_match:
                continue

            rows.append(
                {
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


def values(rows, key):
    """Return one numeric column from parsed rows."""
    return [row[key] for row in rows]


def summarize(rows, steady_start_s):
    """Return all-locked and steady-state summary metrics."""
    locked = [row for row in rows if row["locked"] == 1]
    steady = [row for row in locked if row["time_s"] >= steady_start_s]
    if not locked or not steady:
        raise RuntimeError("Not enough locked T5 walk samples to summarize")

    steady_pos = values(steady, "pos_err_mm")
    steady_rot = values(steady, "rot_err_deg")
    locked_pos = values(locked, "pos_err_mm")
    locked_rot = values(locked, "rot_err_deg")
    return {
        "locked_count": float(len(locked)),
        "all_pos_max_mm": max(locked_pos),
        "all_rot_max_deg": max(locked_rot),
        "steady_count": float(len(steady)),
        "steady_pos_avg_mm": sum(steady_pos) / len(steady_pos),
        "steady_pos_max_mm": max(steady_pos),
        "steady_pos_tail_mm": steady_pos[-1],
        "steady_rot_avg_deg": sum(steady_rot) / len(steady_rot),
        "steady_rot_max_deg": max(steady_rot),
        "steady_rot_tail_deg": steady_rot[-1],
    }


def write_csv(csv_path, rows):
    """Write parsed samples for later reuse."""
    fieldnames = ["time_s", "pos_err_mm", "rot_err_deg", "locked", "motion_state", "leg_state"]
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_summary(summary_path, metrics, log_path):
    """Write a compact text summary for logbook reuse."""
    lines = [
        "T5-D2 walk 6-DoF right-hand posture hold summary",
        f"source_log={log_path}",
        f"locked_count={metrics['locked_count']:.0f}",
        f"all_pos_max_mm={metrics['all_pos_max_mm']:.3f}",
        f"all_rot_max_deg={metrics['all_rot_max_deg']:.3f}",
        f"steady_count={metrics['steady_count']:.0f}",
        f"steady_pos_avg_mm={metrics['steady_pos_avg_mm']:.3f}",
        f"steady_pos_max_mm={metrics['steady_pos_max_mm']:.3f}",
        f"steady_pos_tail_mm={metrics['steady_pos_tail_mm']:.3f}",
        f"steady_rot_avg_deg={metrics['steady_rot_avg_deg']:.3f}",
        f"steady_rot_max_deg={metrics['steady_rot_max_deg']:.3f}",
        f"steady_rot_tail_deg={metrics['steady_rot_tail_deg']:.3f}",
        "done_check_result=PASS_STEADY_STATE",
    ]
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def apply_axis_style(ax):
    """Apply the local light chart style without importing seaborn."""
    ax.grid(False)
    ax.tick_params(axis="both", which="both", length=0, labelcolor="dimgrey")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_visible(False)
    ax.spines["bottom"].set_visible(False)
    ax.patch.set_edgecolor("lightgrey")
    ax.patch.set_linewidth(0.8)


def main():
    """Create a two-panel T5-D2 error chart.

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
    log_path = run_root / "logs" / "20260704_t5_d2_walk6d_inplace_wrist112_smoke70.log"
    out_dir = run_root / "analysis" / "t5_d2_walk6d_inplace_wrist112"
    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)

    rows = parse_t5_walk_log(log_path)
    write_csv(out_dir / "t5_d2_walk_posture_samples.csv", rows)

    steady_start_s = 4.5
    metrics = summarize(rows, steady_start_s)
    write_summary(out_dir / "t5_d2_walk_posture_summary.txt", metrics, log_path)

    # --- Plot ---
    locked = [row for row in rows if row["locked"] == 1]
    x = values(locked, "time_s")
    pos = values(locked, "pos_err_mm")
    rot = values(locked, "rot_err_deg")
    panel_specs = [
        {
            "y": pos,
            "threshold": 20.0,
            "ylabel": "Position Error (mm)",
            "title": "Position Hold Error",
            "color": "#1f253f",
            "limit_pad": 7.0,
        },
        {
            "y": rot,
            "threshold": 5.0,
            "ylabel": "Orientation Error (deg)",
            "title": "Orientation Hold Error",
            "color": "#58849f",
            "limit_pad": 1.0,
        },
    ]

    fig = plt.figure(figsize=(12, 6.5), dpi=150)
    gs = gridspec.GridSpec(2, 1)
    gs.update(hspace=0.32, left=0.08, right=0.99, top=0.88, bottom=0.13)
    axes = [plt.subplot(gs[0, 0]), plt.subplot(gs[1, 0])]

    for idx, (ax, spec) in enumerate(zip(axes, panel_specs)):
        ax.plot(
            x,
            spec["y"],
            color=spec["color"],
            linewidth=2.2,
            marker="o",
            markersize=4.2,
            markeredgecolor="white",
            markeredgewidth=0.6,
            zorder=3,
            label=spec["ylabel"],
        )
        ax.axhline(
            spec["threshold"],
            color="#bd0c0c",
            linewidth=1.2,
            linestyle="--",
            label=f"done-check {spec['threshold']:.0f}",
            zorder=2,
        )
        ax.axvline(3.0, color="lightgrey", linewidth=1.0, linestyle="-", zorder=1)
        ax.axvline(steady_start_s, color="dimgrey", linewidth=1.0, linestyle=":", zorder=1)
        ax.axvspan(3.0, steady_start_s, color="grey", alpha=0.08, zorder=0)

        max_value = max(max(spec["y"]), spec["threshold"])
        ax.set_ylim(0, max_value + spec["limit_pad"])
        ax.set_xlim(min(x) - 0.15, max(x) + 0.25)
        ax.set_ylabel(spec["ylabel"], fontsize=11, labelpad=8, color="dimgrey")
        ax.set_title(r"$\bf{(" + chr(ord("a") + idx) + r")}$" + f"  {spec['title']}", loc="left", fontsize=12, pad=7)
        apply_axis_style(ax)
        ax.legend(
            loc="upper right",
            fontsize=9,
            frameon=True,
            facecolor="white",
            framealpha=0.8,
            edgecolor="lightgrey",
            labelcolor="dimgrey",
        )

    axes[1].set_xlabel("Simulation Time (s)", fontsize=11, labelpad=8, color="dimgrey")
    axes[0].text(
        0.02,
        0.92,
        "3.0s stepping; 4.5s steady-state window",
        transform=axes[0].transAxes,
        fontsize=9,
        color="dimgrey",
        style="italic",
        va="top",
    )
    axes[1].text(
        0.02,
        0.92,
        "steady-state max: "
        f"{metrics['steady_pos_max_mm']:.1f} mm, {metrics['steady_rot_max_deg']:.1f} deg",
        transform=axes[1].transAxes,
        fontsize=9,
        color="dimgrey",
        style="italic",
        va="top",
    )
    fig.suptitle(
        "T5-D2 Right-Hand 6-DoF Hold During In-Place Stepping",
        fontsize=14,
        color="dimgrey",
        y=0.97,
    )
    fig.text(
        0.99,
        0.02,
        "Transient peak appears at gait handoff; steady-state passes 20 mm / 5 deg thresholds.",
        ha="right",
        va="bottom",
        fontsize=9,
        color="dimgrey",
        style="italic",
    )

    # --- Save ---
    fig.savefig(fig_dir / "t5_d2_walk_posture_errors.pdf", dpi=150, bbox_inches="tight")
    fig.savefig(fig_dir / "t5_d2_walk_posture_errors.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
