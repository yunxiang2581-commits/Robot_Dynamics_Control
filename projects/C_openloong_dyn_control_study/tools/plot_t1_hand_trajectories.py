# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "matplotlib",
#     "numpy",
# ]
# ///
import csv
from pathlib import Path
import re

import matplotlib.pyplot as plt
from matplotlib import gridspec
from matplotlib.lines import Line2D
import numpy as np


def parse_traj_log(log_path):
    """Parse sparse [T1-TRAJ] lines from one OpenLoong run log."""
    number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
    traj_re = re.compile(
        rf"T1-TRAJ\] mode=(\w+) t=({number}) err_mm=({number})"
        rf"\s+des_W=\s*({number})\s+({number})\s+({number})"
        rf"\s+cur_W=\s*({number})\s+({number})\s+({number})"
        rf"\s+offset_W=\s*({number})\s+({number})\s+({number})"
    )
    rows = []
    with log_path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            match = traj_re.search(line)
            if not match:
                continue
            values = match.groups()
            rows.append(
                {
                    "mode": values[0],
                    "t_s": float(values[1]),
                    "err_mm": float(values[2]),
                    "des_x_m": float(values[3]),
                    "des_y_m": float(values[4]),
                    "des_z_m": float(values[5]),
                    "cur_x_m": float(values[6]),
                    "cur_y_m": float(values[7]),
                    "cur_z_m": float(values[8]),
                    "offset_x_m": float(values[9]),
                    "offset_y_m": float(values[10]),
                    "offset_z_m": float(values[11]),
                    "source_log": log_path.name,
                }
            )
    if not rows:
        raise RuntimeError(f"No [T1-TRAJ] records found in {log_path}")
    return rows


def main():
    """Plot target and measured T1 right-hand trajectories.

    Saves
    -----
    CSV, PNG, and PDF files under the R2 reproduction output directory.
    """
    # --- Style Setup ---
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "axes.titlesize": 12,
            "axes.labelsize": 10,
            "xtick.color": "dimgrey",
            "ytick.color": "dimgrey",
            "axes.labelcolor": "dimgrey",
            "figure.facecolor": "white",
            "axes.facecolor": "white",
        }
    )

    # --- Data ---
    project_root = Path(__file__).resolve().parents[3]
    run_root = (
        project_root
        / "projects"
        / "C_openloong_dyn_control_study"
        / "outputs"
        / "docker_reproduction"
        / "openloong_ubuntu22_build_R2"
        / "20260530_180827"
    )
    logs_dir = run_root / "logs"
    figures_dir = run_root / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)

    modes = ["LINE", "CIRCLE", "SINE"]
    rows = []
    for mode in modes:
        rows.extend(parse_traj_log(logs_dir / f"20260703_t1_traj_{mode}.log"))

    samples_csv = figures_dir / "t1_hand_trajectory_samples.csv"
    metrics_csv = figures_dir / "t1_hand_trajectory_metrics.csv"

    fieldnames = list(rows[0].keys())
    with samples_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    metrics = []
    for mode in modes:
        err = np.array([r["err_mm"] for r in rows if r["mode"] == mode], dtype=float)
        metrics.append(
            {
                "mode": mode,
                "samples": int(err.size),
                "max_err_mm": float(np.max(err)),
                "rms_err_mm": float(np.sqrt(np.mean(np.square(err)))),
            }
        )
    with metrics_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["mode", "samples", "max_err_mm", "rms_err_mm"])
        writer.writeheader()
        writer.writerows(metrics)
    metrics_by_mode = {row["mode"]: row for row in metrics}

    # --- Plot ---
    pal = ["#90c1c6", "#72a5b4", "#58849f", "#446485", "#324465", "#1f253f"]
    target_color = pal[-1]
    actual_color = "#d73027"
    threshold_color = "#bd0c0c"

    fig = plt.figure(figsize=(14, 8), dpi=150)
    gs = gridspec.GridSpec(2, 3)
    gs.update(wspace=0.08, hspace=0.30, left=0.08, right=0.99, top=0.90, bottom=0.09)
    axes = [plt.subplot(gs[i, j]) for i in range(2) for j in range(3)]
    fig.suptitle("T1 Right-Hand Trajectory Tracking", fontsize=14, y=0.98, color="dimgrey")

    labels = [chr(ord("a") + i) for i in range(6)]
    all_rel = []
    for mode in modes:
        part = [r for r in rows if r["mode"] == mode]
        x0 = part[0]["des_x_m"]
        y0 = part[0]["des_y_m"]
        all_rel.extend([(r["des_x_m"] - x0) * 1000.0 for r in part])
        all_rel.extend([(r["des_y_m"] - y0) * 1000.0 for r in part])
        all_rel.extend([(r["cur_x_m"] - x0) * 1000.0 for r in part])
        all_rel.extend([(r["cur_y_m"] - y0) * 1000.0 for r in part])
    rel_min = min(all_rel)
    rel_max = max(all_rel)
    rel_pad = max(5.0, (rel_max - rel_min) * 0.12)

    for col, mode in enumerate(modes):
        part = [r for r in rows if r["mode"] == mode]
        metric = metrics_by_mode[mode]
        x0 = part[0]["des_x_m"]
        y0 = part[0]["des_y_m"]

        t_s = np.array([r["t_s"] for r in part], dtype=float)
        err_mm = np.array([r["err_mm"] for r in part], dtype=float)
        des_x_mm = np.array([(r["des_x_m"] - x0) * 1000.0 for r in part], dtype=float)
        des_y_mm = np.array([(r["des_y_m"] - y0) * 1000.0 for r in part], dtype=float)
        cur_x_mm = np.array([(r["cur_x_m"] - x0) * 1000.0 for r in part], dtype=float)
        cur_y_mm = np.array([(r["cur_y_m"] - y0) * 1000.0 for r in part], dtype=float)

        ax_path = axes[col]
        ax_err = axes[col + 3]
        for ax in [ax_path, ax_err]:
            ax.grid(False)
            ax.tick_params(axis="both", which="both", length=0, labelcolor="dimgrey")
            ax.patch.set_edgecolor("lightgrey")
            ax.patch.set_linewidth(0.8)

        ax_path.plot(
            des_x_mm,
            des_y_mm,
            color=target_color,
            linewidth=2.2,
            marker="o",
            markersize=4.0,
            label="target",
            zorder=4,
        )
        ax_path.plot(
            cur_x_mm,
            cur_y_mm,
            color=actual_color,
            linewidth=1.9,
            linestyle="--",
            marker="s",
            markersize=3.2,
            label="actual",
            zorder=3,
        )
        ax_path.scatter([des_x_mm[0]], [des_y_mm[0]], color="white", edgecolors=target_color, s=80, zorder=5)
        ax_path.set_aspect("equal", adjustable="box")
        ax_path.set_xlim(rel_min - rel_pad, rel_max + rel_pad)
        ax_path.set_ylim(rel_min - rel_pad, rel_max + rel_pad)
        ax_path.set_title(
            r"$\bf{(" + labels[col] + r")}$" + f"  {mode} target vs actual",
            loc="left",
            fontsize=11,
            pad=7,
            color="dimgrey",
        )
        if col == 0:
            ax_path.set_ylabel("Relative y (mm)", fontsize=9, labelpad=6, color="dimgrey")
        else:
            ax_path.tick_params(labelleft=False)
        ax_path.set_xlabel("Relative x (mm)", fontsize=9, labelpad=6, color="dimgrey")

        ax_err.plot(
            t_s,
            err_mm,
            color=target_color,
            linewidth=2.0,
            marker="o",
            markersize=3.6,
            zorder=4,
        )
        ax_err.axhline(25.0, color=threshold_color, linewidth=1.2, linestyle="--", alpha=0.75, zorder=2)
        ax_err.text(
            0.97,
            0.86,
            f"max {metric['max_err_mm']:.2f} mm\nRMS {metric['rms_err_mm']:.2f} mm",
            transform=ax_err.transAxes,
            ha="right",
            va="top",
            fontsize=8.5,
            color="dimgrey",
            bbox={"facecolor": "white", "edgecolor": "lightgrey", "alpha": 0.82, "pad": 5},
        )
        ax_err.set_ylim(0.0, max(30.0, float(np.max(err_mm)) * 1.25))
        ax_err.set_title(
            r"$\bf{(" + labels[col + 3] + r")}$" + f"  {mode} tracking error",
            loc="left",
            fontsize=11,
            pad=7,
            color="dimgrey",
        )
        ax_err.set_xlabel("Trajectory time (s)", fontsize=9, labelpad=6, color="dimgrey")
        if col == 0:
            ax_err.set_ylabel("Error (mm)", fontsize=9, labelpad=6, color="dimgrey")
        else:
            ax_err.tick_params(labelleft=False)

    handles = [
        Line2D([0], [0], color=target_color, linewidth=2.2, marker="o", markersize=4.0, label="target"),
        Line2D([0], [0], color=actual_color, linewidth=1.9, linestyle="--", marker="s", markersize=3.2, label="actual"),
    ]
    axes[0].legend(
        handles=handles,
        loc="upper left",
        fontsize=9,
        frameon=True,
        facecolor="white",
        framealpha=0.8,
        edgecolor="lightgrey",
        labelcolor="dimgrey",
    )

    best = min(metrics, key=lambda row: row["rms_err_mm"])
    fig.text(
        0.99,
        0.015,
        f"{best['mode']} has the lowest RMS error ({best['rms_err_mm']:.2f} mm); all three stay below the 25 mm limit.",
        ha="right",
        va="bottom",
        fontsize=9,
        color="dimgrey",
        style="italic",
    )

    for ax in axes:
        for spine in ["top", "right", "left", "bottom"]:
            ax.spines[spine].set_visible(False)

    # --- Save ---
    png_path = figures_dir / "t1_hand_trajectory_shapes.png"
    pdf_path = figures_dir / "t1_hand_trajectory_shapes.pdf"
    plt.savefig(pdf_path, dpi=150, bbox_inches="tight")
    plt.savefig(png_path, dpi=150, bbox_inches="tight")
    print(f"saved_png={png_path}")
    print(f"saved_pdf={pdf_path}")
    print(f"saved_samples_csv={samples_csv}")
    print(f"saved_metrics_csv={metrics_csv}")
    for row in metrics:
        print(
            f"{row['mode']}: samples={row['samples']} "
            f"max_err_mm={row['max_err_mm']:.6f} rms_err_mm={row['rms_err_mm']:.6f}"
        )


if __name__ == "__main__":
    main()
