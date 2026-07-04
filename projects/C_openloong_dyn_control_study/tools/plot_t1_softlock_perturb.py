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
from matplotlib.lines import Line2D
import numpy as np


def main():
    """Plot T1 right-hand drift for the soft-lock perturbation run.

    Saves
    -----
    CSV, PNG, and PDF files under the R2 reproduction output directory.
    """
    # --- Style Setup ---
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "axes.titlesize": 14,
            "axes.labelsize": 12,
            "xtick.color": "dimgrey",
            "ytick.color": "dimgrey",
            "axes.labelcolor": "dimgrey",
            "axes.edgecolor": "lightgrey",
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
    log_path = run_root / "logs" / "20260703_t1_perturb_after_softlock.log"
    figures_dir = run_root / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)
    csv_path = figures_dir / "t1_softlock_perturb_drift.csv"

    time_re = re.compile(r"^-+([0-9.]+) s-+")
    drift_re = re.compile(
        r"T1-HOLD.*?drift_mm=([0-9.eE+-]+)"
        r"(?: alpha=([0-9.eE+-]+) hand_w=([0-9.eE+-]+) right_arm_w=([0-9.eE+-]+))?"
    )
    lock_re = re.compile(r"T1-LOCK")
    pert_re = re.compile(r"T1-PERT.*?trigger_time_s=([0-9.eE+-]+)")

    records = []
    current_time = np.nan
    lock_time = np.nan
    perturb_time = np.nan

    with log_path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            time_match = time_re.search(line)
            if time_match:
                current_time = float(time_match.group(1))

            if lock_re.search(line) and np.isnan(lock_time):
                lock_time = current_time

            pert_match = pert_re.search(line)
            if pert_match:
                perturb_time = float(pert_match.group(1))

            drift_match = drift_re.search(line)
            if drift_match:
                records.append(
                    {
                        "time_s": current_time,
                        "drift_mm": float(drift_match.group(1)),
                        "alpha": float(drift_match.group(2) or 0.0),
                        "hand_weight": float(drift_match.group(3) or 1.0),
                        "right_arm_weight": float(drift_match.group(4) or 1.0),
                    }
                )

    if not records:
        raise RuntimeError(f"No T1-HOLD records found in {log_path}")

    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["time_s", "drift_mm", "alpha", "hand_weight", "right_arm_weight"],
        )
        writer.writeheader()
        writer.writerows(records)

    data = np.genfromtxt(csv_path, delimiter=",", names=True)
    if data.ndim == 0:
        data = np.array([data], dtype=data.dtype)
    finite_mask = np.isfinite(data["time_s"]) & np.isfinite(data["drift_mm"])
    data = data[finite_mask]

    time_s = data["time_s"]
    drift_mm = data["drift_mm"]
    alpha = data["alpha"]

    post_mask = time_s >= perturb_time if not np.isnan(perturb_time) else np.zeros_like(time_s, dtype=bool)
    max_post = float(np.max(drift_mm[post_mask])) if np.any(post_mask) else np.nan
    final_drift = float(drift_mm[-1])

    # --- Plot ---
    pal = ["#90c1c6", "#72a5b4", "#58849f", "#446485", "#324465", "#1f253f"]
    fig, (ax, ax_alpha) = plt.subplots(
        2,
        1,
        figsize=(10, 7),
        dpi=150,
        sharex=True,
        gridspec_kw={"height_ratios": [3.2, 1.0], "hspace": 0.10},
    )

    ax.plot(
        time_s,
        drift_mm,
        color=pal[5],
        linewidth=2.4,
        marker="o",
        markersize=4.2,
        label="Right-hand drift",
        zorder=4,
    )
    ax.axhline(10.0, color="#bd0c0c", linewidth=1.2, linestyle="--", alpha=0.75, zorder=2)

    if not np.isnan(lock_time):
        for event_ax in [ax, ax_alpha]:
            event_ax.axvline(lock_time, color="dimgrey", linewidth=1.0, linestyle="--", alpha=0.75, zorder=1)
        ax.text(
            lock_time,
            ax.get_ylim()[1] * 0.90,
            "lock",
            rotation=90,
            ha="right",
            va="top",
            fontsize=9,
            color="dimgrey",
        )

    if not np.isnan(perturb_time):
        for event_ax in [ax, ax_alpha]:
            event_ax.axvline(perturb_time, color="#d73027", linewidth=1.2, linestyle="--", alpha=0.85, zorder=1)
        ax.text(
            perturb_time,
            ax.get_ylim()[1] * 0.90,
            "5 cm perturb",
            rotation=90,
            ha="right",
            va="top",
            fontsize=9,
            color="dimgrey",
        )

    ax_alpha.plot(
        time_s,
        alpha,
        color="#4575b4",
        linewidth=1.8,
        linestyle=":",
        marker="s",
        markersize=3.2,
        zorder=3,
    )
    ax_alpha.set_ylim(-0.05, 1.05)
    ax_alpha.set_yticks([0.0, 0.5, 1.0])

    ax_alpha.set_xlabel("Simulation time (s)", fontsize=12, labelpad=8, color="dimgrey")
    ax.set_ylabel("Right-hand world drift (mm)", fontsize=12, labelpad=8, color="dimgrey")
    ax_alpha.set_ylabel("Alpha", fontsize=12, labelpad=8, color="dimgrey")
    ax.set_title("T1 Soft-Lock Right-Hand Hold Under 5 cm Target Perturbation", fontsize=14, loc="left", pad=7, color="dimgrey")

    ax.text(
        0.98,
        0.18,
        f"Post-perturb max drift: {max_post:.2f} mm\nFinal drift: {final_drift:.2f} mm\nAcceptance line: 10 mm",
        transform=ax.transAxes,
        ha="right",
        va="bottom",
        fontsize=9,
        color="dimgrey",
        bbox={"facecolor": "white", "edgecolor": "lightgrey", "alpha": 0.85, "pad": 6},
    )

    handles = [
        Line2D([0], [0], color=pal[5], linewidth=2.4, marker="o", markersize=4.2, label="Right-hand drift"),
        Line2D([0], [0], color="#4575b4", linewidth=1.8, linestyle=":", label="Soft-lock alpha"),
        Line2D([0], [0], color="#bd0c0c", linewidth=1.2, linestyle="--", label="10 mm target"),
    ]
    ax.legend(
        handles=handles,
        loc="upper left",
        fontsize=10,
        frameon=True,
        facecolor="white",
        framealpha=0.8,
        edgecolor="lightgrey",
        labelcolor="dimgrey",
    )

    ax.tick_params(axis="both", which="both", length=0, labelcolor="dimgrey")
    ax_alpha.tick_params(axis="both", which="both", length=0, labelcolor="dimgrey")
    ax.grid(False)
    ax_alpha.grid(False)
    for plot_ax in [ax, ax_alpha]:
        for spine in ["top", "right", "left", "bottom"]:
            plot_ax.spines[spine].set_visible(False)

    # --- Save ---
    png_path = figures_dir / "t1_softlock_perturb_drift.png"
    pdf_path = figures_dir / "t1_softlock_perturb_drift.pdf"
    plt.savefig(pdf_path, dpi=150, bbox_inches="tight")
    plt.savefig(png_path, dpi=150, bbox_inches="tight")
    print(f"saved_png={png_path}")
    print(f"saved_pdf={pdf_path}")
    print(f"saved_csv={csv_path}")
    print(f"max_post_perturb_drift_mm={max_post:.6f}")
    print(f"final_drift_mm={final_drift:.6f}")


if __name__ == "__main__":
    main()
