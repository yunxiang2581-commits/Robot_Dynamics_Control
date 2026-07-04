# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "matplotlib",
#     "seaborn",
# ]
# ///
"""Analyze OpenLoong T1 hand-hold logs and plot drift curves.

这个脚本只做离线日志分析，不改变控制器数学逻辑。

输入:
    一个 T1 run 目录，里面应包含:
    - logs/02_walk_wbc_runtime.log

输出:
    写入 run_dir/analysis:
    - t1_hold_samples.csv: 手漂移样本
    - t1_stage_d_summary.txt: 关键指标摘要
    - figures/t1_drift_overview.png/pdf: 漂移与分量误差曲线
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import statistics
from pathlib import Path


def _parse_floats(text: str) -> list[float]:
    return [float(x) for x in re.findall(r"[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?", text)]


def parse_runtime_log(runtime_log: Path) -> tuple[list[dict[str, float]], dict[str, float]]:
    """Parse T1 HOLD/LOCK/PERT/FORCE logs.

    [T1-HOLD] 每 500 个 1ms 控制周期打印一次，这里映射为 time_s = sample_index * 0.5。
    """
    if not runtime_log.exists():
        raise FileNotFoundError(f"runtime log not found: {runtime_log}")

    rows: list[dict[str, float]] = []
    meta: dict[str, float] = {
        "lock_count": math.nan,
        "lock_time_s": math.nan,
        "pert_time_s": math.nan,
        "force_time_s": math.nan,
        "force_x": math.nan,
    }
    lock_seen = False
    pert_seen = False
    force_seen = False

    for line in runtime_log.read_text(errors="ignore").splitlines():
        if "[T1-LOCK]" in line:
            lock_match = re.search(r"lock_count=(\d+)", line)
            if lock_match:
                meta["lock_count"] = float(lock_match.group(1))
                meta["lock_time_s"] = meta["lock_count"] * 0.001
            lock_seen = True

        if "[T1-PERT]" in line:
            pert_match = re.search(r"trigger_time_s=([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", line)
            if pert_match:
                meta["pert_time_s"] = float(pert_match.group(1))
            pert_seen = True

        if "[T1-FORCE]" in line and "[T1-FORCE-CONFIG]" not in line:
            force_time_match = re.search(r"trigger_time_s=([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", line)
            force_xyz_match = re.search(
                r"force_xyz=\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"
                r"\s+([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"
                r"\s+([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)",
                line,
            )
            if force_time_match:
                meta["force_time_s"] = float(force_time_match.group(1))
            if force_xyz_match:
                meta["force_x"] = float(force_xyz_match.group(1))
            force_seen = True

        if "[T1-HOLD]" not in line:
            continue

        vals = _parse_floats(line.split("drift_mm=", 1)[1])
        if len(vals) < 7:
            continue

        sample_index = len(rows)
        cycle = sample_index * 500
        rows.append(
            {
                "sample_index": sample_index,
                "cycle": cycle,
                "time_s": cycle * 0.001,
                "drift_mm": vals[0],
                "des_x": vals[1],
                "des_y": vals[2],
                "des_z": vals[3],
                "cur_x": vals[4],
                "cur_y": vals[5],
                "cur_z": vals[6],
                "post_lock": 1.0 if lock_seen else 0.0,
                "post_pert": 1.0 if pert_seen else 0.0,
                "post_force": 1.0 if force_seen else 0.0,
            }
        )

    if not rows:
        raise ValueError(f"no [T1-HOLD] rows found in {runtime_log}")

    for row in rows:
        row["err_x_mm"] = (row["des_x"] - row["cur_x"]) * 1000.0
        row["err_y_mm"] = (row["des_y"] - row["cur_y"]) * 1000.0
        row["err_z_mm"] = (row["des_z"] - row["cur_z"]) * 1000.0

    return rows, meta


def write_csv(path: Path, rows: list[dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _stat_line(name: str, values: list[float]) -> str:
    if not values:
        return f"{name}: no samples"
    return (
        f"{name}: count={len(values)}, "
        f"avg={statistics.fmean(values):.3f}, "
        f"max={max(values):.3f}, "
        f"tail={values[-1]:.3f}"
    )


def summarize(rows: list[dict[str, float]], meta: dict[str, float]) -> str:
    stable_rows = [row for row in rows if row["post_lock"] > 0.5]
    post_lock = [row["drift_mm"] for row in stable_rows]
    post_pert = [row["drift_mm"] for row in stable_rows if row["post_pert"] > 0.5]
    post_force = [row["drift_mm"] for row in stable_rows if row["post_force"] > 0.5]

    lines = [
        "T1 drift offline analysis",
        "",
        f"lock_count={meta['lock_count']:.0f}" if not math.isnan(meta["lock_count"]) else "lock_count=nan",
        f"lock_time_s={meta['lock_time_s']:.3f}" if not math.isnan(meta["lock_time_s"]) else "lock_time_s=nan",
        f"pert_time_s={meta['pert_time_s']:.3f}" if not math.isnan(meta["pert_time_s"]) else "pert_time_s=nan",
        f"force_time_s={meta['force_time_s']:.3f}" if not math.isnan(meta["force_time_s"]) else "force_time_s=nan",
        "analysis_window=post_lock_only",
        _stat_line("post_lock_drift_mm", post_lock),
        _stat_line("post_pert_drift_mm", post_pert),
        _stat_line("post_force_drift_mm", post_force),
    ]
    return "\n".join(lines) + "\n"


def plot_overview(rows: list[dict[str, float]], meta: dict[str, float], figures_dir: Path) -> None:
    """Generate a 2x1 drift overview figure."""
    import matplotlib.pyplot as plt

    try:
        if os.environ.get("OPENLOONG_USE_SEABORN") != "1":
            raise ImportError("seaborn disabled by default for local Anaconda compatibility")
        import seaborn as sns

        sns.set_theme(font_scale=1.0, style="whitegrid", font="DejaVu Sans")
        pal = sns.cubehelix_palette(6, rot=-0.25, light=0.7)
        despine = lambda: sns.despine(left=True, bottom=True)
    except Exception:
        plt.rcParams.update(
            {
                "font.family": "DejaVu Sans",
                "axes.grid": True,
                "grid.color": "lightgrey",
                "grid.alpha": 0.35,
            }
        )
        pal = ["#90c1c6", "#72a5b4", "#58849f", "#446485", "#324465", "#1f253f"]
        despine = lambda: None

    figures_dir.mkdir(parents=True, exist_ok=True)

    lock_time = meta["lock_time_s"]
    plot_rows = [row for row in rows if math.isnan(lock_time) or row["time_s"] >= lock_time]
    if not plot_rows:
        plot_rows = rows
    time_origin = 0.0 if math.isnan(lock_time) else lock_time

    time = [row["time_s"] - time_origin for row in plot_rows]
    drift = [row["drift_mm"] for row in plot_rows]
    err_x = [row["err_x_mm"] for row in plot_rows]
    err_y = [row["err_y_mm"] for row in plot_rows]
    err_z = [row["err_z_mm"] for row in plot_rows]

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), dpi=150, sharex=True)
    fig.suptitle("T1 Hand Drift Overview", fontsize=14, y=0.98, color="dimgrey")

    for ax in axes:
        ax.grid(False)
        ax.tick_params(axis="both", which="both", length=0, labelcolor="dimgrey")
        ax.patch.set_edgecolor("lightgrey")
        ax.patch.set_linewidth(0.8)

    axes[0].bar(time, drift, color="#4575b4", alpha=0.55, width=0.42, zorder=2)
    axes[0].plot(time, drift, color=pal[5], linewidth=2.0, zorder=4)
    axes[0].axhline(10.0, color="#bd0c0c", linewidth=1.0, linestyle="--", zorder=1)
    axes[0].text(time[-1], 10.5, "10 mm target", ha="right", va="bottom", fontsize=8, color="dimgrey")
    axes[0].set_title(r"$\bf{(a)}$  Hand drift magnitude", loc="left", fontsize=11, pad=7)
    axes[0].set_ylabel("Drift (mm)", fontsize=10, labelpad=6, color="dimgrey")

    axes[1].plot(time, err_x, color=pal[3], linewidth=1.8, label="x error")
    axes[1].plot(time, err_y, color=pal[4], linewidth=1.8, label="y error")
    axes[1].plot(time, err_z, color=pal[5], linewidth=1.8, label="z error")
    axes[1].set_title(r"$\bf{(b)}$  Error components", loc="left", fontsize=11, pad=7)
    axes[1].set_ylabel("Error (mm)", fontsize=10, labelpad=6, color="dimgrey")
    x_label = "Time since LOCK (s)" if not math.isnan(lock_time) else "Time (s)"
    axes[1].set_xlabel(x_label, fontsize=10, labelpad=6, color="dimgrey")
    axes[1].legend(
        frameon=True,
        facecolor="white",
        framealpha=0.8,
        edgecolor="lightgrey",
        labelcolor="dimgrey",
        fontsize=9,
        loc="upper right",
    )

    for ax in axes:
        if not math.isnan(meta["lock_time_s"]):
            ax.axvline(0.0, color="dimgrey", linewidth=1.0, linestyle=":", zorder=1)
            ax.text(0.08, ax.get_ylim()[1] * 0.93, "LOCK", color="dimgrey", fontsize=8)
        if not math.isnan(meta["pert_time_s"]):
            pert_rel_time = meta["pert_time_s"] - time_origin
            ax.axvline(pert_rel_time, color="#bd0c0c", linewidth=1.0, linestyle="--", zorder=1)
            ax.text(pert_rel_time + 0.08, ax.get_ylim()[1] * 0.82, "PERT", color="dimgrey", fontsize=8)
        if not math.isnan(meta["force_time_s"]):
            force_rel_time = meta["force_time_s"] - time_origin
            ax.axvline(force_rel_time, color="#fdbf6f", linewidth=1.0, linestyle="--", zorder=1)
            ax.text(force_rel_time + 0.08, ax.get_ylim()[1] * 0.71, "FORCE", color="dimgrey", fontsize=8)

    post_event = [row["drift_mm"] for row in plot_rows if row["post_force"] > 0.5 or row["post_pert"] > 0.5]
    if post_event:
        fig.text(
            0.99,
            0.01,
            f"Post-event max drift = {max(post_event):.2f} mm",
            ha="right",
            va="bottom",
            fontsize=9,
            color="dimgrey",
            style="italic",
        )

    despine()
    for ext in ("pdf", "png"):
        fig.savefig(figures_dir / f"t1_drift_overview.{ext}", dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze a T1 run and draw drift curves.")
    parser.add_argument("run_dir", type=Path, help="OpenLoong T1 run directory")
    args = parser.parse_args()

    run_dir = args.run_dir.resolve()
    runtime_log = run_dir / "logs" / "02_walk_wbc_runtime.log"
    analysis_dir = run_dir / "analysis"
    figures_dir = analysis_dir / "figures"

    rows, meta = parse_runtime_log(runtime_log)
    write_csv(analysis_dir / "t1_hold_samples.csv", rows)

    summary = summarize(rows, meta)
    analysis_dir.mkdir(parents=True, exist_ok=True)
    (analysis_dir / "t1_stage_d_summary.txt").write_text(summary, encoding="utf-8")

    try:
        plot_overview(rows, meta, figures_dir)
    except Exception as exc:
        (analysis_dir / "plot_error.txt").write_text(str(exc), encoding="utf-8")

    print(summary, end="")
    print(f"analysis_dir={analysis_dir}")


if __name__ == "__main__":
    main()
