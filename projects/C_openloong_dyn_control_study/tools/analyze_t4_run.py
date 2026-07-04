# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "matplotlib",
# ]
# ///
"""Analyze OpenLoong T4 in-place stepping hand-hold logs.

这个脚本只做离线日志分析，不改变控制器数学逻辑。

输入:
    一个 T4 run 目录，里面应包含:
    - logs/02_walk_wbc_runtime.log
    - runtime_record/datalog.log

输出:
    写入 run_dir/analysis:
    - t4_hold_samples.csv: 右手世界系位置保持日志样本
    - base_pose_samples.csv: base 位姿抽样
    - t4_stage_d_summary.txt: done-check 指标摘要
    - figures/t4_stage_d_overview.png/pdf: 漂移与 base 轨迹概览图
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
    """从一行诊断日志里提取浮点数，支持科学计数法。"""
    return [float(x) for x in re.findall(r"[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?", text)]


def parse_runtime_log(runtime_log: Path) -> tuple[list[dict[str, float]], dict[str, float]]:
    """解析 [T4-HOLD]、[T4-LOCK]、[T4-INPLACE] 日志。

    [T4-HOLD] 每 500 个 1ms 控制周期打印一次，所以这里把 sample_index
    映射为近似时间 time_s = sample_index * 0.5。
    """
    if not runtime_log.exists():
        raise FileNotFoundError(f"runtime log not found: {runtime_log}")

    hold_rows: list[dict[str, float]] = []
    meta: dict[str, float] = {
        "lock_count": math.nan,
        "lock_time_s": math.nan,
        "anchor_x": math.nan,
        "anchor_y": math.nan,
        "anchor_z": math.nan,
        "anchor_yaw": math.nan,
    }

    for line in runtime_log.read_text(errors="ignore").splitlines():
        if "[T4-INPLACE]" in line and "base_anchor=" in line:
            vals = _parse_floats(line.split("base_anchor=", 1)[1])
            if len(vals) >= 4:
                meta["anchor_x"], meta["anchor_y"], meta["anchor_z"], meta["anchor_yaw"] = vals[:4]

        if "[T4-LOCK]" in line:
            lock_match = re.search(r"lock_count=(\d+)", line)
            if lock_match:
                meta["lock_count"] = float(lock_match.group(1))
                meta["lock_time_s"] = meta["lock_count"] * 0.001

        if "[T4-HOLD]" not in line:
            continue

        vals = _parse_floats(line.split("drift_mm=", 1)[1])
        # 期望格式:
        # drift_mm, des_x, des_y, des_z, cur_x, cur_y, cur_z
        if len(vals) < 7:
            continue
        sample_index = len(hold_rows)
        cycle = sample_index * 500
        hold_rows.append(
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
            }
        )

    if not hold_rows:
        raise ValueError(f"no [T4-HOLD] rows found in {runtime_log}")

    lock_time = meta["lock_time_s"]
    for row in hold_rows:
        row["post_lock"] = 0.0 if math.isnan(lock_time) or row["time_s"] < lock_time else 1.0
        row["err_x_mm"] = (row["des_x"] - row["cur_x"]) * 1000.0
        row["err_y_mm"] = (row["des_y"] - row["cur_y"]) * 1000.0
        row["err_z_mm"] = (row["des_z"] - row["cur_z"]) * 1000.0

    return hold_rows, meta


def parse_base_pose(datalog: Path, meta: dict[str, float], stride: int = 20) -> list[dict[str, float]]:
    """从 datalog.log 中抽取 base 位姿。

    datalog 列约定来自 matlabReadDataScript.txt:
    - simTime: col 1
    - rpy: col 64:66
    - basePos: col 73:75
    """
    if not datalog.exists():
        raise FileNotFoundError(f"datalog not found: {datalog}")

    rows: list[dict[str, float]] = []
    anchor_x = meta.get("anchor_x", math.nan)
    anchor_y = meta.get("anchor_y", math.nan)
    anchor_z = meta.get("anchor_z", math.nan)

    with datalog.open("r", newline="") as f:
        reader = csv.reader(f)
        for idx, parts in enumerate(reader):
            if idx % stride != 0:
                continue
            if len(parts) < 84:
                continue
            vals = [float(x) for x in parts]
            x, y, z = vals[72], vals[73], vals[74]
            roll, pitch, yaw = vals[63], vals[64], vals[65]
            dx = x - anchor_x if not math.isnan(anchor_x) else math.nan
            dy = y - anchor_y if not math.isnan(anchor_y) else math.nan
            dz = z - anchor_z if not math.isnan(anchor_z) else math.nan
            rows.append(
                {
                    "time_s": vals[0],
                    "base_x": x,
                    "base_y": y,
                    "base_z": z,
                    "roll": roll,
                    "pitch": pitch,
                    "yaw": yaw,
                    "base_dx_mm": dx * 1000.0,
                    "base_dy_mm": dy * 1000.0,
                    "base_dz_mm": dz * 1000.0,
                    "base_xy_from_anchor_mm": math.hypot(dx, dy) * 1000.0,
                }
            )

    if not rows:
        raise ValueError(f"no base pose rows parsed from {datalog}")

    return rows


def write_csv(path: Path, rows: list[dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def parse_t5_posture_log(runtime_log: Path) -> tuple[list[dict[str, float]], dict[str, object]]:
    """Parse [T5-POSTURE] diagnostics from a stand smoke log.

    The controller prints one T5 posture diagnostic every 500 control cycles.
    With the 1 ms control step used by this demo, sample_index maps to an
    approximate simulation time of sample_index * 0.5 s.
    """
    if not runtime_log.exists():
        raise FileNotFoundError(f"T5 runtime log not found: {runtime_log}")

    rows: list[dict[str, float]] = []
    meta: dict[str, object] = {
        "sample_period_s": 0.5,
        "lock_count": math.nan,
        "lock_time_s": math.nan,
        "lock_pos_x": math.nan,
        "lock_pos_y": math.nan,
        "lock_pos_z": math.nan,
        "lock_rot_des_angle_deg": math.nan,
        "angular_j_cols": "",
        "model_nv": math.nan,
    }

    last_row: dict[str, float] | None = None
    for line in runtime_log.read_text(errors="ignore").splitlines():
        if "[T5-LOCK]" in line:
            pos_match = re.search(r"hd_r_pos_des_W=(.*?)\s+rot_des_angle_deg=", line)
            rot_match = re.search(r"rot_des_angle_deg=([^\s]+)", line)
            lock_match = re.search(r"lock_count=(\d+)", line)
            if pos_match is not None:
                pos_vals = _parse_floats(pos_match.group(1))
                if len(pos_vals) >= 3:
                    meta["lock_pos_x"], meta["lock_pos_y"], meta["lock_pos_z"] = pos_vals[:3]
            if rot_match is not None:
                meta["lock_rot_des_angle_deg"] = float(rot_match.group(1))
            if lock_match is not None:
                meta["lock_count"] = float(lock_match.group(1))
                meta["lock_time_s"] = float(lock_match.group(1)) * 0.001

        if "[T5-POSTURE] pos_err_mm=" in line:
            match = re.search(
                r"pos_err_mm=([^\s]+)\s+rot_err_deg=([^\s]+)\s+locked=([01])",
                line,
            )
            if match is None:
                continue
            sample_index = len(rows)
            row = {
                "sample_index": float(sample_index),
                "cycle": float(sample_index * 500),
                "time_s": sample_index * 0.5,
                "pos_err_mm": float(match.group(1)),
                "rot_err_deg": float(match.group(2)),
                "locked": float(match.group(3)),
                "angular_j_cols_count": math.nan,
                "model_nv": math.nan,
            }
            rows.append(row)
            last_row = row
            continue

        if "[T5-POSTURE] angular J nonzero cols:" in line and last_row is not None:
            cols_part = line.split("cols:", 1)[1].split("(", 1)[0].strip()
            cols = [int(x) for x in re.findall(r"\d+", cols_part)]
            model_match = re.search(r"model_nv=(\d+)", line)
            model_nv = float(model_match.group(1)) if model_match else math.nan
            last_row["angular_j_cols_count"] = float(len(cols))
            last_row["model_nv"] = model_nv
            meta["angular_j_cols"] = " ".join(str(c) for c in cols)
            meta["model_nv"] = model_nv

    if not rows:
        raise ValueError(f"no [T5-POSTURE] rows found in {runtime_log}")

    return rows, meta


def summarize_t5_posture(rows: list[dict[str, float]], meta: dict[str, object], runtime_log: Path) -> str:
    """Build the T5 stand posture done-check summary."""
    locked_rows = [row for row in rows if row["locked"] > 0.5]

    def stat_line(name: str, values: list[float]) -> str:
        if not values:
            return f"{name}: count=0, avg=nan, max=nan, tail=nan"
        return (
            f"{name}: count={len(values)}, "
            f"avg={statistics.fmean(values):.3f}, "
            f"max={max(values):.3f}, "
            f"tail={values[-1]:.3f}"
        )

    pos_values = [row["pos_err_mm"] for row in locked_rows]
    rot_values = [row["rot_err_deg"] for row in locked_rows]
    pos_pass = bool(pos_values) and max(pos_values) < 20.0
    rot_pass = bool(rot_values) and max(rot_values) < 5.0

    lines = [
        "T5 stand posture smoke analysis",
        "",
        f"runtime_log={runtime_log}",
        f"sample_period_s={float(meta['sample_period_s']):.3f}",
        f"total_samples={len(rows)}",
        f"locked_samples={len(locked_rows)}",
        f"lock_count={float(meta['lock_count']):.0f}",
        f"lock_time_s={float(meta['lock_time_s']):.3f}",
        (
            "lock_pose="
            f"[pos=({float(meta['lock_pos_x']):.6f}, "
            f"{float(meta['lock_pos_y']):.6f}, {float(meta['lock_pos_z']):.6f}), "
            f"rot_des_angle_deg={float(meta['lock_rot_des_angle_deg']):.3f}]"
        ),
        f"angular_j_nonzero_cols(raw)={meta['angular_j_cols']}",
        f"model_nv={float(meta['model_nv']):.0f}",
        "",
        stat_line("locked_pos_err_mm", pos_values),
        stat_line("locked_rot_err_deg", rot_values),
        "",
        "done_check: locked_pos_err_max_mm < 20.0 and locked_rot_err_max_deg < 5.0",
        f"done_check_result={'PASS' if pos_pass and rot_pass else 'FAIL'}",
    ]
    return "\n".join(lines) + "\n"


def plot_t5_posture(rows: list[dict[str, float]], meta: dict[str, object], figures_dir: Path) -> None:
    """Render T5 position/orientation error over diagnostic samples."""
    import matplotlib.pyplot as plt

    try:
        if os.environ.get("OPENLOONG_USE_SEABORN") != "1":
            raise ImportError("seaborn disabled by default for local Anaconda compatibility")
        import seaborn as sns

        sns.set_theme(font_scale=1.0, style="whitegrid", font="DejaVu Sans")
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
        despine = lambda: None

    figures_dir.mkdir(parents=True, exist_ok=True)

    time = [row["time_s"] for row in rows]
    pos = [row["pos_err_mm"] for row in rows]
    rot = [row["rot_err_deg"] for row in rows]
    locked = [row["locked"] > 0.5 for row in rows]
    lock_time = float(meta["lock_time_s"])

    pos_col = "#58849f"
    rot_col = "#bd0c0c"
    lock_col = "dimgrey"

    fig, axes = plt.subplots(2, 1, figsize=(9, 6.0), dpi=150, sharex=True)
    fig.suptitle("T5 stand: 6-DoF right-hand pose hold smoke", fontsize=13, y=0.99, color="dimgrey")

    for ax in axes:
        ax.tick_params(axis="both", which="both", length=0, labelcolor="dimgrey")
        ax.grid(False)
        ax.patch.set_edgecolor("lightgrey")
        ax.patch.set_linewidth(0.8)
        ax.axvline(lock_time, color=lock_col, linewidth=1.0, linestyle=":", label="pose lock")

    axes[0].plot(time, pos, color=pos_col, linewidth=2.0, marker="o", markersize=4, label="position error")
    axes[0].axhline(20.0, color=rot_col, linewidth=1.0, linestyle="--", label="20 mm target")
    axes[0].set_ylabel("Position error (mm)", fontsize=9, labelpad=6, color="dimgrey")
    axes[0].set_title(r"$\bf{(a)}$ Right-hand world position error", loc="left", fontsize=11, pad=7)
    axes[0].legend(
        frameon=True,
        facecolor="white",
        framealpha=0.8,
        edgecolor="lightgrey",
        labelcolor="dimgrey",
        fontsize=8,
        loc="upper right",
    )

    axes[1].plot(time, rot, color=rot_col, linewidth=2.0, marker="o", markersize=4, label="orientation error")
    axes[1].axhline(5.0, color=rot_col, linewidth=1.0, linestyle="--", alpha=0.8, label="5 deg target")
    axes[1].set_ylabel("Orientation error (deg)", fontsize=9, labelpad=6, color="dimgrey")
    axes[1].set_xlabel("Approx. simulation time (s)", fontsize=9, labelpad=6, color="dimgrey")
    axes[1].set_title(r"$\bf{(b)}$ SO(3) log orientation error", loc="left", fontsize=11, pad=7)
    axes[1].legend(
        frameon=True,
        facecolor="white",
        framealpha=0.8,
        edgecolor="lightgrey",
        labelcolor="dimgrey",
        fontsize=8,
        loc="upper right",
    )

    locked_pos = [row["pos_err_mm"] for row in rows if row["locked"] > 0.5]
    locked_rot = [row["rot_err_deg"] for row in rows if row["locked"] > 0.5]
    if locked_pos and locked_rot:
        fig.text(
            0.10,
            0.02,
            (
                f"locked max: {max(locked_pos):.2f} mm, {max(locked_rot):.2f} deg "
                f"(targets: <20 mm, <5 deg)"
            ),
            ha="left",
            va="bottom",
            fontsize=9,
            color="dimgrey",
            style="italic",
            bbox={"facecolor": "white", "alpha": 0.78, "edgecolor": "lightgrey", "pad": 5},
        )

    for ax, vals in zip(axes, (pos, rot)):
        y_max = max(vals + ([20.0] if ax is axes[0] else [5.0]))
        ax.set_ylim(bottom=min(0.0, min(vals) * 0.95), top=y_max * 1.20 if y_max > 0 else 1.0)

    # Explicitly mark pre-lock vs post-lock samples without relying on color alone.
    for ax, vals in zip(axes, (pos, rot)):
        for t, value, is_locked in zip(time, vals, locked):
            if is_locked:
                ax.scatter([t], [value], s=52, facecolors="none", edgecolors="black", linewidths=0.8, zorder=5)

    despine()
    fig.tight_layout(rect=[0, 0.05, 1, 0.96])
    for ext in ("pdf", "png"):
        fig.savefig(figures_dir / f"t5_posture_smoke_errors.{ext}", dpi=150, bbox_inches="tight")
    plt.close(fig)


def analyze_t5_posture_log(runtime_log: Path, output_dir: Path) -> str:
    """Parse T5 posture smoke logs, write CSV/summary/figure, and return summary text."""
    rows, meta = parse_t5_posture_log(runtime_log)
    output_dir.mkdir(parents=True, exist_ok=True)
    figures_dir = output_dir / "figures"

    write_csv(output_dir / "t5_posture_samples.csv", rows)
    summary = summarize_t5_posture(rows, meta, runtime_log)
    (output_dir / "t5_posture_summary.txt").write_text(summary, encoding="utf-8")

    try:
        plot_t5_posture(rows, meta, figures_dir)
    except Exception as exc:
        (output_dir / "t5_posture_plot_error.txt").write_text(str(exc), encoding="utf-8")

    return summary


def compute_compensation(
    hold_rows: list[dict[str, float]],
    base_rows: list[dict[str, float]],
    meta: dict[str, float],
) -> dict[str, float]:
    """量化零空间补偿有效性。

    核心思想: 锁定后 base 实际仍在摆动 (base_xy_from_anchor_mm)，但右手在世界系
    几乎钉住 (drift_mm)。若手臂不补偿，手会跟着 base 一起动；手漂移远小于 base 摆动
    正是零空间补偿在起作用的直接证据。

    compensation_ratio = base_sway / hand_drift (取 post-lock 峰值与均值两种口径)
    compensation_pct    = (1 - hand_drift / base_sway) * 100  # base 运动被补偿掉的比例
    """
    lock_time = meta.get("lock_time_s", math.nan)
    post_hold = [r for r in hold_rows if r["post_lock"] > 0.5]
    post_base = [
        r
        for r in base_rows
        if math.isnan(lock_time) or r["time_s"] >= lock_time
    ]
    result: dict[str, float] = {
        "hand_drift_max_mm": math.nan,
        "hand_drift_avg_mm": math.nan,
        "base_sway_max_mm": math.nan,
        "base_sway_avg_mm": math.nan,
        "comp_ratio_max": math.nan,
        "comp_ratio_avg": math.nan,
        "comp_pct_max": math.nan,
        "comp_pct_avg": math.nan,
    }
    if not post_hold or not post_base:
        return result

    hand_drift_max = max(r["drift_mm"] for r in post_hold)
    hand_drift_avg = statistics.fmean(r["drift_mm"] for r in post_hold)
    base_sway_vals = [
        r["base_xy_from_anchor_mm"]
        for r in post_base
        if not math.isnan(r["base_xy_from_anchor_mm"])
    ]
    if not base_sway_vals:
        return result
    base_sway_max = max(base_sway_vals)
    base_sway_avg = statistics.fmean(base_sway_vals)

    result["hand_drift_max_mm"] = hand_drift_max
    result["hand_drift_avg_mm"] = hand_drift_avg
    result["base_sway_max_mm"] = base_sway_max
    result["base_sway_avg_mm"] = base_sway_avg
    if hand_drift_max > 1e-9:
        result["comp_ratio_max"] = base_sway_max / hand_drift_max
        result["comp_pct_max"] = (1.0 - hand_drift_max / base_sway_max) * 100.0
    if hand_drift_avg > 1e-9:
        result["comp_ratio_avg"] = base_sway_avg / hand_drift_avg
        result["comp_pct_avg"] = (1.0 - hand_drift_avg / base_sway_avg) * 100.0
    return result


def summarize(hold_rows: list[dict[str, float]], base_rows: list[dict[str, float]], meta: dict[str, float]) -> str:
    post_lock = [row for row in hold_rows if row["post_lock"] > 0.5]
    drift = [row["drift_mm"] for row in post_lock]
    base_after_lock = [row for row in base_rows if row["time_s"] >= meta["lock_time_s"]]
    base_xy = [row["base_xy_from_anchor_mm"] for row in base_after_lock]

    def stat_line(name: str, values: list[float]) -> str:
        return (
            f"{name}: count={len(values)}, "
            f"avg={statistics.fmean(values):.3f}, "
            f"max={max(values):.3f}, "
            f"tail={values[-1]:.3f}"
        )

    comp = compute_compensation(hold_rows, base_rows, meta)

    lines = [
        "T4 Stage-D offline analysis",
        "",
        f"lock_count={meta['lock_count']:.0f}",
        f"lock_time_s={meta['lock_time_s']:.3f}",
        (
            "base_anchor="
            f"[{meta['anchor_x']:.6f}, {meta['anchor_y']:.6f}, "
            f"{meta['anchor_z']:.6f}, yaw={meta['anchor_yaw']:.6f}]"
        ),
        stat_line("post_lock_hand_drift_mm", drift),
        stat_line("post_lock_base_xy_from_anchor_mm", base_xy),
        "",
        "nullspace_compensation (base sways, hand stays -> arm compensates):",
        (
            f"  base_sway_max_mm={comp['base_sway_max_mm']:.3f}, "
            f"hand_drift_max_mm={comp['hand_drift_max_mm']:.3f} "
            f"-> comp_ratio_max={comp['comp_ratio_max']:.2f}x "
            f"({comp['comp_pct_max']:.1f}% compensated)"
        ),
        (
            f"  base_sway_avg_mm={comp['base_sway_avg_mm']:.3f}, "
            f"hand_drift_avg_mm={comp['hand_drift_avg_mm']:.3f} "
            f"-> comp_ratio_avg={comp['comp_ratio_avg']:.2f}x "
            f"({comp['comp_pct_avg']:.1f}% compensated)"
        ),
        "",
        "done_check: post_lock_hand_drift_max_mm < 20.0",
        f"done_check_result={'PASS' if max(drift) < 20.0 else 'FAIL'}",
    ]
    return "\n".join(lines) + "\n"


def plot_overview(hold_rows: list[dict[str, float]], base_rows: list[dict[str, float]], meta: dict[str, float], figures_dir: Path) -> None:
    """生成 2x2 概览图；若 matplotlib 不可用，调用方会跳过。"""
    import matplotlib.gridspec as gridspec
    import matplotlib.pyplot as plt

    try:
        if os.environ.get("OPENLOONG_USE_SEABORN") != "1":
            raise ImportError("seaborn disabled by default for local Anaconda compatibility")
        import seaborn as sns

        sns.set_theme(font_scale=1.0, style="whitegrid", font="DejaVu Sans")
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
        despine = lambda: None

    figures_dir.mkdir(parents=True, exist_ok=True)
    pal = ["#90c1c6", "#72a5b4", "#58849f", "#446485", "#324465", "#1f253f"]
    accent_red = "#bd0c0c"

    fig = plt.figure(figsize=(10, 8), dpi=150)
    gs = gridspec.GridSpec(2, 2)
    gs.update(wspace=0.10, hspace=0.30, left=0.08, right=0.99, top=0.91, bottom=0.08)
    axes = [plt.subplot(gs[i, j]) for i in range(2) for j in range(2)]
    fig.suptitle("T4 In-place Stepping: Right-hand Hold and Base Motion", fontsize=14, y=0.98, color="dimgrey")

    time = [row["time_s"] for row in hold_rows]
    drift = [row["drift_mm"] for row in hold_rows]
    err_x = [row["err_x_mm"] for row in hold_rows]
    err_y = [row["err_y_mm"] for row in hold_rows]
    err_z = [row["err_z_mm"] for row in hold_rows]
    lock_time = meta["lock_time_s"]
    base_plot_rows = [row for row in base_rows if row["time_s"] >= lock_time]
    base_plot_time = [row["time_s"] for row in base_plot_rows]
    base_plot_dx = [row["base_dx_mm"] for row in base_plot_rows]
    base_plot_dy = [row["base_dy_mm"] for row in base_plot_rows]
    base_plot_dz = [row["base_dz_mm"] for row in base_plot_rows]
    base_plot_xy = [row["base_xy_from_anchor_mm"] for row in base_plot_rows]

    for ax in axes:
        ax.grid(False)
        ax.tick_params(axis="both", which="both", length=0, labelcolor="dimgrey")
        ax.patch.set_edgecolor("lightgrey")
        ax.patch.set_linewidth(0.8)

    axes[0].plot(time, drift, color=pal[5], linewidth=2.0, zorder=3)
    axes[0].scatter(time, drift, color=pal[5], s=14, edgecolors="white", linewidths=0.5, zorder=4)
    axes[0].axhline(20.0, color=accent_red, linewidth=1.2, linestyle="--")
    axes[0].axvline(lock_time, color="dimgrey", linewidth=1.0, linestyle=":")
    axes[0].text(lock_time + 0.3, 20.8, "20 mm threshold", color="dimgrey", fontsize=8)
    axes[0].set_title(r"$\bf{(a)}$  Right-hand drift", loc="left", fontsize=11, pad=7)
    axes[0].set_ylabel("Drift (mm)", fontsize=9, labelpad=6, color="dimgrey")

    axes[1].plot(time, err_x, color=pal[3], linewidth=1.6, label="x error")
    axes[1].plot(time, err_y, color=pal[4], linewidth=1.6, label="y error")
    axes[1].plot(time, err_z, color=pal[5], linewidth=1.6, label="z error")
    axes[1].axvline(lock_time, color="dimgrey", linewidth=1.0, linestyle=":")
    axes[1].set_title(r"$\bf{(b)}$  Hand error components", loc="left", fontsize=11, pad=7)
    axes[1].set_ylabel("Component error (mm)", fontsize=9, labelpad=6, color="dimgrey")
    axes[1].legend(
        frameon=True,
        facecolor="white",
        framealpha=0.8,
        edgecolor="lightgrey",
        labelcolor="dimgrey",
        fontsize=8,
        loc="upper right",
    )

    axes[2].plot(base_plot_dx, base_plot_dy, color=pal[5], linewidth=1.5)
    axes[2].scatter([0.0], [0.0], color=accent_red, s=30, zorder=4, label="anchor")
    axes[2].scatter([base_plot_dx[-1]], [base_plot_dy[-1]], color=pal[2], s=28, zorder=4, label="tail")
    axes[2].set_aspect("equal", adjustable="datalim")
    axes[2].set_title(r"$\bf{(c)}$  Post-lock base XY offset", loc="left", fontsize=11, pad=7)
    axes[2].set_ylabel("Y offset (mm)", fontsize=9, labelpad=6, color="dimgrey")
    axes[2].set_xlabel("X offset (mm)", fontsize=9, labelpad=6, color="dimgrey")
    axes[2].legend(
        frameon=True,
        facecolor="white",
        framealpha=0.8,
        edgecolor="lightgrey",
        labelcolor="dimgrey",
        fontsize=8,
        loc="best",
    )

    axes[3].plot(base_plot_time, base_plot_xy, color=pal[5], linewidth=1.6, label="XY offset")
    axes[3].plot(base_plot_time, base_plot_dz, color=pal[2], linewidth=1.2, label="Z offset")
    axes[3].axvline(lock_time, color="dimgrey", linewidth=1.0, linestyle=":")
    axes[3].set_title(r"$\bf{(d)}$  Post-lock base offset over time", loc="left", fontsize=11, pad=7)
    axes[3].set_ylabel("Offset (mm)", fontsize=9, labelpad=6, color="dimgrey")
    axes[3].set_xlabel("Time (s)", fontsize=9, labelpad=6, color="dimgrey")
    axes[3].legend(
        frameon=True,
        facecolor="white",
        framealpha=0.8,
        edgecolor="lightgrey",
        labelcolor="dimgrey",
        fontsize=8,
        loc="upper right",
    )

    post_lock = [row for row in hold_rows if row["post_lock"] > 0.5]
    max_drift = max(row["drift_mm"] for row in post_lock)
    fig.text(
        0.99,
        0.01,
        f"Post-lock max hand drift = {max_drift:.2f} mm; threshold = 20 mm.",
        ha="right",
        va="bottom",
        fontsize=9,
        color="dimgrey",
        style="italic",
    )

    despine()
    for ext in ("pdf", "png"):
        fig.savefig(figures_dir / f"t4_stage_d_overview.{ext}", dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_compensation(
    hold_rows: list[dict[str, float]],
    base_rows: list[dict[str, float]],
    meta: dict[str, float],
    comp: dict[str, float],
    figures_dir: Path,
) -> None:
    """核心卖点图: base 摆动 vs 右手世界漂移叠加在同一时间轴。

    一眼看懂 "base 在晃、手是平的" —— 零空间补偿的直接视觉证据。
    单独存文件，便于直接放 README / 简历。
    """
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "axes.grid": True,
            "grid.color": "lightgrey",
            "grid.alpha": 0.35,
        }
    )

    figures_dir.mkdir(parents=True, exist_ok=True)
    base_col = "#58849f"
    hand_col = "#bd0c0c"

    lock_time = meta.get("lock_time_s", math.nan)
    post_hold = [r for r in hold_rows if r["post_lock"] > 0.5]
    post_base = [
        r
        for r in base_rows
        if math.isnan(lock_time) or r["time_s"] >= lock_time
    ]
    if not post_hold or not post_base:
        return

    hand_t = [r["time_s"] for r in post_hold]
    hand_d = [r["drift_mm"] for r in post_hold]
    base_t = [r["time_s"] for r in post_base]
    base_s = [r["base_xy_from_anchor_mm"] for r in post_base]

    fig, ax = plt.subplots(figsize=(9, 4.2), dpi=150)
    ax.tick_params(axis="both", which="both", length=0, labelcolor="dimgrey")
    ax.grid(False)

    ax.plot(base_t, base_s, color=base_col, linewidth=1.8, label="Base XY sway (actual)")
    ax.fill_between(base_t, base_s, color=base_col, alpha=0.12)
    ax.plot(hand_t, hand_d, color=hand_col, linewidth=2.2, label="Right-hand world drift")
    ax.scatter(hand_t, hand_d, color=hand_col, s=16, edgecolors="white", linewidths=0.5, zorder=4)

    ax.set_title(
        "T4: base sways while the hand stays fixed  —  null-space compensation",
        loc="left",
        fontsize=12,
        pad=8,
        color="dimgrey",
    )
    ax.set_xlabel("Time (s)", fontsize=9, labelpad=6, color="dimgrey")
    ax.set_ylabel("Displacement from anchor (mm)", fontsize=9, labelpad=6, color="dimgrey")
    ax.legend(
        frameon=True,
        facecolor="white",
        framealpha=0.85,
        edgecolor="lightgrey",
        labelcolor="dimgrey",
        fontsize=9,
        loc="upper right",
    )

    ratio = comp.get("comp_ratio_max", math.nan)
    pct = comp.get("comp_pct_max", math.nan)
    if not math.isnan(ratio):
        ax.text(
            0.01,
            0.97,
            f"peak base sway / hand drift = {ratio:.1f}x  ({pct:.0f}% of base motion compensated)",
            transform=ax.transAxes,
            ha="left",
            va="top",
            fontsize=9,
            color="dimgrey",
            style="italic",
        )

    for ext in ("pdf", "png"):
        fig.savefig(figures_dir / f"t4_nullspace_compensation.{ext}", dpi=150, bbox_inches="tight")
    plt.close(fig)


def steady_hand_drift(
    hold_rows: list[dict[str, float]],
    meta: dict[str, float],
    steady_start_s: float,
) -> list[float]:
    """取稳态窗口 (t >= steady_start_s 且已锁定) 内的手漂移样本。

    避开启动瞬态: 之前发现 base 峰值 131mm 出现在锁定后 0.7s 的启动过冲，
    拿它算补偿率会虚高。稳态窗口给出诚实的对照。
    """
    lock_time = meta.get("lock_time_s", math.nan)
    out: list[float] = []
    for r in hold_rows:
        if r["post_lock"] <= 0.5:
            continue
        if not math.isnan(lock_time) and r["time_s"] < max(lock_time, steady_start_s):
            continue
        if math.isnan(lock_time) and r["time_s"] < steady_start_s:
            continue
        out.append(r["drift_mm"])
    return out


def compute_counterfactual(
    comp_hold: list[dict[str, float]],
    comp_meta: dict[str, float],
    base_hold: list[dict[str, float]],
    base_meta: dict[str, float],
    steady_start_s: float,
) -> dict[str, float]:
    """真·反事实补偿率: 同样的踏步 base 运动，手臂补偿 vs 不补偿。

    D_on  = 补偿 run 稳态手漂移 (右臂笛卡尔任务在补偿 base 运动)
    D_off = 基线 run 稳态手漂移 (右臂锁死关节姿态，不补偿)
    补偿率 = (1 - D_on/D_off) * 100%   (base 引起的手运动被手臂抵消的比例)

    两个 run 的下肢步态、base 运动相同，唯一变量是右臂补不补偿，
    所以这个比值无可辩驳，不像 base_xy/hand 那样分母选错。
    """
    d_on = steady_hand_drift(comp_hold, comp_meta, steady_start_s)
    d_off = steady_hand_drift(base_hold, base_meta, steady_start_s)
    result: dict[str, float] = {
        "steady_start_s": steady_start_s,
        "d_on_avg_mm": math.nan,
        "d_on_max_mm": math.nan,
        "d_off_avg_mm": math.nan,
        "d_off_max_mm": math.nan,
        "comp_pct_avg": math.nan,
        "comp_pct_max": math.nan,
        "comp_ratio_avg": math.nan,
        "comp_ratio_max": math.nan,
        "n_on": float(len(d_on)),
        "n_off": float(len(d_off)),
    }
    if not d_on or not d_off:
        return result
    d_on_avg = statistics.fmean(d_on)
    d_on_max = max(d_on)
    d_off_avg = statistics.fmean(d_off)
    d_off_max = max(d_off)
    result["d_on_avg_mm"] = d_on_avg
    result["d_on_max_mm"] = d_on_max
    result["d_off_avg_mm"] = d_off_avg
    result["d_off_max_mm"] = d_off_max
    if d_off_avg > 1e-9:
        result["comp_pct_avg"] = (1.0 - d_on_avg / d_off_avg) * 100.0
        result["comp_ratio_avg"] = d_off_avg / d_on_avg
    if d_off_max > 1e-9:
        result["comp_pct_max"] = (1.0 - d_on_max / d_off_max) * 100.0
        result["comp_ratio_max"] = d_off_max / d_on_max
    return result


def plot_counterfactual(
    comp_hold: list[dict[str, float]],
    comp_meta: dict[str, float],
    base_hold: list[dict[str, float]],
    base_meta: dict[str, float],
    cf: dict[str, float],
    figures_dir: Path,
) -> None:
    """诚实的对照图: 手臂补偿(红) vs 不补偿(灰) 的手世界漂移，同一时间轴。

    这是真反事实: 两条曲线来自同样的踏步，唯一区别是右臂补不补偿。
    """
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "axes.grid": True,
            "grid.color": "lightgrey",
            "grid.alpha": 0.35,
        }
    )
    figures_dir.mkdir(parents=True, exist_ok=True)
    on_col = "#bd0c0c"
    off_col = "#8a8f99"

    on_t = [r["time_s"] for r in comp_hold if r["post_lock"] > 0.5]
    on_d = [r["drift_mm"] for r in comp_hold if r["post_lock"] > 0.5]
    off_t = [r["time_s"] for r in base_hold if r["post_lock"] > 0.5]
    off_d = [r["drift_mm"] for r in base_hold if r["post_lock"] > 0.5]
    if not on_t or not off_t:
        return

    fig, ax = plt.subplots(figsize=(9, 4.2), dpi=150)
    ax.tick_params(axis="both", which="both", length=0, labelcolor="dimgrey")
    ax.grid(False)

    ax.plot(off_t, off_d, color=off_col, linewidth=2.0, label="Arm locked (no compensation)")
    ax.fill_between(off_t, off_d, color=off_col, alpha=0.12)
    ax.plot(on_t, on_d, color=on_col, linewidth=2.2, label="Cartesian hand task (compensating)")
    ax.scatter(on_t, on_d, color=on_col, s=14, edgecolors="white", linewidths=0.5, zorder=4)

    steady = cf.get("steady_start_s", math.nan)
    if not math.isnan(steady):
        ax.axvline(steady, color="dimgrey", linewidth=1.0, linestyle=":")
        ax.text(steady + 0.2, ax.get_ylim()[1] * 0.95, "steady-state window",
                color="dimgrey", fontsize=8, va="top")

    ax.set_title(
        "T4 counterfactual: right-hand world drift, arm compensating vs locked",
        loc="left",
        fontsize=12,
        pad=8,
        color="dimgrey",
    )
    ax.set_xlabel("Time (s)", fontsize=9, labelpad=6, color="dimgrey")
    ax.set_ylabel("Hand world drift (mm)", fontsize=9, labelpad=6, color="dimgrey")
    ax.legend(
        frameon=True,
        facecolor="white",
        framealpha=0.85,
        edgecolor="lightgrey",
        labelcolor="dimgrey",
        fontsize=9,
        loc="upper right",
    )

    pct = cf.get("comp_pct_avg", math.nan)
    d_on = cf.get("d_on_avg_mm", math.nan)
    d_off = cf.get("d_off_avg_mm", math.nan)
    if not math.isnan(pct):
        fig.subplots_adjust(bottom=0.22)
        fig.text(
            0.10,
            0.035,
            (
                f"steady-state: {d_on:.1f} mm with task vs {d_off:.1f} mm locked\n"
                f"hand drift reduced by {pct:.0f}%"
            ),
            ha="left",
            va="bottom",
            fontsize=9,
            color="dimgrey",
            style="italic",
            bbox={"facecolor": "white", "alpha": 0.78, "edgecolor": "lightgrey", "pad": 5},
        )

    for ext in ("pdf", "png"):
        fig.savefig(figures_dir / f"t4_counterfactual_compensation.{ext}", dpi=150, bbox_inches="tight")
    plt.close(fig)


def _rows_by_time(rows: list[dict[str, float]], steady_start_s: float) -> dict[float, dict[str, float]]:
    """按时间戳对齐 base samples，保留稳态窗口之后的数据。"""
    return {
        round(row["time_s"], 6): row
        for row in rows
        if row["time_s"] >= steady_start_s
    }


def compute_base_consistency(
    comp_base_rows: list[dict[str, float]],
    comp_meta: dict[str, float],
    baseline_base_rows: list[dict[str, float]],
    baseline_meta: dict[str, float],
    steady_start_s: float,
) -> dict[str, float]:
    """验证两次 run 的 base 运动是否足够接近。

    这里比较的是相对各自 base_anchor 的 XY 轨迹。若两次 run 只有手臂任务不同，
    那么 base_xy_from_anchor_mm 和 (base_dx_mm, base_dy_mm) 应该高度接近。
    """
    comp_by_t = _rows_by_time(comp_base_rows, steady_start_s)
    base_by_t = _rows_by_time(baseline_base_rows, steady_start_s)
    common_times = sorted(set(comp_by_t) & set(base_by_t))

    result: dict[str, float] = {
        "steady_start_s": steady_start_s,
        "common_samples": float(len(common_times)),
        "time_start_s": math.nan,
        "time_end_s": math.nan,
        "comp_base_xy_avg_mm": math.nan,
        "comp_base_xy_max_mm": math.nan,
        "baseline_base_xy_avg_mm": math.nan,
        "baseline_base_xy_max_mm": math.nan,
        "base_xy_abs_delta_avg_mm": math.nan,
        "base_xy_abs_delta_max_mm": math.nan,
        "base_path_delta_avg_mm": math.nan,
        "base_path_delta_max_mm": math.nan,
        "anchor_dx_mm": (baseline_meta["anchor_x"] - comp_meta["anchor_x"]) * 1000.0,
        "anchor_dy_mm": (baseline_meta["anchor_y"] - comp_meta["anchor_y"]) * 1000.0,
        "anchor_dz_mm": (baseline_meta["anchor_z"] - comp_meta["anchor_z"]) * 1000.0,
        "anchor_dyaw_rad": baseline_meta["anchor_yaw"] - comp_meta["anchor_yaw"],
    }
    if not common_times:
        return result

    comp_xy = [comp_by_t[t]["base_xy_from_anchor_mm"] for t in common_times]
    base_xy = [base_by_t[t]["base_xy_from_anchor_mm"] for t in common_times]
    xy_abs_delta = [abs(a - b) for a, b in zip(comp_xy, base_xy)]
    path_delta = [
        math.hypot(
            comp_by_t[t]["base_dx_mm"] - base_by_t[t]["base_dx_mm"],
            comp_by_t[t]["base_dy_mm"] - base_by_t[t]["base_dy_mm"],
        )
        for t in common_times
    ]

    result["time_start_s"] = common_times[0]
    result["time_end_s"] = common_times[-1]
    result["comp_base_xy_avg_mm"] = statistics.fmean(comp_xy)
    result["comp_base_xy_max_mm"] = max(comp_xy)
    result["baseline_base_xy_avg_mm"] = statistics.fmean(base_xy)
    result["baseline_base_xy_max_mm"] = max(base_xy)
    result["base_xy_abs_delta_avg_mm"] = statistics.fmean(xy_abs_delta)
    result["base_xy_abs_delta_max_mm"] = max(xy_abs_delta)
    result["base_path_delta_avg_mm"] = statistics.fmean(path_delta)
    result["base_path_delta_max_mm"] = max(path_delta)
    return result


def base_consistency_summary(metrics: dict[str, float]) -> str:
    """输出 base 一致性摘要，给反事实图做审计脚注。"""
    lines = [
        "T4 base motion consistency check (compensating run vs arm-locked baseline)",
        "",
        f"steady_start_s={metrics['steady_start_s']:.1f}",
        (
            f"common_samples={metrics['common_samples']:.0f}, "
            f"time_window=[{metrics['time_start_s']:.3f}, {metrics['time_end_s']:.3f}] s"
        ),
        (
            "anchor_delta="
            f"[dx={metrics['anchor_dx_mm']:.3f} mm, "
            f"dy={metrics['anchor_dy_mm']:.3f} mm, "
            f"dz={metrics['anchor_dz_mm']:.3f} mm, "
            f"dyaw={metrics['anchor_dyaw_rad']:.6f} rad]"
        ),
        (
            f"comp_base_xy: avg={metrics['comp_base_xy_avg_mm']:.3f} mm, "
            f"max={metrics['comp_base_xy_max_mm']:.3f} mm"
        ),
        (
            f"baseline_base_xy: avg={metrics['baseline_base_xy_avg_mm']:.3f} mm, "
            f"max={metrics['baseline_base_xy_max_mm']:.3f} mm"
        ),
        (
            f"base_xy_magnitude_delta: avg={metrics['base_xy_abs_delta_avg_mm']:.3f} mm, "
            f"max={metrics['base_xy_abs_delta_max_mm']:.3f} mm"
        ),
        (
            f"base_xy_path_delta: avg={metrics['base_path_delta_avg_mm']:.3f} mm, "
            f"max={metrics['base_path_delta_max_mm']:.3f} mm"
        ),
    ]
    return "\n".join(lines) + "\n"


def plot_base_motion_overlay(
    comp_base_rows: list[dict[str, float]],
    baseline_base_rows: list[dict[str, float]],
    metrics: dict[str, float],
    figures_dir: Path,
) -> None:
    """辅证图: 叠加两次 run 的 base XY 摆动与 XY 路径。

    这张图用于回答 "D_on/D_off 是否来自同一套 base 运动"。
    """
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "axes.grid": True,
            "grid.color": "lightgrey",
            "grid.alpha": 0.35,
        }
    )
    figures_dir.mkdir(parents=True, exist_ok=True)

    comp_col = "#58849f"
    base_col = "#8a8f99"
    steady = metrics.get("steady_start_s", 10.0)

    comp_post = [r for r in comp_base_rows if r["time_s"] >= steady]
    base_post = [r for r in baseline_base_rows if r["time_s"] >= steady]
    if not comp_post or not base_post:
        return

    fig, axes = plt.subplots(2, 1, figsize=(9, 6.2), dpi=150)
    for ax in axes:
        ax.tick_params(axis="both", which="both", length=0, labelcolor="dimgrey")
        ax.grid(False)
        ax.patch.set_edgecolor("lightgrey")
        ax.patch.set_linewidth(0.8)

    axes[0].plot(
        [r["time_s"] for r in base_post],
        [r["base_xy_from_anchor_mm"] for r in base_post],
        color=base_col,
        linewidth=1.8,
        label="Arm locked baseline",
    )
    axes[0].plot(
        [r["time_s"] for r in comp_post],
        [r["base_xy_from_anchor_mm"] for r in comp_post],
        color=comp_col,
        linewidth=1.8,
        label="Cartesian hand task",
    )
    axes[0].set_title(r"$\bf{(a)}$ Base XY sway from anchor", loc="left", fontsize=11, pad=7, color="dimgrey")
    axes[0].set_ylabel("XY offset (mm)", fontsize=9, labelpad=6, color="dimgrey")
    axes[0].legend(
        frameon=True,
        facecolor="white",
        framealpha=0.8,
        edgecolor="lightgrey",
        labelcolor="dimgrey",
        fontsize=8,
        loc="upper right",
    )

    axes[1].plot(
        [r["base_dx_mm"] for r in base_post],
        [r["base_dy_mm"] for r in base_post],
        color=base_col,
        linewidth=1.5,
        label="Arm locked baseline",
    )
    axes[1].plot(
        [r["base_dx_mm"] for r in comp_post],
        [r["base_dy_mm"] for r in comp_post],
        color=comp_col,
        linewidth=1.5,
        label="Cartesian hand task",
    )
    axes[1].scatter([0.0], [0.0], color="#bd0c0c", s=24, zorder=4, label="Anchor")
    axes[1].set_aspect("equal", adjustable="datalim")
    axes[1].set_title(r"$\bf{(b)}$ Base XY path relative to each anchor", loc="left", fontsize=11, pad=7, color="dimgrey")
    axes[1].set_xlabel("X offset (mm)", fontsize=9, labelpad=6, color="dimgrey")
    axes[1].set_ylabel("Y offset (mm)", fontsize=9, labelpad=6, color="dimgrey")

    axes[1].text(
        0.02,
        0.04,
        (
            f"path delta avg={metrics['base_path_delta_avg_mm']:.1f} mm, "
            f"max={metrics['base_path_delta_max_mm']:.1f} mm"
        ),
        transform=axes[1].transAxes,
        ha="left",
        va="bottom",
        fontsize=8,
        color="dimgrey",
        style="italic",
        bbox={"facecolor": "white", "alpha": 0.78, "edgecolor": "lightgrey", "pad": 4},
    )

    fig.suptitle(
        "T4 base-motion overlay: checking whether the counterfactual runs share the same base motion",
        fontsize=12,
        y=0.99,
        color="dimgrey",
    )
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    for ext in ("pdf", "png"):
        fig.savefig(figures_dir / f"t4_base_motion_overlay.{ext}", dpi=150, bbox_inches="tight")
    plt.close(fig)


def counterfactual_summary(cf: dict[str, float]) -> str:
    lines = [
        "T4 counterfactual compensation (arm compensating vs locked)",
        "",
        f"steady_start_s={cf['steady_start_s']:.1f}",
        f"samples: n_on={cf['n_on']:.0f}, n_off={cf['n_off']:.0f}",
        (
            f"D_on  (compensating): avg={cf['d_on_avg_mm']:.3f} mm, max={cf['d_on_max_mm']:.3f} mm"
        ),
        (
            f"D_off (arm locked)  : avg={cf['d_off_avg_mm']:.3f} mm, max={cf['d_off_max_mm']:.3f} mm"
        ),
        (
            f"compensation: avg {cf['comp_pct_avg']:.1f}% ({cf['comp_ratio_avg']:.2f}x), "
            f"max {cf['comp_pct_max']:.1f}% ({cf['comp_ratio_max']:.2f}x)"
        ),
    ]
    return "\n".join(lines) + "\n"


def load_run(run_dir: Path) -> tuple[list[dict[str, float]], dict[str, float]]:
    """解析一个 run 的 [T4-HOLD] 手漂移日志 (基线 run 无 base_anchor 也可用)。"""
    runtime_log = run_dir / "logs" / "02_walk_wbc_runtime.log"
    return parse_runtime_log(runtime_log)


def main() -> None:
    """Parse logs, save metrics, and render an overview figure."""
    parser = argparse.ArgumentParser(description="Analyze OpenLoong T4/T5 hand-task logs.")
    parser.add_argument("run_dir", type=Path, nargs="?", help="OpenLoong T4 run directory (compensating)")
    parser.add_argument(
        "--baseline",
        type=Path,
        default=None,
        help="Baseline run dir (OPENLOONG_T4_BASELINE=1, arm locked) for counterfactual comparison",
    )
    parser.add_argument(
        "--steady-start-s",
        type=float,
        default=10.0,
        help="Steady-state window start (s); samples before this are startup transient",
    )
    parser.add_argument(
        "--t5-posture-log",
        type=Path,
        default=None,
        help="Analyze one T5 stand-posture smoke log instead of a T4 run directory",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output directory for --t5-posture-log analysis",
    )
    args = parser.parse_args()

    if args.t5_posture_log is not None:
        output_dir = (
            args.output_dir
            if args.output_dir is not None
            else args.t5_posture_log.resolve().parent / "analysis_t5_posture"
        )
        summary = analyze_t5_posture_log(args.t5_posture_log.resolve(), output_dir.resolve())
        print(summary, end="")
        print(f"analysis_dir={output_dir.resolve()}")
        return

    if args.run_dir is None:
        parser.error("run_dir is required unless --t5-posture-log is provided")

    run_dir = args.run_dir.resolve()
    runtime_log = run_dir / "logs" / "02_walk_wbc_runtime.log"
    datalog = run_dir / "runtime_record" / "datalog.log"
    analysis_dir = run_dir / "analysis"
    figures_dir = analysis_dir / "figures"

    hold_rows, meta = parse_runtime_log(runtime_log)
    base_rows = parse_base_pose(datalog, meta)

    write_csv(analysis_dir / "t4_hold_samples.csv", hold_rows)
    write_csv(analysis_dir / "base_pose_samples.csv", base_rows)

    summary = summarize(hold_rows, base_rows, meta)
    analysis_dir.mkdir(parents=True, exist_ok=True)
    (analysis_dir / "t4_stage_d_summary.txt").write_text(summary, encoding="utf-8")

    comp = compute_compensation(hold_rows, base_rows, meta)
    try:
        plot_overview(hold_rows, base_rows, meta, figures_dir)
        plot_compensation(hold_rows, base_rows, meta, comp, figures_dir)
    except Exception as exc:
        (analysis_dir / "plot_error.txt").write_text(str(exc), encoding="utf-8")

    print(summary, end="")

    # Counterfactual: compare against a baseline run (arm locked, no compensation).
    # This is the honest Stage-D figure — same stepping, only the arm task differs.
    if args.baseline is not None:
        base_run = args.baseline.resolve()
        try:
            base_hold, base_meta = load_run(base_run)
            baseline_base_rows = parse_base_pose(
                base_run / "runtime_record" / "datalog.log", base_meta
            )
        except Exception as exc:
            (analysis_dir / "counterfactual_error.txt").write_text(
                f"failed to load baseline {base_run}: {exc}", encoding="utf-8"
            )
        else:
            cf = compute_counterfactual(
                hold_rows, meta, base_hold, base_meta, args.steady_start_s
            )
            cf_summary = counterfactual_summary(cf)
            (analysis_dir / "t4_counterfactual_summary.txt").write_text(
                cf_summary, encoding="utf-8"
            )
            base_metrics = compute_base_consistency(
                base_rows, meta, baseline_base_rows, base_meta, args.steady_start_s
            )
            base_summary = base_consistency_summary(base_metrics)
            (analysis_dir / "t4_base_consistency_summary.txt").write_text(
                base_summary, encoding="utf-8"
            )
            try:
                plot_counterfactual(
                    hold_rows, meta, base_hold, base_meta, cf, figures_dir
                )
                plot_base_motion_overlay(
                    base_rows, baseline_base_rows, base_metrics, figures_dir
                )
            except Exception as exc:
                (analysis_dir / "counterfactual_plot_error.txt").write_text(
                    str(exc), encoding="utf-8"
                )
            print()
            print(cf_summary, end="")
            print()
            print(base_summary, end="")

    print(f"analysis_dir={analysis_dir}")


if __name__ == "__main__":
    main()
