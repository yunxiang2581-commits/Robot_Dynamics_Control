"""B01 单关节 MPC Phase 4 精细调参。

基于 Phase 1-3 最优结果，围绕降低力矩饱和做局部搜索。
关键修正：torque_limit 降至 1.8-1.9，用硬约束替代软约束。

候选组：
  A: q=5.0, dq=0.10, tau_w=0.002, tau_lim=1.9  — 降低饱和，轻微减振
  B: q=8.0, dq=0.10, tau_w=0.002, tau_lim=1.9  — 保持收敛速度
  C: q=10.0, dq=0.10, tau_w=0.003, tau_lim=1.9 — 更快收敛但抑制力矩
  D: q=5.0, dq=0.20, tau_w=0.002, tau_lim=1.9  — 更强速度阻尼
  E: q=8.0, dq=0.20, tau_w=0.003, tau_lim=1.8  — 保守力矩版本
  F: q=10.0, dq=0.05, tau_w=0.005, tau_lim=1.9 — 强惩罚 torque，测试收敛
"""

from __future__ import annotations

import csv
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

# ── 路径设置 ──────────────────────────────────────────────────────────────
SCRIPT_PATH = Path(__file__).resolve()
SIMULATOR_ROOT = SCRIPT_PATH.parents[1]
PROJECT_ROOT = SIMULATOR_ROOT.parent
REPO_ROOT = PROJECT_ROOT.parents[1]

for _p in (str(REPO_ROOT), str(SIMULATOR_ROOT), str(SCRIPT_PATH.parent)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from run_B01_single_joint_mpc_demo import (  # noqa: E402
    build_arg_parser,
    resolve_run_args,
    run_closed_loop,
)
from run_B01_tuning import (  # noqa: E402
    compute_score,
    check_stop_criteria,
    run_single_experiment,
    generate_comparison_plots,
    generate_metrics_table,
    generate_tuning_summary,
    FIXED_PARAMS,
)

import numpy as np  # noqa: E402
import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# ── Phase 4 候选组 ────────────────────────────────────────────────────────
PHASE4_CANDIDATES: list[dict[str, Any]] = [
    # 组 A: 降低饱和，轻微减振
    {"label": "A", "params": {
        "horizon": 10, "num_candidates": 128, "torque_limit": 1.9,
        "q_weight": 5.0, "dq_weight": 0.10, "torque_weight": 0.002, "terminal_weight": 1.0,
    }},
    # 组 B: 保持收敛速度
    {"label": "B", "params": {
        "horizon": 10, "num_candidates": 128, "torque_limit": 1.9,
        "q_weight": 8.0, "dq_weight": 0.10, "torque_weight": 0.002, "terminal_weight": 1.0,
    }},
    # 组 C: 更快收敛但抑制力矩
    {"label": "C", "params": {
        "horizon": 10, "num_candidates": 128, "torque_limit": 1.9,
        "q_weight": 10.0, "dq_weight": 0.10, "torque_weight": 0.003, "terminal_weight": 1.0,
    }},
    # 组 D: 更强速度阻尼
    {"label": "D", "params": {
        "horizon": 10, "num_candidates": 128, "torque_limit": 1.9,
        "q_weight": 5.0, "dq_weight": 0.20, "torque_weight": 0.002, "terminal_weight": 1.0,
    }},
    # 组 E: 保守力矩版本
    {"label": "E", "params": {
        "horizon": 10, "num_candidates": 128, "torque_limit": 1.8,
        "q_weight": 8.0, "dq_weight": 0.20, "torque_weight": 0.003, "terminal_weight": 1.0,
    }},
    # 组 F: 强惩罚 torque，测试是否仍能收敛
    {"label": "F", "params": {
        "horizon": 10, "num_candidates": 128, "torque_limit": 1.9,
        "q_weight": 10.0, "dq_weight": 0.05, "torque_weight": 0.005, "terminal_weight": 1.0,
    }},
]

# Phase 3 最优组（基线对比）
PHASE3_BEST = {
    "label": "P3_best",
    "params": {
        "horizon": 10, "num_candidates": 128, "torque_limit": 2.0,
        "q_weight": 5.0, "dq_weight": 0.05, "torque_weight": 0.001, "terminal_weight": 1.0,
    },
}


# ── 重点对比图（3 张） ────────────────────────────────────────────────────

def generate_phase4_focus_plots(
    all_results: list[dict[str, Any]],
    figures_dir: Path,
    target_angle: float,
) -> None:
    """生成 3 张重点对比图。"""
    figures_dir.mkdir(parents=True, exist_ok=True)

    labels_map = {r["run_id"]: r.get("label", r["run_id"][-6:]) for r in all_results}

    # 1. 角度误差对比 — 重点关注早期误差是否压下去
    fig, ax = plt.subplots(figsize=(10, 6))
    for r in all_results:
        th = r["result"]["time_history"]
        qh = r["result"]["q_history"]
        targets = r["result"].get("target_history", [target_angle] * len(qh))
        err = [abs(q - target) for q, target in zip(qh, targets)]
        label = labels_map[r["run_id"]]
        ax.plot(th, err, label=label, alpha=0.85, linewidth=1.5)
    ax.axhline(0.05, linestyle="--", color="tab:green", linewidth=1.5, label="0.05 rad threshold")
    ax.axhline(0.10, linestyle="--", color="tab:orange", linewidth=1.0, alpha=0.7, label="0.10 rad threshold")
    ax.set_xlabel("time [s]", fontsize=11)
    ax.set_ylabel("|q - q_target| [rad]", fontsize=11)
    ax.set_title("Phase 4 — Angle Error Compare (early transient focus)", fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9, ncol=2)
    fig.tight_layout()
    fig.savefig(figures_dir / "B01_phase4_angle_error_compare.png", dpi=150)
    plt.close(fig)

    # 2. 力矩对比 — 重点看是否还贴着 ±2.0 Nm
    fig, ax = plt.subplots(figsize=(10, 6))
    for r in all_results:
        th = r["result"]["time_history"]
        tauh = r["result"]["torque_history"]
        label = labels_map[r["run_id"]]
        ax.plot(th, tauh, label=label, alpha=0.85, linewidth=1.2)
    ax.axhline(2.0, linestyle="--", color="tab:red", linewidth=1.5, label="+2.0 Nm (XML limit)")
    ax.axhline(-2.0, linestyle="--", color="tab:red", linewidth=1.5, label="-2.0 Nm (XML limit)")
    ax.axhline(1.9, linestyle=":", color="tab:orange", linewidth=1.0, alpha=0.7, label="+1.9 Nm")
    ax.axhline(-1.9, linestyle=":", color="tab:orange", linewidth=1.0, alpha=0.7, label="-1.9 Nm")
    ax.set_xlabel("time [s]", fontsize=11)
    ax.set_ylabel("tau [Nm]", fontsize=11)
    ax.set_title("Phase 4 — Torque Compare (saturation focus)", fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(figures_dir / "B01_phase4_torque_compare.png", dpi=150)
    plt.close(fig)

    # 3. Score 排名 — 确认不是单纯牺牲力矩换来误差
    fig, ax = plt.subplots(figsize=(8, 5))
    scored = [(compute_score(r["result"]["extended_metrics"]), r) for r in all_results]
    scored.sort(key=lambda x: x[0])
    score_labels = [labels_map[r["run_id"]] for _, r in scored]
    score_values = [s for s, _ in scored]
    colors = ["tab:green" if i == 0 else "tab:blue" for i in range(len(scored))]
    ax.barh(range(len(scored)), score_values, color=colors, alpha=0.8)
    ax.set_yticks(range(len(scored)))
    ax.set_yticklabels(score_labels, fontsize=10)
    ax.set_xlabel("score (lower is better)", fontsize=11)
    ax.set_title("Phase 4 — Score Ranking", fontsize=12)
    ax.grid(True, alpha=0.3, axis="x")
    ax.invert_yaxis()
    fig.tight_layout()
    fig.savefig(figures_dir / "B01_phase4_score_compare.png", dpi=150)
    plt.close(fig)


def generate_phase4_detailed_plots(
    all_results: list[dict[str, Any]],
    figures_dir: Path,
    target_angle: float,
) -> None:
    """生成完整对比图（角度跟踪 + 力矩 + cost + runtime）。"""
    figures_dir.mkdir(parents=True, exist_ok=True)

    labels_map = {r["run_id"]: r.get("label", r["run_id"][-6:]) for r in all_results}

    # 角度跟踪
    fig, ax = plt.subplots(figsize=(10, 6))
    for r in all_results:
        th = r["result"]["time_history"]
        qh = r["result"]["q_history"]
        ax.plot(th, qh, label=labels_map[r["run_id"]], alpha=0.85, linewidth=1.5)
    if all_results and "target_history" in all_results[0]["result"]:
        ax.plot(
            all_results[0]["result"]["time_history"],
            all_results[0]["result"]["target_history"],
            linestyle="--",
            color="tab:red",
            linewidth=2,
            label="target",
        )
    else:
        ax.axhline(target_angle, linestyle="--", color="tab:red", linewidth=2, label="target")
    ax.set_xlabel("time [s]"); ax.set_ylabel("q [rad]")
    ax.set_title("Phase 4 — Angle Tracking Compare")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=9, ncol=2)
    fig.tight_layout()
    fig.savefig(figures_dir / "B01_phase4_angle_tracking_compare.png", dpi=150)
    plt.close(fig)

    # best_cost
    fig, ax = plt.subplots(figsize=(10, 6))
    for r in all_results:
        th = r["result"]["time_history"]
        ch = r["result"]["best_cost_history"]
        ax.plot(th, ch, label=labels_map[r["run_id"]], alpha=0.85, linewidth=1.2)
    ax.set_xlabel("time [s]"); ax.set_ylabel("best cost")
    ax.set_title("Phase 4 — Best Cost Compare")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=9, ncol=2)
    fig.tight_layout()
    fig.savefig(figures_dir / "B01_phase4_best_cost_compare.png", dpi=150)
    plt.close(fig)

    # Runtime 柱状图
    fig, ax = plt.subplots(figsize=(8, 5))
    run_labels = [labels_map[r["run_id"]] for r in all_results]
    mean_rts = [r["result"]["extended_metrics"]["runtime_per_control_step_mean"] for r in all_results]
    max_rts = [r["result"]["extended_metrics"]["runtime_per_control_step_max"] for r in all_results]
    x = np.arange(len(run_labels))
    width = 0.35
    ax.bar(x - width / 2, mean_rts, width, label="mean runtime", alpha=0.8)
    ax.bar(x + width / 2, max_rts, width, label="max runtime", alpha=0.8)
    ax.axhline(0.05, linestyle="--", color="tab:red", linewidth=1.5, label="0.05s limit")
    ax.set_ylabel("runtime per step [s]")
    ax.set_title("Phase 4 — Runtime Compare")
    ax.set_xticks(x); ax.set_xticklabels(run_labels, fontsize=9)
    ax.legend(); ax.grid(True, alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(figures_dir / "B01_phase4_runtime_compare.png", dpi=150)
    plt.close(fig)


def generate_phase4_metrics_table(all_results: list[dict[str, Any]], output_path: Path) -> None:
    """生成 Phase 4 指标汇总 CSV。"""
    output_path.parent.mkdir(parents=True, exist_ok=True)

    headers = ["label", "run_id", "score",
               "horizon", "num_candidates", "torque_limit",
               "q_weight", "dq_weight", "torque_weight", "terminal_weight",
               "final_abs_error", "mean_abs_error", "max_abs_error",
               "settling_time", "max_abs_torque", "mean_abs_torque",
               "torque_saturation_ratio", "mean_best_cost", "final_best_cost",
               "runtime_per_control_step_mean", "runtime_per_control_step_max"]

    rows = []
    for r in all_results:
        ext = r["result"]["extended_metrics"]
        score = compute_score(ext)
        p = r["params"]
        row = [r.get("label", ""), r["run_id"], f"{score:.4f}"]
        for k in ["horizon", "num_candidates", "torque_limit",
                   "q_weight", "dq_weight", "torque_weight", "terminal_weight"]:
            row.append(str(p[k]))
        for k in ["final_abs_error", "mean_abs_error", "max_abs_error",
                   "settling_time", "max_abs_torque", "mean_abs_torque",
                   "torque_saturation_ratio", "mean_best_cost", "final_best_cost",
                   "runtime_per_control_step_mean", "runtime_per_control_step_max"]:
            row.append(f"{ext[k]:.6f}")
        rows.append(row)

    with output_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        writer.writerows(rows)


def generate_phase4_summary(
    all_results: list[dict[str, Any]],
    output_path: Path,
) -> str:
    """生成 Phase 4 summary。"""
    output_path.parent.mkdir(parents=True, exist_ok=True)

    scored = [(compute_score(r["result"]["extended_metrics"]), r) for r in all_results]
    scored.sort(key=lambda x: x[0])
    best_score, best_r = scored[0]
    best_ext = best_r["result"]["extended_metrics"]
    best_params = best_r["params"]
    best_label = best_r.get("label", "")

    total_time = FIXED_PARAMS["num_steps"] * FIXED_PARAMS["dt"]
    criteria = check_stop_criteria(best_ext, best_params["torque_limit"], total_time)

    lines = [
        "=" * 70,
        "B01 Phase 4 精细调参总结",
        "=" * 70,
        "",
        f"候选组数: {len(all_results)}",
        f"总仿真时间: {total_time:.2f} s",
        "",
        "─" * 70,
        "全部候选排名:",
        "─" * 70,
    ]
    for i, (score, r) in enumerate(scored):
        ext = r["result"]["extended_metrics"]
        label = r.get("label", "")
        lines.append(
            f"  #{i+1} [{label}] score={score:.4f}  "
            f"final_err={ext['final_abs_error']:.4f}  "
            f"mean_err={ext['mean_abs_error']:.4f}  "
            f"sat={ext['torque_saturation_ratio']:.3f}  "
            f"max_tau={ext['max_abs_torque']:.3f}  "
            f"runtime={ext['runtime_per_control_step_mean']:.4f}s"
        )

    lines.extend([
        "",
        "─" * 70,
        f"最优组: [{best_label}]",
        "─" * 70,
    ])
    for k, v in best_params.items():
        lines.append(f"  {k}: {v}")

    lines.extend([
        "",
        f"  综合评分 (score): {best_score:.4f}",
        f"  final_abs_error: {best_ext['final_abs_error']:.4f} rad",
        f"  mean_abs_error: {best_ext['mean_abs_error']:.4f} rad",
        f"  max_abs_error: {best_ext['max_abs_error']:.4f} rad",
        f"  settling_time: {best_ext['settling_time']:.2f} s ({best_ext['settling_time']/total_time*100:.1f}%)",
        f"  max_abs_torque: {best_ext['max_abs_torque']:.4f} Nm",
        f"  mean_abs_torque: {best_ext['mean_abs_torque']:.4f} Nm",
        f"  torque_saturation_ratio: {best_ext['torque_saturation_ratio']:.4f} ({best_ext['torque_saturation_ratio']*100:.1f}%)",
        f"  mean_best_cost: {best_ext['mean_best_cost']:.4f}",
        f"  final_best_cost: {best_ext['final_best_cost']:.4f}",
        f"  runtime_per_control_step_mean: {best_ext['runtime_per_control_step_mean']:.4f} s",
        "",
        "─" * 70,
        "停止指标检查:",
        "─" * 70,
    ])
    for name, met in criteria.items():
        status = "PASS" if met else "FAIL"
        lines.append(f"  [{status}] {name}")

    all_met = all(criteria.values())
    lines.extend(["", f"是否全部达标: {'是' if all_met else '否'}"])

    # 瓶颈分析
    lines.extend(["", "─" * 70, "瓶颈分析:", "─" * 70])
    if not criteria.get("final_abs_error", True):
        lines.append("  - final_abs_error 未达标：q_weight 不够或 horizon 太短")
    if not criteria.get("mean_abs_error", True):
        lines.append("  - mean_abs_error 未达标：早期瞬态误差大，需增大 q_weight 或 dq_weight")
    if not criteria.get("settling_time", True):
        lines.append("  - settling_time 未达标：收敛太慢，需增大 q_weight/terminal_weight")
    if not criteria.get("torque_saturation_ratio", True):
        lines.append("  - torque_saturation_ratio 未达标：torque_weight 仍不够或 torque_limit 仍偏高")
    if not criteria.get("max_abs_torque", True):
        lines.append("  - max_abs_torque 未达标：torque_limit 需进一步降低或 torque_weight 需增大")
    if not criteria.get("runtime", True):
        lines.append("  - runtime 未达标：减少 num_candidates 或 horizon")

    lines.extend(["", "=" * 70])
    summary = "\n".join(lines)
    output_path.write_text(summary, encoding="utf-8")
    return summary


# ── 主流程 ────────────────────────────────────────────────────────────────

def main() -> None:
    tuning_run_id = f"phase4_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    output_root = PROJECT_ROOT / "outputs" / "runs"
    tuning_dir = output_root / "B01_single_joint_mpc_demo" / tuning_run_id
    figures_dir = tuning_dir / "figures"
    metrics_dir = tuning_dir / "metrics"
    logs_dir = tuning_dir / "logs"

    for d in (figures_dir, metrics_dir, logs_dir):
        d.mkdir(parents=True, exist_ok=True)

    config_path = PROJECT_ROOT / "configs" / "B01_single_joint_mpc.yaml"

    print("=" * 70)
    print(f"B01 Phase 4 精细调参 — tuning_run_id: {tuning_run_id}")
    print("=" * 70)

    # 先跑基线（Phase 3 最优）
    all_results: list[dict[str, Any]] = []

    print("\n[Baseline] Phase 3 最最优组 ...")
    baseline = PHASE3_BEST.copy()
    run_id = f"{tuning_run_id}_baseline"
    t0 = time.time()
    result = run_single_experiment(baseline["params"], config_path, run_id, output_root)
    elapsed = time.time() - t0
    result["label"] = baseline["label"]
    ext = result["result"]["extended_metrics"]
    score = compute_score(ext)
    print(f"  [{baseline['label']}] score={score:.4f}  final_err={ext['final_abs_error']:.4f}  "
          f"mean_err={ext['mean_abs_error']:.4f}  sat={ext['torque_saturation_ratio']:.3f}  "
          f"max_tau={ext['max_abs_torque']:.3f}  elapsed={elapsed:.1f}s")
    all_results.append(result)

    # 跑 Phase 4 候选
    print(f"\n[Phase 4] 运行 {len(PHASE4_CANDIDATES)} 组候选 ...")
    for i, cand in enumerate(PHASE4_CANDIDATES):
        label = cand["label"]
        params = cand["params"]
        run_id = f"{tuning_run_id}_{label}"
        print(f"  [{label}] {params}")
        t0 = time.time()
        try:
            result = run_single_experiment(params, config_path, run_id, output_root)
            elapsed = time.time() - t0
            result["label"] = label
            ext = result["result"]["extended_metrics"]
            score = compute_score(ext)
            print(f"    score={score:.4f}  final_err={ext['final_abs_error']:.4f}  "
                  f"mean_err={ext['mean_abs_error']:.4f}  sat={ext['torque_saturation_ratio']:.3f}  "
                  f"max_tau={ext['max_abs_torque']:.3f}  runtime={ext['runtime_per_control_step_mean']:.4f}s  "
                  f"elapsed={elapsed:.1f}s")
            all_results.append(result)
        except Exception as e:
            print(f"    ERROR: {e}")

    # 最终排名
    print("\n" + "=" * 70)
    print("Phase 4 最终排名 (score 越低越好):")
    print("=" * 70)
    final_scored = [(compute_score(r["result"]["extended_metrics"]), r) for r in all_results]
    final_scored.sort(key=lambda x: x[0])
    for i, (score, r) in enumerate(final_scored):
        ext = r["result"]["extended_metrics"]
        label = r.get("label", "")
        print(f"  #{i+1} [{label:7s}] score={score:8.4f}  final_err={ext['final_abs_error']:.4f}  "
              f"mean_err={ext['mean_abs_error']:.4f}  sat={ext['torque_saturation_ratio']:.3f}  "
              f"max_tau={ext['max_abs_torque']:.3f}")

    # 生成输出
    print("\n生成重点对比图 (3 张) ...")
    generate_phase4_focus_plots(all_results, figures_dir, FIXED_PARAMS["target_angle"])

    print("生成详细对比图 (4 张) ...")
    generate_phase4_detailed_plots(all_results, figures_dir, FIXED_PARAMS["target_angle"])

    print("生成指标汇总 CSV ...")
    generate_phase4_metrics_table(all_results, metrics_dir / "B01_phase4_metrics_table.csv")

    print("生成 Phase 4 summary ...")
    summary = generate_phase4_summary(all_results, logs_dir / "B01_phase4_summary.txt")

    print("\n" + summary)
    print(f"\n所有输出保存到: {tuning_dir}")


if __name__ == "__main__":
    main()
