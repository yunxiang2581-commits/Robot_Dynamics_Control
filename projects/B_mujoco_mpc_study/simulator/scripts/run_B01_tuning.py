"""B01 单关节 MPC 自动调参脚本。

两阶段调参策略：
  第一阶段：粗搜索 — 用拉丁超立方采样 ~25 组参数，快速筛选最优 3-5 组。
  第二阶段：局部细调 — 围绕最优参数邻域微调 ~12 组，收敛到最终解。

输出目录：
  outputs/runs/B01_single_joint_mpc_demo/<tuning_run_id>/
    figures/  — 7 张对比图
    metrics/  — 汇总 CSV
    logs/     — tuning summary
"""

from __future__ import annotations

import csv
import itertools
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

# 延迟导入 demo 函数（避免模块级 MuJoCo 初始化）
from run_B01_single_joint_mpc_demo import (  # noqa: E402
    build_run_outputs,
    compute_extended_metrics,
    ensure_output_dirs,
    resolve_run_args,
    run_closed_loop,
    build_arg_parser,
)

import numpy as np  # noqa: E402

# ── 搜索空间 ──────────────────────────────────────────────────────────────
SEARCH_SPACE: dict[str, list[float]] = {
    "horizon":         [10, 20, 30, 40],
    "num_candidates":  [64, 128, 256, 512],
    "torque_limit":    [2.0, 3.0, 4.0],
    "q_weight":        [1.0, 5.0, 10.0, 20.0],
    "dq_weight":       [0.05, 0.1, 0.5, 1.0],
    "torque_weight":   [0.0005, 0.001, 0.005, 0.01],
    "terminal_weight": [1.0, 5.0, 10.0, 20.0],
}

PARAM_NAMES = list(SEARCH_SPACE.keys())

# 固定参数（不参与调参）
FIXED_PARAMS: dict[str, Any] = {
    "target_angle": 1.0,
    "initial_q": 0.0,
    "initial_dq": 0.0,
    "dt": 0.01,
    "num_steps": 300,
    "export_video": False,
    "save_figures": False,
    "save_metrics": False,
}

# 停止指标阈值
STOP_CRITERIA = {
    "final_abs_error": 0.05,
    "mean_abs_error": 0.10,
    "settling_time_ratio": 0.60,
    "torque_saturation_ratio": 0.10,
    "max_abs_torque_ratio": 0.95,
    "runtime_per_control_step_mean": 0.05,
}

# 综合评分权重
SCORE_WEIGHTS = {
    "final_abs_error": 10.0,
    "mean_abs_error": 5.0,
    "max_abs_error": 0.5,
    "mean_abs_torque": 0.05,
    "torque_saturation_ratio": 2.0,
    "runtime_per_control_step_mean": 0.2,
}


# ── 评分与停止条件 ─────────────────────────────────────────────────────────

def compute_score(ext: dict[str, float]) -> float:
    """综合评分，越低越好。"""
    return sum(
        SCORE_WEIGHTS[k] * ext[k]
        for k in SCORE_WEIGHTS
        if k in ext
    )


def check_stop_criteria(ext: dict[str, float], torque_limit: float, total_time: float) -> dict[str, bool]:
    """逐项检查停止指标，返回每项是否达标。"""
    effective_limit = min(torque_limit, 2.0)  # XML ctrlrange=[-2,2]
    return {
        "final_abs_error": ext["final_abs_error"] <= STOP_CRITERIA["final_abs_error"],
        "mean_abs_error": ext["mean_abs_error"] <= STOP_CRITERIA["mean_abs_error"],
        "settling_time": ext["settling_time"] <= STOP_CRITERIA["settling_time_ratio"] * total_time,
        "torque_saturation_ratio": ext["torque_saturation_ratio"] <= STOP_CRITERIA["torque_saturation_ratio"],
        "max_abs_torque": ext["max_abs_torque"] <= STOP_CRITERIA["max_abs_torque_ratio"] * effective_limit,
        "runtime": ext["runtime_per_control_step_mean"] <= STOP_CRITERIA["runtime_per_control_step_mean"],
    }


def all_criteria_met(criteria: dict[str, bool]) -> bool:
    return all(criteria.values())


# ── 参数采样 ──────────────────────────────────────────────────────────────

def latin_hypercube_sample(space: dict[str, list], n_samples: int, seed: int = 42) -> list[dict[str, float]]:
    """拉丁超立方采样，保证每个参数的每个水平至少出现一次。"""
    rng = np.random.RandomState(seed)
    keys = list(space.keys())
    n_dims = len(keys)
    samples: list[dict[str, float]] = []

    # 生成拉丁超立方样本
    for i in range(n_samples):
        sample: dict[str, float] = {}
        for j, key in enumerate(keys):
            values = space[key]
            # 分层采样：将 [0,1] 分为 n_samples 层，每层随机选一个点
            low = i / n_samples
            high = (i + 1) / n_samples
            # 加入维度偏移以避免对角线
            u = rng.uniform(low, high)
            idx = min(int(u * len(values)), len(values) - 1)
            sample[key] = values[idx]
        samples.append(sample)

    # 打乱各维度的配对
    rng.shuffle(samples)
    return samples


def generate_phase2_candidates(
    best_params: dict[str, float],
    space: dict[str, list],
    n_candidates: int = 12,
    seed: int = 123,
) -> list[dict[str, float]]:
    """围绕最优参数生成局部细调候选。"""
    rng = np.random.RandomState(seed)
    candidates: list[dict[str, float]] = []

    for _ in range(n_candidates):
        sample: dict[str, float] = {}
        for key in PARAM_NAMES:
            values = sorted(space[key])
            best_val = best_params[key]
            best_idx = values.index(best_val) if best_val in values else 0

            # 在最优值附近 ±1 个水平内随机选
            offsets = [-1, 0, 1]
            offset = rng.choice(offsets)
            new_idx = max(0, min(len(values) - 1, best_idx + offset))
            sample[key] = values[new_idx]
        candidates.append(sample)

    return candidates


# ── 单次运行 ──────────────────────────────────────────────────────────────

def run_single_experiment(
    params: dict[str, float],
    config_path: Path,
    run_id: str,
    output_root: Path,
) -> dict[str, Any]:
    """用给定参数运行一次 B01 MPC 闭环，返回完整结果。"""
    # 构造命令行参数覆盖
    argv = [
        "--config", str(config_path),
        "--run-id", run_id,
        "--output-root", str(output_root),
    ]
    for key, value in params.items():
        argv.append(f"--{key.replace('_', '-')}")
        argv.append(str(value))

    # 加入固定参数
    for key, value in FIXED_PARAMS.items():
        if key == "export_video" or key == "save_figures" or key == "save_metrics":
            if not value:
                argv.append(f"--no-{key.replace('_', '-')}")
        else:
            argv.append(f"--{key.replace('_', '-')}")
            argv.append(str(value))

    # 解析参数并运行
    parser = build_arg_parser()
    args = resolve_run_args(parser.parse_args(argv))

    result = run_closed_loop(args)

    return {
        "run_id": run_id,
        "params": params,
        "result": result,
    }


# ── 图表生成 ──────────────────────────────────────────────────────────────

def generate_comparison_plots(
    all_results: list[dict[str, Any]],
    figures_dir: Path,
    target_angle: float,
) -> None:
    """生成 6 张对比图。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figures_dir.mkdir(parents=True, exist_ok=True)

    # 按 score 排序
    scored = []
    for r in all_results:
        ext = r["result"]["extended_metrics"]
        scored.append((compute_score(ext), r))
    scored.sort(key=lambda x: x[0])

    # 取前 10 组绘图（太多会看不清）
    top_results = [r for _, r in scored[:10]]
    labels = [f"run_{r['run_id'][-6:]}" for r in top_results]

    # 1. 角度跟踪对比
    fig, ax = plt.subplots(figsize=(10, 6))
    for r in top_results:
        th = r["result"]["time_history"]
        qh = r["result"]["q_history"]
        ax.plot(th, qh, label=f"run_{r['run_id'][-6:]}", alpha=0.8)
    if top_results and "target_history" in top_results[0]["result"]:
        ax.plot(
            top_results[0]["result"]["time_history"],
            top_results[0]["result"]["target_history"],
            linestyle="--",
            color="tab:red",
            linewidth=2,
            label="target",
        )
    else:
        ax.axhline(target_angle, linestyle="--", color="tab:red", linewidth=2, label="target")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("q [rad]")
    ax.set_title("B01 Tuning — Angle Tracking Compare")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(figures_dir / "B01_tuning_angle_tracking_compare.png", dpi=150)
    plt.close(fig)

    # 2. 角度误差对比
    fig, ax = plt.subplots(figsize=(10, 6))
    for r in top_results:
        th = r["result"]["time_history"]
        qh = r["result"]["q_history"]
        targets = r["result"].get("target_history", [target_angle] * len(qh))
        err = [abs(q - target) for q, target in zip(qh, targets)]
        ax.plot(th, err, label=f"run_{r['run_id'][-6:]}", alpha=0.8)
    ax.axhline(0.05, linestyle="--", color="tab:green", linewidth=1.5, label="0.05 rad threshold")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("|q - q_target| [rad]")
    ax.set_title("B01 Tuning — Angle Error Compare")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(figures_dir / "B01_tuning_angle_error_compare.png", dpi=150)
    plt.close(fig)

    # 3. 力矩对比
    fig, ax = plt.subplots(figsize=(10, 6))
    for r in top_results:
        th = r["result"]["time_history"]
        tauh = r["result"]["torque_history"]
        ax.plot(th, tauh, label=f"run_{r['run_id'][-6:]}", alpha=0.8)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("tau [Nm]")
    ax.set_title("B01 Tuning — Torque Compare")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(figures_dir / "B01_tuning_torque_compare.png", dpi=150)
    plt.close(fig)

    # 4. best_cost 对比
    fig, ax = plt.subplots(figsize=(10, 6))
    for r in top_results:
        th = r["result"]["time_history"]
        ch = r["result"]["best_cost_history"]
        ax.plot(th, ch, label=f"run_{r['run_id'][-6:]}", alpha=0.8)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("best cost")
    ax.set_title("B01 Tuning — Best Cost Compare")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(figures_dir / "B01_tuning_best_cost_compare.png", dpi=150)
    plt.close(fig)

    # 5. Runtime 柱状图
    fig, ax = plt.subplots(figsize=(10, 6))
    run_labels = [f"run_{r['run_id'][-6:]}" for r in all_results]
    mean_rts = [r["result"]["extended_metrics"]["runtime_per_control_step_mean"] for r in all_results]
    max_rts = [r["result"]["extended_metrics"]["runtime_per_control_step_max"] for r in all_results]
    x = np.arange(len(run_labels))
    width = 0.35
    ax.bar(x - width / 2, mean_rts, width, label="mean runtime", alpha=0.8)
    ax.bar(x + width / 2, max_rts, width, label="max runtime", alpha=0.8)
    ax.axhline(0.05, linestyle="--", color="tab:red", linewidth=1.5, label="0.05s limit")
    ax.set_xlabel("run")
    ax.set_ylabel("runtime per step [s]")
    ax.set_title("B01 Tuning — Runtime Compare")
    ax.set_xticks(x)
    ax.set_xticklabels(run_labels, rotation=45, ha="right", fontsize=7)
    ax.legend()
    ax.grid(True, alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(figures_dir / "B01_tuning_runtime_compare.png", dpi=150)
    plt.close(fig)

    # 6. Score 排名柱状图
    fig, ax = plt.subplots(figsize=(10, 6))
    scores = [(compute_score(r["result"]["extended_metrics"]), r) for r in all_results]
    scores.sort(key=lambda x: x[0])
    score_labels = [f"run_{r['run_id'][-6:]}" for _, r in scores]
    score_values = [s for s, _ in scores]
    colors = ["tab:green" if i < 3 else "tab:blue" for i in range(len(scores))]
    ax.barh(range(len(scores)), score_values, color=colors, alpha=0.8)
    ax.set_yticks(range(len(scores)))
    ax.set_yticklabels(score_labels, fontsize=7)
    ax.set_xlabel("score (lower is better)")
    ax.set_title("B01 Tuning — Score Ranking")
    ax.grid(True, alpha=0.3, axis="x")
    ax.invert_yaxis()
    fig.tight_layout()
    fig.savefig(figures_dir / "B01_tuning_score_compare.png", dpi=150)
    plt.close(fig)


def generate_metrics_table(all_results: list[dict[str, Any]], output_path: Path) -> None:
    """生成指标汇总 CSV。"""
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # 表头
    headers = ["run_id", "score"]
    headers.extend(PARAM_NAMES)
    headers.extend([
        "final_abs_error", "mean_abs_error", "max_abs_error",
        "settling_time", "max_abs_torque", "mean_abs_torque",
        "torque_saturation_ratio", "mean_best_cost", "final_best_cost",
        "runtime_per_control_step_mean", "runtime_per_control_step_max",
    ])

    rows = []
    for r in all_results:
        ext = r["result"]["extended_metrics"]
        score = compute_score(ext)
        row = [r["run_id"], f"{score:.4f}"]
        for p in PARAM_NAMES:
            row.append(str(r["params"][p]))
        for k in [
            "final_abs_error", "mean_abs_error", "max_abs_error",
            "settling_time", "max_abs_torque", "mean_abs_torque",
            "torque_saturation_ratio", "mean_best_cost", "final_best_cost",
            "runtime_per_control_step_mean", "runtime_per_control_step_max",
        ]:
            row.append(f"{ext[k]:.6f}")
        rows.append(row)

    with output_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        writer.writerows(rows)


# ── Tuning Summary ────────────────────────────────────────────────────────

def generate_tuning_summary(
    all_results: list[dict[str, Any]],
    criteria_best: dict[str, bool],
    output_path: Path,
) -> str:
    """生成 tuning summary 文本。"""
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # 按 score 排序
    scored = [(compute_score(r["result"]["extended_metrics"]), r) for r in all_results]
    scored.sort(key=lambda x: x[0])
    best_score, best_r = scored[0]
    best_ext = best_r["result"]["extended_metrics"]
    best_params = best_r["params"]

    total_time = FIXED_PARAMS["num_steps"] * FIXED_PARAMS["dt"]

    lines = [
        "=" * 70,
        "B01 单关节 MPC 自动调参总结",
        "=" * 70,
        "",
        f"调参组数: {len(all_results)}",
        f"总仿真时间: {total_time:.2f} s",
        "",
        "─" * 70,
        "最优参数组合:",
        "─" * 70,
    ]
    for k, v in best_params.items():
        lines.append(f"  {k}: {v}")

    lines.extend([
        "",
        "─" * 70,
        "最优组指标:",
        "─" * 70,
        f"  综合评分 (score): {best_score:.4f}",
        f"  final_abs_error: {best_ext['final_abs_error']:.4f} rad",
        f"  mean_abs_error: {best_ext['mean_abs_error']:.4f} rad",
        f"  max_abs_error: {best_ext['max_abs_error']:.4f} rad",
        f"  settling_time: {best_ext['settling_time']:.2f} s ({best_ext['settling_time']/total_time*100:.1f}% of total)",
        f"  max_abs_torque: {best_ext['max_abs_torque']:.4f} Nm",
        f"  mean_abs_torque: {best_ext['mean_abs_torque']:.4f} Nm",
        f"  torque_saturation_ratio: {best_ext['torque_saturation_ratio']:.4f} ({best_ext['torque_saturation_ratio']*100:.1f}%)",
        f"  mean_best_cost: {best_ext['mean_best_cost']:.4f}",
        f"  final_best_cost: {best_ext['final_best_cost']:.4f}",
        f"  runtime_per_control_step_mean: {best_ext['runtime_per_control_step_mean']:.4f} s",
        f"  runtime_per_control_step_max: {best_ext['runtime_per_control_step_max']:.4f} s",
        "",
        "─" * 70,
        "停止指标检查:",
        "─" * 70,
    ])
    for name, met in criteria_best.items():
        status = "PASS" if met else "FAIL"
        lines.append(f"  [{status}] {name}")

    all_met = all(criteria_best.values())
    lines.extend([
        "",
        f"是否全部达标: {'是' if all_met else '否'}",
    ])

    # 瓶颈分析
    lines.extend([
        "",
        "─" * 70,
        "瓶颈分析:",
        "─" * 70,
    ])
    if not criteria_best.get("final_abs_error", True):
        lines.append("  - final_abs_error 未达标：horizon 不够或 q_weight/terminal_weight 不足")
    if not criteria_best.get("mean_abs_error", True):
        lines.append("  - mean_abs_error 未达标：q_weight 不够或 dq_weight 太低导致振荡")
    if not criteria_best.get("settling_time", True):
        lines.append("  - settling_time 未达标：horizon 不够或 q_weight/terminal_weight 不足")
    if not criteria_best.get("torque_saturation_ratio", True):
        lines.append("  - torque_saturation_ratio 未达标：torque_weight 太低导致力矩过猛")
    if not criteria_best.get("max_abs_torque", True):
        lines.append("  - max_abs_torque 未达标：torque_weight 太低或 torque_limit 太高")
    if not criteria_best.get("runtime", True):
        lines.append("  - runtime 未达标：num_candidates 或 horizon 过大，需要减少计算量")

    # 下一轮建议
    lines.extend([
        "",
        "─" * 70,
        "下一轮调参建议:",
        "─" * 70,
    ])
    if all_met:
        lines.append("  所有指标已达标，无需继续调参。")
    else:
        if not criteria_best.get("runtime", True):
            lines.append(f"  - 减少 num_candidates 至 {max(32, best_params['num_candidates'] // 2)}")
            lines.append(f"  - 减少 horizon 至 {max(5, best_params['horizon'] - 5)}")
        if not criteria_best.get("final_abs_error", True) or not criteria_best.get("settling_time", True):
            lines.append(f"  - 增大 q_weight 至 {best_params['q_weight'] * 2:.1f}")
            lines.append(f"  - 增大 terminal_weight 至 {best_params['terminal_weight'] * 2:.1f}")
            lines.append(f"  - 增大 horizon 至 {best_params['horizon'] + 10}")
        if not criteria_best.get("torque_saturation_ratio", True) or not criteria_best.get("max_abs_torque", True):
            lines.append(f"  - 增大 torque_weight 至 {best_params['torque_weight'] * 2:.4f}")
        if best_params["dq_weight"] < 0.1:
            lines.append(f"  - 增大 dq_weight 至 {best_params['dq_weight'] * 2:.3f} 以减少振荡")

    lines.extend(["", "=" * 70])
    summary = "\n".join(lines)
    output_path.write_text(summary, encoding="utf-8")
    return summary


# ── 主流程 ────────────────────────────────────────────────────────────────

def main() -> None:
    tuning_run_id = f"tuning_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    output_root = PROJECT_ROOT / "outputs" / "runs"
    tuning_dir = output_root / "B01_single_joint_mpc_demo" / tuning_run_id
    figures_dir = tuning_dir / "figures"
    metrics_dir = tuning_dir / "metrics"
    logs_dir = tuning_dir / "logs"

    for d in (figures_dir, metrics_dir, logs_dir):
        d.mkdir(parents=True, exist_ok=True)

    config_path = PROJECT_ROOT / "configs" / "B01_single_joint_mpc.yaml"

    print("=" * 70)
    print(f"B01 自动调参开始 — tuning_run_id: {tuning_run_id}")
    print("=" * 70)

    all_results: list[dict[str, Any]] = []

    # ── 第一阶段：粗搜索 ──────────────────────────────────────────────
    print("\n[Phase 1] 粗搜索：拉丁超立方采样 25 组参数 ...")
    phase1_samples = latin_hypercube_sample(SEARCH_SPACE, n_samples=25, seed=42)

    for i, params in enumerate(phase1_samples):
        run_id = f"{tuning_run_id}_p1_{i:03d}"
        print(f"  [{i+1}/{len(phase1_samples)}] {run_id}: {params}")
        t0 = time.time()
        try:
            result = run_single_experiment(params, config_path, run_id, output_root)
            elapsed = time.time() - t0
            ext = result["result"]["extended_metrics"]
            score = compute_score(ext)
            print(f"    score={score:.4f}  final_err={ext['final_abs_error']:.4f}  "
                  f"mean_err={ext['mean_abs_error']:.4f}  runtime={ext['runtime_per_control_step_mean']:.4f}s  "
                  f"elapsed={elapsed:.1f}s")
            all_results.append(result)
        except Exception as e:
            print(f"    ERROR: {e}")

    # 选出第一阶段 top 3
    scored_phase1 = [(compute_score(r["result"]["extended_metrics"]), r) for r in all_results]
    scored_phase1.sort(key=lambda x: x[0])
    top3_params = [r["params"] for _, r in scored_phase1[:3]]

    print(f"\n[Phase 1] Top 3 参数:")
    for i, (_, r) in enumerate(scored_phase1[:3]):
        ext = r["result"]["extended_metrics"]
        print(f"  #{i+1} score={compute_score(ext):.4f} params={r['params']}")

    # ── 第二阶段：局部细调 ──────────────────────────────────────────
    print(f"\n[Phase 2] 局部细调：围绕 top 3 参数各生成 4 组候选 ...")
    phase2_candidates: list[dict[str, float]] = []
    for j, base_params in enumerate(top3_params):
        candidates = generate_phase2_candidates(base_params, SEARCH_SPACE, n_candidates=4, seed=200 + j)
        phase2_candidates.extend(candidates)

    # 去重（与已有结果比较）
    seen_keys = set()
    for r in all_results:
        key = tuple(sorted(r["params"].items()))
        seen_keys.add(key)

    unique_phase2 = []
    for c in phase2_candidates:
        key = tuple(sorted(c.items()))
        if key not in seen_keys:
            unique_phase2.append(c)
            seen_keys.add(key)

    print(f"  去重后细调组数: {len(unique_phase2)}")

    for i, params in enumerate(unique_phase2):
        run_id = f"{tuning_run_id}_p2_{i:03d}"
        print(f"  [{i+1}/{len(unique_phase2)}] {run_id}: {params}")
        t0 = time.time()
        try:
            result = run_single_experiment(params, config_path, run_id, output_root)
            elapsed = time.time() - t0
            ext = result["result"]["extended_metrics"]
            score = compute_score(ext)
            print(f"    score={score:.4f}  final_err={ext['final_abs_error']:.4f}  "
                  f"mean_err={ext['mean_abs_error']:.4f}  runtime={ext['runtime_per_control_step_mean']:.4f}s  "
                  f"elapsed={elapsed:.1f}s")
            all_results.append(result)
        except Exception as e:
            print(f"    ERROR: {e}")

    # ── 第三阶段：精细调参 ──────────────────────────────────────────
    # 基于 Phase 1+2 结果，针对关键瓶颈做精细调整
    # 策略：以全局 top 3 为基点，手动构造针对性候选
    print("\n[Phase 3] 精细调参：针对关键瓶颈手动构造候选 ...")
    scored_all = [(compute_score(r["result"]["extended_metrics"]), r) for r in all_results]
    scored_all.sort(key=lambda x: x[0])
    global_top3 = [r["params"] for _, r in scored_all[:3]]

    # 构造精细候选：围绕 top3 微调关键参数
    phase3_candidates: list[dict[str, float]] = []
    for base in global_top3:
        # 变体1：增大 dq_weight 减少振荡
        v1 = dict(base); v1["dq_weight"] = min(1.0, base["dq_weight"] * 3)
        phase3_candidates.append(v1)
        # 变体2：增大 torque_weight 减少饱和
        v2 = dict(base); v2["torque_weight"] = min(0.01, base["torque_weight"] * 3)
        phase3_candidates.append(v2)
        # 变体3：torque_limit=2.0 匹配 XML ctrlrange
        v3 = dict(base); v3["torque_limit"] = 2.0
        phase3_candidates.append(v3)
        # 变体4：增大 terminal_weight
        v4 = dict(base); v4["terminal_weight"] = min(20.0, base["terminal_weight"] * 3)
        phase3_candidates.append(v4)

    # 去重
    unique_phase3 = []
    for c in phase3_candidates:
        key = tuple(sorted(c.items()))
        if key not in seen_keys:
            unique_phase3.append(c)
            seen_keys.add(key)

    print(f"  去重后精细调参组数: {len(unique_phase3)}")

    for i, params in enumerate(unique_phase3):
        run_id = f"{tuning_run_id}_p3_{i:03d}"
        print(f"  [{i+1}/{len(unique_phase3)}] {run_id}: {params}")
        t0 = time.time()
        try:
            result = run_single_experiment(params, config_path, run_id, output_root)
            elapsed = time.time() - t0
            ext = result["result"]["extended_metrics"]
            score = compute_score(ext)
            print(f"    score={score:.4f}  final_err={ext['final_abs_error']:.4f}  "
                  f"mean_err={ext['mean_abs_error']:.4f}  runtime={ext['runtime_per_control_step_mean']:.4f}s  "
                  f"sat_ratio={ext['torque_saturation_ratio']:.3f}  "
                  f"elapsed={elapsed:.1f}s")
            all_results.append(result)
        except Exception as e:
            print(f"    ERROR: {e}")

    # ── 最终排名 ──────────────────────────────────────────────────────
    print("\n" + "=" * 70)
    print("最终排名 (score 越低越好):")
    print("=" * 70)
    final_scored = [(compute_score(r["result"]["extended_metrics"]), r) for r in all_results]
    final_scored.sort(key=lambda x: x[0])
    for i, (score, r) in enumerate(final_scored):
        ext = r["result"]["extended_metrics"]
        print(f"  #{i+1:2d} score={score:8.4f}  final_err={ext['final_abs_error']:.4f}  "
              f"params={r['params']}")

    # ── 生成输出 ──────────────────────────────────────────────────────
    best_ext = final_scored[0][1]["result"]["extended_metrics"]
    best_params = final_scored[0][1]["params"]
    total_time = FIXED_PARAMS["num_steps"] * FIXED_PARAMS["dt"]
    criteria = check_stop_criteria(best_ext, best_params.get("torque_limit", 2.0), total_time)

    # 对比图
    print("\n生成对比图表 ...")
    generate_comparison_plots(all_results, figures_dir, FIXED_PARAMS["target_angle"])

    # 指标汇总表
    print("生成指标汇总 CSV ...")
    generate_metrics_table(all_results, metrics_dir / "B01_tuning_metrics_table.csv")

    # Tuning summary
    print("生成 tuning summary ...")
    summary = generate_tuning_summary(all_results, criteria, logs_dir / "B01_tuning_summary.txt")

    print("\n" + summary)
    print(f"\n所有输出保存到: {tuning_dir}")


if __name__ == "__main__":
    main()
