"""Run the B03-R4B mini iLQR-lite toy smoke demo.

本脚本只运行一个 double-integrator toy dynamics：
- 状态 x = [position, velocity]
- 控制 u = [acceleration]

它用于验证 iLQR-lite 的数学闭环是否最小可运行，不启动 MuJoCo，不生成 MP4。
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
import sys
from typing import Any

import numpy as np
import yaml


# 当前脚本自身所在位置，用它向上推导 simulator/project/repo 根目录。
SCRIPT_PATH = Path(__file__).resolve()
# simulator 根目录：projects/B_mujoco_mpc_study/simulator。
SIMULATOR_ROOT = SCRIPT_PATH.parents[1]
# Project B 根目录：projects/B_mujoco_mpc_study。
PROJECT_ROOT = SIMULATOR_ROOT.parent
# 仓库根目录：Robot_Dynamics_Control。
REPO_ROOT = PROJECT_ROOT.parent.parent
# toy iLQR-lite demo 的默认配置文件。
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "configs" / "B03_ilqg_lite.yaml"
# 输出目录中的任务名。
TASK_NAME = "B03_ilqg_lite_demo"

# 把仓库根目录加入 sys.path，保证可以从 projects.* 绝对导入。
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
# 把 simulator 根目录加入 sys.path，兼容 planners.* 这类局部导入。
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

from projects.B_mujoco_mpc_study.simulator.planners.ilqg_solver import ILQGLiteSolver
from projects.B_mujoco_mpc_study.simulator.planners.mpc_solver_interface import MPCProblem


def build_arg_parser() -> argparse.ArgumentParser:
    """Create CLI arguments for the toy iLQR-lite demo."""
    # argparse 只负责命令行参数，不直接启动求解器。
    parser = argparse.ArgumentParser(description="Run B03-R4B mini iLQR-lite toy smoke demo.")
    # 配置文件路径，默认指向 B03_ilqg_lite.yaml。
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH, help="B03 iLQR-lite YAML config path.")
    # run-id 用于固定输出目录名，便于复现实验。
    parser.add_argument("--run-id", type=str, default=None, help="Optional output run id.")
    # 当前 B03-R4B 只支持 toy，不支持 MuJoCo two_link。
    parser.add_argument("--model-family", type=str, default=None, help="Only 'toy' is supported in B03-R4B.")
    # 命令行 horizon 覆盖 YAML 中的 horizon。
    parser.add_argument("--horizon", type=int, default=None, help="Override planning horizon.")
    # 命令行 max_iterations 覆盖 YAML 中的 max_iterations。
    parser.add_argument("--max-iterations", type=int, default=None, help="Override iLQR max iterations.")
    # 是否写 markdown report；默认写，仍然不生成 MP4。
    parser.add_argument("--save-report", action=argparse.BooleanOptionalAction, default=True, help="Save markdown report.")
    # 是否保存 PNG/PDF 图像；图像只用于结果复盘，不是正式视频输出。
    parser.add_argument("--save-figures", action=argparse.BooleanOptionalAction, default=True, help="Save summary PNG/PDF figures.")
    # 返回 parser，main() 中再 parse_args。
    return parser


def load_config(config_path: Path) -> dict[str, Any]:
    """Load YAML config with a clear path error."""
    # 配置不存在时给出清晰错误，不静默使用默认参数。
    if not config_path.exists():
        raise FileNotFoundError(f"Cannot find B03 iLQR-lite config: {config_path}")
    # 按 UTF-8 读取 YAML。
    with config_path.open("r", encoding="utf-8") as config_file:
        # 空 YAML 视为 {}，方便后续用默认值。
        loaded = yaml.safe_load(config_file) or {}
    # 顶层必须是 dict/mapping，不能是 list 或标量。
    if not isinstance(loaded, dict):
        raise ValueError(f"B03 iLQR-lite config must be a mapping, got {type(loaded)!r}")
    # 返回解析后的配置字典。
    return loaded


def build_run_dir(run_id: str | None) -> Path:
    """Build output directory under project outputs."""
    # 如果用户没有给 run_id，就用时间戳生成一个唯一目录名。
    actual_run_id = run_id or datetime.now().strftime("%Y%m%d_%H%M%S")
    # 所有输出都放在 Project B 的 outputs/runs 下。
    return PROJECT_ROOT / "outputs" / "runs" / TASK_NAME / actual_run_id


def _diag_from_config(value: Any, name: str) -> np.ndarray:
    """Read a diagonal vector from YAML and return a diagonal matrix."""
    # YAML 中 Q/R/Q_terminal 用对角线列表表达，先转成 float 向量。
    diag_values = np.asarray(value, dtype=float)
    # 对角线列表必须是一维且非空。
    if diag_values.ndim != 1 or diag_values.size == 0:
        raise ValueError(f"{name} must be a non-empty 1D diagonal list, got shape={diag_values.shape}")
    # 权重不能包含 NaN / inf。
    if not np.isfinite(diag_values).all():
        raise ValueError(f"{name} must contain only finite values")
    # 把 [a, b, c] 转成 diag([a, b, c])。
    return np.diag(diag_values)


def build_toy_dynamics(dt: float):
    """Return double-integrator discrete dynamics."""
    # dt 是离散积分步长，必须为正。
    if dt <= 0.0:
        raise ValueError(f"dt must be positive, got {dt}")

    # 返回一个闭包 dynamics_fn，让 solver 只看到统一接口 f(x, u)。
    def dynamics_fn(x: np.ndarray, u: np.ndarray) -> np.ndarray:
        # 当前状态 x = [position, velocity]。
        x_arr = np.asarray(x, dtype=float)
        # 当前控制 u = [acceleration]。
        u_arr = np.asarray(u, dtype=float)
        # 用最简单的欧拉离散 double-integrator 动力学生成下一状态。
        return np.array(
            [
                # position_next = position + dt * velocity。
                x_arr[0] + dt * x_arr[1],
                # velocity_next = velocity + dt * acceleration。
                x_arr[1] + dt * u_arr[0],
            ],
            dtype=float,
        )

    # 返回给 MPCProblem.dynamics_fn 使用。
    return dynamics_fn


def build_toy_problem(config: dict[str, Any], args: argparse.Namespace) -> MPCProblem:
    """Build the toy double-integrator MPCProblem consumed by ILQGLiteSolver."""
    # simulation 段保存 dt、horizon、初始状态和目标。
    simulation = config.get("simulation", {})
    # ilqg_lite 段保存 solver 参数。
    ilqg_cfg = config.get("ilqg_lite", {})
    # cost 段保存 Q/R/Q_terminal 对角线权重。
    cost_cfg = config.get("cost", {})

    # model_family 可由命令行覆盖；当前只允许 toy。
    model_family = args.model_family or simulation.get("model_family", "toy")
    # B03-R4B 明确不在这个 runner 中运行 MuJoCo。
    if model_family != "toy":
        raise ValueError("B03-R4B iLQR-lite demo currently supports only --model-family toy.")

    # 从配置读取离散时间步长。
    dt = float(simulation.get("dt", 0.1))
    # horizon 优先使用命令行，其次 YAML simulation，最后 YAML ilqg_lite/default。
    horizon = int(args.horizon if args.horizon is not None else simulation.get("horizon", ilqg_cfg.get("horizon", 32)))
    # 初始状态默认是静止在 position=0。
    initial_state = np.asarray(simulation.get("initial_state", [0.0, 0.0]), dtype=float)
    # toy double integrator 的状态维度固定为 2。
    if initial_state.shape != (2,):
        raise ValueError(f"toy initial_state must have shape (2,), got {initial_state.shape}")

    # 目标位置默认是 1.0。
    target_position = float(simulation.get("target_position", 1.0))
    # 目标速度默认是 0.0，表示希望最终停住。
    target_velocity = float(simulation.get("target_velocity", 0.0))
    # x_refs 比控制 horizon 多一项，用于 terminal reference。
    x_refs = np.zeros((horizon + 1, 2), dtype=float)
    # 所有时间步位置参考都设为 target_position。
    x_refs[:, 0] = target_position
    # 所有时间步速度参考都设为 target_velocity。
    x_refs[:, 1] = target_velocity

    # 组装 ILQGLiteSolver 所需的 solver_config。
    solver_config = {
        # 最大 iLQR 迭代次数。
        "max_iterations": int(args.max_iterations if args.max_iterations is not None else ilqg_cfg.get("max_iterations", 10)),
        # Q_uu 求解正则项。
        "regularization": float(ilqg_cfg.get("regularization", 1e-6)),
        # finite difference 的扰动步长。
        "finite_difference_eps": float(ilqg_cfg.get("finite_difference_eps", 1e-5)),
        # cost 改善量小于该值时停止。
        "tolerance": float(ilqg_cfg.get("tolerance", 1e-8)),
        # 控制限幅，作用在 acceleration 上。
        "control_limit": float(ilqg_cfg.get("control_limit", 10.0)),
        # 是否启用 line search。
        "line_search": bool(ilqg_cfg.get("line_search", True)),
        # line search 候选步长。
        "line_search_alphas": list(ilqg_cfg.get("line_search_alphas", [1.0, 0.5, 0.25, 0.1, 0.05])),
    }

    # 从 YAML 对角线列表构造状态权重 Q。
    Q = _diag_from_config(cost_cfg.get("Q", [10.0, 1.0]), "cost.Q")
    # 从 YAML 对角线列表构造控制权重 R。
    R = _diag_from_config(cost_cfg.get("R", [0.1]), "cost.R")
    # 从 YAML 对角线列表构造终端状态权重 Q_terminal。
    Q_terminal = _diag_from_config(cost_cfg.get("Q_terminal", [20.0, 2.0]), "cost.Q_terminal")

    # MPCProblem 是 B03 所有 solver 的统一输入结构。
    return MPCProblem(
        # 当前状态。
        current_state=initial_state,
        # 这里 target_horizon 直接放 x_refs，方便缺省解析。
        target_horizon=x_refs,
        # 规划 horizon。
        horizon=horizon,
        # toy 控制只有一个 acceleration。
        control_dim=1,
        # 离散时间步长。
        dt=dt,
        # 本 runner 把 Q/R 放在 metadata，因此 cost_config 保持为空。
        cost_config={},
        # solver 参数。
        solver_config=solver_config,
        # 外部动力学函数。
        dynamics_fn=build_toy_dynamics(dt),
        # iLQR-lite 当前所需的参考轨迹和二次 cost 权重。
        metadata={
            "x_refs": x_refs,
            "Q": Q,
            "R": R,
            "Q_terminal": Q_terminal,
        },
    )


def save_cost_history_csv(cost_history: list[float], output_path: Path) -> None:
    """Save iLQR iteration cost history."""
    # 确保 CSV 所在目录存在。
    output_path.parent.mkdir(parents=True, exist_ok=True)
    # newline="" 避免 Windows 下 CSV 出现额外空行。
    with output_path.open("w", encoding="utf-8", newline="") as csv_file:
        # 使用标准 csv writer 写表格。
        writer = csv.writer(csv_file)
        # 表头：迭代序号和总 cost。
        writer.writerow(["iteration", "total_cost"])
        # 逐行写入 cost history。
        for iteration, cost in enumerate(cost_history):
            # 使用紧凑浮点格式，既可读又避免太长。
            writer.writerow([iteration, f"{float(cost):.12g}"])


def save_summary_figures(output_dir: Path, result: dict[str, Any]) -> dict[str, Path]:
    """Save PNG/PDF figures for reading the toy iLQR-lite result."""
    # matplotlib/seaborn 只在需要绘图时导入，避免无图运行路径引入额外启动成本。
    import matplotlib.pyplot as plt
    import seaborn as sns

    # 确保 figures 目录存在。
    output_dir.mkdir(parents=True, exist_ok=True)
    # 取出 problem，里面有 dt、horizon、x_refs。
    problem = result["problem"]
    # 取出 solution，里面有 predicted_states、predicted_controls、metadata。
    solution = result["solution"]
    # 预测状态轨迹，shape = (H + 1, 2)。
    states = np.asarray(solution.predicted_states, dtype=float)
    # 预测控制序列，shape = (H, 1)。
    controls = np.asarray(solution.predicted_controls, dtype=float)
    # 参考状态轨迹，shape = (H + 1, 2)。
    x_refs = np.asarray(problem.metadata["x_refs"], dtype=float)
    # cost history 是每次 iLQR 接受更新后的 total cost。
    cost_history = np.asarray(solution.metadata.get("cost_history", []), dtype=float)
    # 状态时间轴长度为 H + 1。
    state_time = np.arange(states.shape[0], dtype=float) * float(problem.dt)
    # 控制时间轴长度为 H。
    control_time = np.arange(controls.shape[0], dtype=float) * float(problem.dt)
    # cost history 的横轴是 iLQR iteration index。
    cost_iter = np.arange(cost_history.size, dtype=int)

    # 设置统一绘图风格：白底、弱网格、DejaVu 字体。
    sns.set_theme(font_scale=1.0, style="whitegrid", font="DejaVu Sans")
    # cubehelix 调色板用于三张线图，避免全蓝或过度鲜艳。
    palette = sns.cubehelix_palette(6, rot=-0.25, light=0.7)
    # 建立 1x3 多面板图，便于同时看收敛、状态、控制。
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8), dpi=150)
    # 总标题说明这张图的阅读目标。
    fig.suptitle("B03-R4B mini iLQR-lite toy smoke summary", fontsize=14, color="dimgrey", y=1.02)

    # --- Panel 1: cost history ---
    ax = axes[0]
    # cost history 用 line + marker，直接看每次 accepted update 是否下降。
    ax.plot(cost_iter, cost_history, marker="o", linewidth=2.0, color=palette[5], label="total cost")
    # 标题加面板编号，方便报告引用。
    ax.set_title(r"$\bf{(a)}$ Cost history", loc="left", fontsize=11, pad=7)
    # x 轴是 iLQR 迭代编号。
    ax.set_xlabel("iLQR accepted iteration", fontsize=9, color="dimgrey")
    # y 轴是 total cost。
    ax.set_ylabel("Total cost", fontsize=9, color="dimgrey")
    # 如果至少有一个 cost，就标注最后 cost。
    if cost_history.size:
        ax.text(
            cost_iter[-1],
            cost_history[-1],
            f"final {cost_history[-1]:.3g}",
            color="dimgrey",
            fontsize=9,
            ha="left",
            va="bottom",
        )

    # --- Panel 2: state trajectory ---
    ax = axes[1]
    # 位置预测轨迹。
    ax.plot(state_time, states[:, 0], linewidth=2.0, color=palette[5], label="position")
    # 位置参考轨迹，用虚线区分参考值。
    ax.plot(state_time, x_refs[:, 0], linewidth=1.6, linestyle="--", color="dimgrey", label="position ref")
    # 速度预测轨迹。
    ax.plot(state_time, states[:, 1], linewidth=2.0, color=palette[3], label="velocity")
    # 速度参考轨迹，用虚线区分参考值。
    ax.plot(state_time, x_refs[:, 1], linewidth=1.6, linestyle="--", color=palette[1], label="velocity ref")
    # 标题说明这是预测状态而不是闭环真实轨迹。
    ax.set_title(r"$\bf{(b)}$ Predicted states", loc="left", fontsize=11, pad=7)
    # 时间轴单位是秒。
    ax.set_xlabel("Time (s)", fontsize=9, color="dimgrey")
    # 状态值共用一个 y 轴，toy 状态单位只作学习示意。
    ax.set_ylabel("State value", fontsize=9, color="dimgrey")
    # 图例说明 position/velocity 与 reference。
    ax.legend(frameon=True, facecolor="white", framealpha=0.8, edgecolor="lightgrey", labelcolor="dimgrey", fontsize=8)

    # --- Panel 3: control sequence ---
    ax = axes[2]
    # acceleration 控制序列用阶梯图，表达控制在每个离散时间步保持常值。
    ax.step(control_time, controls[:, 0], where="post", linewidth=2.0, color=palette[5], label="acceleration")
    # 0 控制线作为参考。
    ax.axhline(0.0, color="lightgrey", linewidth=0.9)
    # 标题说明这是 predicted control。
    ax.set_title(r"$\bf{(c)}$ Predicted control", loc="left", fontsize=11, pad=7)
    # 控制时间轴。
    ax.set_xlabel("Time (s)", fontsize=9, color="dimgrey")
    # 控制量是 acceleration。
    ax.set_ylabel("Acceleration", fontsize=9, color="dimgrey")
    # 标注最大绝对控制，帮助快速判断是否触及限幅。
    if controls.size:
        max_abs_control = float(np.max(np.abs(controls[:, 0])))
        ax.text(
            0.98,
            0.94,
            f"max |u| = {max_abs_control:.3g}",
            transform=ax.transAxes,
            ha="right",
            va="top",
            fontsize=9,
            color="dimgrey",
        )

    # 所有子图统一去除重边框、弱化 tick。
    for axis in axes:
        axis.grid(False)
        axis.tick_params(axis="both", which="both", length=0, labelcolor="dimgrey")
        axis.patch.set_edgecolor("lightgrey")
        axis.patch.set_linewidth(0.8)
    # 应用统一 despine 风格。
    sns.despine(left=True, bottom=True)
    # 底部写一句图级结论，帮助读者理解这张图看什么。
    fig.text(
        0.99,
        0.01,
        f"termination: {solution.metadata.get('termination_reason')} | best cost: {solution.best_cost:.3g}",
        ha="right",
        va="bottom",
        fontsize=9,
        color="dimgrey",
        style="italic",
    )
    # 自动整理布局，给标题和底部注释留空间。
    fig.tight_layout(rect=(0.0, 0.05, 1.0, 0.95))

    # 同时保存 PNG 和 PDF，PNG 方便查看，PDF 方便后续文档引用。
    png_path = output_dir / "B03_ilqg_lite_toy_summary.png"
    pdf_path = output_dir / "B03_ilqg_lite_toy_summary.pdf"
    fig.savefig(png_path, dpi=150, bbox_inches="tight")
    fig.savefig(pdf_path, dpi=150, bbox_inches="tight")
    # 关闭 figure，避免批量运行时占用内存。
    plt.close(fig)
    # 返回输出路径，供 main/report 打印。
    return {"summary_png": png_path, "summary_pdf": pdf_path}


def save_report(output_path: Path, result: dict[str, Any]) -> None:
    """Save a compact markdown smoke report."""
    # 确保 report 所在目录存在。
    output_path.parent.mkdir(parents=True, exist_ok=True)
    # 从 result 中取出 solver 输出。
    solution = result["solution"]
    # metadata 保存 cost_history、accepted_alphas、termination_reason。
    metadata = solution.metadata
    # 组织 markdown 报告内容。
    lines = [
        "# B03-R4B mini iLQR-lite Toy Smoke Report",
        "",
        f"- run_dir: `{result['run_dir']}`",
        f"- model_family: toy",
        f"- horizon: {result['problem'].horizon}",
        f"- dt: {result['problem'].dt}",
        f"- solver_name: {solution.solver_name}",
        f"- success: {solution.solver_stats.success}",
        f"- message: {solution.solver_stats.message}",
        f"- num_iterations: {solution.solver_stats.num_iterations}",
        f"- num_forward_rollouts: {solution.solver_stats.num_rollouts}",
        f"- best_cost: {solution.best_cost:.12g}",
        f"- first_control: {solution.first_control.tolist()}",
        f"- termination_reason: {metadata.get('termination_reason')}",
        f"- accepted_alphas: {metadata.get('accepted_alphas')}",
        f"- summary_png: `{result.get('figure_paths', {}).get('summary_png', '')}`",
        "",
        "当前报告只验证 toy dynamics 的 mini iLQR-lite 核心闭环，不包含 MuJoCo、正式 MP4 或 B02 controller 修改。",
    ]
    # 写入 markdown report。
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_demo(args: argparse.Namespace) -> dict[str, Any]:
    """Run the toy smoke demo and write lightweight outputs."""
    # 读取 YAML 配置。
    config = load_config(args.config)
    # 根据配置和命令行参数构造 toy MPCProblem。
    problem = build_toy_problem(config, args)
    # 调用 iLQR-lite solver，得到统一 MPCSolution。
    solution = ILQGLiteSolver().solve(problem)

    # 构造本次运行输出目录。
    run_dir = build_run_dir(args.run_id)
    # metrics 目录保存 CSV。
    metrics_dir = run_dir / "metrics"
    # reports 目录保存 markdown report。
    reports_dir = run_dir / "reports"
    # figures 目录保存 PNG/PDF 图像。
    figures_dir = run_dir / "figures"
    # 创建 metrics 目录。
    metrics_dir.mkdir(parents=True, exist_ok=True)
    # 创建 reports 目录。
    reports_dir.mkdir(parents=True, exist_ok=True)
    # 创建 figures 目录。
    figures_dir.mkdir(parents=True, exist_ok=True)

    # cost history CSV 的输出路径。
    cost_history_csv = metrics_dir / "B03_ilqg_lite_cost_history.csv"
    # 从 solution.metadata 取出 cost_history 并保存。
    save_cost_history_csv(solution.metadata.get("cost_history", []), cost_history_csv)

    # markdown report 的输出路径。
    report_path = reports_dir / "B03_ilqg_lite_toy_smoke_report.md"
    # 把后续打印和报告需要的信息统一打包。
    result = {
        "problem": problem,
        "solution": solution,
        "run_dir": run_dir,
        "cost_history_csv": cost_history_csv,
        "report_path": report_path,
    }
    # 如果用户开启 figures，就保存 summary PNG/PDF。
    if args.save_figures:
        result["figure_paths"] = save_summary_figures(figures_dir, result)
    # 如果不开启 figures，也保留空字典，方便 report 读取。
    else:
        result["figure_paths"] = {}
    # 如果用户开启 report，就写 markdown 报告。
    if args.save_report:
        save_report(report_path, result)

    # 返回结果，main() 负责打印摘要。
    return result


def main(argv: list[str] | None = None) -> None:
    """CLI entry point."""
    # 构造命令行 parser。
    parser = build_arg_parser()
    # 解析命令行参数。
    args = parser.parse_args(argv)
    # 执行 toy smoke demo。
    result = run_demo(args)
    # 取出 solver 输出，方便打印摘要。
    solution = result["solution"]

    # 打印运行完成状态。
    print("B03-R4B iLQR-lite toy smoke completed.")
    # 打印输出根目录。
    print(f"run_dir: {result['run_dir']}")
    # 打印 cost history CSV 路径。
    print(f"cost_history_csv: {result['cost_history_csv']}")
    # 如果保存了 report，也打印 report 路径。
    if args.save_report:
        print(f"report_path: {result['report_path']}")
    # 如果保存了 figures，也打印 summary 图路径。
    if args.save_figures:
        print(f"summary_png: {result['figure_paths']['summary_png']}")
    # 打印最终 best cost。
    print(f"best_cost: {solution.best_cost:.12g}")
    # 打印 solver success 状态。
    print(f"success: {solution.solver_stats.success}")
    # 打印停止原因，便于判断是 tolerance / max_iterations / line_search_failed。
    print(f"termination_reason: {solution.metadata.get('termination_reason')}")


# 直接用 python 运行脚本时进入 main()。
if __name__ == "__main__":
    main()
