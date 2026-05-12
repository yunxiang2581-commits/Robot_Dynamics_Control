"""Run B01 single-joint MPC closed-loop demo.

本脚本负责把 B01 的环境、planner、controller 串成一个最小闭环：

当前状态 `(q, dq)` -> MPC 规划 -> 执行第一个 torque -> 记录结果。
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml

SCRIPT_PATH = Path(__file__).resolve()
SIMULATOR_ROOT = SCRIPT_PATH.parents[1]
PROJECT_ROOT = SIMULATOR_ROOT.parent
REPO_ROOT = PROJECT_ROOT.parents[1]
OUTPUT_ROOT = PROJECT_ROOT / "outputs"
DEFAULT_MODEL_PATH = SIMULATOR_ROOT / "models" / "B01_single_joint.xml"
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "configs" / "B01_single_joint_mpc.yaml"
TASK_NAME = "B01_single_joint_mpc_demo"

B01_OUTPUT_FILENAMES = {
    "video": ("videos", "B01_single_joint_mpc_demo.mp4"),
    "angle_tracking": ("figures", "B01_angle_tracking.png"),
    "angle_error": ("figures", "B01_angle_error.png"),
    "torque": ("figures", "B01_torque.png"),
    "best_cost": ("figures", "B01_best_cost.png"),
    "metrics": ("metrics", "B01_metrics.csv"),
    "log": ("logs", "B01_run_log.txt"),
}

DEFAULT_RUN_CONFIG: dict[str, Any] = {
    "model_path": DEFAULT_MODEL_PATH,
    "output_root": OUTPUT_ROOT / "runs",
    "run_id": None,
    "target_angle": 1.0,
    "target_profile": "smooth",
    "ramp_duration": 0.5,
    "initial_q": 0.0,
    "initial_dq": 0.0,
    "dt": 0.01,
    "horizon": 20,
    "num_candidates": 64,
    "num_steps": 200,
    "torque_limit": 2.0,
    "q_weight": 1.0,
    "dq_weight": 0.1,
    "torque_weight": 0.001,
    "terminal_weight": 5.0,
    "export_video": False,
    "save_figures": False,
    "save_metrics": False,
    "show_viewer": False,
    "real_time": True,
}


def build_arg_parser() -> argparse.ArgumentParser:
    """创建 B01 命令行参数。"""
    parser = argparse.ArgumentParser(description="Run B01 single-joint MPC demo.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH, help="B01 YAML 配置文件路径。")
    parser.add_argument("--model-path", type=Path, default=None, help="单关节 MuJoCo XML 路径。")
    parser.add_argument("--output-root", type=Path, default=None, help="运行结果根目录，默认 outputs/runs。")
    parser.add_argument("--run-id", type=str, default=None, help="本次运行 ID，默认使用当前时间。")
    parser.add_argument("--target-angle", type=float, default=None, help="目标关节角 q_target。")
    parser.add_argument("--target-profile", choices=("step", "ramp", "smooth"), default=None, help="目标输入方式：step、ramp 或 smooth。")
    parser.add_argument("--ramp-duration", type=float, default=None, help="ramp/smooth 从初始角过渡到目标角的时间。")
    parser.add_argument("--initial-q", type=float, default=None, help="初始关节角 q0。")
    parser.add_argument("--initial-dq", type=float, default=None, help="初始关节角速度 dq0。")
    parser.add_argument("--dt", type=float, default=None, help="MuJoCo 仿真步长。")
    parser.add_argument("--horizon", type=int, default=None, help="MPC 每次向未来预测的步数。")
    parser.add_argument("--num-candidates", type=int, default=None, help="predictive sampling 候选 torque 序列数量。")
    parser.add_argument("--num-steps", type=int, default=None, help="闭环控制总步数。")
    parser.add_argument("--torque-limit", type=float, default=None, help="单关节 actuator 力矩限制。")
    parser.add_argument("--q-weight", type=float, default=None, help="角度误差 cost 权重。")
    parser.add_argument("--dq-weight", type=float, default=None, help="速度误差 cost 权重。")
    parser.add_argument("--torque-weight", type=float, default=None, help="力矩惩罚 cost 权重。")
    parser.add_argument("--terminal-weight", type=float, default=None, help="终端角度误差 cost 权重。")
    parser.add_argument("--export-video", action=argparse.BooleanOptionalAction, default=None, help="是否导出 B01 mp4 视频。")
    parser.add_argument("--save-figures", action=argparse.BooleanOptionalAction, default=None, help="是否保存 B01 图像。")
    parser.add_argument("--save-metrics", action=argparse.BooleanOptionalAction, default=None, help="是否保存 B01 metrics CSV。")
    parser.add_argument("--show-viewer", action=argparse.BooleanOptionalAction, default=None, help="是否打开 MuJoCo 实时窗口。")
    parser.add_argument("--real-time", action=argparse.BooleanOptionalAction, default=None, help="打开 viewer 时是否按 dt 节奏实时播放。")
    return parser


def resolve_project_path(path_value: str | Path) -> Path:
    """把配置中的路径解析为绝对路径。"""
    path = Path(path_value)
    if path.is_absolute():
        return path
    return PROJECT_ROOT / path


def flatten_config(config: dict[str, Any]) -> dict[str, Any]:
    """把分组 YAML 配置转换为 runner 使用的扁平参数。"""
    flat: dict[str, Any] = {}

    model = config.get("model", {})
    target = config.get("target", {})
    initial_state = config.get("initial_state", {})
    simulation = config.get("simulation", {})
    mpc = config.get("mpc", {})
    cost = config.get("cost", {})
    outputs = config.get("outputs", {})
    viewer = config.get("viewer", {})

    if "path" in model:
        flat["model_path"] = resolve_project_path(model["path"])
    if "angle" in target:
        flat["target_angle"] = target["angle"]
    if "profile" in target:
        flat["target_profile"] = target["profile"]
    if "ramp_duration" in target:
        flat["ramp_duration"] = target["ramp_duration"]
    if "q" in initial_state:
        flat["initial_q"] = initial_state["q"]
    if "dq" in initial_state:
        flat["initial_dq"] = initial_state["dq"]
    if "dt" in simulation:
        flat["dt"] = simulation["dt"]
    if "num_steps" in simulation:
        flat["num_steps"] = simulation["num_steps"]
    if "horizon" in mpc:
        flat["horizon"] = mpc["horizon"]
    if "num_candidates" in mpc:
        flat["num_candidates"] = mpc["num_candidates"]
    if "torque_limit" in mpc:
        flat["torque_limit"] = mpc["torque_limit"]
    for key in ("q_weight", "dq_weight", "torque_weight", "terminal_weight"):
        if key in cost:
            flat[key] = cost[key]
    for key in ("export_video", "save_figures", "save_metrics"):
        if key in outputs:
            flat[key] = outputs[key]
    if "show" in viewer:
        flat["show_viewer"] = viewer["show"]
    if "real_time" in viewer:
        flat["real_time"] = viewer["real_time"]
    if "root" in outputs:
        flat["output_root"] = resolve_project_path(outputs["root"])
    if "run_id" in outputs:
        flat["run_id"] = outputs["run_id"]

    # 也允许简单扁平配置，便于临时实验。
    for key in DEFAULT_RUN_CONFIG:
        if key in config:
            value = config[key]
            flat[key] = resolve_project_path(value) if key in ("model_path", "output_root") else value

    return flat


def load_config(config_path: Path) -> dict[str, Any]:
    """读取 B01 YAML 配置。"""
    if not config_path.exists():
        raise FileNotFoundError(f"找不到 B01 配置文件: {config_path}")
    raw_config = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw_config, dict):
        raise ValueError(f"B01 配置文件必须是 YAML mapping: {config_path}")
    return flatten_config(raw_config)


def resolve_run_args(cli_args: argparse.Namespace) -> argparse.Namespace:
    """合并代码默认值、配置文件和命令行覆盖参数。"""
    resolved = deepcopy(DEFAULT_RUN_CONFIG)
    resolved.update(load_config(cli_args.config))

    for key in DEFAULT_RUN_CONFIG:
        value = getattr(cli_args, key)
        if value is not None:
            resolved[key] = value

    resolved["model_path"] = Path(resolved["model_path"])
    resolved["output_root"] = Path(resolved["output_root"])
    if resolved["target_profile"] not in {"step", "ramp", "smooth"}:
        raise ValueError(f"target_profile 必须是 step/ramp/smooth，当前为 {resolved['target_profile']}")
    if resolved["ramp_duration"] <= 0.0:
        raise ValueError(f"ramp_duration 必须为正数，当前为 {resolved['ramp_duration']}")
    if resolved["run_id"] is None:
        resolved["run_id"] = datetime.now().strftime("%Y%m%d_%H%M%S")
    resolved["task_name"] = TASK_NAME
    resolved["run_dir"] = resolved["output_root"] / TASK_NAME / str(resolved["run_id"])
    resolved["config"] = cli_args.config
    return argparse.Namespace(**resolved)


def build_run_outputs(run_dir: Path) -> dict[str, Path]:
    """根据本次运行目录生成 B01 输出路径。"""
    return {
        key: run_dir / subdir / filename
        for key, (subdir, filename) in B01_OUTPUT_FILENAMES.items()
    }


def ensure_output_dirs(outputs: dict[str, Path]) -> None:
    """创建 B01 输出文件所在目录。"""
    for output_path in outputs.values():
        output_path.parent.mkdir(parents=True, exist_ok=True)


def smoothstep(alpha: float) -> float:
    """三次 smoothstep 插值。

    教学说明：
    - 要实现什么：把 0 到 1 的线性进度变成起止速度为 0 的平滑进度。
    - 为什么要实现：阶跃目标会造成瞬时大误差和大力矩，smooth target 更像真实轨迹跟踪。
    - 输入是什么：归一化时间 alpha，范围通常是 [0, 1]。
    - 输出是什么：平滑后的进度，仍在 [0, 1]。
    - 物理意义：目标角从初始角逐渐移动到最终角，减少启动冲击。
    - 数学意义：s(alpha)=3a^2-2a^3，满足 s(0)=0, s(1)=1, s'(0)=s'(1)=0。
    - 验证标准：alpha=0 输出 0，alpha=1 输出 1，中间值单调递增。
    """
    a = max(0.0, min(1.0, alpha))
    return 3.0 * a * a - 2.0 * a * a * a


def compute_target_angle_at_time(
    time_s: float,
    initial_q: float,
    final_target: float,
    profile: str,
    ramp_duration: float,
) -> float:
    """计算某一时刻的目标角 q_target(t)。"""
    if profile == "step":
        return final_target

    alpha = time_s / ramp_duration
    if profile == "ramp":
        blend = max(0.0, min(1.0, alpha))
    elif profile == "smooth":
        blend = smoothstep(alpha)
    else:
        raise ValueError(f"未知 target profile: {profile}")

    return initial_q + blend * (final_target - initial_q)


def build_target_sequence(
    current_time: float,
    horizon: int,
    dt: float,
    initial_q: float,
    final_target: float,
    profile: str,
    ramp_duration: float,
) -> list[float]:
    """生成 MPC horizon 内每一步的目标角序列。

    教学说明：
    - 要实现什么：为未来 H 步生成 `[q*_1, q*_2, ..., q*_H]`。
    - 为什么要实现：MPC cost 评价的是未来 rollout，目标也应该是未来对应时刻的目标。
    - 输入是什么：当前时间、horizon、dt、初始角、最终目标角和目标轨迹类型。
    - 输出是什么：长度为 horizon 的目标角列表。
    - 物理意义：控制器提前看到目标正在平滑移动，而不是突然跳到最终角。
    - 数学意义：cost 中使用时变参考 `q_k - q*_k`。
    - 验证标准：序列长度等于 horizon，最终会收敛到 final_target。
    """
    return [
        compute_target_angle_at_time(
            time_s=current_time + (k + 1) * dt,
            initial_q=initial_q,
            final_target=final_target,
            profile=profile,
            ramp_duration=ramp_duration,
        )
        for k in range(horizon)
    ]


def add_import_roots() -> None:
    """让脚本可从任意工作目录运行。"""
    for path in (REPO_ROOT, SIMULATOR_ROOT):
        path_text = str(path)
        if path_text not in sys.path:
            sys.path.insert(0, path_text)


def import_b01_components() -> tuple[type[Any], type[Any], type[Any]]:
    """延迟导入 B01 组件，避免模块导入阶段触发 MuJoCo 初始化。"""
    add_import_roots()

    from controllers.single_joint_mpc_controller import SingleJointMPCController
    from envs.single_joint_env import SingleJointEnv
    from planners.predictive_sampling import PredictiveSamplingPlanner

    return SingleJointEnv, PredictiveSamplingPlanner, SingleJointMPCController


def compute_summary_metrics(
    q_history: list[float],
    torque_history: list[float],
    runtime_history: list[float],
    q_target: float,
    target_history: list[float] | None = None,
) -> dict[str, float]:
    """计算 B01 最小验收指标。"""
    if not q_history:
        raise ValueError("q_history 不能为空，无法计算 tracking metrics。")

    targets = target_history if target_history is not None else [q_target] * len(q_history)
    errors = [abs(q - target) for q, target in zip(q_history, targets)]
    max_torque = max((abs(tau) for tau in torque_history), default=0.0)
    mean_runtime = sum(runtime_history) / len(runtime_history) if runtime_history else 0.0

    return {
        "final_error": errors[-1],
        "mean_tracking_error": sum(errors) / len(errors),
        "max_torque": max_torque,
        "runtime_per_control_step": mean_runtime,
    }


def compute_extended_metrics(
    q_history: list[float],
    torque_history: list[float],
    best_cost_history: list[float],
    runtime_history: list[float],
    q_target: float,
    torque_limit: float,
    dt: float,
    target_history: list[float] | None = None,
) -> dict[str, float]:
    """计算 B01 完整调参验收指标（11 项）。"""
    if not q_history:
        raise ValueError("q_history 不能为空，无法计算 extended metrics。")

    targets = target_history if target_history is not None else [q_target] * len(q_history)
    abs_errors = [abs(q - target) for q, target in zip(q_history, targets)]
    abs_torques = [abs(tau) for tau in torque_history]

    # 基础误差指标
    final_abs_error = abs_errors[-1]
    mean_abs_error = sum(abs_errors) / len(abs_errors)
    max_abs_error = max(abs_errors)

    # settling_time: 误差首次进入并保持在 0.05 rad 内的时间
    threshold = 0.05
    settling_time = len(q_history) * dt  # 默认为总仿真时间
    for i in range(len(abs_errors) - 1, -1, -1):
        if abs_errors[i] > threshold:
            settling_time = (i + 1) * dt
            break
    else:
        settling_time = 0.0  # 从第一步就在阈值内

    # 力矩指标
    # 注意：MuJoCo XML ctrlrange=[-2,2] 会进一步截断力矩，所以 torque_history
    # 记录的是 env 层截断后的值（受 torque_limit 限制），MuJoCo 层可能再截断。
    max_abs_torque = max(abs_torques) if abs_torques else 0.0
    mean_abs_torque = sum(abs_torques) / len(abs_torques) if abs_torques else 0.0
    # torque_saturation_ratio: 力矩接近实际有效上限的比例（> 90%）
    # 使用 min(torque_limit, 2.0) 作为有效上限，因为 XML ctrlrange=[-2,2]
    effective_limit = min(torque_limit, 2.0)
    sat_threshold = 0.9 * effective_limit
    sat_count = sum(1 for t in abs_torques if t >= sat_threshold)
    torque_saturation_ratio = sat_count / len(abs_torques) if abs_torques else 0.0

    # cost 指标
    mean_best_cost = sum(best_cost_history) / len(best_cost_history) if best_cost_history else 0.0
    final_best_cost = best_cost_history[-1] if best_cost_history else 0.0

    # runtime 指标
    runtime_per_control_step_mean = sum(runtime_history) / len(runtime_history) if runtime_history else 0.0
    runtime_per_control_step_max = max(runtime_history) if runtime_history else 0.0

    return {
        "final_abs_error": final_abs_error,
        "mean_abs_error": mean_abs_error,
        "max_abs_error": max_abs_error,
        "settling_time": settling_time,
        "max_abs_torque": max_abs_torque,
        "mean_abs_torque": mean_abs_torque,
        "torque_saturation_ratio": torque_saturation_ratio,
        "mean_best_cost": mean_best_cost,
        "final_best_cost": final_best_cost,
        "runtime_per_control_step_mean": runtime_per_control_step_mean,
        "runtime_per_control_step_max": runtime_per_control_step_max,
    }


def save_metrics_csv(metrics: dict[str, float], output_path: Path) -> None:
    """保存 B01 metrics CSV。"""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["metric", "value"])
        for name, value in metrics.items():
            writer.writerow([name, value])


def save_run_log(
    args: argparse.Namespace,
    metrics: dict[str, float],
    output_path: Path,
    num_frames: int,
) -> None:
    """保存一次 B01 运行摘要。"""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "B01 single-joint MPC run log",
        "",
        f"config: {getattr(args, 'config', None)}",
        f"task_name: {args.task_name}",
        f"run_id: {args.run_id}",
        f"run_dir: {args.run_dir}",
        f"model_path: {args.model_path}",
        f"target_angle: {args.target_angle}",
        f"target_profile: {args.target_profile}",
        f"ramp_duration: {args.ramp_duration}",
        f"initial_q: {args.initial_q}",
        f"initial_dq: {args.initial_dq}",
        f"dt: {args.dt}",
        f"horizon: {args.horizon}",
        f"num_candidates: {args.num_candidates}",
        f"num_steps: {args.num_steps}",
        f"torque_limit: {args.torque_limit}",
        f"q_weight: {args.q_weight}",
        f"dq_weight: {args.dq_weight}",
        f"torque_weight: {args.torque_weight}",
        f"terminal_weight: {args.terminal_weight}",
        f"show_viewer: {args.show_viewer}",
        f"real_time: {args.real_time}",
        f"recorded_frames: {num_frames}",
        "",
        "metrics:",
    ]
    lines.extend(f"- {name}: {value}" for name, value in metrics.items())
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def save_figures(
    time_history: list[float],
    q_history: list[float],
    torque_history: list[float],
    best_cost_history: list[float],
    q_target: float,
    outputs: dict[str, Path],
    target_history: list[float] | None = None,
) -> None:
    """保存 B01 曲线图。"""
    import matplotlib.pyplot as plt

    def save_line_plot(
        path: Path,
        title: str,
        ylabel: str,
        y_values: list[float],
        target: float | None = None,
        target_values: list[float] | None = None,
    ) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        plt.figure(figsize=(8, 4.5))
        plt.plot(time_history, y_values, label=ylabel)
        if target_values is not None:
            plt.plot(time_history, target_values, linestyle="--", color="tab:red", label="target")
        if target is not None:
            plt.axhline(target, linestyle="--", color="tab:red", label="target")
        plt.xlabel("time [s]")
        plt.ylabel(ylabel)
        plt.title(title)
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig(path)
        plt.close()

    targets = target_history if target_history is not None else [q_target] * len(q_history)
    angle_error = [q - target for q, target in zip(q_history, targets)]
    save_line_plot(outputs["angle_tracking"], "B01 angle tracking", "q [rad]", q_history, target_values=targets)
    save_line_plot(outputs["angle_error"], "B01 angle error", "q - q_target [rad]", angle_error)
    save_line_plot(outputs["torque"], "B01 torque command", "tau [Nm]", torque_history)
    save_line_plot(outputs["best_cost"], "B01 best horizon cost", "best cost", best_cost_history)


def save_video(frames: list[Any], output_path: Path, fps: int) -> None:
    """保存 MuJoCo 渲染帧为 mp4。"""
    if not frames:
        raise ValueError("frames 为空，无法导出视频。")

    try:
        import imageio.v2 as imageio
    except ImportError as exc:
        raise RuntimeError("导出 mp4 需要安装 imageio。") from exc

    output_path.parent.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(output_path, frames, fps=fps)


def launch_realtime_viewer(show_viewer: bool, env: Any) -> Any | None:
    """按需创建 MuJoCo 实时 viewer。

    教学说明：
    - 要实现什么：当 `show_viewer=True` 时打开 MuJoCo GUI 窗口，否则返回空上下文。
    - 为什么要实现：B01 除了离线视频，也需要能实时观察闭环控制过程。
    - 输入是什么：是否显示 viewer，以及持有 `model/data` 的 MuJoCo 环境。
    - 输出是什么：MuJoCo viewer handle；不开启时返回 None。
    - 物理意义：实时窗口显示同一个仿真状态，不改变动力学或控制律。
    - 数学意义：MPC cost、rollout、torque 选择完全不变，只增加状态显示。
    - 验证标准：不开 viewer 时原脚本行为不变；打开 viewer 时窗口能随仿真 step 刷新。
    """
    if not show_viewer:
        return None

    try:
        import mujoco.viewer as mujoco_viewer
    except Exception as exc:
        raise RuntimeError(
            "打开 MuJoCo 实时窗口需要当前 Python 环境支持 mujoco.viewer。"
        ) from exc

    return mujoco_viewer.launch_passive(env.model, env.data)


def run_closed_loop(args: argparse.Namespace) -> dict[str, float]:
    """执行 B01 MPC 闭环并保存请求的输出。"""
    if not args.model_path.exists():
        raise FileNotFoundError(f"找不到 B01 MuJoCo XML: {args.model_path}")

    outputs = build_run_outputs(args.run_dir)
    ensure_output_dirs(outputs)
    SingleJointEnv, PredictiveSamplingPlanner, SingleJointMPCController = import_b01_components()

    env = SingleJointEnv(model_path=str(args.model_path), dt=args.dt)
    planner = PredictiveSamplingPlanner(
        horizon=args.horizon,
        num_candidates=args.num_candidates,
        torque_limit=args.torque_limit,
        q_weight=args.q_weight,
        dq_weight=args.dq_weight,
        torque_weight=args.torque_weight,
        terminal_weight=args.terminal_weight,
    )
    controller = SingleJointMPCController(planner=planner, q_target=args.target_angle)

    state = env.reset(q=args.initial_q, dq=args.initial_dq)

    time_history: list[float] = []
    target_history: list[float] = []
    q_history: list[float] = []
    dq_history: list[float] = []
    torque_history: list[float] = []
    best_cost_history: list[float] = []
    runtime_history: list[float] = []
    frames: list[Any] = []
    viewer = launch_realtime_viewer(args.show_viewer, env)
    if viewer is not None:
        viewer.sync()

    for step_idx in range(args.num_steps):
        step_wall_start = time.perf_counter()
        if viewer is not None and not viewer.is_running():
            break

        current_time = step_idx * args.dt
        horizon_targets = build_target_sequence(
            current_time=current_time,
            horizon=args.horizon,
            dt=args.dt,
            initial_q=args.initial_q,
            final_target=args.target_angle,
            profile=args.target_profile,
            ramp_duration=args.ramp_duration,
        )
        immediate_target = horizon_targets[0]
        controller.update_target(immediate_target)
        start_time = time.perf_counter()
        tau = controller.compute_control(env, state, q_target_sequence=horizon_targets)
        runtime = time.perf_counter() - start_time

        state = env.step(tau)
        q, dq = state

        if controller.last_plan is None:
            raise RuntimeError("controller.last_plan 为空，无法记录 best_cost。")

        # q, dq 是执行当前 torque 之后的新状态，因此记录时间应对齐到 t + dt。
        # target_history 也记录同一时刻的 q_target(t + dt)，否则 ramp/smooth 误差会错位。
        record_time = current_time + args.dt
        time_history.append(record_time)
        target_history.append(immediate_target)
        q_history.append(q)
        dq_history.append(dq)
        torque_history.append(tau)
        runtime_history.append(runtime)
        best_cost_history.append(float(controller.last_plan["best_cost"]))

        if args.export_video:
            frames.append(env.render_frame())

        if viewer is not None:
            viewer.sync()

        if viewer is not None and args.real_time:
            elapsed = time.perf_counter() - step_wall_start
            sleep_time = args.dt - elapsed
            if sleep_time > 0.0:
                time.sleep(sleep_time)

    if viewer is not None:
        viewer.close()

    metrics = compute_summary_metrics(
        q_history=q_history,
        torque_history=torque_history,
        runtime_history=runtime_history,
        q_target=args.target_angle,
        target_history=target_history,
    )

    extended_metrics = compute_extended_metrics(
        q_history=q_history,
        torque_history=torque_history,
        best_cost_history=best_cost_history,
        runtime_history=runtime_history,
        q_target=args.target_angle,
        torque_limit=args.torque_limit,
        dt=args.dt,
        target_history=target_history,
    )

    save_run_log(args=args, metrics=metrics, output_path=outputs["log"], num_frames=len(frames))
    if args.save_metrics:
        save_metrics_csv(metrics, outputs["metrics"])
    if args.save_figures:
        save_figures(
            time_history=time_history,
            q_history=q_history,
            torque_history=torque_history,
            best_cost_history=best_cost_history,
            q_target=args.target_angle,
            outputs=outputs,
            target_history=target_history,
        )
    if args.export_video:
        fps = max(1, int(round(1.0 / args.dt)))
        save_video(frames, outputs["video"], fps=fps)

    return {
        "metrics": metrics,
        "extended_metrics": extended_metrics,
        "time_history": time_history,
        "target_history": target_history,
        "q_history": q_history,
        "dq_history": dq_history,
        "torque_history": torque_history,
        "best_cost_history": best_cost_history,
        "runtime_history": runtime_history,
    }


def main(argv: list[str] | None = None) -> None:
    """B01 demo 主入口。"""
    parser = build_arg_parser()
    args = resolve_run_args(parser.parse_args(argv))
    result = run_closed_loop(args)

    print("B01 single-joint MPC demo finished.")
    for name, value in result["metrics"].items():
        print(f"{name}: {value}")
    print(f"run_dir: {args.run_dir}")
    print(f"log: {build_run_outputs(args.run_dir)['log']}")


if __name__ == "__main__":
    main()
