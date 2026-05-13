"""Run B02 two-link MPC tracking demo.

本脚本把 B02 的环境、planner、controller 串成最小闭环：

当前状态 `(q1, q2, dq1, dq2)` -> 目标末端轨迹 -> MPC 规划 ->
执行第一步 `(tau1, tau2)` -> 记录末端误差、力矩和运行时间。

这里特别保持以下边界不变：
- controller 只负责输出控制量和最优 cost
- logger 只负责统一记录 tracking log
- renderer 只负责根据日志做二次渲染

这样就能逐步支持：
1. 第一版 `overlay`：2D 后处理叠加
2. 第二版 `scene`：MuJoCo scene marker
3. 第三版 `hybrid`：scene marker + 2D overlay 文本
"""

from __future__ import annotations

import argparse
import csv
import math
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
OUTPUT_ROOT = PROJECT_ROOT / "outputs"
DEFAULT_MODEL_PATH = SIMULATOR_ROOT / "models" / "B02_two_link.xml"
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "configs" / "B02_two_link_mpc.yaml"
TASK_NAME = "B02_two_link_mpc_tracking_demo"

REPO_ROOT = PROJECT_ROOT.parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from projects.B_mujoco_mpc_study.simulator.utils.marker_renderer import build_scene_marker_payload
from projects.B_mujoco_mpc_study.simulator.utils.plotting import save_b02_tracking_figures
from projects.B_mujoco_mpc_study.simulator.utils.trajectory_logger import TrackingLogBuffer
from projects.B_mujoco_mpc_study.simulator.utils.video_renderer import render_marked_video


B02_OUTPUT_FILENAMES = {
    "video": ("videos", "B02_two_link_mpc_tracking_demo.mp4"),
    "marked_video": ("videos", "B02_two_link_mpc_tracking_marked.mp4"),
    "ee_trajectory_xy": ("figures", "B02_ee_trajectory_xy.png"),
    "ee_tracking_error": ("figures", "B02_ee_tracking_error.png"),
    "joint_torque": ("figures", "B02_joint_torque.png"),
    "xy_target_vs_actual": ("figures", "B02_xy_target_vs_actual.png"),
    "tracking_error_time": ("figures", "B02_tracking_error_time.png"),
    "control_input_time": ("figures", "B02_control_input_time.png"),
    "metrics": ("metrics", "B02_metrics.csv"),
    "log": ("logs", "B02_control_log.txt"),
    "tracking_log": ("logs", "B02_tracking_log.csv"),
}


DEFAULT_RUN_CONFIG: dict[str, Any] = {
    "model_path": DEFAULT_MODEL_PATH,
    "end_effector_site": "ee_site",
    "output_root": OUTPUT_ROOT / "runs",
    "run_id": None,
    "target_type": "circle",
    "target_center": (0.55, 0.25),
    "target_radius": 0.08,
    "target_x_amplitude": 0.18,
    "target_y_amplitude": 0.10,
    "target_angular_speed": 1.0,
    "fixed_target": (0.55, 0.25),
    "custom_trajectory": None,
    "initial_q": (0.3, 0.4),
    "initial_dq": (0.0, 0.0),
    "dt": 0.01,
    "horizon": 15,
    "num_candidates": 256,
    "num_steps": 300,
    "torque_limit": 2.0,
    "ee_weight": 20.0,
    "dq_weight": 0.1,
    "torque_weight": 0.002,
    "terminal_weight": 5.0,
    "export_video": False,
    "save_figures": False,
    "save_metrics": False,
    "show_viewer": False,
    "real_time": True,
    "render_mode": "overlay",
}


def build_arg_parser() -> argparse.ArgumentParser:
    """创建 B02 命令行参数。"""
    parser = argparse.ArgumentParser(description="Run B02 two-link MPC tracking demo.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH, help="B02 YAML config path.")
    parser.add_argument("--model-path", type=Path, default=None, help="Two-link MuJoCo XML path.")
    parser.add_argument("--end-effector-site", type=str, default=None, help="End-effector site name.")
    parser.add_argument("--output-root", type=Path, default=None, help="Output root directory.")
    parser.add_argument("--run-id", type=str, default=None, help="Optional run id.")
    parser.add_argument("--num-steps", type=int, default=None, help="Number of closed-loop steps.")
    parser.add_argument("--dt", type=float, default=None, help="Simulation timestep.")
    parser.add_argument("--horizon", type=int, default=None, help="MPC horizon.")
    parser.add_argument("--num-candidates", type=int, default=None, help="Number of sampled candidates.")
    parser.add_argument("--torque-limit", type=float, default=None, help="Planner torque limit.")
    parser.add_argument("--target-radius", type=float, default=None, help="Circle target radius.")
    parser.add_argument("--target-angular-speed", type=float, default=None, help="Target angular speed.")
    parser.add_argument("--ee-weight", type=float, default=None, help="End-effector tracking weight.")
    parser.add_argument("--dq-weight", type=float, default=None, help="Velocity penalty weight.")
    parser.add_argument("--torque-weight", type=float, default=None, help="Torque penalty weight.")
    parser.add_argument("--terminal-weight", type=float, default=None, help="Terminal penalty weight.")
    parser.add_argument("--export-video", action=argparse.BooleanOptionalAction, default=None, help="Export MP4 video.")
    parser.add_argument("--save-figures", action=argparse.BooleanOptionalAction, default=None, help="Save PNG figures.")
    parser.add_argument("--save-metrics", action=argparse.BooleanOptionalAction, default=None, help="Save metrics CSV.")
    parser.add_argument("--show-viewer", action=argparse.BooleanOptionalAction, default=None, help="Show MuJoCo viewer.")
    parser.add_argument("--real-time", action=argparse.BooleanOptionalAction, default=None, help="Sync viewer to dt.")
    parser.add_argument(
        "--render-mode",
        type=str,
        choices=("overlay", "scene", "hybrid"),
        default=None,
        help="Marked video render mode.",
    )
    return parser


def resolve_project_path(path_value: str | Path) -> Path:
    """把配置里的相对路径解析到 Project B 根目录。"""
    path = Path(path_value)
    if path.is_absolute():
        return path
    return PROJECT_ROOT / path


def _as_pair(value: Any, name: str) -> tuple[float, float]:
    """把二维序列转换为 `(float, float)`。"""
    if len(value) != 2:
        raise ValueError(f"{name} must contain exactly 2 values, got {value}")
    return float(value[0]), float(value[1])


def _as_pair_sequence(value: Any, name: str) -> list[tuple[float, float]]:
    """把二维点序列转换为 `[(x0, y0), ...]`。"""
    if not isinstance(value, (list, tuple)):
        raise ValueError(f"{name} must be a sequence of xy pairs, got {type(value)!r}")
    return [_as_pair(item, f"{name}[{index}]") for index, item in enumerate(value)]


def flatten_config(config: dict[str, Any]) -> dict[str, Any]:
    """把 YAML 配置转换为 runner 使用的扁平参数。"""
    flat: dict[str, Any] = {}

    model = config.get("model", {})
    target = config.get("target_trajectory", {})
    initial_state = config.get("initial_state", {})
    simulation = config.get("simulation", {})
    mpc = config.get("mpc", {})
    cost = config.get("cost", {})
    outputs = config.get("outputs", {})
    viewer = config.get("viewer", {})

    if "path" in model:
        flat["model_path"] = resolve_project_path(model["path"])
    if "end_effector_site" in model:
        flat["end_effector_site"] = model["end_effector_site"]
    if "type" in target:
        flat["target_type"] = target["type"]
    if "center" in target:
        flat["target_center"] = _as_pair(target["center"], "target_trajectory.center")
    if "radius" in target:
        flat["target_radius"] = float(target["radius"])
    if "x_amplitude" in target:
        flat["target_x_amplitude"] = float(target["x_amplitude"])
    if "y_amplitude" in target:
        flat["target_y_amplitude"] = float(target["y_amplitude"])
    if "angular_speed" in target:
        flat["target_angular_speed"] = float(target["angular_speed"])
    if "fixed_target" in target:
        flat["fixed_target"] = _as_pair(target["fixed_target"], "target_trajectory.fixed_target")
    if "custom_trajectory" in target:
        flat["custom_trajectory"] = _as_pair_sequence(target["custom_trajectory"], "target_trajectory.custom_trajectory")
    if "q" in initial_state:
        flat["initial_q"] = _as_pair(initial_state["q"], "initial_state.q")
    if "dq" in initial_state:
        flat["initial_dq"] = _as_pair(initial_state["dq"], "initial_state.dq")
    if "dt" in simulation:
        flat["dt"] = float(simulation["dt"])
    if "num_steps" in simulation:
        flat["num_steps"] = int(simulation["num_steps"])
    if "horizon" in mpc:
        flat["horizon"] = int(mpc["horizon"])
    if "num_candidates" in mpc:
        flat["num_candidates"] = int(mpc["num_candidates"])
    if "torque_limit" in mpc:
        flat["torque_limit"] = float(mpc["torque_limit"])
    for key in ("ee_weight", "dq_weight", "torque_weight", "terminal_weight"):
        if key in cost:
            flat[key] = float(cost[key])
    for key in ("export_video", "save_figures", "save_metrics", "render_mode"):
        if key in outputs:
            flat[key] = outputs[key]
    if "root" in outputs:
        flat["output_root"] = resolve_project_path(outputs["root"])
    if "run_id" in outputs:
        flat["run_id"] = outputs["run_id"]
    if "show" in viewer:
        flat["show_viewer"] = bool(viewer["show"])
    if "real_time" in viewer:
        flat["real_time"] = bool(viewer["real_time"])

    for key in DEFAULT_RUN_CONFIG:
        if key in config:
            value = config[key]
            flat[key] = resolve_project_path(value) if key in ("model_path", "output_root") else value

    return flat


def load_config(config_path: Path) -> dict[str, Any]:
    """读取 B02 YAML 配置。"""
    if not config_path.exists():
        raise FileNotFoundError(f"Cannot find B02 config: {config_path}")
    raw_config = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw_config, dict):
        raise ValueError(f"B02 config must be a YAML mapping: {config_path}")
    return flatten_config(raw_config)


def resolve_run_args(cli_args: argparse.Namespace) -> argparse.Namespace:
    """合并默认值、配置文件和命令行覆盖参数。"""
    resolved = deepcopy(DEFAULT_RUN_CONFIG)
    resolved.update(load_config(cli_args.config))

    for key in DEFAULT_RUN_CONFIG:
        if hasattr(cli_args, key):
            value = getattr(cli_args, key)
            if value is not None:
                resolved[key] = value

    resolved["model_path"] = Path(resolved["model_path"])
    resolved["output_root"] = Path(resolved["output_root"])
    resolved["target_center"] = _as_pair(resolved["target_center"], "target_center")
    resolved["fixed_target"] = _as_pair(resolved["fixed_target"], "fixed_target")
    resolved["initial_q"] = _as_pair(resolved["initial_q"], "initial_q")
    resolved["initial_dq"] = _as_pair(resolved["initial_dq"], "initial_dq")
    if resolved["custom_trajectory"] is not None:
        resolved["custom_trajectory"] = _as_pair_sequence(resolved["custom_trajectory"], "custom_trajectory")

    if resolved["target_type"] not in ("fixed", "circle", "figure8", "sinusoidal", "custom"):
        raise ValueError(f"unsupported target_type: {resolved['target_type']}")
    if resolved["target_type"] == "custom" and not resolved["custom_trajectory"]:
        raise ValueError("custom target_type requires custom_trajectory")
    if resolved["render_mode"] not in ("overlay", "scene", "hybrid"):
        raise ValueError(f"unsupported render_mode: {resolved['render_mode']}")
    if resolved["target_radius"] < 0.0:
        raise ValueError(f"target_radius must be non-negative, got {resolved['target_radius']}")
    if resolved["target_x_amplitude"] < 0.0:
        raise ValueError(f"target_x_amplitude must be non-negative, got {resolved['target_x_amplitude']}")
    if resolved["target_y_amplitude"] < 0.0:
        raise ValueError(f"target_y_amplitude must be non-negative, got {resolved['target_y_amplitude']}")
    if resolved["run_id"] is None:
        resolved["run_id"] = datetime.now().strftime("%Y%m%d_%H%M%S")

    resolved["task_name"] = TASK_NAME
    resolved["run_dir"] = resolved["output_root"] / TASK_NAME / str(resolved["run_id"])
    resolved["config"] = cli_args.config
    return argparse.Namespace(**resolved)


def build_run_dir(run_id: str | None) -> Path:
    """生成 B02 本次运行目录。"""
    actual_run_id = run_id or datetime.now().strftime("%Y%m%d_%H%M%S")
    return OUTPUT_ROOT / "runs" / TASK_NAME / actual_run_id


def build_run_outputs(run_dir: Path) -> dict[str, Path]:
    """生成 B02 输出文件路径。"""
    return {key: run_dir / subdir / filename for key, (subdir, filename) in B02_OUTPUT_FILENAMES.items()}


def ensure_output_dirs(outputs: dict[str, Path]) -> None:
    """创建输出文件所在目录。"""
    for output_path in outputs.values():
        output_path.parent.mkdir(parents=True, exist_ok=True)


def _sample_custom_target(
    current_time: float,
    dt: float,
    step_offset: int,
    custom_trajectory: list[tuple[float, float]],
) -> tuple[float, float]:
    """按仿真时刻把自定义轨迹映射到目标点。"""
    if not custom_trajectory:
        raise ValueError("custom_trajectory must not be empty")
    target_time = current_time + (step_offset + 1) * dt
    index = min(len(custom_trajectory) - 1, max(0, int(round(target_time / dt)) - 1))
    return custom_trajectory[index]


def build_target_sequence(
    target_type: str,
    current_time: float,
    horizon: int,
    dt: float,
    center: tuple[float, float],
    radius: float,
    angular_speed: float,
    x_amplitude: float | None = None,
    y_amplitude: float | None = None,
    fixed_target: tuple[float, float] | None = None,
    custom_trajectory: list[tuple[float, float]] | None = None,
) -> list[tuple[float, float]]:
    """生成 MPC horizon 内的目标轨迹。

    支持：
    - `fixed`
    - `circle`
    - `figure8`
    - `sinusoidal`
    - `custom`
    """
    cx, cy = center
    sinusoid_x = radius if x_amplitude is None else x_amplitude
    sinusoid_y = radius if y_amplitude is None else y_amplitude
    figure8_x = radius if x_amplitude is None else x_amplitude
    figure8_y = radius if y_amplitude is None else y_amplitude
    fixed_xy = center if fixed_target is None else fixed_target

    targets: list[tuple[float, float]] = []
    for step_offset in range(horizon):
        target_time = current_time + (step_offset + 1) * dt
        phase = angular_speed * target_time

        if target_type == "fixed":
            targets.append((float(fixed_xy[0]), float(fixed_xy[1])))
        elif target_type == "circle":
            targets.append((cx + radius * math.cos(phase), cy + radius * math.sin(phase)))
        elif target_type == "figure8":
            targets.append(
                (
                    cx + figure8_x * math.sin(phase),
                    cy + figure8_y * math.sin(2.0 * phase),
                )
            )
        elif target_type == "sinusoidal":
            targets.append(
                (
                    cx + sinusoid_x * math.sin(phase),
                    cy + sinusoid_y * math.cos(phase),
                )
            )
        elif target_type == "custom":
            if custom_trajectory is None:
                raise ValueError("custom target_type requires custom_trajectory")
            targets.append(_sample_custom_target(current_time, dt, step_offset, custom_trajectory))
        else:
            raise ValueError(f"unsupported target_type: {target_type}")

    return targets


def add_import_roots() -> None:
    """让脚本直接运行时也能导入 simulator 子模块。"""
    if str(SIMULATOR_ROOT) not in sys.path:
        sys.path.insert(0, str(SIMULATOR_ROOT))


def import_b02_components() -> tuple[type[Any], type[Any], type[Any]]:
    """延迟导入 B02 组件，方便测试只覆盖纯函数。"""
    add_import_roots()
    from controllers.two_link_mpc_controller import TwoLinkMPCController
    from envs.two_link_env import TwoLinkEnv
    from planners.two_link_predictive_sampling import TwoLinkPredictiveSamplingPlanner

    return TwoLinkEnv, TwoLinkPredictiveSamplingPlanner, TwoLinkMPCController


def _distance_xy(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def get_scalar_actuator_torque_limit(env: Any) -> float:
    """读取环境里 actuator 的统一安全标量 torque 上限。"""
    ctrlrange = getattr(env.model, "actuator_ctrlrange", None)
    if ctrlrange is None:
        raise AttributeError("env.model is missing actuator_ctrlrange")

    actuator_limits = [min(abs(float(ctrl_min)), abs(float(ctrl_max))) for ctrl_min, ctrl_max in ctrlrange]
    if not actuator_limits:
        raise ValueError("No actuator limits found in environment model")
    return min(actuator_limits)


def validate_planner_torque_limit(args: argparse.Namespace, env: Any) -> None:
    """确保 planner 的 torque_limit 不超过真实 actuator 上限。"""
    actuator_limit = get_scalar_actuator_torque_limit(env)
    if float(args.torque_limit) > actuator_limit + 1e-9:
        raise ValueError(
            "torque_limit is larger than actuator ctrlrange: "
            f"planner torque_limit={args.torque_limit}, actuator_limit={actuator_limit}"
        )


def compute_summary_metrics(
    ee_history: list[tuple[float, float]],
    target_history: list[tuple[float, float]],
    commanded_torque_history: list[tuple[float, float]],
    applied_torque_history: list[tuple[float, float]],
    runtime_history: list[float],
) -> dict[str, float]:
    """计算 B02 最小验收指标。"""
    if not ee_history:
        raise ValueError("ee_history must not be empty")
    if len(ee_history) != len(target_history):
        raise ValueError("ee_history and target_history must have the same length")
    if len(commanded_torque_history) != len(applied_torque_history):
        raise ValueError("commanded_torque_history and applied_torque_history must have the same length")

    errors = [_distance_xy(ee, target) for ee, target in zip(ee_history, target_history)]
    max_abs_command_torque = max((max(abs(t1), abs(t2)) for t1, t2 in commanded_torque_history), default=0.0)
    max_abs_applied_torque = max((max(abs(t1), abs(t2)) for t1, t2 in applied_torque_history), default=0.0)
    mean_runtime = sum(runtime_history) / len(runtime_history) if runtime_history else 0.0

    return {
        "final_ee_error": errors[-1],
        "mean_ee_error": sum(errors) / len(errors),
        "max_ee_error": max(errors),
        "max_abs_torque": max_abs_applied_torque,
        "max_abs_applied_torque": max_abs_applied_torque,
        "max_abs_command_torque": max_abs_command_torque,
        "runtime_per_control_step": mean_runtime,
    }


def save_metrics_csv(metrics: dict[str, float], output_path: Path) -> None:
    """保存 B02 metrics CSV。"""
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
    """保存一次 B02 运行摘要。"""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "B02 two-link MPC tracking run log",
        "",
        f"config: {getattr(args, 'config', None)}",
        f"task_name: {args.task_name}",
        f"run_id: {args.run_id}",
        f"run_dir: {args.run_dir}",
        f"model_path: {args.model_path}",
        f"end_effector_site: {args.end_effector_site}",
        f"target_type: {args.target_type}",
        f"target_center: {args.target_center}",
        f"target_radius: {args.target_radius}",
        f"target_x_amplitude: {args.target_x_amplitude}",
        f"target_y_amplitude: {args.target_y_amplitude}",
        f"target_angular_speed: {args.target_angular_speed}",
        f"fixed_target: {args.fixed_target}",
        f"custom_trajectory_points: {0 if args.custom_trajectory is None else len(args.custom_trajectory)}",
        f"render_mode: {args.render_mode}",
        f"initial_q: {args.initial_q}",
        f"initial_dq: {args.initial_dq}",
        f"dt: {args.dt}",
        f"horizon: {args.horizon}",
        f"num_candidates: {args.num_candidates}",
        f"num_steps: {args.num_steps}",
        f"torque_limit: {args.torque_limit}",
        f"ee_weight: {args.ee_weight}",
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
    ee_history: list[tuple[float, float]],
    target_history: list[tuple[float, float]],
    time_history: list[float],
    applied_torque_history: list[tuple[float, float]],
    outputs: dict[str, Path],
) -> None:
    """兼容旧接口，内部转调到新的 plotting 模块。"""
    tracking_rows = []
    for index, (time_value, target_xy, actual_xy, control) in enumerate(
        zip(time_history, target_history, ee_history, applied_torque_history)
    ):
        tracking_rows.append(
            {
                "step": index,
                "time": time_value,
                "target_x": target_xy[0],
                "target_y": target_xy[1],
                "actual_x": actual_xy[0],
                "actual_y": actual_xy[1],
                "error_norm": _distance_xy(actual_xy, target_xy),
                "u1": control[0],
                "u2": control[1],
                "mpc_cost": 0.0,
            }
        )
    save_b02_tracking_figures(tracking_rows=tracking_rows, outputs=outputs)


def save_video(frames: list[Any], output_path: Path, fps: int) -> None:
    """保存 MuJoCo 视频为 mp4。"""
    if not frames:
        raise ValueError("frames must not be empty")
    try:
        import imageio.v2 as imageio
    except ImportError as exc:
        raise RuntimeError("Exporting mp4 requires imageio") from exc

    output_path.parent.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(output_path, frames, fps=fps)


def launch_realtime_viewer(show_viewer: bool, env: Any) -> Any | None:
    """按需创建 MuJoCo 实时 viewer。"""
    if not show_viewer:
        return None
    try:
        import mujoco.viewer as mujoco_viewer
    except Exception as exc:
        raise RuntimeError("Current Python environment does not support mujoco.viewer") from exc

    return mujoco_viewer.launch_passive(env.model, env.data)


def _build_tracking_rows(tracking_logger: TrackingLogBuffer) -> list[dict[str, float]]:
    """把 logger rows 转成后处理模块消费的字典列表。"""
    return [
        {
            "step": row.step,
            "time": row.time,
            "q1": row.q1,
            "q2": row.q2,
            "dq1": row.dq1,
            "dq2": row.dq2,
            "target_x": row.target_x,
            "target_y": row.target_y,
            "actual_x": row.actual_x,
            "actual_y": row.actual_y,
            "error_norm": row.error_norm,
            "u1": row.u1,
            "u2": row.u2,
            "mpc_cost": row.mpc_cost,
        }
        for row in tracking_logger.to_rows()
    ]


def _build_scene_callbacks(args: argparse.Namespace, env: Any) -> tuple[Any | None, Any | None]:
    """为 scene/hybrid 渲染构造回调。

    输入：
    - `args.render_mode`
    - 当前环境 `env`

    输出：
    - `scene_marker_provider(step_index, tracking_rows)`
    - `scene_frame_renderer(step_index, marker_payload)`

    验证标准：
    - `overlay` 模式下返回 `(None, None)`
    - `scene/hybrid` 模式下两个回调都非空
    """
    if args.render_mode == "overlay":
        return None, None

    def scene_marker_provider(step_index: int, tracking_rows: list[dict[str, float]]) -> Any:
        return build_scene_marker_payload(step_index=step_index, tracking_rows=tracking_rows)

    def scene_frame_renderer(step_index: int, marker_payload: Any) -> Any:
        _ = step_index
        if marker_payload is None:
            return env.render_frame()
        scene_geoms = [
            {
                "geom_type": geom.geom_type,
                "pos": geom.pos,
                "size": geom.size,
                "rgba": geom.rgba,
                "from_pos": geom.from_pos,
                "to_pos": geom.to_pos,
            }
            for geom in marker_payload.geoms
        ]
        return env.render_frame_with_scene_geoms(scene_geoms)

    return scene_marker_provider, scene_frame_renderer


def run_closed_loop(args: argparse.Namespace) -> dict[str, Any]:
    """执行 B02 task-space MPC 闭环并保存请求的输出。"""
    if not args.model_path.exists():
        raise FileNotFoundError(f"Cannot find B02 MuJoCo XML: {args.model_path}")

    outputs = build_run_outputs(args.run_dir)
    ensure_output_dirs(outputs)
    TwoLinkEnv, TwoLinkPredictiveSamplingPlanner, TwoLinkMPCController = import_b02_components()

    env = TwoLinkEnv(
        model_path=str(args.model_path),
        dt=args.dt,
        end_effector_site=args.end_effector_site,
    )
    validate_planner_torque_limit(args, env)
    planner = TwoLinkPredictiveSamplingPlanner(
        horizon=args.horizon,
        num_candidates=args.num_candidates,
        torque_limit=args.torque_limit,
        ee_weight=args.ee_weight,
        dq_weight=args.dq_weight,
        torque_weight=args.torque_weight,
        terminal_weight=args.terminal_weight,
    )
    controller = TwoLinkMPCController(planner=planner)

    state = env.reset(q=args.initial_q, dq=args.initial_dq)

    time_history: list[float] = []
    target_history: list[tuple[float, float]] = []
    ee_history: list[tuple[float, float]] = []
    state_history: list[tuple[float, float, float, float]] = []
    commanded_torque_history: list[tuple[float, float]] = []
    applied_torque_history: list[tuple[float, float]] = []
    best_cost_history: list[float] = []
    runtime_history: list[float] = []
    frames: list[Any] = []
    tracking_logger = TrackingLogBuffer()
    viewer = launch_realtime_viewer(args.show_viewer, env)

    if viewer is not None:
        viewer.sync()

    for step_idx in range(args.num_steps):
        step_wall_start = time.perf_counter()
        if viewer is not None and not viewer.is_running():
            break

        current_time = step_idx * args.dt
        horizon_targets = build_target_sequence(
            target_type=args.target_type,
            current_time=current_time,
            horizon=args.horizon,
            dt=args.dt,
            center=args.target_center,
            radius=args.target_radius,
            angular_speed=args.target_angular_speed,
            x_amplitude=args.target_x_amplitude,
            y_amplitude=args.target_y_amplitude,
            fixed_target=args.fixed_target,
            custom_trajectory=args.custom_trajectory,
        )

        planner_start = time.perf_counter()
        torque = controller.compute_control(env, state, target_sequence=horizon_targets)
        runtime = time.perf_counter() - planner_start

        state = env.step(torque)
        applied_torque = env.get_last_applied_torque()
        ee_position = env.get_end_effector_position()
        immediate_target = horizon_targets[0]

        if controller.last_plan is None:
            raise RuntimeError("controller.last_plan is unexpectedly None")

        record_time = current_time + args.dt
        time_history.append(record_time)
        target_history.append(immediate_target)
        ee_history.append(ee_position)
        state_history.append(state)
        commanded_torque_history.append(torque)
        applied_torque_history.append(applied_torque)
        runtime_history.append(runtime)
        best_cost_history.append(float(controller.last_plan["best_cost"]))
        tracking_logger.append_step(
            step=step_idx,
            time=record_time,
            state=state,
            target_xy=immediate_target,
            actual_xy=ee_position,
            control=applied_torque,
            mpc_cost=float(controller.last_plan["best_cost"]),
        )

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
        ee_history=ee_history,
        target_history=target_history,
        commanded_torque_history=commanded_torque_history,
        applied_torque_history=applied_torque_history,
        runtime_history=runtime_history,
    )

    save_run_log(args=args, metrics=metrics, output_path=outputs["log"], num_frames=len(frames))
    tracking_logger.save_csv(outputs["tracking_log"])
    if args.save_metrics:
        save_metrics_csv(metrics, outputs["metrics"])

    tracking_rows = _build_tracking_rows(tracking_logger)

    if args.save_figures:
        save_b02_tracking_figures(tracking_rows=tracking_rows, outputs=outputs)

    if args.export_video:
        fps = max(1, int(round(1.0 / args.dt)))
        save_video(frames, outputs["video"], fps=fps)
        if args.render_mode == "overlay":
            render_marked_video(
                raw_frames=frames,
                tracking_rows=tracking_rows,
                output_path=outputs["marked_video"],
                fps=fps,
            )
        else:
            scene_marker_provider, scene_frame_renderer = _build_scene_callbacks(args=args, env=env)
            render_marked_video(
                raw_frames=frames,
                tracking_rows=tracking_rows,
                output_path=outputs["marked_video"],
                fps=fps,
                render_mode=args.render_mode,
                scene_marker_provider=scene_marker_provider,
                scene_frame_renderer=scene_frame_renderer,
            )

    return {
        "metrics": metrics,
        "time_history": time_history,
        "target_history": target_history,
        "ee_history": ee_history,
        "state_history": state_history,
        "torque_history": applied_torque_history,
        "commanded_torque_history": commanded_torque_history,
        "applied_torque_history": applied_torque_history,
        "best_cost_history": best_cost_history,
        "runtime_history": runtime_history,
    }


def run_b02_skeleton(args: argparse.Namespace) -> dict[str, Any]:
    """兼容旧测试/旧调用名：现在直接执行 B02 closed loop。"""
    resolved_args = resolve_run_args(args) if not hasattr(args, "run_dir") else args
    result = run_closed_loop(resolved_args)
    return {
        "task_name": TASK_NAME,
        "run_dir": resolved_args.run_dir,
        "outputs": build_run_outputs(resolved_args.run_dir),
        "status": "B02 closed-loop MPC finished.",
        "result": result,
    }


def main(argv: list[str] | None = None) -> None:
    """B02 demo 主入口。"""
    parser = build_arg_parser()
    args = resolve_run_args(parser.parse_args(argv))
    result = run_closed_loop(args)

    print("B02 two-link MPC tracking demo finished.")
    for name, value in result["metrics"].items():
        print(f"{name}: {value}")
    print(f"run_dir: {args.run_dir}")
    print(f"log: {build_run_outputs(args.run_dir)['log']}")


if __name__ == "__main__":
    main()
