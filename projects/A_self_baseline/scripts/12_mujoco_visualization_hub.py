"""A12: MuJoCo visualization hub for mink examples and A_self_baseline.

学习目标：
- 用一个统一入口打开 A 项目 UR5e 模型 viewer。
- 用同一个入口转发运行 mink 的 UR5e examples。
- 把模型路径、keyframe、nq/nv/nu、site/body/actuator 摘要打印出来，便于复盘。

输入：
- `--mode inspect`: 只打印 A 项目模型摘要，不打开 viewer。
- `--mode view_model`: 打开 A 项目 UR5e MuJoCo viewer。
- `--mode mink_ur5e`: 运行上游 mink `arm_ur5e.py`。
- `--mode mink_ur5e_actuator`: 运行上游 mink `arm_ur5e_actuators.py`。

输出：
- inspect / view_model: 终端摘要；view_model 额外打开 MuJoCo viewer。
- mink_*: 转交给上游 example 运行。

数学逻辑是否变化：
- 不变化。本脚本只做可视化和入口分发，不改 IK、Jacobian、QP 或控制律。

可能风险：
- viewer 需要可用图形环境。
- mink example 需要 `mujoco_py311` 环境中安装 mujoco、qpsolvers[daqp]、loop-rate-limiters。
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import numpy as np
import yaml


A_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = A_ROOT.parents[1]
DEFAULT_CONFIG = A_ROOT / "configs" / "robot.yaml"
DEFAULT_A05_TRAJECTORY = A_ROOT / "outputs" / "trajectories" / "A05_qp_ik_q_traj.npy"
SHARED_UR5E_SCENE = REPO_ROOT / "shared" / "robot_assets" / "models" / "mink_universal_robots_ur5e" / "scene.xml"
MINK_UPSTREAM_ROOT = REPO_ROOT / "external" / "mink_upstream"
MINK_SRC_ROOT = MINK_UPSTREAM_ROOT / "src"
MINK_EXAMPLES_ROOT = MINK_UPSTREAM_ROOT / "examples"
OS_PATHSEP = os.pathsep


MINK_EXAMPLE_MODES = {
    "mink_ur5e": "arm_ur5e.py",
    "mink_ur5e_actuator": "arm_ur5e_actuators.py",
}


def resolve_path(path_str: str | Path, base_dir: Path) -> Path:
    """把配置或 CLI 中的路径解析成绝对路径。"""
    path = Path(path_str).expanduser()
    if path.is_absolute():
        return path.resolve()
    return (base_dir / path).resolve()


def load_yaml_config(config_path: str | Path) -> dict[str, Any]:
    """读取 YAML 配置文件，并要求顶层是 mapping。"""
    path = Path(config_path).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")

    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError(f"Config file must contain a YAML mapping: {path}")
    return config


def resolve_a_project_mjcf(config_path: str | Path, mjcf_override: str | Path | None) -> Path:
    """确定 A 项目 view_model / inspect 使用的 MJCF 路径。"""
    if mjcf_override is not None:
        mjcf_path = resolve_path(mjcf_override, A_ROOT)
        source = f"--mjcf {mjcf_override}"
    else:
        config = load_yaml_config(config_path)
        if "mjcf_path" not in config:
            raise KeyError(f"Missing 'mjcf_path' in config: {Path(config_path).resolve()}")
        mjcf_path = resolve_path(config["mjcf_path"], A_ROOT)
        source = f"mjcf_path in {Path(config_path).resolve()}"

    if not mjcf_path.exists():
        raise FileNotFoundError(
            "MJCF file not found.\n"
            f"Source: {source}\n"
            f"Resolved path: {mjcf_path}\n"
            f"Shared UR5e fallback exists: {SHARED_UR5E_SCENE.exists()} at {SHARED_UR5E_SCENE}"
        )
    return mjcf_path


def resolve_mink_example_script(mode: str) -> Path:
    """根据 mode 找到上游 mink example 脚本。"""
    if mode not in MINK_EXAMPLE_MODES:
        raise ValueError(f"Unsupported mink example mode: {mode}")
    script_path = MINK_EXAMPLES_ROOT / MINK_EXAMPLE_MODES[mode]
    if not script_path.exists():
        raise FileNotFoundError(f"Mink example script not found: {script_path}")
    return script_path


def build_mink_example_environment(environ: Mapping[str, str]) -> dict[str, str]:
    """构造运行上游 mink example 的环境变量。

    重点是把 `external/mink_upstream/src` 放到 PYTHONPATH 最前面。
    这样可以不安装 mink 包，也能让 `import mink` 找到本仓库里的上游源码。
    """
    new_env = dict(environ)
    old_pythonpath = new_env.get("PYTHONPATH")
    if old_pythonpath:
        new_env["PYTHONPATH"] = f"{MINK_SRC_ROOT}{OS_PATHSEP}{old_pythonpath}"
    else:
        new_env["PYTHONPATH"] = str(MINK_SRC_ROOT)
    return new_env


def load_model_and_data(mjcf_path: str | Path):
    """从 MJCF 创建 MuJoCo model/data。"""
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(Path(mjcf_path)))
    data = mujoco.MjData(model)
    return model, data


def find_keyframe_id(model: Any, keyframe_name: str) -> int | None:
    """按名称查找 MuJoCo keyframe id。"""
    import mujoco

    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, keyframe_name)
    if key_id < 0:
        return None
    return int(key_id)


def apply_keyframe(model: Any, data: Any, keyframe_name: str | None) -> bool:
    """如果 keyframe 存在，则把它写入当前 data。"""
    if keyframe_name is None:
        return False

    key_id = find_keyframe_id(model, keyframe_name)
    if key_id is None:
        return False

    data.qpos[:] = model.key_qpos[key_id]
    data.qvel[:] = 0.0
    return True


def validate_q_trajectory(trajectory: np.ndarray, expected_nq: int, source: str | Path) -> np.ndarray:
    """校验 q 轨迹是否能直接写入 MuJoCo `data.qpos`。

    输入:
    - trajectory: 期望 shape 为 `(T, nq)` 的 q 序列。
    - expected_nq: 当前模型的 `model.nq`。
    - source: 错误信息中显示的轨迹来源。

    输出:
    - float 类型的二维 ndarray。
    """
    trajectory = np.asarray(trajectory, dtype=float)
    if trajectory.ndim != 2 or trajectory.shape[1] != expected_nq:
        raise ValueError(
            f"Trajectory from {source} expected shape (T, {expected_nq}), "
            f"got {trajectory.shape}"
        )
    if trajectory.shape[0] == 0:
        raise ValueError(f"Trajectory from {source} is empty")
    if not np.all(np.isfinite(trajectory)):
        raise ValueError(f"Trajectory from {source} contains NaN or inf")
    return trajectory


def load_q_trajectory(path: str | Path, expected_nq: int) -> np.ndarray:
    """读取 `.npy` q 轨迹，并校验 shape 和数值有效性。"""
    trajectory_path = Path(path).expanduser().resolve()
    if not trajectory_path.exists():
        raise FileNotFoundError(f"Trajectory file not found: {trajectory_path}")
    trajectory = np.load(trajectory_path)
    return validate_q_trajectory(trajectory, expected_nq=expected_nq, source=trajectory_path)


def apply_q_frame(model: Any, data: Any, q: np.ndarray) -> None:
    """把一帧 q 写入 MuJoCo data 并刷新 forward kinematics。"""
    import mujoco

    if q.shape != data.qpos.shape:
        raise ValueError(f"q frame shape mismatch: expected {data.qpos.shape}, got {q.shape}")
    data.qpos[:] = q
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


def _names_from_model(model: Any, obj_type: Any, count: int) -> list[str]:
    """读取 MuJoCo 对象名称，跳过空名称。"""
    import mujoco

    names: list[str] = []
    for obj_id in range(count):
        name = mujoco.mj_id2name(model, obj_type, obj_id)
        if name:
            names.append(name)
    return names


def collect_model_summary(
    label: str,
    mjcf_path: Path,
    model: Any,
    data: Any,
    keyframe_name: str | None,
    keyframe_applied: bool,
) -> dict[str, Any]:
    """收集模型摘要，供 inspect 打印或测试断言。"""
    import mujoco

    return {
        "label": label,
        "mjcf_path": str(mjcf_path),
        "nq": int(model.nq),
        "nv": int(model.nv),
        "nu": int(model.nu),
        "nbody": int(model.nbody),
        "nsite": int(model.nsite),
        "nkey": int(model.nkey),
        "keyframe_name": keyframe_name,
        "keyframe_applied": bool(keyframe_applied),
        "qpos_shape": tuple(data.qpos.shape),
        "body_names": _names_from_model(model, mujoco.mjtObj.mjOBJ_BODY, model.nbody),
        "site_names": _names_from_model(model, mujoco.mjtObj.mjOBJ_SITE, model.nsite),
        "actuator_names": _names_from_model(model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu),
    }


def print_summary(summary: Mapping[str, Any]) -> None:
    """打印面向学习复盘的模型摘要。"""
    print("[A12] MuJoCo visualization hub")
    print(f"[A12] label: {summary['label']}")
    print(f"[A12] mjcf_path: {summary['mjcf_path']}")
    print(f"[A12] nq={summary['nq']}, nv={summary['nv']}, nu={summary['nu']}")
    print(f"[A12] nbody={summary['nbody']}, nsite={summary['nsite']}, nkey={summary['nkey']}")
    print(
        f"[A12] keyframe: {summary['keyframe_name'] or '<none>'}, "
        f"applied={summary['keyframe_applied']}"
    )
    print(f"[A12] first bodies: {summary['body_names'][:8]}")
    print(f"[A12] first sites: {summary['site_names'][:8]}")
    print(f"[A12] actuators: {summary['actuator_names']}")


def run_viewer(model: Any, data: Any, duration: float, show_ui: bool) -> None:
    """打开 MuJoCo passive viewer。

    这个函数只推进 MuJoCo 默认仿真，不做 IK 或控制器。
    """
    import mujoco
    import mujoco.viewer

    with mujoco.viewer.launch_passive(
        model=model,
        data=data,
        show_left_ui=show_ui,
        show_right_ui=show_ui,
    ) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        start_time = time.monotonic()
        while viewer.is_running():
            if duration > 0.0 and time.monotonic() - start_time >= duration:
                break
            mujoco.mj_step(model, data)
            mujoco.mj_camlight(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)


def replay_q_trajectory(
    model: Any,
    data: Any,
    trajectory: np.ndarray,
    fps: float,
    loop: bool,
    duration: float,
    show_ui: bool,
) -> None:
    """在 MuJoCo viewer 中回放 q 轨迹。

    数学逻辑不变:
    - 这里不求 IK、不算 Jacobian、不改变控制器。
    - 每一帧只是把已有 q 轨迹写入 `data.qpos`，再刷新 viewer。
    """
    import mujoco
    import mujoco.viewer

    if fps <= 0.0:
        raise ValueError(f"fps must be positive, got {fps}")

    frame_dt = 1.0 / fps
    with mujoco.viewer.launch_passive(
        model=model,
        data=data,
        show_left_ui=show_ui,
        show_right_ui=show_ui,
    ) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        start_time = time.monotonic()
        frame_id = 0
        while viewer.is_running():
            if duration > 0.0 and time.monotonic() - start_time >= duration:
                break

            apply_q_frame(model, data, trajectory[frame_id])
            mujoco.mj_camlight(model, data)
            viewer.sync()
            time.sleep(frame_dt)

            frame_id += 1
            if frame_id >= len(trajectory):
                if not loop:
                    break
                frame_id = 0


def run_a_project_mode(args: argparse.Namespace) -> None:
    """运行 A 项目 inspect / view_model 模式。"""
    keyframe_name = args.keyframe if args.keyframe else None
    mjcf_path = resolve_a_project_mjcf(args.config, args.mjcf)
    model, data = load_model_and_data(mjcf_path)
    keyframe_applied = apply_keyframe(model, data, keyframe_name)

    summary = collect_model_summary(
        label=args.mode,
        mjcf_path=mjcf_path,
        model=model,
        data=data,
        keyframe_name=keyframe_name,
        keyframe_applied=keyframe_applied,
    )
    print_summary(summary)

    if keyframe_name is not None and not keyframe_applied:
        print(f"[A12] warning: keyframe '{keyframe_name}' not found; using default qpos.")

    if args.mode == "inspect":
        return

    run_viewer(model=model, data=data, duration=args.duration, show_ui=args.show_ui)


def run_replay_a05_mode(args: argparse.Namespace) -> None:
    """回放 A05 QP-IK 输出的 q 轨迹。"""
    mjcf_path = resolve_a_project_mjcf(args.config, args.mjcf)
    model, data = load_model_and_data(mjcf_path)
    trajectory_path = resolve_path(args.trajectory, A_ROOT)
    trajectory = load_q_trajectory(trajectory_path, expected_nq=model.nq)

    apply_q_frame(model, data, trajectory[0])
    summary = collect_model_summary(
        label=args.mode,
        mjcf_path=mjcf_path,
        model=model,
        data=data,
        keyframe_name=None,
        keyframe_applied=False,
    )
    print_summary(summary)
    print(f"[A12] trajectory: {trajectory_path}")
    print(f"[A12] trajectory_shape: {trajectory.shape}")
    print(f"[A12] replay_fps: {args.fps}")
    print(f"[A12] replay_loop: {args.loop}")

    if args.inspect_only:
        print("[A12] inspect-only enabled; replay viewer will not open.")
        return

    replay_q_trajectory(
        model=model,
        data=data,
        trajectory=trajectory,
        fps=args.fps,
        loop=args.loop,
        duration=args.duration,
        show_ui=args.show_ui,
    )


def run_mink_example_mode(args: argparse.Namespace) -> int:
    """转发运行上游 mink example。"""
    script_path = resolve_mink_example_script(args.mode)
    env = build_mink_example_environment(os.environ)
    command = [sys.executable, str(script_path)]

    print("[A12] Running mink example")
    print(f"[A12] mode: {args.mode}")
    print(f"[A12] script: {script_path}")
    print(f"[A12] python: {sys.executable}")
    print(f"[A12] PYTHONPATH startswith: {MINK_SRC_ROOT}")

    completed = subprocess.run(command, cwd=str(script_path.parent), env=env, check=False)
    return int(completed.returncode)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Unified MuJoCo visualization entry for A_self_baseline and mink UR5e examples."
    )
    parser.add_argument(
        "--mode",
        choices=["inspect", "view_model", "replay_a05", "mink_ur5e", "mink_ur5e_actuator"],
        default="inspect",
        help="可视化模式。inspect 不打开 viewer；view_model 打开模型；replay_a05 回放 A05 轨迹；mink_* 转发上游示例。",
    )
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG),
        help="A 项目 robot.yaml 路径，仅 inspect/view_model 使用。",
    )
    parser.add_argument(
        "--mjcf",
        default=None,
        help="可选：覆盖 robot.yaml 中的 mjcf_path，仅 inspect/view_model 使用。",
    )
    parser.add_argument(
        "--keyframe",
        default="home",
        help="可选：inspect/view_model 启动时应用的 keyframe；空字符串表示不应用。",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="view_model 运行秒数；0 表示直到手动关闭窗口。",
    )
    parser.add_argument(
        "--show-ui",
        action="store_true",
        help="view_model 是否显示 MuJoCo viewer 左右 UI 面板。",
    )
    parser.add_argument(
        "--trajectory",
        default=str(DEFAULT_A05_TRAJECTORY),
        help="replay_a05 使用的 q 轨迹 .npy 文件。",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=60.0,
        help="replay_a05 的回放帧率。",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="replay_a05 播完后是否循环播放。",
    )
    parser.add_argument(
        "--inspect-only",
        action="store_true",
        help="replay_a05 只加载并打印轨迹信息，不打开 viewer。",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.mode in MINK_EXAMPLE_MODES:
        return run_mink_example_mode(args)
    if args.mode == "replay_a05":
        run_replay_a05_mode(args)
        return 0

    run_a_project_mode(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
