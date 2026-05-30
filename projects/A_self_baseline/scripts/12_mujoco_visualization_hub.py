"""A12: MuJoCo visualization hub for A_self_baseline.

当前 pipeline 定位：
- A12 是一个轻量入口，用来统一 A 项目 MuJoCo 模型检查、mink UR5e 示例入口、
  以及后续 A05 q trajectory replay 的路径和非图形逻辑。
- 本文件的测试只覆盖不打开窗口的部分；真正 viewer 仍需要本机图形环境。

学习目标：
- 学会从 `configs/robot.yaml` 稳定定位 A 项目 MJCF。
- 学会把外部/参考 example 的 Python 子进程环境准备清楚。
- 学会读取和验证 q trajectory，并把某一帧 q 写入 MuJoCo `data.qpos` 后
  调用 `mujoco.mj_forward` 更新派生状态。

输入：
- `projects/A_self_baseline/configs/robot.yaml` 中的 `mjcf_path`。
- `projects/A_self_baseline/external/mink/examples/arm_ur5e*.py`。
- 默认 A05 q trajectory 路径；若尚未生成，则使用最小占位轨迹做 hub smoke。

输出：
- headless inspect 模式下的模型摘要 dict / JSON。
- replay 模式下写入 MuJoCo data 的 q frame。

数学逻辑是否变化：
- 不改变 IK、QP、Jacobian 或控制数学。
- 本脚本只负责模型加载、路径解析、轨迹 shape 校验和单帧状态应用。
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import numpy as np
import yaml


# =============================
# 路径常量
# =============================
# 这一块要实现什么：
# - 使用 __file__ 定位 A 项目根目录、仓库根目录和默认配置路径。
#
# 为什么需要：
# - 本仓库把“路径设置”作为正式学习内容；脚本不能依赖 shell 当前工作目录。
#
# 输入：
# - 当前脚本路径。
#
# 输出：
# - A_ROOT / REPO_ROOT / DEFAULT_CONFIG 等稳定路径。
A_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = A_ROOT.parents[1]
DEFAULT_CONFIG = A_ROOT / "configs" / "robot.yaml"
MINK_ROOT = A_ROOT / "external" / "mink"
MINK_EXAMPLES_ROOT = MINK_ROOT / "examples"
MINK_SRC_ROOT = MINK_ROOT / "src"
OS_PATHSEP = os.pathsep

# A05 当前仍是 TODO skeleton。这里的默认轨迹文件如果不存在，hub 只使用
# 内存中的最小零位姿占位轨迹用于 replay smoke，不写入文件，避免制造假产物。
DEFAULT_A05_TRAJECTORY = A_ROOT / "outputs" / "trajectories" / "A05_qp_ik_q_traj.npy"


def resolve_path(path_value: str | Path, base_dir: Path) -> Path:
    """把配置或 CLI 中的路径解析为绝对路径。

    输入：
    - `path_value`：YAML 或命令行给出的路径。
    - `base_dir`：相对路径的解析基准。

    输出：
    - 解析后的绝对路径。
    """
    path = Path(path_value).expanduser()
    if path.is_absolute():
        return path.resolve()
    return (base_dir / path).resolve()


def load_yaml_config(config_path: str | Path) -> dict[str, Any]:
    """读取 A 项目 YAML 配置。

    为什么需要：
    - A 项目把模型入口集中放在 `configs/robot.yaml`。
    - A12 不应该重新写死 scene.xml 路径。
    """
    path = Path(config_path).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")

    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError(f"Config file must contain a YAML mapping: {path}")
    return config


def resolve_a_project_mjcf(config_path: str | Path, mjcf_override: str | Path | None) -> Path:
    """确定 A 项目本次要加载的 MJCF。

    查找逻辑：
    1. 如果 CLI 传入 `--mjcf`，优先使用它；
    2. 否则读取 `robot.yaml` 中的 `mjcf_path`；
    3. 相对路径按 A 项目根目录解析。

    输出：
    - 已存在的 MJCF 文件路径。
    """
    if mjcf_override is not None:
        mjcf_path = resolve_path(mjcf_override, A_ROOT)
        source = f"--mjcf {mjcf_override}"
    else:
        config = load_yaml_config(config_path)
        if "mjcf_path" not in config:
            raise KeyError(f"Missing 'mjcf_path' in config: {Path(config_path).resolve()}")
        mjcf_path = resolve_path(config["mjcf_path"], A_ROOT)
        source = f"config mjcf_path in {Path(config_path).resolve()}"

    if not mjcf_path.exists():
        raise FileNotFoundError(
            "MJCF file not found.\n"
            f"Source: {source}\n"
            f"Resolved path: {mjcf_path}"
        )
    return mjcf_path


def resolve_mink_example_script(mode: str) -> Path:
    """根据 hub mode 定位 mink UR5e 示例脚本。

    输入：
    - `mode`：`mink_ur5e` 或 `mink_ur5e_actuator`。

    输出：
    - 对应 example Python 文件路径。
    """
    script_map = {
        "mink_ur5e": MINK_EXAMPLES_ROOT / "arm_ur5e.py",
        "mink_ur5e_actuator": MINK_EXAMPLES_ROOT / "arm_ur5e_actuators.py",
    }
    if mode not in script_map:
        raise ValueError(f"Unsupported mink example mode: {mode}")

    script_path = script_map[mode]
    if not script_path.exists():
        raise FileNotFoundError(f"mink example script not found: {script_path}")
    return script_path


def build_mink_example_environment(environ: Mapping[str, str]) -> dict[str, str]:
    """构造运行 mink example 的子进程环境。

    要实现什么：
    - 把本地 mink src 路径放到 PYTHONPATH 最前面。
    - 保留用户已有 PYTHONPATH 和其他环境变量。

    为什么需要：
    - 如果后续导入完整 mink 源码，子进程应优先使用 A 项目本地参考副本。
    - 当前仓库只导入了 example 文件时，这个环境仍然是清晰的占位契约。
    """
    new_env = dict(environ)
    existing_pythonpath = new_env.get("PYTHONPATH", "")
    parts = [str(MINK_SRC_ROOT)]
    if existing_pythonpath:
        parts.append(existing_pythonpath)
    new_env["PYTHONPATH"] = OS_PATHSEP.join(parts)
    return new_env


def load_model_and_data(mjcf_path: str | Path):
    """从 MJCF 创建 MuJoCo model/data。

    输入：
    - `mjcf_path`：A 项目 UR5e scene.xml。

    输出：
    - `model`：MuJoCo `MjModel`，包含 nq/nv/nu、site/body 等结构信息。
    - `data`：MuJoCo `MjData`，包含当前 qpos/qvel/site_xpos 等状态缓存。
    """
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(Path(mjcf_path)))
    data = mujoco.MjData(model)
    return model, data


def _mujoco_names(model: Any, obj_type: Any, count: int) -> list[str]:
    """按 MuJoCo 对象类型枚举名称。"""
    import mujoco

    names: list[str] = []
    for obj_id in range(count):
        name = mujoco.mj_id2name(model, obj_type, obj_id)
        if name is not None:
            names.append(name)
    return names


def collect_model_summary(
    *,
    label: str,
    mjcf_path: str | Path,
    model: Any,
    data: Any,
    keyframe_name: str | None,
    keyframe_applied: bool,
) -> dict[str, Any]:
    """收集 A12 headless inspect 摘要。

    输入：
    - label / mjcf_path：本次检查对象。
    - model/data：MuJoCo 模型和状态。
    - keyframe_name/keyframe_applied：是否应用了启动 keyframe。

    输出：
    - dict，可打印或写成 JSON，包含 nq/nv/nu 和模型对象名称。
    """
    import mujoco

    return {
        "label": label,
        "mjcf_path": str(Path(mjcf_path)),
        "nq": int(model.nq),
        "nv": int(model.nv),
        "nu": int(model.nu),
        "time": float(data.time),
        "keyframe_name": keyframe_name,
        "keyframe_applied": bool(keyframe_applied),
        "joint_names": _mujoco_names(model, mujoco.mjtObj.mjOBJ_JOINT, model.njnt),
        "body_names": _mujoco_names(model, mujoco.mjtObj.mjOBJ_BODY, model.nbody),
        "site_names": _mujoco_names(model, mujoco.mjtObj.mjOBJ_SITE, model.nsite),
        "actuator_names": _mujoco_names(model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu),
        "keyframe_names": _mujoco_names(model, mujoco.mjtObj.mjOBJ_KEY, model.nkey),
    }


def _placeholder_q_trajectory(expected_nq: int) -> np.ndarray:
    """返回 A12 hub smoke 用的内存占位 q trajectory。"""
    return np.zeros((1, expected_nq), dtype=float)


def validate_q_trajectory(q_trajectory: np.ndarray, expected_nq: int, source: str) -> None:
    """验证 q trajectory 是否是 `(T, nq)`。

    输入：
    - q_trajectory：待 replay 的关节轨迹。
    - expected_nq：MuJoCo model.nq。
    - source：错误信息中显示的来源路径或标签。

    输出：
    - 无返回；shape 不合格时抛出清晰 ValueError。
    """
    if q_trajectory.ndim != 2 or q_trajectory.shape[1] != expected_nq:
        raise ValueError(
            f"{source} expected shape (T, {expected_nq}), got {q_trajectory.shape}"
        )
    if q_trajectory.shape[0] == 0:
        raise ValueError(f"{source} must contain at least one q frame")
    if not np.isfinite(q_trajectory).all():
        raise ValueError(f"{source} contains non-finite values")


def load_q_trajectory(path: str | Path, expected_nq: int) -> np.ndarray:
    """读取 A05 q trajectory。

    当前 A05 仍是 TODO skeleton，因此默认轨迹不存在时，A12 会返回一个最小
    零位姿占位轨迹，用来验证 replay 入口和 shape contract，但不会写假输出。
    """
    trajectory_path = Path(path).expanduser()
    if not trajectory_path.is_absolute():
        trajectory_path = A_ROOT / trajectory_path
    trajectory_path = trajectory_path.resolve()

    if trajectory_path.exists():
        q_trajectory = np.load(trajectory_path)
    elif trajectory_path == DEFAULT_A05_TRAJECTORY.resolve():
        q_trajectory = _placeholder_q_trajectory(expected_nq)
    else:
        raise FileNotFoundError(f"q trajectory not found: {trajectory_path}")

    q_trajectory = np.asarray(q_trajectory, dtype=float)
    validate_q_trajectory(q_trajectory, expected_nq=expected_nq, source=str(trajectory_path))
    return q_trajectory


def apply_q_frame(model: Any, data: Any, q: np.ndarray) -> None:
    """把一帧 q 写入 MuJoCo data 并更新 forward kinematics。

    输入：
    - model/data：MuJoCo 模型和状态。
    - q：shape=(model.nq,) 的关节位置。

    输出：
    - 无返回；副作用是更新 `data.qpos` 和 MuJoCo 派生状态。

    数学逻辑：
    - 这里只调用 `mj_forward`，用于更新 site/body 位置；不执行控制或仿真步进。
    """
    import mujoco

    q_array = np.asarray(q, dtype=float)
    if q_array.shape != (model.nq,):
        raise ValueError(f"q frame expected shape ({model.nq},), got {q_array.shape}")
    data.qpos[:] = q_array
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A12 MuJoCo visualization hub.")
    parser.add_argument(
        "--mode",
        choices=["inspect_a", "mink_ur5e", "mink_ur5e_actuator", "replay_a05"],
        default="inspect_a",
        help="hub 模式：检查 A 模型、运行 mink 示例，或 replay A05 q trajectory。",
    )
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="A 项目 robot.yaml 路径。")
    parser.add_argument("--mjcf", default=None, help="可选：覆盖 robot.yaml 中的 mjcf_path。")
    parser.add_argument("--q-trajectory", default=str(DEFAULT_A05_TRAJECTORY), help="replay_a05 使用的 q trajectory。")
    parser.add_argument("--frame-index", type=int, default=0, help="replay_a05 使用的轨迹帧索引。")
    parser.add_argument("--summary-json", default=None, help="可选：把 inspect summary 写入 JSON。")
    parser.add_argument("--dry-run", action="store_true", help="只打印将要执行的动作，不打开外部脚本。")
    return parser.parse_args()


def run_mink_example(mode: str, dry_run: bool) -> int:
    """运行或预览 mink UR5e example 子进程。"""
    script_path = resolve_mink_example_script(mode)
    command = [sys.executable, str(script_path)]
    env = build_mink_example_environment(os.environ)

    print(f"[A12] mink example: {script_path}")
    print(f"[A12] PYTHONPATH prefix: {str(MINK_SRC_ROOT)}")
    if dry_run:
        print(f"[A12] dry-run command: {' '.join(command)}")
        return 0
    return subprocess.run(command, env=env, check=False).returncode


def run_inspect_a(args: argparse.Namespace) -> dict[str, Any]:
    """执行 A 项目模型 headless inspect。"""
    mjcf_path = resolve_a_project_mjcf(args.config, args.mjcf)
    model, data = load_model_and_data(mjcf_path)
    summary = collect_model_summary(
        label="a_project",
        mjcf_path=mjcf_path,
        model=model,
        data=data,
        keyframe_name=None,
        keyframe_applied=False,
    )
    if args.summary_json:
        output_path = resolve_path(args.summary_json, A_ROOT)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return summary


def run_replay_a05(args: argparse.Namespace) -> dict[str, Any]:
    """加载 A05 q trajectory 并应用指定帧。"""
    mjcf_path = resolve_a_project_mjcf(args.config, args.mjcf)
    model, data = load_model_and_data(mjcf_path)
    q_trajectory = load_q_trajectory(args.q_trajectory, expected_nq=model.nq)

    if args.frame_index < 0 or args.frame_index >= q_trajectory.shape[0]:
        raise IndexError(
            f"frame_index {args.frame_index} outside trajectory length {q_trajectory.shape[0]}"
        )

    apply_q_frame(model, data, q_trajectory[args.frame_index])
    summary = collect_model_summary(
        label="replay_a05",
        mjcf_path=mjcf_path,
        model=model,
        data=data,
        keyframe_name=None,
        keyframe_applied=False,
    )
    summary["q_trajectory_shape"] = [int(item) for item in q_trajectory.shape]
    summary["frame_index"] = int(args.frame_index)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return summary


def main() -> None:
    args = parse_args()

    if args.mode == "inspect_a":
        run_inspect_a(args)
        return
    if args.mode == "replay_a05":
        run_replay_a05(args)
        return
    if args.mode in {"mink_ur5e", "mink_ur5e_actuator"}:
        raise SystemExit(run_mink_example(args.mode, dry_run=args.dry_run))

    raise ValueError(f"Unsupported mode: {args.mode}")


if __name__ == "__main__":
    main()
