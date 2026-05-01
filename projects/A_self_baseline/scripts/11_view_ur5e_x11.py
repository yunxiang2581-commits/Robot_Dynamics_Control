"""A11: 使用 X11 打开 MuJoCo viewer 观察 A 项目 UR5e 机械臂模型。

当前 pipeline 定位：
- A11 是一个简单的模型可视化辅助脚本。
- 它只负责打开窗口观察 A 项目的 UR5e MJCF 模型。

学习目标：
- 学会从 A 项目 `configs/robot.yaml` 稳定定位 MuJoCo MJCF 模型。
- 学会在导入 `mujoco.viewer` 之前准备 X11 / XWayland 环境。
- 学会用 `mujoco.MjModel`、`mujoco.MjData` 和 `mujoco.viewer.launch_passive`
  建立最小可视化循环。

输入：
- 默认输入是 A 项目配置文件 `projects/A_self_baseline/configs/robot.yaml`。
- 配置中的 `mjcf_path` 指向 UR5e `scene.xml`。
- 可用 `--mjcf` 覆盖模型路径。

输出：
- 一个 MuJoCo viewer 窗口。
- 终端打印模型路径、`nq / nv / nu`、DISPLAY 和 X11 相关环境变量。

明确不做：
- 不做 IK。
- 不做 Jacobian。
- 不做控制器。
- 不调用 mink 替代自己的实现。
- 不改变模型数学逻辑，只做可视化观察。
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import yaml


# =============================
# 路径常量
# =============================
# 要实现什么：
# - 通过 __file__ 定位 A 项目根目录和仓库根目录。
#
# 为什么需要：
# - 本仓库把“路径设置”作为正式学习内容。
# - 脚本不应该依赖从哪个 shell 当前目录启动。
#
# 输入：
# - 当前脚本路径 __file__。
#
# 输出：
# - A_ROOT / REPO_ROOT / DEFAULT_CONFIG。
A_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = A_ROOT.parents[1]
DEFAULT_CONFIG = A_ROOT / "configs" / "robot.yaml"


def build_x11_environment(environ: Mapping[str, str]) -> tuple[dict[str, str], bool]:
    """构造适合 MuJoCo viewer 的 X11 环境变量。

    这一部分实现什么功能：
    - 设置 `XDG_SESSION_TYPE=x11`、`GDK_BACKEND=x11`、`QT_QPA_PLATFORM=xcb`。
    - 移除 `WAYLAND_DISPLAY`，避免 GLFW / Qt / libdecor 走 Wayland 后端。
    - 判断是否需要重新执行当前 Python 进程。

    为什么这一步必要：
    - MuJoCo viewer 的窗口后端通常在 import 或首次创建窗口时确定。
    - 如果环境变量设置太晚，viewer 可能仍然使用 Wayland，导致窗口崩溃或打不开。

    输入：
    - 当前环境变量 mapping，通常是 `os.environ`。

    输出：
    - new_env: 修改后的环境变量副本。
    - needs_reexec: 如果当前进程已经处在 Wayland 环境且还没重启过，则返回 True。
    """
    new_env = dict(environ)
    needs_reexec = bool(
        new_env.get("WAYLAND_DISPLAY")
        and new_env.get("_A11_MUJOCO_FORCE_X11_DONE") != "1"
    )

    new_env["XDG_SESSION_TYPE"] = "x11"
    new_env["GDK_BACKEND"] = "x11"
    new_env["QT_QPA_PLATFORM"] = "xcb"
    new_env.pop("WAYLAND_DISPLAY", None)
    return new_env, needs_reexec


def force_x11_for_viewer() -> None:
    """在导入 mujoco / mujoco.viewer 之前强制切到 X11。

    推荐使用方式：
    - 在 `main()` 一开始调用。
    - 调用之后再 import `mujoco` 和 `mujoco.viewer`。

    可能风险：
    - 如果当前系统没有可用的 X server 或 `DISPLAY` 没有转发，viewer 仍然无法打开。
    - 这个函数只选择图形后端，不修改系统权限，也不绕过 Docker/X11 安全设置。
    """
    new_env, needs_reexec = build_x11_environment(os.environ)
    os.environ.clear()
    os.environ.update(new_env)

    if needs_reexec:
        os.environ["_A11_MUJOCO_FORCE_X11_DONE"] = "1"
        os.execvpe(sys.executable, [sys.executable, *sys.argv], dict(os.environ))


def resolve_path(path_str: str | Path, base_dir: Path) -> Path:
    """把配置或 CLI 中的路径解析为绝对路径。

    输入：
    - path_str: YAML 或命令行给出的路径。
    - base_dir: 相对路径的解析基准。

    输出：
    - 绝对路径 Path。
    """
    path = Path(path_str).expanduser()
    if path.is_absolute():
        return path.resolve()
    return (base_dir / path).resolve()


def load_yaml_config(config_path: str | Path) -> dict[str, Any]:
    """读取 A 项目 robot.yaml 配置。

    为什么需要：
    - A 项目把模型路径集中放在 `configs/robot.yaml`。
    - 这样后续脚本可以复用同一份模型入口，而不是各自写死路径。
    """
    path = Path(config_path).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")

    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError(f"Config file must contain a YAML mapping: {path}")
    return config


def resolve_mjcf_path(config_path: str | Path, mjcf_override: str | Path | None) -> Path:
    """确定本次要打开的 MJCF 文件。

    查找逻辑：
    1. 如果命令行传入 `--mjcf`，优先使用它。
    2. 否则读取 `robot.yaml` 中的 `mjcf_path`。
    3. 相对路径按 A 项目根目录 `A_ROOT` 解析。

    输出：
    - 已存在的 MJCF 文件路径。
    """
    if mjcf_override is not None:
        mjcf_path = resolve_path(mjcf_override, A_ROOT)
        checked_source = f"--mjcf {mjcf_override}"
    else:
        config = load_yaml_config(config_path)
        if "mjcf_path" not in config:
            raise KeyError(f"Missing 'mjcf_path' in config: {Path(config_path).resolve()}")
        mjcf_path = resolve_path(config["mjcf_path"], A_ROOT)
        checked_source = f"config mjcf_path in {Path(config_path).resolve()}"

    if not mjcf_path.exists():
        raise FileNotFoundError(
            "MJCF file not found.\n"
            f"Source: {checked_source}\n"
            f"Resolved path: {mjcf_path}"
        )
    return mjcf_path


def load_model_and_data(mjcf_path: str | Path):
    """从 MJCF 创建 MuJoCo model/data。

    输入：
    - mjcf_path: `scene.xml` 路径。

    输出：
    - model: `mujoco.MjModel`，保存模型结构、维度、关节和几何信息。
    - data: `mujoco.MjData`，保存当前仿真状态，例如 qpos、qvel、site 位姿。
    """
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(Path(mjcf_path)))
    data = mujoco.MjData(model)
    return model, data


def find_keyframe_id(model: Any, keyframe_name: str) -> int | None:
    """按名称查找 MuJoCo keyframe。

    为什么需要：
    - UR5e scene.xml 通常带有 `home` keyframe。
    - 可视化时先进入 home 姿态，比随机或零位姿更适合观察机械臂结构。
    """
    import mujoco

    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, keyframe_name)
    if key_id < 0:
        return None
    return int(key_id)


def apply_keyframe(model: Any, data: Any, keyframe_name: str | None) -> bool:
    """把指定 keyframe 写入当前 MuJoCo data。

    输入：
    - model/data: MuJoCo 模型和状态。
    - keyframe_name: 例如 `home`；传 None 表示不使用 keyframe。

    输出：
    - True: 找到并应用了 keyframe。
    - False: 没有指定 keyframe 或模型里没有这个 keyframe。
    """
    if keyframe_name is None:
        return False

    key_id = find_keyframe_id(model, keyframe_name)
    if key_id is None:
        return False

    data.qpos[:] = model.key_qpos[key_id]
    data.qvel[:] = 0.0
    return True


def print_model_summary(mjcf_path: Path, model: Any, keyframe_name: str | None, applied: bool) -> None:
    """打印本次 viewer 的关键输入和模型维度。"""
    print("[A11] UR5e MuJoCo viewer")
    print(f"[A11] mjcf_path: {mjcf_path}")
    print(f"[A11] nq={model.nq}, nv={model.nv}, nu={model.nu}")
    print(f"[A11] keyframe: {keyframe_name or '<none>'}, applied={applied}")
    print(f"[A11] DISPLAY={os.environ.get('DISPLAY', '<unset>')}")
    print(f"[A11] XDG_SESSION_TYPE={os.environ.get('XDG_SESSION_TYPE', '<unset>')}")
    print(f"[A11] GDK_BACKEND={os.environ.get('GDK_BACKEND', '<unset>')}")
    print(f"[A11] QT_QPA_PLATFORM={os.environ.get('QT_QPA_PLATFORM', '<unset>')}")


def run_viewer(model: Any, data: Any, duration: float, show_ui: bool) -> None:
    """打开 MuJoCo passive viewer 并推进最小仿真循环。

    输入：
    - model/data: MuJoCo 模型和状态。
    - duration: 运行时长；0 表示直到手动关闭窗口。
    - show_ui: 是否显示 MuJoCo viewer 左右 UI 面板。

    输出：
    - MuJoCo viewer 窗口。

    数学逻辑是否变化：
    - 不改变。这里每帧只调用 `mj_step` 推进 MuJoCo 默认仿真，并同步画面。
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Open A_self_baseline UR5e MuJoCo model with X11 viewer."
    )
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG),
        help="A 项目 robot.yaml 路径，默认使用 projects/A_self_baseline/configs/robot.yaml。",
    )
    parser.add_argument(
        "--mjcf",
        default=None,
        help="可选：覆盖 robot.yaml 中的 mjcf_path。",
    )
    parser.add_argument(
        "--keyframe",
        default="home",
        help="可选：启动时应用的 keyframe 名称；传空字符串表示不使用。",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="viewer 运行秒数；0 表示直到手动关闭窗口。",
    )
    parser.add_argument(
        "--show-ui",
        action="store_true",
        help="显示 MuJoCo viewer 左右 UI 面板。",
    )
    parser.add_argument(
        "--headless-check",
        action="store_true",
        help="只加载模型并打印摘要，不打开 viewer。用于无图形环境验证。",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    keyframe_name = args.keyframe if args.keyframe else None

    # 必须在加载 mujoco.viewer 前设置 X11 环境。
    force_x11_for_viewer()

    mjcf_path = resolve_mjcf_path(config_path=args.config, mjcf_override=args.mjcf)
    model, data = load_model_and_data(mjcf_path)
    applied = apply_keyframe(model, data, keyframe_name)

    print_model_summary(mjcf_path, model, keyframe_name, applied)
    if keyframe_name is not None and not applied:
        print(f"[A11] warning: keyframe '{keyframe_name}' not found; using default qpos.")

    if args.headless_check:
        print("[A11] headless-check enabled; viewer will not open.")
        return

    if not os.environ.get("DISPLAY"):
        raise RuntimeError(
            "DISPLAY is not set, so MuJoCo viewer cannot open.\n"
            "请确认已经启用 X11 转发，例如在宿主机允许 X server，并把 DISPLAY 传入容器/终端。"
        )

    run_viewer(model=model, data=data, duration=args.duration, show_ui=args.show_ui)


if __name__ == "__main__":
    main()
