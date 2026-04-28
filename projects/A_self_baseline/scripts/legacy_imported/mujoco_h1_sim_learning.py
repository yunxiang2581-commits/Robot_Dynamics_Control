from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path


def force_x11_for_viewer() -> None:
    """
    在导入 mujoco 之前，优先把图形后端切到 X11 / XWayland。

    这一部分实现什么功能：
    - 避免 Wayland/libdecor 环境下 viewer 直接崩溃。

    为什么这一步需要做：
    - MuJoCo viewer 的稳定性很依赖图形后端。
    - 如果环境变量设置得太晚，底层窗口系统可能已经选定，后面再改就来不及了。

    推荐 API / 方法：
    - os.environ
    - os.execvpe(...)

    输入：
    - 当前 shell 环境变量

    输出：
    - 经过修正的 X11 图形环境
    """
    needs_reexec = (
        os.environ.get("WAYLAND_DISPLAY")
        and os.environ.get("_MUJOCO_FORCE_X11_DONE") != "1"
    )

    os.environ["XDG_SESSION_TYPE"] = "x11"
    os.environ["GDK_BACKEND"] = "x11"
    os.environ["QT_QPA_PLATFORM"] = "xcb"
    os.environ.pop("WAYLAND_DISPLAY", None)

    if needs_reexec:
        os.environ["_MUJOCO_FORCE_X11_DONE"] = "1"
        os.execvpe(sys.executable, [sys.executable, *sys.argv], os.environ)


force_x11_for_viewer()

import mujoco
import numpy as np


# =============================
# 路径常量
# =============================
PROJECT_ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = PROJECT_ROOT / "outputs" / "mujoco_h1_sim_learning"


# =============================
# MJCF 候选路径
# =============================
MJCF_CANDIDATES = [
    PROJECT_ROOT / "third_party/unitree_ros/robots/h1_description/mjcf/scene_with_hand_bright.xml",
    PROJECT_ROOT / "unitree_ros/robots/h1_description/mjcf/scene_with_hand_bright.xml",
    PROJECT_ROOT / "third_party/unitree_ros/robots/h1_description/mjcf/h1_with_hand.xml",
    PROJECT_ROOT / "unitree_ros/robots/h1_description/mjcf/h1_with_hand.xml",
]


# =============================
# 目标对象名称（先用最小集合）
# =============================
JOINT_NAME = "left_knee_joint"
FOOT_BODY_NAME = "left_ankle_link"
PELVIS_BODY_NAME = "pelvis"


def resolve_mjcf() -> Path:
    """
    从候选列表中找到一个真实存在的 MJCF 文件。

    这一部分实现什么功能：
    - 让脚本在不同目录布局下都能找到 H1 场景文件。

    为什么这一步需要做：
    - MuJoCo 建模的入口就是 MJCF；路径不稳，后续所有步骤都无法进行。

    推荐 API / 方法：
    - Path.exists()

    输入：
    - MJCF_CANDIDATES

    输出：
    - mjcf_path（Path）
    """
    for path in MJCF_CANDIDATES:
        if path.exists():
            return path
    raise FileNotFoundError(
        "MJCF not found. Checked:\n" + "\n".join(str(p) for p in MJCF_CANDIDATES)
    )


def main() -> None:
    """
    这个脚本只学习“如何调用 MuJoCo 仿真器实现一个最小仿真模块”。

    学习目标：
    1. 如何从 MJCF 构建 MuJoCo model/data
    2. 如何找到关节和刚体 ID
    3. 如何推进一步仿真（mj_step）
    4. 如何读取 qpos / qvel / body 世界位置
    5. 如何在有需要时接入 viewer

    这份骨架故意不加绘图、不加复杂控制，只保留仿真器调用主线。
    """
    parser = argparse.ArgumentParser(description="Minimal MuJoCo simulation learning skeleton for H1")
    parser.add_argument("--duration", type=float, default=3.0, help="Simulation duration in seconds")
    parser.add_argument("--headless", action="store_true", help="Run without viewer window")
    args = parser.parse_args()

    # TODO 1: 解析 MJCF 路径
    # 这一部分实现什么功能：
    # - 找到 H1 场景文件。
    # 为什么这一步需要做：
    # - 没有 MJCF 就无法创建 MuJoCo model。
    # 推荐 API：
    # - resolve_mjcf()
    # 输入：无
    # 输出：mjcf_path（Path）
    mjcf_path  = resolve_mjcf()
    # TODO 2: 加载 MuJoCo 的 model / data
    # 这一部分实现什么功能：
    # - 从 MJCF 创建 model，并基于 model 创建 data。
    # 为什么这一步需要做：
    # - model 是结构定义，data 是当前仿真状态；这两者是 MuJoCo 最核心的对象。
    # 推荐 API：
    # - mujoco.MjModel.from_xml_path(str(mjcf_path))
    # - mujoco.MjData(model)
    # 输入：mjcf_path
    # 输出：model, data
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    # TODO 3: 找到要观察的对象 ID
    # 这一部分实现什么功能：
    # - 找到左膝 joint、左脚 body、pelvis body 对应的内部 ID。
    # 为什么这一步需要做：
    # - 后续读取关节角、速度、足端位置时都需要这些 ID。
    # 推荐 API：
    # - mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, JOINT_NAME)
    # - mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, FOOT_BODY_NAME)
    # 输入：model
    # 输出：joint_id, foot_body_id, pelvis_body_id
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, JOINT_NAME)
    foot_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, FOOT_BODY_NAME)
    pelvis_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, PELVIS_BODY_NAME)

    # TODO 4: 读取关节在 qpos / qvel 中的地址
    # 这一部分实现什么功能：
    # - 把“关节 ID”转成“状态向量索引位置”。
    # 为什么这一步需要做：
    # - MuJoCo 的 data.qpos / data.qvel 是大向量，要知道某个关节的数据在哪个下标。
    # 推荐 API：
    # - model.jnt_qposadr[joint_id]
    # - model.jnt_dofadr[joint_id]
    # 输入：model, joint_id
    # 输出：qpos_adr, dof_adr
    qpos_adr = model.jnt_qposadr[joint_id]
    dof_adr = model.jnt_dofadr[joint_id]

    # TODO 5: 初始化仿真状态
    # 这一部分实现什么功能：
    # - 让机器人从一个清晰、可复现的初始状态开始。
    # 为什么这一步需要做：
    # - 不初始化的话，不方便对比后续每次实验的结果。
    # 推荐 API：
    # - 如果 model.nkey > 0: data.qpos[:] = model.key_qpos[0]
    # - data.qvel[:] = 0.0
    # - mujoco.mj_forward(model, data)
    # 输入：model, data
    # 输出：初始化后的 data
    if model.nkey > 0:
        data.qpos[:] = model.key_qpos[0]
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    # TODO 6: 打开 viewer（可选）
    # 这一部分实现什么功能：
    # - 在非 headless 模式下打开可视化窗口。
    # 为什么这一步需要做：
    # - 学习阶段最直接的反馈就是“看到机器人在动”。
    # 推荐 API：
    # - import mujoco.viewer as mj_viewer
    # - with mj_viewer.launch_passive(model, data) as viewer:
    # 输入：model, data
    # 输出：viewer
    import mujoco.viewer as mj_viewer
    viewer = None
    if not args.headless:
        viewer = mj_viewer.launch_passive(model, data)

    # TODO 7: 写一个最小 step_once()
    # 这一部分实现什么功能：
    # - 封装“一步仿真里要做什么”。
    # 为什么这一步需要做：
    # - 后续你无论是写 PD、IK 回放还是动力学控制，最终都要落到“一步怎么推进”。
    # 推荐内容：
    # - 读取当前 q / dq
    # - 如有需要，设置 data.ctrl
    # - 调用 mujoco.mj_step(model, data)
    # - 调用 mujoco.mj_forward(model, data)
    # 输入：model, data
    # 输出：更新后的 data
    mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)

    # TODO 8: 写仿真主循环
    # 这一部分实现什么功能：
    # - 按时间持续推进仿真，直到达到 duration。
    # 为什么这一步需要做：
    # - MuJoCo 仿真不是一次函数调用，而是很多个小时间步积累起来的过程。
    # 推荐 API：
    # - while data.time < args.duration:
    # - step_once(...)
    # 输入：data.time, args.duration
    # 输出：完整的仿真时间序列
    while data.time < args.duration:
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)

    # TODO 9: 在循环里读取最小观测量
    # 这一部分实现什么功能：
    # - 读取左膝角度、左膝角速度、左脚世界位置、pelvis 世界位置。
    # 为什么这一步需要做：
    # - 先学会“怎么看仿真状态”，后面才谈控制与分析。
    # 推荐 API：
    # - data.qpos[qpos_adr]
    # - data.qvel[dof_adr]
    # - data.xpos[foot_body_id]
    # - data.xpos[pelvis_body_id]
    # 输入：data, qpos_adr, dof_adr, body_id
    # 输出：q, dq, p_foot, p_pelvis

        q = data.qpos[qpos_adr]
        dq = data.qvel[dof_adr]
        p_foot = data.xpos[foot_body_id]
        p_pelvis = data.xpos[pelvis_body_id]
    # TODO 10: 在有 viewer 时同步画面
    # 这一部分实现什么功能：
    # - 让仿真状态实时显示在窗口里。
    # 为什么这一步需要做：
    # - 如果不调用 viewer.sync()，窗口不会更新。
    # 推荐 API：
    # - if viewer.is_running(): viewer.sync()
    # 输入：viewer
    # 输出：更新后的可视化画面
        if viewer is not None and viewer.is_running():
            viewer.sync()
    # TODO 11: 加入简单的实时节奏控制（可选）
    # 这一部分实现什么功能：
    # - 让可视化播放速度更接近真实时间，而不是瞬间跑完。
    # 为什么这一步需要做：
    # - 学习阶段更容易观察，也更稳定。
    # 推荐 API：
    # - time.sleep(...)
    # 输入：循环节奏
    # 输出：更平滑的观感
            time.sleep(0.01)
    # TODO 12: 输出最小日志
    # 这一部分实现什么功能：
    # - 打印关键元信息，确认仿真器是否按预期工作。
    # 为什么这一步需要做：
    # - 学习阶段先看清楚 model 维度和对象 ID，比直接埋头写控制更重要。
    # 建议输出：
    # - mjcf_path
    # - model.nq, model.nv, model.nu
    # - joint/body id
    # - 仿真是否 headless

    print("MuJoCo H1 simulation learning skeleton")
    print(f"project_root={PROJECT_ROOT}")
    print(f"out_dir={OUT_DIR}")
    print(f"duration={args.duration}")
    print(f"headless={args.headless}")


if __name__ == "__main__":
    main()
