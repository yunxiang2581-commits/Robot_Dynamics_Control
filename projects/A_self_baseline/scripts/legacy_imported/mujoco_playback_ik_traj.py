from __future__ import annotations

import time
from pathlib import Path
import os
import sys
# 注意：mujoco / mujoco.viewer / numpy 的导入也可以作为学习内容。
# 如果你要处理 X11/Wayland 图形后端，应在 import mujoco 前设置环境变量。
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
import mujoco.viewer as mj_viewer
import numpy as np


# =============================
# Step 5B: MuJoCo IK 轨迹离线回放 TODO 骨架
# =============================
# 学习目标：
# - 从 MJCF 创建 MuJoCo model/data。
# - 读取 Step 5A 生成的 q_traj.npy。
# - 检查 q_traj 是否能写入 MuJoCo data.qpos。
# - 使用 viewer 逐帧显示 IK 轨迹。
#
# 当前文件故意保留 TODO，不直接写满核心回放逻辑。
def pin_q_to_mj_qpos(q_pin: np.ndarray, default_qpos: np.ndarray) -> np.ndarray:
    """
    TODO 9.5: 将 Pinocchio q 转换为 MuJoCo qpos。

    输入：
    - q_pin: Pinocchio 配置向量，长度为 model.nq。
    - default_qpos: MuJoCo 默认站立姿态的 qpos，长度为 model.nq。

    输出：
    - q_mj: 转换后的 MuJoCo qpos，长度为 model.nq。

    需要补什么：
    - 创建 q_mj = q_pin.copy()。
    - 修正 freejoint 平移：让 pelvis/base 使用 MuJoCo 默认高度。
    - 修正 freejoint 四元数顺序：Pinocchio 是 [qx, qy, qz, qw]，MuJoCo 是 [qw, qx, qy, qz]。
    - 返回 q_mj。
     """
    q_mj = q_pin.copy()
    q_mj[:3] = default_qpos[:3] + q_pin[:3]
    q_mj[3:7] = np.array([q_pin[6], q_pin[3], q_pin[4], q_pin[5]])
    return q_mj

# =============================
# TODO 1: 路径常量
# =============================
# 要实现什么：
# - 定位项目根目录 PROJECT_ROOT。
# - 定位 Step 5A 输出的 Q_TRAJ_FILE。
#
# 为什么需要：
# - 回放脚本不应该依赖 shell 当前目录。
# - q_traj.npy 是 Pinocchio IK 生成的轨迹。
PROJECT_ROOT = Path(__file__).resolve().parents[1]
Q_TRAJ_FILE = PROJECT_ROOT / "outputs" / "ik" / "h1_left_foot_step5_q_traj.npy"


# =============================
# TODO 2: MJCF 候选路径
# =============================
# 要实现什么：
# - 兼容 unitree_ros 在 third_party 下，或直接在项目根目录下。
#
# 为什么需要：
# - MuJoCo 使用 MJCF 加载模型。
# - 不同环境中模型目录可能不同。
MJCF_CANDIDATES = [
    PROJECT_ROOT / "third_party/unitree_ros/robots/h1_description/mjcf/scene_with_hand_bright.xml",
    PROJECT_ROOT / "unitree_ros/robots/h1_description/mjcf/scene_with_hand_bright.xml",
]


def resolve_mjcf() -> Path:
    """
    TODO 3: 从 MJCF_CANDIDATES 中找到真实存在的 MJCF。

    需要补什么：
    - 遍历 MJCF_CANDIDATES。
    - 如果 path.exists()，返回该 path。
    - 如果都不存在，raise FileNotFoundError，并列出检查过的路径。

    推荐 API：
    - Path.exists()
    """
    for path in MJCF_CANDIDATES:
        if path.exists():
            return path
    raise FileNotFoundError(
        "MJCF not found. Checked:\n" + "\n".join(str(p) for p in MJCF_CANDIDATES)
    )
    


def main() -> None:
    """
    Step 5B 主流程 TODO。

    输入：
    - H1 MJCF
    - outputs/ik/h1_left_foot_step5_q_traj.npy

    输出：
    - MuJoCo viewer 中的 IK 轨迹离线回放。
    """
    # =============================
    # TODO 4: 设置回放帧间隔
    # =============================
    # 需要补什么：
    # - frame_dt = 0.03
    #
    # 为什么需要：
    # - 不加 sleep 会瞬间播完。
    # - 合理延时有助于观察，也能减少 viewer 启动后立即退出的不稳定。
    frame_dt = 0.03
    # =============================
    # TODO 5: 解析 MJCF 路径
    # =============================
    # 需要补什么：
    # - mjcf_path = resolve_mjcf()
    mjcf_path = resolve_mjcf()
    # =============================
    # TODO 6: 加载 MuJoCo model/data
    # =============================
    # 需要补什么：
    # - model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    # - data = mujoco.MjData(model)
    #
    # 为什么需要：
    # - model 是结构和参数。
    # - data 是当前状态，包括 qpos、qvel、body 位姿等。
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    # =============================
    # TODO 7: 读取 q_traj.npy
    # =============================
    # 需要补什么：
    # - 检查 Q_TRAJ_FILE.exists()
    # - q_traj = np.load(Q_TRAJ_FILE)
    #
    # 为什么需要：
    # - q_traj 是 Step 5A 生成的 Pinocchio IK 配置轨迹。
    # - 回放时每一行都会写入 data.qpos。
    if not Q_TRAJ_FILE.exists():
        raise FileNotFoundError(f"q_traj file not found: {Q_TRAJ_FILE}")
    q_traj = np.load(Q_TRAJ_FILE)
    # =============================
    # TODO 8: 检查 q_traj 维度
    # =============================
    # 需要补什么：
    # - q_traj.ndim == 2
    # - q_traj.shape[1] == model.nq
    #
    # 为什么要检查 q_traj.shape[1] == model.nq：
    # - MuJoCo data.qpos 长度必须等于 model.nq。
    # - 如果 Pinocchio URDF 和 MuJoCo MJCF 不是同一个模型，维度可能对不上。
    if q_traj.ndim != 2:
        raise ValueError(f"q_traj should be 2D, but got shape {q_traj.shape}")
    if q_traj.shape[1] != model.nq:
        raise ValueError(
            f"q_traj shape[1] should match model.nq={model.nq}, but got {q_traj.shape[1]}"
        )
    # =============================
    # TODO 9: 输出最小日志
    # =============================
    # 需要补什么：
    # - MJCF 路径
    # - q_traj 路径
    # - MuJoCo nq / nv / nu
    # - q_traj shape
    # - 回放帧数
    
    print(f"Using MJCF: {mjcf_path}")
    print(f"Loaded q_traj from: {Q_TRAJ_FILE}")
    print(f"Model nq={model.nq}, nv={model.nv}, nu={model.nu}")
    print(f"q_traj shape: {q_traj.shape}")
    print(f"Number of frames to play: {q_traj.shape[0]}")

    # =============================
    # TODO 9.5: 将 Pinocchio q 转换为 MuJoCo qpos
    # =============================
    # 需要补什么：
    # - 保存 MuJoCo 默认站立姿态 default_qpos = data.qpos.copy()。
    # - 编写一个小函数，例如 pin_q_to_mj_qpos(q_pin, default_qpos)。
    # - 在函数中创建 q_mj = q_pin.copy()。
    # - 修正 freejoint 平移：让 pelvis/base 使用 MuJoCo 默认高度。
    # - 修正 freejoint 四元数顺序：Pinocchio 是 [qx, qy, qz, qw]，MuJoCo 是 [qw, qx, qy, qz]。
    # - 返回 q_mj。
    #
    # 为什么需要：
    # - Pinocchio neutral 的 base 高度通常是 z=0。
    # - 当前 MJCF 默认站立姿态的 pelvis 高度约为 z=1.1。
    # - 如果直接 data.qpos[:] = q_traj[i]，机器人会以 pelvis 贴近地面的姿态显示，脚会在地面以下。
    #
    # 推荐实现思路：
    # - default_qpos = data.qpos.copy()
    # - q_mj[:3] = default_qpos[:3] + q_pin[:3]
    # - q_mj[3:7] = np.array([q_pin[6], q_pin[3], q_pin[4], q_pin[5]])
    #
    # 完成后在 TODO 11 中不要再直接写：
    # - data.qpos[:] = q_traj[i]
    # 而应该写：
    # - data.qpos[:] = pin_q_to_mj_qpos(q_traj[i], default_qpos)
    default_qpos = data.qpos.copy()
    q_mj = q_traj[0].copy()
    q_mj[:3] = default_qpos[:3] + q_traj[0][:3]
    q_mj[3:7] = np.array([q_traj[0][6], q_traj[0][3], q_traj[0][4], q_traj[0][5]])
    # =============================
    # TODO 10: 打开 viewer
    # =============================
    # 需要补什么：
    # - with mj_viewer.launch_passive(model, data) as viewer:
    #
    # 为什么用 with：
    # - context manager 会在退出时清理 viewer 资源。
    # - 比手动创建后直接结束更稳定。
    with mj_viewer.launch_passive(model, data) as viewer:
        time.sleep(0.1)  # 等 viewer 启动稳定后再开始回放
        # =============================
        # TODO 11: 编写逐帧回放循环
        # =============================
        # 需要补什么：
        # - for i in range(q_traj.shape[0]):
        # - if not viewer.is_running(): break
        # - 将 q_traj[i] 先通过 TODO 9.5 转成 MuJoCo qpos。
        # - data.qpos[:] = q_mj
        # - mujoco.mj_forward(model, data)
        # - viewer.sync()
        # - time.sleep(frame_dt)
        #
        # 为什么 MuJoCo 这里只做回放，不做控制：
        # - Step 5A 已经生成每一帧目标 q。
        # - 本阶段只验证这些 q 能否在 MuJoCo 中正确显示。
        # - actuator 控制会引入力矩、PD、接触稳定性等新问题。
        #
        # 为什么不能直接 data.qpos[:] = q_traj[i]：
        # - Pinocchio 和 MuJoCo 的 freejoint 表达不同。
        # - 需要先修正 base 高度和四元数顺序，否则机器人会显示在错误高度。
        #
        # 为什么每帧后要 mujoco.mj_forward：
        # - 修改 qpos 后，MuJoCo 需要重新计算 body/geom 世界位姿等派生状态。
        # - 不调用 mj_forward，viewer 可能显示旧状态。
        for i in range(q_traj.shape[0]):
            if not viewer.is_running():
                print("Viewer closed, stopping playback.")
                break
            data.qpos[:] = pin_q_to_mj_qpos(q_traj[i], default_qpos)
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(frame_dt)
        


if __name__ == "__main__":
    main()
