from __future__ import annotations

from pathlib import Path

import numpy as np
import pinocchio as pin


# =============================
# 路径常量
# =============================
PROJECT_ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = PROJECT_ROOT / "outputs" / "ik"
OUT_FILE = OUT_DIR / "h1_left_foot_step4_apply_update.txt"


# =============================
# URDF 候选路径（按优先级尝试）
# =============================
URDF_CANDIDATES = [
    PROJECT_ROOT / "third_party/unitree_ros/robots/h1_description/urdf/h1_with_hand.urdf",
    PROJECT_ROOT / "unitree_ros/robots/h1_description/urdf/h1_with_hand.urdf",
]


# =============================
# 左脚 frame 候选名（按优先级尝试）
# =============================
LEFT_FOOT_FRAME_CANDIDATES = [
    "left_ankle_link",
    "left_ankle_joint",
    "left_foot",
    "left_sole",
]


def resolve_urdf() -> Path:
    """从候选路径中返回第一个存在的 URDF。"""
    for urdf_path in URDF_CANDIDATES:
        if urdf_path.exists():
            return urdf_path
    raise FileNotFoundError(
        "URDF not found. Checked:\n" + "\n".join(str(p) for p in URDF_CANDIDATES)
    )


def pick_frame(model: pin.Model, candidates: list[str]) -> str:
    """按候选顺序挑选模型中真实存在的 frame 名。"""
    frame_names = {frame.name for frame in model.frames}
    for name in candidates:
        if name in frame_names:
            return name
    raise RuntimeError("Target frame not found. Tried: " + ", ".join(candidates))


def main() -> None:
    """
    任务 4.4：把一步速度更新应用到姿态（q）并比较误差变化（骨架版）。

    学习重点：
    1) 为什么要用 pin.integrate，而不是 q + v * dt
       - 机器人配置空间不总是欧式空间（尤其包含 free-flyer / 四元数时）。
       - 直接做 q + v*dt 可能破坏配置约束（例如四元数归一化、流形结构）。
       - pin.integrate(model, q, v*dt) 会按模型配置流形做“合法更新”。

    2) 为什么这一步是从“速度更新”走到“姿态更新”
       - 上一步求出的 v_step 是“广义速度方向/大小”，不是新姿态。
       - 这一节把速度在小时间步 dt 上积分，得到新的 q_next。
       - 这一步把 IK 从“速度层”推进到“配置层”。

    3) 为什么要比较 err_before 和 err_after
       - err_before 表示更新前末端到目标点的距离。
       - err_after 表示应用 q_next 后的新距离。
       - 若 err_after < err_before，说明这一步更新方向有效。
    """

    # TODO 1: 解析 URDF
    # 推荐：urdf_path = resolve_urdf()
    urdf_path = resolve_urdf()

    # TODO 2: 构建浮动基模型
    # 推荐：
    # - model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    # - data = model.createData()
    model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    data = model.createData()

    # TODO 3: neutral configuration
    # 推荐：q = pin.neutral(model)
    q = pin.neutral(model)

    # TODO 4: FK + updateFramePlacements
    # 推荐：
    # - pin.forwardKinematics(model, data, q)
    # - pin.updateFramePlacements(model, data)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    # TODO 5: 选择左脚 frame
    # 推荐：
    # - left_foot_frame = pick_frame(model, LEFT_FOOT_FRAME_CANDIDATES)
    # - frame_id = model.getFrameId(left_foot_frame)
    left_foot_frame = pick_frame(model, LEFT_FOOT_FRAME_CANDIDATES)
    frame_id = model.getFrameId(left_foot_frame)

    # TODO 6: 读取 p_cur
    # 推荐：p_cur = data.oMf[frame_id].translation
    p_cur = data.oMf[frame_id].translation.copy()

    # TODO 7: 定义 p_des
    # 示例：p_des = p_cur + np.array([0.02, 0.0, 0.0])
    p_des = p_cur + np.array([0.02, 0.0, 0.0])

    # TODO 8: computeJointJacobians
    # 推荐：pin.computeJointJacobians(model, data, q)
    pin.computeJointJacobians(model, data, q)

    # TODO 9: getFrameJacobian
    # 推荐：J = pin.getFrameJacobian(model, data, frame_id, pin.LOCAL_WORLD_ALIGNED)
    J = pin.getFrameJacobian(model, data, frame_id, pin.LOCAL_WORLD_ALIGNED)

    # TODO 10: 提取 J_pos
    # 推荐：J_pos = J[:3, :]
    J_pos = J[:3, :]

    # TODO 11: 定义误差 e
    # 推荐：e = p_des - p_cur
    e = p_des - p_cur

    # TODO 12: 设置 gain / damping
    # 示例：gain = 0.5, damping = 1e-3
    gain = 0.5
    damping = 1e-3

    # TODO 13: 求 v_step
    # 提示：使用阻尼最小二乘得到一步广义速度解
    v_des = gain * e
    v_step = J_pos.T @ np.linalg.inv(J_pos @ J_pos.T + damping**2 * np.eye(3)) @ v_des

    # TODO 14: 设置 dt
    # 示例：dt = 0.05 或更小
    dt = 0.05
    # TODO 15: 用 pin.integrate 得到 q_next
    # 推荐：q_next = pin.integrate(model, q, v_step * dt)
    q_next = pin.integrate(model, q, v_step * dt)
    # TODO 16: 对 q_next 做 FK 得到 p_next
    # 推荐：
    # - pin.forwardKinematics(model, data, q_next)
    # - pin.updateFramePlacements(model, data)
    # - p_next = data.oMf[frame_id].translation
    pin.forwardKinematics(model, data, q_next)
    pin.updateFramePlacements(model, data)
    p_next = data.oMf[frame_id].translation.copy()
    # TODO 17: 比较 err_before / err_after
    # 推荐：
    # - err_before = np.linalg.norm(p_des - p_cur)
    # - err_after = np.linalg.norm(p_des - p_next)
    err_before = np.linalg.norm(p_des - p_cur)
    err_after = np.linalg.norm(p_des - p_next)
    print(f"Error before update: {err_before:.6f}")
    print(f"Error after update: {err_after:.6f}")

    # TODO 18: 输出并保存到 outputs/ik/h1_left_foot_step4_apply_update.txt
    # 推荐：
    # - OUT_DIR.mkdir(parents=True, exist_ok=True)
    # - OUT_FILE.write_text(report, encoding="utf-8")
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    OUT_FILE.write_text(
        f"URDF Path: {urdf_path}\n"
        f"Left Foot Frame: {left_foot_frame}\n"
        f"Current Position: {p_cur}\n"
        f"Desired Position: {p_des}\n"
        f"Position after Update: {p_next}\n"
        f"Error before Update: {err_before:.6f}\n"
        f"Error after Update: {err_after:.6f}\n",
        encoding="utf-8",
    )
    print("[Progress] TODO 1~13 completed. Please continue TODO 14~18.")


if __name__ == "__main__":
    main()
