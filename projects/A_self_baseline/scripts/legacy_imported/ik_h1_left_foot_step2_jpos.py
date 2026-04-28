from __future__ import annotations

from pathlib import Path

import numpy as np
import pinocchio as pin

import argparse
# =============================
# 路径常量
# =============================
PROJECT_ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = PROJECT_ROOT / "outputs" / "ik"
OUT_FILE = OUT_DIR / "h1_left_foot_step2_jpos.txt"


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
    """
    解析 URDF 路径。

    功能：从候选列表里找到第一个存在的 URDF。
    输入：无
    输出：urdf_path（Path）
    """
    for urdf_path in URDF_CANDIDATES:
        if urdf_path.exists():
            return urdf_path
    raise FileNotFoundError(
        "URDF not found. Checked:\n" + "\n".join(str(p) for p in URDF_CANDIDATES)
    )


def pick_frame(model: pin.Model, candidates: list[str]) -> str:
    """
    选择目标 frame 名。

    功能：按候选顺序找到模型中实际存在的 frame。
    输入：model, candidates
    输出：frame_name（str）
    """
    frame_names = {frame.name for frame in model.frames}
    for name in candidates:
        if name in frame_names:
            return name
    raise RuntimeError("Target frame not found. Tried: " + ", ".join(candidates))


def main() -> None:



    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=str, default=None)
    parser.add_argument("--out", type=str, default=None)
    args = parser.parse_args()

    """
    任务 4.2：左脚位置 IK 的 Jacobian（J_pos）骨架。

    学习重点（请先理解再补 TODO）：
    1. 为什么“位置 IK”只需要 J 的前 3 行：
       - 完整末端速度关系通常写作：v_frame = J * v
       - 其中前 3 行对应线速度（dx, dy, dz），后 3 行对应角速度（wx, wy, wz）
       - 仅做“位置目标”时，我们关心的是末端点位置变化，所以主要使用 J_pos = J[:3, :]

    2. 为什么先做 FK 再算 Jacobian：
       - Jacobian 是“在当前构型 q 下”的局部线性映射
       - FK + updateFramePlacements 先把当前位姿状态更新到 data
       - 后续读 frame 位姿（p0）与理解 Jacobian 作用点时才一致

    3. J 和 J_pos 的区别：
       - J: 6 x nv，描述末端线速度 + 角速度
       - J_pos: 3 x nv，只保留线速度部分，直接服务位置误差控制
    """

    # TODO 1: 解析 URDF
    # 推荐：urdf_path = resolve_urdf()
    # 输入：无
    # 输出：urdf_path（Path）
    urdf_path = resolve_urdf()
    # TODO 2: 构建浮动基模型
    # 推荐 API：
    # - model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    # - data = model.createData()
    # 输入：urdf_path
    # 输出：model, data
    model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    data = model.createData()

    # TODO 3: neutral configuration
    # 推荐 API：q = pin.neutral(model)
    # 输入：model
    # 输出：q

    q = pin.neutral(model)

    # TODO 4: FK + updateFramePlacements
    # 推荐 API：
    # - pin.forwardKinematics(model, data, q)
    # - pin.updateFramePlacements(model, data)
    # 输入：model, data, q
    # 输出：data（位姿已更新）
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    # TODO 5: 选择左脚 frame
    # 推荐 API：
    # - left_foot_frame = pick_frame(model, LEFT_FOOT_FRAME_CANDIDATES)
    # - frame_id = model.getFrameId(left_foot_frame)
    # 输入：model
    # 输出：left_foot_frame, frame_id
    left_foot_frame = pick_frame(model, LEFT_FOOT_FRAME_CANDIDATES)
    frame_id = model.getFrameId(left_foot_frame)
    # TODO 6: 读取 p0 并定义 p_des
    # 推荐 API：
    # - p0 = data.oMf[frame_id].translation
    # - p_des = p0 + np.array([0.02, 0.0, 0.0])
    # 输入：data, frame_id
    # 输出：p0, p_des

    p0 = data.oMf[frame_id].translation
    p_des = p0 + np.array([0.02, 0.0, 0.0])

    # TODO 7: computeJointJacobians
    # 推荐 API：pin.computeJointJacobians(model, data, q)
    # 输入：model, data, q
    # 输出：data 内 Jacobian 相关缓存更新
    pin.computeJointJacobians(model, data, q)
    # TODO 8: getFrameJacobian
    # 推荐 API：
    # J = pin.getFrameJacobian(model, data, frame_id, pin.LOCAL_WORLD_ALIGNED)
    # 输入：model, data, frame_id
    # 输出：J（6 x nv）
    J = pin.getFrameJacobian(model,data,frame_id,pin.LOCAL_WORLD_ALIGNED)

    # TODO 9: 提取 J_pos = J[:3, :]
    # 说明：
    # - 这是位置 IK 的核心矩阵
    # - J_pos 把关节广义速度 v 映射到末端线速度 p_dot
    # 输入：J
    # 输出：J_pos（3 x nv）
    J_pos = J[:3, :]
    # TODO 10: 输出并保存到 outputs/ik/h1_left_foot_step2_jpos.txt
    # 推荐 API：
    # - OUT_DIR.mkdir(parents=True, exist_ok=True)
    # - OUT_FILE.write_text(report, encoding="utf-8")
    # 建议报告内容：urdf_path / frame_name / nq,nv,njoints / p0 / p_des / J.shape / J_pos.shape
    if args.out is None:
        out_file = OUT_FILE
    else:
        out_file = Path(args.out).resolve()
    out_file.write_text(f"URDF Path: {urdf_path}\nLeft Foot Frame: {left_foot_frame}\nCurrent Position: {p0}\nDesired Position: {p_des}\nJacobian Shape: {J.shape}\nPosition Jacobian Shape: {J_pos.shape}", encoding="utf-8")

    print("[Skeleton] Task 4.2 created. Please fill TODO 1~10 step by step.")


if __name__ == "__main__":
    main()
