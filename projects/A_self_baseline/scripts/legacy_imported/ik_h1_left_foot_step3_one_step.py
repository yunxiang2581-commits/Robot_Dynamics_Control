from __future__ import annotations

from pathlib import Path

import numpy as np
import pinocchio as pin


# =============================
# 路径常量
# =============================
PROJECT_ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = PROJECT_ROOT / "outputs" / "ik"
OUT_FILE = OUT_DIR / "h1_left_foot_step3_one_step.txt"


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

    功能：
    - 从候选路径列表中找到第一个真实存在的 URDF 文件。

    为什么需要这一步：
    - 后续 buildModelFromUrdf(...) 必须先知道具体要读取哪个 URDF。

    输入：
    - 无

    输出：
    - urdf_path（Path）
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

    功能：
    - 按候选顺序，在模型真实 frame 名中找到第一个匹配项。

    为什么需要这一步：
    - 不同 URDF 里左脚末端的命名可能不同，不能把名字写死成一个。

    输入：
    - model：Pinocchio 模型
    - candidates：候选 frame 名列表

    输出：
    - frame_name（str）
    """
    frame_names = {frame.name for frame in model.frames}
    for name in candidates:
        if name in frame_names:
            return name

    raise RuntimeError("Target frame not found. Tried: " + ", ".join(candidates))


def main() -> None:
    """
    任务 4.3：左脚位置 IK 的一步速度解（one-step velocity command）骨架。

    学习重点（请先理解，再补新的 TODO）：
    1. 为什么要定义误差 e = p_des - p_cur
       - IK 的核心不是直接“猜关节怎么动”，而是先明确“末端当前位置离目标点还差多少”。
       - 这里的 e 是一个 3 维位置误差向量，表示当前左脚末端还需要沿 x/y/z 各移动多少。
       - 后面我们会把“末端应该怎么动”转成“关节应该怎么动”。

    2. 为什么位置 IK 使用 J_pos 而不是完整 J
       - 完整 frame Jacobian J 的尺寸通常是 6 x nv。
       - 前 3 行描述线速度（dx, dy, dz），后 3 行描述角速度（wx, wy, wz）。
       - 本任务只做“左脚位置”一步更新，不处理姿态误差，所以只取 J_pos = J[:3, :]。

    3. 为什么第一版使用阻尼最小二乘
       - 直接求逆通常要求矩阵方阵且满秩，但 J_pos 往往是 3 x nv，并不是方阵。
       - 末端靠近奇异位形时，直接伪逆也可能数值不稳定。
       - 阻尼最小二乘（damped least squares）会在求解中加入一个小的 damping，
         让一步速度解更稳定，也更适合作为第一版学习入口。

    4. v_step 的物理意义是什么
       - v_step 是“一步广义速度命令”或“一步关节/基座速度增量解”。
       - 它不是新的姿态 q_new，本质上回答的是：
         “如果现在想立刻朝着目标走一小步，各个广义自由度应该朝什么方向、以多大速度变化？”
       - 后续任务里，才会进一步学习怎么把 v_step 映射成 q 的更新。
    """

    # TODO 1: 解析 URDF
    # 这个块要做什么：确定后续建模使用的 URDF 文件路径。
    # 为什么需要这一步：Pinocchio 建模必须先拿到 URDF 路径。
    # 期望 API：resolve_urdf()
    # 输入：无
    # 输出：urdf_path（Path）
    urdf_path = resolve_urdf()

    # TODO 2: 构建浮动基模型
    # 这个块要做什么：从 URDF 构建 free-flyer 模型，并创建 data。
    # 为什么需要这一步：后续 FK、Jacobian、frame 位姿读取都依赖 model 和 data。
    # 期望 API：
    # - pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    # - model.createData()
    # 输入：urdf_path
    # 输出：model, data
    model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    data = model.createData()

    # TODO 3: neutral configuration
    # 这个块要做什么：生成当前模型的中立位形 q。
    # 为什么需要这一步：FK 与 Jacobian 都是在某个具体构型 q 下定义的。
    # 期望 API：pin.neutral(model)
    # 输入：model
    # 输出：q
    q = pin.neutral(model)

    # TODO 4: FK + updateFramePlacements
    # 这个块要做什么：在当前 q 下更新全模型 frame 位姿。
    # 为什么需要这一步：只有更新后，读取到的 p_cur 才对应当前构型。
    # 期望 API：
    # - pin.forwardKinematics(model, data, q)
    # - pin.updateFramePlacements(model, data)
    # 输入：model, data, q
    # 输出：data 中位姿被更新
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    # TODO 5: 选择左脚 frame
    # 这个块要做什么：从候选名中找到当前模型真实存在的左脚 frame，并拿到 frame_id。
    # 为什么需要这一步：不同 URDF 命名不统一，要先做兼容查找。
    # 期望 API：
    # - left_foot_frame = pick_frame(model, LEFT_FOOT_FRAME_CANDIDATES)
    # - frame_id = model.getFrameId(left_foot_frame)
    # 输入：model
    # 输出：left_foot_frame, frame_id
    left_foot_frame = pick_frame(model, LEFT_FOOT_FRAME_CANDIDATES)
    frame_id = model.getFrameId(left_foot_frame)

    # TODO 6: 读取 p_cur
    # 这个块要做什么：读取当前左脚 frame 的当前位置。
    # 为什么需要这一步：误差 e 必须由“目标位置 - 当前位置”得到。
    # 期望 API：data.oMf[frame_id].translation
    # 输入：data, frame_id
    # 输出：p_cur（shape=(3,)）
    p_cur = data.oMf[frame_id].translation.copy()

    # TODO 7: 定义 p_des
    # 这个块要做什么：先定义一个简单目标点，例如让左脚沿 x 方向前移 2cm。
    # 为什么需要这一步：一步 IK 解必须先有目标位置，才能谈误差与速度解。
    # 期望 API：np.array([0.02, 0.0, 0.0])
    # 输入：p_cur
    # 输出：p_des
    p_des = p_cur + np.array([0.02, 0.0, 0.0])

    # TODO 8: computeJointJacobians
    # 这个块要做什么：在当前 q 下计算关节 Jacobian 相关缓存。
    # 为什么需要这一步：后面的 getFrameJacobian 要依赖这些缓存结果。
    # 期望 API：pin.computeJointJacobians(model, data, q)
    # 输入：model, data, q
    # 输出：data 内 Jacobian 相关缓存被更新
    pin.computeJointJacobians(model, data, q)

    # TODO 9: getFrameJacobian
    # 这个块要做什么：读取左脚 frame 在当前构型下的完整 Jacobian。
    # 为什么需要这一步：我们先拿到完整 6 x nv Jacobian，再从中提取位置部分。
    # 期望 API：
    # - pin.getFrameJacobian(model, data, frame_id, pin.LOCAL_WORLD_ALIGNED)
    # 输入：model, data, frame_id
    # 输出：J（6 x nv）
    J = pin.getFrameJacobian(model, data, frame_id, pin.LOCAL_WORLD_ALIGNED)

    # TODO 10: 提取 J_pos
    # 这个块要做什么：从完整 Jacobian 中取出线速度部分。
    # 为什么需要这一步：
    # - 位置 IK 只关心末端点位置变化，不关心当前这一步的姿态误差。
    # - 因此只保留前 3 行，得到 J_pos = J[:3, :]。
    # 输入：J
    # 输出：J_pos（3 x nv）
    J_pos = J[:3, :]

    # TODO 11: 定义误差 e
    # 这个块要做什么：计算当前位置到目标位置的差值。
    # 为什么需要这一步：
    # - e = p_des - p_cur 是“末端还差多少”的直接表达。
    # - 后续一步速度解的目标，就是让 J_pos @ v_step 尽量逼近这个误差方向。
    # 期望 API：向量减法
    # 输入：p_des, p_cur
    # 输出：e（shape=(3,)）
    e = p_des - p_cur
    # TODO 12: 设置 gain / damping
    # 这个块要做什么：设置一步更新时的比例增益 gain 和阻尼系数 damping。
    # 为什么需要这一步：
    # - gain 决定“这一步想追误差追多快”。
    # - damping 决定“在奇异或病态方向上，解要多稳”。
    # 期望 API：普通标量，例如 gain = 0.5, damping = 1e-3
    # 输入：无
    # 输出：gain, damping
    gain = 0.5
    damping = 1e-3
    # TODO 13: 用阻尼最小二乘求 v_step
    # 这个块要做什么：根据位置误差 e 与 J_pos，求解一步广义速度 v_step。
    # 为什么第一版用阻尼最小二乘：
    # - J_pos 往往不是方阵，不能直接求逆。
    # - 加入 damping 后，数值更稳定，也更适合教学第一版。
    # 推荐你后续补的公式思路：
    # - 先定义期望末端线速度 v_des = gain * e
    # - 再用阻尼最小二乘解一个一步速度：
    #   v_step = J_pos.T @ inv(J_pos @ J_pos.T + damping^2 * I) @ v_des
    # 这里的物理意义：
    # - v_step 表示“当前这一小步里，各广义自由度应该怎么动”，
    #   它是速度解，不是新的姿态 q。
    # 期望 API：
    # - np.eye(3)
    # - 矩阵乘法 @
    # - np.linalg.inv(...)
    # 输入：J_pos, e, gain, damping
    # 输出：v_step（shape=(model.nv,)）
    v_des = gain * e
    v_step = J_pos.T @ np.linalg.inv(J_pos @ J_pos.T + damping**2 * np.eye(3)) @ v_des


    # TODO 14: 输出并保存到 outputs/ik/h1_left_foot_step3_one_step.txt
    # 这个块要做什么：把本步 one-step IK 的关键中间量整理后写入文件。
    # 为什么需要这一步：便于你检查误差、Jacobian 维度和一步速度解是否合理。
    # 推荐报告内容：
    # - urdf_path
    # - left_foot_frame
    # - model.nq, model.nv, model.njoints
    # - p_cur
    # - p_des
    # - e
    # - J.shape
    # - J_pos.shape
    # - gain, damping
    # - v_step
    # 期望 API：
    # - OUT_DIR.mkdir(parents=True, exist_ok=True)
    # - OUT_FILE.write_text(report, encoding="utf-8")
    report = (
    f"urdf_path: {urdf_path}/n"
    f"left_foot_frame: {left_foot_frame}\n"
    f"model.nq: {model.nq}\n"
    f"model.nv: {model.nv}\n"
    f"model.njoints: {model.njoints}\n"
    f"p_cur: {p_cur}\n"
    f"p_des: {p_des}\n"
    f"e: {e}\n"
    f"J.shape: {J.shape}\n"
    f"J_pos.shape: {J_pos.shape}\n"
    f"gain: {gain}, damping: {damping}\n"
    f"v_step: {v_step}\n"
    )
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    OUT_FILE.write_text(report, encoding="utf-8")
    print("[Skeleton] Task 4.3 created. Please fill TODO 11~14 step by step.")


if __name__ == "__main__":
    main()
