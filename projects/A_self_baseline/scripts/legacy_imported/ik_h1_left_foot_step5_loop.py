from __future__ import annotations

import csv
from pathlib import Path

import numpy as np
import pinocchio as pin


# =============================
# Step 5A: Pinocchio 左脚 IK 轨迹生成 TODO 骨架
# =============================
# 学习目标：
# - 从 H1 URDF 构建 Pinocchio 浮动基模型。
# - 使用 FK 读取左脚当前位置。
# - 使用 Jacobian + 阻尼最小二乘逐步靠近目标位置。
# - 保存 q_history，作为后续 MuJoCo 离线回放输入。
#
# 当前文件故意保留 TODO，不直接写满核心逻辑。
# 按 AGENTS.md：先理解每一步，再逐段补全。


# =============================
# TODO 1: 路径常量
# =============================
# 要实现什么：
# - 定位项目根目录 PROJECT_ROOT。
# - 定义输出目录 OUT_DIR。
# - 定义报告、误差 CSV、q 轨迹 NPY 的输出文件路径。
#
# 为什么需要：
# - 脚本不应该依赖 shell 当前工作目录。
# - q_traj.npy 后续会被 MuJoCo 回放脚本读取。
#
# 推荐 API：
# - Path(__file__).resolve().parents[1]
# - PROJECT_ROOT / "outputs" / "ik"
PROJECT_ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = PROJECT_ROOT / "outputs" / "ik"
REPORT_FILE = OUT_DIR / "h1_left_foot_step5_loop.txt"
ERROR_CSV_FILE = OUT_DIR / "h1_left_foot_step5_errors.csv"
Q_TRAJ_FILE = OUT_DIR / "h1_left_foot_step5_q_traj.npy"


# =============================
# TODO 2: URDF 候选路径
# =============================
# 要实现什么：
# - 兼容 unitree_ros 在 third_party 下，或直接在项目根目录下。
#
# 为什么需要：
# - 不同机器上模型目录可能不同。
# - 候选路径能减少硬编码路径带来的错误。
URDF_CANDIDATES = [
    PROJECT_ROOT / "third_party/unitree_ros/robots/h1_description/urdf/h1_with_hand.urdf",
    PROJECT_ROOT / "unitree_ros/robots/h1_description/urdf/h1_with_hand.urdf",
]


# =============================
# TODO 3: 左脚 frame 候选列表
# =============================
# 要实现什么：
# - 定义可能代表左脚/左踝的 frame 名。
#
# 为什么需要：
# - IK 控制目标必须是 Pinocchio model.frames 中的一个 frame。
# - 不同 URDF 版本可能命名略有不同。
LEFT_FOOT_FRAME_CANDIDATES = [
    "left_ankle_link",
    "left_ankle_joint",
    "left_foot",
    "left_sole",
]


def resolve_urdf() -> Path:
    """
    TODO 4: 从 URDF_CANDIDATES 中找到真实存在的 URDF。

    需要补什么：
    - 遍历 URDF_CANDIDATES。
    - 如果 path.exists()，返回该 path。
    - 如果都不存在，raise FileNotFoundError，并列出检查过的路径。

    推荐 API：
    - Path.exists()
    - "\\n".join(...)

    完成后应得到：
    - urdf_path: Path
    """
    for path in URDF_CANDIDATES:
        if path.exists():
            return path
    raise FileNotFoundError("None of the URDF candidates exist:\n" + "\n".join(str(p) for p in URDF_CANDIDATES))


def pick_frame(model: pin.Model, candidates: list[str]) -> str:
    """
    TODO 5: 从 model.frames 中选择左脚目标 frame。

    需要补什么：
    - frame_names = [frame.name for frame in model.frames]
    - 先做精确匹配。
    - 再做小写包含匹配。
    - 找不到时 raise RuntimeError，并输出候选名和部分可用 frame。

    为什么需要：
    - frame 选错，FK/Jacobian/IK 都会围绕错误目标计算。

    推荐 API：
    - model.frames
    - model.getFrameId(name)
    """
    frame_names = [frame.name for frame in model.frames]
    for candidate in candidates:
        if candidate in frame_names:
            return candidate
        if candidate.lower() in [name.lower() for name in frame_names]:
            return next(name for name in frame_names if name.lower() == candidate.lower())
    raise RuntimeError(f"None of the candidates found in model frames.\nCandidates: {candidates}\nAvailable frames: {frame_names}")


def format_vec(vec: np.ndarray) -> str:
    """
    TODO 6: 格式化向量，便于写入报告。

    需要补什么：
    - 使用 np.array2string(vec, precision=8, suppress_small=False)。
    """
    return np.array2string(vec, precision=8, suppress_small=False)
    


def main() -> None:
    """
    Step 5A 主流程 TODO。

    输入：
    - H1 URDF
    - neutral configuration
    - 左脚目标位置 p_des

    输出：
    - outputs/ik/h1_left_foot_step5_loop.txt
    - outputs/ik/h1_left_foot_step5_errors.csv
    - outputs/ik/h1_left_foot_step5_q_traj.npy
    """
    # =============================
    # TODO 7: 创建输出目录
    # =============================
    # 需要补什么：
    # - OUT_DIR.mkdir(parents=True, exist_ok=True)
    #
    # 为什么需要：
    # - 后续保存报告、CSV、NPY 文件前，目录必须存在。
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    # =============================
    # TODO 8: 解析 URDF 路径
    # =============================
    # 需要补什么：
    # - urdf_path = resolve_urdf()
    #
    # 完成后应得到：
    # - urdf_path: Path
    urdf_path = resolve_urdf()
    # =============================
    # TODO 9: 构建 Pinocchio 浮动基模型
    # =============================
    # 需要补什么：
    # - model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    # - data = model.createData()
    #
    # 为什么使用 JointModelFreeFlyer：
    # - H1 是人形机器人，基座不是固定在世界中的机械臂。
    # - 浮动基模型会把 pelvis/base 的空间运动也纳入配置。
    model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    data = model.createData()
    # =============================
    # TODO 10: 获取 neutral configuration
    # =============================
    # 需要补什么：
    # - q = pin.neutral(model)
    #
    # 为什么不用全零向量：
    # - 浮动基姿态通常包含四元数，neutral 能保证配置合法。
    q = pin.neutral(model)
    # =============================
    # TODO 11: 选择左脚 frame
    # =============================
    # 需要补什么：
    # - target_frame = pick_frame(model, LEFT_FOOT_FRAME_CANDIDATES)
    # - frame_id = model.getFrameId(target_frame)
    #
    # 完成后应得到：
    # - target_frame: str
    # - frame_id: int
    target_frame = pick_frame(model, LEFT_FOOT_FRAME_CANDIDATES)
    frame_id = model.getFrameId(target_frame)
    # =============================
    # TODO 12: 初始 FK，读取 p0
    # =============================
    # 需要补什么：
    # - pin.forwardKinematics(model, data, q)
    # - pin.updateFramePlacements(model, data)
    # - p0 = data.oMf[frame_id].translation.copy()
    #
    # 为什么先做 FK：
    # - data.oMf 不会自动随 q 更新。
    # - 必须先根据当前 q 更新运动学结果，再读取 frame 位置。
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    p0 = data.oMf[frame_id].translation.copy()
    # =============================
    # TODO 13: 定义目标位置 p_des
    # =============================
    # 需要补什么：
    # - p_des = p0 + np.array([0.02, 0.0, 0.0])
    #
    # 为什么这样定义：
    # - 第一版只让左脚沿世界 x 方向移动 2 cm。
    # - 目标小且简单，便于观察误差收敛。
    p_des = p0 + np.array([0.02, 0.0, 0.0])
    # =============================
    # TODO 14: 设置 IK 参数
    # =============================
    # 需要补什么：
    # - max_iter = 200
    # - dt = 0.1
    # - gain = 1.0
    # - damping = 1e-6
    # - tol = 1e-4
    #
    # 参数含义：
    # - max_iter: 最大迭代次数
    # - dt: 每次配置更新步长
    # - gain: 误差反馈增益
    # - damping: 阻尼最小二乘稳定项
    # - tol: 停止阈值
    max_iter = 200
    dt = 0.1
    gain = 1.0
    damping = 1e-6
    tol = 1e-4
    # =============================
    # TODO 15: 初始化历史记录
    # =============================
    # 需要补什么：
    # - error_history = []
    # - q_history = [q.copy()]
    # - converged = False
    #
    # 为什么 q_history 要用 q.copy()：
    # - q 是可变 NumPy 数组。
    # - copy() 保存每一帧的独立快照。
    # - 后续 MuJoCo 回放会逐帧读取这些 q。
    error_history = []
    q_history = [q.copy()]
    converged = False
    # =============================
    # TODO 16: 编写 IK 主循环的 FK 和误差部分
    # =============================
    # 需要补什么：
    # - for _ in range(max_iter):
    # - 每轮 forwardKinematics
    # - 每轮 updateFramePlacements
    # - 读取 p_cur
    # - e = p_des - p_cur
    # - err_norm = np.linalg.norm(e)
    # - error_history.append(err_norm)
    # - err_norm < tol 时停止
    #
    # 为什么 IK 要循环：
    # - Jacobian 是当前位置附近的线性近似。
    # - 一次更新通常只能靠近目标，不能保证直接到达。
    #
    # 为什么每轮都重新 FK：
    # - q 每轮都会变化，左脚当前位置 p_cur 也会变化。
    for _ in range(max_iter):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        p_cur = data.oMf[frame_id].translation.copy()
        e = p_des - p_cur
        err_norm = np.linalg.norm(e)
        error_history.append(err_norm)
        if err_norm < tol:
            converged = True
            break
    # =============================
    # TODO 17: 编写 IK 主循环的 Jacobian 部分
    # =============================
    # 需要补什么：
    # - pin.computeJointJacobians(model, data, q)
    # - J = pin.getFrameJacobian(model, data, frame_id, pin.LOCAL_WORLD_ALIGNED)
    # - J_pos = J[:3, :]
    #
    # 为什么每轮都重新 Jacobian：
    # - Jacobian 依赖当前 q。
    # - 姿态变化后，关节速度到左脚速度的映射也会变化。
    #
    # 为什么 J_pos 只取前 3 行：
    # - 本任务只控制左脚位置，不控制左脚姿态。
    # - Pinocchio frame Jacobian 前 3 行是线速度部分。
        pin.computeJointJacobians(model, data, q)
        J = pin.getFrameJacobian(model, data, frame_id, pin.LOCAL_WORLD_ALIGNED)
        J_pos = J[:3, :]
    # =============================
    # TODO 18: 阻尼最小二乘求 v_step，并更新 q
    # =============================
    # 需要补什么：
    # - A = J_pos @ J_pos.T + damping * np.eye(3)
    # - b = gain * e
    # - v_step = J_pos.T @ np.linalg.solve(A, b)
    # - q = pin.integrate(model, q, v_step * dt)
    # - q_history.append(q.copy())
    #
    # 为什么用 pin.integrate 而不是 q + v * dt：
    # - 浮动基 q 中包含四元数。
    # - 普通加法可能破坏四元数单位长度。
    # - pin.integrate 会按配置空间规则更新 q。
        A = J_pos @ J_pos.T + damping * np.eye(3)
        b = gain * e
        v_step = J_pos.T @ np.linalg.solve(A, b)
        q = pin.integrate(model, q, v_step * dt)
        q_history.append(q.copy())
    # =============================
    # TODO 19: 循环结束后计算最终结果
    # =============================
    # 需要补什么：
    # - 对最终 q 再做 FK。
    # - 读取 p_final。
    # - 计算 err_initial、err_final、iter_used。
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    p_final = data.oMf[frame_id].translation.copy()
    # =============================
    # TODO 20: 保存 q_traj.npy
    # =============================
    # 需要补什么：
    # - q_traj = np.stack(q_history, axis=0)
    # - print(q_traj.shape)
    # - np.save(Q_TRAJ_FILE, q_traj)
    #
    # 为什么 q_traj.npy 是 MuJoCo 回放输入：
    # - MuJoCo 的 data.qpos 就是一帧广义位置。
    # - q_traj 每一行对应一帧 data.qpos。
    q_traj = np.stack(q_history, axis=0)
    print(q_traj.shape)
    print(f"Saving q trajectory to {Q_TRAJ_FILE}")
    np.save(Q_TRAJ_FILE, q_traj)
    # =============================
    # TODO 21: 保存 error_history CSV
    # =============================
    # 需要补什么：
    # - 使用 csv.writer 写入 iter,error_norm。
    #
    # 为什么需要：
    # - 后续可以画 IK 误差收敛曲线。
    with open(ERROR_CSV_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["iter", "error_norm"])
        for i, err in enumerate(error_history):
            writer.writerow([i, err])
    # =============================
    # TODO 22: 保存文本报告
    # =============================
    # 需要补什么：
    # - URDF 路径
    # - target frame
    # - nq / nv / njoints / nframes
    # - p0 / p_des / p_final
    # - err_initial / err_final
    # - max_iter / iter_used / dt / gain / damping / tol
    # - converged
    # - q_traj shape
    report_lines = [
        f"URDF path: {urdf_path}",
        f"Target frame: {target_frame} (id={frame_id})",
        f"Model dofs: nq={model.nq}, nv={model.nv}, njoints={model.njoints}, nframes={model.nframes}",
        f"Initial position p0: {format_vec(p0)}",
        f"Desired position p_des: {format_vec(p_des)}",
        f"Final position p_final: {format_vec(p_final)}",
        f"Initial error norm: {error_history[0]:.6f}",
        f"Final error norm: {error_history[-1]:.6f}",
        f"Max iterations: {max_iter}",
        f"Iterations used: {len(error_history)}",
        f"Time step dt: {dt}",
        f"Gain: {gain}",
        f"Damping: {damping}",
        f"Tolerance: {tol}",
        f"Converged: {converged}",
        f"q_traj shape: {q_traj.shape}",
        f"q_traj saved to: {Q_TRAJ_FILE}"
        
    ]
    report = "\n".join(report_lines)
    REPORT_FILE.write_text(report + "\n", encoding="utf-8")
    print(report)

   


if __name__ == "__main__":
    main()
