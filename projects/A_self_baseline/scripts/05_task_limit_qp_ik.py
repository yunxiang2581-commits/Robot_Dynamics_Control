"""A05 task + limit + QP-IK minimal implementation.

Pipeline 步骤：
- A05 task + limit + QP-IK。

对标 mink 的概念：
- FrameTask：把末端 site 的位置 / 姿态误差写成任务残差。
- PostureTask：让 q 保持接近参考姿态。
- ConfigurationLimit：限制积分后的 q 不越过关节位置边界。
- VelocityLimit：限制每一步 dq 的速度边界。
- QP-based differential IK：把任务误差、阻尼和 box constraints 写成二次规划。

与 A04 的关系：
- A04 是无约束 DLS differential IK。
- A05 复用 A04 的目标定义和 A04 q trajectory 作为对照，加入 task weight、
  posture task、velocity limit 和 joint position limit。

本脚本输入：
- `projects/A_self_baseline/configs/robot.yaml`。
- `projects/A_self_baseline/configs/qp_ik.yaml`。
- `projects/A_self_baseline/outputs/cache/A01_model_summary.json`。
- `projects/A_self_baseline/outputs/cache/A02_site_pose.json`。
- `projects/A_self_baseline/outputs/cache/A03_jacobian_check.json`。
- `projects/A_self_baseline/outputs/trajectories/A04_dls_ik_q_traj.npy`。
- target site、target offset、joint limits、velocity limits、posture reference。

本脚本输出：
- `outputs/trajectories/A05_qp_ik_q_traj.npy`。
- `outputs/logs/A05_qp_ik_error.csv`。
- `outputs/logs/A05_qp_ik_constraints.csv`。
- `outputs/figures/A05_qp_ik_error.png`。
- `outputs/reports/A05_qp_ik_report.md`。

当前边界：
- 不调用 mink 替代自己的实现。
- 不做 actuator tracking / MuJoCo 控制 / collision avoidance / video recording。
- 只生成受约束 IK 轨迹和诊断输出，A06/A07/A08 后续再消费这些结果。
"""

from __future__ import annotations

import argparse
import csv
import json
import logging
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import mujoco
import numpy as np
from scipy import sparse
from scipy.optimize import Bounds, minimize
from scipy.spatial.transform import Rotation


A_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROBOT_CONFIG = A_ROOT / "configs" / "robot.yaml"
DEFAULT_QP_IK_CONFIG = A_ROOT / "configs" / "qp_ik.yaml"
DEFAULT_A01_SUMMARY = A_ROOT / "outputs" / "cache" / "A01_model_summary.json"
DEFAULT_A02_POSE = A_ROOT / "outputs" / "cache" / "A02_site_pose.json"
DEFAULT_A03_JACOBIAN = A_ROOT / "outputs" / "cache" / "A03_jacobian_check.json"
DEFAULT_A04_TRAJECTORY = A_ROOT / "outputs" / "trajectories" / "A04_dls_ik_q_traj.npy"
DEFAULT_OUTPUT_DIR = A_ROOT / "outputs"
DEFAULT_SITE_NAME = "attachment_site"

SRC_ROOT = A_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.append(str(SRC_ROOT))

from robot_baseline import model_loader  # noqa: E402


# =============================
# TODO 0: CLI 和通用小工具
# =============================
# 这些函数不属于具体数学任务，只负责解析参数、读取文件和处理路径。
# 学习重点：让 A05 不依赖 shell 当前目录，并让错误信息足够清楚。
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A05: task + limit + QP-IK.")
    parser.add_argument("--robot-config", default=str(DEFAULT_ROBOT_CONFIG), help="robot.yaml 路径。")
    parser.add_argument("--qp-ik-config", default=str(DEFAULT_QP_IK_CONFIG), help="qp_ik.yaml 路径。")
    parser.add_argument("--a01-summary", default=str(DEFAULT_A01_SUMMARY), help="A01 model summary。")
    parser.add_argument("--a02-pose", default=str(DEFAULT_A02_POSE), help="A02 site pose cache。")
    parser.add_argument("--a03-jacobian", default=str(DEFAULT_A03_JACOBIAN), help="A03 Jacobian check cache。")
    parser.add_argument("--a04-trajectory", default=str(DEFAULT_A04_TRAJECTORY), help="A04 DLS IK q trajectory。")
    parser.add_argument("--mjcf", default=None, help="可选：覆盖 robot.yaml 中的 scene.xml 路径。")
    parser.add_argument("--site", default=DEFAULT_SITE_NAME, help="A03/A04 使用的目标 site。")
    parser.add_argument("--task-mode", choices=("position", "pose_6d"), default=None)
    parser.add_argument("--solver", choices=("scipy", "osqp"), default=None, help="QP solver 后端。")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR), help="A05 输出根目录。")
    parser.add_argument("--log-level", default="INFO", help="日志级别，例如 INFO / DEBUG。")
    return parser.parse_args()


def read_json(path: Path) -> dict:
    """读取 JSON 前置产物，保持错误信息清楚。"""
    if not path.exists():
        raise FileNotFoundError(f"找不到前置产物: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def parse_vector(value: object, *, length: int, name: str) -> np.ndarray:
    """把 YAML/CLI 中的向量统一转成 numpy array。"""
    if isinstance(value, str):
        parts = [float(v.strip()) for v in value.split(",")]
    else:
        parts = [float(v) for v in value]
    vector = np.asarray(parts, dtype=np.float64)
    if vector.shape != (length,):
        raise ValueError(f"{name} 维度错误: {vector.shape}, 期望 ({length},)")
    return vector


def as_output_root(path_value: str | Path) -> Path:
    """解析输出根目录；相对路径默认相对 A 项目根目录。"""
    path = Path(path_value).expanduser()
    if path.is_absolute():
        return path
    return A_ROOT / path


# =============================
# TODO 4: 定义 FrameTask
# =============================
# 以下 helper 都服务于 FrameTask：
# - site_pose: 从当前 q 对应的 MuJoCo data 中读取 site pose。
# - rotation_error_vector: 用 SO(3) log map 计算姿态误差。
# - build_frame_task: 输出加权后的 e_frame 和 J_frame。
def site_pose(model: mujoco.MjModel, data: mujoco.MjData, site_id: int) -> tuple[np.ndarray, np.ndarray]:
    """读取当前 site 的位置和旋转矩阵。"""
    mujoco.mj_forward(model, data)
    position = data.site_xpos[site_id].copy()
    rotation = data.site_xmat[site_id].reshape(3, 3).copy()
    return position, rotation


def rotation_error_vector(target_rotation: np.ndarray, current_rotation: np.ndarray) -> np.ndarray:
    """计算 SO(3) log map 形式的姿态误差。

    这里使用 `R_err = R_target R_current^T`，返回 rotation vector。
    """
    rotation_delta = target_rotation @ current_rotation.T
    return Rotation.from_matrix(rotation_delta).as_rotvec()


def build_frame_task(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    site_id: int,
    target_position: np.ndarray,
    target_rotation: np.ndarray,
    *,
    task_mode: str,
    position_weight: float,
    orientation_weight: float,
) -> tuple[np.ndarray, np.ndarray, float, float]:
    """构造 FrameTask 的加权误差和 Jacobian。

    输入：
    - 当前 q 已经写入 data.qpos。
    - target_position / target_rotation 是 A05 的目标 site pose。

    输出：
    - J_frame: position mode 为 (3, nv)，pose_6d mode 为 (6, nv)。
    - e_frame: 与 J_frame 行数一致。
    """
    current_position, current_rotation = site_pose(model, data, site_id)

    e_pos = target_position - current_position
    e_rot = rotation_error_vector(target_rotation, current_rotation)

    jacp = np.zeros((3, model.nv), dtype=np.float64)
    jacr = np.zeros((3, model.nv), dtype=np.float64)
    mujoco.mj_jacSite(model, data, jacp, jacr, site_id)

    if task_mode == "position":
        e_frame = position_weight * e_pos
        J_frame = position_weight * jacp
    elif task_mode == "pose_6d":
        e_frame = np.concatenate(
            [
                position_weight * e_pos,
                orientation_weight * e_rot,
            ]
        )
        J_frame = np.vstack(
            [
                position_weight * jacp,
                orientation_weight * jacr,
            ]
        )
    else:
        raise ValueError(f"不支持的 task_mode: {task_mode}")

    if J_frame.shape[0] != e_frame.shape[0]:
        raise ValueError(f"FrameTask 维度不一致: J={J_frame.shape}, e={e_frame.shape}")
    if J_frame.shape[1] != model.nv:
        raise ValueError(f"FrameTask Jacobian 列数不是 nv: {J_frame.shape}")

    return J_frame, e_frame, float(np.linalg.norm(e_pos)), float(np.linalg.norm(e_rot))


# =============================
# TODO 5: 定义 PostureTask
# =============================
# 这段代码实现 e_posture = q_ref - q，以及 J_posture = I。
# A05 中 posture task 是软任务，进入 QP 目标函数，不作为硬约束。
def build_posture_task(q: np.ndarray, q_ref: np.ndarray, posture_weight: float, nv: int) -> tuple[np.ndarray, np.ndarray]:
    """构造 PostureTask。

    对固定基 UR5e，`nq == nv == 6`，这里可直接使用 identity 近似。
    """
    e_posture = posture_weight * (q_ref - q)
    J_posture = posture_weight * np.eye(nv)
    if e_posture.shape != (nv,):
        raise ValueError(f"PostureTask e_posture shape 错误: {e_posture.shape}")
    return J_posture, e_posture


# =============================
# TODO 7: 构造 QP 目标函数
# =============================
# 这段代码把 J_task dq ≈ v_task 展开成标准 QP：
# minimize 1/2 dq^T H dq + c^T dq
def build_qp_objective(J_task: np.ndarray, v_task: np.ndarray, regularization: float) -> tuple[np.ndarray, np.ndarray]:
    """从 least-squares 任务构造标准 QP 的 H 和 c。

    使用约定：
    minimize 1/2 dq^T H dq + c^T dq

    对应：
    H = J_task.T @ J_task + regularization * I
    c = -J_task.T @ v_task
    """
    nv = J_task.shape[1]
    H = J_task.T @ J_task + regularization * np.eye(nv)
    c = -J_task.T @ v_task
    H = 0.5 * (H + H.T)

    if H.shape != (nv, nv):
        raise ValueError(f"H shape 错误: {H.shape}")
    if c.shape != (nv,):
        raise ValueError(f"c shape 错误: {c.shape}")
    return H, c


# =============================
# TODO 8: 定义 VelocityLimit
# =============================
# 这段代码把配置中的速度上限转换为 dq 的 box bounds。
# A05 统一约定 dq 是 joint velocity，单位 rad/s。
def build_velocity_bounds(velocity_limit: float | list[float], nv: int) -> tuple[np.ndarray, np.ndarray]:
    """构造 VelocityLimit。

    A05 统一把 dq 解释为 joint velocity，单位 rad/s。
    """
    if isinstance(velocity_limit, (list, tuple)):
        velocity_upper = np.asarray(velocity_limit, dtype=np.float64)
        if velocity_upper.shape != (nv,):
            raise ValueError(f"velocity_limit shape 错误: {velocity_upper.shape}")
    else:
        velocity_upper = float(velocity_limit) * np.ones(nv)
    velocity_lower = -velocity_upper

    if np.any(velocity_upper <= 0):
        raise ValueError(f"velocity_limit 必须为正数: {velocity_upper}")
    return velocity_lower, velocity_upper


# =============================
# TODO 9: 定义 JointPositionLimit
# =============================
# 这段代码把 q_min <= q + dq * dt <= q_max 转换成 dq 的上下界。
def build_position_bounds(
    model: mujoco.MjModel,
    q: np.ndarray,
    dt: float,
    position_margin: float,
) -> tuple[np.ndarray, np.ndarray]:
    """把 JointPositionLimit 转换成 dq 的上下界。

    q_min <= q + dq * dt <= q_max
    等价于：
    (q_min - q) / dt <= dq <= (q_max - q) / dt
    """
    q_min = model.jnt_range[:, 0].astype(np.float64).copy()
    q_max = model.jnt_range[:, 1].astype(np.float64).copy()
    limited = model.jnt_limited.astype(bool)

    position_lower = np.full(model.nv, -np.inf, dtype=np.float64)
    position_upper = np.full(model.nv, np.inf, dtype=np.float64)

    for joint_index in range(model.njnt):
        dof_adr = model.jnt_dofadr[joint_index]
        qpos_adr = model.jnt_qposadr[joint_index]
        if dof_adr < 0 or qpos_adr < 0:
            continue
        if not limited[joint_index]:
            continue
        lower_q = q_min[joint_index] + position_margin
        upper_q = q_max[joint_index] - position_margin
        if lower_q > upper_q:
            raise ValueError(
                f"position_margin={position_margin} 过大，joint {joint_index} 可行区间为空"
            )
        position_lower[dof_adr] = (lower_q - q[qpos_adr]) / dt
        position_upper[dof_adr] = (upper_q - q[qpos_adr]) / dt

    return position_lower, position_upper


# =============================
# TODO 10: 构造 QP 约束
# =============================
# 这段代码合并 VelocityLimit 和 JointPositionLimit：
# lower = max(velocity_lower, position_lower)
# upper = min(velocity_upper, position_upper)
def merge_bounds(
    velocity_lower: np.ndarray,
    velocity_upper: np.ndarray,
    position_lower: np.ndarray,
    position_upper: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    lower = np.maximum(velocity_lower, position_lower)
    upper = np.minimum(velocity_upper, position_upper)
    if np.any(lower > upper):
        bad = np.where(lower > upper)[0].tolist()
        raise ValueError(f"VelocityLimit 与 JointPositionLimit 冲突，bad dof={bad}")
    return lower, upper


# =============================
# TODO 11: 选择 QP solver
# =============================
# TODO 11 的实现必须同时支持 scipy.optimize 和 osqp。
# 两个 backend 都求解同一个 box-constrained QP，只是输入格式不同。
def solve_qp_scipy(H: np.ndarray, c: np.ndarray, lower: np.ndarray, upper: np.ndarray, max_iter: int) -> tuple[np.ndarray, str]:
    """使用 scipy.optimize.minimize 求解 box-constrained QP。"""
    bounds = Bounds(lower, upper)

    def objective(dq_candidate: np.ndarray) -> float:
        return float(0.5 * dq_candidate @ H @ dq_candidate + c @ dq_candidate)

    def gradient(dq_candidate: np.ndarray) -> np.ndarray:
        return H @ dq_candidate + c

    result = minimize(
        objective,
        x0=np.zeros(H.shape[0], dtype=np.float64),
        jac=gradient,
        method="SLSQP",
        bounds=bounds,
        options={"maxiter": max_iter, "ftol": 1e-9},
    )
    if not result.success:
        raise RuntimeError(f"scipy QP solver failed: {result.message}")
    return np.asarray(result.x, dtype=np.float64), f"scipy:{result.message}"


def solve_qp_osqp(
    H: np.ndarray,
    c: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    *,
    max_iter: int,
    eps_abs: float,
    eps_rel: float,
) -> tuple[np.ndarray, str]:
    """使用 OSQP 求解 box-constrained QP。"""
    try:
        import osqp
    except ImportError as exc:
        raise ImportError("选择了 OSQP solver，但当前 Python 环境没有安装 osqp。") from exc

    nv = H.shape[0]
    problem = osqp.OSQP()
    problem.setup(
        P=sparse.csc_matrix(H),
        q=np.asarray(c, dtype=np.float64),
        A=sparse.eye(nv, format="csc"),
        l=lower,
        u=upper,
        verbose=False,
        max_iter=max_iter,
        eps_abs=eps_abs,
        eps_rel=eps_rel,
    )
    result = problem.solve()
    status = result.info.status
    if "solved" not in status.lower():
        raise RuntimeError(f"OSQP solver failed: {status}")
    return np.asarray(result.x, dtype=np.float64), f"osqp:{status}"


def solve_box_qp(
    H: np.ndarray,
    c: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    *,
    solver_name: str,
    max_iter: int,
    eps_abs: float,
    eps_rel: float,
) -> tuple[np.ndarray, str]:
    """统一封装 TODO 11 要求的 scipy.optimize / osqp 两种求解器。"""
    if solver_name == "scipy":
        dq, status = solve_qp_scipy(H, c, lower, upper, max_iter)
    elif solver_name == "osqp":
        dq, status = solve_qp_osqp(
            H,
            c,
            lower,
            upper,
            max_iter=max_iter,
            eps_abs=eps_abs,
            eps_rel=eps_rel,
        )
    else:
        raise ValueError(f"不支持的 QP solver: {solver_name}")

    if dq.shape != (H.shape[0],):
        raise ValueError(f"dq shape 错误: {dq.shape}")
    if not np.all(np.isfinite(dq)):
        raise ValueError(f"dq 中存在 NaN 或 inf: {dq}")
    return dq, status


# =============================
# TODO 13: 记录约束 violation
# =============================
# 这段代码计算 dq 是否违反 lower <= dq <= upper。
# 它服务于 A05_qp_ik_constraints.csv 和报告中的 max constraint violation。
def constraint_violation(dq: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> float:
    lower_violation = np.maximum(lower - dq, 0.0)
    upper_violation = np.maximum(dq - upper, 0.0)
    return float(max(np.max(lower_violation), np.max(upper_violation)))


# =============================
# TODO 14: 规划误差图输出
# =============================
# 这段代码输出 A05_qp_ik_error.png，并在 A04 日志存在时叠加 A04 DLS 曲线。
def write_error_plot(error_rows: list[dict], figure_path: Path, a04_log_path: Path) -> None:
    figure_path.parent.mkdir(parents=True, exist_ok=True)
    iterations = [row["iteration"] for row in error_rows]
    position_errors = [row["position_error_norm"] for row in error_rows]
    task_errors = [row["task_error_norm"] for row in error_rows]
    posture_errors = [row["posture_error_norm"] for row in error_rows]

    plt.figure(figsize=(8, 5))
    plt.plot(iterations, position_errors, marker="o", label="A05 position error")
    plt.plot(iterations, task_errors, marker="s", label="A05 task error")
    plt.plot(iterations, posture_errors, marker="^", label="A05 posture error")

    if a04_log_path.exists():
        a04_iterations = []
        a04_position_errors = []
        with a04_log_path.open("r", encoding="utf-8", newline="") as f:
            for row in csv.DictReader(f):
                a04_iterations.append(int(row["iteration"]))
                a04_position_errors.append(float(row["position_error_norm"]))
        plt.plot(a04_iterations, a04_position_errors, linestyle="--", label="A04 DLS position error")

    plt.xlabel("iteration")
    plt.ylabel("error norm")
    plt.title("A05 QP-IK error")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(figure_path, dpi=150)
    plt.close()


def write_csv_rows(path: Path, rows: list[dict]) -> None:
    if not rows:
        raise RuntimeError(f"没有可写入的日志行: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


# =============================
# TODO 15: 规划 Markdown report 输出
# =============================
# 这段代码写出 A05_qp_ik_report.md，用于复盘 QP 形式、收敛结果和输出产物。
def write_report(
    report_path: Path,
    *,
    solver_name: str,
    task_mode: str,
    target_site: str,
    target_offset: np.ndarray,
    q_traj: np.ndarray,
    error_rows: list[dict],
    constraint_rows: list[dict],
    output_paths: dict[str, Path],
    converged: bool,
    stop_reason: str,
    final_position_error: float,
    dt: float,
    max_iters: int,
    tolerance: float,
    regularization: float,
    velocity_limit: object,
    position_margin: float,
    posture_weight: float,
) -> None:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    initial_error = error_rows[0]["position_error_norm"]
    max_constraint = max(row["max_constraint_violation"] for row in constraint_rows)

    lines = [
        "# A05 Task + Limit + QP-IK Report",
        "",
        "## 目标",
        "",
        "- 从 A04 的无约束 DLS IK 过渡到带 task weight 和 limit 的 QP-IK。",
        "- 第一版只生成受约束 IK 轨迹，不进入 actuator tracking、MuJoCo 控制、collision avoidance 或 video。",
        "",
        "## 输入与配置",
        "",
        f"- target site: {target_site}",
        f"- task mode: {task_mode}",
        f"- solver: {solver_name}",
        f"- target offset: {target_offset.tolist()}",
        f"- dt: {dt}",
        f"- max_iters: {max_iters}",
        f"- tolerance: {tolerance}",
        f"- regularization: {regularization}",
        f"- velocity_limit: {velocity_limit}",
        f"- position_margin: {position_margin}",
        f"- posture_weight: {posture_weight}",
        "",
        "## QP 形式",
        "",
        "```text",
        "minimize 1/2 dq^T H dq + c^T dq",
        "subject to lower <= dq <= upper",
        "",
        "H = J_task.T @ J_task + regularization * I",
        "c = -J_task.T @ v_task",
        "```",
        "",
        "## 结果",
        "",
        f"- converged: {converged}",
        f"- stop_reason: {stop_reason}",
        f"- initial position error: {initial_error}",
        f"- final position error: {final_position_error}",
        f"- q trajectory shape: {q_traj.shape}",
        f"- max constraint violation: {max_constraint}",
        "",
        "## 输出文件",
        "",
        f"- trajectory: {output_paths['trajectory']}",
        f"- error log: {output_paths['error_log']}",
        f"- constraint log: {output_paths['constraint_log']}",
        f"- error figure: {output_paths['figure']}",
        "",
        "## 与 A04 / A06 / A07 的关系",
        "",
        "- A04 提供无约束 DLS baseline。",
        "- A05 输出受约束 QP-IK q trajectory。",
        "- A06 后续可以把 A05 trajectory 作为 target tracking 输入。",
        "- A07 后续才把 trajectory 送入 actuator tracking。",
        "",
    ]
    report_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(levelname)s: %(message)s",
    )

    # TODO 1: 读取 A01/A02/A03/A04 前置产物。
    # 这一步确保 A05 继续沿用已经验证过的模型、site、Jacobian 和 A04 baseline。
    model_summary = read_json(Path(args.a01_summary))
    pose_summary = read_json(Path(args.a02_pose))
    jacobian_summary = read_json(Path(args.a03_jacobian))
    a04_q_traj = np.load(Path(args.a04_trajectory))

    nq = int(model_summary["nq"])
    nv = int(model_summary["nv"])
    nu = int(model_summary["nu"])
    if a04_q_traj.ndim != 2 or a04_q_traj.shape[1] != nq:
        raise ValueError(f"A04 trajectory shape 错误: {a04_q_traj.shape}, 期望 (N, {nq})")

    # TODO 2: 读取 robot.yaml / qp_ik.yaml。
    robot_config = model_loader.load_yaml_config(args.robot_config)
    qp_ik_config = model_loader.load_yaml_config(args.qp_ik_config)

    mjcf_path = model_loader.resolve_path(args.mjcf or robot_config["mjcf_path"], A_ROOT)
    weights = qp_ik_config.get("weights", {})
    limits = qp_ik_config.get("limits", {})
    solver_config = qp_ik_config.get("solver", {})
    model_config = qp_ik_config.get("model", {})
    output_config = qp_ik_config.get("output", {})

    target_site = model_config.get("target_site", args.site)
    task_mode = args.task_mode or model_config.get("task_mode", "position")
    target_offset = parse_vector(model_config.get("target_offset", [0.03, 0.0, 0.0]), length=3, name="target_offset")

    position_weight = float(weights.get("position", weights.get("task", 1.0)))
    orientation_weight = float(weights.get("orientation", 0.2))
    posture_weight = float(weights.get("posture", 0.1))
    regularization = float(weights.get("regularization", solver_config.get("damping", 1e-4)))

    velocity_limit = limits.get("velocity_limit", 0.5)
    position_margin = float(limits.get("position_margin", 0.05))

    solver_name = (args.solver or solver_config.get("name", "scipy")).lower()
    dt = float(solver_config.get("dt", 0.02))
    max_iters = int(solver_config.get("max_iters", solver_config.get("max_iter", 100)))
    solver_max_iter = int(solver_config.get("max_iter", 1000))
    tolerance = float(solver_config.get("tolerance", 1e-3))
    gain = float(solver_config.get("gain", 1.0))
    eps_abs = float(solver_config.get("eps_abs", 1e-4))
    eps_rel = float(solver_config.get("eps_rel", 1e-4))

    if dt <= 0:
        raise ValueError(f"dt 必须 > 0，当前是 {dt}")
    if max_iters < 1:
        raise ValueError(f"max_iters 必须 >= 1，当前是 {max_iters}")
    if tolerance <= 0:
        raise ValueError(f"tolerance 必须 > 0，当前是 {tolerance}")
    if regularization < 0:
        raise ValueError(f"regularization 必须 >= 0，当前是 {regularization}")

    output_root = as_output_root(args.output_dir)
    if "directory" in output_config:
        logging.info("qp_ik.yaml output.directory=%s；A05 标准产物仍写入 outputs/{trajectories,logs,figures,reports}", output_config["directory"])

    trajectory_path = output_root / "trajectories" / "A05_qp_ik_q_traj.npy"
    error_log_path = output_root / "logs" / "A05_qp_ik_error.csv"
    constraint_log_path = output_root / "logs" / "A05_qp_ik_constraints.csv"
    figure_path = output_root / "figures" / "A05_qp_ik_error.png"
    report_path = output_root / "reports" / "A05_qp_ik_report.md"

    # TODO 3: 加载 MuJoCo model / data。
    model = model_loader.load_mujoco_model(mjcf_path)
    data = mujoco.MjData(model)
    if (model.nq, model.nv, model.nu) != (nq, nv, nu):
        raise ValueError(
            f"MuJoCo model 维度与 A01 不一致: model={(model.nq, model.nv, model.nu)}, A01={(nq, nv, nu)}"
        )
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, target_site)
    if site_id == -1:
        raise ValueError(f"找不到 target site: {target_site}")
    if jacobian_summary.get("site_name") != target_site:
        raise ValueError(f"A03 site={jacobian_summary.get('site_name')} 与 A05 target_site={target_site} 不一致")

    # 起点沿用 A04 / A02 的初始 q，目标沿用 A04 风格的小位移。
    q = np.asarray(a04_q_traj[0], dtype=np.float64).copy()
    q_ref = q.copy()
    data.qpos[:] = q
    current_position, current_rotation = site_pose(model, data, site_id)
    target_position = current_position + target_offset
    target_rotation = np.asarray(pose_summary.get("site_rotation_matrix"), dtype=np.float64)
    if target_rotation.shape != (3, 3):
        target_rotation = current_rotation.copy()

    q_traj = [q.copy()]
    error_rows: list[dict] = []
    constraint_rows: list[dict] = []
    converged = False
    stop_reason = "max_iters_reached"

    # TODO 11A / TODO 12: QP-IK 外层循环。
    # 每轮都重新 forward、重新构造 task / limit / QP，再积分 q。
    for iteration in range(max_iters):
        data.qpos[:] = q
        mujoco.mj_forward(model, data)

        # TODO 4: FrameTask。
        J_frame, e_frame, position_error_norm, orientation_error_norm = build_frame_task(
            model,
            data,
            site_id,
            target_position,
            target_rotation,
            task_mode=task_mode,
            position_weight=position_weight,
            orientation_weight=orientation_weight,
        )

        # TODO 5: PostureTask。
        J_posture, e_posture = build_posture_task(q, q_ref, posture_weight, nv)

        # TODO 6: 组合任务目标。
        J_task = np.vstack([J_frame, J_posture])
        v_task = gain * np.concatenate([e_frame, e_posture])
        if J_task.shape[0] != v_task.shape[0] or J_task.shape[1] != nv:
            raise ValueError(f"J_task/v_task 维度错误: J_task={J_task.shape}, v_task={v_task.shape}")

        # TODO 7: 构造 QP 目标函数。
        H, c = build_qp_objective(J_task, v_task, regularization)

        # TODO 8 / TODO 9: VelocityLimit + JointPositionLimit。
        velocity_lower, velocity_upper = build_velocity_bounds(velocity_limit, nv)
        position_lower, position_upper = build_position_bounds(model, q, dt, position_margin)
        lower, upper = merge_bounds(velocity_lower, velocity_upper, position_lower, position_upper)

        # TODO 10 / TODO 11: box constraints + scipy.optimize / osqp 求解器。
        dq, solver_status = solve_box_qp(
            H,
            c,
            lower,
            upper,
            solver_name=solver_name,
            max_iter=solver_max_iter,
            eps_abs=eps_abs,
            eps_rel=eps_rel,
        )

        dq_norm = float(np.linalg.norm(dq))
        max_constraint_violation = constraint_violation(dq, lower, upper)
        posture_error_norm = float(np.linalg.norm(e_posture))
        task_error_norm = float(np.linalg.norm(np.concatenate([e_frame, e_posture])))

        error_rows.append(
            {
                "iteration": iteration,
                "position_error_norm": position_error_norm,
                "orientation_error_norm": orientation_error_norm,
                "posture_error_norm": posture_error_norm,
                "task_error_norm": task_error_norm,
                "dq_norm": dq_norm,
                "max_constraint_violation": max_constraint_violation,
                "solver_status": solver_status,
                "task_mode": task_mode,
            }
        )
        constraint_rows.append(
            {
                "iteration": iteration,
                "lower_min": float(np.min(lower)),
                "upper_max": float(np.max(upper)),
                "dq_min": float(np.min(dq)),
                "dq_max": float(np.max(dq)),
                "max_constraint_violation": max_constraint_violation,
                "solver_status": solver_status,
                "task_mode": task_mode,
            }
        )

        if position_error_norm < tolerance:
            converged = True
            stop_reason = "tolerance_reached"
            logging.info("A05 converged at iteration %s, position_error_norm=%s", iteration, position_error_norm)
            break

        q_next = q.copy()
        mujoco.mj_integratePos(model, q_next, dq, dt)
        if q_next.shape != (nq,) or not np.all(np.isfinite(q_next)):
            raise ValueError(f"q_next 非法: shape={q_next.shape}, value={q_next}")
        q = q_next
        q_traj.append(q.copy())

    # 记录积分后的最终误差，便于报告对齐最终 q。
    data.qpos[:] = q
    final_position, final_rotation = site_pose(model, data, site_id)
    final_position_error = float(np.linalg.norm(target_position - final_position))
    if final_position_error < tolerance and not converged:
        converged = True
        stop_reason = "tolerance_reached_after_update"

    q_traj_array = np.asarray(q_traj, dtype=np.float64)

    # TODO 13 / TODO 14 / TODO 15: 输出 trajectory、logs、figure、report。
    trajectory_path.parent.mkdir(parents=True, exist_ok=True)
    np.save(trajectory_path, q_traj_array)
    write_csv_rows(error_log_path, error_rows)
    write_csv_rows(constraint_log_path, constraint_rows)
    write_error_plot(error_rows, figure_path, output_root / "logs" / "A04_dls_ik_error.csv")

    write_report(
        report_path,
        solver_name=solver_name,
        task_mode=task_mode,
        target_site=target_site,
        target_offset=target_offset,
        q_traj=q_traj_array,
        error_rows=error_rows,
        constraint_rows=constraint_rows,
        output_paths={
            "trajectory": trajectory_path,
            "error_log": error_log_path,
            "constraint_log": constraint_log_path,
            "figure": figure_path,
        },
        converged=converged,
        stop_reason=stop_reason,
        final_position_error=final_position_error,
        dt=dt,
        max_iters=max_iters,
        tolerance=tolerance,
        regularization=regularization,
        velocity_limit=velocity_limit,
        position_margin=position_margin,
        posture_weight=posture_weight,
    )

    logging.info("A05 outputs written:")
    logging.info("- trajectory: %s", trajectory_path)
    logging.info("- error log: %s", error_log_path)
    logging.info("- constraint log: %s", constraint_log_path)
    logging.info("- figure: %s", figure_path)
    logging.info("- report: %s", report_path)
    logging.info("A05 final position error: %s", final_position_error)
    logging.info("A05 converged: %s, stop_reason=%s", converged, stop_reason)


if __name__ == "__main__":
    main()
