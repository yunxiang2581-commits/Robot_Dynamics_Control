"""IK interface TODO skeleton.

A04 和 A05 后续都走本模块：
- A04: ``solver_type="dls"``，学习无约束 DLS differential IK。
- A05: ``solver_type="qp_scipy"`` 或 ``"qp_osqp"``，学习带 box limit 的 QP-IK。

本模块当前只定义接口和数学边界，不调用 mink，不启动 viewer，不写 ``data.ctrl``。
"""

from __future__ import annotations

from typing import Any

import numpy as np

from robot_baseline.motion_types import IkRequest, IkResult


def build_frame_task(site_name: str, task_mode: str, weights: dict[str, float]) -> dict[str, Any]:
    """规划 frame task。

    TODO:
    - 要做什么：把 site_name、task_mode、权重组织成 IK task 配置。
    - 为什么：A04/A05 都需要同一个末端任务，而不应在脚本里重复写 target 逻辑。
    - 对标 mink：FrameTask。
    - 输入：site 名称、position/pose task mode、权重。
    - 输出：task dict。
    - 验证：task 中包含 site_name、task_mode、position_weight。
    """
    return {"site_name": site_name, "task_mode": task_mode, "weights": dict(weights)}


def build_posture_task(q_ref: np.ndarray | None, weight: float) -> dict[str, Any]:
    """规划 posture regularization task。

    对标 mink 的 PostureTask。当前只保存配置，不求解。
    """
    return {"q_ref": None if q_ref is None else np.asarray(q_ref, dtype=float), "weight": float(weight)}


def build_dls_step(j_task: np.ndarray, error: np.ndarray, damping: float, gain: float) -> np.ndarray:
    """DLS 单步接口。

    数学目标：
    ``dq = J^T (J J^T + lambda^2 I)^-1 gain e``。

    TODO:
    - 输入：J_task shape=(m,nv)，error shape=(m,)。
    - 输出：dq shape=(nv,)。
    - 验证：J_task @ dq 的方向应减小 task error。
    - 风险：奇异点、damping 太小、gain 太大导致振荡。
    """
    raise NotImplementedError("TODO R2: 在 IK interface 中补 DLS 单步；A04 wrapper 调用这里。")


def build_qp_objective(j_task: np.ndarray, error: np.ndarray, damping: float, gain: float) -> tuple[np.ndarray, np.ndarray]:
    """构造 QP-IK 目标。

    数学形式：
    ``min ||J_task dq - gain error||^2 + damping ||dq||^2``。

    当前只定义接口，不强行接入 scipy/osqp。
    """
    raise NotImplementedError("TODO R2/R3: 构造 H, g，并验证维度与对称性。")


def build_velocity_bounds(nv: int, velocity_limit: float) -> tuple[np.ndarray, np.ndarray]:
    """构造速度 box bounds。

    输出：lower/upper shape=(nv,)，表示 ``-v_max <= dq <= v_max``。
    """
    if nv <= 0:
        raise ValueError("nv must be positive")
    limit = float(velocity_limit)
    if limit <= 0.0:
        raise ValueError("velocity_limit must be positive")
    return -limit * np.ones(nv), limit * np.ones(nv)


def build_position_bounds(q: np.ndarray, q_lower: np.ndarray, q_upper: np.ndarray, dt: float, margin: float) -> tuple[np.ndarray, np.ndarray]:
    """规划 position limit 对 dq 的近似 bounds。

    TODO:
    - 要做什么：由 q_next = q + dq * dt 推出 dq bounds。
    - 为什么：A05 要学习 joint position limit。
    - 对标 mink：ConfigurationLimit。
    - 输入：q、q_lower、q_upper、dt、margin。
    - 输出：lower/upper shape=(nv,)。
    - 验证：积分后的 q_next 不越过 limit。
    """
    raise NotImplementedError("TODO R3: 补固定基 UR5e 的 position bounds；浮动基需单独处理 nq/nv。")


def merge_bounds(*bounds: tuple[np.ndarray, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    """合并多组 box bounds。

    输出：逐元素 max(lower) 和 min(upper)。
    """
    if not bounds:
        raise ValueError("At least one bounds tuple is required")
    lowers = [np.asarray(item[0], dtype=float) for item in bounds]
    uppers = [np.asarray(item[1], dtype=float) for item in bounds]
    return np.maximum.reduce(lowers), np.minimum.reduce(uppers)


def solve_box_qp(objective: tuple[np.ndarray, np.ndarray], bounds: tuple[np.ndarray, np.ndarray], solver_type: str) -> np.ndarray:
    """求解 box-constrained QP。

    TODO:
    - solver_type 支持 ``qp_scipy`` 和 ``qp_osqp``。
    - 当前不实现 solver backend，避免把 A05 完整实现塞进骨架。
    """
    raise NotImplementedError(f"TODO R3: 实现 {solver_type} backend，并记录 solver_status。")


def solve_ik_step(request: IkRequest) -> IkResult:
    """统一 IK 单步入口。

    TODO:
    - ``request.solver_type == 'dls'`` 时调用 build_dls_step。
    - ``request.solver_type in {'qp_scipy', 'qp_osqp'}`` 时构造 QP。
    - 输入：IkRequest，包括 q_init、TargetDefinition、weights、limits。
    - 输出：IkResult。
    - 验证：error norm 下降，shape 与 nq/nv 一致。
    """
    raise NotImplementedError("TODO R2/R3: 实现 IK step dispatch；当前只保留统一入口。")


def solve_ik_trajectory(request: IkRequest) -> IkResult:
    """统一 IK trajectory 入口。

    A04/A05 后续 wrapper 只负责 CLI 和报告路径，实际迭代由这里统一管理。
    """
    raise NotImplementedError("TODO R2/R3: 实现 IK trajectory loop；不进入 viewer / actuator。")
