"""A 项目统一 IK interface。

本模块当前收口到 R3：只实现 IK 纯数学 helper。

已实现：
- DLS 单步；
- QP 目标函数；
- 速度边界；
- 固定基位置边界到速度边界的转换；
- 多组 box bounds 合并；
- box QP 的 SciPy / OSQP 后端分发。

未实现：
- 不读取 MuJoCo model；
- 不计算 site Jacobian；
- 不执行 IK trajectory loop；
- 不启动 viewer；
- 不写 ``data.ctrl``；
- 不调用 mink 替代自己的实现。

学习边界：
- A04 后续使用 ``solver_type="dls"``；
- A05 后续使用 ``solver_type="qp_scipy"`` 或 ``"qp_osqp"``；
- A07 才能进入 actuator tracking 和 ``data.ctrl``。
"""

from __future__ import annotations

from typing import Any

import numpy as np

from robot_baseline.motion_types import IkRequest, IkResult


def _as_1d_float_array(value: Any, name: str) -> np.ndarray:
    """把输入转换成一维浮点数组，并检查数值有限。"""
    array = np.asarray(value, dtype=float).reshape(-1)
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values")
    return array


def _as_2d_float_array(value: Any, name: str) -> np.ndarray:
    """把输入转换成二维浮点数组，并检查数值有限。"""
    array = np.asarray(value, dtype=float)
    if array.ndim != 2:
        raise ValueError(f"{name} must be a 2D matrix")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values")
    return array


def build_frame_task(site_name: str, task_mode: str, weights: dict[str, float]) -> dict[str, Any]:
    """规划 frame task 的轻量配置。

    这是后续 A04/A05 接入 MuJoCo site pose / Jacobian 前的配置占位：
    - 对标 mink 的 ``FrameTask``；
    - 输入是 site 名称、task 模式和权重；
    - 输出是简单 dict，不计算 Jacobian。
    """
    if not isinstance(site_name, str) or not site_name.strip():
        raise ValueError("site_name must be a non-empty string")
    if task_mode not in ("position", "pose_6d"):
        raise ValueError(f"task_mode must be position or pose_6d, got {task_mode!r}")
    return {"site_name": site_name, "task_mode": task_mode, "weights": dict(weights)}


def build_posture_task(q_ref: np.ndarray | None, weight: float) -> dict[str, Any]:
    """规划 posture regularization task 的轻量配置。

    对标 mink 的 ``PostureTask``。R3 只保留配置，不构造完整 task stack。
    """
    if weight < 0.0:
        raise ValueError("posture weight must be non-negative")
    return {"q_ref": None if q_ref is None else _as_1d_float_array(q_ref, "q_ref"), "weight": float(weight)}


def build_dls_step(j_task: np.ndarray, error: np.ndarray, damping: float, gain: float) -> np.ndarray:
    """计算 DLS differential IK 的单步关节速度。

    数学公式：
    ``dq = J.T (J J.T + lambda^2 I)^(-1) gain e``

    输入：
    - ``j_task``: task Jacobian，shape=(m, nv)；
    - ``error``: task error，shape=(m,)；
    - ``damping``: DLS 阻尼 lambda，必须大于 0；
    - ``gain``: task-space error gain，必须大于 0。

    输出：
    - ``dq``: 关节速度，shape=(nv,)。
    """
    j_task = _as_2d_float_array(j_task, "j_task")
    error = _as_1d_float_array(error, "error")

    if error.shape != (j_task.shape[0],):
        raise ValueError(f"error shape must be ({j_task.shape[0]},), got {error.shape}")
    if damping <= 0.0:
        raise ValueError("damping must be positive")
    if gain <= 0.0:
        raise ValueError("gain must be positive")

    task_dim = j_task.shape[0]
    v_task = gain * error
    lhs = j_task @ j_task.T + damping**2 * np.eye(task_dim)
    return j_task.T @ np.linalg.solve(lhs, v_task)


def build_qp_objective(j_task: np.ndarray, error: np.ndarray, damping: float, gain: float) -> tuple[np.ndarray, np.ndarray]:
    """构造 QP-IK 的二次目标。

    数学形式：
    ``min 1/2 ||J_task dq - gain error||^2 + 1/2 damping ||dq||^2``

    标准 QP 形式：
    ``min 1/2 dq.T H dq + c.T dq``

    因此：
    - ``H = J.T J + damping I``；
    - ``c = -J.T gain error``。
    """
    j_task = _as_2d_float_array(j_task, "j_task")
    error = _as_1d_float_array(error, "error")

    if error.shape != (j_task.shape[0],):
        raise ValueError(f"error shape must be ({j_task.shape[0]},), got {error.shape}")
    if damping < 0.0:
        raise ValueError("damping must be non-negative")
    if gain <= 0.0:
        raise ValueError("gain must be positive")

    nv = j_task.shape[1]
    v_task = gain * error
    hessian = j_task.T @ j_task + damping * np.eye(nv)
    gradient = -j_task.T @ v_task
    hessian = 0.5 * (hessian + hessian.T)
    return hessian, gradient


def build_velocity_bounds(nv: int, velocity_limit: float | np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """构造 velocity limit 对 ``dq`` 的 box bounds。

    支持：
    - 标量速度限制：所有关节共用同一个 limit；
    - shape=(nv,) 的每关节速度限制。
    """
    if nv <= 0:
        raise ValueError("nv must be positive")

    limit = np.asarray(velocity_limit, dtype=float)
    if limit.ndim == 0:
        limit = np.full(nv, float(limit))
    else:
        limit = limit.reshape(-1)
        if limit.shape != (nv,):
            raise ValueError(f"velocity_limit shape must be scalar or ({nv},), got {limit.shape}")

    if not np.all(np.isfinite(limit)):
        raise ValueError("velocity_limit must contain only finite values")
    if np.any(limit <= 0.0):
        raise ValueError("all velocity limits must be positive")

    return -limit, limit


def build_position_bounds(
    q: np.ndarray,
    q_lower: np.ndarray,
    q_upper: np.ndarray,
    dt: float,
    margin: float,
) -> tuple[np.ndarray, np.ndarray]:
    """把固定基位置限制转换成 ``dq`` bounds。

    数学推导：
    ``q_next = q + dq dt``
    ``q_lower + margin <= q_next <= q_upper - margin``

    因此：
    ``(q_lower + margin - q) / dt <= dq <= (q_upper - margin - q) / dt``

    注意：
    - 这是固定基 UR5e 的第一版近似；
    - 浮动基模型中 ``nq`` 和 ``nv`` 可能不同，后续需要单独处理。
    """
    q = _as_1d_float_array(q, "q")
    q_lower = _as_1d_float_array(q_lower, "q_lower")
    q_upper = _as_1d_float_array(q_upper, "q_upper")

    if q_lower.shape != q.shape or q_upper.shape != q.shape:
        raise ValueError("q, q_lower, and q_upper must have the same shape")
    if dt <= 0.0:
        raise ValueError("dt must be positive")
    if margin < 0.0:
        raise ValueError("margin must be non-negative")
    if np.any(q_lower > q_upper):
        bad = np.where(q_lower > q_upper)[0].tolist()
        raise ValueError(f"q_lower must be <= q_upper at indices: {bad}")

    safe_lower = q_lower + margin
    safe_upper = q_upper - margin
    if np.any(safe_lower > safe_upper):
        bad = np.where(safe_lower > safe_upper)[0].tolist()
        raise ValueError(f"position margin is too large at indices: {bad}")

    lower = (safe_lower - q) / dt
    upper = (safe_upper - q) / dt
    return lower, upper


def merge_bounds(*bounds: tuple[np.ndarray, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    """合并多组 box bounds。

    数学逻辑：
    - lower 取逐元素最大值；
    - upper 取逐元素最小值。
    """
    if not bounds:
        raise ValueError("At least one bounds tuple is required")

    lowers: list[np.ndarray] = []
    uppers: list[np.ndarray] = []
    expected_shape: tuple[int, ...] | None = None

    for lower, upper in bounds:
        lower = _as_1d_float_array(lower, "lower")
        upper = _as_1d_float_array(upper, "upper")

        if lower.shape != upper.shape:
            raise ValueError(f"lower and upper must have same shape, got {lower.shape} and {upper.shape}")
        if expected_shape is None:
            expected_shape = lower.shape
        elif lower.shape != expected_shape:
            raise ValueError(f"all bounds must have shape {expected_shape}, got {lower.shape}")

        lowers.append(lower)
        uppers.append(upper)

    merged_lower = np.maximum.reduce(lowers)
    merged_upper = np.minimum.reduce(uppers)

    if np.any(merged_lower > merged_upper):
        bad = np.where(merged_lower > merged_upper)[0].tolist()
        raise ValueError(f"infeasible bounds at indices: {bad}")

    return merged_lower, merged_upper


def _prepare_box_qp_inputs(
    objective: tuple[np.ndarray, np.ndarray],
    bounds: tuple[np.ndarray, np.ndarray],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """统一检查 box QP 输入。"""
    hessian, gradient = objective
    lower, upper = bounds

    hessian = _as_2d_float_array(hessian, "hessian")
    gradient = _as_1d_float_array(gradient, "gradient")
    lower = _as_1d_float_array(lower, "lower")
    upper = _as_1d_float_array(upper, "upper")

    if hessian.shape[0] != hessian.shape[1]:
        raise ValueError("hessian must be a square matrix")

    nv = hessian.shape[0]
    if gradient.shape != (nv,):
        raise ValueError(f"gradient shape must be ({nv},), got {gradient.shape}")
    if lower.shape != (nv,) or upper.shape != (nv,):
        raise ValueError(f"bounds shape must be ({nv},), got {lower.shape} and {upper.shape}")
    if np.any(lower > upper):
        bad = np.where(lower > upper)[0].tolist()
        raise ValueError(f"infeasible bounds at indices: {bad}")

    hessian = 0.5 * (hessian + hessian.T)
    return hessian, gradient, lower, upper


def _check_box_qp_solution(
    dq: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    tolerance: float,
    solver_name: str,
) -> np.ndarray:
    """检查 QP 解是否满足 box bounds。"""
    dq = _as_1d_float_array(dq, "dq")
    if dq.shape != lower.shape:
        raise RuntimeError(f"{solver_name} returned dq shape {dq.shape}, expected {lower.shape}")

    violation = max(
        float(np.max(lower - dq)),
        float(np.max(dq - upper)),
        0.0,
    )
    if violation > tolerance:
        raise RuntimeError(f"{solver_name} returned infeasible dq, violation={violation}")
    return dq


def _solve_box_qp_scipy(hessian: np.ndarray, gradient: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    """使用 SciPy SLSQP 求解 box-constrained QP。"""
    from scipy.optimize import Bounds, minimize

    nv = hessian.shape[0]

    def fun(dq: np.ndarray) -> float:
        return 0.5 * float(dq @ hessian @ dq) + float(gradient @ dq)

    def jac(dq: np.ndarray) -> np.ndarray:
        return hessian @ dq + gradient

    x0 = np.clip(np.zeros(nv), lower, upper)
    result = minimize(
        fun,
        x0,
        jac=jac,
        bounds=Bounds(lower, upper),
        method="SLSQP",
        options={"ftol": 1.0e-9, "maxiter": 100},
    )
    if not result.success:
        raise RuntimeError(f"scipy box QP failed: {result.message}")

    return _check_box_qp_solution(result.x, lower, upper, tolerance=1.0e-8, solver_name="scipy box QP")


def _solve_box_qp_osqp(hessian: np.ndarray, gradient: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    """使用 OSQP 求解 box-constrained QP。

    OSQP 标准形式：
    ``min 1/2 x.T P x + q.T x``
    ``s.t. l <= A x <= u``

    本项目的映射：
    - x = dq；
    - P = hessian；
    - q_osqp = gradient；
    - A = I；
    - l = lower；
    - u = upper。
    """
    try:
        import osqp
        from scipy import sparse
    except ImportError as exc:
        raise ImportError("qp_osqp requires osqp and scipy to be installed") from exc

    nv = hessian.shape[0]
    p_osqp = sparse.csc_matrix(hessian)
    q_osqp = gradient
    a_osqp = sparse.eye(nv, format="csc")

    solver = osqp.OSQP()
    solver.setup(
        P=p_osqp,
        q=q_osqp,
        A=a_osqp,
        l=lower,
        u=upper,
        verbose=False,
        eps_abs=1.0e-8,
        eps_rel=1.0e-8,
        max_iter=10000,
        polish=True,
    )
    result = solver.solve()

    # OSQP status_val: 1=solved, 2=solved inaccurate。第一版接受二者，但仍做 bounds 检查。
    if result.info.status_val not in (1, 2):
        raise RuntimeError(f"OSQP failed: status={result.info.status}")

    return _check_box_qp_solution(result.x, lower, upper, tolerance=1.0e-7, solver_name="OSQP")


def solve_box_qp(objective: tuple[np.ndarray, np.ndarray], bounds: tuple[np.ndarray, np.ndarray], solver_type: str) -> np.ndarray:
    """求解 box-constrained QP。

    支持：
    - ``qp_scipy``；
    - ``qp_osqp``。

    本函数只处理纯数学 QP：
    ``min 1/2 dq.T H dq + c.T dq``
    ``s.t. lower <= dq <= upper``

    不读取 target，不计算 Jacobian，不积分 q，不写 ``data.ctrl``。
    """
    hessian, gradient, lower, upper = _prepare_box_qp_inputs(objective, bounds)

    if solver_type == "qp_scipy":
        return _solve_box_qp_scipy(hessian, gradient, lower, upper)
    if solver_type == "qp_osqp":
        return _solve_box_qp_osqp(hessian, gradient, lower, upper)

    raise ValueError(f"unsupported solver_type: {solver_type}")


def solve_ik_step(request: IkRequest) -> IkResult:
    """统一 IK 单步入口。

    TODO R4/R5：
    - ``request.solver_type == "dls"`` 时调用 ``build_dls_step``；
    - ``request.solver_type in {"qp_scipy", "qp_osqp"}`` 时构造 QP；
    - 输入是完整 ``IkRequest``；
    - 输出是 ``IkResult``。

    当前 R3 不接 MuJoCo pose / Jacobian，因此保留 TODO。
    """
    raise NotImplementedError("TODO R4/R5: 实现 IK step dispatch；R3 只收口纯数学 helper。")


def solve_ik_trajectory(request: IkRequest) -> IkResult:
    """统一 IK trajectory 入口。

    TODO R4/R5：
    - A04/A05 wrapper 只负责 CLI 和报告路径；
    - 实际迭代由这里统一管理；
    - 不进入 viewer / actuator。
    """
    raise NotImplementedError("TODO R4/R5: 实现 IK trajectory loop；不进入 viewer / actuator。")
