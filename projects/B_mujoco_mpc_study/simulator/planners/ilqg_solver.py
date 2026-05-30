"""B03 mini iLQR / iLQG-lite core loop.

本文件实现的是教学用 deterministic iLQR-lite，不是工业级 iLQG：
- 不加入随机噪声传播；
- 不加入 full DDP 的二阶动力学项；
- 不接 OSQP / IPOPT / acados；
- 不写入 MuJoCo 或 B02 controller 细节。

当前学习目标是把一条 nominal trajectory 周围的局部 LQ 子问题拆清楚：
rollout -> dynamics linearization -> cost quadratization -> backward pass -> forward pass。
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Callable, Sequence

import numpy as np

from planners.mpc_solver_interface import BaseMPCSolver, MPCProblem, MPCSolution, SolverStats
from planners.sampling_mpc_solvers import shift_control_sequence


ArrayFn = Callable[[np.ndarray, np.ndarray], np.ndarray]
StageCostFn = Callable[[np.ndarray, np.ndarray, int], float]
TerminalCostFn = Callable[[np.ndarray], float]


@dataclass(frozen=True)
class ILQGConfig:
    """iLQR-lite 配置。

    horizon 是控制序列长度；其余参数只覆盖本步最小核心闭环需要的数值设置。
    """

    horizon: int
    max_iterations: int = 10
    line_search: bool = True
    regularization: float = 1e-6
    finite_difference_eps: float = 1e-5
    line_search_alphas: tuple[float, ...] = (1.0, 0.5, 0.25, 0.1, 0.05)
    tolerance: float = 1e-6
    control_limit: float | None = None


@dataclass(frozen=True)
class NominalTrajectory:
    """给定控制序列 rollout 得到的 nominal trajectory。"""

    states: np.ndarray
    controls: np.ndarray
    costs: np.ndarray
    total_cost: float


@dataclass(frozen=True)
class LinearizedDynamics:
    """沿 nominal trajectory 的时变线性化动力学。

    A[t] = d f / d x, B[t] = d f / d u。
    """

    A: np.ndarray
    B: np.ndarray


@dataclass(frozen=True)
class QuadraticCostApproximation:
    """沿 nominal trajectory 的二次 tracking cost 近似。"""

    l_x: np.ndarray
    l_u: np.ndarray
    l_xx: np.ndarray
    l_uu: np.ndarray
    l_ux: np.ndarray
    terminal_x: np.ndarray
    terminal_xx: np.ndarray


@dataclass(frozen=True)
class ILQGBackwardPassResult:
    """backward pass 给出的前馈增量 k 和反馈增益 K。"""

    feedforward_gains: np.ndarray
    feedback_gains: np.ndarray
    expected_cost_reduction: float
    success: bool
    message: str

    @property
    def k_feedforward(self) -> np.ndarray:
        """兼容旧 skeleton 名称。"""
        # 旧版 skeleton 用 k_feedforward 命名，这里只做别名，不复制数组。
        return self.feedforward_gains

    @property
    def K_feedback(self) -> np.ndarray:
        """兼容旧 skeleton 名称。"""
        # 旧版 skeleton 用 K_feedback 命名，这里只做别名，不改变数据。
        return self.feedback_gains


@dataclass(frozen=True)
class ILQGForwardPassResult:
    """forward pass 在真实非线性动力学上验证一组更新后的控制序列。"""

    states: np.ndarray
    controls: np.ndarray
    costs: np.ndarray
    total_cost: float
    alpha: float
    accepted: bool
    message: str


def _as_float_array(value: Any, name: str) -> np.ndarray:
    # 把输入统一转成 float 数组，后续矩阵运算都依赖浮点数。
    array = np.asarray(value, dtype=float)
    # iLQR 的线性化和 backward pass 不能接受 NaN / inf。
    if not np.isfinite(array).all():
        # 报错时带上变量名和 shape，方便定位是哪一个输入坏了。
        raise ValueError(f"{name} must contain only finite values, got shape={array.shape}")
    # 返回已经通过检查的 float 数组。
    return array


def _require_shape(array: np.ndarray, expected: tuple[int, ...], name: str) -> None:
    # 检查数组 shape 是否等于调用者期望的 shape。
    if array.shape != expected:
        # shape 不匹配时立即报错，避免后面的矩阵乘法报出更难懂的错误。
        raise ValueError(f"{name} shape must be {expected}, got {array.shape}")


def _require_square_matrix(matrix: np.ndarray, name: str) -> None:
    # 二次 cost 矩阵 Q/R/Q_terminal 都必须是二维方阵。
    if matrix.ndim != 2 or matrix.shape[0] != matrix.shape[1]:
        # 不是方阵时说明 cost 权重维度和状态/控制维度无法匹配。
        raise ValueError(f"{name} must be a square matrix, got shape={matrix.shape}")


def _symmetrize(matrix: np.ndarray) -> np.ndarray:
    # 数值计算中矩阵可能有极小非对称误差，这里取 (M + M.T) / 2。
    return 0.5 * (matrix + matrix.T)


def finite_difference_jacobian(
    fn: Callable[[np.ndarray], np.ndarray],
    x: np.ndarray,
    eps: float = 1e-5,
) -> np.ndarray:
    """用中心差分计算向量函数的 Jacobian。

    输入是某个局部变量 x；输出矩阵形状为 `(output_dim, input_dim)`。
    """
    # 先把输入变量统一成 float 向量，并拒绝 NaN / inf。
    # iLQR 后面的矩阵递推依赖数值稳定性，所以这里尽早做检查。
    x0 = _as_float_array(x, "x")
    if x0.ndim != 1:
        raise ValueError(f"x must be 1D, got shape={x0.shape}")
    if eps <= 0.0:
        raise ValueError(f"eps must be positive, got {eps}")

    # 先计算一次原始输出，用它确定输出维度 output_dim。
    # 本函数只处理向量到向量的函数：fn(x).shape == (output_dim,)。
    y0 = _as_float_array(fn(x0), "fn(x)")
    if y0.ndim != 1:
        raise ValueError(f"fn(x) must be 1D, got shape={y0.shape}")

    # Jacobian 的行对应输出维度，列对应输入维度：
    # jacobian[row, col] = d fn[row] / d x[col]。
    jacobian = np.zeros((y0.size, x0.size), dtype=float)

    # 一次只扰动 x 的一个分量，因此循环每次填 Jacobian 的一列。
    for index in range(x0.size):
        delta = np.zeros_like(x0)
        delta[index] = eps

        # 中心差分：同时看 x[index] 左右两侧的函数值变化。
        # 公式：df/dx_i ~= (f(x + eps e_i) - f(x - eps e_i)) / (2 eps)。
        y_plus = _as_float_array(fn(x0 + delta), "fn(x + delta)")
        y_minus = _as_float_array(fn(x0 - delta), "fn(x - delta)")

        # 扰动前后输出 shape 必须不变，否则无法组成同一个 Jacobian 矩阵。
        _require_shape(y_plus, y0.shape, "fn(x + delta)")
        _require_shape(y_minus, y0.shape, "fn(x - delta)")
        jacobian[:, index] = (y_plus - y_minus) / (2.0 * eps)

    # 最后再检查一次整体结果，避免把 NaN / inf 传给 backward pass。
    if not np.isfinite(jacobian).all():
        raise ValueError("finite_difference_jacobian produced non-finite values")
    return jacobian


def linearize_discrete_dynamics(
    dynamics_fn: ArrayFn,
    x: np.ndarray,
    u: np.ndarray,
    eps: float = 1e-5,
) -> tuple[np.ndarray, np.ndarray]:
    """线性化离散动力学 `x_next = f(x, u)`。

    输出：
    - A = d f / d x，形状 `(state_dim, state_dim)`
    - B = d f / d u，形状 `(state_dim, control_dim)`
    """
    # 把当前线性化点的状态转成 float 向量。
    x0 = _as_float_array(x, "x")
    # 把当前线性化点的控制转成 float 向量。
    u0 = _as_float_array(u, "u")
    # 离散动力学这里约定状态是 1D 向量。
    if x0.ndim != 1:
        raise ValueError(f"x must be 1D, got shape={x0.shape}")
    # 控制也约定是 1D 向量。
    if u0.ndim != 1:
        raise ValueError(f"u must be 1D, got shape={u0.shape}")

    # 先调用一次 dynamics，确认输出状态维度是否和输入状态一致。
    next_state = _as_float_array(dynamics_fn(x0, u0), "dynamics_fn(x, u)")
    # 对离散系统 x_next = f(x, u)，x_next 应该仍是 state_dim 维。
    _require_shape(next_state, x0.shape, "dynamics_fn(x, u)")

    # 固定 u，只扰动 x，得到 A = df/dx。
    A = finite_difference_jacobian(lambda x_var: dynamics_fn(x_var, u0), x0, eps=eps)
    # 固定 x，只扰动 u，得到 B = df/du。
    B = finite_difference_jacobian(lambda u_var: dynamics_fn(x0, u_var), u0, eps=eps)
    # A 的 shape 应为 (state_dim, state_dim)。
    _require_shape(A, (x0.size, x0.size), "A")
    # B 的 shape 应为 (state_dim, control_dim)。
    _require_shape(B, (x0.size, u0.size), "B")
    # 返回当前时刻的局部线性动力学矩阵。
    return A, B


def rollout_nominal_trajectory(
    dynamics_fn: ArrayFn,
    initial_state: np.ndarray,
    controls: np.ndarray,
    cost_fn: StageCostFn,
    terminal_cost_fn: TerminalCostFn | None = None,
) -> NominalTrajectory:
    """从初始状态和控制序列生成 nominal trajectory。

    这一步是 iLQR 的基准轨迹：后续线性化和二次近似都围绕它展开。
    """
    # 初始状态 x0 必须是有限的 float 向量。
    x0 = _as_float_array(initial_state, "initial_state")
    # 控制序列 U 的 shape 约定为 (H, control_dim)。
    U = _as_float_array(controls, "controls")
    # 状态必须是一维向量，例如 toy demo 中是 [position, velocity]。
    if x0.ndim != 1:
        raise ValueError(f"initial_state must be 1D, got shape={x0.shape}")
    # 控制序列必须是二维数组，每一行对应一个时间步。
    if U.ndim != 2:
        raise ValueError(f"controls must be 2D, got shape={U.shape}")

    # horizon 是控制步数 H；_control_dim 当前只由 U.shape 推断，不在本函数直接使用。
    horizon, _control_dim = U.shape
    # 状态维度由初始状态长度决定。
    state_dim = x0.size
    # states 比 controls 多一行：x_0, x_1, ..., x_H。
    states = np.zeros((horizon + 1, state_dim), dtype=float)
    # costs 保存 H 个 stage cost，不含 terminal cost。
    costs = np.zeros(horizon, dtype=float)
    # rollout 的第一项必须等于当前初始状态。
    states[0] = x0

    # 按时间从 0 到 H-1 前向模拟。
    for t in range(horizon):
        # 计算当前状态和当前控制对应的 stage cost。
        stage_cost = float(cost_fn(states[t], U[t], t))
        # cost 必须是有限标量，否则后续 line search 无法比较。
        if not np.isfinite(stage_cost):
            raise ValueError(f"cost_fn returned non-finite value at t={t}: {stage_cost}")
        # 保存第 t 步 stage cost。
        costs[t] = stage_cost

        # 用真实离散动力学计算下一步状态。
        next_state = _as_float_array(dynamics_fn(states[t], U[t]), f"dynamics_fn at t={t}")
        # 下一步状态 shape 必须保持为 (state_dim,)。
        _require_shape(next_state, (state_dim,), f"dynamics_fn output at t={t}")
        # 写入 x_{t+1}。
        states[t + 1] = next_state

    # terminal cost 是可选项；没有给时默认为 0。
    terminal_cost = 0.0
    # 如果调用者提供 terminal_cost_fn，就在最后一个状态上计算终端代价。
    if terminal_cost_fn is not None:
        terminal_cost = float(terminal_cost_fn(states[-1]))
        # terminal cost 同样必须是有限标量。
        if not np.isfinite(terminal_cost):
            raise ValueError(f"terminal_cost_fn returned non-finite value: {terminal_cost}")

    # 总 cost = 所有 stage cost 之和 + terminal cost。
    total_cost = float(np.sum(costs) + terminal_cost)
    # 总 cost 也要检查，防止数值问题进入 solver 主循环。
    if not np.isfinite(total_cost):
        raise ValueError("nominal trajectory total_cost is non-finite")

    # 打包成 NominalTrajectory，供后续 linearization / quadratization 使用。
    return NominalTrajectory(
        states=states,
        controls=U.copy(),
        costs=costs,
        total_cost=total_cost,
    )


def linearize_trajectory_dynamics(
    dynamics_fn: ArrayFn,
    nominal_trajectory: NominalTrajectory,
    eps: float = 1e-5,
) -> LinearizedDynamics:
    """沿 nominal trajectory 逐步线性化离散动力学。"""
    # 取出 nominal states，并检查其中没有 NaN / inf。
    states = _as_float_array(nominal_trajectory.states, "nominal_trajectory.states")
    # 取出 nominal controls，并检查其中没有 NaN / inf。
    controls = _as_float_array(nominal_trajectory.controls, "nominal_trajectory.controls")
    # states 和 controls 都必须是按时间排列的二维数组。
    if states.ndim != 2 or controls.ndim != 2:
        raise ValueError("nominal states and controls must both be 2D arrays")
    # controls.shape = (H, control_dim)，H 是 horizon。
    horizon, control_dim = controls.shape
    # states.shape = (H + 1, state_dim)，第二维给出状态维度。
    state_dim = states.shape[1]
    # 检查 states 的时间长度确实比 controls 多 1。
    _require_shape(states, (horizon + 1, state_dim), "nominal_trajectory.states")

    # 预分配所有时间步的 A 矩阵。
    A = np.zeros((horizon, state_dim, state_dim), dtype=float)
    # 预分配所有时间步的 B 矩阵。
    B = np.zeros((horizon, state_dim, control_dim), dtype=float)
    # 沿 nominal trajectory 的每个时间步单独线性化。
    for t in range(horizon):
        # 在 (states[t], controls[t]) 这一点计算 A_t 和 B_t。
        A_t, B_t = linearize_discrete_dynamics(dynamics_fn, x=states[t], u=controls[t], eps=eps)
        # 保存第 t 步的 A_t。
        A[t] = A_t
        # 保存第 t 步的 B_t。
        B[t] = B_t

    # 线性化结果必须全是有限数，才能进入 backward pass。
    if not np.isfinite(A).all() or not np.isfinite(B).all():
        raise ValueError("linearized dynamics contain non-finite values")
    # 打包返回时变线性化动力学。
    return LinearizedDynamics(A=A, B=B)


def quadratize_trajectory_cost(
    states: np.ndarray,
    controls: np.ndarray,
    x_refs: np.ndarray,
    Q: np.ndarray,
    R: np.ndarray,
    Q_terminal: np.ndarray,
) -> QuadraticCostApproximation:
    """对 tracking cost 写出解析一阶/二阶项。

    stage cost:
        1/2 (x - x_ref)^T Q (x - x_ref) + 1/2 u^T R u
    terminal cost:
        1/2 (x_H - x_ref_H)^T Q_terminal (x_H - x_ref_H)
    """
    # X 是 nominal states，shape 应为 (H + 1, state_dim)。
    X = _as_float_array(states, "states")
    # U 是 nominal controls，shape 应为 (H, control_dim)。
    U = _as_float_array(controls, "controls")
    # X_ref 是参考状态轨迹，shape 应与 X 一致。
    X_ref = _as_float_array(x_refs, "x_refs")
    # Q 是 stage state tracking 权重矩阵。
    Q_mat = _as_float_array(Q, "Q")
    # R 是 stage control effort 权重矩阵。
    R_mat = _as_float_array(R, "R")
    # Q_terminal 是终端状态 tracking 权重矩阵。
    Q_terminal_mat = _as_float_array(Q_terminal, "Q_terminal")

    # states、controls、x_refs 都必须按时间展开成二维数组。
    if X.ndim != 2 or U.ndim != 2 or X_ref.ndim != 2:
        raise ValueError("states, controls and x_refs must be 2D arrays")
    # 从控制序列中读出 horizon 和控制维度。
    horizon, control_dim = U.shape
    # 从状态轨迹中读出状态维度。
    state_dim = X.shape[1]
    # 检查状态轨迹长度是否为 H + 1。
    _require_shape(X, (horizon + 1, state_dim), "states")
    # 检查参考轨迹是否和状态轨迹完全对齐。
    _require_shape(X_ref, (horizon + 1, state_dim), "x_refs")
    # Q 必须是方阵。
    _require_square_matrix(Q_mat, "Q")
    # R 必须是方阵。
    _require_square_matrix(R_mat, "R")
    # Q_terminal 必须是方阵。
    _require_square_matrix(Q_terminal_mat, "Q_terminal")
    # Q 的维度必须匹配 state_dim。
    _require_shape(Q_mat, (state_dim, state_dim), "Q")
    # R 的维度必须匹配 control_dim。
    _require_shape(R_mat, (control_dim, control_dim), "R")
    # Q_terminal 的维度必须匹配 state_dim。
    _require_shape(Q_terminal_mat, (state_dim, state_dim), "Q_terminal")

    # 对称化 Q，避免输入矩阵存在微小非对称数值误差。
    Q_sym = _symmetrize(Q_mat)
    # 对称化 R。
    R_sym = _symmetrize(R_mat)
    # 对称化终端权重矩阵。
    Q_terminal_sym = _symmetrize(Q_terminal_mat)

    # l_x 保存每一步 stage cost 对状态的一阶导。
    l_x = np.zeros((horizon, state_dim), dtype=float)
    # l_u 保存每一步 stage cost 对控制的一阶导。
    l_u = np.zeros((horizon, control_dim), dtype=float)
    # l_xx 保存每一步 stage cost 对状态的二阶导。
    l_xx = np.zeros((horizon, state_dim, state_dim), dtype=float)
    # l_uu 保存每一步 stage cost 对控制的二阶导。
    l_uu = np.zeros((horizon, control_dim, control_dim), dtype=float)
    # 当前 tracking cost 没有 x-u 交叉项，因此 l_ux 全为 0。
    l_ux = np.zeros((horizon, control_dim, state_dim), dtype=float)

    # 对每一个 stage 写出解析导数。
    for t in range(horizon):
        # l_x = Q (x_t - x_ref_t)。
        l_x[t] = Q_sym @ (X[t] - X_ref[t])
        # l_u = R u_t。
        l_u[t] = R_sym @ U[t]
        # l_xx = Q。
        l_xx[t] = Q_sym
        # l_uu = R。
        l_uu[t] = R_sym

    # terminal_x = Q_terminal (x_H - x_ref_H)。
    terminal_x = Q_terminal_sym @ (X[-1] - X_ref[-1])

    # 把所有导数项放到一起做 finite 检查。
    arrays = (l_x, l_u, l_xx, l_uu, l_ux, terminal_x, Q_terminal_sym)
    # 任何 NaN / inf 都会让 backward pass 不可信。
    if not all(np.isfinite(array).all() for array in arrays):
        raise ValueError("quadratic cost approximation contains non-finite values")

    # 打包返回 cost 的一阶和二阶近似。
    return QuadraticCostApproximation(
        l_x=l_x,
        l_u=l_u,
        l_xx=l_xx,
        l_uu=l_uu,
        l_ux=l_ux,
        terminal_x=terminal_x,
        terminal_xx=Q_terminal_sym,
    )


def _finite_difference_scalar_gradient(
    fn: Callable[[np.ndarray], float],
    z: np.ndarray,
    eps: float,
    name: str,
) -> np.ndarray:
    """用中心差分计算标量函数的一阶导。"""
    # 通用 task-space cost 不是简单的 x_ref/Q/R，所以这里用数值微分。
    z0 = _as_float_array(z, name)
    if z0.ndim != 1:
        raise ValueError(f"{name} must be 1D, got shape={z0.shape}")
    if eps <= 0.0:
        raise ValueError(f"eps must be positive, got {eps}")

    gradient = np.zeros_like(z0, dtype=float)
    for index in range(z0.size):
        delta = np.zeros_like(z0)
        delta[index] = eps
        cost_plus = float(fn(z0 + delta))
        cost_minus = float(fn(z0 - delta))
        if not np.isfinite(cost_plus) or not np.isfinite(cost_minus):
            raise ValueError(f"{name} finite-difference cost is non-finite at index={index}")
        gradient[index] = (cost_plus - cost_minus) / (2.0 * eps)

    if not np.isfinite(gradient).all():
        raise ValueError(f"{name} gradient contains non-finite values")
    return gradient


def quadratize_custom_trajectory_cost(
    states: np.ndarray,
    controls: np.ndarray,
    stage_cost_fn: StageCostFn,
    terminal_cost_fn: TerminalCostFn | None,
    eps: float = 1e-5,
) -> QuadraticCostApproximation:
    """用有限差分对通用 stage/terminal cost 做二次近似。

    这个函数服务于 task-space tracking：代价可以写成
    `||p_ee(q)-p_ref||`，不再要求一定是 `||x-x_ref||_Q`。
    """
    X = _as_float_array(states, "states")
    U = _as_float_array(controls, "controls")
    if X.ndim != 2 or U.ndim != 2:
        raise ValueError("states and controls must both be 2D arrays")
    horizon, control_dim = U.shape
    state_dim = X.shape[1]
    _require_shape(X, (horizon + 1, state_dim), "states")
    if not callable(stage_cost_fn):
        raise ValueError("stage_cost_fn must be callable")
    if terminal_cost_fn is not None and not callable(terminal_cost_fn):
        raise ValueError("terminal_cost_fn must be callable when provided")

    l_x = np.zeros((horizon, state_dim), dtype=float)
    l_u = np.zeros((horizon, control_dim), dtype=float)
    l_xx = np.zeros((horizon, state_dim, state_dim), dtype=float)
    l_uu = np.zeros((horizon, control_dim, control_dim), dtype=float)
    l_ux = np.zeros((horizon, control_dim, state_dim), dtype=float)

    for t in range(horizon):
        z0 = np.concatenate([X[t], U[t]]).astype(float, copy=True)

        def stage_on_z(z: np.ndarray, step_index: int = t) -> float:
            z_arr = np.asarray(z, dtype=float)
            x_part = z_arr[:state_dim]
            u_part = z_arr[state_dim:]
            return float(stage_cost_fn(x_part, u_part, step_index))

        grad = _finite_difference_scalar_gradient(stage_on_z, z0, eps, f"stage_cost[{t}]")
        hessian = finite_difference_jacobian(
            lambda z_var, step_index=t: _finite_difference_scalar_gradient(
                lambda z_inner: stage_on_z(z_inner, step_index),
                z_var,
                eps,
                f"stage_cost[{step_index}]",
            ),
            z0,
            eps=eps,
        )
        hessian = _symmetrize(hessian)

        l_x[t] = grad[:state_dim]
        l_u[t] = grad[state_dim:]
        l_xx[t] = hessian[:state_dim, :state_dim]
        l_uu[t] = hessian[state_dim:, state_dim:]
        l_ux[t] = hessian[state_dim:, :state_dim]

    if terminal_cost_fn is None:
        terminal_x = np.zeros(state_dim, dtype=float)
        terminal_xx = np.zeros((state_dim, state_dim), dtype=float)
    else:
        terminal_x = _finite_difference_scalar_gradient(
            lambda x_var: float(terminal_cost_fn(x_var)),
            X[-1],
            eps,
            "terminal_cost",
        )
        terminal_xx = finite_difference_jacobian(
            lambda x_var: _finite_difference_scalar_gradient(
                lambda x_inner: float(terminal_cost_fn(x_inner)),
                x_var,
                eps,
                "terminal_cost",
            ),
            X[-1],
            eps=eps,
        )
        terminal_xx = _symmetrize(terminal_xx)

    arrays = (l_x, l_u, l_xx, l_uu, l_ux, terminal_x, terminal_xx)
    if not all(np.isfinite(array).all() for array in arrays):
        raise ValueError("custom quadratic cost approximation contains non-finite values")

    return QuadraticCostApproximation(
        l_x=l_x,
        l_u=l_u,
        l_xx=l_xx,
        l_uu=l_uu,
        l_ux=l_ux,
        terminal_x=terminal_x,
        terminal_xx=terminal_xx,
    )


def backward_pass(
    linearized_dynamics: LinearizedDynamics,
    quadratic_cost: QuadraticCostApproximation,
    config: ILQGConfig | None = None,
) -> ILQGBackwardPassResult:
    """反向求解局部 LQ 子问题，得到 feedforward k 和 feedback K。"""
    # 取出整条轨迹上的 A 矩阵序列。
    A_all = _as_float_array(linearized_dynamics.A, "linearized_dynamics.A")
    # 取出整条轨迹上的 B 矩阵序列。
    B_all = _as_float_array(linearized_dynamics.B, "linearized_dynamics.B")
    # A/B 都应该是三维数组：时间维 + 矩阵二维。
    if A_all.ndim != 3 or B_all.ndim != 3:
        raise ValueError("A and B must be 3D arrays")

    # A_all.shape = (H, state_dim, state_dim)。
    horizon, state_dim, state_dim_2 = A_all.shape
    # 每一步的 A 必须是方阵。
    if state_dim != state_dim_2:
        raise ValueError(f"A must be square per step, got shape={A_all.shape}")
    # B_all.shape = (H, state_dim, control_dim)，这里确认前两维与 A 对齐。
    _require_shape(B_all, (horizon, state_dim, B_all.shape[2]), "linearized_dynamics.B")
    # 从 B 的最后一维读出 control_dim。
    control_dim = B_all.shape[2]

    # 如果调用者没传 config，就用当前 horizon 构造默认配置。
    cfg = config or ILQGConfig(horizon=horizon)
    # regularization 用于稳定 Q_uu 求解，不能是负数。
    if cfg.regularization < 0.0:
        raise ValueError(f"regularization must be non-negative, got {cfg.regularization}")

    # 读取 stage cost 对状态的一阶导。
    l_x = _as_float_array(quadratic_cost.l_x, "quadratic_cost.l_x")
    # 读取 stage cost 对控制的一阶导。
    l_u = _as_float_array(quadratic_cost.l_u, "quadratic_cost.l_u")
    # 读取 stage cost 对状态的二阶导。
    l_xx = _as_float_array(quadratic_cost.l_xx, "quadratic_cost.l_xx")
    # 读取 stage cost 对控制的二阶导。
    l_uu = _as_float_array(quadratic_cost.l_uu, "quadratic_cost.l_uu")
    # 读取 stage cost 对控制-状态的交叉二阶导。
    l_ux = _as_float_array(quadratic_cost.l_ux, "quadratic_cost.l_ux")
    # 读取 terminal cost 对最终状态的一阶导。
    terminal_x = _as_float_array(quadratic_cost.terminal_x, "quadratic_cost.terminal_x")
    # 读取 terminal cost 对最终状态的二阶导。
    terminal_xx = _as_float_array(quadratic_cost.terminal_xx, "quadratic_cost.terminal_xx")

    # l_x 每步一个 state_dim 向量。
    _require_shape(l_x, (horizon, state_dim), "l_x")
    # l_u 每步一个 control_dim 向量。
    _require_shape(l_u, (horizon, control_dim), "l_u")
    # l_xx 每步一个 state_dim x state_dim 矩阵。
    _require_shape(l_xx, (horizon, state_dim, state_dim), "l_xx")
    # l_uu 每步一个 control_dim x control_dim 矩阵。
    _require_shape(l_uu, (horizon, control_dim, control_dim), "l_uu")
    # l_ux 每步一个 control_dim x state_dim 矩阵。
    _require_shape(l_ux, (horizon, control_dim, state_dim), "l_ux")
    # terminal_x 是最终状态上的 state_dim 向量。
    _require_shape(terminal_x, (state_dim,), "terminal_x")
    # terminal_xx 是最终状态上的 state_dim x state_dim 矩阵。
    _require_shape(terminal_xx, (state_dim, state_dim), "terminal_xx")

    # value function 的一阶项从 terminal cost 开始初始化。
    V_x = terminal_x.copy()
    # value function 的二阶项从 terminal Hessian 开始初始化，并强制对称。
    V_xx = _symmetrize(terminal_xx.copy())
    # feedforward 保存每一步的 k_t。
    feedforward = np.zeros((horizon, control_dim), dtype=float)
    # feedback 保存每一步的 K_t。
    feedback = np.zeros((horizon, control_dim, state_dim), dtype=float)
    # 记录每一步近似 expected reduction，便于诊断。
    expected_reductions: list[float] = []

    # control 空间的单位阵，用于 Q_uu regularization。
    eye_u = np.eye(control_dim, dtype=float)
    # backward pass 必须从最后一个 stage 往前递推。
    for t in reversed(range(horizon)):
        # 取出第 t 步的 A_t。
        A = A_all[t]
        # 取出第 t 步的 B_t。
        B = B_all[t]

        # Q_x 是局部 Q 函数对状态的一阶导。
        Q_x = l_x[t] + A.T @ V_x
        # Q_u 是局部 Q 函数对控制的一阶导。
        Q_u = l_u[t] + B.T @ V_x
        # Q_xx 是局部 Q 函数对状态的二阶导。
        Q_xx = l_xx[t] + A.T @ V_xx @ A
        # Q_uu 是局部 Q 函数对控制的二阶导，也是后面要求解的控制 Hessian。
        Q_uu = l_uu[t] + B.T @ V_xx @ B
        # Q_ux 是局部 Q 函数对控制-状态的交叉二阶导。
        Q_ux = l_ux[t] + B.T @ V_xx @ A

        # 给 Q_uu 加正则项，避免矩阵奇异或病态。
        Q_uu_reg = _symmetrize(Q_uu) + float(cfg.regularization) * eye_u
        # 用线性方程求解 k 和 K，不显式求逆，数值上更稳。
        try:
            # k = - Q_uu^{-1} Q_u，是 open-loop feedforward 修正。
            k = -np.linalg.solve(Q_uu_reg, Q_u)
            # K = - Q_uu^{-1} Q_ux，是 closed-loop feedback 修正。
            K = -np.linalg.solve(Q_uu_reg, Q_ux)
        except np.linalg.LinAlgError as exc:
            # 如果 Q_uu 仍不可解，返回失败结果而不是继续传播错误矩阵。
            return ILQGBackwardPassResult(
                feedforward_gains=feedforward,
                feedback_gains=feedback,
                expected_cost_reduction=float(np.sum(expected_reductions)),
                success=False,
                message=f"Q_uu regularized solve failed at t={t}: {exc}",
            )

        # 保存第 t 步的 k_t。
        feedforward[t] = k
        # 保存第 t 步的 K_t。
        feedback[t] = K
        # 记录一个简单的一阶 expected reduction 近似。
        expected_reductions.append(float(-Q_u.T @ k))

        # 更新 value function 的一阶项，传给前一个时间步。
        V_x = Q_x + K.T @ Q_uu @ k + K.T @ Q_u + Q_ux.T @ k
        # 更新 value function 的二阶项，传给前一个时间步。
        V_xx = Q_xx + K.T @ Q_uu @ K + K.T @ Q_ux + Q_ux.T @ K
        # 保持 Hessian 对称，减少浮点误差积累。
        V_xx = _symmetrize(V_xx)

    # backward pass 结束后，检查所有增益是否有限。
    if not np.isfinite(feedforward).all() or not np.isfinite(feedback).all():
        # 如果出现 NaN / inf，返回失败结果，solver 主循环会停止。
        return ILQGBackwardPassResult(
            feedforward_gains=feedforward,
            feedback_gains=feedback,
            expected_cost_reduction=float(np.sum(expected_reductions)),
            success=False,
            message="backward pass produced non-finite gains",
        )

    # 成功时返回整条轨迹的 k/K 以及总 expected reduction。
    return ILQGBackwardPassResult(
        feedforward_gains=feedforward,
        feedback_gains=feedback,
        expected_cost_reduction=float(np.sum(expected_reductions)),
        success=True,
        message="backward pass succeeded",
    )


def forward_pass(
    dynamics_fn: ArrayFn,
    cost_fn: StageCostFn,
    nominal_trajectory: NominalTrajectory,
    backward_result: ILQGBackwardPassResult,
    initial_state: np.ndarray,
    alpha: float,
    config: ILQGConfig,
    terminal_cost_fn: TerminalCostFn | None = None,
) -> ILQGForwardPassResult:
    """用 k/K 在真实动力学上前向 rollout 一条候选轨迹。"""
    # line search 步长必须为正。
    if alpha <= 0.0:
        raise ValueError(f"alpha must be positive, got {alpha}")
    # forward pass 只能消费成功的 backward pass 结果。
    if not backward_result.success:
        raise ValueError(f"backward_result is not successful: {backward_result.message}")

    # 初始状态转换为有限 float 向量。
    x0 = _as_float_array(initial_state, "initial_state")
    # x_bar 是 nominal states。
    x_bar = _as_float_array(nominal_trajectory.states, "nominal_trajectory.states")
    # u_bar 是 nominal controls。
    u_bar = _as_float_array(nominal_trajectory.controls, "nominal_trajectory.controls")
    # k_all 是每一步的 feedforward 修正。
    k_all = _as_float_array(backward_result.feedforward_gains, "feedforward_gains")
    # K_all 是每一步的 feedback 增益。
    K_all = _as_float_array(backward_result.feedback_gains, "feedback_gains")

    # 从 nominal control 序列读取 horizon 和 control_dim。
    horizon, control_dim = u_bar.shape
    # 从初始状态读取 state_dim。
    state_dim = x0.size
    # x_bar 必须是 (H + 1, state_dim)。
    _require_shape(x_bar, (horizon + 1, state_dim), "nominal_trajectory.states")
    # k_all 必须是 (H, control_dim)。
    _require_shape(k_all, (horizon, control_dim), "feedforward_gains")
    # K_all 必须是 (H, control_dim, state_dim)。
    _require_shape(K_all, (horizon, control_dim, state_dim), "feedback_gains")

    # 新候选状态轨迹。
    states = np.zeros((horizon + 1, state_dim), dtype=float)
    # 新候选控制序列。
    controls = np.zeros((horizon, control_dim), dtype=float)
    # 新候选 stage cost 序列。
    costs = np.zeros(horizon, dtype=float)
    # forward pass 从同一个 initial_state 开始。
    states[0] = x0

    # 按时间顺序在真实 dynamics 上前向 rollout。
    for t in range(horizon):
        # dx 表示当前候选状态相对 nominal 状态的偏差。
        dx = states[t] - x_bar[t]
        # iLQR 控制更新：u = u_bar + alpha*k + K*dx。
        u_new = u_bar[t] + float(alpha) * k_all[t] + K_all[t] @ dx
        # 如果设置了控制限幅，就把候选控制裁剪到安全范围。
        if config.control_limit is not None:
            u_new = np.clip(u_new, -float(config.control_limit), float(config.control_limit))
        # 保存第 t 步候选控制。
        controls[t] = u_new

        # 计算第 t 步候选轨迹的 stage cost。
        stage_cost = float(cost_fn(states[t], controls[t], t))
        # cost 必须是有限标量。
        if not np.isfinite(stage_cost):
            raise ValueError(f"cost_fn returned non-finite value at t={t}: {stage_cost}")
        # 保存第 t 步 cost。
        costs[t] = stage_cost

        # 用真实 dynamics 推进到下一状态。
        next_state = _as_float_array(dynamics_fn(states[t], controls[t]), f"dynamics_fn at t={t}")
        # 下一状态 shape 必须保持为 (state_dim,)。
        _require_shape(next_state, (state_dim,), f"dynamics_fn output at t={t}")
        # 写入 x_{t+1}。
        states[t + 1] = next_state

    # terminal cost 默认没有。
    terminal_cost = 0.0
    # 如果提供 terminal_cost_fn，就在最后状态上计算 terminal cost。
    if terminal_cost_fn is not None:
        terminal_cost = float(terminal_cost_fn(states[-1]))
        # terminal cost 必须有限。
        if not np.isfinite(terminal_cost):
            raise ValueError(f"terminal_cost_fn returned non-finite value: {terminal_cost}")
    # 总 cost = stage costs 之和 + terminal cost。
    total_cost = float(np.sum(costs) + terminal_cost)
    # 总 cost 必须有限，才能参与 line search 比较。
    if not np.isfinite(total_cost):
        raise ValueError("forward pass total_cost is non-finite")

    # 返回候选轨迹；是否最终接受由 solve() 的 line search 判断。
    return ILQGForwardPassResult(
        states=states,
        controls=controls,
        costs=costs,
        total_cost=total_cost,
        alpha=float(alpha),
        accepted=True,
        message="forward pass produced a finite candidate",
    )


def linearize_dynamics(
    env: Any,
    nominal_states: np.ndarray,
    nominal_controls: np.ndarray,
    dt: float,
) -> LinearizedDynamics:
    """兼容旧 skeleton 的薄 wrapper。

    env 可以是可调用对象，也可以提供 dynamics_fn/discrete_dynamics 方法。
    dt 只保留为旧接口参数，本 wrapper 不直接使用。
    """
    # dt 是旧接口留下的参数；当前 wrapper 使用外部 dynamics_fn 自己处理时间步。
    _ = dt
    # 如果 env 本身可调用，就把 env 当成 dynamics_fn 使用。
    if callable(env):
        dynamics_fn = env
    # 如果 env 有 dynamics_fn 属性，就使用这个属性作为离散动力学。
    elif hasattr(env, "dynamics_fn"):
        dynamics_fn = env.dynamics_fn
    # 如果 env 有 discrete_dynamics 方法，也可以作为离散动力学。
    elif hasattr(env, "discrete_dynamics"):
        dynamics_fn = env.discrete_dynamics
    # 三种形式都没有时，无法线性化。
    else:
        raise ValueError("linearize_dynamics requires a callable env or env.dynamics_fn/discrete_dynamics")

    # 旧接口直接传 states，这里转成有限 float 数组。
    states = _as_float_array(nominal_states, "nominal_states")
    # 旧接口直接传 controls，这里转成有限 float 数组。
    controls = _as_float_array(nominal_controls, "nominal_controls")
    # wrapper 不关心 cost，只构造一个全零 cost 占位。
    costs = np.zeros(controls.shape[0], dtype=float)
    # 用旧接口输入临时组装 NominalTrajectory。
    nominal = NominalTrajectory(
        states=states,
        controls=controls,
        costs=costs,
        total_cost=float(np.sum(costs)),
    )
    # 复用新实现完成沿轨迹线性化。
    return linearize_trajectory_dynamics(dynamics_fn, nominal)


def quadratize_cost(
    nominal_states: np.ndarray,
    nominal_controls: np.ndarray,
    target_horizon: np.ndarray,
    cost_config: dict[str, Any],
) -> QuadraticCostApproximation:
    """兼容旧 skeleton 的 cost wrapper。"""
    # 旧接口传入 nominal_states，这里检查并转成 float 数组。
    states = _as_float_array(nominal_states, "nominal_states")
    # 旧接口传入 nominal_controls，这里检查并转成 float 数组。
    controls = _as_float_array(nominal_controls, "nominal_controls")
    # 从 states 的第二维读出 state_dim。
    state_dim = states.shape[1]
    # 从 controls 的第二维读出 control_dim。
    control_dim = controls.shape[1]
    # 把旧 target_horizon 统一成 (H + 1, state_dim) 的 x_refs。
    x_refs = _normalize_x_refs(target_horizon, horizon=controls.shape[0], state_dim=state_dim)
    # 从 cost_config 读取 Q；缺省时用单位阵。
    Q = _matrix_from_config(cost_config, "Q", state_dim, default_scale=1.0)
    # 从 cost_config 读取 R；缺省时用单位阵。
    R = _matrix_from_config(cost_config, "R", control_dim, default_scale=1.0)
    # 从 cost_config 读取 Q_terminal；缺省时用单位阵。
    Q_terminal = _matrix_from_config(cost_config, "Q_terminal", state_dim, default_scale=1.0)
    # 复用新实现生成 quadratic cost approximation。
    return quadratize_trajectory_cost(states, controls, x_refs, Q, R, Q_terminal)


def _matrix_from_config(config: dict[str, Any], key: str, dim: int, default_scale: float) -> np.ndarray:
    # 如果配置里显式提供了矩阵，就读取它。
    if key in config:
        matrix = _as_float_array(config[key], key)
    # 如果没有提供，就用 default_scale * I 作为教学默认值。
    else:
        matrix = np.eye(dim, dtype=float) * float(default_scale)
    # 矩阵维度必须和对应状态/控制维度一致。
    _require_shape(matrix, (dim, dim), key)
    # 返回可直接用于 cost 的矩阵。
    return matrix


def _normalize_x_refs(value: Any, *, horizon: int, state_dim: int) -> np.ndarray:
    # 把输入参考轨迹转成有限 float 数组。
    x_refs = _as_float_array(value, "x_refs")
    # 最标准的形式是 (H + 1, state_dim)，包含 terminal reference。
    if x_refs.shape == (horizon + 1, state_dim):
        return x_refs
    # 如果只给了 H 个 stage reference，就复制最后一个作为 terminal reference。
    if x_refs.shape == (horizon, state_dim):
        return np.vstack([x_refs, x_refs[-1]])
    # 其他 shape 都不明确，直接报错并说明期望维度。
    raise ValueError(
        "x_refs must have shape (horizon + 1, state_dim) or (horizon, state_dim), "
        f"got {x_refs.shape} for horizon={horizon}, state_dim={state_dim}"
    )


def _resolve_config(problem: MPCProblem) -> ILQGConfig:
    # solver_config 是当前 solver 的主要配置来源。
    cfg = dict(problem.solver_config or {})
    # metadata 作为补充配置来源，便于 toy demo 或外部调用传参。
    metadata = dict(problem.metadata or {})
    # line_search_alphas 优先从 solver_config 读取，其次从 metadata 读取，最后用默认值。
    line_search_alphas_raw = cfg.get("line_search_alphas", metadata.get("line_search_alphas", (1.0, 0.5, 0.25, 0.1, 0.05)))
    # 把所有 alpha 转成 float，并固定为 tuple。
    line_search_alphas = tuple(float(alpha) for alpha in line_search_alphas_raw)
    # line search 至少需要一个候选步长。
    if not line_search_alphas:
        raise ValueError("line_search_alphas must contain at least one alpha")
    # alpha 必须为正，否则 forward pass 的更新方向没有明确意义。
    if any(alpha <= 0.0 for alpha in line_search_alphas):
        raise ValueError(f"line_search_alphas must be positive, got {line_search_alphas}")

    # 控制限幅优先使用 control_limit，也兼容 sampling solver 常用的 torque_limit。
    control_limit_raw = cfg.get("control_limit", cfg.get("torque_limit", metadata.get("control_limit")))
    # None 表示不做控制限幅。
    control_limit = None if control_limit_raw is None else float(control_limit_raw)
    # 如果提供限幅，必须是正数。
    if control_limit is not None and control_limit <= 0.0:
        raise ValueError(f"control_limit must be positive when provided, got {control_limit}")

    # 打包成 ILQGConfig，供 solve/backward/forward 使用。
    return ILQGConfig(
        horizon=int(problem.horizon),
        max_iterations=int(cfg.get("max_iterations", cfg.get("num_iterations", metadata.get("max_iterations", 10)))),
        line_search=bool(cfg.get("line_search", metadata.get("line_search", True))),
        regularization=float(cfg.get("regularization", metadata.get("regularization", 1e-6))),
        finite_difference_eps=float(cfg.get("finite_difference_eps", metadata.get("finite_difference_eps", 1e-5))),
        line_search_alphas=line_search_alphas,
        tolerance=float(cfg.get("tolerance", cfg.get("improvement_tolerance", metadata.get("tolerance", 1e-6)))),
        control_limit=control_limit,
    )


def _resolve_dynamics_fn(problem: MPCProblem) -> ArrayFn:
    # dynamics_fn 优先使用 MPCProblem 顶层字段，也兼容 metadata 中的写法。
    dynamics_fn = problem.dynamics_fn or problem.metadata.get("dynamics_fn")
    # B03-R4B 必须显式提供 dynamics_fn，不在 solver 内写 MuJoCo 细节。
    if dynamics_fn is None:
        raise ValueError("B03-R4B ILQGLiteSolver requires problem.dynamics_fn or metadata['dynamics_fn'].")
    # dynamics_fn 必须可调用，接口约定是 dynamics_fn(x, u) -> x_next。
    if not callable(dynamics_fn):
        raise ValueError("dynamics_fn must be callable as dynamics_fn(x, u) -> x_next.")
    # 返回解析出的动力学函数。
    return dynamics_fn


def _resolve_problem_arrays(problem: MPCProblem, state_dim: int, control_dim: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    # metadata 中通常放 x_refs/Q/R/Q_terminal。
    metadata = dict(problem.metadata or {})
    # horizon 来自统一 MPCProblem。
    horizon = int(problem.horizon)

    # 优先使用 metadata["x_refs"] 作为参考状态轨迹。
    if "x_refs" in metadata:
        x_refs = _normalize_x_refs(metadata["x_refs"], horizon=horizon, state_dim=state_dim)
    # 如果 target_horizon 已经是状态参考轨迹，也允许直接使用。
    elif np.asarray(problem.target_horizon).shape == (horizon + 1, state_dim):
        x_refs = _normalize_x_refs(problem.target_horizon, horizon=horizon, state_dim=state_dim)
    # 否则当前 B03-R4B 不猜测如何从 target_horizon 构造状态参考。
    else:
        raise ValueError(
            "B03-R4B ILQGLiteSolver requires x_refs in problem.metadata or "
            "problem.target_horizon with shape (horizon + 1, state_dim)."
        )

    # cost_config 和 metadata 合并；metadata 可以覆盖或补充 Q/R/Q_terminal。
    merged_cost_config = {**dict(problem.cost_config or {}), **metadata}
    # 读取状态 stage 权重 Q。
    Q = _matrix_from_config(merged_cost_config, "Q", state_dim, default_scale=1.0)
    # 读取控制 stage 权重 R。
    R = _matrix_from_config(merged_cost_config, "R", control_dim, default_scale=1.0)
    # 读取 terminal 状态权重 Q_terminal。
    Q_terminal = _matrix_from_config(merged_cost_config, "Q_terminal", state_dim, default_scale=1.0)
    # 返回 solve() 需要的参考轨迹和权重矩阵。
    return x_refs, Q, R, Q_terminal


def _build_tracking_cost_fns(
    x_refs: np.ndarray,
    Q: np.ndarray,
    R: np.ndarray,
    Q_terminal: np.ndarray,
) -> tuple[StageCostFn, TerminalCostFn]:
    # stage_cost_fn 计算每一步 tracking cost。
    def stage_cost_fn(x: np.ndarray, u: np.ndarray, t: int) -> float:
        # dx 是当前状态相对第 t 个参考状态的误差。
        dx = np.asarray(x, dtype=float) - x_refs[t]
        # u_arr 是当前控制向量。
        u_arr = np.asarray(u, dtype=float)
        # 返回 1/2 dx^T Q dx + 1/2 u^T R u。
        return float(0.5 * dx.T @ Q @ dx + 0.5 * u_arr.T @ R @ u_arr)

    # terminal_cost_fn 计算最后状态的 terminal tracking cost。
    def terminal_cost_fn(x: np.ndarray) -> float:
        # terminal dx 使用最后一个参考状态 x_refs[-1]。
        dx = np.asarray(x, dtype=float) - x_refs[-1]
        # 返回 1/2 dx^T Q_terminal dx。
        return float(0.5 * dx.T @ Q_terminal @ dx)

    # 返回两个闭包，rollout/forward pass 会调用它们。
    return stage_cost_fn, terminal_cost_fn


def _resolve_cost_model(
    problem: MPCProblem,
    state_dim: int,
    control_dim: int,
) -> tuple[StageCostFn, TerminalCostFn | None, str, tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None]:
    """解析 iLQR-lite 使用的 cost 模型。

    默认仍使用 R4C 的 state-quadratic tracking；当 metadata 显式提供
    `stage_cost_fn` 时，切换到通用 cost + 有限差分二次化路径。
    """
    metadata = dict(problem.metadata or {})
    stage_cost_candidate = metadata.get("stage_cost_fn")
    if stage_cost_candidate is not None:
        if not callable(stage_cost_candidate):
            raise ValueError("metadata['stage_cost_fn'] must be callable when provided.")
        terminal_cost_candidate = metadata.get("terminal_cost_fn")
        if terminal_cost_candidate is not None and not callable(terminal_cost_candidate):
            raise ValueError("metadata['terminal_cost_fn'] must be callable when provided.")
        return stage_cost_candidate, terminal_cost_candidate, "custom_finite_difference", None

    x_refs, Q, R, Q_terminal = _resolve_problem_arrays(problem, state_dim, control_dim)
    stage_cost_fn, terminal_cost_fn = _build_tracking_cost_fns(x_refs, Q, R, Q_terminal)
    return stage_cost_fn, terminal_cost_fn, "state_quadratic", (x_refs, Q, R, Q_terminal)


def _compute_predicted_ee_positions(
    problem: MPCProblem,
    states: np.ndarray,
) -> np.ndarray | None:
    """如果 problem 提供 end_effector_fn，就把 predicted_states 映射到末端 xy。"""
    end_effector_fn = dict(problem.metadata or {}).get("end_effector_fn")
    if end_effector_fn is None:
        return None
    if not callable(end_effector_fn):
        raise ValueError("metadata['end_effector_fn'] must be callable when provided.")

    X = _as_float_array(states, "states")
    if X.ndim != 2:
        raise ValueError(f"states must be 2D, got shape={X.shape}")
    ee_positions = []
    for index, state in enumerate(X):
        ee_xy = _as_float_array(end_effector_fn(state), f"end_effector_fn(states[{index}])")
        if ee_xy.shape != (2,):
            raise ValueError(f"end_effector_fn must return shape (2,), got {ee_xy.shape}")
        ee_positions.append(ee_xy)
    return np.asarray(ee_positions, dtype=float)


def _resolve_initial_controls(
    problem: MPCProblem,
    previous_solution: MPCSolution | None,
) -> np.ndarray:
    # 初始控制序列必须匹配 (H, control_dim)。
    expected_shape = (int(problem.horizon), int(problem.control_dim))
    # 如果上一拍有解，优先尝试 warm start。
    if previous_solution is not None and previous_solution.predicted_controls is not None:
        # 取出上一拍预测控制序列。
        previous_controls = np.asarray(previous_solution.predicted_controls, dtype=float)
        # 只有 shape 合法且数值有限时才复用。
        if previous_controls.shape == expected_shape and np.isfinite(previous_controls).all():
            # MPC warm start：左移一格，最后一步保持上一序列末端。
            return shift_control_sequence(previous_controls)

    # 如果 metadata 显式提供 initial_controls，则作为第二优先级。
    if "initial_controls" in problem.metadata:
        # 检查 initial_controls 是否有限。
        initial_controls = _as_float_array(problem.metadata["initial_controls"], "initial_controls")
        # 检查 shape 是否匹配 horizon/control_dim。
        _require_shape(initial_controls, expected_shape, "initial_controls")
        # 返回副本，避免后续迭代修改调用者原数组。
        return initial_controls.copy()

    # 没有 warm start 和 explicit initial controls 时，用零控制序列。
    return np.zeros(expected_shape, dtype=float)


class ILQGLiteSolver(BaseMPCSolver):
    """B03 mini deterministic iLQR-lite solver."""

    solver_name = "ilqg_lite"

    def solve(
        self,
        problem: MPCProblem,
        previous_solution: MPCSolution | None = None,
    ) -> MPCSolution:
        """执行最小可运行 iLQR-lite。

        当前版本只要求外部提供 `dynamics_fn` 与 tracking cost 所需的 `x_refs/Q/R/Q_terminal`。
        """
        # 记录起始时间，用于最后写入 SolverStats.runtime_ms。
        start_time = time.perf_counter()
        # 从 MPCProblem 解析 iLQR 配置。
        cfg = _resolve_config(problem)
        # 迭代次数必须为正，否则 solve 没有可执行的优化循环。
        if cfg.max_iterations <= 0:
            raise ValueError(f"max_iterations must be positive, got {cfg.max_iterations}")
        # config horizon 必须与 problem horizon 对齐。
        if cfg.horizon != int(problem.horizon):
            raise ValueError("ILQGConfig horizon must match problem.horizon")

        # 解析外部提供的离散动力学函数。
        dynamics_fn = _resolve_dynamics_fn(problem)
        # 当前状态是本次 MPC 求解的初始状态 x0。
        initial_state = _as_float_array(problem.current_state, "problem.current_state")
        # 当前实现要求 current_state 是一维向量。
        if initial_state.ndim != 1:
            raise ValueError(f"problem.current_state must be 1D, got shape={initial_state.shape}")
        # 状态维度由 current_state 长度决定。
        state_dim = initial_state.size
        # 控制维度来自统一 MPCProblem。
        control_dim = int(problem.control_dim)
        # 解析 cost 模型：默认 state quadratic；R4D 可提供通用 task-space cost。
        stage_cost_fn, terminal_cost_fn, cost_model, quadratic_tracking_arrays = _resolve_cost_model(
            problem,
            state_dim,
            control_dim,
        )

        # 解析初始控制序列：previous_solution -> initial_controls -> zeros。
        controls = _resolve_initial_controls(problem, previous_solution)
        # 如果配置了控制限幅，先把初始控制序列裁剪到允许范围。
        if cfg.control_limit is not None:
            controls = np.clip(controls, -float(cfg.control_limit), float(cfg.control_limit))

        # 对初始控制序列做一次 rollout，建立当前 best 轨迹。
        best_nominal = rollout_nominal_trajectory(
            dynamics_fn=dynamics_fn,
            initial_state=initial_state,
            controls=controls,
            cost_fn=stage_cost_fn,
            terminal_cost_fn=terminal_cost_fn,
        )
        # best_states 保存当前最好状态轨迹。
        best_states = best_nominal.states
        # best_controls 保存当前最好控制序列。
        best_controls = best_nominal.controls
        # best_cost 保存当前最好总 cost。
        best_cost = float(best_nominal.total_cost)

        # cost_history 从初始 rollout 的 cost 开始记录。
        cost_history: list[float] = [best_cost]
        # accepted_alphas 记录每次成功 line search 使用的 alpha。
        accepted_alphas: list[float] = []
        # 统计 forward pass 候选 rollout 次数。
        num_forward_rollouts = 0
        # 统计实际完成的 backward/iteration 次数。
        actual_iterations = 0
        # success 标记 solver 是否遇到硬失败。
        success = True
        # 默认 message；如果提前停止，后面会覆盖。
        message = "max_iterations reached"
        # 默认终止原因；如果提前停止，后面会覆盖。
        termination_reason = "max_iterations"

        # iLQR 主循环：每次围绕当前 controls 重新构造局部 LQ 子问题。
        for _iteration in range(cfg.max_iterations):
            # 1. nominal rollout：用当前 controls 生成 nominal trajectory。
            nominal = rollout_nominal_trajectory(
                dynamics_fn=dynamics_fn,
                initial_state=initial_state,
                controls=controls,
                cost_fn=stage_cost_fn,
                terminal_cost_fn=terminal_cost_fn,
            )
            # 2. dynamics linearization：沿 nominal trajectory 求 A_t/B_t。
            linearized = linearize_trajectory_dynamics(
                dynamics_fn=dynamics_fn,
                nominal_trajectory=nominal,
                eps=cfg.finite_difference_eps,
            )
            # 3. cost quadratization：沿 nominal trajectory 写出 cost 导数。
            if quadratic_tracking_arrays is None:
                quad_cost = quadratize_custom_trajectory_cost(
                    states=nominal.states,
                    controls=nominal.controls,
                    stage_cost_fn=stage_cost_fn,
                    terminal_cost_fn=terminal_cost_fn,
                    eps=cfg.finite_difference_eps,
                )
            else:
                x_refs, Q, R, Q_terminal = quadratic_tracking_arrays
                quad_cost = quadratize_trajectory_cost(
                    states=nominal.states,
                    controls=nominal.controls,
                    x_refs=x_refs,
                    Q=Q,
                    R=R,
                    Q_terminal=Q_terminal,
                )
            # 4. backward pass：求 k_t/K_t。
            backward = backward_pass(linearized, quad_cost, cfg)
            # 只要跑完一次 backward 尝试，就计入一次 iteration。
            actual_iterations += 1
            # 如果 backward pass 失败，停止并返回当前 best。
            if not backward.success:
                success = False
                message = backward.message
                termination_reason = "backward_failed"
                break

            # 当前还没有找到被 line search 接受的候选轨迹。
            accepted_candidate: ILQGForwardPassResult | None = None
            # 如果开启 line search，逐个尝试配置中的 alpha；否则只用 alpha=1。
            alphas: Sequence[float] = cfg.line_search_alphas if cfg.line_search else (1.0,)
            # 5. forward pass with line search：尝试不同步长。
            for alpha in alphas:
                # 用真实 dynamics rollout 一条候选轨迹。
                candidate = forward_pass(
                    dynamics_fn=dynamics_fn,
                    cost_fn=stage_cost_fn,
                    nominal_trajectory=nominal,
                    backward_result=backward,
                    initial_state=initial_state,
                    alpha=float(alpha),
                    config=cfg,
                    terminal_cost_fn=terminal_cost_fn,
                )
                # 记录一次 forward rollout。
                num_forward_rollouts += 1
                # 只有 candidate cost 低于当前 nominal cost 才接受。
                if candidate.total_cost < nominal.total_cost:
                    accepted_candidate = candidate
                    break

            # 所有 alpha 都不能降低 cost 时，停止优化。
            if accepted_candidate is None:
                message = "line search failed to improve nominal cost; returning current best trajectory"
                termination_reason = "line_search_failed"
                break

            # 计算本次接受更新带来的 cost 改善量。
            improvement = float(nominal.total_cost - accepted_candidate.total_cost)
            # 下一轮 iLQR 以接受后的控制序列为 nominal controls。
            controls = accepted_candidate.controls
            # 更新当前 best control sequence。
            best_controls = accepted_candidate.controls
            # 更新当前 best state trajectory。
            best_states = accepted_candidate.states
            # 更新当前 best cost。
            best_cost = float(accepted_candidate.total_cost)
            # 把 accepted candidate 包装成 NominalTrajectory，便于最终有效性检查。
            best_nominal = NominalTrajectory(
                states=accepted_candidate.states,
                controls=accepted_candidate.controls,
                costs=accepted_candidate.costs,
                total_cost=accepted_candidate.total_cost,
            )
            # 记录新的 best cost。
            cost_history.append(best_cost)
            # 记录本次接受的 line search alpha。
            accepted_alphas.append(float(accepted_candidate.alpha))

            # 如果改善量小于阈值，认为已经收敛。
            if improvement < cfg.tolerance:
                message = "cost improvement below tolerance"
                termination_reason = "tolerance"
                break

        # 最后确认至少有一条 best trajectory，并且状态轨迹 shape 合法。
        if not cost_history or best_nominal.states.shape != (int(problem.horizon) + 1, state_dim):
            raise RuntimeError("ILQGLiteSolver failed to produce a valid best trajectory.")

        # 如果没有提前停止，说明达到最大迭代次数。
        if termination_reason == "max_iterations":
            message = "max_iterations reached"

        # 如果提供 end_effector_fn，就把 state trajectory 映射成 task-space 轨迹。
        predicted_ee_positions = _compute_predicted_ee_positions(problem, best_states)

        # 按统一 B03 MPCSolution schema 返回结果。
        return MPCSolution(
            # MPC 只执行第一步控制。
            first_control=np.asarray(best_controls[0], dtype=float),
            # predicted_states 是本次 solver 认为的未来状态轨迹。
            predicted_states=np.asarray(best_states, dtype=float),
            # predicted_controls 是本次 solver 认为的未来控制序列。
            predicted_controls=np.asarray(best_controls, dtype=float),
            # best_cost 是当前最佳轨迹总 cost。
            best_cost=float(best_cost),
            # solver_name 用于日志和 benchmark 区分 solver family。
            solver_name=self.solver_name,
            # solver_stats 保存 runtime、rollout 次数、迭代次数和状态。
            solver_stats=SolverStats(
                runtime_ms=(time.perf_counter() - start_time) * 1000.0,
                num_rollouts=int(num_forward_rollouts),
                num_iterations=int(actual_iterations),
                success=bool(success and np.isfinite(best_cost)),
                message=message,
            ),
            predicted_ee_positions=predicted_ee_positions,
            # metadata 保存教学和调试信息。
            metadata={
                "cost_history": cost_history,
                "accepted_alphas": accepted_alphas,
                "termination_reason": termination_reason,
                "cost_model": cost_model,
            },
        )
