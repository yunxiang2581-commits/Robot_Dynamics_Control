"""B03 MPC solver unified interface.

本模块的作用是把不同风格的 MPC solver 放到统一输入/输出接口下：
- runner 不需要知道内部是 shooting、CEM、MPPI、iLQG 还是 SQP；
- logger 和 visualizer 只消费 `MPCSolution` 与 `SolverStats`；
- 后续 B03-R1/R2 可以在不改外层调用方式的前提下逐步补 solver 内部逻辑。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np


@dataclass(frozen=True)
class MPCProblem:
    """统一 solver 输入问题描述。"""

    current_state: np.ndarray
    target_horizon: np.ndarray
    horizon: int
    control_dim: int
    dt: float
    cost_config: dict[str, Any]
    solver_config: dict[str, Any]
    rollout_cost_fn: Callable[[np.ndarray, "MPCProblem"], Any] | None = None
    dynamics_fn: Callable[..., Any] | None = None
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class SolverStats:
    """统一 solver 运行统计。"""

    runtime_ms: float
    num_rollouts: int
    num_iterations: int
    success: bool
    message: str = ""


@dataclass(frozen=True)
class MPCSolution:
    """统一 solver 输出结果。"""

    first_control: np.ndarray
    predicted_states: np.ndarray | None
    predicted_controls: np.ndarray
    best_cost: float
    solver_name: str
    solver_stats: SolverStats
    predicted_ee_positions: np.ndarray | None = None
    selected_index: int | None = None
    metadata: dict[str, Any] = field(default_factory=dict)


class BaseMPCSolver:
    """B03 所有 solver 的统一抽象基类。

    设计目的：
    - sampling solver 和 trajectory-optimization solver 共享外层调用方式；
    - runner 不把 solver 内部逻辑写死；
    - 可视化和 benchmark 只依赖标准输出结构。
    """

    solver_name = "base_solver"

    def solve(
        self,
        problem: MPCProblem,
        previous_solution: MPCSolution | None = None,
    ) -> MPCSolution:
        """TODO: 由各具体 solver 返回当前控制周期的 `MPCSolution`。

        输入：
        - `problem.current_state`：当前时刻状态。
        - `problem.target_horizon`：未来 horizon 参考目标。
        - `previous_solution`：上一控制周期的最优预测结果，可用于 warm start。

        输出：
        - `first_control`：当前真正要施加到环境的一步控制。
        - `predicted_states / predicted_controls`：当前 solver 认为的最优未来轨迹。
        - `best_cost`：当前最优目标值。
        - `solver_stats`：rollout 数、迭代数、耗时和成功状态。

        验证标准：
        - 不同 solver 都能返回统一 schema。
        """
        _ = problem, previous_solution
        raise NotImplementedError("B03-A only defines the BaseMPCSolver interface.")
