"""B02 two-link MPC controller skeleton."""

from __future__ import annotations

from typing import Any


class TwoLinkMPCController:
    """二连杆 MPC 控制器骨架。

    控制器负责把当前状态和未来末端目标轨迹交给 planner，
    并返回当前控制步要执行的第一个双关节 torque。
    """

    def __init__(self, planner: Any) -> None:
        """初始化控制器。

        TODO:
        - 要实现什么：保存 planner 和最近一次规划结果。
        - 为什么要实现：控制器需要调用 planner，并保留 last_plan 供日志和 cost 曲线使用。
        - 输入是什么：二连杆 predictive sampling planner。
        - 输出是什么：初始化后的控制器对象。
        - 物理意义：控制器是 torque 命令的直接来源。
        - 数学意义：封装 receding horizon 中的 `u_0` 选择。
        - 验证标准：`compute_control()` 后应更新 `last_plan`。
        """
        self.planner = planner
        self.last_plan: dict[str, Any] | None = None

    def compute_control(
        self,
        env: Any,
        current_state: tuple[float, float, float, float],
        target_sequence: list[tuple[float, float]],
    ) -> tuple[float, float]:
        """计算当前控制步的双关节 torque。

        TODO:
        - 要实现什么：调用 planner.plan，并返回 best sequence 的第一项 `[tau1,tau2]`。
        - 为什么要实现：MPC 每轮规划未来 horizon，但只执行当前第一步。
        - 输入是什么：环境、当前状态、未来目标末端轨迹。
        - 输出是什么：当前要执行的 `(tau1,tau2)`。
        - 物理意义：两个 actuator 的当前力矩命令。
        - 数学意义：从最优序列 `u_0:H-1` 中取 `u_0`。
        - 验证标准：返回值长度为 2，且在 torque limit 内。
        """
        plan_result = self.planner.plan(
            env=env,
            current_state=current_state,
            target_sequence=target_sequence,
        )
        self.last_plan = plan_result

        if "best_torque" not in plan_result:
            raise KeyError("planner.plan() 的返回结果缺少 best_torque。")

        best_torque = plan_result["best_torque"]
        if len(best_torque) != 2:
            raise ValueError("planner.plan() 返回的 best_torque 必须是 (tau1, tau2)。")

        tau1, tau2 = best_torque
        return float(tau1), float(tau2)
