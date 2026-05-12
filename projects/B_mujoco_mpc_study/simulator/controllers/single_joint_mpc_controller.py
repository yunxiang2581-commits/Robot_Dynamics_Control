"""B01 single-joint MPC controller skeleton."""

from __future__ import annotations

from typing import Any


class SingleJointMPCController:
    """单关节 MPC 控制器骨架。

    控制器负责把当前状态和目标角交给 planner，并返回当前步要执行的第一个 torque。
    """

    def __init__(self, planner: Any, q_target: float = 0.0) -> None:
        """初始化控制器。

        TODO:
        - 要实现什么：保存 planner 和目标角度。
        - 为什么要实现：控制器需要持有目标，并在每个控制周期调用 planner。
        - 输入是什么：`planner` 是 predictive sampling planner，`q_target` 是目标关节角。
        - 输出是什么：初始化后的控制器对象。
        - 物理意义：定义单关节想要到达的目标姿态。
        - 数学意义：固定 cost 中的参考值 `q_target`。
        - 验证标准：目标角可被 `update_target()` 修改，并被 `compute_control()` 使用。
        """
        self.planner = planner
        self.q_target = q_target
        self.last_plan: dict[str, Any] | None = None

    def update_target(self, q_target: float) -> None:
        """更新目标角度。

        TODO:
        - 要实现什么：修改控制器内部保存的目标角。
        - 为什么要实现：后续可以支持 step target 或 trajectory target。
        - 输入是什么：新的目标角 `q_target`。
        - 输出是什么：无返回值，但内部目标应更新。
        - 物理意义：改变关节希望跟踪的位置。
        - 数学意义：改变 residual `q - q_target` 中的参考值。
        - 验证标准：更新后下一次规划应使用新目标。
        """
        self.q_target = q_target

    def compute_control(
        self,
        env: Any,
        current_state: tuple[float, float],
        q_target_sequence: list[float] | None = None,
    ) -> float:
        """计算当前控制步的 torque。

        TODO:
        - 要实现什么：调用 planner.plan，并返回 best sequence 的第一个 torque。
        - 为什么要实现：receding horizon control 每次只执行未来最优序列的第一步。
        - 输入是什么：环境 `env` 和当前状态 `(q,dq)`。
        - 输出是什么：当前步要执行的 torque。
        - 物理意义：给单关节 actuator 的实际力矩命令。
        - 数学意义：从有限时域最优序列 `u_0:H-1` 中取 `u_0`。
        - 验证标准：返回值应在 torque limit 内，并保存 last_plan 方便日志和 cost 曲线。
        """
        # 如果 runner 传入 horizon 内的目标序列，planner 会按 q_target[k] 计算 cost。
        # 如果没有传入，则退回旧逻辑：使用控制器内部保存的常数目标 self.q_target。
        # 这样 step target 和 smooth/ramp target 可以共用同一个控制器入口。
        planner_target = q_target_sequence if q_target_sequence is not None else self.q_target

        plan_result = self.planner.plan(
            env=env,
            current_state=current_state,
            q_target=planner_target,
        )
        self.last_plan = plan_result

        if "best_torque" not in plan_result:
            raise KeyError("planner.plan() 的返回结果缺少 best_torque。")

        return float(plan_result["best_torque"])
