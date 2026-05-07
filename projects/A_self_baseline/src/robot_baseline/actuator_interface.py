"""Actuator interface TODO skeleton.

A07 是唯一规划 ``data.ctrl`` 和 ``mujoco.mj_step`` 控制闭环的接口层。
Step R-C 只定义骨架，不执行 actuator tracking，不启动 viewer，不录 video。
"""

from __future__ import annotations

from typing import Any

import numpy as np


def inspect_actuators(model: Any) -> dict[str, Any]:
    """检查 actuator 信息。

    TODO:
    - 要做什么：读取 ``model.nu``、actuator names、ctrlrange、actuator_trnid。
    - 为什么：A07 要知道 q_des 如何映射到 actuator command。
    - 对标 mink：``arm_ur5e_actuators.py`` 中的 actuator control。
    - 输入：MuJoCo model。
    - 输出：actuator summary dict。
    - 验证：actuator 数量与 ``model.nu`` 一致。
    """
    return {"nu": getattr(model, "nu", None), "status": "todo_inspect_actuators"}


def validate_actuator_names(model: Any, names: list[str]) -> list[str]:
    """验证 actuator name 列表。

    当前只保留接口，后续读取 MuJoCo actuator names 后做精确匹配和候选提示。
    """
    if not isinstance(names, list):
        raise TypeError("names must be list[str]")
    return names


def map_q_des_to_ctrl(q_des: np.ndarray, control_mode: str) -> np.ndarray:
    """规划 q_des 到 ctrl 的映射。

    TODO:
    - position actuator 第一版可规划 ``ctrl = q_des[actuated_dofs]``。
    - torque / velocity actuator 需要不同物理意义，不能混用。
    - 当前不写 ``data.ctrl``，只返回未来映射接口。
    """
    raise NotImplementedError(f"TODO R8: 实现 {control_mode} mode 的 q_des -> ctrl 映射。")


def plan_pd_tracking_loop() -> None:
    """规划 PD tracking loop。

    TODO:
    - 输入：q_des trajectory、q_actual、qvel、kp、kd。
    - 输出：ctrl command 和 tracking log。
    - A07 才允许写 ``data.ctrl``。
    - A07 才允许调用 ``mujoco.mj_step``。
    """
    raise NotImplementedError("TODO R8: 规划并实现 A07 PD / position actuator tracking loop。")


def run_actuator_tracking_loop(*args: Any, **kwargs: Any) -> None:
    """A07 actuator tracking 主循环接口。

    当前不实现完整 tracking，不启动 viewer，不录 video。
    """
    raise NotImplementedError("TODO R8: A07 才实现 actuator tracking control loop。")


def write_actuator_tracking_report(*args: Any, **kwargs: Any) -> None:
    """写 actuator tracking report 的接口。"""
    raise NotImplementedError("TODO R8: 写 A07 tracking error / ctrlrange / trajectory_source 报告。")
