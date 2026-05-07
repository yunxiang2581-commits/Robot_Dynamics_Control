"""A 项目四接口统一 schema。

本模块只定义数据结构，不启动 MuJoCo，不求解 IK，不调用 mink，也不写
``data.ctrl``。

四接口学习边界：
- Target interface: 目标从哪里来。
- IK interface: 如何由 target 求 ``q_traj``。
- Viewer interface: 如何规划 viewer / mocap target。
- Actuator interface: 如何由 ``q_traj`` 进入 MuJoCo actuator tracking。

姿态约定：
- 本项目配置和 JSON 中的 quaternion 统一使用 ``wxyz`` 顺序。
- SciPy ``Rotation.from_quat`` 使用 ``xyzw`` 顺序，后续真正接入 SciPy 时必须显式转换。
- 内部误差计算推荐使用 rotation matrix，避免直接用 RPY 做姿态误差。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass
class TargetDefinition:
    """目标定义 schema。

    字段含义：
    - ``target_id``: 目标编号，例如 ``A06_fixed_target``。
    - ``source``: 目标来源，例如 ``offset_from_current``、``fixed_pose``、``mocap``。
    - ``target_site``: 末端 site 名称，例如 ``attachment_site``。
    - ``target_body``: 可选 body 名称，例如 ``wrist_3_link``。
    - ``coordinate_frame``: 坐标系，第一版建议使用 ``world``。
    - ``position``: 目标位置，shape=(3,)，单位 m。
    - ``rotation_matrix``: 目标姿态旋转矩阵，shape=(3, 3)。
    - ``quat_wxyz``: 目标姿态四元数，shape=(4,)，顺序 wxyz。
    - ``position_offset``: 相对当前末端位置的偏移，shape=(3,)，单位 m。
    - ``orientation_mode``: ``keep_current``、``fixed_rpy`` 或 ``fixed_quat``。
    - ``metadata``: 记录来源、时间戳、验证信息等辅助字段。
    """

    target_id: str
    source: str
    target_site: str
    target_body: str | None = None
    coordinate_frame: str = "world"
    position: np.ndarray | None = None
    rotation_matrix: np.ndarray | None = None
    quat_wxyz: np.ndarray | None = None
    position_offset: np.ndarray | None = None
    orientation_mode: str = "keep_current"
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class IkRequest:
    """IK 请求 schema。

    A04 和 A05 后续都应通过这个结构进入 IK interface：
    - A04: ``solver_type="dls"``，无约束 DLS IK。
    - A05: ``solver_type="qp_scipy"`` 或 ``"qp_osqp"``，带 box limit 的 QP-IK。

    ``q_init`` 是初始关节位置，shape=(nq,)。``weights``、``limits`` 和
    ``solver_config`` 只保存配置，不在本 schema 中执行算法。
    """

    solver_type: str
    task_mode: str
    site_name: str
    q_init: np.ndarray | None = None
    target: TargetDefinition | None = None
    weights: dict[str, float] = field(default_factory=dict)
    limits: dict[str, Any] = field(default_factory=dict)
    solver_config: dict[str, Any] = field(default_factory=dict)
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class IkResult:
    """IK 输出 schema。

    ``q_traj`` 是 IK 产生的关节轨迹，shape=(N, nq)。``error_rows`` 和
    ``constraint_rows`` 用于写 CSV。当前 Step R-C 只定义结构，不生成真实输出。
    """

    q_traj: np.ndarray | None = None
    error_rows: list[dict[str, Any]] = field(default_factory=list)
    constraint_rows: list[dict[str, Any]] = field(default_factory=list)
    converged: bool = False
    stop_reason: str = "not_started"
    solver_status: str = "todo"
    trajectory_source: str | None = None
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class TrajectorySource:
    """轨迹来源 schema。

    A06 只记录 target 和 trajectory metadata；A07 后续消费这里的路径读取
    ``q_des``，再进入 actuator tracking。
    """

    name: str
    path: str
    source_step: str
    shape: tuple[int, ...] | None = None
    description: str = ""


@dataclass
class ViewerTargetSpec:
    """viewer target 规划 schema。

    viewer target 是在 viewer 中可视化或交互移动的任务目标。mocap-style target
    用 MuJoCo mocap body 表示，其 pose 来自 ``data.mocap_pos`` /
    ``data.mocap_quat``。这些 pose 只 feed IK，不是 actuator control。
    """

    enabled: bool = False
    viewer_mode: str = "mouse_keyboard"
    derived_mjcf: str = ""
    mocap_body_name: str = "a06_target"
    mocap_site_name: str = "a06_target_site"
    keyboard_step: float = 0.01
    enable_ik_follow: bool = False
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class ActuatorTrackingSpec:
    """A07 actuator tracking 规划 schema。

    A07 是唯一允许规划 ``data.ctrl`` 和 ``mujoco.mj_step`` 控制闭环的层。
    Step R-C 只定义接口字段，不执行控制。
    """

    enabled: bool = False
    trajectory_source: str = ""
    control_mode: str = "position"
    actuator_names: list[str] = field(default_factory=list)
    kp: float | None = None
    kd: float | None = None
    metadata: dict[str, Any] = field(default_factory=dict)
