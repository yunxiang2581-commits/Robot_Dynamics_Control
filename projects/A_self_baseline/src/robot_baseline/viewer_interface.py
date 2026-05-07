"""Viewer interface TODO skeleton.

viewer target:
在 viewer 中可视化或交互移动的任务目标。

mocap-style target:
用 MuJoCo mocap body 表示可移动目标，其 pose 来自 data.mocap_pos /
data.mocap_quat。

target pose feeds IK:
target pose 是 IK 的任务输入，不是 actuator control 输出。

本模块只规划 viewer / mocap target 数据流，不创建 MJCF，不启动真实 viewer，
不调用 ``mujoco.viewer.launch_passive``，不写 ``data.ctrl``。
"""

from __future__ import annotations

from typing import Any

from robot_baseline.motion_types import ViewerTargetSpec


def plan_derived_mjcf(scene_xml: str, output_xml: str, spec: ViewerTargetSpec) -> dict[str, Any]:
    """规划派生 MJCF。

    TODO:
    - 要做什么：未来创建 ``scene_a06_mocap_target.xml``，不修改原始 scene.xml。
    - 为什么：mocap body 必须作为 world child、无 joints，才能由 mocap data 驱动。
    - 对标 mink：viewer 中可拖动的 target。
    - 推荐 API：xml.etree.ElementTree。
    - 输入：原始 scene.xml、派生输出路径、ViewerTargetSpec。
    - 输出：派生 MJCF 规划 dict。
    - 验证：加载派生 MJCF 后 ``model.nmocap > 0``。
    """
    return {"scene_xml": scene_xml, "output_xml": output_xml, "mocap_body_name": spec.mocap_body_name, "status": "todo"}


def check_mocap_capability(model_n_mocap: int, mocap_pos_shape: tuple[int, ...], mocap_quat_shape: tuple[int, ...]) -> dict[str, Any]:
    """检查 mocap capability 的 shape。

    学习要点：
    - ``model.nmocap`` 表示 mocap body 数量。
    - ``data.mocap_pos.shape == (model.nmocap, 3)``。
    - ``data.mocap_quat.shape == (model.nmocap, 4)``。
    - ``model.nmocap == 0`` 时不能访问 ``data.mocap_pos[0]``。
    """
    ok = model_n_mocap > 0 and mocap_pos_shape == (model_n_mocap, 3) and mocap_quat_shape == (model_n_mocap, 4)
    return {"model_n_mocap": model_n_mocap, "mocap_pos_shape": mocap_pos_shape, "mocap_quat_shape": mocap_quat_shape, "ok": ok}


def plan_mouse_drag_target() -> None:
    """规划 mouse drag target。

    TODO:
    - 未来依赖 MuJoCo viewer 对 mocap body 的 perturbation。
    - 每帧调用 ``viewer.sync()`` 后读取 ``data.mocap_pos`` / ``data.mocap_quat``。
    - 不自己实现底层 mouse ray casting。
    - 验证：鼠标拖动后 target pose 变化。
    """
    raise NotImplementedError("TODO R6: 验证 mouse drag target；Step R-C 不启动 viewer。")


def plan_keyboard_target_movement() -> None:
    """规划 keyboard target movement。

    TODO:
    - 未来用 ``key_callback`` 记录 W/S、A/D、Q/E、R、SPACE。
    - 主循环消费 pending_delta。
    - 修改 ``data.mocap_pos`` 时使用 ``viewer.lock()``。
    - 调用 ``viewer.sync()`` 同步 GUI 与 data。
    - 验证：按键后 target pose 变化。
    """
    raise NotImplementedError("TODO R5: 实现 keyboard target movement；Step R-C 只保留接口。")


def plan_target_pose_feeds_ik() -> None:
    """规划 target pose feeds IK。

    TODO:
    - 从 mocap target 读取 ``p_target`` / ``R_target``。
    - 从当前 site 读取 ``p_current`` / ``R_current``。
    - 计算 ``e_pos = p_target - p_current``。
    - 可选计算 ``e_rot = log(R_target R_current^T)``。
    - 把误差交给 A04/A05 IK interface。
    - 不写 ``data.ctrl``，因为这不是 actuator control。
    """
    raise NotImplementedError("TODO R6/R7: 接通 target pose 到 IK request。")


def plan_kinematic_ik_follow() -> None:
    """规划 kinematic IK follow。

    TODO:
    - 未来调用 IK interface 得到 dq。
    - 只更新 qpos，用于 viewer 中的运动学预览。
    - A06 不写 ``data.ctrl``，因为 actuator tracking 属于 A07。
    - A07 才允许规划 ``data.ctrl`` 和 ``mujoco.mj_step`` 控制闭环。
    """
    raise NotImplementedError("TODO R7: 实现运动学 IK follow；Step R-C 不执行。")
