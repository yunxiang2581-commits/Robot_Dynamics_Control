"""A07 Actuator interface thin CLI wrapper.

A07 归入 Actuator interface：
- 当前只保留函数级 TODO learning skeleton。
- 完整公式、符号表、验证标准和常见错误见
  ``projects/A_self_baseline/docs/A07_Actuator_TODO_full_plan.md``。

A07 是未来唯一规划 ``data.ctrl`` / ``mujoco.mj_step`` 控制闭环的模块。
当前不执行 actuator tracking，不启动 viewer，不录 video。
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
from typing import Any


A_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = A_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from robot_baseline.motion_types import ActuatorTrackingSpec  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A07 wrapper: Actuator interface TODO.")
    parser.add_argument("--motion-task-config", default=str(A_ROOT / "configs" / "motion_task.yaml"))
    parser.add_argument("--trajectory-source", default=None)
    parser.add_argument("--control-mode", choices=["position", "velocity", "torque"], default=None)
    parser.add_argument("--output-dir", default=str(A_ROOT / "outputs"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def build_actuator_request(args: argparse.Namespace) -> ActuatorTrackingSpec:
    """构造 A07 actuator tracking 请求。

    TODO:
    - 要实现什么：只消费 trajectory_source，构造 ActuatorTrackingSpec。
    - 为什么需要：A07 不定义 target，不重新求 IK。
    - 对标 mink：solve_ik 之后的 actuator execution。
    - 输入：CLI args、motion_task.yaml。
    - 输出：ActuatorTrackingSpec。
    - 推荐 API：``trajectory_io.make_trajectory_source``。
    - 验证标准：trajectory_source 路径明确，control_mode 合法。
    """
    return ActuatorTrackingSpec(
        enabled=False,
        trajectory_source=args.trajectory_source or str(A_ROOT / "outputs" / "trajectories" / "A05_qp_ik_q_traj.npy"),
        control_mode=args.control_mode or "position",
        metadata={
            "motion_task_config": args.motion_task_config,
            "note": "A07 wrapper keeps only high-level CLI overrides; actuator gains and loop timing live in motion_task.yaml.",
        },
    )


def load_tracking_trajectory(args: argparse.Namespace) -> Any:
    """读取 q trajectory。

    TODO:
    - 要实现什么：读取 q_traj，验证 shape=(N,nq)、无 NaN。
    - 为什么需要：A07 是 trajectory executor，必须先确认输入轨迹有效。
    - 对标 mink：solve_ik 后的 configuration trajectory。
    - 输入：A04/A05 q_traj 或 A06 trajectory_source。
    - 输出：q_traj。
    - 推荐 API：``numpy.load``、``trajectory_io.load_q_traj``。
    - 验证标准：N>0，shape=(N,nq)，无 NaN。
    """
    raise NotImplementedError("TODO A07: 读取 tracking trajectory；当前不加载 npy。")


def inspect_actuators(args: argparse.Namespace) -> dict[str, Any]:
    """检查 actuator 信息。

    TODO:
    - 要实现什么：读取 model.nu、actuator names、ctrlrange、joint-actuator mapping。
    - 为什么需要：ctrl 维度和物理意义由 actuator 决定。
    - 对标 mink：arm_ur5e_actuators.py 的 actuator 层。
    - 输入：robot.yaml / MuJoCo model。
    - 输出：actuator summary。
    - 推荐 API：``model.nu``、``model.actuator_ctrlrange``、``mujoco.mj_id2name``。
    - 验证标准：actuator 数量与 model.nu 一致。
    """
    raise NotImplementedError("TODO A07: inspect actuators；当前不加载 MuJoCo model。")


def plan_position_actuator_tracking(args: argparse.Namespace) -> dict[str, Any]:
    """规划 position actuator tracking。

    TODO:
    - 要实现什么：规划 ``q_des -> data.ctrl`` 的 position actuator 映射。
    - 为什么需要：position actuator 的 ctrl 不是 torque。
    - 对标 mink：actuator execution 层。
    - 输入：q_des、actuator mapping、ctrlrange。
    - 输出：ctrl plan。
    - 推荐 API：``numpy.clip``。
    - 验证标准：ctrl shape=(model.nu,)，当前不写 ``data.ctrl``。
    """
    raise NotImplementedError("TODO A07: 规划 position actuator tracking；当前不写 data.ctrl。")


def plan_pd_tracking(args: argparse.Namespace) -> dict[str, Any]:
    """规划 PD tracking。

    TODO:
    - 要实现什么：规划 ``u = Kp(q_des-q)+Kd(dq_des-dq)``。
    - 为什么需要：后续 torque actuator 或合适控制模型可能需要外部 PD。
    - 对标 mink：actuator command 层。
    - 输入：q_des、q、dq_des、dq、Kp、Kd。
    - 输出：PD control plan。
    - 推荐 API：``numpy.asarray``。
    - 验证标准：当前不实现、不写 ctrl；说明 position actuator 与 torque actuator 区别。
    """
    raise NotImplementedError("TODO A07: 规划 PD tracking；当前不计算控制量。")


def plan_control_loop(args: argparse.Namespace) -> dict[str, Any]:
    """规划 MuJoCo control loop。

    TODO:
    - 要实现什么：规划 ``data.ctrl`` + ``mujoco.mj_step`` 控制循环。
    - 为什么需要：A07 是未来唯一 actuator executor。
    - 对标 mink：viewer loop + actuator step。
    - 输入：q_traj、model、data、control_mode。
    - 输出：control loop plan。
    - 推荐 API：``mujoco.mj_step``。
    - 验证标准：当前不执行；A06 禁止 ``data.ctrl``。
    """
    raise NotImplementedError("TODO A07: 规划 control loop；当前不执行 mj_step。")


def write_actuator_outputs(result: Any, output_paths: dict[str, Path]) -> None:
    """写 A07 输出。

    TODO:
    - 要实现什么：未来输出 tracking log、figure、report、video placeholder。
    - 为什么需要：actuator tracking 需要误差、ctrl range 和展示路径。
    - 对标 mink：viewer 可见结果在本项目中落成日志/报告。
    - 输入：tracking result 和输出路径。
    - 输出：CSV、PNG、Markdown、video placeholder path。
    - 推荐 API：``csv.DictWriter``、``matplotlib``、``Path.write_text``。
    - 验证标准：当前不生成 video。
    """
    raise NotImplementedError("TODO A07: 写 actuator outputs；当前不生成 outputs。")


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO), format="%(levelname)s: %(message)s")

    output_root = Path(args.output_dir)
    if not output_root.is_absolute():
        output_root = A_ROOT / output_root

    planned_outputs = {
        "tracking_log": output_root / "logs" / "A07_actuator_tracking.csv",
        "tracking_figure": output_root / "figures" / "A07_tracking_error.png",
        "report": output_root / "reports" / "A07_actuator_tracking_report.md",
        "video": output_root / "videos" / "A07_ur5e_actuator_tracking_demo.mp4",
    }
    request = build_actuator_request(args)

    logging.info("A07 Actuator wrapper: function-level TODO skeleton.")
    logging.info("trajectory_source=%s, control_mode=%s", request.trajectory_source, request.control_mode)
    logging.info("planned_outputs=%s", planned_outputs)

    raise NotImplementedError("A07 wrapper TODO: 当前只建立 ActuatorTrackingSpec，不执行控制。")


if __name__ == "__main__":
    main()
