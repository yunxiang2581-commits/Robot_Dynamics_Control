"""A07 MuJoCo Actuator Tracking TODO skeleton.

本脚本是 A07：MuJoCo actuator tracking 的学习骨架。

学习定位：
- A07 消费 A04/A05 产生的 q_traj 或 trajectory_source。
- A07 不重新定义 target。
- A07 不重新求 IK。
- A07 才负责规划 actuator command / data.ctrl / mj_step control loop。
- 当前仍是 TODO skeleton，不实现真实 actuator tracking，不录 video。

输入规划：
- robot.yaml / motion_task.yaml
- A04_dls_ik_q_traj.npy 或 A05_qp_ik_q_traj.npy
- MuJoCo model actuator 信息

未来输出规划：
- outputs/logs/A07_actuator_tracking.csv
- outputs/figures/A07_tracking_error.png
- outputs/reports/A07_actuator_tracking_report.md
- outputs/videos/A07_ur5e_actuator_tracking_demo.mp4

当前状态：
- TODO learning skeleton。
- 不运行 mj_step control loop。
- 不写 data.ctrl。
- 不生成 video。
- 不调用 mink。
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve()
A_ROOT = SCRIPT_PATH.parents[1]


TODO_ITEMS = [
    "TODO 1: 读取 robot.yaml / motion_task.yaml，加载 MuJoCo model 和 data。",
    "TODO 2: 读取 trajectory_source，验证 q_traj shape = (N, nq 或 nu 对应维度)。",
    "TODO 3: inspect actuators，确认 actuator name、nu、ctrlrange、actuator_trnid。",
    "TODO 4: 规划 q_des -> ctrl 的映射；position actuator 可先 ctrl = q_des[actuated_dofs]。",
    "TODO 5: 未来在 A07 中写 data.ctrl，并调用 mujoco.mj_step 执行动力学仿真。",
    "TODO 6: 记录 q_des、q_actual、tracking_error、ctrl、time。",
    "TODO 7: 输出 tracking log、error figure、report；video 后续再做。",
    "TODO 8: 验证 tracking error、ctrlrange violation、是否正确消费 A05 trajectory_source。",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A07 TODO skeleton: MuJoCo actuator tracking.")
    parser.add_argument("--robot-config", default=str(A_ROOT / "configs" / "robot.yaml"))
    parser.add_argument("--motion-task-config", default=str(A_ROOT / "configs" / "motion_task.yaml"))
    parser.add_argument("--trajectory-source", default=str(A_ROOT / "outputs" / "trajectories" / "A05_qp_ik_q_traj.npy"))
    parser.add_argument("--producer-step", choices=["A04", "A05"], default="A05")
    parser.add_argument("--control-mode", choices=["position", "velocity", "torque"], default="position")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--output-dir", default=str(A_ROOT / "outputs"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


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

    logging.info("A07 当前是 actuator tracking TODO learning skeleton。")
    logging.info("输入: trajectory_source=%s, producer_step=%s", args.trajectory_source, args.producer_step)
    logging.info("参数: control_mode=%s, duration=%s, dt=%s", args.control_mode, args.duration, args.dt)
    logging.info("未来输出: %s", planned_outputs)
    for item in TODO_ITEMS:
        logging.info(item)

    raise NotImplementedError("A07 TODO: 未来实现 actuator tracking；当前不写 data.ctrl、不运行 mj_step、不录 video。")


if __name__ == "__main__":
    main()
