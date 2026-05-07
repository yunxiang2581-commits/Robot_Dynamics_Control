"""A04 DLS Differential IK TODO skeleton.

本脚本是 A04：阻尼最小二乘（DLS）微分 IK 的学习骨架。

学习定位：
- A04 负责学习“给定 target pose，如何用 Jacobian 计算关节速度 dq”。
- A04 是无约束 IK 对照组，不处理 joint limit，不进入 QP。
- A05 才学习 box-constrained QP-IK。
- A06 负责管理 target 来源。
- A07 才负责 MuJoCo actuator tracking。

输入规划：
- robot.yaml / motion_task.yaml
- A02_site_pose.json 中的 current site pose
- target position 或 position offset

未来输出规划：
- outputs/trajectories/A04_dls_ik_q_traj.npy
- outputs/logs/A04_dls_ik_error.csv
- outputs/reports/A04_dls_ik_report.md

当前状态：
- TODO learning skeleton。
- 不运行完整 IK。
- 不生成 outputs。
- 不调用 mink。
- 不写 data.ctrl。
- 不启动 viewer。
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve()
A_ROOT = SCRIPT_PATH.parents[1]


TODO_ITEMS = [
    "TODO 1: 读取 robot.yaml / motion_task.yaml，确认模型路径、site 名称、dt、gain、damping。",
    "TODO 2: 读取 A02_site_pose.json，获得 current site position 作为 IK 起点检查。",
    "TODO 3: 定义 target pose，第一版只做 position target，orientation 保留 keep_current。",
    "TODO 4: 加载 MuJoCo 模型和 data，读取 q_initial、site_id、nq、nv。",
    "TODO 5: 每步计算 e_pos = p_target - p_current。",
    "TODO 6: 计算 site position Jacobian J_pos。",
    "TODO 7: 实现 DLS dq = J^T (J J^T + lambda^2 I)^-1 gain e_pos。",
    "TODO 8: 用 mujoco.mj_integratePos 或固定基 qpos += dq * dt 更新 q。",
    "TODO 9: 记录 error history、q trajectory 和 report。",
    "TODO 10: 验证 final position error、q_traj shape，不进入 QP / control / viewer。",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A04 TODO skeleton: DLS differential IK.")
    parser.add_argument("--robot-config", default=str(A_ROOT / "configs" / "robot.yaml"))
    parser.add_argument("--motion-task-config", default=str(A_ROOT / "configs" / "motion_task.yaml"))
    parser.add_argument("--a02-pose", default=str(A_ROOT / "outputs" / "cache" / "A02_site_pose.json"))
    parser.add_argument("--site", default="attachment_site")
    parser.add_argument("--body", default="wrist_3_link")
    parser.add_argument("--target-position-offset", default="0.03,0.00,0.00")
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--gain", type=float, default=1.0)
    parser.add_argument("--damping", type=float, default=1.0e-3)
    parser.add_argument("--max-iters", type=int, default=100)
    parser.add_argument("--tolerance", type=float, default=1.0e-3)
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
        "trajectory": output_root / "trajectories" / "A04_dls_ik_q_traj.npy",
        "error_log": output_root / "logs" / "A04_dls_ik_error.csv",
        "report": output_root / "reports" / "A04_dls_ik_report.md",
    }

    logging.info("A04 当前是 DLS IK TODO learning skeleton。")
    logging.info("输入: robot_config=%s, a02_pose=%s, site=%s", args.robot_config, args.a02_pose, args.site)
    logging.info("参数: dt=%s, gain=%s, damping=%s", args.dt, args.gain, args.damping)
    logging.info("未来输出: %s", planned_outputs)
    for item in TODO_ITEMS:
        logging.info(item)

    raise NotImplementedError("A04 TODO: 实现 DLS differential IK；当前只保留学习骨架。")


if __name__ == "__main__":
    main()
