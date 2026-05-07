"""A05 Task + Limit + QP-IK TODO skeleton.

本脚本是 A05：带任务误差和 box limit 的 QP-IK 学习骨架。

学习定位：
- A05 在 A04 DLS IK 之后，引入 joint velocity / position limit。
- A05 的数学目标是把 IK 写成带约束的最小二乘问题。
- A05 不做 actuator tracking，不写 data.ctrl，不启动 viewer。
- A06 负责 target management。
- A07 才负责 MuJoCo actuator tracking。

输入规划：
- robot.yaml / qp_ik.yaml / motion_task.yaml
- A02_site_pose.json 中的 current site pose
- target definition 或 CLI target offset

未来输出规划：
- outputs/cache/A05_target_definition.json
- outputs/trajectories/A05_qp_ik_q_traj.npy
- outputs/logs/A05_qp_ik_error.csv
- outputs/logs/A05_qp_ik_constraints.csv
- outputs/reports/A05_qp_ik_report.md

当前状态：
- TODO learning skeleton。
- 不运行完整 QP-IK。
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
    "TODO 1: 读取 robot.yaml / qp_ik.yaml / motion_task.yaml，确认模型、权重、dt、limit 参数。",
    "TODO 2: 统一读取 TargetDefinition；若没有 A06 target，就用 current position + offset。",
    "TODO 3: 构造 position task error: e_pos = p_target - p_current。",
    "TODO 4: 计算 J_pos，并形成 QP 目标 ||J_pos dq - gain e_pos||^2 + damping ||dq||^2。",
    "TODO 5: 构造 velocity limit: -v_max <= dq <= v_max。",
    "TODO 6: 构造 position limit 近似约束，避免一步积分越界。",
    "TODO 7: 调用 scipy.optimize.minimize 或后续 osqp backend 求 dq。",
    "TODO 8: 积分 q_next，并记录 position error、constraint violation、q trajectory。",
    "TODO 9: 输出 A05_target_definition.json、q_traj、error log、constraint log、report。",
    "TODO 10: 验证 q_traj shape、final error、max constraint violation；不进入 actuator / viewer。",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A05 TODO skeleton: task + limit + QP-IK.")
    parser.add_argument("--robot-config", default=str(A_ROOT / "configs" / "robot.yaml"))
    parser.add_argument("--qp-ik-config", default=str(A_ROOT / "configs" / "qp_ik.yaml"))
    parser.add_argument("--motion-task-config", default=str(A_ROOT / "configs" / "motion_task.yaml"))
    parser.add_argument("--a02-pose", default=str(A_ROOT / "outputs" / "cache" / "A02_site_pose.json"))
    parser.add_argument("--target-source", choices=["offset_from_current", "target_definition", "fixed_pose"], default="offset_from_current")
    parser.add_argument("--target-definition", default=str(A_ROOT / "outputs" / "cache" / "A06_target_definition.json"))
    parser.add_argument("--target-position", default=None)
    parser.add_argument("--target-position-offset", default="0.03,0.00,0.00")
    parser.add_argument("--target-orientation-mode", choices=["keep_current", "fixed_rpy", "fixed_quat"], default="keep_current")
    parser.add_argument("--solver-type", choices=["qp_scipy", "qp_osqp"], default="qp_scipy")
    parser.add_argument("--site", default="attachment_site")
    parser.add_argument("--body", default="wrist_3_link")
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--gain", type=float, default=1.0)
    parser.add_argument("--damping", type=float, default=1.0e-4)
    parser.add_argument("--velocity-limit", type=float, default=0.5)
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
        "target_definition": output_root / "cache" / "A05_target_definition.json",
        "trajectory": output_root / "trajectories" / "A05_qp_ik_q_traj.npy",
        "error_log": output_root / "logs" / "A05_qp_ik_error.csv",
        "constraint_log": output_root / "logs" / "A05_qp_ik_constraints.csv",
        "report": output_root / "reports" / "A05_qp_ik_report.md",
    }

    logging.info("A05 当前是 QP-IK TODO learning skeleton。")
    logging.info("输入: robot_config=%s, qp_ik_config=%s, target_source=%s", args.robot_config, args.qp_ik_config, args.target_source)
    logging.info("参数: solver_type=%s, dt=%s, velocity_limit=%s", args.solver_type, args.dt, args.velocity_limit)
    logging.info("未来输出: %s", planned_outputs)
    for item in TODO_ITEMS:
        logging.info(item)

    raise NotImplementedError("A05 TODO: 实现 box-constrained QP-IK；当前只保留学习骨架。")


if __name__ == "__main__":
    main()
