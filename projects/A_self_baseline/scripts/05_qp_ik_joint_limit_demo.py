"""A05 pipeline 第五步: task + limit + QP-IK 学习型 TODO 骨架。

所属 pipeline 步骤:
- A05 task + limit + QP-IK。

对标 mink 的概念:
- `FrameTask`、`PostureTask`、`ConfigurationLimit`、`VelocityLimit`。
- 这是 A 项目最贴近 mink 核心抽象的一步。

本脚本输入:
- A04 的 task/error 定义。
- A03 的 site Jacobian。
- 关节速度限制、位置限制、QP 权重和 damping。

本脚本输出:
- `outputs/trajectories/A05_qp_ik_q_traj.npy`。
- `outputs/logs/A05_qp_ik_constraints.csv`。
- `outputs/reports/A05_qp_ik_report.md`。

当前状态:
- TODO learning skeleton。
- 第一版只做最小 QP-IK 设计说明, 不实现完整 collision avoidance。
- 不调用 mink 替代自己的实现。
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a constrained QP-IK learning placeholder.")
    parser.add_argument("--urdf", default="../../shared/robot_assets/models/your_robot.urdf")
    parser.add_argument("--frame", default="left_foot")
    parser.add_argument("--output-dir", default=str(PROJECT_ROOT / "outputs" / "05_qp_ik_joint_limit_demo"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    logging.info("QP-IK output directory placeholder: %s", args.output_dir)

    # Pipeline TODO(中文):
    # - 前置产物: A04 的 DLS-IK 目标、误差定义和 q 更新基线。
    # - 本步产物: 带 task/limit 的 QP-IK 解、QP 求解状态和约束检查报告。
    # - 后续消费: A06 可消费 A05 轨迹做 target tracking, A07 复用 actuator tracking 所需的 q_des/dq_des。
    # - 输出路径: 后续应通过 pipeline_io 生成 A05 report/trajectory/cache 路径。
    # TODO(中文):
    # 1. 要实现什么: 构造 min ||J dq - v_des||^2 + damping ||dq||^2, 并加入 dq_min <= dq <= dq_max。
    # 2. 为什么这一步存在: 它把 A04 的无约束 IK 推进到带 task 和 limit 的 QP-IK。
    # 3. 对标 mink 的哪个概念: FrameTask、PostureTask、ConfigurationLimit、VelocityLimit。
    # 4. 推荐 API: mujoco.mj_jacSite, scipy.optimize 或 osqp, scipy.sparse。
    # 5. 输入是什么: q、site Jacobian、期望任务速度 v_des、关节/速度上下限、QP 权重。
    # 6. 输出是什么: dq、q 轨迹、约束日志、QP 状态和 Markdown 报告。
    # 7. 如何验证: dq 满足上下限, task error 下降, QP 状态可解释, 约束日志无异常。
    # 8. 暂不做什么: 第一版不实现完整 collision avoidance, 只保留后续 TODO。
    raise NotImplementedError("TODO: implement constrained QP-IK learning step.")


if __name__ == "__main__":
    main()
