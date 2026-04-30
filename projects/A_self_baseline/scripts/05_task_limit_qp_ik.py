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


A_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A05 TODO skeleton: task + limit + QP-IK.")
    parser.add_argument("--config", default=str(A_ROOT / "configs" / "qp_ik.yaml"))
    parser.add_argument("--mjcf", default=None, help="可选：覆盖 robot.yaml 中的 scene.xml 路径。")
    parser.add_argument("--site", default="attachment_site", help="A03/A04 使用的目标 site。")
    parser.add_argument("--task-source", default="A04", help="TODO：未来 task/error 来源。")
    parser.add_argument("--output-dir", default=str(A_ROOT / "outputs"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    logging.info("A05 当前状态: TODO learning skeleton")
    logging.info("site/task source: %s / %s", args.site, args.task_source)
    logging.info("future trajectory: %s", Path(args.output_dir) / "trajectories" / "A05_qp_ik_q_traj.npy")
    logging.info("future constraint log: %s", Path(args.output_dir) / "logs" / "A05_qp_ik_constraints.csv")

    # =============================
    # TODO 1: 定义 task 和 v_des
    # =============================
    # - 前置产物: A04 的 DLS-IK 目标、误差定义和 q 更新基线。
    # - 要实现什么: 后续把 A04 的误差转换为期望任务速度 v_des。
    # - 为什么需要: QP-IK 的目标函数来自 task，而不是直接调用 mink 抽象。
    # - 对标 mink 的概念: FrameTask、PostureTask。
    # - 推荐 API: numpy.ndarray、mujoco.mj_jacSite。
    # - 输入是什么: q、target pose、current pose、J。
    # - 输出是什么: v_des 和 task 权重。
    # - 如何验证: v_des 维度与 J dq 的任务空间维度一致。

    # =============================
    # TODO 2: 保留最小 QP 形式，不实现 QP
    # =============================
    # - 要实现什么: 后续把 task、limit 和 damping 写成最小 QP。
    # - 为什么需要: 这是从 A04 无约束 DLS 进入 mink-style task + limit 的关键桥梁。
    # - 对标 mink 的概念: ConfigurationLimit、VelocityLimit。
    # - 推荐 API: scipy.optimize 或 osqp、scipy.sparse。
    # - 输入是什么: J、v_des、damping、dq_min、dq_max、QP 权重。
    # - 输出是什么: dq、QP solver status。
    # - 如何验证: dq 满足上下限，task error 有下降趋势，solver 状态可解释。
    # - 必须保留的最小 QP 形式:
    #   minimize ||J dq - v_des||² + damping ||dq||²
    #   subject to dq_min <= dq <= dq_max

    # =============================
    # TODO 3: 记录 QP-IK 轨迹和 constraint log
    # =============================
    # - 要实现什么: 后续写 q_traj、constraint log 和 A05 report。
    # - 为什么需要: A07 actuator tracking 需要 q_des 或 dq_des，A09 需要比较日志。
    # - 推荐 API: numpy.save、csv、Path.write_text。
    # - 输入是什么: 每步 q/dq、上下限、solver status、task error。
    # - 输出是什么: A05_qp_ik_q_traj.npy、A05_qp_ik_constraints.csv、报告。
    # - 如何验证: 约束日志中没有越界项，字段能解释每个限制。
    raise NotImplementedError("TODO: A05 remains a task + limit + QP-IK learning skeleton.")


if __name__ == "__main__":
    main()
