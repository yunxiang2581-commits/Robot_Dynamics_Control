"""A05 pipeline step: formulate QP-IK with joint velocity and position limits.

Pipeline role:
- Step: A05, upgrades A04 DLS-IK into a constrained QP-IK formulation.
- Consumes: A04 task definition, A03 Jacobian logic, joint position/velocity limits.
- Produces: QP status, constrained dq or q trajectory, constraint violation summary.
- Downstream: A06 can track the constrained IK trajectory; A07 reuses the QP structure.
- Output contract: reports/A05_qp_ik_joint_limit.md, trajectories/A05_qp_ik_q_traj.csv,
  and cache/A05_qp_status.json.

This is a TODO learning entry. No complete legacy implementation is copied.
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
    # - 本步产物: 带关节限制的 QP-IK 解、QP 求解状态和约束检查报告。
    # - 后续消费: A06 可消费 A05 轨迹做更安全的 PD tracking, A07 复用 QP 目标/约束组织方式。
    # - 输出路径: 后续应通过 pipeline_io 生成 A05 report/trajectory/cache 路径。
    # TODO(中文):
    # 1. 要实现什么: 构造 min ||J dq - v_task||^2 + 正则项, 并加入关节速度/位置约束。
    # 2. 求职重要性: QP-IK 是从 DLS IK 走向约束控制和 WBC 的关键桥梁。
    # 3. 推荐 API: pin.getFrameJacobian, osqp.OSQP, scipy.sparse.csc_matrix, pin.integrate。
    # 4. 输入: q、目标任务速度、关节上下限、速度上下限、QP 权重。
    # 5. 输出: dq、QP 状态、约束违反量、误差曲线和文本报告。
    # 6. 如何验证: 检查 dq 满足上下限, task error 下降, OSQP 状态为 solved。
    # 7. legacy 参考: 当前没有完整 legacy QP-IK, 后续可参考 DLS IK 和 Jacobian 脚本。
    raise NotImplementedError("TODO: implement constrained QP-IK learning step.")


if __name__ == "__main__":
    main()
