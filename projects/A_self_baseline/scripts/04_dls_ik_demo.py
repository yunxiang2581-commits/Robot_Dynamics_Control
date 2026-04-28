"""A04 pipeline step: run a Damped Least Squares IK learning loop.

Pipeline role:
- Step: A04, converts a target frame position error into an iterative joint update.
- Consumes: A01 model metadata, A02 target frame, A03 validated Jacobian logic.
- Produces: DLS-IK error curve, final q, and a lightweight q trajectory.
- Downstream: A05 compares constrained QP-IK against this baseline; A06 can track the trajectory.
- Output contract: reports/A04_dls_ik.md, trajectories/A04_dls_ik_q_traj.csv,
  and figures/A04_dls_ik_error.png.

This is a TODO learning entry. Legacy references:
scripts/legacy_imported/ik_h1_left_foot_step*.py
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a DLS IK learning demo placeholder.")
    parser.add_argument("--urdf", default="../../shared/robot_assets/models/your_robot.urdf")
    parser.add_argument("--frame", default="left_foot")
    parser.add_argument("--max-iter", type=int, default=50)
    parser.add_argument("--output-dir", default=str(PROJECT_ROOT / "outputs" / "04_dls_ik_demo"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    logging.info("DLS IK output directory placeholder: %s", args.output_dir)

    # Pipeline TODO(中文):
    # - 前置产物: A03 验证过的 frame Jacobian 和 A02 的目标 frame 位姿定义。
    # - 本步产物: DLS-IK 误差历史、q 轨迹、最终 frame 位姿报告。
    # - 后续消费: A05 使用相同任务定义构造 QP-IK, A06 可读取 q 轨迹做 MuJoCo PD tracking。
    # - 输出路径: 后续应通过 pipeline_io 生成 A04 report/trajectory/figure 路径。
    # TODO(中文):
    # 1. 要实现什么: 使用 Damped Least Squares 根据位置误差和 Jacobian 计算 dq, 迭代更新 q。
    # 2. 求职重要性: IK 是把任务空间目标转成关节运动的基础, 能展示运动学和数值优化能力。
    # 3. 推荐 API: pin.forwardKinematics, pin.getFrameJacobian, pin.integrate, numpy.linalg.solve。
    # 4. 输入: URDF、目标 frame、初始 q、目标位置、阻尼系数、步长、最大迭代次数。
    # 5. 输出: q 轨迹、误差曲线、最终 frame 位置和文本报告。
    # 6. 如何验证: 误差应下降, 最终误差低于阈值, q 更新没有越界或 NaN。
    # 7. legacy 参考: scripts/legacy_imported/ik_h1_left_foot_step*.py。
    raise NotImplementedError("TODO: implement DLS IK learning loop.")


if __name__ == "__main__":
    main()
