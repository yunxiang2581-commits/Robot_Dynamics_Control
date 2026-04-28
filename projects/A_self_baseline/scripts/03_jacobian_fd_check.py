"""Compute a frame Jacobian and verify it with finite differences.

This is a TODO learning entry. Legacy references:
scripts/legacy_imported/jacobian_h1.py
scripts/legacy_imported/jacobian_h1_check.py
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check frame Jacobian with finite differences.")
    parser.add_argument("--urdf", default="../../shared/robot_assets/models/your_robot.urdf")
    parser.add_argument("--frame", default="left_foot")
    parser.add_argument("--dt", type=float, default=1e-6)
    parser.add_argument("--output-dir", default=str(PROJECT_ROOT / "outputs" / "03_jacobian_fd_check"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    logging.info("Jacobian check output directory placeholder: %s", args.output_dir)

    # TODO(中文):
    # 1. 要实现什么: 计算 frame Jacobian, 再用 q 与 q+dq 的位姿差做有限差分验证。
    # 2. 求职重要性: Jacobian 是速度映射、IK、QP 控制和 WBC 的核心, 会被重点考察。
    # 3. 推荐 API: pin.computeJointJacobians, pin.updateFramePlacements, pin.getFrameJacobian, pin.integrate。
    # 4. 输入: URDF、目标 frame、q、dq、dt。
    # 5. 输出: 解析 Jacobian、有限差分速度、误差范数和可选曲线/文本报告。
    # 6. 如何验证: 误差范数应随 dt 合理减小, 且平移速度方向与扰动一致。
    # 7. legacy 参考: scripts/legacy_imported/jacobian_h1.py 和 jacobian_h1_check.py。
    raise NotImplementedError("TODO: implement Jacobian finite-difference check.")


if __name__ == "__main__":
    main()
