"""Inspect a URDF model and print basic Pinocchio model information.

This is a TODO learning entry. It intentionally does not implement the full
inspection logic yet. Legacy reference:
scripts/legacy_imported/task1_inspect_humanoid_model.py
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Inspect URDF joints, frames, nq, and nv.")
    parser.add_argument("--urdf", default="../../shared/robot_assets/models/your_robot.urdf")
    parser.add_argument("--output-dir", default=str(PROJECT_ROOT / "outputs" / "01_inspect_urdf"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    output_dir = Path(args.output_dir)
    logging.info("Output directory placeholder: %s", output_dir)

    # TODO(中文):
    # 1. 要实现什么: 使用 Pinocchio 加载 URDF, 输出 nq、nv、关节名、frame 名和模型摘要。
    # 2. 求职重要性: 面试中常要求说明机器人模型自由度、浮动基和固定基区别, 这是 FK/Jacobian/IK 的入口。
    # 3. 推荐 API: pin.buildModelFromUrdf, pin.JointModelFreeFlyer, model.names, model.frames, pin.neutral。
    # 4. 输入: --urdf 指向机器人 URDF, 可选 package_dirs 用于 mesh/package 查找。
    # 5. 输出: outputs/01_inspect_urdf/model_summary.txt 中保存模型摘要。
    # 6. 如何验证: 检查终端和文本文件中 nq、nv、joint/frame 数量与预期一致。
    # 7. legacy 参考: scripts/legacy_imported/task1_inspect_humanoid_model.py。
    raise NotImplementedError("TODO: implement URDF inspection learning step.")


if __name__ == "__main__":
    main()
