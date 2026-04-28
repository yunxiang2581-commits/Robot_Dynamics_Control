"""A02 pipeline step: compute a target frame pose with forward kinematics.

Pipeline role:
- Step: A02, consumes the model and target frame identified by A01.
- Consumes: URDF path, target frame name, initial or configured joint state q.
- Produces: target frame SE3 pose report and lightweight frame-pose cache.
- Downstream: A03 validates the same frame's Jacobian; A04/A05 use the frame target.
- Output contract: reports/A02_fk_frame_pose.md and cache/A02_frame_pose.json.

This is a TODO learning entry. It intentionally avoids implementing full FK.
Legacy reference: scripts/legacy_imported/fk_h1.py
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compute target frame pose using FK.")
    parser.add_argument("--urdf", default="../../shared/robot_assets/models/your_robot.urdf")
    parser.add_argument("--frame", default="left_foot")
    parser.add_argument("--output-dir", default=str(PROJECT_ROOT / "outputs" / "02_fk_frame_pose"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    logging.info("FK output directory placeholder: %s", args.output_dir)

    # Pipeline TODO(中文):
    # - 前置产物: A01 的模型摘要和候选 frame 清单。
    # - 本步产物: 指定 frame 的位置、姿态和 SE3 摘要。
    # - 后续消费: A03 使用同一个 frame 做 Jacobian 验证; A04/A05 使用该 frame 定义 IK 目标。
    # - 输出路径: 后续应通过 pipeline_io 生成 A02 report/cache 路径。
    # TODO(中文):
    # 1. 要实现什么: 加载模型, 构造 q, 调用正运动学并读取指定 frame 的 SE3 位姿。
    # 2. 求职重要性: FK 是运动学、控制和调试机器人模型的基础, 面试常问 frame pose 如何得到。
    # 3. 推荐 API: pin.forwardKinematics, pin.updateFramePlacements, model.getFrameId, data.oMf。
    # 4. 输入: URDF 路径、目标 frame 名、关节配置 q。
    # 5. 输出: 目标 frame 的位置、旋转矩阵/四元数和保存的文本摘要。
    # 6. 如何验证: 使用中性位姿或已知关节扰动, 检查 frame 位姿变化方向是否合理。
    # 7. legacy 参考: scripts/legacy_imported/fk_h1.py。
    raise NotImplementedError("TODO: implement FK frame pose learning step.")


if __name__ == "__main__":
    main()
