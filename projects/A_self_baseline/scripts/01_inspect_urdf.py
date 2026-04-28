"""A01 pipeline step: inspect a URDF model with Pinocchio.

Pipeline role:
- Step: A01, the first component of the A self-baseline pipeline.
- Consumes: robot URDF path, optional package/mesh search paths, floating-base choice.
- Produces: model summary, joint/frame list, candidate target frames for A02-A07.
- Downstream: A02 uses selected frame names; A03-A05 use nq/nv and joint/frame metadata.
- Output contract: reports/A01_inspect_urdf.md and cache/A01_model_summary.json.

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

    # Pipeline TODO(中文):
    # - 前置产物: 无, A01 是 pipeline 起点。
    # - 本步产物: 模型摘要 JSON、joint/frame 清单 Markdown、候选 feet/pelvis frame。
    # - 后续消费: A02 读取 frame 名, A03 复用模型维度, A04/A05 复用目标 frame 约定。
    # - 输出路径: 后续应通过 robot_baseline.pipeline_io.build_step_output_paths("A01", A_ROOT) 统一生成。
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
