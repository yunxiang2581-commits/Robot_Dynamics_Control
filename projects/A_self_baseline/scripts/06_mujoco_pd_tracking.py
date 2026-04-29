"""A06 pipeline 第六步: target / mocap-style tracking 学习型 TODO 骨架。

所属 pipeline 步骤:
- A06 target / mocap-style tracking。

对标 mink 的概念:
- `arm_ur5e.py` 中 viewer target / mocap target 的思想。
- 第一版可以先使用 fixed target, 后续再扩展 mocap-style target。

本脚本输入:
- UR5e MJCF 模型。
- 目标 site 名称。
- fixed target 或后续 mocap target。

本脚本输出:
- `outputs/logs/A06_target_tracking.csv`。
- `outputs/reports/A06_target_tracking_report.md`。

当前状态:
- TODO learning skeleton。
- 不实现完整 target tracking 或 viewer 交互。
- 不调用 mink 替代自己的实现。

References:
- root experiments/exp_pd_joint_control.py
- `scripts/legacy_imported/mujoco_h1_left_knee_perturb.py`
- `scripts/legacy_imported/mujoco_h1_sim_learning.py`
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a MuJoCo PD tracking placeholder.")
    parser.add_argument("--model-xml", default="../../shared/robot_assets/models/your_robot.xml")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--output-dir", default=str(PROJECT_ROOT / "outputs" / "06_mujoco_pd_tracking"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    logging.info("MuJoCo PD output directory placeholder: %s", args.output_dir)

    # Pipeline TODO(中文):
    # - 前置产物: A02/A03 的 site pose/Jacobian 概念, A04/A05 的 IK 入口, 以及 MuJoCo XML。
    # - 本步产物: fixed target 或 mocap-style target tracking 日志和报告。
    # - 后续消费: A07 使用 target tracking 产生的期望 q_des/dq_des 进入 actuator tracking。
    # - 输出路径: 后续应通过 pipeline_io 生成 A06 report/log/figure/video 路径。
    # TODO(中文):
    # 1. 要实现什么: 加载 MuJoCo XML, 设置 fixed target, 后续扩展 mocap target, 并记录目标跟踪误差。
    # 2. 为什么这一步存在: mink UR5e 示例的交互目标需要先被转成可复盘的 target tracking 流程。
    # 3. 对标 mink 的哪个概念: viewer target / mocap target。
    # 4. 推荐 API: mujoco viewer, data.mocap_pos, data.mocap_quat, mujoco.mj_step。
    # 5. 输入是什么: MuJoCo XML、目标 site、fixed target 或 mocap target、仿真时长。
    # 6. 输出是什么: A06 target tracking CSV 日志和 Markdown 报告。
    # 7. 如何验证: 目标值、当前 site pose 和误差在日志中可解释, 仿真无 NaN。
    # 8. legacy 参考: root PD 实验和 scripts/legacy_imported/mujoco_*.py。
    raise NotImplementedError("TODO: implement MuJoCo PD tracking learning step.")


if __name__ == "__main__":
    main()
