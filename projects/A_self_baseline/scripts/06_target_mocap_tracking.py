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


A_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A06 TODO skeleton: target / mocap-style tracking.")
    parser.add_argument("--config", default=str(A_ROOT / "configs" / "robot.yaml"))
    parser.add_argument("--model-xml", default=None, help="可选：覆盖 robot.yaml 中的 scene.xml 路径。")
    parser.add_argument("--site", default="attachment_site", help="目标 site。")
    parser.add_argument("--target-mode", default="fixed", choices=["fixed", "mocap"])
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--output-dir", default=str(A_ROOT / "outputs"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    logging.info("A06 当前状态: TODO learning skeleton")
    logging.info("target mode: %s", args.target_mode)
    logging.info("第一版只保留 fixed target TODO，不实现 tracking。")
    logging.info("future log: %s", Path(args.output_dir) / "logs" / "A06_target_tracking.csv")
    logging.info("future report: %s", Path(args.output_dir) / "reports" / "A06_target_tracking_report.md")

    # =============================
    # TODO 1: fixed target 输入
    # =============================
    # - 前置产物: A02/A03 的 site pose/Jacobian 概念, A04/A05 的 IK 入口, 以及 MuJoCo XML。
    # - 要实现什么: 第一版只定义 fixed target 的来源、坐标系和目标 site。
    # - 为什么需要: 先把 viewer target 的概念固定成可复盘输入，再进入实时/mocap 交互。
    # - 对标 mink 的概念: arm_ur5e.py 的 viewer target。
    # - 推荐 API: numpy.ndarray、MuJoCo site pose 查询。
    # - 输入是什么: scene.xml、目标 site、fixed target position/pose。
    # - 输出是什么: target definition 和 tracking log 占位。
    # - 如何验证: 日志能同时说明 target、current site pose 和 error 定义。

    # =============================
    # TODO 2: 后续 mocap-style target 数据流
    # =============================
    # - 要实现什么: 后续把目标写入 data.mocap_pos / data.mocap_quat 并在 viewer 中显示。
    # - 为什么需要: mink 示例使用交互 target，A 项目需要先理解 MuJoCo mocap target 数据流。
    # - 对标 mink 的概念: arm_ur5e.py 的 mocap target / viewer target。
    # - 推荐 API: viewer、data.mocap_pos、data.mocap_quat、mujoco.mj_step。
    # - 输入是什么: fixed target 或后续用户交互/mocap target。
    # - 输出是什么: A06_target_tracking.csv 和 Markdown report。
    # - 如何验证: 目标值、当前 site pose 和误差在日志中可解释，仿真无 NaN。
    raise NotImplementedError("TODO: A06 remains a target / mocap-style tracking learning skeleton.")


if __name__ == "__main__":
    main()
