"""A07 pipeline 第七步: MuJoCo actuator tracking / mini task-space QP 过渡骨架。

所属 pipeline 步骤:
- A07 MuJoCo actuator tracking。

对标 mink 的概念:
- `arm_ur5e_actuators.py`。
- 把 A04/A05 生成的 `q_des` 或 `dq_des` 送入 MuJoCo actuator / `data.ctrl`。

本脚本输入:
- A04/A05 生成的期望轨迹。
- UR5e MJCF 模型中的 actuator 名称和 control range。
- 可选 mini task-space QP 结构说明。

本脚本输出:
- `outputs/logs/A07_actuator_tracking.csv`。
- `outputs/figures/A07_tracking_error.png`。
- `outputs/videos/A07_ur5e_actuator_tracking_demo.mp4`。
- `outputs/reports/A07_actuator_tracking_report.md`。

当前状态:
- TODO learning skeleton。
- 当前阶段不承诺完整 humanoid WBC。
- 不实现完整 actuator tracking 或 QP 控制。
- 不调用 mink 替代自己的实现。
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path


A_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A07 TODO skeleton: MuJoCo actuator tracking.")
    parser.add_argument("--config", default=str(A_ROOT / "configs" / "mujoco_pd.yaml"))
    parser.add_argument("--model-xml", default=None, help="可选：覆盖 robot.yaml 中的 scene.xml 路径。")
    parser.add_argument("--q-des", default=None, help="TODO：未来 A04/A05 q_des 轨迹路径。")
    parser.add_argument("--dq-des", default=None, help="TODO：未来 A04/A05 dq_des 轨迹路径。")
    parser.add_argument("--output-dir", default=str(A_ROOT / "outputs"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    logging.info("A07 当前状态: TODO learning skeleton")
    logging.info("q_des/dq_des placeholders: %s / %s", args.q_des, args.dq_des)
    logging.info("future log: %s", Path(args.output_dir) / "logs" / "A07_actuator_tracking.csv")
    logging.info("future figure: %s", Path(args.output_dir) / "figures" / "A07_tracking_error.png")
    logging.info("future video: %s", Path(args.output_dir) / "videos" / "A07_ur5e_actuator_tracking_demo.mp4")

    # =============================
    # TODO 1: 读取 q_des / dq_des
    # =============================
    # - 前置产物: A04/A05 的 q_des 或 dq_des, A06 的 target tracking 概念, 以及 actuator 名称。
    # - 要实现什么: 后续读取 A04/A05 产生的期望关节轨迹或速度轨迹。
    # - 为什么需要: actuator tracking 不是重新求 IK，而是把期望量送入 MuJoCo control。
    # - 对标 mink 的概念: arm_ur5e_actuators.py 的 actuator tracking 输入。
    # - 推荐 API: numpy.load、Path.exists、shape 检查。
    # - 输入是什么: q_des 或 dq_des、MuJoCo XML、仿真时长。
    # - 输出是什么: 已校验的期望轨迹和时间索引。
    # - 如何验证: 轨迹维度与 model.nq/model.nv 可解释，长度大于 0。

    # =============================
    # TODO 2: actuator / data.ctrl 数据流
    # =============================
    # - 要实现什么: 后续检查 actuator 名称和 control range，再写 data.ctrl 并调用 mj_step。
    # - 为什么需要: 这是从 kinematic IK 进入 MuJoCo actuator 仿真的接口。
    # - 对标 mink 的概念: arm_ur5e_actuators.py。
    # - 推荐 API: data.ctrl、mujoco.mj_step、actuator names、control range 检查。
    # - 输入是什么: model actuator 列表、q_des/dq_des、当前 q/qvel。
    # - 输出是什么: actuator tracking log、error figure、video 路径说明。
    # - 如何验证: actuator 名称匹配，ctrl 维度等于 nu，tracking error 有界，仿真无 NaN。

    # =============================
    # TODO 3: 记录 A07 展示产物
    # =============================
    # - 要实现什么: 后续输出 A07_actuator_tracking.csv、A07_tracking_error.png 和视频。
    # - 为什么需要: A09 对比报告和 A10 展示 demo 都依赖这些产物。
    # - 推荐 API: csv、matplotlib、后续 video recorder。
    # - 输入是什么: tracking error history、控制输入、仿真状态。
    # - 输出是什么: log、figure、outputs/videos/A07_ur5e_actuator_tracking_demo.mp4。
    # - 如何验证: 日志、图和视频路径都能在报告中追踪。
    raise NotImplementedError("TODO: A07 remains a MuJoCo actuator tracking learning skeleton.")


if __name__ == "__main__":
    main()
