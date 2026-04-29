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


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build a Mini-WBC QP placeholder.")
    parser.add_argument("--config", default=str(PROJECT_ROOT / "configs" / "mini_wbc.yaml"))
    parser.add_argument("--output-dir", default=str(PROJECT_ROOT / "outputs" / "07_mini_wbc_qp_demo"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    logging.info("Mini-WBC output directory placeholder: %s", args.output_dir)

    # Pipeline TODO(中文):
    # - 前置产物: A04/A05 的 q_des 或 dq_des, A06 的 target tracking 概念, 以及 actuator 名称。
    # - 本步产物: actuator tracking 日志、误差图、可选视频和报告。
    # - 后续消费: A09 用它和 mink arm_ur5e_actuators.py 做对照; B 项目 WBC 阅读只作为远期概念桥梁。
    # - 输出路径: 后续应通过 pipeline_io 生成 A07 report/cache 路径。
    # TODO(中文):
    # 1. 要实现什么: 读取 A04/A05 的期望轨迹, 映射到 MuJoCo actuator/control, 并记录 tracking error。
    # 2. 为什么这一步存在: mink UR5e actuator 示例展示了 IK 结果如何进入仿真控制输入。
    # 3. 对标 mink 的哪个概念: arm_ur5e_actuators.py 中的 actuator tracking。
    # 4. 推荐 API: data.ctrl, mujoco.mj_step, actuator names, control range 检查。
    # 5. 输入是什么: q_des 或 dq_des、MuJoCo XML、actuator 名称、仿真时长。
    # 6. 输出是什么: A07 actuator tracking CSV、误差图、可选视频和 Markdown 报告。
    # 7. 如何验证: actuator 名称匹配, ctrl 维度等于 nu, tracking error 有界, 仿真无 NaN。
    # 8. 暂不做什么: 当前不承诺完整 humanoid WBC, 只保留 mini task-space QP 作为远期过渡说明。
    raise NotImplementedError("TODO: implement Mini-WBC QP structure learning step.")


if __name__ == "__main__":
    main()
