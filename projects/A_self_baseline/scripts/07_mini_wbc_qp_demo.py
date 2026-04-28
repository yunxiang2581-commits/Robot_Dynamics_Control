"""A07 pipeline step: sketch a teaching Mini-WBC QP structure.

Pipeline role:
- Step: A07, bridges A project kinematics/QP/PD learning toward WBC concepts.
- Consumes: A03 task Jacobian concept, A05 QP constraint structure, and A06 tracking feedback.
- Produces: WBC variable/task/constraint structure report and QP matrix dimension notes.
- Downstream: B project legged_control WBC/NMPC reading can use this as a concept bridge.
- Output contract: reports/A07_mini_wbc_qp.md and cache/A07_qp_structure.json.

This is only a QP structure skeleton. It does not implement full WBC.
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
    # - 前置产物: A03 的 Jacobian 概念、A05 的 QP 目标/约束结构、A06 的 tracking 误差概念。
    # - 本步产物: Mini-WBC QP 变量定义、任务项、约束项和矩阵维度报告。
    # - 后续消费: B 项目 legged_control 阅读时对照 WBC/NMPC 结构, 不在此处复刻完整框架。
    # - 输出路径: 后续应通过 pipeline_io 生成 A07 report/cache 路径。
    # TODO(中文):
    # 1. 要实现什么: 构造教学版 WBC QP 的变量、目标项和约束项, 但暂不实现完整动力学控制。
    # 2. 求职重要性: WBC 是腿足机器人控制岗位高频能力点, 需要理解任务、约束和 QP 结构。
    # 3. 推荐 API: osqp.OSQP, scipy.sparse, Pinocchio dynamics APIs 后续再逐步引入。
    # 4. 输入: 任务权重、接触约束、关节限制、期望加速度或任务空间目标。
    # 5. 输出: QP 矩阵维度、变量解释、求解状态占位和结构化报告。
    # 6. 如何验证: 先验证矩阵维度、约束数量和单位一致, 再逐步接入动力学。
    # 7. legacy 参考: 当前无 Mini-WBC legacy, 后续可参考 B 项目 legged_control 拆解文档。
    raise NotImplementedError("TODO: implement Mini-WBC QP structure learning step.")


if __name__ == "__main__":
    main()
