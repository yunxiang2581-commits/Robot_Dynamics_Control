"""A09 comparison report TODO learning skeleton.

当前 pipeline 定位：
- A09 comparison report。
- 对标 mink UR5e examples，对比“自己实现”和“mink 抽象”的边界。

本脚本输入：
- A01-A08 的 reports / logs / trajectories。

本脚本输出：
- `outputs/reports/A09_mink_comparison_report.md`。

明确不做：
- 不实现算法。
- 不调用 mink 替代自己的实现。
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path


A_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A09 TODO skeleton: comparison report.")
    parser.add_argument("--output-dir", default=str(A_ROOT / "outputs"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    logging.info("A09 当前状态: TODO learning skeleton")
    logging.info("future report: %s", Path(args.output_dir) / "reports" / "A09_mink_comparison_report.md")

    # =============================
    # TODO 1: 汇总 A01-A08 产物
    # =============================
    # - 要实现什么: 后续读取 reports/logs/trajectories 的路径和存在状态。
    # - 为什么需要: 对比报告必须基于自己 pipeline 的可复盘产物。
    # - 对标 mink 的概念: UR5e examples 的完整 workflow。
    # - 推荐 API: pathlib.Path.exists、Markdown 表格。
    # - 输入是什么: A01-A08 reports、logs、trajectories。
    # - 输出是什么: artifact summary table。
    # - 如何验证: 每个 A 步骤都有对应输入或明确缺失说明。

    # =============================
    # TODO 2: 比较自己实现和 mink 抽象
    # =============================
    # - 要实现什么: 后续按 model/configuration/Jacobian/IK/QP/target/actuator/avoidance 分栏比较。
    # - 为什么需要: 学习目标是理解 mink 抽象背后的数据流，而不是直接调用 mink。
    # - 对标 mink 的概念: Configuration、FrameTask、limits、solve_ik、actuator examples。
    # - 推荐 API: Markdown 表格和短文本总结。
    # - 输入是什么: A01-A08 产物和 reference_mink_ur5e.md。
    # - 输出是什么: A09_mink_comparison_report.md。
    # - 如何验证: 报告能说明哪些是自己实现、哪些只是 TODO、哪些对应 mink 抽象。
    raise NotImplementedError("TODO: A09 remains a comparison report learning skeleton.")


if __name__ == "__main__":
    main()
