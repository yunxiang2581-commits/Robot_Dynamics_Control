"""A08 collision avoidance TODO learning skeleton.

当前 pipeline 定位：
- A08 collision avoidance TODO。
- 对标 mink collision avoidance constraint，只做概念、输入输出和未来实现 TODO。

为什么放在 QP-IK 之后：
- collision avoidance 本质上是额外不等式约束。
- 学习顺序应先理解 A05 的 task + limit + QP-IK，再把避障作为新约束加入。

明确不做：
- 不实现 collision avoidance。
- 不调用 mink 替代自己的实现。
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path


A_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A08 TODO skeleton: collision avoidance concept.")
    parser.add_argument("--config", default=str(A_ROOT / "configs" / "qp_ik.yaml"))
    parser.add_argument("--output-dir", default=str(A_ROOT / "outputs"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    logging.info("A08 当前状态: TODO learning skeleton")
    logging.info("future report: %s", Path(args.output_dir) / "reports" / "A08_collision_avoidance_todo.md")

    # =============================
    # TODO 1: 定义 collision avoidance 的输入输出
    # =============================
    # - 要实现什么: 后续列出几何对象、最小距离、safety margin 和相关 Jacobian。
    # - 为什么需要: 避障约束需要建立在已理解的 QP-IK 变量 dq 之上。
    # - 对标 mink 的概念: collision avoidance constraint。
    # - 推荐 API: MuJoCo geom/contact 查询、距离计算、QP inequality 形式。
    # - 输入是什么: model/data、geom pairs、q、safety margin。
    # - 输出是什么: 未来 inequality constraint 设计说明。
    # - 如何验证: 文档能说明每个约束的物理含义和符号方向。

    # =============================
    # TODO 2: 说明后续如何接入 A05 QP-IK
    # =============================
    # - 要实现什么: 后续把 collision avoidance 写成额外不等式，不改 A04 DLS 主线。
    # - 为什么需要: A08 是 A05 的扩展，不应该提前污染最小 IK 教学路径。
    # - 推荐 API: 与 A05 相同的 QP solver 接口。
    # - 输入是什么: A05 的 dq 变量、limit constraints、collision constraints。
    # - 输出是什么: A08 collision avoidance TODO report。
    # - 如何验证: 报告明确说明“先 QP-IK，后避障”的学习原因。
    raise NotImplementedError("TODO: A08 remains a collision avoidance concept skeleton.")


if __name__ == "__main__":
    main()
