"""A10 demo showcase / video recording TODO learning skeleton.

当前 pipeline 定位：
- A10 demo showcase / video recording。
- 对标最终展示 demo，只规划输入、输出和展示文档。

本脚本输入：
- q_traj、model xml、tracking log。

本脚本输出：
- `outputs/reports/A10_demo_showcase.md`。
- 最终主 demo 路径说明：`outputs/videos/A07_ur5e_actuator_tracking_demo.mp4`。

明确不做：
- 不实现完整视频录制。
- 不实现 MuJoCo 控制或 tracking。
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path


A_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A10 TODO skeleton: demo showcase / video.")
    parser.add_argument("--q-traj", default=None, help="TODO：未来 A04/A05/A07 轨迹路径。")
    parser.add_argument("--model-xml", default=None, help="TODO：未来 MuJoCo model xml 路径。")
    parser.add_argument("--tracking-log", default=None, help="TODO：未来 A07 tracking log 路径。")
    parser.add_argument("--output-dir", default=str(A_ROOT / "outputs"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    logging.info("A10 当前状态: TODO learning skeleton")
    logging.info("q_traj/model_xml/tracking_log placeholders: %s / %s / %s", args.q_traj, args.model_xml, args.tracking_log)
    logging.info("future showcase: %s", Path(args.output_dir) / "reports" / "A10_demo_showcase.md")
    logging.info("final main demo video: %s", Path(args.output_dir) / "videos" / "A07_ur5e_actuator_tracking_demo.mp4")

    # =============================
    # TODO 1: 汇总最终 demo 输入
    # =============================
    # - 要实现什么: 后续检查 q_traj、model xml、tracking log 是否存在且来自 A07。
    # - 为什么需要: 展示 demo 必须能回溯到 A pipeline 的真实产物。
    # - 对标 mink 的概念: UR5e example 的最终可视化展示。
    # - 推荐 API: pathlib.Path.exists、shape/log header 检查。
    # - 输入是什么: q_traj、model xml、tracking log。
    # - 输出是什么: demo input summary。
    # - 如何验证: 缺失文件会在 Markdown 中明确列出，而不是静默失败。

    # =============================
    # TODO 2: 规划视频和 showcase 文档
    # =============================
    # - 要实现什么: 后续写 demo_showcase.md，说明视频路径、生成命令和关键误差指标。
    # - 为什么需要: A10 是展示入口，不应该重新实现控制算法。
    # - 推荐 API: Path.write_text、Markdown 列表、后续 video recorder。
    # - 输入是什么: A07 video path、tracking error summary、model xml。
    # - 输出是什么: A10_demo_showcase.md 和 mp4/gif 路径说明。
    # - 如何验证: 文档明确最终主 demo 为 outputs/videos/A07_ur5e_actuator_tracking_demo.mp4。
    raise NotImplementedError("TODO: A10 remains a demo showcase / video recording skeleton.")


if __name__ == "__main__":
    main()
