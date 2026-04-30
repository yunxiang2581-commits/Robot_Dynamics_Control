"""A02 pipeline 第二步: configuration / site pose 学习型 TODO 骨架。

所属 pipeline 步骤:
- A02 configuration / site pose。

对标 mink 的概念:
- `mink.Configuration`。
- 从关节配置 `q` 更新 MuJoCo `data`, 再查询 body/site pose。

本脚本输入:
- A01 确认的 MuJoCo MJCF 模型。
- 末端 site/body 名称, 例如 `attachment_site` 或 `tool0`。
- 初始或配置文件给定的关节配置 `q`。

本脚本输出:
- `outputs/reports/A02_site_pose_report.md`。
- `outputs/cache/A02_site_pose.json`。

当前状态:
- TODO learning skeleton。
- 不实现完整 FK/site pose 逻辑。
- 不调用 mink 替代自己的实现。

Legacy reference:
- `scripts/legacy_imported/fk_h1.py` 仅作为历史学习参考。
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path


A_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A02 TODO skeleton: configuration / site pose.")
    parser.add_argument("--config", default=str(A_ROOT / "configs" / "robot.yaml"))
    parser.add_argument("--mjcf", default=None, help="可选：覆盖 robot.yaml 中的 scene.xml 路径。")
    parser.add_argument("--site", default="attachment_site", help="A01 确认后的目标 site 名称。")
    parser.add_argument("--body", default=None, help="可选：A01 确认后的目标 body 名称。")
    parser.add_argument("--q-source", default="neutral", help="TODO：未来 q 的来源，例如 neutral/keyframe/file。")
    parser.add_argument("--output-dir", default=str(A_ROOT / "outputs"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    logging.info("A02 当前状态: TODO learning skeleton")
    logging.info("config: %s", args.config)
    logging.info("mjcf override: %s", args.mjcf)
    logging.info("target site/body: %s / %s", args.site, args.body)
    logging.info("future report: %s", Path(args.output_dir) / "reports" / "A02_site_pose_report.md")
    logging.info("future cache: %s", Path(args.output_dir) / "cache" / "A02_site_pose.json")

    # =============================
    # TODO 1: 确认 A02 输入
    # =============================
    # - 前置产物: A01 的模型摘要和候选 site/body 清单。
    # - 要实现什么: 读取 config/MJCF、目标 site/body 名称和 q 来源。
    # - 为什么需要: A02 是从模型对象进入位姿查询的第一步，后续 A03-A05 都复用同一目标。
    # - 推荐 API: pathlib.Path、A01 生成的 cache、MuJoCo name lookup。
    # - 输入是什么: configs/robot.yaml、scene.xml、target site/body、q。
    # - 输出是什么: 已确认的模型路径、目标名称和 q 来源。
    # - 如何验证: 目标名称来自 A01 摘要，不靠猜测写死。

    # =============================
    # TODO 2: q -> MuJoCo data -> site pose 数据流
    # =============================
    # - 要实现什么: 创建 MjData，把 q 写入 data.qpos，调用 mj_forward，再读取 site/body pose。
    # - 为什么需要: 这是对标 mink.Configuration 的最小数据流，但本步骤不实现完整 FK 算法。
    # - 推荐 API: mujoco.MjData、mujoco.mj_forward、data.site_xpos、data.site_xmat、data.xpos、data.xmat。
    # - 输入是什么: model、data、q、site_id 或 body_id。
    # - 输出是什么: site/body 的位置、旋转矩阵和未来 JSON/Markdown 摘要。
    # - 如何验证: 改变一个关节角后，site pose 应连续变化且没有 NaN。

    # =============================
    # TODO 3: 写出 A02 可复盘结果
    # =============================
    # - 要实现什么: 后续把 pose 摘要写到 reports/A02_site_pose_report.md 和 cache/A02_site_pose.json。
    # - 为什么需要: A03 Jacobian check 和 A04/A05 IK 需要复用同一目标位姿定义。
    # - 推荐 API: Path.mkdir、Path.write_text、json.dumps。
    # - 输入是什么: pose summary dict。
    # - 输出是什么: Markdown report 和 JSON cache。
    # - 如何验证: 报告包含 q 来源、site/body 名称、位置和旋转矩阵 shape。
    raise NotImplementedError("TODO: A02 remains a configuration / site pose learning skeleton.")


if __name__ == "__main__":
    main()
