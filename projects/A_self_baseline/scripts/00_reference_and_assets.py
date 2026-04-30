"""A00 reference and assets TODO learning skeleton.

当前 pipeline 定位：
- A00 reference and assets。
- 对标 mink examples 和 UR5e assets，只做本地参考和资产审计。

本脚本输入：
- `projects/A_self_baseline/external/mink` 或后续 `external/mink_upstream` 参考目录。
- copied UR5e assets / shared robot assets。

本脚本输出：
- `outputs/reports/A00_reference_asset_audit.md`。
- `outputs/cache/A00_reference_asset_paths.json`。

明确不做：
- 不复制整个 mink。
- 不实现 FK / Jacobian / IK / QP / WBC / MuJoCo 控制。
- 不调用 mink 替代自己的实现。
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path


A_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = A_ROOT.parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A00 TODO skeleton: reference and asset audit.")
    parser.add_argument("--mink-reference", default=str(A_ROOT / "external" / "mink"))
    parser.add_argument("--asset-root", default=str(REPO_ROOT / "shared" / "robot_assets"))
    parser.add_argument("--output-dir", default=str(A_ROOT / "outputs"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> None:
    """A00 主流程占位：只说明资产审计 TODO，不执行复制或算法。"""
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    logging.info("A00 当前状态: TODO learning skeleton")
    logging.info("mink reference: %s", args.mink_reference)
    logging.info("asset root: %s", args.asset_root)
    logging.info("future audit report: %s", Path(args.output_dir) / "reports" / "A00_reference_asset_audit.md")
    logging.info("future path cache: %s", Path(args.output_dir) / "cache" / "A00_reference_asset_paths.json")

    # =============================
    # TODO 1: 检查本地参考路径
    # =============================
    # - 要实现什么: 后续检查 mink reference、UR5e scene.xml、mesh/texture 等路径是否存在。
    # - 为什么需要: A01 之前必须确认参考代码和模型资产来自哪里，避免路径错误阻塞主线。
    # - 对标 mink 的概念: examples 依赖本地 scene.xml 和 assets。
    # - 推荐 API: pathlib.Path.exists、Path.rglob、清晰候选路径列表。
    # - 输入是什么: --mink-reference、--asset-root。
    # - 输出是什么: reference asset audit / path summary。
    # - 如何验证: 报告列出检查过的路径、存在状态和缺失原因。

    # =============================
    # TODO 2: 检查许可证和最小复制边界
    # =============================
    # - 要实现什么: 后续只记录必要 asset 的来源、许可证文件和最小复制清单。
    # - 为什么需要: 学习项目不应该把整个 mink 或无关资源复制进来。
    # - 对标 mink 的概念: 只对齐 UR5e examples 需要的资源边界。
    # - 推荐 API: Path.read_text、Markdown 表格。
    # - 输入是什么: LICENSE、README、asset 文件列表。
    # - 输出是什么: 许可证摘要和“最小复制边界”表。
    # - 如何验证: 报告明确说明哪些文件需要、哪些不复制、为什么不复制整个 mink。
    raise NotImplementedError("TODO: A00 remains a reference and assets learning skeleton.")


if __name__ == "__main__":
    main()
