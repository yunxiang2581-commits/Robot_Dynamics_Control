"""A01 model inspect / MJCF inspect CLI TODO learning skeleton.

当前实际定位:
- A01 model inspect / MJCF inspect。
- 文件名暂时保持 `01_inspect_urdf.py`，但当前 A 主线对标 mink UR5e `scene.xml`。

对标 mink 的概念:
- MuJoCo model loading。
- UR5e scene inspect。
- `Configuration` 创建前的模型维度和对象名称检查。

当前状态:
- TODO learning skeleton。
- 不完整调用 MuJoCo model loading。
- 不完整生成模型摘要。
- 不完整写 JSON/Markdown 报告。
- 不调用 mink 替代自己的实现。
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path


# 路径定位是正式学习内容。
# A_ROOT: projects/A_self_baseline
# REPO_ROOT: 仓库根目录 /home/ubuntu/Robot_Dynamics_Control
A_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = A_ROOT.parents[1]
SRC_ROOT = A_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from robot_baseline import model_loader  # noqa: E402,F401


TODO_ITEMS = [
    "TODO 1: 读取 robot.yaml",
    "TODO 2: 解析 mjcf_path",
    "TODO 3: 加载 MuJoCo model",
    "TODO 4: 枚举 joint/body/site/actuator/keyframe",
    "TODO 5: 检查 end-effector candidates",
    "TODO 6: 写 JSON summary",
    "TODO 7: 写 Markdown report",
]


def parse_args() -> argparse.Namespace:
    """解析 A01 CLI 参数。

    CLI 说明(中文教学):
    - 要做什么: 保留 `--config`, `--mjcf`, `--output-dir` 三个入口参数。
    - 为什么这一步存在: A01 后续既要支持配置文件默认值，也要支持命令行覆盖。
    - 对标 mink 的哪个概念: mink 示例明确指定 UR5e scene.xml；这里用 CLI 让 scene.xml
      路径可显式覆盖。
    - 推荐使用什么 API: `argparse.ArgumentParser`, `add_argument`。
    - 输入是什么: 用户命令行参数。
    - 输出是什么: `argparse.Namespace`。
    - 如何验证: 运行 `python projects/A_self_baseline/scripts/01_inspect_urdf.py --help`。
    """
    parser = argparse.ArgumentParser(
        description="A01 TODO skeleton: model inspect / MJCF inspect for UR5e-style baseline."
    )
    parser.add_argument(
        "--config",
        default=str(A_ROOT / "configs" / "robot.yaml"),
        help="robot.yaml 配置路径。当前只保留参数，不完整读取。",
    )
    parser.add_argument(
        "--mjcf",
        default=None,
        help="可选: 用命令行覆盖 robot.yaml 中的 mjcf_path。当前只记录，不完整加载。",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="可选: 输出目录覆盖项。当前只记录，不创建真实报告或缓存。",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        help="日志级别，例如 INFO 或 DEBUG。",
    )
    return parser.parse_args()


def main() -> None:
    """打印 A01 TODO skeleton 状态，不运行完整 model inspect。

    A01 主流程说明(中文教学):
    - 要做什么: 后续按 TODO_ITEMS 的顺序逐步补 A01 最小可运行实现。
    - 为什么这一步存在: A01 是 A02-A07 的模型基础信息准备阶段，不应跳过模型检查。
    - 对标 mink 的哪个概念: MuJoCo model loading、UR5e scene inspect、Configuration
      前置模型维度检查。
    - 推荐使用什么 API: `logging`, `Path`, 后续再接 `model_loader.*`。
    - 输入是什么: `--config`, `--mjcf`, `--output-dir`。
    - 输出是什么: 当前只输出 TODO skeleton 日志，不生成真实 outputs/reports 或 outputs/cache。
    - 如何验证: 脚本能 py_compile，运行时日志明确说明“不是完整实现”。
    """
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(levelname)s: %(message)s",
    )

    planned_report = A_ROOT / "outputs" / "reports" / "A01_model_inspect_report.md"
    planned_summary = A_ROOT / "outputs" / "cache" / "A01_model_summary.json"

    logging.info("A01 当前状态: TODO learning skeleton")
    logging.info("当前不会完整加载 MuJoCo model，也不会生成真实报告或缓存。")
    logging.info("A_ROOT: %s", A_ROOT)
    logging.info("REPO_ROOT: %s", REPO_ROOT)
    logging.info("config argument: %s", args.config)
    logging.info("mjcf override argument: %s", args.mjcf)
    logging.info("output-dir argument: %s", args.output_dir)
    logging.info("未来 JSON summary 目标: %s", planned_summary)
    logging.info("未来 Markdown report 目标: %s", planned_report)

    logging.info("A01 最终输出目标:")
    logging.info("- nq / nv / nu")
    logging.info("- joint names / body names / site names / actuator names / keyframe names")
    logging.info("- end-effector candidates: attachment_site, tool0, ee_link, wrist_3_link")

    logging.info("后续实现顺序:")
    for item in TODO_ITEMS:
        logging.info("- %s", item)

    logging.warning("Step 9A 只整理 TODO 骨架；Step 9B 才补最小可运行 model inspect。")


if __name__ == "__main__":
    main()
