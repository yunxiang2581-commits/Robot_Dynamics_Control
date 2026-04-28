"""A01 pipeline 第一步: URDF 检查学习型 TODO 骨架。

本文件在 A 项目中的角色:
- 步骤位置: A01, 是 A self-baseline pipeline 的第一环。
- 输入: `configs/robot.yaml` 和一个最终确认的 URDF 路径。
- 输出:
  - `outputs/reports/A01_inspect_urdf_report.md`
  - `outputs/cache/A01_model_summary.json`
- 下游关系: A02 会消费这里确认的 frame 名称和模型摘要。

当前文件有意保持为学习型 TODO 骨架。
它不会完整跑通 Pinocchio 的 URDF 加载流程。

legacy 参考:
`scripts/legacy_imported/task1_inspect_humanoid_model.py`
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path


# 路径定位是 A01 的正式学习内容之一, 因此这里保留显式实现。
A_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = A_ROOT.parents[1]
SRC_ROOT = A_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from robot_baseline import model_loader  # noqa: E402


def parse_args() -> argparse.Namespace:
    """Parse A01 CLI arguments.

    TODO(中文):
    - 要补什么: 后续可以补充 `--package-dir`、`--free-flyer`、`--report-name`
      等更细的学习参数, 但当前先保留最小入口。
    - 为什么需要这一步: 学习脚本应有清晰 CLI, 让配置输入、命令行覆盖和验证
      行为都可复现。
    - 推荐使用什么函数/API: `argparse.ArgumentParser`, `add_argument`。
    - 输入是什么: 命令行参数。
    - 输出是什么: `argparse.Namespace`。
    - 如何验证: 运行 `python .../01_inspect_urdf.py --help` 检查参数是否清晰。
    """
    parser = argparse.ArgumentParser(description="A01 learning skeleton for URDF inspection.")
    parser.add_argument(
        "--config",
        default=str(A_ROOT / "configs" / "robot.yaml"),
        help="A01 配置文件路径, 默认指向 projects/A_self_baseline/configs/robot.yaml。",
    )
    parser.add_argument(
        "--urdf",
        default=None,
        help="可选: 用命令行覆盖 YAML 中的 urdf_path。",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        help="日志级别, 例如 INFO/DEBUG。",
    )
    return parser.parse_args()


def planned_output_paths() -> dict[str, Path]:
    """Return the planned A01 output paths.

    TODO(中文):
    - 要补什么: 后续可迁移到 `pipeline_io.py` 做统一管理, 当前先在 A01 内部
      显式写出标准输出位置。
    - 为什么需要这一步: 用户需要从第一步开始就看清楚 pipeline 的输入输出
      契约, 不应等到算法实现后再补路径规范。
    - 推荐使用什么函数/API: `Path`, `/` 运算符, `mkdir`。
    - 输入是什么: 无显式输入, 依赖 A_ROOT。
    - 输出是什么: 含 `report` 和 `summary_json` 的路径字典。
    - 如何验证: 打印出来的路径应都落在 `projects/A_self_baseline/outputs/` 下。
    """
    return {
        "report": A_ROOT / "outputs" / "reports" / "A01_inspect_urdf_report.md",
        "summary_json": A_ROOT / "outputs" / "cache" / "A01_model_summary.json",
    }


def main() -> None:
    """Run the A01 skeleton flow.

    TODO(中文):
    - 要补什么: 后续按顺序补配置读取、候选 URDF 构造、Pinocchio 模型加载、
      模型摘要、frame 关键词搜索、Markdown/JSON 输出。
    - 为什么需要这一步: A01 是整个 A pipeline 的入口; 这一步先把学习顺序、
      输入、输出和失败提示框架搭起来。
    - 推荐使用什么函数/API: `logging`, `Path`, `json.dumps`,
      `model_loader.*` 系列函数。
    - 输入是什么: `configs/robot.yaml` 和可选 `--urdf`。
    - 输出是什么: 规划中的报告路径、JSON 路径和 TODO 骨架日志。
    - 如何验证: 当前阶段验证脚本可编译、日志清晰、不会伪装成完整实现。
    """
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))

    output_paths = planned_output_paths()
    output_paths["report"].parent.mkdir(parents=True, exist_ok=True)
    output_paths["summary_json"].parent.mkdir(parents=True, exist_ok=True)

    logging.info("A01 config input: %s", args.config)
    logging.info("A01 CLI URDF override: %s", args.urdf)
    logging.info("A01 report output: %s", output_paths["report"])
    logging.info("A01 summary output: %s", output_paths["summary_json"])

    # 路径教学说明:
    # 1. A_ROOT 用于定位 A 项自己的 configs、outputs 和 src。
    # 2. REPO_ROOT 用于定位 shared/robot_assets 等共享资源。
    # 3. 当前先把路径框架写清楚, 模型加载和搜索逻辑后续再逐个补 TODO。
    logging.info("A_ROOT resolved from __file__: %s", A_ROOT)
    logging.info("REPO_ROOT resolved from A_ROOT.parents[1]: %s", REPO_ROOT)

    # Pipeline TODO(中文):
    # - 前置输入: configs/robot.yaml 和最终确认的 URDF 路径。
    # - 本步产物: A01 Markdown 报告、模型摘要 JSON、候选 frame 搜索结果。
    # - 后续消费: A02 读取 frame 名称和模型摘要, A03 复用模型维度与 frame 选择。
    # - 验证重点: 路径定位正确、输出路径固定、日志能说明当前做到了哪一步。

    try:
        # TODO(中文):
        # - 要补什么: 调用 model_loader.load_yaml_config 读取配置。
        # - 为什么需要这一步: 避免把 URDF 路径、关键词和 free_flyer 配置写死在脚本里。
        # - 推荐使用什么函数/API: model_loader.load_yaml_config。
        # - 输入是什么: args.config。
        # - 输出是什么: config dict。
        # - 如何验证: 打印关键字段并与 robot.yaml 对照。
        raise NotImplementedError("TODO: load A01 YAML config.")
    except NotImplementedError as exc:
        skeleton_preview = {
            "status": "TODO skeleton",
            "config_path": args.config,
            "cli_urdf_override": args.urdf,
            "report_path": str(output_paths["report"]),
            "summary_json_path": str(output_paths["summary_json"]),
            "next_todos": [
                "读取 YAML 配置",
                "构造 URDF 候选路径",
                "调用 Pinocchio 加载模型",
                "提取模型摘要",
                "按关键词搜索 frame",
                "生成 Markdown 报告和 JSON 摘要",
            ],
        }
        logging.warning("当前是 TODO 骨架: %s", exc)
        logging.info("A01 skeleton preview:\n%s", json.dumps(skeleton_preview, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
