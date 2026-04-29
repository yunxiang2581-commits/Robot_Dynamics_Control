"""A01 pipeline 第一步: model inspect / MJCF inspect 学习型 TODO 骨架。

所属 pipeline 步骤:
- A01 model inspect, 是 UR5e/mink-style A 项目主线的第一步。

对标 mink 的概念:
- MuJoCo model loading。
- UR5e `scene.xml` 中的 joint、body、site、actuator、keyframe 检查。
- 后续 `Configuration`、site pose 和 IK 都依赖这里确认的模型维度与对象名称。

本脚本输入:
- `configs/robot.yaml`。
- 第一版重点输入是 `mjcf_path`, 对标 mink UR5e `scene.xml`。
- `urdf_path` 只作为可选补充, 不是当前主线阻塞项。

本脚本输出:
- `outputs/reports/A01_model_inspect_report.md`。
- `outputs/cache/A01_model_summary.json`。

当前状态:
- TODO learning skeleton。
- 不完整实现 MuJoCo/Pinocchio 加载流程。
- 不调用 mink 替代自己的实现。

legacy 参考:
- `scripts/legacy_imported/task1_inspect_humanoid_model.py` 仅作为历史学习参考。
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
      模型摘要、body/site/actuator/keyframe 搜索、Markdown/JSON 输出。
    - 为什么需要这一步: A01 是整个 UR5e/mink-style pipeline 的入口; 后续
      site pose、Jacobian、IK 和 actuator tracking 都依赖这里确认的模型对象。
    - 对标 mink 的哪个概念: MuJoCo model loading 和 UR5e scene inspect。
    - 推荐使用什么函数/API: `mujoco.MjModel.from_xml_path`, `logging`,
      `Path`, `json.dumps`, 后续可保留 `model_loader.*` 作为可选 URDF 辅助。
    - 输入是什么: `configs/robot.yaml` 中的 `mjcf_path`, 可选 `urdf_path`。
    - 输出是什么: A01 model inspect 报告路径、JSON 摘要路径和 TODO 骨架日志。
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
    # 3. 当前先把路径框架写清楚, MJCF 模型加载和对象搜索逻辑后续再逐个补 TODO。
    logging.info("A_ROOT resolved from __file__: %s", A_ROOT)
    logging.info("REPO_ROOT resolved from A_ROOT.parents[1]: %s", REPO_ROOT)

    # Pipeline TODO(中文):
    # - 前置输入: configs/robot.yaml 和最终确认的 UR5e scene.xml / MJCF 路径。
    # - 本步产物: A01 Markdown 报告、模型摘要 JSON、joint/body/site/actuator/keyframe 清单。
    # - 后续消费: A02 读取 site 名称和模型摘要, A03 复用模型维度与 site 选择。
    # - 验证重点: 路径定位正确、输出路径固定、日志能说明当前做到了哪一步。

    try:
        # TODO(中文):
        # - 要补什么: 调用 model_loader.load_yaml_config 读取配置。
        # - 为什么需要这一步: 避免把 MJCF 路径、末端候选名称和搜索关键词写死在脚本里。
        # - 对标 mink 的哪个概念: mink 示例先加载 MuJoCo model, 再基于模型对象做 configuration 和 task。
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
                "确认 mjcf_path 和可选 urdf_path",
                "调用 mujoco.MjModel.from_xml_path 加载 scene.xml",
                "检查 nq、nv、nu",
                "列出 joint、body、site、actuator、keyframe",
                "搜索 attachment_site、tool0、ee_link、wrist_3_link 等末端候选",
                "生成 Markdown 报告和 JSON 摘要",
            ],
        }
        logging.warning("当前是 TODO 骨架: %s", exc)
        logging.info("A01 skeleton preview:\n%s", json.dumps(skeleton_preview, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
