"""A01 model inspect / MJCF inspect minimal runnable script.

当前 pipeline 定位：
- A01 model inspect。
- 对标 MuJoCo model loading，当前 A 主线重点检查 UR5e `scene.xml`。

学习目标：
- 学会稳定定位 A 项目路径、配置文件和 MuJoCo MJCF 模型文件。
- 学会先检查 MuJoCo model 的 `nq / nv / nu` 和对象名称。
- 为后续 A02-A07 准备 joint/body/site/actuator/keyframe 和末端候选信息。

当前状态：
- A01 最小可运行 model inspect 已补齐。
- 脚本会生成 `outputs/cache/A01_model_summary.json` 和
  `outputs/reports/A01_model_inspect_report.md`。
- 中文 TODO 注释继续保留，用于解释每段实现的学习目的。

明确不做：
- 不做 FK / Jacobian / IK / QP / WBC / MuJoCo 控制。
- 不调用 mink 替代自己的实现。
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path


# =============================
# TODO 1: 路径常量
# =============================
# 要实现什么：
# - 用 __file__ 定位 A 项目根目录 A_ROOT。
# - 用 A_ROOT 定位仓库根目录 REPO_ROOT。
# - 把 A 项目的 src 加入 sys.path，方便导入 robot_baseline。
#
# 为什么需要：
# - 路径定位是本仓库正式学习内容。
# - 脚本不应该依赖 shell 当前工作目录。
#
# 对标 mink 的哪个概念：
# - mink 示例通常有明确的模型资源路径；A 项目需要先学会稳定定位自己的 scene.xml。
#
# 推荐 API：
# - Path(__file__).resolve()
# - Path.parents
# - sys.path.insert
#
# 输入是什么：
# - 当前脚本文件路径 __file__。
#
# 输出是什么：
# - A_ROOT / REPO_ROOT / SRC_ROOT。
#
# 如何验证：
# - 运行脚本时打印 A_ROOT 和 REPO_ROOT，确认路径指向预期目录。
A_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = A_ROOT.parents[1]
SRC_ROOT = A_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from robot_baseline import model_loader  # noqa: E402,F401


# =============================
# TODO 2: 默认输入与输出路径
# =============================
# 要实现什么：
# - 定义默认 robot.yaml 路径。
# - 定义 A01 JSON summary 和 Markdown report 的目标路径。
#
# 为什么需要：
# - A01 是 A02-A07 的模型基础信息入口，输出路径需要稳定。
# - 当前用于生成真实报告和缓存路径。
#
# 对标 mink 的哪个概念：
# - 对标 mink UR5e 示例中固定的 scene.xml 输入和后续 configuration/task 复用模型信息。
#
# 推荐 API：
# - pathlib.Path
# - 路径 `/` 运算符
#
# 输入是什么：
# - A_ROOT。
#
# 输出是什么：
# - DEFAULT_CONFIG / DEFAULT_OUTPUT_DIR / DEFAULT_REPORT_FILE / DEFAULT_SUMMARY_FILE。
#
# 如何验证：
# - 打印这些路径，确认都落在 projects/A_self_baseline 下。
DEFAULT_CONFIG = A_ROOT / "configs" / "robot.yaml"
DEFAULT_OUTPUT_DIR = A_ROOT / "outputs"
DEFAULT_REPORT_FILE = DEFAULT_OUTPUT_DIR / "reports" / "A01_model_inspect_report.md"
DEFAULT_SUMMARY_FILE = DEFAULT_OUTPUT_DIR / "cache" / "A01_model_summary.json"


# =============================
# TODO 3: 末端候选名称
# =============================
# 要实现什么：
# - 显式列出 A01 最终要检查的末端候选名称。
#
# 为什么需要：
# - A02 的 site pose、A03 Jacobian、A04 IK 都需要稳定的末端目标。
# - 不应该靠猜测直接写死一个末端名。
#
# 对标 mink 的哪个概念：
# - 对标 UR5e 示例围绕 attachment site / tool site 做任务空间控制。
#
# 推荐 API：
# - Python list
# - 后续配合 model_loader.summarize_mujoco_model
#
# 输入是什么：
# - robot.yaml 中的 end_effector_candidates，当前这里保留默认学习列表。
#
# 输出是什么：
# - END_EFFECTOR_CANDIDATES。
#
# 如何验证：
# - 后续 A01 最小实现中应确认 attachment_site 命中 site，wrist_3_link 命中 body。
END_EFFECTOR_CANDIDATES = [
    "attachment_site",
    "tool0",
    "ee_link",
    "wrist_3_link",
]


def parse_args() -> argparse.Namespace:
    """
    TODO 4: 解析 CLI 参数。

    当前实现：
    - 保留 `--config`、`--mjcf`、`--output-dir`、`--log-level`。
    - 解析 CLI 参数，供 A01 model inspect 主流程使用。

    为什么需要：
    - A01 后续既要支持 robot.yaml 默认值，也要支持命令行覆盖。

    对标 mink 的哪个概念：
    - mink 示例明确指定 UR5e scene.xml；这里通过 CLI 让 scene.xml 路径可覆盖。

    推荐 API：
    - argparse.ArgumentParser
    - parser.add_argument

    输入是什么：
    - 命令行参数。

    输出是什么：
    - argparse.Namespace。

    如何验证：
    - 运行 `python projects/A_self_baseline/scripts/01_model_inspect.py --help`。
    """
    parser = argparse.ArgumentParser(
        description="A01 completed minimal model inspect / MJCF inspect for UR5e-style baseline."
    )
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG),
        help="robot.yaml 配置路径。TODO 7 才读取。",
    )
    parser.add_argument(
        "--mjcf",
        default=None,
        help="可选：覆盖 robot.yaml 中的 mjcf_path。TODO 8 才解析。",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="可选：覆盖输出目录。TODO 5 规划路径，TODO 12/13 才写文件。",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        help="日志级别，例如 INFO / DEBUG。",
    )
    return parser.parse_args()


def planned_output_paths(output_dir: str | None) -> dict[str, Path]:
    """
    TODO 5: 计算输出路径。

    当前实现：
    - 根据 `--output-dir` 或 DEFAULT_OUTPUT_DIR 得到 report 和 summary_json 路径。
    - 当前只返回路径；目录创建和写文件由 TODO 12/13 对应函数完成。

    为什么需要：
    - A01 后续会生成 JSON summary 和 Markdown report。
    - 路径先明确，后续补实现时不容易写散。

    对标 mink 的哪个概念：
    - mink 示例中的模型信息会被后续 task/configuration 使用；A 项目用固定输出路径缓存检查结果。

    推荐 API：
    - Path.expanduser
    - Path.is_absolute
    - pathlib 路径拼接

    输入是什么：
    - output_dir: CLI `--output-dir`。

    输出是什么：
    - dict: {"report": Path, "summary_json": Path}。

    如何验证：
    - 打印返回路径，确认默认落在 `projects/A_self_baseline/outputs` 下。
    """
    base = Path(output_dir).expanduser() if output_dir else DEFAULT_OUTPUT_DIR
    if not base.is_absolute():
        base = A_ROOT / base
    return {
        "report": base / "reports" / "A01_model_inspect_report.md",
        "summary_json": base / "cache" / "A01_model_summary.json",
    }


def log_a01_final_targets() -> None:
    """
    TODO 6: 打印 A01 检查目标。

    当前实现：
    - 输出 A01 已检查或需要复核的模型对象清单。
    - 这些目标会写入 JSON summary 和 Markdown report。

    为什么需要：
    - 学习脚本先让读者明确本步骤要产出什么，再进入实现。

    对标 mink 的哪个概念：
    - 对标 UR5e scene inspect 和 Configuration 前置维度检查。

    推荐 API：
    - logging.info

    输入是什么：
    - 无。

    输出是什么：
    - 日志文本。

    如何验证：
    - 运行脚本应能看到 nq/nv/nu、对象名称、末端候选等目标说明。
    """
    logging.info("A01 最终输出目标:")
    logging.info("- nq / nv / nu")
    logging.info("- joint names / body names / site names / actuator names / keyframe names")
    logging.info("- end-effector candidates: %s", ", ".join(END_EFFECTOR_CANDIDATES))


def main() -> None:
    """
    A01 最小可运行 model inspect 主流程。

    输入：
    - robot.yaml 配置；
    - 可选 MJCF 路径覆盖项；
    - 输出目录。

    输出：
    - A01_model_summary.json；
    - A01_model_inspect_report.md。

    数学逻辑：
    - 本步骤只检查模型对象，不改变 FK/Jacobian/IK/QP 数学逻辑。
    """
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(levelname)s: %(message)s",
    )

    output_paths = planned_output_paths(args.output_dir)

    logging.info("A01 当前状态: minimal runnable model inspect")
    logging.info("A_ROOT: %s", A_ROOT)
    logging.info("REPO_ROOT: %s", REPO_ROOT)
    logging.info("config argument: %s", args.config)
    logging.info("mjcf override argument: %s", args.mjcf)
    logging.info("output-dir argument: %s", args.output_dir)
    logging.info("JSON summary 目标: %s", output_paths["summary_json"])
    logging.info("Markdown report 目标: %s", output_paths["report"])

    log_a01_final_targets()

    # =============================
    # TODO 7: 读取 robot.yaml
    # =============================
    # 当前实现：
    # - config = model_loader.load_yaml_config(args.config)
    #
    # 为什么需要：
    # - A01 不应把 mjcf_path、末端候选、输出目录写死在脚本中。
    #
    # 对标 mink 的哪个概念：
    # - 对标 mink 示例中先确定 UR5e scene.xml 输入，再创建模型和 configuration。
    #
    # 推荐 API：
    # - model_loader.load_yaml_config
    # - yaml.safe_load
    # - pathlib.Path
    #
    # 输入是什么：
    # - args.config
    #
    # 输出是什么：
    # - config: dict
    #
    # 如何验证：
    # - 打印 config keys，确认包含 mjcf_path、end_effector_candidates、output_dir。
    config = model_loader.load_yaml_config(args.config)
    logging.info("Loaded config keys: %s", list(config.keys()))
    # =============================
    # TODO 8: 解析 mjcf_path
    # =============================
    # 当前实现：
    # - 从 args.mjcf 或 config["mjcf_path"] 得到 MJCF 路径。
    # - 调用 model_loader.resolve_path(...) 得到绝对 Path。
    #
    # 为什么需要：
    # - MuJoCo model loading 依赖 scene.xml 路径，路径错误会阻塞后续全部任务。
    #
    # 对标 mink 的哪个概念：
    # - 对标 mink UR5e 示例中明确加载 scene.xml 的模型入口。
    #
    # 推荐 API：
    # - model_loader.resolve_path
    # - Path.exists
    #
    # 输入是什么：
    # - args.mjcf
    # - config["mjcf_path"]
    # - A_ROOT 或配置文件所在目录
    #
    # 输出是什么：
    # - mjcf_path: Path
    #
    # 如何验证：
    # - 打印 mjcf_path，并确认 mjcf_path.exists() 为 True。
    mjcf_value = args.mjcf or config["mjcf_path"]
    mjcf_path = model_loader.resolve_path(mjcf_value, A_ROOT)

    logging.info("Resolved MJCF path: %s", mjcf_path)
    logging.info("MJCF path exists: %s", mjcf_path.exists())

    # =============================
    # TODO 9: 加载 MuJoCo model
    # =============================
    # 当前实现：
    # - model = model_loader.load_mujoco_model(mjcf_path)
    #
    # 为什么需要：
    # - A01 的 nq、nv、nu 和对象名称都来自 MuJoCo model。
    #
    # 对标 mink 的哪个概念：
    # - 对标 mujoco.MjModel.from_xml_path 和 mink.Configuration(model) 的前置模型对象。
    #
    # 推荐 API：
    # - model_loader.load_mujoco_model
    # - mujoco.MjModel.from_xml_path
    #
    # 输入是什么：
    # - mjcf_path
    #
    # 输出是什么：
    # - model: mujoco.MjModel
    #
    # 如何验证：
    # - 打印 model.nq、model.nv、model.nu，UR5e 预期为 6/6/6。
    model = model_loader.load_mujoco_model(mjcf_path)
    logging.info("Model nq: %d", model.nq)
    logging.info("Model nv: %d", model.nv)
    logging.info("Model nu: %d", model.nu)
    # =============================
    # TODO 10: 枚举 joint/body/site/actuator/keyframe
    # =============================
    # 当前实现：
    # - 分别调用 model_loader.get_mujoco_names(model, obj_type)。
    #
    # 为什么需要：
    # - 后续 A02-A07 必须知道 site、body、actuator 的真实名字，不能靠猜。
    #
    # 对标 mink 的哪个概念：
    # - 对标 mink 任务中使用 site/task/actuator 名称前的 scene inspect。
    #
    # 推荐 API：
    # - model_loader.get_mujoco_names
    # - mujoco.mj_id2name
    # - mujoco.mjtObj
    #
    # 输入是什么：
    # - model
    # - obj_type 字符串
    #
    # 输出是什么：
    # - joint_names / body_names / site_names / actuator_names / keyframe_names
    #
    # 如何验证：
    # - 列表长度应与 model.njnt、model.nbody、model.nsite、model.nu、model.nkey 一致。
    joint_names = model_loader.get_mujoco_names(model, "joint")
    body_names = model_loader.get_mujoco_names(model, "body")
    site_names = model_loader.get_mujoco_names(model, "site")
    actuator_names = model_loader.get_mujoco_names(model, "actuator")
    keyframe_names = model_loader.get_mujoco_names(model, "keyframe")
    logging.info("Joint names: %s", joint_names)
    logging.info("Body names: %s", body_names)
    logging.info("Site names: %s", site_names)
    logging.info("Actuator names: %s", actuator_names)
    logging.info("Keyframe names: %s", keyframe_names)
    # =============================
    # TODO 11: 检查 end-effector candidates
    # =============================
    # 当前实现：
    # - 调用 model_loader.summarize_mujoco_model(model, candidates)。
    # - 查看 attachment_site、tool0、ee_link、wrist_3_link 是否存在。
    #
    # 为什么需要：
    # - A02 的 site pose 和后续 IK 需要稳定的末端目标名称。
    #
    # 对标 mink 的哪个概念：
    # - 对标 UR5e 示例围绕 attachment_site 或 tool site 做任务空间控制。
    #
    # 推荐 API：
    # - model_loader.summarize_mujoco_model
    # - Python set/list membership
    #
    # 输入是什么：
    # - model
    # - config["end_effector_candidates"] 或 END_EFFECTOR_CANDIDATES
    #
    # 输出是什么：
    # - summary: dict
    # - end_effector_check: dict[str, bool]
    #
    # 如何验证：
    # - attachment_site 应命中 site，wrist_3_link 应命中 body。
    candidates = config.get("end_effector_candidates", END_EFFECTOR_CANDIDATES)
    summary = model_loader.summarize_mujoco_model(model, candidates)
    logging.info("end_effector_check: %s", summary["end_effector_check"])
    # =============================
    # TODO 12: 写 JSON summary
    # =============================
    # 当前实现：
    # - model_loader.write_json_summary(summary, output_paths["summary_json"])
    #
    # 为什么需要：
    # - A02-A07 可以复用 JSON 摘要，减少重复人工检查。
    #
    # 对标 mink 的哪个概念：
    # - 对标 mink 中模型对象被后续 configuration/task 复用的工作流。
    #
    # 推荐 API：
    # - model_loader.write_json_summary
    # - json.dumps
    # - Path.write_text
    #
    # 输入是什么：
    # - summary dict
    # - outputs/cache/A01_model_summary.json
    #
    # 输出是什么：
    # - A01_model_summary.json
    #
    # 如何验证：
    # - 打开 JSON，确认 nq/nv/nu 和名称列表字段完整。
    model_loader.write_json_summary(summary, output_paths["summary_json"])

    logging.info("Wrote JSON summary to: %s", output_paths["summary_json"])
    # =============================
    # TODO 13: 写 Markdown report
    # =============================
    # 当前实现：
    # - model_loader.write_model_report(summary, output_paths["report"])
    #
    # 为什么需要：
    # - 报告便于复盘、调试和后续任务确认对象名称。
    #
    # 对标 mink 的哪个概念：
    # - 对标 UR5e scene inspect 的人工可读模型对象清单。
    #
    # 推荐 API：
    # - model_loader.write_model_report
    # - pathlib.Path.write_text
    # - Markdown 列表
    #
    # 输入是什么：
    # - summary dict
    # - outputs/reports/A01_model_inspect_report.md
    #
    # 输出是什么：
    # - A01_model_inspect_report.md
    #
    # 如何验证：
    # - 报告包含 nq/nv/nu、joint/body/site/actuator/keyframe、末端候选检查。
    model_loader.write_model_report(summary, output_paths["report"])
    logging.info("Wrote Markdown report to: %s", output_paths["report"])


if __name__ == "__main__":
    main()
