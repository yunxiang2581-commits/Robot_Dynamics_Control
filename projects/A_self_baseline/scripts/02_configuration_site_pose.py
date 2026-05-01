"""A02 configuration / site pose TODO learning skeleton.

Pipeline 步骤：
- A02 configuration / site pose。

对标 mink 的概念：
- `mink.Configuration`。
- A02 要学习的是：给定关节配置 `q`，如何刷新 MuJoCo `data`，再查询
  site/body 在世界坐标系下的位置和姿态。

与 A01 的关系：
- A01 已完成 model inspect，确认 `nq=6`、`nv=6`、`nu=6`。
- A01 已确认目标 site `attachment_site` 存在。
- A01 已确认目标 body `wrist_3_link` 存在。
- A02 未来应读取 A01 的 `A01_model_summary.json`，避免靠猜测写 site/body 名称。

本脚本输入：
- `projects/A_self_baseline/configs/robot.yaml`。
- `projects/A_self_baseline/outputs/cache/A01_model_summary.json`。
- `shared/robot_assets/models/mink_universal_robots_ur5e/scene.xml`。
- target site/body，例如 `attachment_site` / `wrist_3_link`。
- q source，例如 keyframe `home`、zero/default q、文件轨迹或 CLI joint values。

本脚本未来输出：
- `outputs/reports/A02_site_pose_report.md`。
- `outputs/cache/A02_site_pose.json`。

当前状态：
- TODO learning skeleton。
- 不实现真实 MuJoCo site/body pose 查询逻辑。
- 不调用 mink 替代自己的实现。
- 不做 Jacobian / finite difference / IK / QP / WBC / MuJoCo 控制 / video recording。
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path


A_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = A_ROOT / "configs" / "robot.yaml"
DEFAULT_A01_SUMMARY = A_ROOT / "outputs" / "cache" / "A01_model_summary.json"
DEFAULT_OUTPUT_DIR = A_ROOT / "outputs"
DEFAULT_SITE_NAME = "attachment_site"
DEFAULT_BODY_NAME = "wrist_3_link"
DEFAULT_Q_SOURCE = "keyframe:home"


PRINCIPLE_NOTES = [
    "configuration 表示机器人当前广义坐标 q；在 MuJoCo 中，q 通常写入 data.qpos。",
    "从 q 更新到 data 是必要的，因为 MuJoCo 的 site/body 位姿都存放在 forward 后的 data 中。",
    "先做 site/body pose，再做 Jacobian，是为了先确认目标对象和坐标系，再验证速度映射。",
    "site 是 MJCF 中用于任务目标或测量的附着点；body 是刚体节点，二者都有世界系 pose。",
    "attachment_site 是 A 项目默认末端 site，wrist_3_link 是用于对照的末端 body。",
    "A02 输出的 pose summary 会成为 A03 Jacobian check、A04 DLS IK 和 A05 QP-IK 的输入依据。",
]


TODO_TITLES = [
    "TODO 1: 读取 A01 model summary",
    "TODO 2: 读取 robot.yaml 和解析 scene.xml",
    "TODO 3: 加载 MuJoCo model 并创建 data",
    "TODO 4: 选择 q source",
    "TODO 5: 执行 forward kinematics 数据刷新",
    "TODO 6: 查询 site pose",
    "TODO 7: 查询 body pose",
    "TODO 8: 组织 pose summary",
    "TODO 9: 规划 JSON 输出",
    "TODO 10: 规划 Markdown report 输出",
    "TODO 11: 说明 A02 不进入 A03/A04",
]


def parse_args() -> argparse.Namespace:
    """解析 A02 TODO skeleton 的 CLI 参数。"""
    parser = argparse.ArgumentParser(
        description="A02 TODO skeleton: configuration / site pose data flow."
    )
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG),
        help="robot.yaml 配置路径。TODO 2 未来读取。",
    )
    parser.add_argument(
        "--a01-summary",
        default=str(DEFAULT_A01_SUMMARY),
        help="A01_model_summary.json 路径。TODO 1 未来读取。",
    )
    parser.add_argument(
        "--mjcf",
        default=None,
        help="可选：覆盖 robot.yaml 中的 scene.xml 路径。TODO 2 未来解析。",
    )
    parser.add_argument(
        "--site",
        default=DEFAULT_SITE_NAME,
        help="目标 site 名称，默认使用 A01 已确认的 attachment_site。",
    )
    parser.add_argument(
        "--body",
        default=DEFAULT_BODY_NAME,
        help="目标 body 名称，默认使用 A01 已确认的 wrist_3_link。",
    )
    parser.add_argument(
        "--q-source",
        default=DEFAULT_Q_SOURCE,
        help="未来 q 来源，例如 keyframe:home / zero / file:path / cli:values。",
    )
    parser.add_argument(
        "--output-dir",
        default=str(DEFAULT_OUTPUT_DIR),
        help="未来输出根目录。TODO 9/10 规划 cache/report 路径。",
    )
    parser.add_argument("--log-level", default="INFO", help="日志级别，例如 INFO / DEBUG。")
    return parser.parse_args()


def log_principles() -> None:
    """打印 A02 的原理说明，帮助学习者先理解数据流边界。"""
    logging.info("A02 原理说明:")
    for note in PRINCIPLE_NOTES:
        logging.info("- %s", note)


def log_todo_titles() -> None:
    """打印 TODO 1-11 简要列表。"""
    logging.info("A02 TODO 任务清单:")
    for title in TODO_TITLES:
        logging.info("- %s", title)


def main() -> None:
    """A02 TODO learning skeleton 主流程。

    当前只打印 A02 的未来输入、未来输出、原理说明和 TODO 清单。
    核心 site/body pose 查询逻辑继续保留为 NotImplementedError。
    """
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(levelname)s: %(message)s",
    )

    output_root = Path(args.output_dir).expanduser()
    if not output_root.is_absolute():
        output_root = A_ROOT / output_root

    future_cache = output_root / "cache" / "A02_site_pose.json"
    future_report = output_root / "reports" / "A02_site_pose_report.md"

    logging.info("A02 当前状态: TODO learning skeleton")
    logging.info("本步骤不实现 site/body pose 查询，只规划 q -> data -> pose 数据流。")
    logging.info("未来输入:")
    logging.info("- config: %s", args.config)
    logging.info("- A01 summary: %s", args.a01_summary)
    logging.info("- mjcf override: %s", args.mjcf)
    logging.info("- target site/body: %s / %s", args.site, args.body)
    logging.info("- q source: %s", args.q_source)
    logging.info("未来输出:")
    logging.info("- cache: %s", future_cache)
    logging.info("- report: %s", future_report)

    log_principles()
    log_todo_titles()

    # =============================
    # TODO 1: 读取 A01 model summary
    # =============================
    # 要做什么：
    # - 未来读取 `outputs/cache/A01_model_summary.json`。
    # - 确认 `nq/nv/nu`、site_names、body_names、keyframe_names。
    # - 重点确认 `attachment_site` 与 `wrist_3_link`。
    #
    # 为什么这一步存在：
    # - A02 不能靠猜测直接写 site/body 名称；必须消费 A01 的模型检查结果。
    # - 这能把 A01 model inspect 和 A02 pose 查询串成稳定 pipeline。
    #
    # 对标 mink 的哪个概念：
    # - 对标 `mink.Configuration` 创建前先确认 MuJoCo model 和目标 frame/site。
    #
    # 推荐 API：
    # - json.loads
    # - Path.read_text
    #
    # 输入是什么：
    # - A01_model_summary.json。
    #
    # 输出是什么：
    # - model_summary: dict。
    #
    # 如何验证：
    # - summary 中 `nq=6, nv=6, nu=6`。
    # - `attachment_site` 存在于 site_names。
    # - `wrist_3_link` 存在于 body_names。
    # model_summary = json.loads(Path(args.a01_summary).read_text(encoding="utf-8"))

    # =============================
    # TODO 2: 读取 robot.yaml 和解析 scene.xml
    # =============================
    # 要做什么：
    # - 未来读取 `configs/robot.yaml`。
    # - 从配置或 `--mjcf` 解析 `mjcf_path`。
    # - 确保 A02 与 A01 使用同一个模型。
    #
    # 为什么这一步存在：
    # - site/body pose 必须来自同一个 MuJoCo scene.xml，否则 A02 输出无法和 A01 summary 对齐。
    #
    # 对标 mink 的哪个概念：
    # - 对标 mink 示例中先加载 UR5e scene.xml，再基于同一个 model 创建 configuration。
    #
    # 推荐 API：
    # - model_loader.load_yaml_config
    # - model_loader.resolve_path
    #
    # 输入是什么：
    # - robot.yaml。
    #
    # 输出是什么：
    # - mjcf_path: Path。
    #
    # 如何验证：
    # - scene.xml exists=True。
    # config = model_loader.load_yaml_config(args.config)
    # mjcf_path = model_loader.resolve_path(args.mjcf or config["mjcf_path"], A_ROOT)

    # =============================
    # TODO 3: 加载 MuJoCo model 并创建 data
    # =============================
    # 要做什么：
    # - 未来通过 `load_mujoco_model` 加载 model。
    # - 创建 `mujoco.MjData(model)`。
    # - 本步骤不实现 Jacobian。
    #
    # 为什么这一步存在：
    # - MuJoCo 的位姿查询结果存放在 data 中；只有 model 不足以读取 site/body pose。
    #
    # 对标 mink 的哪个概念：
    # - 对标 `mink.Configuration(model)` 内部持有 model/data 并随 q 更新状态。
    #
    # 推荐 API：
    # - model_loader.load_mujoco_model
    # - mujoco.MjData
    #
    # 输入是什么：
    # - model。
    #
    # 输出是什么：
    # - data: mujoco.MjData。
    #
    # 如何验证：
    # - data.qpos shape 与 nq 对齐。
    # - data.qvel shape 与 nv 对齐。
    # model = model_loader.load_mujoco_model(mjcf_path)
    # data = mujoco.MjData(model)

    # =============================
    # TODO 4: 选择 q source
    # =============================
    # 要做什么：
    # - 规划未来几种 q 来源：zero/default q、keyframe home、JSON 或 npy 轨迹文件、CLI joint values。
    # - 第一版最小实现优先规划 keyframe `home`。
    #
    # 为什么这一步存在：
    # - site/body pose 是在某个具体 configuration q 下定义的；没有 q，就没有明确的位姿。
    #
    # 对标 mink 的哪个概念：
    # - 对标 `Configuration.q`，以及示例中从 keyframe 或默认姿态初始化 configuration。
    #
    # 推荐 API：
    # - model.key
    # - data.qpos
    # - model.key_qpos
    #
    # 输入是什么：
    # - --q-source。
    #
    # 输出是什么：
    # - q: array-like，长度为 model.nq。
    #
    # 如何验证：
    # - q.shape == (model.nq,)。
    # q = ...

    # =============================
    # TODO 5: 执行 forward kinematics 数据刷新
    # =============================
    # 要做什么：
    # - 未来把 q 写入 data.qpos。
    # - 调用 `mujoco.mj_forward(model, data)`。
    #
    # 为什么这一步存在：
    # - data.site_xpos / data.xpos 等字段只有在 forward 后才代表当前 q 的世界系状态。
    #
    # 对标 mink 的哪个概念：
    # - 对标 `Configuration.update()` 或配置改变后刷新 frame/site placement 的过程。
    #
    # 推荐 API：
    # - mujoco.mj_forward
    #
    # 输入是什么：
    # - model, data, q。
    #
    # 输出是什么：
    # - 更新后的 data。
    #
    # 如何验证：
    # - data.site_xpos / data.xpos 不含 NaN。
    # data.qpos[:] = q
    # mujoco.mj_forward(model, data)

    # =============================
    # TODO 6: 查询 site pose
    # =============================
    # 要做什么：
    # - 目标 site 默认 `attachment_site`。
    # - 未来根据 site name 找 site id。
    # - 读取 position 和 rotation。
    #
    # 为什么这一步存在：
    # - A03 Jacobian 和 A04 IK 的任务目标通常围绕末端 site pose 定义。
    #
    # 对标 mink 的哪个概念：
    # - 对标 Configuration 中查询 frame/site 当前位姿，供 FrameTask 使用。
    #
    # 推荐 API：
    # - model.site
    # - mujoco.mj_name2id 或已有 helper
    # - data.site_xpos
    # - data.site_xmat
    #
    # 输入是什么：
    # - site name。
    #
    # 输出是什么：
    # - site position。
    # - site rotation matrix。
    #
    # 如何验证：
    # - site pose shape 为 position=(3,), rotation=(3,3)。
    # site_position = ...
    # site_rotation_matrix = ...

    # =============================
    # TODO 7: 查询 body pose
    # =============================
    # 要做什么：
    # - 目标 body 默认 `wrist_3_link`。
    # - 未来读取 body position 和 rotation matrix，用于和 site pose 对照。
    #
    # 为什么这一步存在：
    # - site 是任务点，body 是刚体；对照两者可以帮助理解末端附着点和实际 link 的差异。
    #
    # 对标 mink 的哪个概念：
    # - 对标 Configuration 中 frame/body/site pose 查询的统一概念。
    #
    # 推荐 API：
    # - data.xpos
    # - data.xmat
    #
    # 输入是什么：
    # - body name。
    #
    # 输出是什么：
    # - body position。
    # - body rotation matrix。
    #
    # 如何验证：
    # - body pose shape 为 position=(3,), rotation=(3,3)。
    # body_position = ...
    # body_rotation_matrix = ...

    # =============================
    # TODO 8: 组织 pose summary
    # =============================
    # 要做什么：
    # - 汇总 q source、site pose、body pose、模型维度。
    # - 不做 Jacobian。
    # - 输出给 A03 使用。
    #
    # 为什么这一步存在：
    # - A03 需要知道在同一个 q 和同一个 target site 下验证 Jacobian。
    #
    # 对标 mink 的哪个概念：
    # - 对标 Configuration 当前状态和 task target 的可复用状态记录。
    #
    # 推荐 API：
    # - dict
    # - list
    # - float conversion
    #
    # 输入是什么：
    # - q。
    # - site pose。
    # - body pose。
    #
    # 输出是什么：
    # - JSON-serializable pose summary。
    #
    # 如何验证：
    # - summary 可 json.dumps。
    # pose_summary = {...}

    # =============================
    # TODO 9: 规划 JSON 输出
    # =============================
    # 要做什么：
    # - 未来输出 `outputs/cache/A02_site_pose.json`。
    # - 字段至少包括 q_source、q、site_name、site_position、site_rotation_matrix、
    #   body_name、body_position、body_rotation_matrix、model_nq/nv/nu、source_mjcf。
    #
    # 为什么这一步存在：
    # - JSON cache 让 A03 可以复用 A02 的 q 和目标 pose，不需要重新猜输入。
    #
    # 对标 mink 的哪个概念：
    # - 对标 Configuration 状态被后续 task / solver 复用。
    #
    # 推荐 API：
    # - json.dump
    # - Path.write_text
    #
    # 输入是什么：
    # - pose_summary。
    #
    # 输出是什么：
    # - outputs/cache/A02_site_pose.json。
    #
    # 如何验证：
    # - JSON 可读取，字段完整。
    # Path(...).write_text(...)

    # =============================
    # TODO 10: 规划 Markdown report 输出
    # =============================
    # 要做什么：
    # - 未来输出 `outputs/reports/A02_site_pose_report.md`。
    # - 内容至少包括 A02 目标、输入模型、q source、site pose、body pose、
    #   与 A03 Jacobian 的关系、当前不做什么。
    #
    # 为什么这一步存在：
    # - Markdown report 用来复盘坐标系、目标对象和后续 Jacobian 验证的前置条件。
    #
    # 对标 mink 的哪个概念：
    # - 对标示例中 configuration/task 状态可被人读懂和调试。
    #
    # 推荐 API：
    # - Markdown 字符串
    # - Path.write_text
    #
    # 输入是什么：
    # - pose_summary。
    #
    # 输出是什么：
    # - outputs/reports/A02_site_pose_report.md。
    #
    # 如何验证：
    # - 报告可读，能明确 target site/body。
    # report_path.write_text(...)

    # =============================
    # TODO 11: 说明 A02 不进入 A03/A04
    # =============================
    # 要做什么：
    # - 明确不计算 Jacobian。
    # - 明确不做 finite difference。
    # - 明确不做 IK。
    # - 明确不做 QP。
    #
    # 为什么这一步存在：
    # - A02 只负责 pose 数据流，A03 才验证 velocity mapping；边界清楚才能逐步学习。
    #
    # 对标 mink 的哪个概念：
    # - 对标 Configuration 和 solver/task 的职责分离：A02 只学 configuration 状态查询。
    #
    # 推荐 API：
    # - logging.info
    # - Markdown 边界说明。
    #
    # 输入是什么：
    # - 当前 pipeline 阶段说明。
    #
    # 输出是什么：
    # - 日志和文档中的边界说明。
    #
    # 如何验证：
    # - 运行脚本只打印 TODO skeleton 状态，并继续 raise NotImplementedError。
    logging.info("A02 边界: 不计算 Jacobian，不做 finite difference，不做 IK，不做 QP。")
    raise NotImplementedError("TODO: A02 remains a configuration / site pose learning skeleton.")


if __name__ == "__main__":
    main()
