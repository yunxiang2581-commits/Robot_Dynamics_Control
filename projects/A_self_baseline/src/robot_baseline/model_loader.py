"""A01 model inspect / MJCF inspect helper.

本模块服务于 A_self_baseline 的 Step 9A / A01：model inspect。
当前已具备 A01 最小可运行 model inspect 能力：读取配置、解析 MJCF 路径、
加载 MuJoCo model、枚举模型对象，并写出 JSON summary / Markdown report。

A01 对标 mink UR5e 示例中的前置模型检查：先确认 MuJoCo model 的维度
和对象名称，再进入 A02-A07 的 Configuration、FK/Jacobian/IK/QP/控制。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any
import json
import mujoco
import yaml


def load_yaml_config(config_path: str | Path) -> dict:
    """读取 A01 配置文件的最小实现。

    中文教学说明:
    - 要做什么: 读取 `configs/robot.yaml`，得到 robot_name、model_type、mjcf_path、
      end_effector_candidates、frame_keywords、output_dir 等字段。
    - 为什么这一步存在: A01 不应把模型路径和末端候选名称写死在脚本里；配置文件
      是后续 A02-A07 共享输入契约的起点。
    - 对标 mink 的哪个概念: mink UR5e 示例先明确 scene.xml，再基于该模型创建
      configuration 和 task；这里先把 scene.xml 路径放入配置。
    - 推荐使用什么 API: `pathlib.Path`, `Path.read_text`, `yaml.safe_load`。
    - 输入是什么: `config_path`，来自 CLI `--config` 或默认配置路径。
    - 输出是什么: `dict`，表示 YAML 中的配置内容。
    - 如何验证: A01 运行时打印 key 列表，并检查是否包含 `mjcf_path`
      和 `end_effector_candidates`。
    """
    if not isinstance(config_path, (str, Path)):
        raise TypeError(f"config_path must be str or Path, got {type(config_path)!r}")
    
    path = Path(config_path).expanduser().resolve()

    if not path.exists():
        raise FileNotFoundError(f"Config file not found at {path}")

    text = path.read_text(encoding="utf-8")
    config = yaml.safe_load(text)
    if not isinstance(config, dict):
        raise ValueError(f"Config file {path} did not contain a YAML mapping (dict), got {type(config)!r}")

    return config


def resolve_path(path_str: str | Path, base_dir: Path) -> Path:
    """解析配置或 CLI 路径的最小实现。

    中文教学说明:
    - 要做什么: 把绝对路径、相对路径、`~` 路径统一解析成 `Path`。
    - 为什么这一步存在: A01 的核心学习内容之一是稳定定位模型资产；路径不清楚，
      后续 model loading、报告输出都会反复失败。
    - 对标 mink 的哪个概念: mink 示例通过明确的 scene.xml 路径加载 MuJoCo model；
      这里学习如何在自己的项目中稳定解析该路径。
    - 推荐使用什么 API: `pathlib.Path`, `expanduser`, `is_absolute`, `resolve`。
    - 输入是什么: `path_str` 是 YAML 或 CLI 给出的路径，`base_dir` 是相对路径基准。
    - 输出是什么: 解析后的 `Path`。
    - 如何验证: A01 运行时打印解析前后的 MJCF 路径并确认文件存在。
    """
    if not isinstance(path_str, (str, Path)):
        raise TypeError(f"path_str must be str or Path, got {type(path_str)!r}")
    if not isinstance(base_dir, Path):
        raise TypeError(f"base_dir must be Path, got {type(base_dir)!r}")
    

    path = Path(path_str).expanduser()

    if path.is_absolute():
        return path.resolve()
    return (base_dir / path).resolve()


def load_mujoco_model(mjcf_path: str | Path):
    """加载 MuJoCo MJCF model 的最小实现。

    中文教学说明:
    - 要做什么: 使用 MJCF 文件创建 MuJoCo `model`。
    - 为什么这一步存在: A01 最终要检查 `nq`, `nv`, `nu` 以及 joint/body/site/
      actuator/keyframe 名称，这些信息都来自 MuJoCo model。
    - 对标 mink 的哪个概念: 对标 mink UR5e 示例中的 `mujoco.MjModel.from_xml_path`
      和后续 `Configuration(model)` 的模型前置条件。
    - 推荐使用什么 API: `mujoco.MjModel.from_xml_path(str(mjcf_path))`。
    - 输入是什么: `mjcf_path`，指向 UR5e `scene.xml`。
    - 输出是什么: MuJoCo `MjModel` 对象。
    - 如何验证: A01 运行时打印 `model.nq`, `model.nv`, `model.nu`，不进入控制。
    """
    if not isinstance(mjcf_path, (str, Path)):
        raise TypeError(f"mjcf_path must be str or Path, got {type(mjcf_path)!r}")
    path = Path(mjcf_path).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"MJCF file not found at {path}")

    return mujoco.MjModel.from_xml_path(str(path))


def get_mujoco_names(model: Any, obj_type: str) -> list[str]:
    """枚举 MuJoCo 对象名称的最小实现。

    中文教学说明:
    - 要做什么: 按对象类型枚举 joint、body、site、actuator、keyframe 名称。
    - 为什么这一步存在: A01 要先确认模型对象名称，后续 A02 site pose、A03
      Jacobian、A06 actuator tracking 都依赖这些名称。
    - 对标 mink 的哪个概念: mink UR5e 示例围绕 site/task/actuator 名称配置任务，
      A01 先自己检查这些名字，而不是盲目猜测。
    - 推荐使用什么 API: `mujoco.mj_id2name`, `mujoco.mjtObj`, 以及 model 中的数量字段。
    - 输入是什么: `model` 是 MuJoCo MjModel，`obj_type` 是字符串，例如 `joint`、
      `body`、`site`、`actuator`、`keyframe`。
    - 输出是什么: `list[str]`，对应对象类型的名称列表。
    - 如何验证: A01 检查列表长度和 MuJoCo model 的数量字段是否一致。
    """
    if not isinstance(obj_type, str):
        raise TypeError(f"obj_type must be str, got {type(obj_type)!r}")
    
    obj_type_map = {
        "joint": (mujoco.mjtObj.mjOBJ_JOINT, model.njnt),
        "body": (mujoco.mjtObj.mjOBJ_BODY, model.nbody),
        "site": (mujoco.mjtObj.mjOBJ_SITE, model.nsite),
        "actuator": (mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu),
        "keyframe": (mujoco.mjtObj.mjOBJ_KEY, model.nkey),
    }


    if obj_type not in obj_type_map:
        raise ValueError(f"Unsupported object type: {obj_type}")
    
    mj_obj_type, count = obj_type_map[obj_type]

    names = []
    for obj_id in range(count):
        name = mujoco.mj_id2name(model, mj_obj_type, obj_id)
        if name is None:
            raise ValueError(f"Object ID {obj_id} of type {obj_type} has no name.")
        names.append(name)
    return names


def summarize_mujoco_model(model: Any, end_effector_candidates: list[str] | None = None) -> dict:
    """生成 MuJoCo 模型摘要的最小实现。

    中文教学说明:
    - 要做什么: 组织 `nq`, `nv`, `nu`, joint/body/site/actuator/keyframe 名称，
      并检查末端候选名称是否存在。
    - 为什么这一步存在: A01 的最终产物是后续任务可复用的模型基础信息，不只是终端打印。
    - 对标 mink 的哪个概念: 对标 UR5e scene inspect 和 `Configuration` 创建前的模型
      维度/对象名称确认。
    - 推荐使用什么 API: `model.nq`, `model.nv`, `model.nu`, `get_mujoco_names`,
      Python dict/list。
    - 输入是什么: `model` 是 MuJoCo MjModel，`end_effector_candidates` 来自 robot.yaml。
    - 输出是什么: `dict`，写入 `A01_model_summary.json`。
    - 如何验证: A01 确认 summary 包含维度字段和末端候选检查结果。
    """

    if end_effector_candidates is not None and not isinstance(end_effector_candidates, list):
        raise TypeError("end_effector_candidates must be list[str] or None.")
    joint_names = get_mujoco_names(model, "joint")
    body_names = get_mujoco_names(model, "body")
    site_names = get_mujoco_names(model, "site")
    actuator_names = get_mujoco_names(model, "actuator")
    keyframe_names = get_mujoco_names(model, "keyframe")

    if end_effector_candidates is None:
        end_effector_candidates = []

    available_names = set(body_names) | set(site_names)

    end_effector_check = {}
    for candidate in end_effector_candidates:
        if candidate in available_names:
            end_effector_check[candidate] = True
        else:
            end_effector_check[candidate] = False
    summary = {
        "nq": int(model.nq),
        "nv": int(model.nv),
        "nu": int(model.nu),
        "joint_names": joint_names,
        "body_names": body_names,
        "site_names": site_names,
        "actuator_names": actuator_names,
        "keyframe_names": keyframe_names,
        "end_effector_candidates": end_effector_candidates,
        "end_effector_check": end_effector_check,
    }
    return summary



def write_json_summary(summary: dict, output_path: Path) -> None:
    """写入 JSON 模型摘要的最小实现。

    中文教学说明:
    - 要做什么: 把 `summary` 保存为 JSON 文件。
    - 为什么这一步存在: A02-A07 可以读取 JSON 摘要，避免重复人工检查模型对象名称。
    - 对标 mink 的哪个概念: mink 示例中模型对象被代码直接使用；A 项目先把检查结果
      缓存下来，便于学习和复盘。
    - 推荐使用什么 API: `json.dumps`, `Path.parent.mkdir`, `Path.write_text`。
    - 输入是什么: `summary` 是模型摘要 dict，`output_path` 是 JSON 输出路径。
    - 输出是什么: 无返回；副作用是写入 `outputs/cache/A01_model_summary.json`。
    - 如何验证: 打开 JSON，检查字段和终端日志一致。
    """
    if not isinstance(summary, dict):
        raise TypeError(f"summary must be dict, got {type(summary)!r}")
    if not isinstance(output_path, Path):
        raise TypeError(f"output_path must be Path, got {type(output_path)!r}")
    
    output_path.parent.mkdir(parents=True, exist_ok=True)  # 确保目录存在，避免写入失败。
    json_text = json.dumps(summary, indent=2, ensure_ascii=False)
    output_path.write_text(json_text, encoding="utf-8")


def write_model_report(summary: dict, output_path: Path) -> None:
    """写入 Markdown 模型检查报告的最小实现。

    中文教学说明:
    - 要做什么: 把模型摘要整理成 Markdown 报告。
    - 为什么这一步存在: A01 需要一个人类可读的模型检查报告，便于确认后续任务用哪个
      site/body/actuator 名称。
    - 对标 mink 的哪个概念: 对标 UR5e scene inspect；先用报告明确模型对象，再进入
      mink-style task / configuration 学习。
    - 推荐使用什么 API: `Path.parent.mkdir`, `Path.write_text`, f-string, Markdown 列表。
    - 输入是什么: `summary` 是模型摘要 dict，`output_path` 是 Markdown 报告路径。
    - 输出是什么: 无返回；副作用是写入 `outputs/reports/A01_model_inspect_report.md`。
    - 如何验证: 打开报告，确认 `nq/nv/nu` 和对象名称清单完整。
    """
    if not isinstance(summary, dict):
        raise TypeError(f"summary must be dict, got {type(summary)!r}")
    if not isinstance(output_path, Path):
        raise TypeError(f"output_path must be Path, got {type(output_path)!r}")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    lines = []

    lines.append("# A01 Model Inspect Report")
    lines.append("")
    lines.append("## Model Dimensions")
    lines.append(f"- nq: {summary.get('nq')}")
    lines.append(f"- nv: {summary.get('nv')}")
    lines.append(f"- nu: {summary.get('nu')}")
    lines.append("")

    lines.append("## Joint Names")
    for name in summary.get("joint_names", []):
        lines.append(f"- {name}")
    lines.append("")

    lines.append("## Body Names")
    for name in summary.get("body_names", []):
        lines.append(f"- {name}")
    lines.append("")

    lines.append("## Site Names")
    for name in summary.get("site_names", []):
        lines.append(f"- {name}")
    lines.append("")

    lines.append("## Actuator Names")
    for name in summary.get("actuator_names", []):
        lines.append(f"- {name}")
    lines.append("")

    lines.append("## Keyframe Names")
    for name in summary.get("keyframe_names", []):
        lines.append(f"- {name}")
    lines.append("")

    lines.append("## End Effector Candidates")
    end_effector_check = summary.get("end_effector_check", {})
    for name, exists in end_effector_check.items():
        lines.append(f"- {name}: {exists}")
    lines.append("")

    report_text = "\n".join(lines)

    output_path.write_text(report_text, encoding="utf-8")
