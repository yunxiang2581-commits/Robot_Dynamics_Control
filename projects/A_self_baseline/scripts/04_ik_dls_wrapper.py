"""A04 DLS IK thin CLI wrapper.

A04 归入 IK interface：
- wrapper 定位：``solver_type="dls"`` 的无约束 DLS IK 学习入口。
- R2 当前只完成 ``IkRequest`` 的配置读取和字段规划。
- 不运行完整 DLS，不生成 outputs，不调用 mink，不启动 viewer，不写 ``data.ctrl``。

学习边界：
- 输入：``motion_task.yaml``、可选 ``TargetDefinition``、少量 CLI 覆盖。
- 输出：内存中的 ``IkRequest(solver_type="dls")``。
- 数学逻辑：本步不改变 DLS 公式，只准备后续 R4 要用的参数。
- 风险：A04 必须强制使用 ``solver_type="dls"``，不能照抄配置里给 A05 用的 QP solver。
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml


A_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = A_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from robot_baseline.motion_types import IkRequest, TargetDefinition  # noqa: E402
from robot_baseline.target_interface import (  # noqa: E402
    load_target_definition,
    parse_rotation_matrix_optional,
    parse_vector_optional,
    quat_wxyz_to_rotation_matrix,
    rotation_matrix_to_quat_wxyz,
    validate_target_definition,
)


def _resolve_a_project_path(path_value: str | Path) -> Path:
    """把 A 项目配置中的路径解析为稳定 ``Path``。

    输入：
    - 绝对路径：直接使用；
    - 相对路径：默认相对于 ``projects/A_self_baseline``。

    输出：
    - 可用于读取配置、target JSON 或输出规划的路径。
    """
    path = Path(path_value).expanduser()
    if path.is_absolute():
        return path
    return A_ROOT / path


def parse_args() -> argparse.Namespace:
    """解析 A04 wrapper 的少量入口参数。

    R2 的长期结构是：默认实验参数来自 ``motion_task.yaml``，CLI 只用于临时覆盖。
    这些参数不会触发 IK 求解，只影响 ``IkRequest`` 的字段。
    """
    parser = argparse.ArgumentParser(description="A04 wrapper: DLS IK via unified IK interface.")
    parser.add_argument("--robot-config", default=str(A_ROOT / "configs" / "robot.yaml"))
    parser.add_argument("--motion-task-config", default=str(A_ROOT / "configs" / "motion_task.yaml"))
    parser.add_argument("--target-definition", default=None)
    parser.add_argument("--a02-pose", default=str(A_ROOT / "outputs" / "cache" / "A02_site_pose.json"))
    parser.add_argument("--site", default=None)
    parser.add_argument("--task-mode", choices=["position", "pose_6d"], default=None)
    parser.add_argument("--target-position-offset", default=None)
    parser.add_argument("--dt", type=float, default=None)
    parser.add_argument("--gain", type=float, default=None)
    parser.add_argument("--damping", type=float, default=None)
    parser.add_argument("--max-iters", type=int, default=None)
    parser.add_argument("--tolerance", type=float, default=None)
    parser.add_argument("--output-dir", default=str(A_ROOT / "outputs"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def load_dls_motion_task_config(args: argparse.Namespace) -> dict[str, Any]:
    """读取 A04/A05 共用的 ``motion_task.yaml``。

    要实现什么：
    - 读取 ``args.motion_task_config``；
    - 检查顶层字段 ``target``、``ik``、``limits`` 是否存在；
    - 本函数只负责读取和基本结构检查，不处理 CLI 覆盖。

    为什么需要：
    - A04 不应该把 dt/gain/damping/target offset 长期私有放在 CLI 中；
    - R2 之后，默认参数来自统一配置，wrapper 只负责入口组织。

    对标 mink：
    - mink 先组织 FrameTask 和 solver 参数，再进入 solve_ik；
    - 本项目不调用 mink，但同样需要先把 task config 组织清楚。

    输入：
    - ``args.motion_task_config``。

    输出：
    - 配置 dict，后续用于构造 ``IkRequest(solver_type="dls")``。

    推荐 API：
    - ``Path.read_text``；
    - ``yaml.safe_load``。

    验证：
    - config 是 dict；
    - ``target``、``ik``、``limits`` 都是 mapping。
    """
    config_path = Path(args.motion_task_config).expanduser()
    if not config_path.exists():
        raise FileNotFoundError(f"motion_task config not found: {config_path}")

    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError(f"motion_task config must contain a YAML mapping, got {type(config)!r}")

    for section in ("target", "ik", "limits"):
        if section not in config:
            raise KeyError(f"motion_task.yaml missing required section: {section}")
        if not isinstance(config[section], dict):
            raise ValueError(f"motion_task.yaml section {section!r} must be a mapping")

    return config


def plan_dls_request_fields(config: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    """从配置规划 A04 DLS ``IkRequest`` 字段。

    要实现什么：
    - 从 ``config["target"]`` 读取 site / target source；
    - 从 ``config["ik"]`` 读取 task_mode、weights、solver_config；
    - 强制 ``solver_type="dls"``；
    - 只规划字段，不运行 DLS。

    为什么需要：
    - ``IkRequest`` 是 A04/A05 进入 IK interface 的统一入口；
    - R2 先让 request 结构稳定，R3/R4 再补数学和求解。

    对标 mink：
    - FrameTask 由 target/site/task_mode 决定；
    - solve_ik config 由 gain/dt/damping 等决定。

    输入：
    - motion_task config；
    - CLI args，仅用于 metadata。

    输出：
    - 可传给 ``IkRequest`` 的字段 dict。

    验证：
    - ``solver_type == "dls"``；
    - ``task_mode`` 属于 ``position`` / ``pose_6d``；
    - ``site_name`` 非空；
    - ``damping > 0``。

    常见错误：
    - 在 A04 中读取 velocity_limit / position_margin 并误以为 DLS 支持约束；
    - 把 ``motion_task.yaml`` 中给 A05 的 ``qp_scipy`` 当成 A04 solver。
    """
    target_config = config["target"]
    ik_config = config["ik"]

    task_mode = ik_config.get("task_mode", "position")
    if task_mode not in ("position", "pose_6d"):
        raise ValueError(f"ik.task_mode must be position or pose_6d, got {task_mode!r}")

    site_name = target_config.get("site_name")
    if not isinstance(site_name, str) or not site_name.strip():
        raise ValueError("target.site_name must be a non-empty string")

    weights = {
        "position": float(ik_config.get("position_weight", 1.0)),
        "orientation": float(ik_config.get("orientation_weight", 0.0)),
    }
    if weights["position"] <= 0.0:
        raise ValueError("ik.position_weight must be positive")
    if weights["orientation"] < 0.0:
        raise ValueError("ik.orientation_weight must be non-negative")

    # A04 的 DLS 使用 lambda 作为阻尼。统一配置暂时没有 damping 字段时，
    # 复用 regularization 作为教学版默认值，后续可在 motion_task.yaml 中单独加 damping。
    damping = ik_config.get("damping", ik_config.get("regularization", 1.0e-3))
    solver_config = {
        "dt": float(ik_config.get("dt", 0.02)),
        "gain": float(ik_config.get("gain", 1.0)),
        "damping": float(damping),
        "max_iters": int(ik_config.get("max_iters", 100)),
        "tolerance": float(ik_config.get("tolerance", 1.0e-3)),
    }
    if solver_config["dt"] <= 0.0:
        raise ValueError("ik.dt must be positive")
    if solver_config["gain"] <= 0.0:
        raise ValueError("ik.gain must be positive")
    if solver_config["damping"] <= 0.0:
        raise ValueError("ik.damping or ik.regularization must be positive for DLS")
    if solver_config["max_iters"] <= 0:
        raise ValueError("ik.max_iters must be positive")
    if solver_config["tolerance"] <= 0.0:
        raise ValueError("ik.tolerance must be positive")

    target_definition_path = target_config.get("target_definition")
    target_definition = None
    if target_definition_path:
        path = _resolve_a_project_path(target_definition_path)
        target_definition = load_target_definition(path)
        target_definition_path = str(path)

    return {
        "solver_type": "dls",
        "task_mode": task_mode,
        "site_name": site_name,
        "target": target_definition,
        "weights": weights,
        "limits": {},
        "solver_config": solver_config,
        "metadata": {
            "robot_config": args.robot_config,
            "motion_task_config": args.motion_task_config,
            "a02_pose": args.a02_pose,
            "target_source": target_config.get("source"),
            "target_definition": target_definition_path,
            "target_body": target_config.get("body_name"),
            "target_position_offset": target_config.get("position_offset"),
            "note": "A04 request fields are planned from motion_task.yaml; DLS solver is not executed in R2.",
        },
    }


def apply_dls_cli_overrides(request_fields: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    """应用 A04 CLI 覆盖，但不运行 IK。

    要实现什么：
    - ``--site`` 覆盖 target site；
    - ``--task-mode`` 覆盖 task mode；
    - ``--target-definition`` 覆盖配置中的 target definition；
    - ``--dt`` / ``--gain`` / ``--damping`` 等只作为临时实验覆盖。

    为什么需要：
    - 默认实验应固定在 ``motion_task.yaml``；
    - CLI 只用于短期对照，不能改变 A04 的 solver 边界。

    输入：
    - request_fields；
    - CLI args。

    输出：
    - 覆盖后的 request_fields。

    验证：
    - ``solver_type`` 始终是 ``dls``；
    - 覆盖后的字段仍能构造合法 ``IkRequest``。
    """
    fields = dict(request_fields)
    metadata = dict(fields.get("metadata", {}))
    solver_config = dict(fields.get("solver_config", {}))

    if args.site is not None:
        if not args.site.strip():
            raise ValueError("--site must be a non-empty string")
        fields["site_name"] = args.site

    if args.task_mode is not None:
        if args.task_mode not in ("position", "pose_6d"):
            raise ValueError(f"task_mode must be position or pose_6d, got {args.task_mode!r}")
        fields["task_mode"] = args.task_mode

    if args.target_definition is not None:
        path = _resolve_a_project_path(args.target_definition)
        fields["target"] = load_target_definition(path)
        metadata["target_definition"] = str(path)
        metadata["target_source"] = "target_definition"

    if args.target_position_offset is not None:
        metadata["target_position_offset"] = args.target_position_offset

    if args.dt is not None:
        if args.dt <= 0.0:
            raise ValueError("--dt must be positive")
        solver_config["dt"] = args.dt

    if args.gain is not None:
        if args.gain <= 0.0:
            raise ValueError("--gain must be positive")
        solver_config["gain"] = args.gain

    if args.damping is not None:
        if args.damping <= 0.0:
            raise ValueError("--damping must be positive")
        solver_config["damping"] = args.damping

    if args.max_iters is not None:
        if args.max_iters <= 0:
            raise ValueError("--max-iters must be positive")
        solver_config["max_iters"] = args.max_iters

    if args.tolerance is not None:
        if args.tolerance <= 0.0:
            raise ValueError("--tolerance must be positive")
        solver_config["tolerance"] = args.tolerance

    fields["solver_type"] = "dls"
    fields["solver_config"] = solver_config
    fields["metadata"] = metadata
    return fields


def build_dls_request(args: argparse.Namespace) -> IkRequest:
    """构造 A04 DLS ``IkRequest``。

    当前 R2 已实现：
    - 读取 ``motion_task.yaml``；
    - 规划 request 字段；
    - 应用少量 CLI 覆盖；
    - 返回 ``IkRequest(solver_type="dls")``。

    当前 R2 不实现：
    - 不加载 MuJoCo model；
    - 不计算 Jacobian；
    - 不执行 DLS；
    - 不生成 q_traj。
    """
    config = load_dls_motion_task_config(args)
    request_fields = plan_dls_request_fields(config, args)
    request_fields = apply_dls_cli_overrides(request_fields, args)
    return IkRequest(**request_fields)


def load_dls_inputs(args: argparse.Namespace) -> dict[str, Any]:
    """读取 A04 前置输入。

    TODO:
    - 要实现什么：读取 ``robot.yaml``、``motion_task.yaml``、A01/A02/A03 cache。
    - 为什么需要：DLS 依赖模型维度、site pose 和 A03 Jacobian 验证结果。
    - 对标 mink：创建 Configuration / FrameTask 前先确认 model 和 target frame。
    - 输入：CLI args 中的配置和 cache 路径。
    - 输出：包含 nq/nv/nu、site、Jacobian 可用性的上下文 dict。
    - 推荐 API：``Path.read_text``、``json.loads``。
    - 验证：nq/nv/nu 与 A01 一致；site 为 ``attachment_site``；A03 Jacobian 可用。
    """
    raise NotImplementedError("TODO A04: 读取 DLS 前置输入；当前只保留函数级骨架。")


def _resolve_dls_target_legacy_todo(args: argparse.Namespace) -> Any:
    """解析 A04 TargetDefinition。

    TODO:
    - 要实现什么：支持 ``offset_from_current``、``fixed_pose``、``target_definition_json``。
    - 为什么需要：A04 不再私有生成 target，应复用统一 TargetDefinition。
    - 对标 mink：FrameTask target。
    - 输入：A02 current pose、CLI target、A06 target definition。
    - 输出：TargetDefinition。
    - 推荐 API：``target_interface.load_target_definition``。
    - 验证：position shape=(3,)，rotation_matrix shape=(3,3)，quat 使用 wxyz。
    """
    raise NotImplementedError("TODO A04: 解析 DLS target；当前不生成真实 TargetDefinition。")


def _rpy_xyz_to_rotation_matrix(rpy: np.ndarray) -> np.ndarray:
    """把 xyz RPY 转成 rotation matrix。

    输入单位固定为 rad。这里采用常见的外部 xyz RPY 约定：
    ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``。
    """
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    return rz @ ry @ rx


def resolve_dls_target(
    args: argparse.Namespace,
    current_position: np.ndarray,
    current_rotation: np.ndarray,
) -> TargetDefinition:
    """解析 A04 DLS 要追踪的 target pose。

    本函数只负责把配置或 JSON 转成 ``TargetDefinition``。

    输入：
    - ``args``：A04 CLI 参数；
    - ``current_position``：当前 site 位置，shape=(3,)，单位 m；
    - ``current_rotation``：当前 site 姿态，shape=(3,3)。

    输出：
    - ``TargetDefinition``。

    支持：
    - ``--target-definition``：读取 A06 或手写 target JSON；
    - ``source=offset_from_current``；
    - ``source=fixed_pose``；
    - ``orientation_mode=keep_current``；
    - ``orientation_mode=fixed_quat``，四元数顺序为 wxyz；
    - ``orientation_mode=fixed_rpy``，RPY 单位为 rad。

    边界：
    - 不加载 MuJoCo model；
    - 不计算 Jacobian；
    - 不运行 DLS；
    - 不生成 A06_target_definition.json。
    """
    current_position_vec = parse_vector_optional(current_position, 3, "current_position")
    current_rotation_mat = parse_rotation_matrix_optional(current_rotation, "current_rotation")
    if current_position_vec is None or current_rotation_mat is None:
        raise ValueError("current_position and current_rotation cannot be None")

    config = load_dls_motion_task_config(args)
    target_config = config["target"]

    expected_site = args.site or target_config.get("site_name", "attachment_site")
    if not isinstance(expected_site, str) or not expected_site.strip():
        raise ValueError("target.site_name must be a non-empty string")

    if args.target_definition is not None:
        path = _resolve_a_project_path(args.target_definition)
        target = load_target_definition(path)
        if target.target_site != expected_site:
            raise ValueError(
                f"target_definition site mismatch: expected {expected_site!r}, got {target.target_site!r}"
            )
        if target.rotation_matrix is None and target.quat_wxyz is not None:
            target.rotation_matrix = quat_wxyz_to_rotation_matrix(target.quat_wxyz)
        if target.quat_wxyz is None and target.rotation_matrix is not None:
            target.quat_wxyz = rotation_matrix_to_quat_wxyz(target.rotation_matrix)
        validate_target_definition(target)
        return target

    source = target_config.get("source", "offset_from_current")
    site_name = target_config.get("site_name", expected_site)
    body_name = target_config.get("body_name")
    coordinate_frame = target_config.get("coordinate_frame", "world")
    orientation_mode = target_config.get("orientation_mode", "keep_current")

    offset_value = args.target_position_offset if args.target_position_offset is not None else target_config.get("position_offset")
    if source == "offset_from_current":
        offset = parse_vector_optional(offset_value, 3, "position_offset")
        if offset is None:
            raise ValueError("offset_from_current requires target.position_offset")
        position = current_position_vec + offset
        position_offset = offset
        target_id = "A04_offset_target"
    elif source == "fixed_pose":
        position = parse_vector_optional(target_config.get("position"), 3, "position")
        if position is None:
            raise ValueError("fixed_pose requires target.position")
        position_offset = None
        target_id = "A04_fixed_target"
    else:
        raise ValueError(f"unsupported A04 target source: {source!r}")

    if orientation_mode == "keep_current":
        rotation_matrix = current_rotation_mat
        quat_wxyz = rotation_matrix_to_quat_wxyz(rotation_matrix)
    elif orientation_mode == "fixed_quat":
        quat_wxyz = parse_vector_optional(target_config.get("quat_wxyz"), 4, "quat_wxyz")
        if quat_wxyz is None:
            raise ValueError("fixed_quat requires target.quat_wxyz")
        rotation_matrix = quat_wxyz_to_rotation_matrix(quat_wxyz)
    elif orientation_mode == "fixed_rpy":
        rpy = parse_vector_optional(target_config.get("rpy"), 3, "rpy")
        if rpy is None:
            raise ValueError("fixed_rpy requires target.rpy")
        rotation_matrix = _rpy_xyz_to_rotation_matrix(rpy)
        quat_wxyz = rotation_matrix_to_quat_wxyz(rotation_matrix)
    else:
        raise ValueError(f"unsupported target.orientation_mode: {orientation_mode!r}")

    target = TargetDefinition(
        target_id=target_id,
        source=source,
        target_site=site_name,
        target_body=body_name,
        coordinate_frame=coordinate_frame,
        position=position,
        rotation_matrix=rotation_matrix,
        quat_wxyz=quat_wxyz,
        position_offset=position_offset,
        orientation_mode=orientation_mode,
        metadata={
            "created_by": "A04.resolve_dls_target",
            "motion_task_config": args.motion_task_config,
            "note": "A04 consumes target pose; it does not generate A06 target outputs.",
        },
    )
    validate_target_definition(target)
    return target


def run_dls_trajectory(request: IkRequest) -> Any:
    """运行 DLS trajectory。

    TODO:
    - 要实现什么：未来调用 ``ik_interface.solve_ik_trajectory``。
    - 为什么需要：把 position-only / pose_6d DLS 从 wrapper 迁到 IK interface。
    - 对标 mink：无约束教学版 solve_ik。
    - 输入：IkRequest。
    - 输出：IkResult。
    - 推荐 API：``ik_interface.solve_ik_trajectory``。
    - 验证：position_error_norm 下降；pose_6d 中 orientation_error_norm 下降。
    """
    raise NotImplementedError("TODO A04: 运行 DLS trajectory；当前不执行 IK。")


def write_dls_outputs(result: Any, output_paths: dict[str, Path]) -> None:
    """写 A04 输出。

    TODO:
    - 要实现什么：保存 q_traj、error log、figure、report。
    - 为什么需要：IK 结果需要可复盘，并给后续 A07 提供 trajectory_source。
    - 对标 mink：viewer 实时结果在本项目中落成文件。
    - 输入：IkResult 和输出路径。
    - 输出：``A04_dls_ik_q_traj.npy``、CSV、PNG、Markdown。
    - 推荐 API：``numpy.save``、``csv.DictWriter``、``matplotlib``、``Path.write_text``。
    - 验证：q_traj shape=(N,nq)，日志字段稳定。
    """
    raise NotImplementedError("TODO A04: 写 DLS 输出；当前不生成 outputs。")


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO), format="%(levelname)s: %(message)s")

    output_root = Path(args.output_dir)
    if not output_root.is_absolute():
        output_root = A_ROOT / output_root

    planned_outputs = {
        "trajectory": output_root / "trajectories" / "A04_dls_ik_q_traj.npy",
        "error_log": output_root / "logs" / "A04_dls_ik_error.csv",
        "error_figure": output_root / "figures" / "A04_dls_ik_error.png",
        "report": output_root / "reports" / "A04_dls_ik_report.md",
    }
    request = build_dls_request(args)

    logging.info("A04 DLS wrapper: R2 IkRequest config reading is implemented.")
    logging.info("solver_type=%s, task_mode=%s, site=%s", request.solver_type, request.task_mode, request.site_name)
    logging.info("planned_outputs=%s", planned_outputs)

    raise NotImplementedError("A04 wrapper TODO: 当前只建立 IkRequest，不运行完整 DLS。")


if __name__ == "__main__":
    main()
