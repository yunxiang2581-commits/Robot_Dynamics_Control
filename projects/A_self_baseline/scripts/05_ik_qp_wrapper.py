"""A05 QP-IK thin CLI wrapper.

A05 归入 IK interface：
- wrapper 定位：``solver_type=qp_scipy`` / ``qp_osqp`` 的 QP-IK 学习入口。
- 当前只保留函数级 TODO learning skeleton。
- 完整公式、符号表、验证标准和常见错误见
  ``projects/A_self_baseline/docs/A05_QP_IK_TODO_full_plan.md``。

当前不运行完整 QP-IK，不生成 outputs，不调用 mink，不启动 viewer，不写
``data.ctrl``。
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
from typing import Any

import yaml


A_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = A_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from robot_baseline.motion_types import IkRequest  # noqa: E402
from robot_baseline.target_interface import load_target_definition  # noqa: E402


def _resolve_a_project_path(path_value: str | Path) -> Path:
    """把 A 项目配置里的路径解析为稳定 Path。

    优先规则：
    - 绝对路径直接使用；
    - 相对路径先按 A 项目根目录解析。
    """
    path = Path(path_value).expanduser()
    if path.is_absolute():
        return path
    return A_ROOT / path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A05 wrapper: QP-IK via unified IK interface.")
    parser.add_argument("--motion-task-config", default=str(A_ROOT / "configs" / "motion_task.yaml"))
    parser.add_argument("--target-definition", default=None)
    parser.add_argument("--solver-type", choices=["qp_scipy", "qp_osqp"], default=None)
    parser.add_argument("--task-mode", choices=["position", "pose_6d"], default=None)
    parser.add_argument("--output-dir", default=str(A_ROOT / "outputs"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def load_qp_motion_task_config(args: argparse.Namespace) -> dict[str, Any]:
    """TODO R2: 读取 A05 QP-IK 的统一配置。

    要实现什么：
    - 读取 ``args.motion_task_config``；
    - 检查顶层字段 ``target``、``ik``、``limits``；
    - 不在本函数处理 CLI 覆盖，覆盖统一交给 ``apply_qp_cli_overrides``。

    为什么需要：
    A05 的 QP-IK 参数较多，不能全部堆在 ``parse_args``。R2 以后，默认值来自
    ``motion_task.yaml``，CLI 只保留 solver_type / task_mode / target_definition
    这类高层覆盖。

    对标 mink：
    mink 的完整链路是 target -> FrameTask / PostureTask -> limits -> solve_ik。
    本项目不调用 mink，但 A05 需要先把这些配置组织成 ``IkRequest``。

    输入：
    - ``args.motion_task_config``；
    - 本函数只读取 ``args.motion_task_config``。

    输出：
    - 配置 dict，后续用于构造 ``IkRequest(solver_type="qp_scipy"|"qp_osqp")``。

    推荐 API：
    - ``Path.read_text``；
    - ``yaml.safe_load`` 或项目已有 ``model_loader.load_yaml_config``。

    验证标准：
    - ``ik.solver_type`` 属于 ``qp_scipy`` / ``qp_osqp``；
    - ``ik.task_mode`` 属于 ``position`` / ``pose_6d``；
    - ``limits.velocity_limit > 0``；
    - ``target.site_name`` 非空。

    常见错误：
    - OSQP 里的 q 变量和机器人 q 混淆；
    - YAML 和 CLI 默认值冲突。
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


def plan_qp_request_fields(config: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    """TODO R2: 从配置规划 QP IkRequest 字段。

    要实现什么：
    - 从 ``config["target"]`` 读取 site / target source；
    - 从 ``config["ik"]`` 读取 solver_type、task_mode、weights、solver_config；
    - 从 ``config["limits"]`` 读取 velocity_limit、position_margin；
    - 只规划 request 字段，不构造 QP，不求解。

    为什么需要：
    A05 后续的 FrameTask / PostureTask / limits 都依赖同一个 ``IkRequest``。
    先让 request 字段稳定，后续 R3/R5 再补数学小函数和 trajectory loop。

    对标 mink：
    - FrameTask: target pose feeds IK；
    - PostureTask: q_ref 正则；
    - VelocityLimit / ConfigurationLimit: QP bounds；
    - solve_ik: solver backend。

    输入：
    - motion_task config；
    - CLI args。

    输出：
    - 可传给 ``IkRequest`` 的字段 dict。

    推荐 API：
    - ``target_interface.load_target_definition``；
    - ``target_interface.validate_target_definition``；
    - 普通 dict 读取和显式 ValueError。

    验证标准：
    - ``weights`` 包含 position/orientation/posture；
    - ``limits`` 包含 velocity_limit/position_margin；
    - ``solver_config`` 包含 dt/gain/regularization/max_iters/tolerance；
    - 若提供 target_definition，应能通过 R1 的 load/validate。

    常见错误：
    - 把 target 仍然写死成 current + offset；
    - 忘记把 A06 target JSON 作为优先 target 来源；
    - 在 wrapper 里直接开始写 QP objective。
    """
    target_config = config["target"]
    ik_config = config["ik"]
    limits_config = config["limits"]

    solver_type = ik_config.get("solver_type", "qp_scipy")
    if solver_type not in ("qp_scipy", "qp_osqp"):
        raise ValueError(f"ik.solver_type must be qp_scipy or qp_osqp, got {solver_type!r}")

    task_mode = ik_config.get("task_mode", "position")
    if task_mode not in ("position", "pose_6d"):
        raise ValueError(f"ik.task_mode must be position or pose_6d, got {task_mode!r}")

    site_name = target_config.get("site_name")
    if not isinstance(site_name, str) or not site_name.strip():
        raise ValueError("target.site_name must be a non-empty string")

    weights = {
        "position": float(ik_config.get("position_weight", 1.0)),
        "orientation": float(ik_config.get("orientation_weight", 0.0)),
        "posture": float(ik_config.get("posture_weight", 0.0)),
    }
    if weights["position"] <= 0.0:
        raise ValueError("ik.position_weight must be positive")
    if weights["orientation"] < 0.0:
        raise ValueError("ik.orientation_weight must be non-negative")
    if weights["posture"] < 0.0:
        raise ValueError("ik.posture_weight must be non-negative")

    solver_config = {
        "dt": float(ik_config.get("dt", 0.02)),
        "gain": float(ik_config.get("gain", 1.0)),
        "regularization": float(ik_config.get("regularization", 1.0e-4)),
        "max_iters": int(ik_config.get("max_iters", 100)),
        "tolerance": float(ik_config.get("tolerance", 1.0e-3)),
    }
    if solver_config["dt"] <= 0.0:
        raise ValueError("ik.dt must be positive")
    if solver_config["gain"] <= 0.0:
        raise ValueError("ik.gain must be positive")
    if solver_config["regularization"] < 0.0:
        raise ValueError("ik.regularization must be non-negative")
    if solver_config["max_iters"] <= 0:
        raise ValueError("ik.max_iters must be positive")
    if solver_config["tolerance"] <= 0.0:
        raise ValueError("ik.tolerance must be positive")

    limits = {
        "velocity_limit": float(limits_config.get("velocity_limit", 0.5)),
        "position_margin": float(limits_config.get("position_margin", 0.05)),
    }
    if limits["velocity_limit"] <= 0.0:
        raise ValueError("limits.velocity_limit must be positive")
    if limits["position_margin"] < 0.0:
        raise ValueError("limits.position_margin must be non-negative")

    target_definition_path = target_config.get("target_definition")
    target_definition = None
    if target_definition_path:
        path = _resolve_a_project_path(target_definition_path)
        target_definition = load_target_definition(path)
        target_definition_path = str(path)

    return {
        "solver_type": solver_type,
        "task_mode": task_mode,
        "site_name": site_name,
        "target": target_definition,
        "weights": weights,
        "limits": limits,
        "solver_config": solver_config,
        "metadata": {
            "motion_task_config": args.motion_task_config,
            "target_source": target_config.get("source"),
            "target_definition": target_definition_path,
            "target_body": target_config.get("body_name"),
            "note": "A05 request fields are planned from motion_task.yaml; QP solver is not executed in R2.",
        },
    }


def apply_qp_cli_overrides(request_fields: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    """TODO R2: 应用 A05 CLI 少量覆盖。

    要实现什么：
    - ``--solver-type`` 覆盖 config 中的 solver_type；
    - ``--task-mode`` 覆盖 config 中的 task_mode；
    - ``--target-definition`` 覆盖 config 中的 target.target_definition；
    - 不覆盖 dt/gain/limits 等低层数值参数。

    为什么需要：
    CLI 只做临时实验入口，默认实验配置应固定在 ``motion_task.yaml``，便于复盘。

    输入：
    - request_fields；
    - CLI args。

    输出：
    - 覆盖后的 request_fields。

    验证标准：
    - 覆盖后 solver_type 仍在 ``qp_scipy`` / ``qp_osqp``；
    - 覆盖后 task_mode 仍在 ``position`` / ``pose_6d``；
    - 不写 ``data.ctrl``，不运行 solver。
    """
    fields = dict(request_fields)
    metadata = dict(fields.get("metadata", {}))

    if args.solver_type is not None:
        if args.solver_type not in ("qp_scipy", "qp_osqp"):
            raise ValueError(f"solver_type must be qp_scipy or qp_osqp, got {args.solver_type!r}")
        fields["solver_type"] = args.solver_type

    if args.task_mode is not None:
        if args.task_mode not in ("position", "pose_6d"):
            raise ValueError(f"task_mode must be position or pose_6d, got {args.task_mode!r}")
        fields["task_mode"] = args.task_mode

    if args.target_definition is not None:
        path = _resolve_a_project_path(args.target_definition)
        fields["target"] = load_target_definition(path)
        metadata["target_definition"] = str(path)
        metadata["target_source"] = "target_definition"

    fields["metadata"] = metadata
    return fields


def build_qp_request(args: argparse.Namespace) -> IkRequest:
    """构造 A05 QP-IK request。

    TODO:
    - 要实现什么：构造支持 ``qp_scipy`` / ``qp_osqp`` 的 IkRequest。
    - 为什么需要：wrapper 只负责 CLI 和配置入口，QP 细节应进入 IK interface。
    - 对标 mink：FrameTask / PostureTask / limits 进入 solve_ik。
    - 输入：CLI args、motion_task.yaml、TargetDefinition。
    - 输出：IkRequest。
    - 推荐 API：``target_interface``、``ik_interface``。
    - 验证标准：solver_type 合法；task_mode 支持 position / pose_6d。
    """
    config = load_qp_motion_task_config(args)
    request_fields = plan_qp_request_fields(config, args)
    request_fields = apply_qp_cli_overrides(request_fields, args)
    return IkRequest(**request_fields)


def load_qp_inputs(args: argparse.Namespace) -> dict[str, Any]:
    """读取 QP-IK 输入。

    TODO:
    - 要实现什么：读取 robot.yaml、motion_task.yaml、A01/A02/A03、target_definition。
    - 为什么需要：QP-IK 需要模型维度、site、当前 pose、Jacobian 前置验证和 target。
    - 对标 mink：Configuration + FrameTask + limits 创建前的上下文准备。
    - 输入：CLI args。
    - 输出：QP 上下文 dict。
    - 推荐 API：``Path.read_text``、``json.loads``、``model_loader.load_yaml_config``。
    - 验证标准：site 存在；nq/nv/nu 与 A01 一致。
    """
    raise NotImplementedError("TODO A05: 读取 QP 输入；当前只保留函数级骨架。")


def resolve_qp_target(args: argparse.Namespace) -> Any:
    """解析 QP target。

    TODO:
    - 要实现什么：支持 offset_from_current / fixed_pose / target_definition_json。
    - 为什么需要：A05 应消费统一 TargetDefinition，并规划 A05_target_definition.json。
    - 对标 mink：FrameTask target。
    - 输入：A02 pose、CLI target、A06 target definition。
    - 输出：TargetDefinition。
    - 推荐 API：``target_interface.load_target_definition``、``save_target_definition``。
    - 验证标准：target_position、target_rotation、target_site 完整。
    """
    raise NotImplementedError("TODO A05: 解析 QP target；当前不写 target definition。")


def build_qp_task_plan(request: IkRequest) -> dict[str, Any]:
    """规划 QP task stack。

    TODO:
    - 要实现什么：组合 FrameTask、PostureTask、VelocityLimit、JointPositionLimit。
    - 为什么需要：A05 的核心是任务与限制统一进入 QP。
    - 对标 mink：FrameTask / PostureTask / VelocityLimit / ConfigurationLimit。
    - 输入：IkRequest。
    - 输出：任务规划 dict，后续包含 J_task、v_task、bounds。
    - 推荐 API：``ik_interface.build_frame_task``、``build_posture_task``、``merge_bounds``。
    - 验证标准：J_task 列数为 nv；lower/upper shape=(nv,)。
    """
    raise NotImplementedError("TODO A05: 规划 QP task stack；当前不构造 H/c。")


def run_qp_trajectory(request: IkRequest) -> Any:
    """运行 QP-IK trajectory。

    TODO:
    - 要实现什么：未来调用 ``ik_interface.solve_ik_trajectory``。
    - 为什么需要：统一 SciPy / OSQP backend 和外层循环。
    - 对标 mink：solve_ik。
    - 输入：IkRequest。
    - 输出：IkResult。
    - 推荐 API：``ik_interface.solve_ik_trajectory``。
    - 验证标准：error 下降；max_constraint_violation 接近 0。
    """
    raise NotImplementedError("TODO A05: 运行 QP trajectory；当前不执行 solver。")


def write_qp_outputs(result: Any, output_paths: dict[str, Path]) -> None:
    """写 A05 输出。

    TODO:
    - 要实现什么：保存 target definition、q_traj、dq_traj、error log、constraint log、figure、report。
    - 为什么需要：A07 后续消费 q_traj，QP 调试需要 constraint log。
    - 对标 mink：实时 solve_ik 结果在本项目中落成可复盘文件。
    - 输入：IkResult 和输出路径。
    - 输出：A05 cache/trajectory/log/figure/report。
    - 推荐 API：``trajectory_io``、``csv.DictWriter``、``matplotlib``。
    - 验证标准：q_traj shape=(N,nq)，dq_traj shape=(N,nv)，constraint CSV 字段完整。
    """
    raise NotImplementedError("TODO A05: 写 QP 输出；当前不生成 outputs。")


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO), format="%(levelname)s: %(message)s")

    output_root = Path(args.output_dir)
    if not output_root.is_absolute():
        output_root = A_ROOT / output_root

    planned_outputs = {
        "target_definition": output_root / "cache" / "A05_target_definition.json",
        "trajectory": output_root / "trajectories" / "A05_qp_ik_q_traj.npy",
        "dq_trajectory": output_root / "trajectories" / "A05_qp_ik_dq_traj.npy",
        "error_log": output_root / "logs" / "A05_qp_ik_error.csv",
        "constraint_log": output_root / "logs" / "A05_qp_ik_constraints.csv",
        "error_figure": output_root / "figures" / "A05_qp_ik_error.png",
        "report": output_root / "reports" / "A05_qp_ik_report.md",
    }
    request = build_qp_request(args)

    logging.info("A05 QP wrapper: function-level TODO skeleton.")
    logging.info("solver_type=%s, task_mode=%s, site=%s", request.solver_type, request.task_mode, request.site_name)
    logging.info("planned_outputs=%s", planned_outputs)

    raise NotImplementedError("A05 wrapper TODO: 当前只建立 IkRequest，不运行完整 QP-IK。")


if __name__ == "__main__":
    main()
