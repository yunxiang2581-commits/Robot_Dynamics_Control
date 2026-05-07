"""A04 DLS IK thin CLI wrapper.

A04 归入 IK interface：
- wrapper 定位：``solver_type=dls`` 的无约束 IK 学习入口。
- 当前只保留函数级 TODO learning skeleton。
- 完整公式、符号表、验证标准和常见错误见
  ``projects/A_self_baseline/docs/A04_DLS_IK_TODO_full_plan.md``。

当前不运行完整 DLS，不生成 outputs，不调用 mink，不启动 viewer，不写
``data.ctrl``。
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
from typing import Any


A_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = A_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from robot_baseline.motion_types import IkRequest  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A04 wrapper: DLS IK via unified IK interface.")
    parser.add_argument("--robot-config", default=str(A_ROOT / "configs" / "robot.yaml"))
    parser.add_argument("--motion-task-config", default=str(A_ROOT / "configs" / "motion_task.yaml"))
    parser.add_argument("--a02-pose", default=str(A_ROOT / "outputs" / "cache" / "A02_site_pose.json"))
    parser.add_argument("--site", default="attachment_site")
    parser.add_argument("--task-mode", choices=["position", "pose_6d"], default="position")
    parser.add_argument("--target-position-offset", default="0.03,0.00,0.00")
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--gain", type=float, default=1.0)
    parser.add_argument("--damping", type=float, default=1.0e-3)
    parser.add_argument("--max-iters", type=int, default=100)
    parser.add_argument("--tolerance", type=float, default=1.0e-3)
    parser.add_argument("--output-dir", default=str(A_ROOT / "outputs"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def build_dls_request(args: argparse.Namespace) -> IkRequest:
    """构造 A04 DLS IkRequest 的最小骨架。

    TODO:
    - 要实现什么：从 CLI、``motion_task.yaml`` 和 A02 pose 规划 IkRequest。
    - 为什么需要：A04 wrapper 只负责入口组织，DLS 算法应进入 IK interface。
    - 对标 mink：把 FrameTask target 和 solver config 组织好，再进入 solve_ik。
    - 输入：CLI args、未来配置文件和 TargetDefinition。
    - 输出：``IkRequest(solver_type="dls")``。
    - 推荐 API：``model_loader.load_yaml_config``、``target_interface``。
    - 验证标准：solver_type 固定为 dls；task_mode 支持 position / pose_6d。
    """
    return IkRequest(
        solver_type="dls",
        task_mode=args.task_mode,
        site_name=args.site,
        weights={"position": 1.0, "orientation": 0.2},
        solver_config={
            "dt": args.dt,
            "gain": args.gain,
            "damping": args.damping,
            "max_iters": args.max_iters,
            "tolerance": args.tolerance,
        },
        metadata={
            "robot_config": args.robot_config,
            "motion_task_config": args.motion_task_config,
            "a02_pose": args.a02_pose,
            "target_position_offset": args.target_position_offset,
        },
    )


def load_dls_inputs(args: argparse.Namespace) -> dict[str, Any]:
    """读取 A04 前置输入。

    TODO:
    - 要实现什么：读取 ``robot.yaml``、``motion_task.yaml``、A01/A02/A03 cache。
    - 为什么需要：DLS 依赖模型维度、site pose 和 A03 Jacobian 验证结果。
    - 对标 mink：创建 Configuration / FrameTask 前先确认 model 和 target frame。
    - 输入：CLI args 中的配置和 cache 路径。
    - 输出：包含 nq/nv/nu、site、Jacobian 可用性的上下文 dict。
    - 推荐 API：``Path.read_text``、``json.loads``、``model_loader.load_yaml_config``。
    - 验证标准：nq/nv/nu 与 A01 一致；site 为 ``attachment_site``；A03 Jacobian 可用。
    """
    raise NotImplementedError("TODO A04: 读取 DLS 前置输入；当前只保留函数级骨架。")


def resolve_dls_target(args: argparse.Namespace) -> Any:
    """解析 A04 TargetDefinition。

    TODO:
    - 要实现什么：支持 ``offset_from_current``、``fixed_pose``、``target_definition_json``。
    - 为什么需要：A04 不再私有生成 target，应复用统一 TargetDefinition。
    - 对标 mink：FrameTask target。
    - 输入：A02 current pose、CLI target、A06 target definition。
    - 输出：TargetDefinition。
    - 推荐 API：``target_interface.load_target_definition``。
    - 验证标准：position shape=(3,)，rotation_matrix shape=(3,3)，quat 使用 wxyz。
    """
    raise NotImplementedError("TODO A04: 解析 DLS target；当前不生成真实 TargetDefinition。")


def run_dls_trajectory(request: IkRequest) -> Any:
    """运行 DLS trajectory。

    TODO:
    - 要实现什么：未来调用 ``ik_interface.solve_ik_trajectory``。
    - 为什么需要：把 position-only / pose_6d DLS 从 wrapper 迁到 IK interface。
    - 对标 mink：无约束教学版 solve_ik。
    - 输入：IkRequest。
    - 输出：IkResult。
    - 推荐 API：``ik_interface.solve_ik_trajectory``。
    - 验证标准：position_error_norm 下降；pose_6d 下 orientation_error_norm 下降。
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
    - 验证标准：q_traj shape=(N,nq)，日志字段稳定。
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

    logging.info("A04 DLS wrapper: function-level TODO skeleton.")
    logging.info("solver_type=%s, task_mode=%s, site=%s", request.solver_type, request.task_mode, request.site_name)
    logging.info("planned_outputs=%s", planned_outputs)

    raise NotImplementedError("A04 wrapper TODO: 当前只建立 IkRequest，不运行完整 DLS。")


if __name__ == "__main__":
    main()
