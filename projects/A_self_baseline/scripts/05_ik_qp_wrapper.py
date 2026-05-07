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


A_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = A_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from robot_baseline.motion_types import IkRequest  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A05 wrapper: QP-IK via unified IK interface.")
    parser.add_argument("--motion-task-config", default=str(A_ROOT / "configs" / "motion_task.yaml"))
    parser.add_argument("--target-definition", default=None)
    parser.add_argument("--solver-type", choices=["qp_scipy", "qp_osqp"], default=None)
    parser.add_argument("--task-mode", choices=["position", "pose_6d"], default=None)
    parser.add_argument("--output-dir", default=str(A_ROOT / "outputs"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


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
    return IkRequest(
        solver_type=args.solver_type or "qp_scipy",
        task_mode=args.task_mode or "position",
        site_name="attachment_site",
        weights={},
        limits={},
        solver_config={},
        metadata={
            "motion_task_config": args.motion_task_config,
            "target_definition": args.target_definition,
            "note": "A05 wrapper keeps only high-level CLI overrides; detailed IK parameters live in motion_task.yaml.",
        },
    )


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
