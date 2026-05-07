"""A06 Target + Viewer interface thin CLI wrapper.

A06 归入 Target interface + Viewer interface：
- 当前只保留函数级 TODO learning skeleton。
- 完整公式、符号表、验证标准和常见错误见
  ``projects/A_self_baseline/docs/A06_Target_Viewer_TODO_full_plan.md``。

viewer target:
在 viewer 中可视化或交互移动的任务目标。

mocap-style target:
用 MuJoCo mocap body 表示可移动目标。

target pose feeds IK:
target pose 是 IK 的任务输入，不是 actuator control 输出。

当前不创建派生 MJCF，不启动 viewer，不重新求 IK，不写 ``data.ctrl``。
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
from typing import Any


A_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = A_ROOT.parents[1]
SRC_ROOT = A_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from robot_baseline.motion_types import ViewerTargetSpec  # noqa: E402


DEFAULT_DERIVED_MJCF = REPO_ROOT / "shared" / "robot_assets" / "models" / "mink_universal_robots_ur5e" / "scene_a06_mocap_target.xml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A06 wrapper: Target + Viewer interface TODO.")
    parser.add_argument("--motion-task-config", default=str(A_ROOT / "configs" / "motion_task.yaml"))
    parser.add_argument("--target-mode", choices=["fixed_pose", "pose_sequence", "mocap_placeholder", "interactive_viewer_todo"], default=None)
    parser.add_argument("--interactive-viewer", action="store_true", help="TODO only: 未来打开 MuJoCo passive viewer。")
    parser.add_argument("--viewer-mode", choices=["mouse_keyboard", "keyboard_only", "mouse_only"], default=None)
    parser.add_argument("--output-dir", default=str(A_ROOT / "outputs"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def build_target_request(args: argparse.Namespace) -> dict[str, Any]:
    """规划 A06 target request。

    TODO:
    - 要实现什么：读取 target mode，规划 fixed_pose / pose_sequence / mocap_placeholder / interactive_viewer_todo。
    - 为什么需要：A06 负责 target 管理，不求 IK。
    - 对标 mink：viewer/mocap target 最终 feed FrameTask。
    - 输入：CLI args、motion_task.yaml、A02 pose。
    - 输出：target request dict 或 TargetDefinition。
    - 推荐 API：``target_interface``。
    - 验证标准：target_mode 合法，site/body 明确。
    """
    return {
        "target_mode": args.target_mode or "fixed_pose",
        "site": "attachment_site",
        "body": "wrist_3_link",
        "motion_task_config": args.motion_task_config,
    }


def build_fixed_target(args: argparse.Namespace) -> Any:
    """生成 fixed TargetDefinition。

    TODO:
    - 要实现什么：支持 ``p_target = p_current + offset`` 或直接固定 position。
    - 为什么需要：fixed target 是最小可复现实验目标。
    - 对标 mink：手动设置 FrameTask target。
    - 输入：A02 current pose、target-position、target-position-offset。
    - 输出：TargetDefinition。
    - 推荐 API：``target_interface.target_from_offset``。
    - 验证标准：position shape=(3,)，quat_wxyz shape=(4,)。
    """
    raise NotImplementedError("TODO A06: 生成 fixed target；当前不写 target JSON。")


def build_pose_sequence(args: argparse.Namespace) -> list[Any]:
    """生成 pose sequence。

    TODO:
    - 要实现什么：使用 ``numpy.linspace`` 生成 waypoints。
    - 为什么需要：支持多目标点和后续轨迹 target。
    - 对标 mink：每帧更新 task target。
    - 输入：start pose、end pose、num_waypoints、duration。
    - 输出：waypoints。
    - 推荐 API：``numpy.linspace``。
    - 验证标准：waypoints 数量等于 num_waypoints；orientation 暂时 keep_current。
    """
    raise NotImplementedError("TODO A06: 生成 pose sequence；当前不插值。")


def plan_mocap_placeholder(args: argparse.Namespace) -> dict[str, Any]:
    """规划 mocap placeholder。

    TODO:
    - 要实现什么：未来检查 ``model.nmocap`` 和 mocap arrays shape。
    - 为什么需要：interactive target 前必须确认模型是否支持 mocap body。
    - 对标 mink：mocap-style target。
    - 输入：MuJoCo model metadata。
    - 输出：mocap capability report。
    - 推荐 API：``model.nmocap``、``data.mocap_pos.shape``、``data.mocap_quat.shape``。
    - 验证标准：不启动 viewer，不写 ``data.mocap_pos``。
    """
    raise NotImplementedError("TODO A06: 检查 mocap placeholder；当前不加载 viewer model。")


def plan_interactive_viewer(args: argparse.Namespace) -> dict[str, Any]:
    """规划 interactive viewer。

    TODO:
    - 要实现什么：规划派生 MJCF、mouse drag、keyboard movement、target pose feeds IK。
    - 为什么需要：对齐 mink viewer target 数据流。
    - 对标 mink：viewer target / mocap target。
    - 输入：ViewerTargetSpec、派生 MJCF 路径。
    - 输出：viewer implementation plan。
    - 推荐 API：``viewer_interface``。
    - 验证标准：当前不创建 MJCF，不启动 viewer，不写 ``data.ctrl``。
    """
    raise NotImplementedError("TODO A06: 规划 interactive viewer；当前不启动 viewer。")


def write_target_outputs(result: Any, output_paths: dict[str, Path]) -> None:
    """写 A06 输出。

    TODO:
    - 要实现什么：输出 target definition、tracking log、path preview、report。
    - 为什么需要：target 数据流需要可复盘，并给 A05/A07 提供 metadata。
    - 对标 mink：viewer 实时 target 在本项目中落成文件。
    - 输入：TargetDefinition / tracking rows / output paths。
    - 输出：A06_target_definition.json、CSV、PNG、Markdown。
    - 推荐 API：``json``、``csv``、``matplotlib``。
    - 验证标准：TODO 步骤不生成运行 outputs。
    """
    raise NotImplementedError("TODO A06: 写 target outputs；当前不生成 outputs。")


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO), format="%(levelname)s: %(message)s")

    output_root = Path(args.output_dir)
    if not output_root.is_absolute():
        output_root = A_ROOT / output_root

    viewer_spec = ViewerTargetSpec(
        enabled=args.interactive_viewer,
        viewer_mode=args.viewer_mode or "mouse_keyboard",
        derived_mjcf=str(DEFAULT_DERIVED_MJCF),
        mocap_body_name="a06_target",
        mocap_site_name="a06_target_site",
        keyboard_step=0.01,
        enable_ik_follow=False,
        metadata={
            "motion_task_config": args.motion_task_config,
            "note": "A06 wrapper keeps only high-level CLI overrides; detailed target/viewer parameters live in motion_task.yaml.",
        },
    )
    target_request = build_target_request(args)
    planned_outputs = {
        "target_definition": output_root / "cache" / "A06_target_definition.json",
        "tracking_log": output_root / "logs" / "A06_target_tracking.csv",
        "path_preview": output_root / "figures" / "A06_target_path_preview.png",
        "report": output_root / "reports" / "A06_target_tracking_report.md",
    }

    logging.info("A06 Target+Viewer wrapper: function-level TODO skeleton.")
    logging.info("target_request=%s", target_request)
    logging.info("viewer_mode=%s, mocap_body=%s", viewer_spec.viewer_mode, viewer_spec.mocap_body_name)
    logging.info("planned_outputs=%s", planned_outputs)

    raise NotImplementedError("A06 wrapper TODO: 当前只规划 target/viewer schema，不启动 viewer、不写 data.ctrl。")


if __name__ == "__main__":
    main()
