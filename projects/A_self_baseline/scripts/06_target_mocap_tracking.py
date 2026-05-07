"""A06 Target / Mocap-Style Tracking TODO skeleton.

本脚本是 A06：target manager 的学习骨架。

三句核心概念：
- viewer target: 在 viewer 中可视化或交互移动的任务目标。
- mocap-style target: 用 MuJoCo mocap body 表示可移动目标，其 pose 来自 data.mocap_pos / data.mocap_quat。
- target pose feeds IK: target pose 是 IK 的任务输入，不是 actuator control 输出。

学习定位：
- A06 定义 target pose，管理 fixed target / pose sequence / mocap placeholder。
- A06 把 target metadata 提供给 A04/A05 IK。
- A06 不重新实现 IK。
- A06 不写 data.ctrl。
- A06 不做 actuator tracking。
- A06 不启动真实 interactive viewer。
- A07 才负责 actuator tracking。

未来输出规划：
- outputs/cache/A06_target_definition.json
- outputs/logs/A06_target_tracking.csv
- outputs/reports/A06_target_tracking_report.md
- outputs/figures/A06_target_path_preview.png

当前状态：
- TODO learning skeleton。
- 不创建派生 MJCF。
- 不实现 mouse drag / keyboard target。
- 不实现 kinematic IK follow。
- 不调用 mink。
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve()
A_ROOT = SCRIPT_PATH.parents[1]
REPO_ROOT = A_ROOT.parents[1]


DEFAULT_DERIVED_MJCF = REPO_ROOT / "shared" / "robot_assets" / "models" / "mink_universal_robots_ur5e" / "scene_a06_mocap_target.xml"


TODO_ITEMS = [
    "TODO 1: 读取 A02/A04/A05 前置产物，确认 current pose 和可用 trajectory_source。",
    "TODO 2: 读取 robot.yaml / target_tracking.yaml，确认 target site、body、mode、duration。",
    "TODO 3: 定义 TargetDefinition schema: position(3), quat_wxyz(4), frame, timestamp。",
    "TODO 4: fixed_pose 模式: target_position = current_position + offset，orientation keep_current。",
    "TODO 5: pose_sequence 模式: 未来用 numpy.linspace 规划 waypoint list。",
    "TODO 6: mocap_placeholder 模式: 未来检查 model.nmocap 和 data.mocap_pos / data.mocap_quat。",
    "TODO 7: interactive_viewer_todo: 未来规划 viewer target / mouse drag / keyboard movement。",
    "TODO 8: target error 定义: e_pos = p_target - p_current, e_rot = log(R_target R_current^T)。",
    "TODO 9: 记录 ik_backend 和 trajectory_source，给 A07 读取 q_des。",
    "TODO 10: 输出 target definition、tracking log、report、path preview；当前不生成。",
    "TODO 11: 明确 A06 不调用 data.ctrl，不运行 mj_step control loop，不录 video。",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A06 TODO skeleton: target / mocap-style tracking manager.")
    parser.add_argument("--robot-config", default=str(A_ROOT / "configs" / "robot.yaml"))
    parser.add_argument("--target-config", default=str(A_ROOT / "configs" / "target_tracking.yaml"))
    parser.add_argument("--motion-task-config", default=str(A_ROOT / "configs" / "motion_task.yaml"))
    parser.add_argument("--a02-pose", default=str(A_ROOT / "outputs" / "cache" / "A02_site_pose.json"))
    parser.add_argument("--a04-traj", default=str(A_ROOT / "outputs" / "trajectories" / "A04_dls_ik_q_traj.npy"))
    parser.add_argument("--a05-traj", default=str(A_ROOT / "outputs" / "trajectories" / "A05_qp_ik_q_traj.npy"))
    parser.add_argument("--ik-backend", choices=["a04_dls", "a05_qp"], default="a05_qp")
    parser.add_argument("--target-mode", choices=["fixed_pose", "pose_sequence", "mocap_placeholder", "interactive_viewer_todo"], default="fixed_pose")
    parser.add_argument("--target-position", default="0.52,0.13,0.49")
    parser.add_argument("--target-position-offset", default="0.03,0.00,0.00")
    parser.add_argument("--target-orientation-mode", choices=["keep_current", "fixed_rpy", "fixed_quat"], default="keep_current")
    parser.add_argument("--num-waypoints", type=int, default=5)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--interactive-viewer", action="store_true", help="TODO only: 未来打开 MuJoCo passive viewer。")
    parser.add_argument("--viewer-mode", choices=["mouse_keyboard", "keyboard_only", "mouse_only"], default="mouse_keyboard")
    parser.add_argument("--interactive-mjcf", default=str(DEFAULT_DERIVED_MJCF))
    parser.add_argument("--mocap-body-name", default="a06_target")
    parser.add_argument("--mocap-site-name", default="a06_target_site")
    parser.add_argument("--keyboard-step", type=float, default=0.01)
    parser.add_argument("--viewer-rate-hz", type=float, default=60.0)
    parser.add_argument("--max-viewer-seconds", type=float, default=60.0)
    parser.add_argument("--enable-ik-follow", action="store_true", help="TODO only: 未来只更新 qpos，不写 data.ctrl。")
    parser.add_argument("--site", default="attachment_site")
    parser.add_argument("--body", default="wrist_3_link")
    parser.add_argument("--output-dir", default=str(A_ROOT / "outputs"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO), format="%(levelname)s: %(message)s")

    output_root = Path(args.output_dir)
    if not output_root.is_absolute():
        output_root = A_ROOT / output_root

    planned_outputs = {
        "target_definition": output_root / "cache" / "A06_target_definition.json",
        "tracking_log": output_root / "logs" / "A06_target_tracking.csv",
        "path_preview": output_root / "figures" / "A06_target_path_preview.png",
        "report": output_root / "reports" / "A06_target_tracking_report.md",
    }

    logging.info("A06 当前是 target manager TODO learning skeleton。")
    logging.info("target_mode=%s, ik_backend=%s, site=%s", args.target_mode, args.ik_backend, args.site)
    logging.info("interactive_viewer=%s, viewer_mode=%s, enable_ik_follow=%s", args.interactive_viewer, args.viewer_mode, args.enable_ik_follow)
    logging.info("未来输出: %s", planned_outputs)
    for item in TODO_ITEMS:
        logging.info(item)

    raise NotImplementedError("A06 TODO: 定义 target metadata；当前不启动 viewer、不求 IK、不写 data.ctrl。")


if __name__ == "__main__":
    main()
