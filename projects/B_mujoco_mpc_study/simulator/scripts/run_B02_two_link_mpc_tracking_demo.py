"""Run B02 two-link MPC tracking demo skeleton.

本脚本是 B02 的教学型入口。
当前阶段只规划 argparse、路径、输出目录和 TODO 流程，不实现完整二连杆 MPC。
"""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
from typing import Any


SCRIPT_PATH = Path(__file__).resolve()
SIMULATOR_ROOT = SCRIPT_PATH.parents[1]
PROJECT_ROOT = SIMULATOR_ROOT.parent
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "configs" / "B02_two_link_mpc.yaml"
TASK_NAME = "B02_two_link_mpc_tracking_demo"


B02_OUTPUT_FILENAMES = {
    "video": ("videos", "B02_two_link_mpc_tracking_demo.mp4"),
    "ee_trajectory_xy": ("figures", "B02_ee_trajectory_xy.png"),
    "ee_tracking_error": ("figures", "B02_ee_tracking_error.png"),
    "joint_torque": ("figures", "B02_joint_torque.png"),
    "metrics": ("metrics", "B02_metrics.csv"),
    "log": ("logs", "B02_control_log.txt"),
}


def build_arg_parser() -> argparse.ArgumentParser:
    """创建 B02 命令行参数。

    TODO:
    - 要实现什么：解析配置文件、run_id、horizon、num_candidates、输出开关等参数。
    - 为什么要实现：B02 必须能复现实验，也要能临时覆盖参数。
    - 输入是什么：命令行参数。
    - 输出是什么：argparse parser。
    - 物理意义：指定二连杆模型、目标轨迹和控制约束。
    - 数学意义：指定 horizon cost 和 predictive sampling 的求解参数。
    - 验证标准：`--help` 能显示所有 B02 参数。
    """
    parser = argparse.ArgumentParser(description="Run B02 two-link MPC tracking demo skeleton.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH, help="B02 YAML 配置文件。")
    parser.add_argument("--run-id", type=str, default=None, help="本次运行 ID，默认使用当前时间。")
    parser.add_argument("--show-viewer", action=argparse.BooleanOptionalAction, default=None, help="是否打开 MuJoCo 实时窗口。")
    parser.add_argument("--export-video", action=argparse.BooleanOptionalAction, default=None, help="是否导出 mp4。")
    parser.add_argument("--save-figures", action=argparse.BooleanOptionalAction, default=None, help="是否保存 figures。")
    parser.add_argument("--save-metrics", action=argparse.BooleanOptionalAction, default=None, help="是否保存 metrics CSV。")
    return parser


def build_run_dir(run_id: str | None) -> Path:
    """生成 B02 本次运行目录。

    TODO:
    - 要实现什么：按 `outputs/runs/<task_name>/<run_id>/` 生成路径。
    - 为什么要实现：Project B 要求每次实验按任务名和时间单独保存。
    - 输入是什么：可选 run_id。
    - 输出是什么：本次运行目录。
    - 物理意义：把视频、图像、日志和指标归档到同一次实验。
    - 数学意义：不涉及数学，只保证实验可复现和可比较。
    - 验证标准：为空时使用当前时间 `YYYYMMDD_HHMMSS`。
    """
    actual_run_id = run_id or datetime.now().strftime("%Y%m%d_%H%M%S")
    return PROJECT_ROOT / "outputs" / "runs" / TASK_NAME / actual_run_id


def build_run_outputs(run_dir: Path) -> dict[str, Path]:
    """生成 B02 输出文件路径。"""
    return {
        key: run_dir / subdir / filename
        for key, (subdir, filename) in B02_OUTPUT_FILENAMES.items()
    }


def ensure_output_dirs(outputs: dict[str, Path]) -> None:
    """创建输出目录。"""
    for output_path in outputs.values():
        output_path.parent.mkdir(parents=True, exist_ok=True)


def run_b02_skeleton(args: argparse.Namespace) -> dict[str, Any]:
    """B02 教学型运行骨架。

    TODO:
    - 要实现什么：加载配置、创建 TwoLinkEnv、planner、controller，并执行闭环。
    - 为什么要实现：这是 B02 从文档进入可运行 demo 的主入口。
    - 输入是什么：解析后的命令行参数。
    - 输出是什么：包含输出路径和 TODO 状态的字典。
    - 物理意义：后续会驱动二连杆末端跟踪目标轨迹。
    - 数学意义：后续会执行 task-space MPC 闭环。
    - 验证标准：当前阶段应能创建输出目录并明确提示核心算法尚未实现。
    """
    run_dir = build_run_dir(args.run_id)
    outputs = build_run_outputs(run_dir)
    ensure_output_dirs(outputs)

    return {
        "task_name": TASK_NAME,
        "run_dir": run_dir,
        "outputs": outputs,
        "status": "TODO skeleton only; B02 full MPC is not implemented yet.",
    }


def main(argv: list[str] | None = None) -> None:
    """B02 demo skeleton 主入口。"""
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    result = run_b02_skeleton(args)

    print("B02 two-link MPC tracking demo skeleton is ready.")
    print(f"run_dir: {result['run_dir']}")
    print(result["status"])


if __name__ == "__main__":
    main()
