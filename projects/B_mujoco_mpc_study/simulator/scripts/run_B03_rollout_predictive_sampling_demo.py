"""Run B03 rollout predictive sampling demo skeleton.

B03-A 只创建薄 runner：
- 解析命令行参数。
- 解析输出目录。
- 创建 logs / figures / metrics / videos 子目录。
- 提示当前核心 rollout / cost / receding horizon 仍是 TODO。

本脚本不会运行长时间 MuJoCo 仿真，也不会生成正式 B03 MP4。
"""

from __future__ import annotations

import argparse
from datetime import datetime
import logging
from pathlib import Path
from typing import Any


SCRIPT_PATH = Path(__file__).resolve()
SIMULATOR_ROOT = SCRIPT_PATH.parents[1]
PROJECT_ROOT = SIMULATOR_ROOT.parent
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "configs" / "B03_rollout_predictive_sampling.yaml"
TASK_NAME = "B03_rollout_predictive_sampling_demo"


def build_arg_parser() -> argparse.ArgumentParser:
    """创建 B03 命令行参数。

    这些参数先固定教学接口，后续 B03-R1/R2 再逐步接入 planner 和 MuJoCo env。
    """
    parser = argparse.ArgumentParser(description="Run B03 rollout predictive sampling demo skeleton.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH, help="B03 YAML config path.")
    parser.add_argument("--run-id", type=str, default=None, help="Optional run id.")
    parser.add_argument("--num-steps", type=int, default=None, help="Override closed-loop step count.")
    parser.add_argument("--horizon", type=int, default=None, help="Override predictive horizon.")
    parser.add_argument("--num-candidates", type=int, default=None, help="Override sampled candidate count.")
    parser.add_argument("--seed", type=int, default=None, help="Override random seed.")
    parser.add_argument("--export-video", action=argparse.BooleanOptionalAction, default=None, help="Export MP4 later.")
    parser.add_argument("--save-figures", action=argparse.BooleanOptionalAction, default=None, help="Save figures later.")
    parser.add_argument("--save-metrics", action=argparse.BooleanOptionalAction, default=None, help="Save metrics later.")
    parser.add_argument("--no-show-viewer", action="store_true", help="Keep MuJoCo viewer disabled.")
    parser.add_argument("--log-level", type=str, default="INFO", help="Python logging level.")
    return parser


def build_run_dir(run_id: str | None) -> Path:
    """生成 B03 输出目录。

    输出路径：
    `outputs/runs/B03_rollout_predictive_sampling_demo/<run_id>/`
    """
    actual_run_id = run_id or datetime.now().strftime("%Y%m%d_%H%M%S")
    return PROJECT_ROOT / "outputs" / "runs" / TASK_NAME / actual_run_id


def build_run_outputs(run_dir: Path) -> dict[str, Path]:
    """生成 B03 标准输出子目录。"""
    return {
        "run_dir": run_dir,
        "logs": run_dir / "logs",
        "figures": run_dir / "figures",
        "metrics": run_dir / "metrics",
        "videos": run_dir / "videos",
    }


def ensure_output_dirs(outputs: dict[str, Path]) -> None:
    """创建输出目录。"""
    for output_path in outputs.values():
        output_path.mkdir(parents=True, exist_ok=True)


def run_b03_skeleton(args: argparse.Namespace) -> dict[str, Any]:
    """执行 B03-A skeleton runner。

    当前只做路径和参数检查，不进入 MuJoCo rollout。
    """
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    run_dir = build_run_dir(args.run_id)
    outputs = build_run_outputs(run_dir)
    ensure_output_dirs(outputs)

    if not args.config.exists():
        raise FileNotFoundError(f"Cannot find B03 config: {args.config}")

    logging.info("B03 skeleton runner prepared outputs at %s", run_dir)
    logging.info("B03-A keeps MuJoCo rollout, cost evaluation, video export, and receding horizon as TODO.")

    return {
        "task_name": TASK_NAME,
        "config": args.config,
        "run_dir": run_dir,
        "outputs": outputs,
        "num_steps": args.num_steps,
        "horizon": args.horizon,
        "num_candidates": args.num_candidates,
        "seed": args.seed,
        "export_video": args.export_video,
        "save_figures": args.save_figures,
        "save_metrics": args.save_metrics,
        "show_viewer": False,
        "status": "B03 TODO skeleton prepared; no long MuJoCo simulation was run.",
    }


def main(argv: list[str] | None = None) -> None:
    """B03 skeleton 主入口。"""
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    result = run_b03_skeleton(args)

    print(result["status"])
    print(f"run_dir: {result['run_dir']}")
    print("Core rollout / cost / receding-horizon logic remains TODO for B03-R1.")


if __name__ == "__main__":
    main()
