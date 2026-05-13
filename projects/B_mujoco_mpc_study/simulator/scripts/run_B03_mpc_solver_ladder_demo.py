"""Run B03 MPC solver ladder demo skeleton.

B03-REDESIGN-A 只创建薄 runner：
- 读取 B03 config；
- 解析 solver 列表和输出目录；
- 创建统一 benchmark logger；
- 预留 simple-model env / solver comparison loop 接口；
- 当前不运行长时间 MuJoCo 仿真，不生成正式 MP4。
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
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "configs" / "B03_mpc_solver_ladder.yaml"
TASK_NAME = "B03_mpc_solver_ladder_demo"


def build_arg_parser() -> argparse.ArgumentParser:
    """创建 B03 solver ladder 参数。"""
    parser = argparse.ArgumentParser(description="Run B03 MPC solver ladder demo skeleton.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH, help="B03 YAML config path.")
    parser.add_argument("--run-id", type=str, default=None, help="Optional run id.")
    parser.add_argument("--model-family", type=str, default=None, help="Model family override.")
    parser.add_argument("--solvers", nargs="*", default=None, help="Enabled solver names.")
    parser.add_argument("--num-steps", type=int, default=None, help="Override closed-loop step count.")
    parser.add_argument("--horizon", type=int, default=None, help="Override horizon.")
    parser.add_argument("--num-candidates", type=int, default=None, help="Override candidate count.")
    parser.add_argument("--seed", type=int, default=None, help="Override random seed.")
    parser.add_argument("--export-video", action=argparse.BooleanOptionalAction, default=None, help="Export MP4 later.")
    parser.add_argument("--save-figures", action=argparse.BooleanOptionalAction, default=None, help="Save figures later.")
    parser.add_argument("--save-metrics", action=argparse.BooleanOptionalAction, default=None, help="Save metrics later.")
    parser.add_argument("--no-show-viewer", action="store_true", help="Keep viewer disabled.")
    parser.add_argument("--log-level", type=str, default="INFO", help="Python logging level.")
    return parser


def build_run_dir(run_id: str | None) -> Path:
    """生成 B03 solver ladder 输出目录。"""
    actual_run_id = run_id or datetime.now().strftime("%Y%m%d_%H%M%S")
    return PROJECT_ROOT / "outputs" / "runs" / TASK_NAME / actual_run_id


def build_run_outputs(run_dir: Path) -> dict[str, Path]:
    """生成标准输出目录。"""
    return {
        "run_dir": run_dir,
        "videos": run_dir / "videos",
        "figures": run_dir / "figures",
        "metrics": run_dir / "metrics",
        "logs": run_dir / "logs",
    }


def ensure_output_dirs(outputs: dict[str, Path]) -> None:
    """创建输出目录。"""
    for output_path in outputs.values():
        output_path.mkdir(parents=True, exist_ok=True)


def run_b03_solver_ladder_skeleton(args: argparse.Namespace) -> dict[str, Any]:
    """执行 B03 solver ladder skeleton runner。

    当前职责：
    - 验证 config 路径存在；
    - 创建标准输出目录；
    - 记录当前启用的 solver 名称；
    - 预留 future comparison loop 接口。
    """
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))

    if not args.config.exists():
        raise FileNotFoundError(f"Cannot find B03 config: {args.config}")

    run_dir = build_run_dir(args.run_id)
    outputs = build_run_outputs(run_dir)
    ensure_output_dirs(outputs)

    enabled_solvers = [] if args.solvers is None else list(args.solvers)
    logging.info("B03 solver ladder skeleton prepared outputs at %s", run_dir)
    logging.info("Enabled solver overrides: %s", enabled_solvers)
    logging.info("B03-A keeps long MuJoCo runs, formal video export, and advanced solvers as TODO.")

    return {
        "task_name": TASK_NAME,
        "config": args.config,
        "run_dir": run_dir,
        "outputs": outputs,
        "model_family": args.model_family,
        "enabled_solvers": enabled_solvers,
        "num_steps": args.num_steps,
        "horizon": args.horizon,
        "num_candidates": args.num_candidates,
        "seed": args.seed,
        "export_video": args.export_video,
        "save_figures": args.save_figures,
        "save_metrics": args.save_metrics,
        "show_viewer": False,
        "status": "B03 solver ladder TODO skeleton prepared; no long MuJoCo simulation was run.",
    }


def main(argv: list[str] | None = None) -> None:
    """B03 solver ladder 主入口。"""
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    result = run_b03_solver_ladder_skeleton(args)

    print(result["status"])
    print(f"run_dir: {result['run_dir']}")
    print("Sampling / CEM / MPPI / iLQG / SQP / direct multiple shooting remain staged learning tasks.")


if __name__ == "__main__":
    main()
