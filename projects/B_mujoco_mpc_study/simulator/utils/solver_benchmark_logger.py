"""B03 solver benchmark logging utilities."""

from __future__ import annotations

from dataclasses import dataclass
import csv
import math
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class SolverStepRow:
    """单步 solver benchmark 记录。"""

    step: int
    time: float
    solver_name: str
    final_ee_error: float
    current_ee_error: float
    best_cost: float
    runtime_ms: float
    num_rollouts: int
    num_iterations: int
    max_abs_torque: float
    control_smoothness: float
    trajectory_smoothness: float
    success: bool


@dataclass(frozen=True)
class SolverSummaryRow:
    """单个 solver 的汇总记录。"""

    solver_name: str
    final_ee_error: float
    mean_ee_error: float
    max_ee_error: float
    runtime_per_control_step: float
    max_abs_torque: float
    mean_abs_torque: float
    control_smoothness: float
    trajectory_smoothness: float
    best_cost: float
    cost_std: float
    solver_success_rate: float


class SolverBenchmarkLogBuffer:
    """B03 solver comparison 日志缓冲。"""

    def __init__(self) -> None:
        self._step_rows: list[SolverStepRow] = []
        self._summary_rows: list[SolverSummaryRow] = []

    def append_step(
        self,
        step: int,
        time: float,
        solver_name: str,
        final_ee_error: float,
        current_ee_error: float,
        best_cost: float,
        runtime_ms: float,
        num_rollouts: int,
        num_iterations: int,
        max_abs_torque: float,
        control_smoothness: float,
        trajectory_smoothness: float,
        success: bool,
    ) -> None:
        """追加一步 solver benchmark 记录。"""
        self._step_rows.append(
            SolverStepRow(
                step=int(step),
                time=float(time),
                solver_name=solver_name,
                final_ee_error=float(final_ee_error),
                current_ee_error=float(current_ee_error),
                best_cost=float(best_cost),
                runtime_ms=float(runtime_ms),
                num_rollouts=int(num_rollouts),
                num_iterations=int(num_iterations),
                max_abs_torque=float(max_abs_torque),
                control_smoothness=float(control_smoothness),
                trajectory_smoothness=float(trajectory_smoothness),
                success=bool(success),
            )
        )

    def append_summary(
        self,
        solver_name: str,
        final_ee_error: float,
        mean_ee_error: float,
        max_ee_error: float,
        runtime_per_control_step: float,
        max_abs_torque: float,
        mean_abs_torque: float,
        control_smoothness: float,
        trajectory_smoothness: float,
        best_cost: float,
        cost_std: float,
        solver_success_rate: float,
    ) -> None:
        """追加单个 solver 的汇总行。"""
        self._summary_rows.append(
            SolverSummaryRow(
                solver_name=solver_name,
                final_ee_error=float(final_ee_error),
                mean_ee_error=float(mean_ee_error),
                max_ee_error=float(max_ee_error),
                runtime_per_control_step=float(runtime_per_control_step),
                max_abs_torque=float(max_abs_torque),
                mean_abs_torque=float(mean_abs_torque),
                control_smoothness=float(control_smoothness),
                trajectory_smoothness=float(trajectory_smoothness),
                best_cost=float(best_cost),
                cost_std=float(cost_std),
                solver_success_rate=float(solver_success_rate),
            )
        )

    def step_rows(self) -> list[SolverStepRow]:
        return list(self._step_rows)

    def summary_rows(self) -> list[SolverSummaryRow]:
        return list(self._summary_rows)

    def save_step_csv(self, output_path: Path) -> None:
        """保存逐步 solver 日志 CSV。"""
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "step",
                    "time",
                    "solver_name",
                    "final_ee_error",
                    "current_ee_error",
                    "best_cost",
                    "runtime_ms",
                    "num_rollouts",
                    "num_iterations",
                    "max_abs_torque",
                    "control_smoothness",
                    "trajectory_smoothness",
                    "success",
                ]
            )
            for row in self._step_rows:
                writer.writerow(
                    [
                        row.step,
                        row.time,
                        row.solver_name,
                        row.final_ee_error,
                        row.current_ee_error,
                        row.best_cost,
                        row.runtime_ms,
                        row.num_rollouts,
                        row.num_iterations,
                        row.max_abs_torque,
                        row.control_smoothness,
                        row.trajectory_smoothness,
                        row.success,
                    ]
                )

    def save_summary_csv(self, output_path: Path) -> None:
        """保存 solver 汇总指标 CSV。"""
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "solver_name",
                    "final_ee_error",
                    "mean_ee_error",
                    "max_ee_error",
                    "runtime_per_control_step",
                    "max_abs_torque",
                    "mean_abs_torque",
                    "control_smoothness",
                    "trajectory_smoothness",
                    "best_cost",
                    "cost_std",
                    "solver_success_rate",
                ]
            )
            for row in self._summary_rows:
                writer.writerow(
                    [
                        row.solver_name,
                        row.final_ee_error,
                        row.mean_ee_error,
                        row.max_ee_error,
                        row.runtime_per_control_step,
                        row.max_abs_torque,
                        row.mean_abs_torque,
                        row.control_smoothness,
                        row.trajectory_smoothness,
                        row.best_cost,
                        row.cost_std,
                        row.solver_success_rate,
                    ]
                )


def _safe_float(values: list[float], agg: str = "mean") -> float:
    """对有限值列表做聚合，全为非有限时返回 NaN。"""
    finite = [v for v in values if math.isfinite(v)]
    if not finite:
        return float("nan")
    if agg == "mean":
        return float(np.mean(finite))
    if agg == "max":
        return float(np.max(finite))
    if agg == "std":
        return float(np.std(finite))
    if agg == "last":
        return finite[-1]
    return float(np.mean(finite))


COMPARISON_METRICS_COLUMNS = [
    "solver_name",
    "final_ee_error",
    "mean_ee_error",
    "max_ee_error",
    "mean_best_cost",
    "std_best_cost",
    "mean_runtime_ms",
    "max_runtime_ms",
    "mean_abs_torque",
    "max_abs_torque",
    "control_smoothness",
    "trajectory_smoothness",
    "success_rate",
    "horizon",
    "num_candidates",
    "num_iterations",
]


def build_comparison_metrics_rows(
    step_rows_by_solver: dict[str, list[SolverStepRow]],
    solver_configs: dict[str, dict[str, int | float]] | None = None,
) -> list[dict[str, float | str | int]]:
    """从每个 solver 的 step rows 构建统一 comparison metrics。

    每个 solver 输出一行，包含所需全部字段。
    solver_configs 可选，用于填充 horizon / num_candidates / num_iterations。
    """
    if not step_rows_by_solver:
        return []

    solver_configs = solver_configs or {}
    rows: list[dict[str, float | str | int]] = []

    for solver_name, steps in step_rows_by_solver.items():
        if not steps:
            rows.append({col: solver_name if col == "solver_name" else float("nan") for col in COMPARISON_METRICS_COLUMNS})
            continue

        ee_errors = [s.current_ee_error for s in steps]
        best_costs = [s.best_cost for s in steps]
        runtimes = [s.runtime_ms for s in steps]
        torques = [s.max_abs_torque for s in steps]
        success_flags = [1.0 if s.success else 0.0 for s in steps]
        last_step = steps[-1]

        cfg = solver_configs.get(solver_name, {})
        horizon = int(cfg.get("horizon", 0))
        num_candidates = int(cfg.get("num_candidates", 0))
        num_iterations = int(cfg.get("num_iterations", 0))

        rows.append({
            "solver_name": solver_name,
            "final_ee_error": _safe_float(ee_errors, "last"),
            "mean_ee_error": _safe_float(ee_errors, "mean"),
            "max_ee_error": _safe_float(ee_errors, "max"),
            "mean_best_cost": _safe_float(best_costs, "mean"),
            "std_best_cost": _safe_float(best_costs, "std"),
            "mean_runtime_ms": _safe_float(runtimes, "mean"),
            "max_runtime_ms": _safe_float(runtimes, "max"),
            "mean_abs_torque": _safe_float(torques, "mean"),
            "max_abs_torque": _safe_float(torques, "max"),
            "control_smoothness": float(last_step.control_smoothness),
            "trajectory_smoothness": float(last_step.trajectory_smoothness),
            "success_rate": _safe_float(success_flags, "mean"),
            "horizon": horizon,
            "num_candidates": num_candidates,
            "num_iterations": num_iterations,
        })

    return rows


def save_comparison_metrics_csv(
    step_rows_by_solver: dict[str, list[SolverStepRow]],
    output_path: Path,
    solver_configs: dict[str, dict[str, int | float]] | None = None,
) -> None:
    """构建并保存统一 comparison metrics CSV。"""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    rows = build_comparison_metrics_rows(step_rows_by_solver, solver_configs)
    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=COMPARISON_METRICS_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
