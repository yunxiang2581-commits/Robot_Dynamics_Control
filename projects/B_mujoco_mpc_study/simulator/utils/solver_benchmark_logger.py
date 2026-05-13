"""B03 solver benchmark logging utilities."""

from __future__ import annotations

from dataclasses import dataclass
import csv
from pathlib import Path


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
