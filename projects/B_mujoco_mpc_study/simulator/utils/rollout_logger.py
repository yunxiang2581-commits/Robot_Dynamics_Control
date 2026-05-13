"""B03 rollout logging utilities.

本模块只记录 predictive sampling 的教学型中间量，不负责 MuJoCo 仿真。
"""

from __future__ import annotations

from dataclasses import dataclass
import csv
from pathlib import Path
from typing import Sequence


@dataclass(frozen=True)
class SelectedRolloutRow:
    """每个控制周期被选中的 best rollout 摘要。"""

    step: int
    time: float
    selected_index: int
    best_cost: float
    mean_cost: float
    first_control_u1: float
    first_control_u2: float
    actual_x: float
    actual_y: float
    target_x: float
    target_y: float


@dataclass(frozen=True)
class CandidateCostRow:
    """单个候选 rollout 的 cost 排名记录。"""

    step: int
    candidate_index: int
    rank: int
    cost: float
    is_selected: bool


class RolloutLogBuffer:
    """B03 rollout 日志缓冲。

    这里故意保持成一个小类：
    - selected rows 用于复盘每个控制周期选了谁。
    - candidate cost rows 用于画 cost distribution 和 best cost 曲线。
    """

    def __init__(self) -> None:
        self._selected_rows: list[SelectedRolloutRow] = []
        self._candidate_cost_rows: list[CandidateCostRow] = []

    def append_selected_rollout(
        self,
        step: int,
        time: float,
        selected_index: int,
        best_cost: float,
        mean_cost: float,
        first_control: tuple[float, float],
        current_actual: tuple[float, float],
        current_target: tuple[float, float],
    ) -> None:
        """追加当前控制周期的 best rollout 摘要。

        输入：
        - 当前 step/time。
        - cost 排名选出的 `selected_index`。
        - best/mean cost。
        - best sequence 的第一项控制。
        - 当前实际末端位置和目标末端位置。

        输出：
        - 无返回值，内部 selected rows 增加一行。
        """
        u1, u2 = first_control
        actual_x, actual_y = current_actual
        target_x, target_y = current_target
        self._selected_rows.append(
            SelectedRolloutRow(
                step=int(step),
                time=float(time),
                selected_index=int(selected_index),
                best_cost=float(best_cost),
                mean_cost=float(mean_cost),
                first_control_u1=float(u1),
                first_control_u2=float(u2),
                actual_x=float(actual_x),
                actual_y=float(actual_y),
                target_x=float(target_x),
                target_y=float(target_y),
            )
        )

    def append_candidate_costs(
        self,
        step: int,
        costs: Sequence[float],
        sorted_indices: Sequence[int],
        selected_index: int,
    ) -> None:
        """按排序结果追加所有候选 rollout 的 cost。

        输入：
        - `costs[i]`：第 i 个候选的 cost。
        - `sorted_indices`：从低 cost 到高 cost 的候选编号。
        - `selected_index`：本步被选中的候选编号。

        输出：
        - 每个候选追加一行，方便后处理画分布或排名。
        """
        for rank, candidate_index in enumerate(sorted_indices):
            cost = costs[int(candidate_index)]
            self._candidate_cost_rows.append(
                CandidateCostRow(
                    step=int(step),
                    candidate_index=int(candidate_index),
                    rank=int(rank),
                    cost=float(cost),
                    is_selected=int(candidate_index) == int(selected_index),
                )
            )

    def selected_rows(self) -> list[SelectedRolloutRow]:
        """返回 selected rollout 日志副本。"""
        return list(self._selected_rows)

    def candidate_cost_rows(self) -> list[CandidateCostRow]:
        """返回 candidate cost 日志副本。"""
        return list(self._candidate_cost_rows)

    def save_selected_rollout_csv(self, output_path: Path) -> None:
        """保存 selected rollout CSV。"""
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "step",
                    "time",
                    "selected_index",
                    "best_cost",
                    "mean_cost",
                    "first_control_u1",
                    "first_control_u2",
                    "actual_x",
                    "actual_y",
                    "target_x",
                    "target_y",
                ]
            )
            for row in self._selected_rows:
                writer.writerow(
                    [
                        row.step,
                        row.time,
                        row.selected_index,
                        row.best_cost,
                        row.mean_cost,
                        row.first_control_u1,
                        row.first_control_u2,
                        row.actual_x,
                        row.actual_y,
                        row.target_x,
                        row.target_y,
                    ]
                )

    def save_candidate_costs_csv(self, output_path: Path) -> None:
        """保存 candidate cost CSV。"""
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["step", "candidate_index", "rank", "cost", "is_selected"])
            for row in self._candidate_cost_rows:
                writer.writerow([row.step, row.candidate_index, row.rank, row.cost, row.is_selected])
