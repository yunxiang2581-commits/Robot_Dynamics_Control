"""B02 tracking log utilities.

本模块负责把 B02 闭环仿真的每一步记录成统一 schema，供：
- CSV 复盘
- 图表后处理
- 标记视频后处理

第一版只实现最小可用的日志缓冲和 CSV 输出。
"""

from __future__ import annotations

from dataclasses import dataclass
import csv
import math
from pathlib import Path


@dataclass(frozen=True)
class TrackingLogRow:
    """B02 单步 tracking 记录。"""

    step: int
    time: float
    q1: float
    q2: float
    dq1: float
    dq2: float
    target_x: float
    target_y: float
    actual_x: float
    actual_y: float
    error_norm: float
    u1: float
    u2: float
    mpc_cost: float


class TrackingLogBuffer:
    """B02 tracking 日志缓冲。"""

    def __init__(self) -> None:
        self._rows: list[TrackingLogRow] = []

    def append_step(
        self,
        step: int,
        time: float,
        state: tuple[float, float, float, float],
        target_xy: tuple[float, float],
        actual_xy: tuple[float, float],
        control: tuple[float, float],
        mpc_cost: float,
    ) -> None:
        """追加一步 tracking 日志。

        TODO:
        - 要实现什么：把单步 tracking 数据转换成统一 schema，并追加到内部缓冲。
        - 为什么需要：后续 CSV、figures、video 都要共享同一份事实数据。
        - 输入是什么：step、time、state、target_xy、actual_xy、control、mpc_cost。
        - 输出是什么：无返回值，但内部 row 数量增加 1。
        - 验证标准：追加 N 步后 `len(to_rows()) == N`，且 `error_norm` 等于目标与实际末端的欧氏距离。
        """
        q1, q2, dq1, dq2 = state
        target_x, target_y = target_xy
        actual_x, actual_y = actual_xy
        u1, u2 = control
        error_norm = math.hypot(actual_x - target_x, actual_y - target_y)
        self._rows.append(
            TrackingLogRow(
                step=step,
                time=time,
                q1=q1,
                q2=q2,
                dq1=dq1,
                dq2=dq2,
                target_x=target_x,
                target_y=target_y,
                actual_x=actual_x,
                actual_y=actual_y,
                error_norm=error_norm,
                u1=u1,
                u2=u2,
                mpc_cost=mpc_cost,
            )
        )

    def to_rows(self) -> list[TrackingLogRow]:
        """返回日志副本，避免调用方原地修改内部缓冲。"""
        return list(self._rows)

    def save_csv(self, output_path: Path) -> None:
        """保存 tracking CSV。

        TODO:
        - 要实现什么：把 tracking rows 写成一行一步的 CSV 文件。
        - 为什么需要：独立后处理要让 figures 和 video 共享同一份可复盘日志。
        - 输入是什么：输出路径。
        - 输出是什么：`B02_tracking_log.csv`。
        - 验证标准：CSV 行数应等于仿真步数加表头 1 行。
        """
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "step", "time", "q1", "q2", "dq1", "dq2",
                    "target_x", "target_y", "actual_x", "actual_y",
                    "error_norm", "u1", "u2", "mpc_cost",
                ]
            )
            for row in self._rows:
                writer.writerow(
                    [
                        row.step, row.time, row.q1, row.q2, row.dq1, row.dq2,
                        row.target_x, row.target_y, row.actual_x, row.actual_y,
                        row.error_norm, row.u1, row.u2, row.mpc_cost,
                    ]
                )
