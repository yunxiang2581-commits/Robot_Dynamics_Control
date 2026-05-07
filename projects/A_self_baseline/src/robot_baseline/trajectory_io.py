"""Trajectory IO helpers for A 项目。

本模块只负责轨迹文件和日志文件的 IO / shape check，不求 IK，不控制
MuJoCo，不写 ``data.ctrl``。
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any

import numpy as np

from robot_baseline.motion_types import TrajectorySource


def load_q_traj(path: str | Path, expected_nq: int | None = None) -> np.ndarray:
    """读取 q trajectory。

    输入：``.npy`` 路径。
    输出：q_traj，shape=(N, nq)。
    验证：如果 expected_nq 不为 None，则第二维必须等于 expected_nq。
    """
    q_traj = np.load(Path(path))
    validate_q_traj(q_traj, expected_nq)
    return q_traj


def save_q_traj(path: str | Path, q_traj: np.ndarray) -> None:
    """保存 q trajectory，不生成控制命令。"""
    validate_q_traj(q_traj, None)
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.save(output_path, q_traj)


def validate_q_traj(q_traj: np.ndarray, expected_nq: int | None) -> None:
    """验证 q trajectory shape。

    TODO 学习重点：A04/A05 输出的是 IK 轨迹；A07 才消费它进行 actuator tracking。
    """
    array = np.asarray(q_traj)
    if array.ndim != 2:
        raise ValueError(f"q_traj must be 2D with shape (N, nq), got {array.shape}")
    if array.shape[0] <= 0:
        raise ValueError("q_traj must contain at least one waypoint")
    if expected_nq is not None and array.shape[1] != expected_nq:
        raise ValueError(f"q_traj second dimension must be {expected_nq}, got {array.shape[1]}")


def _save_rows(path: str | Path, rows: list[dict[str, Any]]) -> None:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        output_path.write_text("", encoding="utf-8")
        return
    fieldnames = list(rows[0].keys())
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def save_error_log(path: str | Path, rows: list[dict[str, Any]]) -> None:
    """保存 IK error log。"""
    _save_rows(path, rows)


def save_constraint_log(path: str | Path, rows: list[dict[str, Any]]) -> None:
    """保存 QP constraint log。"""
    _save_rows(path, rows)


def make_trajectory_source(name: str, path: str | Path, source_step: str) -> TrajectorySource:
    """创建 TrajectorySource metadata。

    如果文件存在，会读取 shape；如果不存在，shape 保持 None，供 TODO skeleton 报告。
    """
    traj_path = Path(path)
    shape = None
    if traj_path.exists():
        shape = tuple(int(item) for item in np.load(traj_path).shape)
    return TrajectorySource(name=name, path=str(traj_path), source_step=source_step, shape=shape, description=f"{source_step} trajectory source")
