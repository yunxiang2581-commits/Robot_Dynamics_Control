"""Metrics and output helpers for learning scripts."""

from __future__ import annotations

from pathlib import Path
from typing import Any


def compute_norm_error(actual: Any, expected: Any) -> float:
    """Compute a norm error.

    TODO(中文):
    - 要实现什么: 计算 actual 与 expected 的向量范数误差。
    - 输入是什么: 实际值 actual 和期望值 expected。
    - 输出是什么: float 误差。
    - 求职重要性: 误差度量是验证 FK/Jacobian/IK/控制结果的通用工具。
    - 推荐 API: numpy.asarray, numpy.linalg.norm。
    - 如何验证: 相同输入误差为 0, 已知差值能得到预期范数。
    """
    raise NotImplementedError("TODO: compute norm error.")


def save_curve_csv(path: str | Path, rows: list[dict[str, Any]]) -> None:
    """Save learning curves to CSV.

    TODO(中文):
    - 要实现什么: 将误差曲线或跟踪曲线保存为 CSV。
    - 输入是什么: 输出路径 path 和行数据 rows。
    - 输出是什么: CSV 文件。
    - 求职重要性: 可复现实验需要日志, 不是只看终端输出。
    - 推荐 API: csv.DictWriter, pathlib.Path.mkdir。
    - 如何验证: 文件存在, 表头正确, 行数与输入 rows 一致。
    """
    raise NotImplementedError("TODO: save curve CSV.")


def plot_curve_placeholder(path: str | Path, rows: list[dict[str, Any]]) -> None:
    """Placeholder for plotting learning curves.

    TODO(中文):
    - 要实现什么: 后续用 matplotlib 绘制误差或跟踪曲线。
    - 输入是什么: 输出图片路径和曲线数据 rows。
    - 输出是什么: PNG 图像。
    - 求职重要性: 图像能快速说明控制效果和收敛过程。
    - 推荐 API: matplotlib.pyplot。
    - 如何验证: PNG 文件可打开, 坐标轴和曲线含义清楚。
    """
    raise NotImplementedError("TODO: plot curve placeholder.")
