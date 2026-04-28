"""Pinocchio model loading helpers for the A self baseline project."""

from __future__ import annotations

from typing import Any


def load_pinocchio_model(urdf_path: str, package_dirs: list[str] | None = None) -> Any:
    """Load a Pinocchio model from URDF.

    TODO(中文):
    - 要实现什么: 根据 urdf_path 和 package_dirs 构建 Pinocchio model/data。
    - 输入是什么: urdf_path 是 URDF 路径, package_dirs 是 mesh/package 搜索目录。
    - 输出是什么: Pinocchio model, 后续可同时返回 data 或由调用方创建。
    - 求职重要性: 正确加载模型是 FK、Jacobian、IK、WBC 的共同前提。
    - 推荐 API: pin.buildModelFromUrdf, pin.JointModelFreeFlyer, model.createData。
    - 如何验证: 打印 nq、nv、joint 数量和 frame 数量, 与 URDF 预期一致。
    """
    raise NotImplementedError("TODO: load Pinocchio model from URDF.")


def summarize_model(model: Any) -> dict[str, Any]:
    """Return a structured summary of a Pinocchio model.

    TODO(中文):
    - 要实现什么: 汇总 nq、nv、关节名、frame 名、根关节类型等信息。
    - 输入是什么: 已加载的 Pinocchio model。
    - 输出是什么: 可写入日志或 JSON 的 dict 摘要。
    - 求职重要性: 能快速定位模型自由度和 frame 命名问题。
    - 推荐 API: model.nq, model.nv, model.names, model.frames。
    - 如何验证: 与 01_inspect_urdf.py 输出交叉检查。
    """
    raise NotImplementedError("TODO: summarize Pinocchio model.")
