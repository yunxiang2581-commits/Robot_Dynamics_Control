"""Forward kinematics helpers for frame pose queries."""

from __future__ import annotations

from typing import Any


def compute_frame_pose(model: Any, data: Any, q: Any, frame_name: str) -> Any:
    """Compute the SE3 pose of a frame.

    TODO(中文):
    - 要实现什么: 对 q 执行正运动学并返回 frame_name 对应的世界位姿。
    - 输入是什么: model、data、关节配置 q、目标 frame_name。
    - 输出是什么: Pinocchio SE3 或结构化 position/rotation。
    - 求职重要性: FK 是任务空间控制和传感器 frame 调试的基础。
    - 推荐 API: pin.forwardKinematics, pin.updateFramePlacements, model.getFrameId, data.oMf。
    - 如何验证: 用中性位姿和单关节扰动检查 frame 位姿变化是否合理。
    """
    raise NotImplementedError("TODO: compute frame pose.")


def list_candidate_frames(model: Any, keywords: list[str]) -> list[str]:
    """List frames matching one or more keywords.

    TODO(中文):
    - 要实现什么: 根据关键词筛选可能的脚端、膝关节或 base frame 名称。
    - 输入是什么: Pinocchio model 和关键词列表。
    - 输出是什么: 匹配到的 frame 名称列表。
    - 求职重要性: 实际机器人模型 frame 命名不统一, 会影响 IK/Jacobian 选点。
    - 推荐 API: [frame.name for frame in model.frames]。
    - 如何验证: 给定 foot、knee、base 等关键词时能列出合理候选。
    """
    raise NotImplementedError("TODO: list candidate frames.")
