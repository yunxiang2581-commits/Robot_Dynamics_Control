"""A self baseline robotics learning package.

当前结构：
- A01-A03 保留为基础验证层，继续使用 ``model_loader``。
- A04-A07 收口到四接口骨架：Target / IK / Viewer / Actuator。
- 当前只定义 schema 和 TODO learning skeleton，不调用 mink，不启动 viewer，
  不写 ``data.ctrl``。
"""

__all__ = [
    "actuator_interface",
    "ik_interface",
    "model_loader",
    "motion_types",
    "target_interface",
    "trajectory_io",
    "viewer_interface",
]
