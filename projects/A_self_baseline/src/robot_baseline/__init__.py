"""A self baseline robotics learning package.

当前最小化保留：只暴露 A01-A03 已实际使用的 model_loader。
后续如果重新需要 kinematics / IK / control helper，再按学习步骤增量创建。
"""

__all__ = [
    "model_loader",
]
