"""Pytest configuration for B_mujoco_mpc_study tests."""

from __future__ import annotations

import os

import pytest


# 测试环境通常没有稳定的交互式 OpenGL 窗口上下文；默认使用 EGL 做 MuJoCo 离屏渲染。
# 这只影响 pytest 进程，不改变控制器、MPC 或 rollout 的数学逻辑。
os.environ.setdefault("MUJOCO_GL", "egl")


def pytest_configure(config: pytest.Config) -> None:
    config.addinivalue_line("markers", "mujoco: test requires mujoco package")
