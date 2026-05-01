"""A11 UR5e X11 viewer 脚本的非图形逻辑测试。

这些测试不打开 MuJoCo 窗口，只检查：
- robot.yaml 中 MJCF 路径能被稳定解析。
- X11 环境变量会在 import mujoco 之前准备好。
- 可选 keyframe 能正确写入 data.qpos。
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_PATH = REPO_ROOT / "projects" / "A_self_baseline" / "scripts" / "11_view_ur5e_x11.py"


def load_viewer_script():
    spec = importlib.util.spec_from_file_location("view_ur5e_x11", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_resolve_mjcf_path_uses_robot_yaml_relative_to_a_root():
    script = load_viewer_script()

    mjcf_path = script.resolve_mjcf_path(config_path=script.DEFAULT_CONFIG, mjcf_override=None)

    assert mjcf_path.exists()
    assert mjcf_path.name == "scene.xml"
    assert mjcf_path.parent.name == "mink_universal_robots_ur5e"


def test_build_x11_environment_removes_wayland_and_sets_x11():
    script = load_viewer_script()
    env = {
        "DISPLAY": ":0",
        "WAYLAND_DISPLAY": "wayland-0",
        "XDG_SESSION_TYPE": "wayland",
    }

    new_env, needs_reexec = script.build_x11_environment(env)

    assert needs_reexec is True
    assert new_env["XDG_SESSION_TYPE"] == "x11"
    assert new_env["GDK_BACKEND"] == "x11"
    assert new_env["QT_QPA_PLATFORM"] == "xcb"
    assert "WAYLAND_DISPLAY" not in new_env
    assert env["WAYLAND_DISPLAY"] == "wayland-0"


def test_apply_keyframe_writes_home_qpos():
    script = load_viewer_script()
    mjcf_path = script.resolve_mjcf_path(config_path=script.DEFAULT_CONFIG, mjcf_override=None)
    model, data = script.load_model_and_data(mjcf_path)

    applied = script.apply_keyframe(model, data, "home")

    home_id = script.find_keyframe_id(model, "home")
    assert applied is True
    assert home_id is not None
    np.testing.assert_allclose(data.qpos, model.key_qpos[home_id])
