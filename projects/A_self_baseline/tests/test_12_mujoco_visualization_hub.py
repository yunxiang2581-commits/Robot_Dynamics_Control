"""A12 MuJoCo visualization hub 的非图形逻辑测试。

这些测试不打开 MuJoCo viewer，只检查：
- A 项目和 mink example 的路径能稳定解析。
- inspect mode 能生成模型摘要。
- mink example mode 会准备正确的 Python 子进程环境。
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_PATH = (
    REPO_ROOT
    / "projects"
    / "A_self_baseline"
    / "scripts"
    / "12_mujoco_visualization_hub.py"
)


def load_hub_script():
    spec = importlib.util.spec_from_file_location("mujoco_visualization_hub", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_resolve_a_project_mjcf_from_robot_yaml():
    script = load_hub_script()

    mjcf_path = script.resolve_a_project_mjcf(script.DEFAULT_CONFIG, mjcf_override=None)

    assert mjcf_path.exists()
    assert mjcf_path.name == "scene.xml"
    assert mjcf_path.parent.name == "mink_universal_robots_ur5e"


def test_resolve_mink_example_script_for_ur5e_modes():
    script = load_hub_script()

    plain = script.resolve_mink_example_script("mink_ur5e")
    actuator = script.resolve_mink_example_script("mink_ur5e_actuator")

    assert plain == script.MINK_EXAMPLES_ROOT / "arm_ur5e.py"
    assert actuator == script.MINK_EXAMPLES_ROOT / "arm_ur5e_actuators.py"
    assert plain.exists()
    assert actuator.exists()


def test_build_mink_example_environment_prepends_mink_src():
    script = load_hub_script()
    env = {"PYTHONPATH": "existing_path", "OTHER": "1"}

    new_env = script.build_mink_example_environment(env)

    parts = new_env["PYTHONPATH"].split(script.OS_PATHSEP)
    assert parts[0] == str(script.MINK_SRC_ROOT)
    assert "existing_path" in parts
    assert new_env["OTHER"] == "1"


def test_collect_model_summary_contains_key_dimensions():
    script = load_hub_script()
    mjcf_path = script.resolve_a_project_mjcf(script.DEFAULT_CONFIG, mjcf_override=None)
    model, data = script.load_model_and_data(mjcf_path)

    summary = script.collect_model_summary(
        label="a_project",
        mjcf_path=mjcf_path,
        model=model,
        data=data,
        keyframe_name="home",
        keyframe_applied=False,
    )

    assert summary["label"] == "a_project"
    assert summary["mjcf_path"] == str(mjcf_path)
    assert summary["nq"] == model.nq
    assert summary["nv"] == model.nv
    assert summary["nu"] == model.nu
    assert "attachment_site" in summary["site_names"]


def test_load_q_trajectory_accepts_default_a05_file():
    script = load_hub_script()

    trajectory = script.load_q_trajectory(script.DEFAULT_A05_TRAJECTORY, expected_nq=6)

    assert trajectory.ndim == 2
    assert trajectory.shape[1] == 6


def test_validate_q_trajectory_rejects_wrong_shape():
    script = load_hub_script()
    bad_trajectory = np.zeros((3, 5))

    try:
        script.validate_q_trajectory(bad_trajectory, expected_nq=6, source="bad")
    except ValueError as exc:
        assert "expected shape (T, 6)" in str(exc)
    else:
        raise AssertionError("Expected ValueError for wrong trajectory shape")


def test_apply_q_frame_writes_data_qpos_and_runs_forward():
    script = load_hub_script()
    mjcf_path = script.resolve_a_project_mjcf(script.DEFAULT_CONFIG, mjcf_override=None)
    model, data = script.load_model_and_data(mjcf_path)
    q = np.array([-1.0, -1.1, 1.2, -1.3, -1.4, 0.2])

    script.apply_q_frame(model, data, q)

    np.testing.assert_allclose(data.qpos, q)


def test_replay_a05_mode_is_available_in_parser(monkeypatch):
    script = load_hub_script()
    monkeypatch.setattr("sys.argv", ["hub", "--mode", "replay_a05"])

    args = script.parse_args()

    assert args.mode == "replay_a05"
