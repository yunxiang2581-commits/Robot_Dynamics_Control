from __future__ import annotations

from pathlib import Path
import math
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SIMULATOR_ROOT = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "simulator"
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

from envs.two_link_env import TwoLinkEnv


def test_two_link_env_initializes_default_model_and_respects_dt() -> None:
    env = TwoLinkEnv(dt=0.02)

    assert env.model is not None
    assert env.data is not None
    assert env.model_path == SIMULATOR_ROOT / "models" / "B02_two_link.xml"
    assert env.model.opt.timestep == 0.02
    assert env.get_state() == (0.0, 0.0, 0.0, 0.0)
    assert env.get_end_effector_position() == pytest.approx((0.8, 0.0), abs=1e-6)


def test_two_link_env_reset_set_state_and_step_keep_finite_state() -> None:
    env = TwoLinkEnv(dt=0.01)

    assert env.reset(q=(0.1, -0.2), dq=(0.3, -0.4)) == pytest.approx((0.1, -0.2, 0.3, -0.4))

    env.set_state((0.2, 0.1, 0.0, 0.0))
    next_state = env.step((100.0, -100.0))

    assert env.data.ctrl[0] == pytest.approx(10.0)
    assert env.data.ctrl[1] == pytest.approx(-10.0)
    assert env.get_last_applied_torque() == pytest.approx((10.0, -10.0))
    assert len(next_state) == 4
    assert all(math.isfinite(value) for value in next_state)


def test_two_link_rollout_returns_horizon_plus_one_and_restores_state() -> None:
    env = TwoLinkEnv(dt=0.01)
    env.set_state((0.3, -0.1, 0.2, -0.2))
    real_state = env.get_state()

    states = env.rollout(
        initial_state=(0.0, 0.0, 0.0, 0.0),
        torque_sequence=[(0.5, -0.2), (0.1, 0.3), (0.0, 0.0)],
    )

    assert len(states) == 4
    assert states[0] == (0.0, 0.0, 0.0, 0.0)
    assert env.get_state() == pytest.approx(real_state)


def test_render_frame_with_scene_geoms_returns_rgb_array() -> None:
    env = TwoLinkEnv(dt=0.01)
    env.reset(q=(0.3, 0.4), dq=(0.0, 0.0))

    frame = env.render_frame_with_scene_geoms(
        [
            {
                "geom_type": "sphere",
                "pos": (0.4, 0.2, 0.0),
                "size": (0.02, 0.02, 0.02),
                "rgba": (1.0, 0.0, 0.0, 1.0),
            }
        ]
    )

    assert frame.ndim == 3
    assert frame.shape[2] == 3
