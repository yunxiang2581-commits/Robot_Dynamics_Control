from __future__ import annotations

from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[3]
SIMULATOR_ROOT = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "simulator"
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

from envs.single_joint_env import SingleJointEnv


def test_env_initializes_default_model_and_respects_dt() -> None:
    env = SingleJointEnv(dt=0.02)

    assert env.model is not None
    assert env.data is not None
    assert env.model_path == SIMULATOR_ROOT / "models" / "B01_single_joint.xml"
    assert env.model.opt.timestep == 0.02
    assert env.get_state() == (0.0, 0.0)
