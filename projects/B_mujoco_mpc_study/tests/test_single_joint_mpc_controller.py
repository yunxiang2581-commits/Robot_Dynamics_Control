from __future__ import annotations

from pathlib import Path
import sys
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[3]
SIMULATOR_ROOT = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "simulator"
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

from controllers.single_joint_mpc_controller import SingleJointMPCController


class FakePlanner:
    def plan(self, env: Any, current_state: tuple[float, float], q_target: float) -> dict[str, Any]:
        return {
            "best_sequence": [0.1, 0.2],
            "best_torque": 0.7,
            "best_cost": 3.0,
            "q_target": q_target,
            "current_state": current_state,
        }


def test_compute_control_uses_planner_best_torque_and_saves_plan() -> None:
    controller = SingleJointMPCController(planner=FakePlanner(), q_target=1.25)

    tau = controller.compute_control(env=object(), current_state=(0.0, 0.0))

    assert tau == 0.7
    assert controller.last_plan is not None
    assert controller.last_plan["best_cost"] == 3.0
    assert controller.last_plan["q_target"] == 1.25
