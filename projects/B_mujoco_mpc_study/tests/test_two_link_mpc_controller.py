from __future__ import annotations

from pathlib import Path
import sys
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[3]
SIMULATOR_ROOT = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "simulator"
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

from controllers.two_link_mpc_controller import TwoLinkMPCController


class FakeTwoLinkPlanner:
    def plan(
        self,
        env: Any,
        current_state: tuple[float, float, float, float],
        target_sequence: list[tuple[float, float]],
    ) -> dict[str, Any]:
        return {
            "best_sequence": [(0.1, -0.2), (0.0, 0.0)],
            "best_torque": (0.7, -0.4),
            "best_cost": 2.5,
            "current_state": current_state,
            "target_sequence": target_sequence,
        }


def test_compute_control_uses_planner_best_torque_and_saves_plan() -> None:
    controller = TwoLinkMPCController(planner=FakeTwoLinkPlanner())
    current_state = (0.0, 0.1, 0.0, -0.1)
    target_sequence = [(0.8, 0.0), (0.75, 0.1)]

    torque = controller.compute_control(
        env=object(),
        current_state=current_state,
        target_sequence=target_sequence,
    )

    assert torque == (0.7, -0.4)
    assert controller.last_plan is not None
    assert controller.last_plan["best_cost"] == 2.5
    assert controller.last_plan["current_state"] == current_state
    assert controller.last_plan["target_sequence"] == target_sequence


def test_compute_control_requires_best_torque_from_planner() -> None:
    class MissingBestTorquePlanner:
        def plan(
            self,
            env: Any,
            current_state: tuple[float, float, float, float],
            target_sequence: list[tuple[float, float]],
        ) -> dict[str, Any]:
            return {"best_cost": 1.0}

    controller = TwoLinkMPCController(planner=MissingBestTorquePlanner())

    try:
        controller.compute_control(
            env=object(),
            current_state=(0.0, 0.0, 0.0, 0.0),
            target_sequence=[(0.8, 0.0)],
        )
    except KeyError as exc:
        assert "best_torque" in str(exc)
    else:
        raise AssertionError("compute_control should require planner best_torque")
