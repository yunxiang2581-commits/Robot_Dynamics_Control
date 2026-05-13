from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SIMULATOR_ROOT = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "simulator"
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

from utils.solver_comparison_visualizer import (
    compute_control_smoothness,
    compute_trajectory_smoothness,
)


def test_compute_control_smoothness() -> None:
    controls = np.array(
        [
            [0.0, 0.0],
            [1.0, 0.0],
            [1.0, 1.0],
        ],
        dtype=float,
    )

    smoothness = compute_control_smoothness(controls)

    expected = (1.0 + 1.0) / 2.0
    assert smoothness == pytest.approx(expected)


def test_compute_trajectory_smoothness() -> None:
    points = np.array(
        [
            [0.0, 0.0],
            [1.0, 0.0],
            [3.0, 0.0],
            [6.0, 0.0],
        ],
        dtype=float,
    )

    smoothness = compute_trajectory_smoothness(points)

    expected = (np.linalg.norm(np.array([1.0, 0.0])) + np.linalg.norm(np.array([1.0, 0.0]))) / 2.0
    assert smoothness == pytest.approx(expected)


def test_short_sequence_edge_cases() -> None:
    assert compute_control_smoothness(np.zeros((0, 2))) == pytest.approx(0.0)
    assert compute_control_smoothness(np.zeros((1, 2))) == pytest.approx(0.0)
    assert compute_trajectory_smoothness(np.zeros((0, 2))) == pytest.approx(0.0)
    assert compute_trajectory_smoothness(np.zeros((2, 2))) == pytest.approx(0.0)
