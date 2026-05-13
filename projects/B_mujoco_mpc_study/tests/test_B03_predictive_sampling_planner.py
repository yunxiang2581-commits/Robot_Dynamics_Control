from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SIMULATOR_ROOT = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "simulator"
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

from planners.predictive_sampling_planner import rank_rollouts, sample_candidate_controls


def test_sample_candidate_controls_returns_expected_shape() -> None:
    batch = sample_candidate_controls(
        num_candidates=7,
        horizon=5,
        control_dim=2,
        sampling_std=0.4,
        torque_limit=1.5,
        seed=123,
    )

    assert batch.controls.shape == (7, 5, 2)
    assert batch.num_candidates == 7
    assert batch.horizon == 5
    assert batch.control_dim == 2


def test_sample_candidate_controls_clips_to_torque_limit() -> None:
    batch = sample_candidate_controls(
        num_candidates=20,
        horizon=4,
        control_dim=2,
        sampling_std=100.0,
        torque_limit=0.75,
        seed=3,
    )

    assert not np.isnan(batch.controls).any()
    assert float(np.max(np.abs(batch.controls))) <= 0.75 + 1e-12


def test_sample_candidate_controls_is_reproducible_with_seed() -> None:
    first = sample_candidate_controls(
        num_candidates=6,
        horizon=3,
        control_dim=2,
        sampling_std=0.8,
        torque_limit=2.0,
        seed=42,
    )
    second = sample_candidate_controls(
        num_candidates=6,
        horizon=3,
        control_dim=2,
        sampling_std=0.8,
        torque_limit=2.0,
        seed=42,
    )

    np.testing.assert_allclose(first.controls, second.controls)


def test_rank_rollouts_selects_min_cost() -> None:
    result = rank_rollouts([4.0, 1.25, 3.0, 2.0])

    assert result.selected_index == 1
    assert result.sorted_indices == [1, 3, 2, 0]
    assert result.best_cost == pytest.approx(1.25)
    assert result.min_cost == pytest.approx(1.25)
    assert result.max_cost == pytest.approx(4.0)
    assert result.mean_cost == pytest.approx(2.5625)


def test_rank_rollouts_rejects_nan() -> None:
    with pytest.raises(ValueError, match="NaN"):
        rank_rollouts([1.0, float("nan"), 2.0])
