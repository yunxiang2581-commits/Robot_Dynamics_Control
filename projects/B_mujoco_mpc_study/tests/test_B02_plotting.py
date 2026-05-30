"""Tests for save_b02_tracking_figures (all 6 B02 figures)."""

from __future__ import annotations

from pathlib import Path
import sys
import uuid

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from projects.B_mujoco_mpc_study.simulator.utils.plotting import save_b02_tracking_figures


TEMP_DIR = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "outputs" / "pytest_tmp"

TRACKING_ROWS = [
    {
        "time": 0.01,
        "target_x": 0.5,
        "target_y": 0.2,
        "actual_x": 0.48,
        "actual_y": 0.19,
        "error_norm": 0.03,
        "u1": 0.1,
        "u2": -0.1,
        "mpc_cost": 1.0,
    },
    {
        "time": 0.02,
        "target_x": 0.52,
        "target_y": 0.21,
        "actual_x": 0.50,
        "actual_y": 0.20,
        "error_norm": 0.025,
        "u1": 0.08,
        "u2": -0.05,
        "mpc_cost": 0.9,
    },
    {
        "time": 0.03,
        "target_x": 0.51,
        "target_y": 0.22,
        "actual_x": 0.505,
        "actual_y": 0.215,
        "error_norm": 0.01,
        "u1": 0.05,
        "u2": -0.02,
        "mpc_cost": 0.5,
    },
]


def _make_outputs(*keys: str) -> dict[str, Path]:
    TEMP_DIR.mkdir(parents=True, exist_ok=True)
    return {key: TEMP_DIR / f"{key}_{uuid.uuid4().hex}.png" for key in keys}


def _cleanup(outputs: dict[str, Path]) -> None:
    for path in outputs.values():
        if path.exists():
            path.unlink()


def test_save_b02_tracking_figures_original_three() -> None:
    outputs = _make_outputs("xy_target_vs_actual", "tracking_error_time", "control_input_time")
    try:
        save_b02_tracking_figures(tracking_rows=TRACKING_ROWS, outputs=outputs)
        for path in outputs.values():
            assert path.exists(), f"missing: {path.name}"
            assert path.stat().st_size > 0, f"empty: {path.name}"
    finally:
        _cleanup(outputs)


def test_save_b02_tracking_figures_new_three() -> None:
    outputs = _make_outputs("ee_trajectory_xy", "ee_tracking_error", "joint_torque")
    try:
        save_b02_tracking_figures(tracking_rows=TRACKING_ROWS, outputs=outputs)
        for path in outputs.values():
            assert path.exists(), f"missing: {path.name}"
            assert path.stat().st_size > 0, f"empty: {path.name}"
    finally:
        _cleanup(outputs)


def test_save_b02_tracking_figures_all_six_at_once() -> None:
    all_keys = [
        "xy_target_vs_actual",
        "tracking_error_time",
        "control_input_time",
        "ee_trajectory_xy",
        "ee_tracking_error",
        "joint_torque",
    ]
    outputs = _make_outputs(*all_keys)
    try:
        save_b02_tracking_figures(tracking_rows=TRACKING_ROWS, outputs=outputs)
        for key, path in outputs.items():
            assert path.exists(), f"missing: {key}"
            assert path.stat().st_size > 0, f"empty: {key}"
    finally:
        _cleanup(outputs)


def test_save_b02_tracking_figures_skips_missing_keys() -> None:
    outputs = _make_outputs("xy_target_vs_actual")
    try:
        save_b02_tracking_figures(tracking_rows=TRACKING_ROWS, outputs=outputs)
        assert outputs["xy_target_vs_actual"].exists()
    finally:
        _cleanup(outputs)


def test_save_b02_tracking_figures_rejects_empty_rows() -> None:
    with pytest.raises(ValueError, match="不能为空"):
        save_b02_tracking_figures(tracking_rows=[], outputs=_make_outputs("xy_target_vs_actual"))
