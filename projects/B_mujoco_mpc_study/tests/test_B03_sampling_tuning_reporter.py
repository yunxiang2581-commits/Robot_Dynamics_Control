from __future__ import annotations

import math
from pathlib import Path
import sys
from uuid import uuid4

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SIMULATOR_ROOT = REPO_ROOT / "projects" / "B_mujoco_mpc_study" / "simulator"
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

from utils.sampling_tuning_reporter import (
    SWEEP_METRICS_COLUMNS,
    compute_tuning_score,
    save_sweep_metrics_csv,
    select_best_variants,
    write_tuning_report,
)


def _workspace_tmp_dir() -> Path:
    tmp_dir = REPO_ROOT / ".pytest_tmp" / f"B03_tuning_reporter_{uuid4().hex}"
    tmp_dir.mkdir(parents=True, exist_ok=True)
    return tmp_dir


# ---------------------------------------------------------------------------
# compute_tuning_score
# ---------------------------------------------------------------------------

def test_compute_tuning_score_feasible() -> None:
    row = {
        "mean_ee_error": 0.02,
        "mean_runtime_ms": 100.0,
        "control_smoothness": 0.1,
        "trajectory_smoothness": 0.05,
        "success_rate": 1.0,
    }
    weights = {
        "mean_ee_error_weight": 1.0,
        "runtime_weight": 0.2,
        "control_smoothness_weight": 0.5,
        "trajectory_smoothness_weight": 0.5,
    }
    constraints = {
        "max_mean_runtime_ms": 500.0,
        "max_mean_ee_error": 0.05,
        "min_success_rate": 0.95,
    }

    score, is_feasible, reason = compute_tuning_score(row, weights, constraints)

    assert is_feasible is True
    assert reason == "feasible"
    assert math.isfinite(score)
    assert score > 0


def test_compute_tuning_score_infeasible_runtime() -> None:
    row = {
        "mean_ee_error": 0.02,
        "mean_runtime_ms": 600.0,
        "control_smoothness": 0.1,
        "trajectory_smoothness": 0.05,
        "success_rate": 1.0,
    }
    weights = {
        "mean_ee_error_weight": 1.0,
        "runtime_weight": 0.2,
        "control_smoothness_weight": 0.5,
        "trajectory_smoothness_weight": 0.5,
    }
    constraints = {
        "max_mean_runtime_ms": 500.0,
        "max_mean_ee_error": 0.05,
        "min_success_rate": 0.95,
    }

    score, is_feasible, reason = compute_tuning_score(row, weights, constraints)

    assert is_feasible is False
    assert score == float("inf")
    assert "mean_runtime_ms" in reason


def test_compute_tuning_score_infeasible_error() -> None:
    row = {
        "mean_ee_error": 0.10,
        "mean_runtime_ms": 100.0,
        "control_smoothness": 0.1,
        "trajectory_smoothness": 0.05,
        "success_rate": 1.0,
    }
    weights = {
        "mean_ee_error_weight": 1.0,
        "runtime_weight": 0.2,
        "control_smoothness_weight": 0.5,
        "trajectory_smoothness_weight": 0.5,
    }
    constraints = {
        "max_mean_runtime_ms": 500.0,
        "max_mean_ee_error": 0.05,
        "min_success_rate": 0.95,
    }

    score, is_feasible, reason = compute_tuning_score(row, weights, constraints)

    assert is_feasible is False
    assert score == float("inf")
    assert "mean_ee_error" in reason


def test_compute_tuning_score_infeasible_success_rate() -> None:
    row = {
        "mean_ee_error": 0.02,
        "mean_runtime_ms": 100.0,
        "control_smoothness": 0.1,
        "trajectory_smoothness": 0.05,
        "success_rate": 0.80,
    }
    weights = {
        "mean_ee_error_weight": 1.0,
        "runtime_weight": 0.2,
        "control_smoothness_weight": 0.5,
        "trajectory_smoothness_weight": 0.5,
    }
    constraints = {
        "max_mean_runtime_ms": 500.0,
        "max_mean_ee_error": 0.05,
        "min_success_rate": 0.95,
    }

    score, is_feasible, reason = compute_tuning_score(row, weights, constraints)

    assert is_feasible is False
    assert score == float("inf")
    assert "success_rate" in reason


def test_compute_tuning_score_handles_nan() -> None:
    row = {
        "mean_ee_error": float("nan"),
        "mean_runtime_ms": 100.0,
        "control_smoothness": 0.1,
        "trajectory_smoothness": 0.05,
        "success_rate": 1.0,
    }
    weights = {
        "mean_ee_error_weight": 1.0,
        "runtime_weight": 0.2,
        "control_smoothness_weight": 0.5,
        "trajectory_smoothness_weight": 0.5,
    }
    constraints = {
        "max_mean_runtime_ms": 500.0,
        "max_mean_ee_error": 0.05,
        "min_success_rate": 0.95,
    }

    score, is_feasible, reason = compute_tuning_score(row, weights, constraints)

    assert is_feasible is False
    assert score == float("inf")


def test_compute_tuning_score_handles_missing_fields() -> None:
    row: dict = {}
    weights: dict = {}
    constraints: dict = {}

    score, is_feasible, reason = compute_tuning_score(row, weights, constraints)

    # success_rate defaults to NaN, which is < 0.95 -> infeasible
    assert is_feasible is False
    assert score == float("inf")


# ---------------------------------------------------------------------------
# select_best_variants
# ---------------------------------------------------------------------------

def test_select_best_variants_returns_family_best() -> None:
    scored_rows = [
        {
            "variant_name": "cem_fast",
            "solver_family": "cem",
            "score": 0.5,
            "is_feasible": True,
        },
        {
            "variant_name": "cem_stable",
            "solver_family": "cem",
            "score": 0.3,
            "is_feasible": True,
        },
        {
            "variant_name": "mppi_temp_1p0",
            "solver_family": "mppi_lite",
            "score": 0.4,
            "is_feasible": True,
        },
        {
            "variant_name": "mppi_temp_5p0",
            "solver_family": "mppi_lite",
            "score": 0.6,
            "is_feasible": True,
        },
        {
            "variant_name": "warm_start_default",
            "solver_family": "warm_start_sampling",
            "score": 0.45,
            "is_feasible": True,
        },
    ]

    best = select_best_variants(scored_rows)

    assert best["best_overall"] is not None
    assert best["best_overall"]["variant_name"] == "cem_stable"
    assert best["best_cem"] is not None
    assert best["best_cem"]["variant_name"] == "cem_stable"
    assert best["best_mppi"] is not None
    assert best["best_mppi"]["variant_name"] == "mppi_temp_1p0"
    assert best["best_warm_start"] is not None
    assert best["best_warm_start"]["variant_name"] == "warm_start_default"


def test_select_best_variants_all_infeasible() -> None:
    scored_rows = [
        {
            "variant_name": "cem_fast",
            "solver_family": "cem",
            "score": float("inf"),
            "is_feasible": False,
        },
    ]

    best = select_best_variants(scored_rows)

    assert best["best_overall"] is None
    assert best["best_cem"] is None
    assert best["best_mppi"] is None
    assert best["best_warm_start"] is None


def test_select_best_variants_partial_families() -> None:
    scored_rows = [
        {
            "variant_name": "cem_default",
            "solver_family": "cem",
            "score": 0.4,
            "is_feasible": True,
        },
    ]

    best = select_best_variants(scored_rows)

    assert best["best_overall"] is not None
    assert best["best_cem"] is not None
    assert best["best_mppi"] is None
    assert best["best_warm_start"] is None


# ---------------------------------------------------------------------------
# write_tuning_report
# ---------------------------------------------------------------------------

def test_write_tuning_report_creates_markdown() -> None:
    tmp_dir = _workspace_tmp_dir()
    output_path = tmp_dir / "reports" / "tuning_report.md"

    scored_rows = [
        {
            "variant_name": "cem_default",
            "solver_family": "cem",
            "mean_ee_error": 0.02,
            "mean_runtime_ms": 100.0,
            "control_smoothness": 0.1,
            "trajectory_smoothness": 0.05,
            "success_rate": 1.0,
            "score": 0.3,
            "is_feasible": True,
            "feasibility_reason": "feasible",
        },
        {
            "variant_name": "mppi_temp_1p0",
            "solver_family": "mppi_lite",
            "mean_ee_error": 0.03,
            "mean_runtime_ms": 120.0,
            "control_smoothness": 0.15,
            "trajectory_smoothness": 0.08,
            "success_rate": 1.0,
            "score": 0.45,
            "is_feasible": True,
            "feasibility_reason": "feasible",
        },
    ]
    best_variants = {
        "best_overall": scored_rows[0],
        "best_cem": scored_rows[0],
        "best_mppi": scored_rows[1],
        "best_warm_start": None,
    }
    weights = {
        "mean_ee_error_weight": 1.0,
        "runtime_weight": 0.2,
        "control_smoothness_weight": 0.5,
        "trajectory_smoothness_weight": 0.5,
    }
    constraints = {
        "max_mean_runtime_ms": 500.0,
        "max_mean_ee_error": 0.05,
        "min_success_rate": 0.95,
    }

    write_tuning_report(scored_rows, best_variants, weights, constraints, output_path)

    assert output_path.exists()
    content = output_path.read_text(encoding="utf-8")
    assert "# B03-R3T Sampling Solver Parameter Tuning Report" in content
    assert "cem_default" in content
    assert "mppi_temp_1p0" in content
    assert "Best Variants" in content


# ---------------------------------------------------------------------------
# save_sweep_metrics_csv
# ---------------------------------------------------------------------------

def test_save_sweep_metrics_csv_creates_file() -> None:
    tmp_dir = _workspace_tmp_dir()
    output_path = tmp_dir / "metrics" / "sweep.csv"

    scored_rows = [
        {
            "variant_name": "cem_default",
            "solver_family": "cem",
            "mean_ee_error": 0.02,
            "max_ee_error": 0.04,
            "mean_runtime_ms": 100.0,
            "max_runtime_ms": 150.0,
            "control_smoothness": 0.1,
            "trajectory_smoothness": 0.05,
            "success_rate": 1.0,
            "weight_entropy": 0.0,
            "max_weight": 0.0,
            "score": 0.3,
            "is_feasible": True,
            "feasibility_reason": "feasible",
        },
    ]

    save_sweep_metrics_csv(scored_rows, output_path)

    assert output_path.exists()
    lines = output_path.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == 2  # header + 1 row
    header = lines[0]
    for col in SWEEP_METRICS_COLUMNS:
        assert col in header


def test_save_sweep_metrics_csv_empty_list() -> None:
    tmp_dir = _workspace_tmp_dir()
    output_path = tmp_dir / "metrics" / "empty_sweep.csv"

    save_sweep_metrics_csv([], output_path)

    assert output_path.exists()
    lines = output_path.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == 1  # header only
