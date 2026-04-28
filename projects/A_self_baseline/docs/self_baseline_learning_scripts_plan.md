# Self Baseline Learning Scripts Plan

## 1. Step 5 Goal

Create standard TODO learning script entries, source module skeletons, configuration templates, and documentation for the A self-developed robot motion control baseline.

The current step creates teaching scaffolds only. Core FK, Jacobian, IK, QP-IK, MuJoCo control, and Mini-WBC logic remains TODO.

## 2. Why Seven Standard Learning Scripts

The seven scripts follow a practical robotics learning sequence:

```text
URDF inspection -> FK -> Jacobian validation -> DLS IK -> QP-IK -> MuJoCo PD -> Mini-WBC QP
```

This sequence turns legacy one-off scripts into stable learning entries that can be implemented, tested, and reviewed one topic at a time.

## 3. Script Learning Goals

- `01_inspect_urdf.py`: inspect model nq, nv, joints, and frames.
- `02_fk_frame_pose.py`: compute a target frame pose with forward kinematics.
- `03_jacobian_fd_check.py`: compute a frame Jacobian and verify it with finite differences.
- `04_dls_ik_demo.py`: build a Damped Least Squares IK learning loop.
- `05_qp_ik_joint_limit_demo.py`: introduce constrained QP-IK with joint velocity and position limits.
- `06_mujoco_pd_tracking.py`: create a MuJoCo joint PD tracking learning entry.
- `07_mini_wbc_qp_demo.py`: outline a teaching Mini-WBC QP structure.

## 4. Script Inputs

- Robot model paths from `configs/robot.yaml`.
- Solver parameters from `configs/ik.yaml`, `configs/qp_ik.yaml`, and `configs/mini_wbc.yaml`.
- MuJoCo model and PD parameters from `configs/mujoco_pd.yaml`.
- CLI arguments for quick experiments and output directory selection.

## 5. Script Outputs

Planned outputs include:

- Text summaries for model, FK, Jacobian, IK, and QP status.
- CSV logs for error curves and PD tracking.
- PNG plots for learning curves and tracking results.
- Structured reports under each script's `outputs/<script_name>/` directory.

The files are placeholders until the TODO algorithms are implemented.

## 6. Acceptance Standards

- Every script has argparse, logging, `main()`, and an executable entry guard.
- Every script documents inputs, outputs, recommended APIs, validation method, and legacy references.
- Every module has function signatures, type annotations, and Chinese TODO explanations.
- Config templates avoid old absolute paths.
- No complete algorithms are implemented in Step 5.

## 7. Relationship With legacy_imported

`scripts/legacy_imported/` remains the historical source reference. Standard scripts should not import or mutate those files directly.

Later implementation should read legacy scripts, extract the teaching intent, and then implement clean A project modules under `src/robot_baseline/`.

## 8. Recommended Implementation Order

1. `01_inspect_urdf.py`
2. `02_fk_frame_pose.py`
3. `03_jacobian_fd_check.py`
4. `04_dls_ik_demo.py`
5. `05_qp_ik_joint_limit_demo.py`
6. `06_mujoco_pd_tracking.py`
7. `07_mini_wbc_qp_demo.py`

## 9. Current Non-Goals

- Do not implement full FK, Jacobian, IK, QP, WBC, or MuJoCo control logic.
- Do not refactor legacy scripts.
- Do not create B/C project algorithm code.
- Do not download external repositories.
- Do not assume robot frame names without model inspection.

## 10. Step 6 Suggestions

- Update repository-level README and path rules to explain A/B/C project boundaries.
- Add run examples for TODO scripts once implementation starts.
- Add ignore rules for generated A project outputs and logs.
- Define a lightweight verification command for syntax and documentation checks.
