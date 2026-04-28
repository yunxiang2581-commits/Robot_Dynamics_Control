# A Self Baseline

This project is the self-developed baseline system for robot motion control.

The target stack is Pinocchio + MuJoCo + OSQP. Later stages are expected to include URDF loading, FK, Jacobian, IK, QP-IK, MuJoCo PD control, and a Mini-WBC learning path.

The current stage only creates the project skeleton. It does not contain migrated legacy assets or implemented control algorithms yet.

This project must stay independent from B and C. Do not mix reproduction notes, external study code, or RL training outputs from the other projects into this directory.

## Document Entry

- `docs/README.md`: A self-developed baseline project notes migrated from the root project docs.
- `docs/legacy_imported/README_from_Pinocchio_URDF.md`: legacy README imported from the old Pinocchio_URDF project for reference.
- `docs/legacy_imported/AGENTS_from_Pinocchio_URDF.md`: legacy agent guidance imported from the old Pinocchio_URDF project.
- `docs/legacy_imported/AGENT_from_Pinocchio_URDF.md`: legacy agent guidance imported from the old Pinocchio_URDF project.
- `docs/legacy_script_mapping.md`: mapping from imported legacy scripts to standard A learning scripts.
- `docs/self_baseline_learning_scripts_plan.md`: implementation plan for the standard TODO learning scripts.

## Standard Learning Scripts

- `scripts/01_inspect_urdf.py`: inspect URDF model nq, nv, joints, and frames.
- `scripts/02_fk_frame_pose.py`: TODO entry for target frame forward kinematics.
- `scripts/03_jacobian_fd_check.py`: TODO entry for Jacobian finite-difference validation.
- `scripts/04_dls_ik_demo.py`: TODO entry for Damped Least Squares IK.
- `scripts/05_qp_ik_joint_limit_demo.py`: TODO entry for constrained QP-IK.
- `scripts/06_mujoco_pd_tracking.py`: TODO entry for MuJoCo joint PD tracking.
- `scripts/07_mini_wbc_qp_demo.py`: TODO entry for teaching Mini-WBC QP structure.

## Source Modules

- `src/robot_baseline/model_loader.py`: Pinocchio model loading and summary TODOs.
- `src/robot_baseline/kinematics.py`: FK and frame candidate TODOs.
- `src/robot_baseline/jacobian_check.py`: Jacobian and finite-difference TODOs.
- `src/robot_baseline/ik.py`: DLS IK TODOs.
- `src/robot_baseline/qp_ik.py`: constrained QP-IK TODOs.
- `src/robot_baseline/pd_controller.py`: joint PD torque TODO.
- `src/robot_baseline/mini_wbc.py`: teaching Mini-WBC QP TODOs.
- `src/robot_baseline/metrics.py`: error, CSV, and plot helper TODOs.

## Config Files

- `configs/robot.yaml`: robot model path and frame placeholders.
- `configs/ik.yaml`: DLS IK solver template.
- `configs/qp_ik.yaml`: constrained QP-IK template.
- `configs/mujoco_pd.yaml`: MuJoCo PD tracking template.
- `configs/mini_wbc.yaml`: Mini-WBC QP structure template.

## Current Status

The current A project is a TODO teaching skeleton. Scripts and modules define entry points, signatures, comments, and validation expectations, but they do not implement full FK, Jacobian, IK, QP, WBC, or MuJoCo control algorithms.

## Role Of legacy_imported

`legacy_imported/` stores old project scripts and documents as references. Do not edit those files directly during standard implementation. Use them to understand the learning intent, then implement clean A project code in `scripts/` and `src/robot_baseline/`.

## Recommended Implementation Order

1. Inspect URDF and confirm model/frame names.
2. Implement FK frame pose.
3. Implement Jacobian and finite-difference validation.
4. Implement DLS IK.
5. Implement constrained QP-IK.
6. Implement MuJoCo PD tracking.
7. Implement Mini-WBC QP structure.
