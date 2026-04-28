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
