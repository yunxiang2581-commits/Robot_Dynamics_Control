# Repository Structure Plan

> Chinese version: [05_repository_structure_plan.md](05_repository_structure_plan.md)

## Reading Navigation

| Previous | Current | Next |
| --- | --- | --- |
| [04 Environment requirements](04_environment_requirements_en.md) | Repository structure plan | [06 External project plan](06_external_project_plan_en.md) |

## Purpose

This document defines directory responsibilities so both Codex and human work follow the same monorepo structure.

## Key Takeaways

| Item | Meaning |
| --- | --- |
| Main doc directories | `docs/00_preparation/`, `docs/00_project_management/` |
| A/B/C entry points | `projects/A_self_baseline/`, `projects/B_legged_control_study/`, `projects/C_unitree_rl_mjlab_study/` |
| A standard code location | `projects/A_self_baseline/src/robot_baseline/` |
| Output rule | root `outputs/` is the global index; project outputs live under `projects/*/outputs/` |

## Recommended Final Layout

```text
Robot_Dynamics_Control/
├── projects/
│   ├── A_self_baseline/
│   ├── B_legged_control_study/
│   └── C_unitree_rl_mjlab_study/
├── shared/
├── tools/
├── docs/
├── exports/
├── outputs/
└── requirements.txt
```

## Responsibility Summary

- `README.md`: project entry and links
- `.gitignore`: ignore rules for large/generated files
- `projects/*/external/`: external project notes only
- `projects/A_self_baseline/scripts/`: A standard learning scripts
- `projects/A_self_baseline/src/robot_baseline/`: A reusable modules
- `projects/A_self_baseline/configs/`: A config templates
- `projects/A_self_baseline/tests/`: A test entry points
