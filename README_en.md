# Robot Dynamics Control

> Chinese version: [README.md](README.md)

This repository is a job-oriented baseline for robot motion control internships and entry-level control engineering roles.

The repository is still in the preparation phase. It defines goals, scope, tasks, directory structure, templates, and workflow rules. It does not implement full FK, Jacobian, IK, QP, WBC, or RL algorithms yet.

Current working directory:

```text
/home/ubuntu/Robot_Dynamics_Control
```

## Three Tracks

| Track | Name | Goal |
| --- | --- | --- |
| A | Self-developed robot motion control baseline | Build a teaching pipeline with Pinocchio, MuJoCo, and OSQP |
| B | `legged_control` reproduction and breakdown | Study NMPC, WBC, state estimation, and torque control engineering |
| C | `unitree_rl_mjlab` reproduction and breakdown | Study MuJoCo RL workflows, Train, Play, Sim2Real, observation, action, and reward |

## Monorepo Entry Points

- [A self baseline](projects/A_self_baseline/README.md): source code, scripts, configs, tests, and imported A assets live under `projects/A_self_baseline/`.
- [B legged_control study](projects/B_legged_control_study/README.md): reproduction notes and external project notes live under `projects/B_legged_control_study/`.
- [C unitree_rl_mjlab study](projects/C_unitree_rl_mjlab_study/README.md): RL reproduction notes and external project notes live under `projects/C_unitree_rl_mjlab_study/`.
- `shared/`: shared environments, robot assets, and templates.
- `tools/`: repository-level tools such as export scripts and Codex skills.
- `docs/00_preparation/`: preparation-phase documentation.
- `docs/00_project_management/`: migration and project-management records.

## Codex Skills

- `tools/codex_skills/pinocchio-learning/`: migrated repository-local Codex skill for A-project Pinocchio, MuJoCo, FK, Jacobian, IK, and PD learning workflow.
- The original old-repository `AGENTS.md` and `AGENT.MD` are stored under the skill `references/` directory.

## Preparation Docs

### Overview and goals

- [Preparation overview](docs/00_preparation/00_preparation_overview_en.md)
- [Job target and skill matrix](docs/00_preparation/01_job_target_and_skill_matrix_en.md)
- [Three-track scope](docs/00_preparation/02_three_project_scope_en.md)

### Tasks, environment, and structure

- [Preparation task table](docs/00_preparation/03_preparation_task_table_en.md)
- [Environment requirements](docs/00_preparation/04_environment_requirements_en.md)
- [Repository structure plan](docs/00_preparation/05_repository_structure_plan_en.md)

### External projects, assets, and rules

- [External project plan](docs/00_preparation/06_external_project_plan_en.md)
- [Robot model and asset plan](docs/00_preparation/07_robot_model_and_asset_plan_en.md)
- [Codex workflow rules](docs/00_preparation/08_codex_workflow_rules_en.md)

### Risks and acceptance

- [Risk and fallback plan](docs/00_preparation/09_risk_and_fallback_plan_en.md)
- [Preparation acceptance checklist](docs/00_preparation/10_preparation_acceptance_checklist_en.md)

## Recommended Order

| Phase | Content | Status |
| --- | --- | --- |
| PREP | project-management docs, templates, directory structure, rules | in progress |
| A0 | code skeletons and TODO learning skeletons for A | not started |
| A1 | URDF, FK, Jacobian | not started |
| A2 | DLS-IK, QP-IK, MuJoCo PD, Mini-WBC | not started |
| B | `legged_control` reading, reproduction notes, NMPC/WBC breakdown | not started |
| C | `unitree_rl_mjlab` reading, Play/Train notes, RL breakdown | not started |

## What The Repository Does Not Do Yet

- It does not implement full robot-control algorithms yet.
- It does not download large third-party repositories into the repo.
- It does not commit large videos, logs, checkpoints, or training weights.
- It does not aim for RL convergence in the current preparation phase.

## Export Tools

The repository includes Markdown-to-Word export scripts under `tools/export/`.

- Windows: `powershell -ExecutionPolicy Bypass -File tools/export/export_md_to_docx.ps1`
- Bash: `bash tools/export/export_md_to_docx.sh`

If Pandoc is not installed, the scripts print installation guidance and exit without auto-installing anything.
