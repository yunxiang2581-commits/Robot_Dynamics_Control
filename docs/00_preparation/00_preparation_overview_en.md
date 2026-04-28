# Preparation Overview

> Chinese version: [00_preparation_overview.md](00_preparation_overview.md)

> The current phase is limited to project-management docs, directory structure, templates, and workflow rules. It does not implement FK, Jacobian, IK, QP, WBC, or RL algorithms.

## Reading Navigation

| Previous | Current | Next |
| --- | --- | --- |
| README project entry | Preparation overview | [01 Job target and skill matrix](01_job_target_and_skill_matrix_en.md) |

## Purpose

This document explains why the repository starts with a preparation phase, what its scope is, and what outputs and acceptance criteria it must produce before algorithm work begins.

## Key Takeaways

| Item | Conclusion |
| --- | --- |
| Current phase | PREP preparation phase |
| Current focus | goals, requirements, tasks, directory structure, templates, acceptance rules |
| Not done now | robot-control algorithm implementation, large third-party downloads, large-file commits |
| Next direction | A self baseline, B `legged_control`, C `unitree_rl_mjlab` |

## Three Tracks

| Track | Name | Role of the preparation phase |
| --- | --- | --- |
| A | Self-developed robot motion control baseline | define Pinocchio, MuJoCo, OSQP, scripts, figures, and acceptance criteria |
| B | `legged_control` reproduction and breakdown | define reading, reproduction, and architecture-study goals |
| C | `unitree_rl_mjlab` reproduction and breakdown | define RL workflow, Train, Play, Sim2Real, observation, action, and reward study goals |

## Phase Boundary

| Type | Done in this phase | Not done in this phase |
| --- | --- | --- |
| Documentation | preparation docs, task tables, risk tables, checklists | full technical reports |
| Code | directory skeleton and placeholder README files | robot algorithms |
| External projects | purpose, URLs, reading targets, reproduction strategy | downloading large repositories |
| Assets | URDF, MJCF, mesh, video, log, and weight management rules | committing large assets |
| Environment | recommended versions and check commands | full installation enforcement |

## Preparation Requirements

| ID | Requirement | Meaning |
| --- | --- | --- |
| P0-R1 | Define the goal first | the project must support robot control, dynamics, simulation, optimization, and RL job keywords |
| P0-R2 | Fix the scope first | the three tracks are A self baseline, B `legged_control`, C `unitree_rl_mjlab` |
| P0-R3 | Fix the workflow first | Codex should implement later work through TODO teaching skeletons, not one-shot large algorithm drops |
| P0-R4 | Fix acceptance first | every phase needs clear outputs and checkable criteria |

## Key Tasks

- PREP-001: create `docs/00_preparation/`
- PREP-002: write this overview
- PREP-003: create A/B/C placeholder structure
- PREP-004: establish large-file and ignore rules

## Outputs

- Preparation docs under `docs/00_preparation/`
- Root repository entry in `README.md`
- External-project notes under B/C external directories
- Placeholder doc directories under A/B/C

## Acceptance

| Check | Standard |
| --- | --- |
| Documentation completeness | all preparation documents exist under `docs/00_preparation/` |
| Phase boundary clarity | docs explicitly say algorithm work has not started |
| Job relevance | content stays focused on robot control, dynamics, simulation, optimization, and RL |
| Next-step readiness | Codex can continue with skeleton and implementation work from the task tables |
