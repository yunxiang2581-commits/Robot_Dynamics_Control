# External Project Plan

> Chinese version: [06_external_project_plan.md](06_external_project_plan.md)

## Reading Navigation

| Previous | Current | Next |
| --- | --- | --- |
| [05 Repository structure plan](05_repository_structure_plan_en.md) | External project plan | [07 Robot model and asset plan](07_robot_model_and_asset_plan_en.md) |

## Purpose

This document defines how two external projects support the baseline. During the preparation phase, the repo records purpose, reading goals, reproduction goals, and output locations, but does not download large source trees.

## Key Takeaways

| Project | Role | Current action |
| --- | --- | --- |
| `qiayuanl/legged_control` | model-based control, NMPC, WBC, state estimation | document reading and reproduction plan only |
| `unitreerobotics/unitree_rl_mjlab` | MuJoCo RL, Train, Play, Sim2Real | document RL workflow and decomposition only |

## Preparation Targets

| Project | Type | Goal in preparation | Output location |
| --- | --- | --- | --- |
| `qiayuanl/legged_control` | model-based control | define NMPC/WBC/estimation study targets | `projects/B_legged_control_study/docs/`, `projects/B_legged_control_study/external/` |
| `unitreerobotics/unitree_rl_mjlab` | RL control | define MuJoCo RL, Train, Play, and Sim2Real study targets | `projects/C_unitree_rl_mjlab_study/docs/`, `projects/C_unitree_rl_mjlab_study/external/` |
| OCS2 | reference | NMPC solver-family reference | listed in B notes |
| Unitree RL Lab | extension | IsaacLab-related future extension | listed in C notes |

## `legged_control` Plan

- Official repo: `https://github.com/qiayuanl/legged_control`
- Main focus: NMPC, WBC, state estimation, torque-control engineering chain
- Reading target: README, dependencies, directory layout, core controllers, WBC, estimator
- Reproduction target: environment, dependencies, build commands, logs, failure reasons, fallbacks
- Comparison target: compare its WBC structure against A-project Mini-WBC
- Current restriction: do not commit the full third-party source tree

## `unitree_rl_mjlab` Plan

- Official repo: `https://github.com/unitreerobotics/unitree_rl_mjlab`
- Main focus: Train, Play, Sim2Real, observation/action/reward workflow
- Reading target: README, training scripts, environment definitions, model files, deployment flow
- Reproduction target: prefer Play or short training runs, not full convergence
- Breakdown target: observation, action, reward, termination, and policy output to control command
- Current restriction: do not commit full source, policy weights, or training artifacts
