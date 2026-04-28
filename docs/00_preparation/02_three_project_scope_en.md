# Three-Track Scope

> Chinese version: [02_three_project_scope.md](02_three_project_scope.md)

## Reading Navigation

| Previous | Current | Next |
| --- | --- | --- |
| [01 Job target and skill matrix](01_job_target_and_skill_matrix_en.md) | Three-track scope | [03 Preparation task table](03_preparation_task_table_en.md) |

## Purpose

This document fixes the scope of A, B, and C so the repository does not drift into an unbounded humanoid-control project.

## Key Takeaways

| Track | Must be defined in preparation | Not done in preparation |
| --- | --- | --- |
| A self baseline | module boundary, file naming, IO, acceptance figures | no full algorithm implementation |
| B `legged_control` study | reading target, environment records, NMPC/WBC/estimator breakdown goals | no immediate full build, no source modification |
| C `unitree_rl_mjlab` study | Train/Play/Sim2Real, observation/action/reward breakdown goals | no immediate large-scale training |

## Scope Table

| Track | Positioning | Preparation output | Condition to enter development |
| --- | --- | --- | --- |
| A | prove fundamental control-system implementation ability | define URDF, FK, Jacobian, DLS-IK, QP-IK, MuJoCo PD, Mini-WBC naming, IO, and acceptance plots | directories, config, script names, and acceptance outputs are fixed |
| B | support model-based control interview stories | define reading targets, setup templates, and NMPC/WBC/estimator notes | setup and reading templates exist |
| C | support RL control interview stories | define Train/Play/Sim2Real and observation/action/reward study goals | RL note and log templates exist |

## A Scope

- Includes model loading, FK, Jacobian, finite-difference validation, DLS-IK, QP-IK, MuJoCo PD tracking, and a teaching Mini-WBC pipeline.
- Excludes full production-grade humanoid WBC and real-robot deployment.

## B Scope

- Includes project survey, environment records, architecture breakdown, and comparison with A.
- Excludes rewriting the upstream controller stack.

## C Scope

- Includes project survey, model analysis, RL observation/action/reward breakdown, and Sim2Real notes.
- Excludes building an RL framework from scratch or requiring training convergence.
