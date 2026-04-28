# Risk And Fallback Plan

> Chinese version: [09_risk_and_fallback_plan.md](09_risk_and_fallback_plan.md)

## Reading Navigation

| Previous | Current | Next |
| --- | --- | --- |
| [08 Codex workflow rules](08_codex_workflow_rules_en.md) | Risk and fallback plan | [10 Preparation acceptance checklist](10_preparation_acceptance_checklist_en.md) |

## Purpose

This document records environment, build, training, model-asset, and repo-management risks before they block the job-project baseline.

## Key Takeaways

| Risk type | Handling principle |
| --- | --- |
| Environment risk | record checks and fall back to Docker or a reduced target when needed |
| Build risk | keep logs and failure analysis instead of hiding build failures |
| Training risk | prefer Play and short runs, not large-scale convergence |
| Model-asset risk | validate the workflow on small models first, then switch to realistic assets |
| Repo-management risk | large files ignored by default, third-party source not copied in |

## Main Risks

| Risk | Cause | Impact | Fallback |
| --- | --- | --- | --- |
| `legged_control` build failure | ROS, OCS2, dependency complexity | B demo cannot run | finish architecture notes and keep build logs |
| `unitree_rl_mjlab` slow training | insufficient GPU or env issues | C cannot complete long training | run Play or short training and focus on workflow analysis |
| Missing MuJoCo models | MJCF or mesh path mismatch | A/C simulation blocked | use a simple model for PD baseline first |
| Pinocchio URDF mesh errors | mesh path or format issues | incomplete model loading | load the kinematic tree first without rendering |
| ROS2 version conflict | Ubuntu mismatch | ROS tasks blocked | keep ROS2 as a later extension |
| Codex writes too much at once | weak prompt constraints | learning workflow becomes opaque | enforce TODO teaching skeletons |
| Large files pollute Git | videos, meshes, logs, checkpoints | bloated repo | control `.gitignore` and asset rules up front |

## Execution Principles

- If build fails, record system info, commands, errors, and attempted fixes.
- If training does not converge, explain observation, action, reward, and termination instead of chasing performance.
- If model loading fails, fall back to smaller models first.
- If dependencies conflict, isolate the environment and prefer containers when needed.
