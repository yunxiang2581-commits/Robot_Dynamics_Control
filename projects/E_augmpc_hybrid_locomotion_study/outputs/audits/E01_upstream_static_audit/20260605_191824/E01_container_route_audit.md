# E01 Container Route Audit

## Top-Level Decision

上游公开文档明确偏向 `Apptainer / Singularity`，而不是 Docker。

## Observed Routes

| Route | Intended Role | Static Evidence |
|---|---|---|
| `ibrido_u22` | Ubuntu 22 + Isaac Sim + ROS 2 + IBRIDO | README and `u22_isaac.def` presence |
| `ibrido_u20` | Ubuntu 20 + MuJoCo CPU + XBot2 + ROS 1 + IBRIDO | README and `u20_xbot.def` presence |
| `ibrido_u24` | Ubuntu 24 + newer Isaac Sim route, still WIP | README and `u24_isaac.def` presence |

## Entrypoint Files

- `ibrido_u20/singularity/setup_container.sh`
- `ibrido_u20/singularity/run_interactive.sh`
- `ibrido_u20/singularity/execute.sh`
- `ibrido_u22/singularity/setup_container.sh`
- `ibrido_u22/singularity/run_interactive.sh`
- `ibrido_u22/singularity/execute.sh`
- `ibrido_u24/singularity/setup.sh`
- `ibrido_u24/singularity/run_interactive.sh`
- `ibrido_u24/singularity/execute.sh`
- `ibrido_u24/singularity/utils/launch_bundle.sh`

## Static Risks

- IsaacSim route may require NVIDIA / NGC related setup and image access.
- Public Kyon-related paths can still reference private robot-description repositories.
- `u24` is documented as work in progress, so it is a weaker first reproduction target.
- The official execution contract is script-driven; skipping these wrapper scripts would likely miss environment variables, mount contracts, and bundle path conventions.

## Recommendation

第一优先阅读和后续验证路线应是 `ibrido_u20` / `ibrido_u22`，不要把 `u24` 当成第一复现目标。
