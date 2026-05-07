# Environment Requirements

> Chinese version: [04_environment_requirements.md](04_environment_requirements.md)

## Reading Navigation

| Previous | Current | Next |
| --- | --- | --- |
| [03 Preparation task table](03_preparation_task_table_en.md) | Environment requirements | [05 Repository structure plan](05_repository_structure_plan_en.md) |

## Purpose

This document defines the environment expectations for the robot motion control baseline. In the preparation phase it records recommended versions and check commands only.

## Key Takeaways

| Item | Meaning |
| --- | --- |
| Minimum usable setup | Linux environment, Python 3.10+, Git |
| Recommended setup | Ubuntu 22.04/24.04, Python 3.11, Docker, MuJoCo 3.x, Pinocchio 3.x |
| Current action | define check commands and dependency plan |
| Not done now | no forced ROS, Isaac Sim, or GPU-training installation |

## Environment Table

| Category | Minimum | Recommended | Use | Check command |
| --- | --- | --- | --- | --- |
| OS | Linux environment | Ubuntu + Docker | Pinocchio, MuJoCo, ROS2, external projects | `lsb_release -a` |
| Python | 3.10+ | 3.11 | A baseline, MuJoCo, OSQP | `python --version` |
| Package manager | pip or conda | conda/mamba + pip | Python env management | `conda info`, `pip --version` |
| Dynamics library | Pinocchio | Pinocchio 3.x | URDF, FK, Jacobian, dynamics | `python -c "import pinocchio; print(pinocchio.__version__)"` |
| Simulation | MuJoCo | MuJoCo 3.x | PD control, RL | `python -c "import mujoco; print(mujoco.__version__)"` |
| QP solver | OSQP or cvxpy | OSQP + scipy | QP-IK, Mini-WBC | `python -c "import osqp, scipy"` |
| Version control | Git | Git + GitHub | repo management | `git --version` |
| Containers | optional Docker | Docker Compose | environment reproduction | `docker --version` |
| ROS | later | ROS2 Humble or Jazzy | `legged_control`, ROS2 extensions | record only in prep phase |
| GPU | optional | NVIDIA GPU | RL training, IsaacLab | `nvidia-smi` |

## Python Dependencies

- `numpy`
- `scipy`
- `pinocchio`
- `mujoco`
- `osqp`
- `matplotlib`
- `pyyaml`

## Suggested Environment Logs

- `outputs/logs/python_env_check.log`
- `outputs/logs/mujoco_check.log`
- `outputs/logs/ros_check.log`
- `outputs/logs/legged_control_build.log`
- `outputs/logs/rl_train_short.log`
