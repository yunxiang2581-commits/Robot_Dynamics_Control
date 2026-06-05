# Project Structure

## Repository Role

本仓库是一个 simulation-only 的机器人运动学、动力学与控制学习工作区。顶层结构按“项目主线 + 共享资源 + 文档管理”划分。

## Top-Level Map

```text
projects/                       各项目学习主线
docs/00_project_management/     仓库级状态、路线图和结构文档
docs/01_self_baseline/          A 主线补充文档
docs/02_legged_control/         早期四足阅读文档
docs/04_compare/                对比类文档
external/                       仓库级外部参考资源
outputs/                        仓库级公共输出
shared/                         共享依赖和资源
tools/                          辅助工具与提示词文档
```

## Active Project Map

| Project | Path | Role | Important Subdirectories |
|---|---|---|---|
| Project A | `projects/A_self_baseline/` | 基础运动学 / URDF / Pinocchio / MuJoCo 学习基线 | `scripts/`, `src/robot_baseline/`, `configs/`, `docs/`, `tests/` |
| Project B | `projects/B_mujoco_mpc_study/` | MuJoCo MPC / solver ladder 学习主线 | `simulator/`, `configs/`, `docs/`, `tests/`, `outputs/` |
| Project C | `projects/C_openloong_dyn_control_study/` | OpenLoong humanoid WBC / MPC 学习线 | `docs/`, `notes/`, `logbook/`, `docker/`, `outputs/`, `simulator/` |
| Project D | `projects/D_legged_control_study/` | legged_control / OCS2 四足学习线 | `docs/`, `notes/`, `outputs/`, `simulator/` |
| Project E | `projects/E_augmpc_hybrid_locomotion_study/` | AugMPC / LRHControl / IBRIDO RL-augmented MPC 复现线 | `docs/`, `scripts/`, `configs/`, `notes/`, `outputs/`, `external/` |

## Project Responsibilities

### Project A

- A00-A03：基础验证层
- A04-A07：统一 motion interface 学习骨架
- `src/robot_baseline/`：可复用的 schema 和接口

### Project B

- B02：two-link MPC tracking 基线
- B03：sampling / CEM / MPPI-lite / iLQR / iLQG-lite 等 solver ladder
- `simulator/`：环境、planner、controller、脚本入口

### Project C

- 面向 OpenLoong-Dyn-Control 的源码阅读与 simulation-only 重构规划
- `logbook/`：源码阅读记录
- `docker/`：官方工程复现辅助文件

### Project D

- 面向 `legged_control` / OCS2 的四足 NMPC / WBC / estimation 学习
- 当前以文档和规划为主，后续才进入最小仿真 demo

### Project E

- 面向 AugMPC / LRHControl / IBRIDO 的 RL-augmented MPC locomotion 复现线
- 当前先做 retarget cleanup、上游静态审查、容器 readiness 和 public bundle / eval 路线规划

## Legacy Directory

`projects/B_legged_control_study/` 是历史骨架目录，用于保留早期 `legged_control` 阅读结构。当前仓库级命名中：

- Project B 指 `projects/B_mujoco_mpc_study/`
- Project D 指 `projects/D_legged_control_study/`

## Placement Rules

- 仓库级状态和路线图：`docs/00_project_management/`
- 项目内部说明：各自 `projects/<project_name>/README.md` 与 `docs/`
- 项目运行输出：优先放在各自 `projects/<project_name>/outputs/`
- 上游源码镜像或只读参考：优先放在项目自己的 `external/` 或仓库级 `external/`
