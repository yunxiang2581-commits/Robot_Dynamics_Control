# Robot Dynamics Control

`Robot_Dynamics_Control` 是一个 simulation-only 机器人运动学、动力学与控制学习仓库。仓库主线从 URDF / FK / Jacobian / IK 出发，逐步连接到 MuJoCo MPC、人形 WBC-MPC，以及 RL-augmented MPC locomotion 复现学习。

## Repository Scope

- 只做仿真、离线算法阅读与最小复现
- 优先建立可解释、可验证、可复盘的学习链路
- 不做真实机器人部署、sim2real、电机驱动、固件或硬件接口

## Project Index

| Project | Path | Focus | Current Stage | Next Step |
|---|---|---|---|---|
| Project A | `projects/A_self_baseline/` | 基础运动学、Pinocchio、URDF、MuJoCo 验证 | A01-A03 foundation retained; A04-A07 TODO wrappers | R1 `TargetDefinition` load/save/validate |
| Project B | `projects/B_mujoco_mpc_study/` | MuJoCo MPC、tracking、solver ladder、iLQR / iLQG-lite | B02 baseline retained; B03 solver ladder active | continue B03 sampling / iLQR line |
| Project C | `projects/C_openloong_dyn_control_study/` | OpenLoong-Dyn-Control humanoid WBC / MPC 学习 | source reading + simulation-only planning active | continue source audit / simulator mapping |
| Project D | `projects/D_legged_control_study/` | legged_control / OCS2 四足 NMPC / WBC / state estimation 学习 | planning / reading line | continue quadruped simulator planning |
| Project E | `projects/E_augmpc_hybrid_locomotion_study/` | RL-augmented MPC reproduction target based on AugMPC / LRHControl / IBRIDO. Focus on high-level RL contact schedules and twist commands with low-level MPC execution. Container-first, public resource audit before execution, simulation-only. | E00-E02 completed | E03 public resource completeness audit |

## Project Boundaries

- Project A：自写学习基线，不直接把 `mink` 当替代实现
- Project B：MuJoCo MPC 学习主线，不替代 B02 benchmark / regression 基线
- Project C：OpenLoong 上游源码只读参考，不在 `external/open_source_repos/` 内直接改源码
- Project D：当前只做 simulation-only 四足架构学习，不做 `legged_hw` 或真实机器人接口
- Project E：当前只做 RL-augmented MPC locomotion simulation-only 复现，不运行真实机器人部署链路

## Legacy Note

`projects/B_legged_control_study/` 是早期 `legged_control` 项目骨架，当前不作为主线 Project B 使用。当前 Project B 指 `projects/B_mujoco_mpc_study/`，Project D 承接四足 `legged_control` 学习线。

## Entry Points

- [Project status](docs/00_project_management/PROJECT_STATUS.md)
- [Project roadmap](docs/00_project_management/PROJECT_ROADMAP.md)
- [Project structure](docs/00_project_management/PROJECT_STRUCTURE.md)
- [Project A README](projects/A_self_baseline/README.md)
- [Project B README](projects/B_mujoco_mpc_study/README.md)
- [Project C README](projects/C_openloong_dyn_control_study/README.md)
- [Project D README](projects/D_legged_control_study/README.md)
- [Project E README](projects/E_augmpc_hybrid_locomotion_study/README.md)
