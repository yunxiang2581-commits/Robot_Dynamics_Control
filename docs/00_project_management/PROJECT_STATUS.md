# Project Status

## Repository Status

`Robot_Dynamics_Control` 当前是一个 simulation-only 机器人控制学习仓库。仓库级状态以项目为单位管理，优先回答三个问题：

- 每个项目当前在做什么
- 当前做到哪一阶段
- 推荐下一步是什么

## Project Status Table

| Project | Path | Status | Current Stage | Next Step | Note |
|---|---|---|---|---|---|
| Project A | `projects/A_self_baseline/` | active | A01-A03 foundation retained; A04-A07 TODO wrappers | R1 `TargetDefinition` load/save/validate | 仍是当前基础学习主线 |
| Project B | `projects/B_mujoco_mpc_study/` | active | B02 two-link tracking retained; B03 solver ladder active | continue B03 sampling / iLQR line | B03 不替代 B02 |
| Project C | `projects/C_openloong_dyn_control_study/` | active | source reading + simulation-only reproduction planning | continue audit / architecture reading | `external` 上游源码只读 |
| Project D | `projects/D_legged_control_study/` | planned | quadruped NMPC / WBC / state estimation reading line | continue simulator planning | 只做 simulation-only |
| Project E | `projects/E_augmpc_hybrid_locomotion_study/` | initialized | E00 retarget cleanup / skeleton initialized | E01 upstream static audit | AugMPC / IBRIDO clone not yet |

## Project A Notes

- Foundation validation layer retained: A00-A03
- Unified motion interface wrappers retained: A04-A07
- 当前未完成项仍包括 `TargetDefinition`、A06 target/viewer、A07 actuator tracking

## Project B Notes

- B02 是稳定基线
- B03 是 MPC solver ladder 与 sampling / iLQR 学习线
- 进入 B03 之前，优先保留 B02 benchmark / regression 健康检查口径

## Project C / D / E Shared Boundaries

- 只做 simulation-only
- 不做真实机器人部署
- 不做 sim2real
- 不做电机驱动、固件或硬件接口

## Legacy Directory

`projects/B_legged_control_study/` 为历史骨架目录，当前不作为仓库级 Project B 主线状态表的一部分。
