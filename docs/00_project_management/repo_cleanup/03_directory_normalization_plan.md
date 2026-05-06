# 目录规范化计划

## 目标项目目录结构

目标结构：

```text
projects/
├── A_self_baseline/
├── B_mujoco_mpc_study/
├── C_openloong_dyn_control_study/
└── D_legged_control_study/
```

## 项目定位

### A_self_baseline

`projects/A_self_baseline/` 是自研基础运动控制项目。

当前职责：

- MuJoCo / Pinocchio 基础。
- FK / Jacobian。
- IK / QP-IK。
- task-space tracking。
- 可验证输出和学习脚本。

整理原则：

- 不删除。
- 不大规模重构。
- 只在明确任务中进行小范围修改。

### B_mujoco_mpc_study

`projects/B_mujoco_mpc_study/` 是 MuJoCo MPC simulation-only 仿真复现项目。

最终目标：

- MuJoCo MPC runnable simulator。
- video demo。
- metrics。
- README 运行说明。

### C_openloong_dyn_control_study

`projects/C_openloong_dyn_control_study/` 是 OpenLoong-inspired humanoid MPC/WBC simulation-only 仿真复现项目。

最终目标：

- 简化 humanoid / biped WBC 仿真器。
- 接触力分配 demo。
- simplified WBC-QP balance demo。
- MPC-WBC pipeline visual demo。

### D_legged_control_study

`projects/D_legged_control_study/` 是 legged_control-inspired quadruped NMPC/WBC simulation-only 仿真复现项目。

最终目标：

- 四足 contact QP demo。
- trot gait schedule visual demo。
- state estimation tracking demo。

## 外部源码参考区

`external/open_source_repos/` 是外部源码只读参考区。

当前包含：

```text
external/open_source_repos/mujoco_mpc/
external/open_source_repos/OpenLoong-Dyn-Control/
external/open_source_repos/legged_control/
```

规则：

- 只读参考。
- 不提交进主仓库。
- 不合并源码到 Project B/C/D。
- 不在当前整理阶段修改外部仓库内部文件。

## B/C/D 统一交付要求

每个 Project 最终都要具备：

- runnable simulator。
- video demo。
- metrics。
- README 运行说明。
- `simulator/` 目录。
- `outputs/videos/`、`outputs/figures/`、`outputs/logs/`、`outputs/metrics/`。

当前文档整理阶段只规划，不实现控制逻辑。
