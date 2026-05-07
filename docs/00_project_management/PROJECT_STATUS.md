# Project Status

## 2026-05-07 当前有效总状态

当前阶段是：

**仓库整理 + B/C/D simulation-only 项目骨架 + 大项目文档归属规则已完成，下一步进入 Project B 的 B01 单关节 MPC demo TODO 骨架。**

当前仓库是 simulation-only 机器人运动控制学习与求职项目。核心目标是用仿真证明运动控制算法能力，最终用 runnable simulator、video demo 和 metrics 展示，而不是做实物机器人部署。

### A/B/C/D 当前状态

| Project | 当前状态 | 下一步 |
|---|---|---|
| A_self_baseline | 自研基础主线项目，已有 MuJoCo / Pinocchio / QP-IK / task-space tracking / A05 最小 box-constrained QP-IK 相关能力 | 保持稳定，不在本轮修改源码，作为 B/C/D 基础能力来源 |
| B_mujoco_mpc_study | Project B，骨架完成，外部源码 `mujoco_mpc` 已作为只读参考隔离 | 第一优先级：创建 `B01_single_joint_mpc_demo` TODO 骨架 |
| C_openloong_dyn_control_study | Project C，骨架完成，外部源码 `OpenLoong-Dyn-Control` 已作为只读参考隔离 | 第二优先级：阅读并抽象 MPC-WBC-PVT 数据流，后续 C01 |
| D_legged_control_study | Project D，骨架完成，外部源码 `legged_control` 已作为只读参考隔离 | 第三优先级：阅读并抽象四足 NMPC/WBC/状态估计，后续 D01 |

### 已完成

- `external/open_source_repos/` 已通过 `.gitignore` 隔离。
- 三个外部开源仓库已 clone 到本地作为只读源码参考：`mujoco_mpc`、`OpenLoong-Dyn-Control`、`legged_control`。
- repo cleanup 整理方案文档已创建。
- B/C/D 项目骨架已创建。
- B/C/D 均已规划 `simulator/`、`outputs/videos/`、`outputs/metrics/`。
- 根 README 已改为简洁入口。
- `docs/00_project_management/` 已新增项目状态、结构、文档归属规则和路线图。
- `docs/00_preparation/` 已归档到 `docs/archive/preparation_history/`。
- 旧 `projects/B_legged_control_study/` 已归档到 `projects/archive/legacy_studies/B_legged_control_study/`。
- 旧 `projects/C_unitree_rl_mjlab_study/` 已归档到 `projects/archive/future_studies/C_unitree_rl_mjlab_study/`。

### 未完成

- 尚未实现 B01/B02/B03。
- 尚未运行仿真。
- 尚未导出视频 demo。
- 尚未生成 demo metrics。
- 已清理低风险缓存；尚未做更激进的环境目录清理。
- 旧 B/C 主题目录已经归档，后续只需确认是否长期保留 archive。
- 历史 step 文档已经归档，后续只需确认是否长期保留 archive。

### 当前禁止事项

- 不做实物部署。
- 不做 sim2real。
- 不接电机。
- 不写硬件接口。
- 不做真实机器人安全测试。
- 不直接运行复杂外部仓库 demo。
- 不修改 `external/open_source_repos/`。

### 下一步

不要继续泛泛整理文档。下一阶段必须进入：

```text
Project B -> B01_single_joint_mpc_demo TODO skeleton
```

推荐顺序：

1. 提交当前状态文档。
2. 创建 B01 TODO 骨架。
3. 实现 B01 单关节 MPC 仿真。
4. 导出 B01 MP4 视频和 metrics。
5. 再进入 B02 二连杆 MPC。
6. 再进入 C01 简化 WBC-QP。

本文记录当前整个大项目的状态。它是仓库整理、后续提交和进入第一个仿真 demo 前的状态基线。

## 1. 当前阶段

当前阶段是：

**仓库整理 + Project B/C/D 仿真项目骨架建立完成，下一步进入 Project B 的 `B01_single_joint_mpc_demo`。**

本项目当前定位是 simulation-only 机器人运动控制学习与求职项目。所有 A/B/C/D 项目都以可运行仿真、视频 demo、误差指标和可复现实验命令为最终展示形式，不做实物机器人部署。

## 2. 子项目状态

| Project | 目录 | 定位 | 当前状态 | 下一步 |
|---|---|---|---|---|
| A | `projects/A_self_baseline/` | 自研基础 baseline：MuJoCo / Pinocchio / QP-IK / task-space tracking | 主线基础项目已建立 | 保持稳定，为 B/C/D 提供基础能力参考 |
| B | `projects/B_mujoco_mpc_study/` | MuJoCo MPC simulation-only simulator | 文档和 simulator 骨架已建立 | 创建并实现 B01 单关节 MPC demo |
| C | `projects/C_openloong_dyn_control_study/` | OpenLoong-inspired humanoid MPC/WBC demo | 文档和 simulator 骨架已建立 | 后续设计 C01 接触力分配 demo |
| D | `projects/D_legged_control_study/` | legged_control-inspired quadruped NMPC/WBC demo | 文档和 simulator 骨架已建立 | 后续设计 D01 四足 contact QP demo |

## 3. 已完成

- `external/open_source_repos/` 已通过 `.gitignore` 隔离，作为外部源码只读参考区。
- `docs/00_project_management/repo_cleanup/` 仓库整理方案文档已建立。
- `projects/B_mujoco_mpc_study/` 项目骨架已建立。
- `projects/C_openloong_dyn_control_study/` 项目骨架已建立。
- `projects/D_legged_control_study/` 项目骨架已建立。
- B/C/D 都已规划 `simulator/`、`outputs/videos/`、`outputs/logs/`、`outputs/figures/`、`outputs/metrics/`。
- B/C/D 都已明确最终目标：simulation-only runnable simulator + video demo + metrics。

## 4. 未完成

- 尚未实现 `B01_single_joint_mpc_demo`。
- 尚未实现 `B02_two_link_mpc_tracking_demo`。
- 尚未实现 `B03_rollout_predictive_sampling_demo`。
- 尚未运行任何 B/C/D 仿真。
- 尚未导出任何 B/C/D 视频 demo。
- 尚未生成 B/C/D demo metrics。
- 尚未归档旧 B/C 主题文档。
- 尚未清理缓存文件。

## 5. 当前禁止事项

- 不做实物机器人部署。
- 不做 sim2real 实机测试。
- 不接电机，不使用电机 SDK。
- 不写 CAN、EtherCAT、串口、固件或硬件接口。
- 不做真实机器人安全测试。
- 不做真实传感器标定。
- 不删除高风险内容。
- 不直接合并外部开源仓库源码。

## 6. 下一步建议

1. 提交当前 repo cleanup + B/C/D skeleton + 文档规范。
2. 创建 `B01_single_joint_mpc_demo` 的 TODO 骨架。
3. 实现 B01 单关节 MPC 最小可运行仿真。
4. 导出第一个 MP4 demo，并记录 final error、mean tracking error、max torque、runtime per control step。
5. 再整理旧 B/C legacy 文档，按文档归属规则迁移或归档。

## 7. 主线任务最新状态

当前主线任务已经从“仓库整理 + B/C/D 项目骨架建立”推进到：

**全仓库状态同步 + docs 归属整理完成，下一步进入 Project B 的 B01 单关节 MPC demo 设计与 TODO 骨架。**

本轮 docs 整理后：

- `docs/` 只保留大项目管理、跨项目索引、历史归档和 interview 材料。
- 历史 `step*.md` 已迁移到 `docs/archive/project_history/`。
- 旧 `docs/01_self_baseline/` 入口已迁移到 `projects/A_self_baseline/docs/legacy_migrated/`。
- 旧 `docs/02_legged_control/` 入口已迁移到 `projects/D_legged_control_study/docs/legacy_migrated/`。
- 旧 `docs/03_unitree_rl_mjlab/` 入口已迁移到 `projects/archive/future_studies/C_unitree_rl_mjlab_study/`。
- 旧 `docs/04_compare/` 入口已迁移到 `docs/archive/legacy_compare/`。
- 早期 `docs/00_preparation/` 已迁移到 `docs/archive/preparation_history/`。
- 旧 `projects/B_legged_control_study/` 已迁移到 `projects/archive/legacy_studies/B_legged_control_study/`。
- 旧 `projects/C_unitree_rl_mjlab_study/` 已迁移到 `projects/archive/future_studies/C_unitree_rl_mjlab_study/`。

Project A 仍是主线基础项目；Project B 是下一步实现入口；Project C/D 暂时保持为文档和 simulator 骨架阶段。

详细主线状态见：

- `docs/00_project_management/MAINLINE_TASK_STATUS.md`
