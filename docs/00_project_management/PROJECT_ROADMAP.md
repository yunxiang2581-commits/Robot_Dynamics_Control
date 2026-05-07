# Project Roadmap

## 2026-05-07 当前路线

当前路线必须从文档整理切换到 Project B 的 B01 demo：

```text
B01_single_joint_mpc_demo
-> B02_two_link_mpc_tracking_demo
-> B03_rollout_predictive_sampling_demo
-> C01_contact_force_allocation_demo
-> D01_quadruped_contact_qp_demo
```

不要继续泛泛整理文档。当前大项目状态、仓库状态和文档归属规则已经足够支撑下一阶段进入 B01。

近期阶段定义：

| 阶段 | 目标 | 输出 |
|---|---|---|
| Phase 1 | B01 TODO 骨架 | B01 设计文档和学习型 TODO skeleton |
| Phase 2 | B01 可运行仿真 | 单关节 MPC simulation-only demo |
| Phase 3 | B01 视频和 metrics | MP4、final error、mean tracking error、max torque、runtime per control step |
| Phase 4 | B02 二连杆 MPC | 二连杆末端 tracking demo |
| Phase 5 | B03 predictive sampling | rollout 可视化和 receding horizon demo |
| Phase 6 | C01 简化 WBC-QP | 双足 contact force allocation demo |
| Phase 7 | D01 四足 contact QP | 四足接触力分配 demo |

本文给出从当前状态到第一个可展示视频 demo 的路线。当前总目标是：simulation-only runnable simulator + video demo + metrics。

## Phase 0：仓库整理和文档规范

目标：

- 明确根 README 作为大项目入口。
- 明确 `docs/` 与各 project 内部 docs 的职责边界。
- 保持 `external/open_source_repos/` 作为只读源码参考区。
- 完成 B/C/D 仿真项目骨架。

当前状态：已完成。`docs/` 已整理为大项目管理、跨项目索引、历史归档和 interview 材料入口；历史 `step*.md` 已迁移到 `docs/archive/project_history/`。

主线状态记录见：

- `docs/00_project_management/MAINLINE_TASK_STATUS.md`

## Phase 1：Project B B01 TODO 骨架

目标：

- 创建 `B01_single_joint_mpc_demo` 的最小 TODO 骨架。
- 规划 `simulator/envs/`、`simulator/controllers/`、`simulator/planners/`、`simulator/scripts/`、`simulator/utils/` 的职责。
- 只生成学习型骨架和中文注释，不一次性写完整控制算法。

预期输出：

- B01 demo 设计文档。
- B01 Python TODO skeleton。
- README 中的最小运行命令占位。

## Phase 2：Project B B01 可运行仿真

目标：

- 实现单关节 MuJoCo 模型。
- 实现目标角度跟踪。
- 实现最小 MPC horizon cost。
- 记录角度误差、控制输入和单步运行时间。

预期输出：

- 可运行 simulator。
- 可复现实验命令。
- 初版日志和 metrics。

## Phase 3：Project B B01 视频导出和 metrics

目标：

- 实现视频导出脚本。
- 导出第一个 MP4 demo。
- 输出 final error、mean tracking error、max torque、runtime per control step。

预期输出：

- `projects/B_mujoco_mpc_study/outputs/videos/` 下的 B01 视频。
- `projects/B_mujoco_mpc_study/outputs/metrics/` 下的指标结果。
- `projects/B_mujoco_mpc_study/outputs/figures/` 下的误差曲线。

## Phase 4：Project B B02 二连杆 MPC

目标：

- 从单关节扩展到二连杆机械臂。
- 实现末端轨迹 tracking。
- 对比 A_self_baseline 中 QP-IK / task-space tracking 的思想与 MPC horizon cost。

预期输出：

- B02 二连杆 tracking 视频。
- tracking error 和 torque 曲线。

## Phase 5：Project C C01 简化 WBC-QP

目标：

- 从 OpenLoong-Dyn-Control 中抽象出简化双足接触力分配问题。
- 不做完整人形行走，不做实物部署。
- 用 QP 分配左右脚接触力，并可视化力箭头。

预期输出：

- C01 contact force allocation 视频或动画。
- CoM tracking error、contact force constraint violation、QP solve status。

## Phase 6：Project D D01 四足 contact QP

目标：

- 从 legged_control 中抽象出四足接触力分配问题。
- 不运行 `legged_hw`，不做 Unitree 实机接口。
- 用简化 QP 分配四个足端接触力，并检查摩擦锥近似约束。

预期输出：

- D01 quadruped contact QP 视频或动画。
- contact force residual、friction cone violation、gait phase correctness。
