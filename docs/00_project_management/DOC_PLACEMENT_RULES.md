# Doc Placement Rules

## 2026-05-07 状态文档放置规则

状态文档按层级放置：

- 大项目状态放在 `docs/00_project_management/`。
- 子项目当前状态放在各自 `projects/*/docs/*_CURRENT_STATUS.md`。
- 不要把 B/C/D 的详细设计堆到根 `README.md`。
- 不要把 A 的具体任务细节堆到 `docs/00_project_management/`。

当前推荐状态文档：

| 文档 | 位置 |
|---|---|
| 大项目总状态 | `docs/00_project_management/PROJECT_STATUS.md` |
| 主线任务状态 | `docs/00_project_management/MAINLINE_TASK_STATUS.md` |
| 当前仓库快照 | `docs/00_project_management/CURRENT_REPO_STATE.md` |
| A 当前状态 | `projects/A_self_baseline/docs/A_CURRENT_STATUS.md` |
| B 当前状态 | `projects/B_mujoco_mpc_study/docs/B_CURRENT_STATUS.md` |
| C 当前状态 | `projects/C_openloong_dyn_control_study/docs/C_CURRENT_STATUS.md` |
| D 当前状态 | `projects/D_legged_control_study/docs/D_CURRENT_STATUS.md` |

本文定义 Markdown 文档的归属规则。目标是让主目录和 `docs/` 更简洁，让子项目说明文档尽量独立放置在各自 project 目录中。

## A. 大项目级文档

大项目级文档放在：

```text
docs/00_project_management/
```

示例：

- `PROJECT_STATUS.md`
- `PROJECT_STRUCTURE.md`
- `PROJECT_ROADMAP.md`
- `DOC_PLACEMENT_RULES.md`
- `DOC_MIGRATION_CANDIDATES.md`
- `repo_cleanup/`

这类文档描述整个仓库的状态、目录规则、整理策略、路线图和提交边界。

## B. 开源项目总览

Project B/C/D 的总览放在：

```text
docs/06_open_source_project_study/
```

示例：

- `project_bcd_overview.md`
- `project_bcd_priority.md`
- `project_bcd_simulation_only_roadmap.md`
- `project_bcd_video_demo_requirements.md`

这类文档只做跨项目比较和总览，不承载某一个 project 的详细设计。

## C. Project A 细节文档

A 项目细节放在：

```text
projects/A_self_baseline/docs/
```

示例：

- `A_pipeline_contract.md`
- `A_simulation_only_full_motion_control_plan.md`
- `A_mink_alignment_plan.md`
- task-specific docs

与 A 的 MuJoCo、Pinocchio、QP-IK、task-space tracking、mink 对齐和具体脚本任务相关的内容，应优先放到 A 项目内部。

## D. Project B 细节文档

B 项目细节放在：

```text
projects/B_mujoco_mpc_study/docs/
```

示例：

- `B01_single_joint_mpc_demo_design.md`
- `B02_two_link_mpc_tracking_demo_design.md`
- `B03_rollout_predictive_sampling_demo_design.md`

与 MuJoCo MPC、predictive sampling、rollout、planner、horizon、receding horizon control 和 B 系列 demo 相关的内容，应放到 B 项目内部。

## E. Project C 细节文档

C 项目细节放在：

```text
projects/C_openloong_dyn_control_study/docs/
```

与 OpenLoong、人形 MPC/WBC、PVT、GaitScheduler、FootPlacement、StateEstimator 和 C 系列 demo 相关的内容，应放到 C 项目内部。

## F. Project D 细节文档

D 项目细节放在：

```text
projects/D_legged_control_study/docs/
```

与 legged_control、OCS2、quadruped NMPC、WBC、state estimation、gait schedule 和 D 系列 demo 相关的内容，应放到 D 项目内部。

## G. 历史 step 文档

历史 `step*.md` 文档暂时留在：

```text
docs/00_project_management/
```

后续按 `repo_cleanup/` 方案判断是否迁移、合并或归档，不直接删除。

## H. 外部仓库 README

外部仓库 README 不复制、不改写、不提交到主仓库文档体系中。正确做法是：

- 在本项目文档中引用外部源码路径。
- 在本项目文档中写阅读地图、模块理解和复现计划。
- 保持 `external/open_source_repos/` 作为只读参考区。
