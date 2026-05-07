# Doc Migration Candidates

本文只列出文档迁移候选。本轮不移动、不重命名、不删除任何文件。所有迁移都需要在单独任务中人工确认后执行。

## A. 后续可迁移到 A_self_baseline/docs/ 的候选

| 当前路径 | 建议目标 | 理由 | 风险等级 | 是否需要人工确认 |
|---|---|---|---|---|
| `docs/00_project_management/step8_6_A_pipeline_components.md` | `projects/A_self_baseline/docs/` | 与 A pipeline 组件划分相关 | 中 | 是 |
| `docs/00_project_management/step8_9_align_A_with_mink_ur5e.md` | `projects/A_self_baseline/docs/` | 与 A 和 mink/UR5e 对齐相关 | 中 | 是 |
| `docs/00_project_management/step8_11_define_mink_capability_and_A_requirements.md` | `projects/A_self_baseline/docs/` | 与 A 的 mink 能力边界和需求相关 | 中 | 是 |
| `docs/00_project_management/step11P_simulation_first_full_motion_control_plan.md` | `projects/A_self_baseline/docs/` | 与 A simulation-only 全流程控制计划相关 | 中 | 是 |
| `docs/00_project_management/step9A_A01_model_inspect_todo_skeleton.md` | `projects/A_self_baseline/docs/` | 与 A01 任务骨架相关 | 低 | 是 |
| `docs/00_project_management/step10B_A02_configuration_site_pose_todo_skeleton.md` | `projects/A_self_baseline/docs/` | 与 A02 任务骨架相关 | 低 | 是 |
| `docs/00_project_management/step11A_A03_site_jacobian_check_todo_skeleton.md` | `projects/A_self_baseline/docs/` | 与 A03 Jacobian 检查相关 | 低 | 是 |
| `docs/00_project_management/step12A_A04_dls_differential_ik_todo_skeleton.md` | `projects/A_self_baseline/docs/` | 与 A04 DLS IK 相关 | 低 | 是 |
| `docs/00_project_management/step13A_A05_task_limit_qp_ik_todo_skeleton.md` | `projects/A_self_baseline/docs/` | 与 A05 QP-IK 相关 | 低 | 是 |

## B. 后续可迁移到 B_mujoco_mpc_study/docs/ 的候选

| 当前路径 | 建议目标 | 理由 | 风险等级 | 是否需要人工确认 |
|---|---|---|---|---|
| `docs/06_open_source_project_study/project_bcd_simulation_only_roadmap.md` 中 Project B 相关内容 | `projects/B_mujoco_mpc_study/docs/` | 与 MuJoCo MPC demo 路线相关 | 中 | 是 |
| `docs/06_open_source_project_study/project_bcd_video_demo_requirements.md` 中 Project B 相关内容 | `projects/B_mujoco_mpc_study/docs/` | 与 B01/B02/B03 视频 demo 要求相关 | 中 | 是 |
| 后续新增的 MuJoCo MPC、rollout、predictive sampling 细节笔记 | `projects/B_mujoco_mpc_study/docs/` | 属于 B 项目详细设计 | 低 | 是 |

## C. 后续可迁移到 C_openloong_dyn_control_study/docs/ 的候选

| 当前路径 | 建议目标 | 理由 | 风险等级 | 是否需要人工确认 |
|---|---|---|---|---|
| `docs/06_open_source_project_study/project_bcd_simulation_only_roadmap.md` 中 Project C 相关内容 | `projects/C_openloong_dyn_control_study/docs/` | 与 C01/C02/C03 仿真路线相关 | 中 | 是 |
| `docs/06_open_source_project_study/project_bcd_video_demo_requirements.md` 中 Project C 相关内容 | `projects/C_openloong_dyn_control_study/docs/` | 与 OpenLoong-inspired 视频 demo 要求相关 | 中 | 是 |
| 后续新增的 OpenLoong、人形 MPC/WBC、PVT、gait scheduler、foot placement 细节笔记 | `projects/C_openloong_dyn_control_study/docs/` | 属于 C 项目详细设计 | 低 | 是 |

## D. 后续可迁移到 D_legged_control_study/docs/ 的候选

| 当前路径 | 建议目标 | 理由 | 风险等级 | 是否需要人工确认 |
|---|---|---|---|---|
| `docs/02_legged_control/` | 已迁移到 `projects/D_legged_control_study/docs/legacy_migrated/` | 与 legged_control 主题高度相关，已作为 D 的 legacy 文档保留 | 中 | 否 |
| `docs/06_open_source_project_study/project_bcd_simulation_only_roadmap.md` 中 Project D 相关内容 | `projects/D_legged_control_study/docs/` | 与 D01/D02/D03 仿真路线相关 | 中 | 是 |
| `docs/06_open_source_project_study/project_bcd_video_demo_requirements.md` 中 Project D 相关内容 | `projects/D_legged_control_study/docs/` | 与四足视频 demo 要求相关 | 中 | 是 |
| 后续新增的 OCS2、quadruped NMPC、WBC、state estimation、gait schedule 细节笔记 | `projects/D_legged_control_study/docs/` | 属于 D 项目详细设计 | 低 | 是 |

## E. 应留在 docs/00_project_management/ 的候选

| 当前路径 | 建议目标 | 理由 | 风险等级 | 是否需要人工确认 |
|---|---|---|---|---|
| `docs/00_project_management/repo_cleanup/` | 保持不变 | 仓库整理方案属于大项目级管理文档 | 低 | 否 |
| `docs/00_project_management/PROJECT_STATUS.md` | 保持不变 | 大项目状态入口 | 低 | 否 |
| `docs/00_project_management/PROJECT_STRUCTURE.md` | 保持不变 | 仓库结构规则 | 低 | 否 |
| `docs/00_project_management/DOC_PLACEMENT_RULES.md` | 保持不变 | 文档归属规则 | 低 | 否 |
| `docs/00_project_management/PROJECT_ROADMAP.md` | 保持不变 | 跨项目路线图 | 低 | 否 |

## F. 应归档但不删除的候选

| 当前路径 | 建议目标 | 理由 | 风险等级 | 是否需要人工确认 |
|---|---|---|---|---|
| `docs/00_project_management/step*.md` | 后续 archive 目录 | 属于历史阶段记录，仍有追溯价值 | 中 | 是 |
| `projects/B_legged_control_study/` | 已迁移到 `projects/archive/legacy_studies/B_legged_control_study/` | 旧命名与新 Project D 存在主题重叠，已归档 | 中 | 否 |
| `docs/02_legged_control/` | 后续 archive 或迁移到 D | 可能是早期 legged_control 学习文档 | 高 | 是 |
| `docs/03_unitree_rl_mjlab/` | 已迁入 `projects/archive/future_studies/C_unitree_rl_mjlab_study/` | 与当前 Project C OpenLoong 主题不同，作为 future study 保留 | 中 | 否 |
| `projects/C_unitree_rl_mjlab_study/` | 已迁移到 `projects/archive/future_studies/C_unitree_rl_mjlab_study/` | 与 OpenLoong 不同主题，作为 future study 保留 | 中 | 否 |
| `legacy_imported/` | 后续 archive 目录 | 可能是历史导入内容，不能直接删除 | 高 | 是 |
| `root_imported/` | 后续 archive 目录 | 可能包含历史根目录导入内容 | 高 | 是 |
| `root_imported_src/` | 后续 archive 目录 | 可能包含历史源码导入内容 | 高 | 是 |

