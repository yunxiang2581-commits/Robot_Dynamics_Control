# docs 文档总入口

本目录现在只承担大项目级文档、跨项目索引和历史归档入口，不再作为所有子项目细节文档的堆放处。

当前大项目是 simulation-only 机器人运动控制学习与求职项目。A/B/C/D 的具体算法设计、demo 设计、simulator 说明和指标记录，应优先放在各自 project 目录中。

## 当前有效入口

| 目录 | 用途 |
|---|---|
| `00_project_management/` | 大项目状态、目录结构、文档归属规则、路线图和仓库整理记录 |
| `06_open_source_project_study/` | Project B/C/D 开源项目横向总览、优先级和 simulation-only 路线 |
| `archive/` | 已迁移的准备阶段资料、历史阶段记录、旧主题入口和 legacy 比较文档 |
| `interview/` | 求职展示、面试和项目表达相关材料 |

## 子项目细节归属

| 内容类型 | 推荐位置 |
|---|---|
| Project A 细节 | `projects/A_self_baseline/docs/` |
| Project B MuJoCo MPC 细节 | `projects/B_mujoco_mpc_study/docs/` |
| Project C OpenLoong 细节 | `projects/C_openloong_dyn_control_study/docs/` |
| Project D legged_control 细节 | `projects/D_legged_control_study/docs/` |
| 旧 Unitree RL/MJLab study | `projects/archive/future_studies/C_unitree_rl_mjlab_study/` |

## 当前主线状态

当前主线状态入口是：

- `docs/00_project_management/MAINLINE_TASK_STATUS.md`

它记录 A_self_baseline、B/C/D 仿真项目、docs 整理和下一步 B01 demo 的最新状态。

## 已完成的本轮整理

- `docs/00_project_management/step*.md` 已迁移到 `docs/archive/project_history/`。
- `docs/00_preparation/` 已迁移到 `docs/archive/preparation_history/`。
- `docs/01_self_baseline/README.md` 已迁移到 `projects/A_self_baseline/docs/legacy_migrated/`。
- `docs/02_legged_control/README.md` 已迁移到 `projects/D_legged_control_study/docs/legacy_migrated/`。
- `docs/03_unitree_rl_mjlab/README.md` 已迁移到 `projects/archive/future_studies/C_unitree_rl_mjlab_study/`。
- `docs/04_compare/README.md` 已迁移到 `docs/archive/legacy_compare/`。
- 旧 `projects/B_legged_control_study/` 已迁移到 `projects/archive/legacy_studies/B_legged_control_study/`。
- 旧 `projects/C_unitree_rl_mjlab_study/` 已迁移到 `projects/archive/future_studies/C_unitree_rl_mjlab_study/`。

说明：迁移后旧空目录可能仍在本地文件系统中，但 Git 不跟踪空目录；后续可在具备权限时清理这些空目录。

## 继续整理时的原则

- 先迁移和合并，再归档，最后才删除。
- 不删除可能有学习价值的 Markdown。
- 不复制或改写外部仓库 README，只在本项目文档中引用路径和阅读地图。
- 输出视频、metrics、正式图表属于项目成果，不按缓存删除。
