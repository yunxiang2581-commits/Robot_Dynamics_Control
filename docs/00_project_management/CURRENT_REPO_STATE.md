# 当前仓库状态快照

本文记录 2026-05-07 的仓库状态快照。它用于帮助后续提交、整理和进入 B01 demo 前保持全仓库认知一致。

## 一、顶层结构说明

| 顶层项 | 当前职责 |
|---|---|
| `README.md` | 大项目简洁入口，说明 simulation-only 边界和 A/B/C/D 主线 |
| `docs/` | 大项目管理、跨项目索引、历史归档和 interview 材料 |
| `projects/` | A/B/C/D 子项目主体，以及 `projects/archive/` 历史项目归档 |
| `external/` | 外部源码参考区和上游学习材料 |
| `shared/` | 多项目共享模型、机器人资产和通用资源 |
| `tools/` | 仓库级工具和可复用辅助脚本 |
| `outputs/` | 仓库级汇总输出，不替代各 project 的 outputs |
| `exports/` | 对外展示、报告或导出材料 |

## 二、当前关键目录职责

| 目录 | 职责 |
|---|---|
| `docs/00_project_management/` | 项目状态、主线状态、仓库结构、文档归属规则、路线图和 repo cleanup 规则 |
| `docs/06_open_source_project_study/` | Project B/C/D 开源项目横向总览、优先级和 simulation-only 路线 |
| `docs/archive/preparation_history/` | 早期准备阶段历史资料，不再作为当前执行入口 |
| `docs/archive/project_history/` | 旧 `step*.md` 阶段记录 |
| `projects/A_self_baseline/` | 自研基础主线项目，提供 MuJoCo / Pinocchio / QP-IK / task-space tracking 能力 |
| `projects/B_mujoco_mpc_study/` | Project B：MuJoCo MPC simulation-only runnable simulator + video demo |
| `projects/C_openloong_dyn_control_study/` | Project C：OpenLoong-inspired humanoid MPC/WBC simulation-only demo |
| `projects/D_legged_control_study/` | Project D：legged_control-inspired quadruped NMPC/WBC/state-estimation demo |
| `projects/archive/legacy_studies/` | 旧命名项目归档，不作为当前主线入口 |
| `projects/archive/future_studies/` | future study 候选归档 |
| `external/open_source_repos/` | 三个外部开源仓库的只读源码参考区 |

## 三、Git / 仓库管理状态

- `external/open_source_repos/` 已被 `.gitignore` 忽略。
- 不应提交外部开源仓库源码。
- B/C/D 项目文档和仿真器骨架应提交。
- A 项目源码本轮不修改。
- 当前主线状态文档、仓库状态文档和子项目状态文档应作为下一次文档提交的一部分。

## 四、当前未完成整理项

- 旧 `projects/B_legged_control_study/` 已归档到 `projects/archive/legacy_studies/B_legged_control_study/`。
- 旧 `projects/C_unitree_rl_mjlab_study/` 已归档到 `projects/archive/future_studies/C_unitree_rl_mjlab_study/`。
- `docs/02_legged_control` 相关旧资料已迁移到 `projects/D_legged_control_study/docs/legacy_migrated/`。
- `docs/03_unitree_rl_mjlab` 相关旧资料已进入 `projects/archive/future_studies/C_unitree_rl_mjlab_study/`。
- `docs/00_project_management/step*.md` 已归档到 `docs/archive/project_history/`。
- `docs/00_preparation/` 已归档到 `docs/archive/preparation_history/`。
- 缓存清理：已清理 `.pytest_cache`、根 `debug.log`、A 项目和 `external/mink_upstream` 下的 `__pycache__`；未触碰 `.venv`。

说明：如果工作树中已经出现旧文档迁移或归档变更，应在提交前确认这些变更是否作为单独 docs cleanup 提交处理。

## 五、当前推荐提交分组

1. 提交 1：repo cleanup + B/C/D skeleton。
2. 提交 2：project status + document placement rules。
3. 提交 3：B01 TODO skeleton。
4. 提交 4：B01 runnable simulator。
5. 提交 5：B01 video demo + metrics。

## 六、当前下一步

不要继续泛泛整理文档。当前推荐把文档状态提交后，进入：

```text
Project B -> B01_single_joint_mpc_demo TODO skeleton
```
