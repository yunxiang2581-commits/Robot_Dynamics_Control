# Docs Reorganization Log

本文记录本轮 `docs/` 文件夹整理结果。

## 1. 整理目标

本轮目标不是简单减少文件数量，而是结合整个仓库重新划分文档职责：

- `docs/` 只保留大项目级管理文档、跨项目索引和历史归档入口。
- A/B/C/D 的项目细节尽量放回各自 `projects/*/docs/`。
- 历史阶段记录进入 archive，不再堆在当前管理目录中。
- 不修改 Python/C++ 源码，不修改外部开源仓库源码。

## 2. 已迁移内容

| 原路径 | 新路径 | 处理方式 |
|---|---|---|
| `docs/00_project_management/step*.md` | `docs/archive/project_history/` | 迁移为历史阶段记录 |
| `docs/01_self_baseline/README.md` | `projects/A_self_baseline/docs/legacy_migrated/README_from_docs_01_self_baseline.md` | 迁移到 A 项目 legacy 文档 |
| `docs/02_legged_control/README.md` | `projects/D_legged_control_study/docs/legacy_migrated/README_from_docs_02_legged_control.md` | 迁移到 D 项目 legacy 文档 |
| `docs/03_unitree_rl_mjlab/README.md` | `projects/archive/future_studies/C_unitree_rl_mjlab_study/docs/legacy_migrated/README_from_docs_03_unitree_rl_mjlab.md` | 迁移到旧 Unitree future study 归档 |
| `docs/04_compare/README.md` | `docs/archive/legacy_compare/README_from_docs_04_compare.md` | 迁移为旧横向比较资料 |

## 3. 新增入口

- 新增 `docs/README.md` 作为 docs 总入口。
- 新增 `docs/00_project_management/README.md` 作为项目管理文档入口。

## 4. 仍保留在 docs/ 的内容

- `docs/00_project_management/`：大项目状态、结构、规则、路线图和 repo cleanup。
- `docs/06_open_source_project_study/`：Project B/C/D 横向总览。
- `docs/interview/`：求职展示相关资料。

## 5. 未处理事项

- `docs/interview/` 后续可按简历、项目展示、问答稿进一步细分。
- 空旧目录可能因本地文件系统权限暂时保留，但 Git 不跟踪空目录。

## 7. 第二轮全仓库归档整理

为减少当前主线入口混乱，已进一步归档以下内容：

| 原路径 | 新路径 | 处理方式 |
|---|---|---|
| `docs/00_preparation/` | `docs/archive/preparation_history/` | 迁移为早期准备阶段历史资料 |
| `projects/B_legged_control_study/` | `projects/archive/legacy_studies/B_legged_control_study/` | 迁移为旧命名 legged_control study |
| `projects/C_unitree_rl_mjlab_study/` | `projects/archive/future_studies/C_unitree_rl_mjlab_study/` | 迁移为 future study 候选 |

归档后当前有效项目入口保持为：

- `projects/A_self_baseline/`
- `projects/B_mujoco_mpc_study/`
- `projects/C_openloong_dyn_control_study/`
- `projects/D_legged_control_study/`

## 8. 低风险缓存清理

已清理以下低风险缓存或临时文件：

- `.pytest_cache/`
- 根目录 `debug.log`
- `projects/A_self_baseline/` 下的 `__pycache__`
- `external/mink_upstream/` 下的 `__pycache__`

未清理 `.venv/`，避免破坏本地 Python 虚拟环境。

## 6. 风险控制

- 本轮没有删除有内容的学习资料。
- 被删除的原路径均已先复制到新位置。
- 没有修改 `projects/A_self_baseline` 源码。
- 没有修改 `external/open_source_repos`。
- 没有运行仿真或编译。
