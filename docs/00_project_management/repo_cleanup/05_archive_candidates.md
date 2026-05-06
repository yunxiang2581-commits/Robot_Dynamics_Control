# 归档候选清单

本文件列出建议归档但不立即删除的内容。归档前需要人工确认内容是否已经迁移到规范主文档。

## 1. projects/B_legged_control_study/

为什么是 archive candidate：

- 当前 Project D 已使用 `projects/D_legged_control_study/` 作为 legged_control-inspired 主目录。
- `projects/B_legged_control_study/` 可能属于旧项目编号时期的 legged_control 学习目录。

为什么不能直接删除：

- 可能包含早期学习记录和 README 摘要。
- 删除会损失项目演化历史。

建议归档位置：

```text
docs/00_project_management/archive/projects/B_legged_control_study/
```

需要人工确认的问题：

- 其中是否有未迁移到 Project D 的重要结论。
- 是否仍被 README 或其他文档引用。

## 2. docs/02_legged_control/

为什么是 archive candidate：

- 与新 Project D 的主题相近。
- 可能是早期 legged_control 学习文档。

为什么不能直接删除：

- 可能记录了早期阅读路径或项目定位。

建议归档位置：

```text
docs/00_project_management/archive/docs/02_legged_control/
```

需要人工确认的问题：

- 是否需要把核心内容迁移到 `projects/D_legged_control_study/docs/`。

## 3. docs/03_unitree_rl_mjlab/

为什么是 archive candidate：

- 当前 Project C 主目录已经变为 `projects/C_openloong_dyn_control_study/`。
- Unitree RL / MJLab 与 OpenLoong 主题不同，可能转为 future / legacy study。

为什么不能直接删除：

- 它不是 OpenLoong 的重复内容，可能属于另一条学习路线。

建议归档位置：

```text
docs/00_project_management/archive/docs/03_unitree_rl_mjlab/
```

需要人工确认的问题：

- 是否保留为未来强化学习控制项目。
- 是否需要重新编号或重命名。

## 4. docs/00_project_management/step*.md 中过时的阶段记录

为什么是 archive candidate：

- step 文档数量较多，许多是迁移、重命名、TODO 生成和阶段状态记录。
- 当前已有更规范的项目目录和 B/C/D 文档。

为什么不能直接删除：

- step 文档是项目演化记录。
- 后续排查为什么这样组织目录时可能有价值。

建议归档位置：

```text
docs/00_project_management/archive/steps/
```

需要人工确认的问题：

- 哪些 step 文档仍是当前规则来源。
- 哪些已经被新主文档完全覆盖。

## 5. legacy_imported/

为什么是 archive candidate：

- 目录名表明其内容来自历史导入。
- 审计发现 A 项目中有多个 `legacy_imported/`。

为什么不能直接删除：

- 可能保留了迁移前的重要配置、说明或占位文件。
- A 项目仍是核心学习项目。

建议归档位置：

```text
docs/00_project_management/archive/A_legacy_imported/
```

需要人工确认的问题：

- 是否仍被脚本或文档引用。
- 是否包含无法从 Git 历史恢复的信息。

## 6. root_imported/

为什么是 archive candidate：

- 目录名表明其内容来自仓库根目录迁移。
- 可能是迁移后保留的历史来源。

为什么不能直接删除：

- 可能包含仍有参考价值的旧实验脚本或图。

建议归档位置：

```text
docs/00_project_management/archive/A_root_imported/
```

需要人工确认的问题：

- 是否与当前 `src/robot_baseline/` 或 `scripts/` 内容重复。
- 是否仍有报告或文档引用。

## 7. root_imported_src/

为什么是 archive candidate：

- 目录名表明是旧源码迁移来源。

为什么不能直接删除：

- 可能与当前源码有差异，直接删除前需要 diff 或人工阅读。

建议归档位置：

```text
docs/00_project_management/archive/A_root_imported_src/
```

需要人工确认的问题：

- 是否已完全并入 `projects/A_self_baseline/src/robot_baseline/`。
- 是否还有未迁移函数或注释。
