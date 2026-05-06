# 重复/相似内容候选报告

本报告只列出重复或相似内容候选，不建议直接删除。所有候选都需要人工确认后再决定是否合并、归档或保留。

## A. legged_control 重复候选

候选：

```text
projects/B_legged_control_study/
projects/D_legged_control_study/
docs/02_legged_control/
external/open_source_repos/legged_control/
```

建议：

- `projects/D_legged_control_study/` 作为新 Project D 主目录。
- `projects/B_legged_control_study/` 暂定为旧命名历史目录，后续人工确认是否归档。
- `docs/02_legged_control/` 暂定为早期学习文档，后续人工确认是否迁移到 D。
- `external/open_source_repos/legged_control/` 仅作只读源码参考，不合并进 D。

不能直接删除的原因：

- 旧目录可能包含历史学习记录。
- 新目录包含当前 Project D 的 simulation-only 规划。
- 外部仓库是源码阅读参考，有 nested git repo 边界。

## B. Project C 重复/相似候选

候选：

```text
projects/C_unitree_rl_mjlab_study/
projects/C_openloong_dyn_control_study/
docs/03_unitree_rl_mjlab/
```

建议：

- `projects/C_openloong_dyn_control_study/` 作为新 Project C 主目录。
- `projects/C_unitree_rl_mjlab_study/` 与 OpenLoong 主题不同，不能直接删除。
- `docs/03_unitree_rl_mjlab/` 暂定为早期学习文档。
- 后续可考虑把 `C_unitree_rl_mjlab_study` 改为 future / legacy study，但本轮不处理。

不能直接删除的原因：

- Unitree RL / MJLab 与 OpenLoong MPC/WBC 不是同一主题。
- 旧内容可能仍有强化学习或仿真学习价值。

## C. Mink 参考重复候选

候选：

```text
external/mink_upstream/
projects/A_self_baseline/external/mink/
projects/A_self_baseline/mink_code_reading/source_annotated/
shared/robot_assets/models/mink_universal_robots_ur5e/
```

建议：

- 暂不删除。
- 区分 upstream 原始源码、A 项目内引用、annotated 阅读版、机器人模型资产。

建议理解：

- `external/mink_upstream/`：上游原始源码镜像。
- `projects/A_self_baseline/external/mink/`：A 项目内局部引用。
- `projects/A_self_baseline/mink_code_reading/source_annotated/`：带注释的源码阅读版本。
- `shared/robot_assets/models/mink_universal_robots_ur5e/`：UR5e 模型资产，不等同于源码。

## D. 文档历史候选

候选：

```text
docs/00_project_management/step*.md
```

建议：

- 作为项目演化记录，适合后续归档。
- 不适合直接删除。
- 若已有新主文档覆盖旧内容，应先完成内容合并，再把旧 step 文档标记为 archive candidate。

不能直接删除的原因：

- step 文档记录了迁移、重命名、任务设计和阶段性决策。
- 后续排查项目历史时可能需要这些记录。
