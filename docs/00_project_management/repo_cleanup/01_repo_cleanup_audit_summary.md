# 仓库清理审计摘要

## Git 状态摘要

根据当前 Repo Cleanup Audit Report：

- 当前分支为 `main`。
- 当前无 tracked 文件修改。
- 当前存在 untracked 内容：
  - `docs/06_open_source_project_study/`
  - `external/`
  - `projects/B_mujoco_mpc_study/`
  - `projects/C_openloong_dyn_control_study/`
  - `projects/D_legged_control_study/`

## A_self_baseline 状态

`projects/A_self_baseline/` 当前无 git 状态变化。

该目录是当前主学习项目，包含：

- MuJoCo / Pinocchio 学习脚本。
- FK / Jacobian / IK / QP-IK 代码。
- 可复盘输出。
- Mink 阅读和对齐资料。

因此本轮不修改、不移动、不删除该目录。

## 外部源码仓库状态

`external/open_source_repos/` 下存在三个 nested git repo：

```text
external/open_source_repos/mujoco_mpc/
external/open_source_repos/OpenLoong-Dyn-Control/
external/open_source_repos/legged_control/
```

用途：

- 只作为源码阅读参考。
- 不进入主仓库提交。
- 不合并进 Project B/C/D 主目录。
- 不在当前仓库中编译或运行。

## .gitignore 隔离策略

`external/open_source_repos/` 应被 `.gitignore` 忽略，避免外部开源仓库被误提交。

不能忽略整个 `external/`，因为：

- `external/mink_upstream/` 与 A 项目学习、Mink 对齐和源码阅读有关。
- 是否保留或归档 `external/mink_upstream/` 需要单独决策。
