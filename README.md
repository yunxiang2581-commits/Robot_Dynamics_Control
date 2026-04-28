# Robot Dynamics Control

面向机器人运动控制算法实习、机器人控制算法工程师、人形机器人运动控制方向的求职 baseline。

当前仓库处于 **准备阶段**：只建立目标、需求、任务、目录、模板和规则，不实现 FK、Jacobian、IK、QP、WBC、RL 等完整算法。

当前唯一工作目录：

```text
/home/ubuntu/Robot_Dynamics_Control
```

## 三条主线

| 主线 | 名称 | 目标 |
| --- | --- | --- |
| A | 自研机器人运动控制基础系统 | 用 Pinocchio + MuJoCo + OSQP 建立 URDF、FK、Jacobian、IK、QP-IK、PD、Mini-WBC 教学链路 |
| B | legged_control 复现与拆解 | 拆解 `qiayuanl/legged_control` 中 NMPC、WBC、状态估计、关节力矩控制链路 |
| C | unitree_rl_mjlab 复现与拆解 | 拆解 `unitreerobotics/unitree_rl_mjlab` 中 Train、Play、Sim2Real、obs/action/reward/policy 部署流程 |

## Monorepo 项目入口

A 项目代码、实验、配置、测试和早期根目录资产已经归入 `projects/A_self_baseline/`。根目录不再直接放 A 项目的控制代码或实验入口。

- [A 自研机器人运动控制基础系统](projects/A_self_baseline/README.md)：代码、脚本、配置、测试都在 `projects/A_self_baseline/`。
- [B legged_control 复现与拆解](projects/B_legged_control_study/README.md)：复现笔记和外部项目说明都在 `projects/B_legged_control_study/`。
- [C unitree_rl_mjlab 复现与拆解](projects/C_unitree_rl_mjlab_study/README.md)：复现笔记和强化学习项目说明都在 `projects/C_unitree_rl_mjlab_study/`。
- `shared/`：放共享环境、机器人模型资源和模板。
- `tools/`：放仓库级工具，例如 `tools/export/`。
- `docs/00_preparation/`：总项目准备阶段文档。
- `docs/00_project_management/`：迁移和项目管理记录。

根目录保留项：

- `requirements.txt`：当前作为总项目环境入口，先保留在根目录；后续可以按 A/B/C 或 shared 需求拆分到 `shared/env/`。
- `outputs/`：当前作为总输出索引保留；项目级输出应放到 `projects/*/outputs/`，根 `outputs/` 中的历史运行结果默认不进入 Git。

`legacy_imported/` 与 `root_imported/` 的区别：

- `legacy_imported/`：从旧项目导入的历史参考材料，只用于理解旧脚本意图，不直接作为标准实现。
- `root_imported/`、`root_imported_src/`、`root_imported_utils/`：从本仓库早期根目录归位来的 A 项目资产，后续需要逐步重构到标准 `src/`、`projects/A_self_baseline/scripts/`、`projects/A_self_baseline/configs/`、`projects/A_self_baseline/tests/`。

下一步建议从 A 项目第一个标准学习脚本开始：

```bash
python projects/A_self_baseline/scripts/01_inspect_urdf.py --help
```

## 准备阶段入口

### 总览与目标

- [准备阶段总览](docs/00_preparation/00_preparation_overview.md)
- [求职目标与能力矩阵](docs/00_preparation/01_job_target_and_skill_matrix.md)
- [三总项目范围说明](docs/00_preparation/02_three_project_scope.md)

### 任务、环境与结构

- [准备阶段任务总表](docs/00_preparation/03_preparation_task_table.md)
- [环境需求文档](docs/00_preparation/04_environment_requirements.md)
- [仓库结构规划](docs/00_preparation/05_repository_structure_plan.md)

### 外部项目、资源与规则

- [开源项目计划](docs/00_preparation/06_external_project_plan.md)
- [机器人模型与资源计划](docs/00_preparation/07_robot_model_and_asset_plan.md)
- [Codex 工作流规则](docs/00_preparation/08_codex_workflow_rules.md)

### 风险与验收

- [风险与备选方案](docs/00_preparation/09_risk_and_fallback_plan.md)
- [准备阶段验收清单](docs/00_preparation/10_preparation_acceptance_checklist.md)

## 推荐推进顺序

| 阶段 | 内容 | 当前状态 |
| --- | --- | --- |
| PREP | 项目管理文档、目录、模板、规则 | 进行中 |
| A0 | 自研基础系统代码骨架和 TODO 教学骨架 | 未开始 |
| A1 | URDF、FK、Jacobian | 未开始 |
| A2 | DLS-IK、QP-IK、MuJoCo PD、Mini-WBC | 未开始 |
| B | legged_control 阅读、复现记录、NMPC/WBC 拆解 | 未开始 |
| C | unitree_rl_mjlab 阅读、Play/Train 记录、RL 拆解 | 未开始 |
| Compare | 模型控制 vs 强化学习控制对比 | 未开始 |
| Interview | 三条主线面试讲稿 | 未开始 |

## 当前阶段不做什么

| 不做事项 | 原因 |
| --- | --- |
| 不实现完整机器人控制算法 | 先把项目边界、任务和验收标准定清楚 |
| 不下载大型第三方仓库 | 保持仓库轻量，避免污染历史 |
| 不提交大视频、训练日志、权重文件 | 这些产物应通过外部存储或摘要文档管理 |
| 不追求 RL 大规模训练收敛 | 第一版重点是理解 Train/Play/Sim2Real 数据流 |

## 大文件规则

本仓库默认忽略视频、日志、checkpoint、第三方源码、ROS 构建目录和 Python 缓存。精选结果图和小型 Markdown 报告可以提交，大型模型、mesh、视频和训练权重不直接进入 Git。

## 导出 Word 文档

本仓库提供 Pandoc 导出脚本，可把根 `README.md`、`docs/**/*.md` 和项目文档转换为 Word 文档。

| 项目 | 说明 |
| --- | --- |
| Windows 运行命令 | `powershell -ExecutionPolicy Bypass -File tools/export/export_md_to_docx.ps1` |
| Bash 运行命令 | `bash tools/export/export_md_to_docx.sh` |
| 单文件输出 | `exports/word/single/` |
| 合并版输出 | `exports/word/combined/robot_motion_control_job_project_docs.docx` |
| 导出日志 | `exports/word/logs/export_md_to_docx.log` |

单文件 Word 是每个 Markdown 文件单独导出的结果；合并版 Word 会按 README、准备阶段文档、A/B/C 文档、对比文档、面试文档、external 说明的顺序合并。

如果系统没有安装 Pandoc，脚本会打印安装提示并退出，不会自动安装。常用安装命令：

```bash
# pip，适合当前 Anaconda/Python 环境
python -m pip install pypandoc-binary

# Windows
winget install --id JohnMacFarlane.Pandoc

# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y pandoc

# macOS
brew install pandoc
```
