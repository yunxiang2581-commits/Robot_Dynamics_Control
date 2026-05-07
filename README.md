# Robot Dynamics Control

## 当前有效状态（2026-05-07）

本仓库是 **simulation-only 机器人运动控制学习与求职项目**。当前主线是：A 提供自研基础能力，B/C/D 从复杂开源运动控制项目中抽象可复现的仿真 demo。

统一边界：不做实物部署、不做 sim2real、不接电机、不写硬件接口、不做真实机器人安全测试。最终关键项目都要形成 runnable simulator + video demo + metrics。

| Project | 目录 | 目标 | 当前状态 | 近期重点 |
|---|---|---|---|---|
| A | `projects/A_self_baseline/` | 自研基础主线：MuJoCo / Pinocchio / QP-IK / task-space tracking | 已具备基础 QP-IK、task tracking 和 simulation-only 方向能力；本轮不改源码 | 作为 B/C/D 的基础能力来源 |
| B | `projects/B_mujoco_mpc_study/` | MuJoCo MPC simulation-only runnable simulator + video demo | 骨架完成，尚未实现 B01 | 第一优先级：创建 `B01_single_joint_mpc_demo` TODO 骨架 |
| C | `projects/C_openloong_dyn_control_study/` | OpenLoong-inspired humanoid MPC/WBC simulation-only demo | 骨架完成，尚未实现 C01 | 阅读并抽象 MPC-WBC-PVT 数据流 |
| D | `projects/D_legged_control_study/` | legged_control-inspired quadruped NMPC/WBC/state-estimation demo | 骨架完成，尚未实现 D01 | 阅读并抽象四足 NMPC/WBC/状态估计 |

当前状态入口：

- [大项目总状态](docs/00_project_management/PROJECT_STATUS.md)
- [主线任务状态](docs/00_project_management/MAINLINE_TASK_STATUS.md)
- [当前仓库状态快照](docs/00_project_management/CURRENT_REPO_STATE.md)
- [项目路线图](docs/00_project_management/PROJECT_ROADMAP.md)
- [文档归属规则](docs/00_project_management/DOC_PLACEMENT_RULES.md)

> English version: [README_en.md](README_en.md)

面向机器人运动控制算法实习、机器人控制算法工程师、人形机器人运动控制方向的求职 baseline。

当前仓库已完成准备阶段、A 项目基础能力建设、B/C/D 开源复杂运动控制项目骨架和 docs 归属整理。下一阶段不再继续泛泛整理文档，而是进入 **Project B 的 B01 单关节 MPC demo TODO 骨架**。

当前实现主线是：先用 A_self_baseline 积累 MuJoCo / Pinocchio / QP-IK / task-space tracking 基础能力，再在 B/C/D 中把复杂开源项目抽象成 simulation-only 的 runnable simulator、video demo 和 metrics。

当前唯一工作目录：

```text
/home/ubuntu/Robot_Dynamics_Control
```

## 当前项目主线

| Project | 名称 | 目标 |
| --- | --- | --- |
| A | A_self_baseline 自研基础主线 | 建立 MuJoCo / Pinocchio / QP-IK / task-space tracking 基础能力，作为 B/C/D 的能力来源 |
| B | MuJoCo MPC / MJPC 仿真复现线 | 从 `mujoco_mpc` 学习 task、cost、rollout、planner、horizon 和 receding horizon control，优先实现 B01 单关节 MPC demo |
| C | OpenLoong-inspired 人形 MPC/WBC 仿真复现线 | 从 `OpenLoong-Dyn-Control` 学习 MPC-WBC-PVT 数据流，后续实现 C01 contact force allocation demo |
| D | legged_control-inspired 四足 NMPC/WBC 仿真复现线 | 从 `legged_control` 学习四足 NMPC、WBC、状态估计和 contact QP，后续实现 D01 quadruped contact QP demo |

## Monorepo 项目入口

A 项目代码、实验、配置、测试和早期根目录资产已经归入 `projects/A_self_baseline/`。根目录不再直接放 A 项目的控制代码或实验入口。

- [A 自研机器人运动控制基础系统](projects/A_self_baseline/README.md)：代码、脚本、配置、测试都在 `projects/A_self_baseline/`。
- [B MuJoCo MPC simulation-only study](projects/B_mujoco_mpc_study/README.md)：MuJoCo MPC 仿真复现、B01/B02/B03 demo 规划和 simulator 骨架。
- [C OpenLoong-Dyn-Control simulation-only study](projects/C_openloong_dyn_control_study/README.md)：人形 MPC/WBC/PVT 数据流阅读和 C01/C02/C03 demo 规划。
- [D legged_control simulation-only study](projects/D_legged_control_study/README.md)：四足 NMPC/WBC/状态估计阅读和 D01/D02/D03 demo 规划。
- `projects/archive/`：旧命名项目和 future study 资料归档区，不作为当前主线入口。
- `shared/`：放共享环境、机器人模型资源和模板。
- `tools/`：放仓库级工具，例如 `tools/export/`。
- `tools/codex_skills/`：放从旧仓库规则迁移来的项目内 Codex skill 技能库。
- `docs/archive/preparation_history/`：早期准备阶段历史文档。
- `docs/00_project_management/`：迁移和项目管理记录。

## Codex Skill 技能库

旧仓库的 Codex 协作规则已迁为项目内技能库：

- [pinocchio-learning skill](tools/codex_skills/pinocchio-learning/SKILL.md)：A 项目 Pinocchio、MuJoCo、FK、Jacobian、IK、PD 学习流程规则。
- [旧仓库 AGENTS 原文](tools/codex_skills/pinocchio-learning/references/AGENTS_from_Pinocchio_URDF.md)：完整迁移自 `/home/ubuntu/robot_proj/Pinocchio_URDF/AGENTS.md`。
- [旧仓库 AGENT 原文](tools/codex_skills/pinocchio-learning/references/AGENT_from_Pinocchio_URDF.md)：完整迁移自 `/home/ubuntu/robot_proj/Pinocchio_URDF/AGENT.MD`。

当前仓库没有把 skill 放入 `.codex/`，因为 `.codex/` 属于本地状态并被 `.gitignore` 忽略。可提交的技能库统一放在 `tools/codex_skills/`。

根目录保留项：

- `requirements.txt`：当前作为总项目环境入口，先保留在根目录；后续可以按 A/B/C/D 或 shared 需求拆分到 `shared/env/`。
- `outputs/`：当前作为总输出索引保留；项目级输出应放到 `projects/*/outputs/`，根 `outputs/` 中的历史运行结果默认不进入 Git。

`legacy_imported/` 与 `root_imported/` 的区别：

- `legacy_imported/`：从旧项目导入的历史参考材料，只用于理解旧脚本意图，不直接作为标准实现。
- `root_imported/`、`root_imported_src/`、`root_imported_utils/`：从本仓库早期根目录归位来的 A 项目资产，后续需要逐步重构到标准 `src/`、`projects/A_self_baseline/scripts/`、`projects/A_self_baseline/configs/`、`projects/A_self_baseline/tests/`。

当前下一步从 Project B 的 B01 单关节 MPC demo 开始。先创建 B01 TODO 骨架，再实现最小可运行仿真，最后导出 MP4 和 metrics。

## 历史准备阶段资料

准备阶段资料已经归档为历史记录，不作为当前主线执行入口。当前执行入口以顶部状态表和 `docs/00_project_management/PROJECT_STATUS.md` 为准。

- [准备阶段总览](docs/archive/preparation_history/00_preparation_overview.md)
- [求职目标与能力矩阵](docs/archive/preparation_history/01_job_target_and_skill_matrix.md)
- [三总项目范围说明](docs/archive/preparation_history/02_three_project_scope.md)

### 任务、环境与结构（历史）

- [准备阶段任务总表](docs/archive/preparation_history/03_preparation_task_table.md)
- [环境需求文档](docs/archive/preparation_history/04_environment_requirements.md)
- [仓库结构规划](docs/archive/preparation_history/05_repository_structure_plan.md)

### 外部项目、资源与规则（历史）

- [开源项目计划](docs/archive/preparation_history/06_external_project_plan.md)
- [机器人模型与资源计划](docs/archive/preparation_history/07_robot_model_and_asset_plan.md)
- [Codex 工作流规则](docs/archive/preparation_history/08_codex_workflow_rules.md)

### 风险与验收（历史）

- [风险与备选方案](docs/archive/preparation_history/09_risk_and_fallback_plan.md)
- [准备阶段验收清单](docs/archive/preparation_history/10_preparation_acceptance_checklist.md)

## 推荐推进顺序

| 阶段 | 内容 | 当前状态 |
| --- | --- | --- |
| PREP | 项目管理文档、目录、模板、规则 | 已完成 |
| A | A_self_baseline 基础能力：MuJoCo / Pinocchio / QP-IK / task tracking | 已建立，作为 B/C/D 基础 |
| B01 | 单关节 MuJoCo MPC demo TODO 骨架 | 下一步 |
| B01-run | B01 可运行仿真 | 未开始 |
| B01-video | B01 MP4 视频导出和 metrics | 未开始 |
| B02 | 二连杆 MPC tracking demo | 未开始 |
| B03 | predictive sampling rollout demo | 未开始 |
| C01 | OpenLoong-inspired contact force allocation demo | 未开始 |
| D01 | legged_control-inspired quadruped contact QP demo | 未开始 |
| Interview | A/B/C/D simulation-only 项目面试讲稿 | 未开始 |

## 当前阶段不做什么

| 不做事项 | 原因 |
| --- | --- |
| 不做实物部署 | 用户没有实物机器人，当前目标是 simulation-only |
| 不做 sim2real | 当前只用仿真证明控制算法能力 |
| 不接电机、不写硬件接口 | 不涉及电机 SDK、CAN、EtherCAT、串口、固件或真实机器人安全测试 |
| 不直接运行复杂外部仓库 demo | B/C/D 外部仓库只做源码阅读参考，先抽象最小 demo |
| 不继续无限整理文档 | 当前状态已经足够，下一步必须进入 B01 TODO 骨架 |
| 不提交大视频、训练日志、权重文件 | 大型产物应通过输出目录、摘要文档或外部存储管理 |

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

单文件 Word 是每个 Markdown 文件单独导出的结果；合并版 Word 会按 README、项目管理文档、A/B/C/D 子项目文档、archive 历史资料、面试文档和 external 说明的顺序合并。

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
