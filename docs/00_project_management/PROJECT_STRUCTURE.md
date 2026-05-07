# Project Structure

本文说明本仓库的理想目录结构和职责边界。目标是让根目录保持简洁，让大项目级文档留在 `docs/`，让子项目细节尽量放回各自 project 目录。

## 1. 推荐目标树

```text
Robot_Dynamics_Control/
├── README.md
├── README_en.md
├── AGENTS.md
├── requirements.txt
├── .gitignore
├── docs/
├── projects/
├── external/
├── shared/
├── tools/
├── outputs/
└── exports/
```

根目录只保留入口文件、配置文件和一级目录。具体项目说明、算法笔记、demo 设计和源码阅读地图应尽量进入对应子目录。

## 2. 一级目录职责

| 目录 | 职责 |
|---|---|
| `docs/` | 大项目级管理文档、开源项目总览、仓库整理方案和跨项目路线图 |
| `projects/` | A/B/C/D 子项目，以及 `projects/archive/` 中的旧项目归档 |
| `external/` | 外部源码参考、上游代码阅读材料，不作为主线实现目录 |
| `shared/` | 多个项目共享的机器人模型、资产或通用材料 |
| `tools/` | 仓库级工具脚本或后续导出辅助工具 |
| `outputs/` | 仓库级汇总输出，不替代各 project 自己的 outputs |
| `exports/` | 对外展示或交付材料，例如整理后的文档、报告或展示包 |

## 3. projects/ 职责边界

| Project | 目录 | 职责 |
|---|---|---|
| A | `projects/A_self_baseline/` | 自研基础运动控制 baseline，重点是 MuJoCo / Pinocchio / QP-IK / task-space tracking |
| B | `projects/B_mujoco_mpc_study/` | MuJoCo MPC 仿真复现项目，目标是 runnable simulator + video demo |
| C | `projects/C_openloong_dyn_control_study/` | OpenLoong-inspired 人形 MPC/WBC 仿真复现项目 |
| D | `projects/D_legged_control_study/` | legged_control-inspired 四足 NMPC/WBC/状态估计仿真复现项目 |

旧命名项目不再作为当前入口，已归入：

```text
projects/archive/
```

## 4. external/ 规则

`external/open_source_repos/` 是外部开源仓库只读源码参考区：

- `external/open_source_repos/mujoco_mpc/` 对应 Project B。
- `external/open_source_repos/OpenLoong-Dyn-Control/` 对应 Project C。
- `external/open_source_repos/legged_control/` 对应 Project D。

该目录已通过 `.gitignore` 隔离，不进入主仓库提交。后续阅读源码时，只在本项目文档中记录阅读地图、模块理解和复现计划，不修改外部仓库源码。

## 5. outputs/ 与 project outputs/ 区别

- 根目录 `outputs/` 用于仓库级汇总结果，例如跨项目报告、总览图或最终展示材料。
- `projects/*/outputs/` 用于对应子项目的 demo 输出。
- `projects/*/outputs/videos/` 保存该 project 的视频 demo。
- `projects/*/outputs/metrics/` 保存该 project 的指标结果，例如 `metrics.json`。
- `projects/*/outputs/logs/` 保存该 project 的运行日志。
- `projects/*/outputs/figures/` 保存该 project 的正式图表。

输出视频和 metrics 是正式成果，不按普通缓存处理。
