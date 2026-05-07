# 环境需求文档

> English version: [04_environment_requirements_en.md](04_environment_requirements_en.md)

## 阅读导航

| 上一篇 | 当前文档 | 下一篇 |
| --- | --- | --- |
| [03 准备阶段任务总表](03_preparation_task_table.md) | 环境需求文档 | [05 仓库结构规划](05_repository_structure_plan.md) |

## 本文用途

本文件定义机器人运动控制求职 baseline 的环境需求。准备阶段只记录推荐版本、用途和检查命令，不强制完成安装。

## 核心结论

| 项目 | 说明 |
| --- | --- |
| 最低可推进环境 | 可用 Linux 环境、Python 3.10+、Git |
| 推荐环境 | Ubuntu 22.04/24.04、Python 3.11、Docker、MuJoCo 3.x、Pinocchio 3.x |
| 当前阶段动作 | 写清检查命令和依赖规划 |
| 当前阶段不做 | 不强制安装 ROS、Isaac Sim、GPU 训练环境 |

## 环境需求表

| 类别 | 最低要求 | 推荐要求 | 用途 | 检查命令 |
| --- | --- | --- | --- | --- |
| OS | Ubuntu 22.04/24.04 或可用 Linux 环境 | Ubuntu + Docker | Pinocchio、MuJoCo、ROS2、开源项目 | `lsb_release -a` |
| Python | 3.10+ | Python 3.11 | 自研 baseline、MuJoCo、OSQP | `python --version` |
| 包管理 | pip 或 conda | conda/mamba + pip | 管理 Python 环境 | `conda info`、`pip --version` |
| 动力学库 | Pinocchio | Pinocchio 3.x | URDF、FK、Jacobian、动力学 | `python -c "import pinocchio; print(pinocchio.__version__)"` |
| 仿真 | MuJoCo | MuJoCo 3.x | PD 控制、RL 项目 | `python -c "import mujoco; print(mujoco.__version__)"` |
| 优化求解器 | OSQP 或 cvxpy | OSQP + scipy | QP-IK、Mini-WBC | `python -c "import osqp, scipy"` |
| 版本管理 | Git | Git + GitHub 仓库 | 项目管理 | `git --version` |
| 容器 | Docker 可选 | Docker Compose | 环境复现 | `docker --version` |
| ROS | 后续安装 | ROS2 Humble 或 Jazzy | `legged_control`、ROS2 扩展 | 准备阶段只记录风险 |
| GPU | 非必须 | NVIDIA GPU | RL 训练、IsaacLab | `nvidia-smi` |

## Python 依赖规划

| 依赖 | 阶段 | 用途 | 准备阶段动作 |
| --- | --- | --- | --- |
| `numpy` | A | 数值计算 | 加入后续 requirements |
| `scipy` | A | 线性代数、优化辅助 | 加入后续 requirements |
| `pinocchio` | A/B | URDF、运动学、动力学 | 记录安装方式 |
| `mujoco` | A/C | 仿真和 RL 环境 | 记录安装方式 |
| `osqp` | A/B | QP 求解 | 记录安装方式 |
| `matplotlib` | A/C | 误差曲线和训练曲线 | 加入后续 requirements |
| `pyyaml` | A | 配置读取 | 加入后续 requirements |

## 环境检查任务

| 任务编号 | 任务 | 输出 | 验收标准 |
| --- | --- | --- | --- |
| PREP-006 | 写清最低和推荐环境 | 本文件环境需求表 | 能判断当前机器是否适合进入 A/B/C 阶段 |
| PREP-006 | 写清检查命令 | 检查命令列 | 后续可以复制命令做环境记录 |
| PREP-006 | 写清 ROS 风险 | 风险说明 | ROS 版本冲突不会阻塞准备阶段 |

## 后续环境日志建议

| 日志 | 路径 | 内容 |
| --- | --- | --- |
| Python 环境日志 | `outputs/logs/python_env_check.log` | Python、pip、conda、依赖版本 |
| MuJoCo 检查日志 | `outputs/logs/mujoco_check.log` | MuJoCo import 和渲染检查 |
| ROS 检查日志 | `outputs/logs/ros_check.log` | ROS 版本、工作空间、依赖状态 |
| `legged_control` 编译日志 | `outputs/logs/legged_control_build.log` | 成功或失败都保留摘要 |
| RL 短训练日志 | `outputs/logs/rl_train_short.log` | reward、报错、运行环境 |
