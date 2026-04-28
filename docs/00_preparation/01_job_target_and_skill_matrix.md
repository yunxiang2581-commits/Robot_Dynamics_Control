# 求职目标与能力矩阵

## 阅读导航

| 上一篇 | 当前文档 | 下一篇 |
| --- | --- | --- |
| [00 准备阶段总览](00_preparation_overview.md) | 求职目标与能力矩阵 | [02 三总项目范围说明](02_three_project_scope.md) |

## 本文用途

本文件把机器人运动控制岗位要求转成项目能力点，并映射到 A/B/C 三条主线。后续每个模块都必须能回答：它为什么对求职有用。

## 核心结论

| 目标 | 说明 |
| --- | --- |
| 求职方向 | 机器人运动控制算法实习、机器人控制算法工程师、人形机器人运动控制方向 |
| 主要能力 | 运动学、动力学、仿真、优化控制、ROS 工程、强化学习、Sim2Real |
| 项目策略 | 用 A 自研证明基础能力，用 B 支撑模型控制，用 C 支撑强化学习控制 |

## 岗位关键词映射

| 岗位关键词 | 能力要求 | 项目支撑 |
| --- | --- | --- |
| 机器人控制 | 理解控制闭环、状态、命令、反馈和误差 | A 自研 PD、IK、Mini-WBC；B WBC |
| 动力学分析 | 理解关节空间、任务空间、质心、接触力和力矩 | A Pinocchio；B OCS2/WBC |
| MuJoCo / Isaac Sim | 使用仿真环境验证控制策略 | A MuJoCo PD；C MuJoCo RL；Isaac 作为扩展 |
| ROS / ROS2 | 理解机器人软件栈和控制接口 | B `ros-control`；ROS2 作为后续扩展 |
| C++ / Python | 编写算法原型并理解工程源码 | A Python baseline；B C++/ROS 项目阅读 |
| PID / 阻抗控制 | 解释低层控制和柔顺控制基础 | A PD 控制；后续扩展阻抗控制 |
| MPC / WBC / QP | 理解优化控制问题建模 | A QP-IK、Mini-WBC；B NMPC/WBC |
| PPO / SAC / TD3 | 理解 RL 控制策略训练和调参 | C `unitree_rl_mjlab`；SAC/TD3 作为对比扩展 |
| Sim2Real | 理解从仿真到部署的安全和误差问题 | C Sim2Real 分析；B 真机控制链路理解 |

## 能力矩阵

| 能力点 | A 自研基础系统 | B `legged_control` | C `unitree_rl_mjlab` | 求职表达 |
| --- | --- | --- | --- | --- |
| 模型加载 | Pinocchio 加载 URDF | 阅读机器人模型和 `ros-control` 配置 | 阅读 MJCF 和 Unitree 模型 | 能处理真实机器人模型 |
| 运动学 | FK、Jacobian、DLS-IK、QP-IK | 理解 WBC 任务空间控制 | 理解 policy 输入中的关节状态 | 能把任务空间目标转成关节空间控制 |
| 动力学 | Mini-WBC 教学版 | NMPC、WBC、接触力、关节力矩 | MuJoCo 物理引擎隐式动力学 | 能理解模型控制的核心变量 |
| 优化 | OSQP/cvxpy QP 骨架 | OCS2、QP-WBC | PPO 优化策略参数 | 能对比数值优化和策略优化 |
| 仿真 | MuJoCo PD 跟踪 | Gazebo/ROS 仿真记录 | MuJoCo Play/Train | 能用仿真验证控制效果 |
| 部署意识 | 日志、图表、参数配置 | 控制频率、状态估计、接口 | Sim2Real、安全保护 | 能讲清工程落地风险 |

## 项目模块映射

| 项目模块 | 对应岗位能力 | 输出产物 | 验收标准 |
| --- | --- | --- | --- |
| URDF/FK/Jacobian | 机器人运动学基础 | A 项文档、脚本、测试 | 能说明 `q`、`dq`、frame、Jacobian 的含义 |
| QP-IK/Mini-WBC | 优化控制和约束处理 | A 项 QP 文档 | 能写出变量、目标函数和约束 |
| `legged_control` 架构拆解 | 传统模型控制工程能力 | B 项架构、NMPC、WBC 笔记 | 能解释 NMPC 和 WBC 分工 |
| `unitree_rl_mjlab` 拆解 | 强化学习运动控制能力 | C 项 obs/action/reward 文档 | 能解释 policy 如何输出控制命令 |
| 模型控制 vs RL 对比 | 面试表达能力 | 对比文档和讲稿 | 能讲清两类方法的本质区别 |

## 准备任务

| 任务编号 | 任务 | 输入 | 输出 | 验收标准 |
| --- | --- | --- | --- | --- |
| PREP-003 | 提炼岗位关键词 | 机器人控制岗位要求 | 本文关键词表 | 关键词覆盖控制、动力学、仿真、ROS、RL |
| PREP-003 | 建立能力矩阵 | A/B/C 三条主线 | 本文能力矩阵 | 每个能力点都能映射到至少一个项目产物 |

