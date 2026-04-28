# 三总项目范围说明

## 阅读导航

| 上一篇 | 当前文档 | 下一篇 |
| --- | --- | --- |
| [01 求职目标与能力矩阵](01_job_target_and_skill_matrix.md) | 三总项目范围说明 | [03 准备阶段任务总表](03_preparation_task_table.md) |

## 本文用途

本文件固定 A/B/C 三条主线的范围，防止项目发散。第一版目标是形成可展示、可讲解、可迭代的求职 baseline，而不是一次性完成完整人形机器人控制系统。

## 核心结论

| 主线 | 准备阶段要明确 | 准备阶段不做 |
| --- | --- | --- |
| A 自研运动控制基础系统 | 模块边界、文件命名、输入输出、验收图表 | 不实现完整算法 |
| B `legged_control` 复现与拆解 | 阅读目标、环境记录、NMPC/WBC/Estimator 拆解目标 | 不立即编译大项目，不修改源码 |
| C `unitree_rl_mjlab` 复现与拆解 | Train/Play/Sim2Real、obs/action/reward 拆解目标 | 不立即大规模训练 |

## 范围总表

| 主线 | 项目定位 | 准备阶段要明确什么 | 后续进入开发的条件 |
| --- | --- | --- | --- |
| A 自研运动控制基础系统 | 证明自己能写基础控制模块 | URDF、FK、Jacobian、DLS-IK、QP-IK、MuJoCo PD、Mini-WBC 的命名、输入输出和验收图表 | 目录、配置、脚本命名、验收图表标准都确定 |
| B `legged_control` 复现与拆解 | 支撑传统模型控制岗位表达 | 阅读目标、环境记录目标、NMPC/WBC/状态估计拆解目标 | 有 setup 模板、源码阅读模板、复现日志模板 |
| C `unitree_rl_mjlab` 复现与拆解 | 支撑强化学习运动控制岗位表达 | Train/Play/Sim2Real、obs/action/reward、policy 部署拆解目标 | 有 RL 文档模板、日志模板、视频输出标准 |

## A 项目范围

| 分类 | 包含 | 不包含 |
| --- | --- | --- |
| 模型 | Pinocchio 加载 URDF、frame 查询 | 大型 mesh 管理、复杂机器人模型适配 |
| 运动学 | FK、Jacobian、有限差分验证 | 高阶动力学推导 |
| 逆运动学 | DLS-IK、QP-IK 教学实现 | 高性能全身 IK |
| 控制 | MuJoCo PD 轨迹跟踪、Mini-WBC QP | 完整人形机器人 WBC |
| 输出 | 误差曲线、日志、文档、面试讲稿 | 真机部署 |

## B 项目范围

| 分类 | 包含 | 不包含 |
| --- | --- | --- |
| 项目调研 | README、论文、目录结构、核心模块职责 | 大规模重写 |
| 环境记录 | ROS、OCS2、依赖、编译日志 | 保证所有平台一次编译成功 |
| 架构拆解 | NMPC -> WBC -> Motor PD -> Estimator 数据流 | 完整 NMPC 求解器开发 |
| 自研对照 | Mini-WBC 与工程 WBC 对比 | 替换原项目控制器 |

## C 项目范围

| 分类 | 包含 | 不包含 |
| --- | --- | --- |
| 项目调研 | Train、Play、Sim2Real 流程 | 从零开发 RL 框架 |
| 模型分析 | MJCF、关节、action 维度 | 大规模模型资产提交 |
| RL 拆解 | observation、action、reward、termination | 保证训练收敛到高性能 policy |
| 部署分析 | policy 输入输出、安全限制、Sim2Real 风险 | 真机部署 |

## 准备任务

| 任务编号 | 任务 | 输出 | 验收标准 |
| --- | --- | --- | --- |
| PREP-004 | 固定 A/B/C 范围 | 本文件 | 每条主线都有做什么和不做什么 |
| PREP-004 | 定义进入开发条件 | 范围总表 | 能判断何时从准备阶段切到开发阶段 |

