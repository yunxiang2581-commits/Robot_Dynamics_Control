# legged_control 外部项目说明

## 项目用途

`qiayuanl/legged_control` 是本 baseline 的传统模型控制支撑项目，用于学习 NMPC、WBC、状态估计、ros-control、腿足机器人动力学和关节力矩控制的工程链路。

## 官方仓库地址

| 项目 | 地址 |
| --- | --- |
| legged_control | `https://github.com/qiayuanl/legged_control` |

## 本项目如何使用它

| 用途 | 说明 |
| --- | --- |
| 架构学习 | 拆解 NMPC -> WBC -> Motor PD -> Estimator 的数据流 |
| 源码阅读 | 记录核心文件职责、输入、输出和调用关系 |
| 复现记录 | 保存环境、依赖、编译命令、失败原因和 fallback |
| 自研对照 | 与 A 项 Mini-WBC 对比变量、目标函数、约束和输出 |

## 准备阶段限制

准备阶段不下载源码，不复制大型第三方仓库，不提交构建产物。后续如需引入源码，优先使用外部目录、submodule 或单独工作区，并只在本仓库记录阅读笔记和复现日志摘要。

## 后续记录位置

| 内容 | 路径 |
| --- | --- |
| 项目阅读入口 | `docs/02_legged_control/README.md` |
| 环境和编译日志摘要 | `outputs/logs/legged_control_build.log` |
| 自研对照文档 | `docs/04_compare/` |

