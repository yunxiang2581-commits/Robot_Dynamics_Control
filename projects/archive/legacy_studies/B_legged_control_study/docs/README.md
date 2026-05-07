# B legged_control 复现与拆解

## 当前状态

本目录是 B 项目的文档占位。准备阶段不下载或编译 `legged_control` 源码。

## 后续内容

| 文档 | 目标 | 验收标准 |
| --- | --- | --- |
| `legged_control_overview.md` | 项目概览 | 能说明项目解决什么问题 |
| `legged_control_setup.md` | 环境和编译记录 | 成功或失败都有完整摘要 |
| `legged_control_architecture.md` | 总体架构 | 有 NMPC -> WBC -> Motor PD -> Estimator 数据流 |
| `legged_control_nmpc_notes.md` | NMPC 拆解 | 能讲清优化变量、代价和约束 |
| `legged_control_wbc_notes.md` | WBC 拆解 | 能讲清 QP 变量、任务、约束和输出 |
| `legged_control_estimator_notes.md` | 状态估计拆解 | 能讲清 base 状态和接触估计意义 |

