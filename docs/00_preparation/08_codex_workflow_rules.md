# Codex 工作流规则

## 阅读导航

| 上一篇 | 当前文档 | 下一篇 |
| --- | --- | --- |
| [07 机器人模型与资源计划](07_robot_model_and_asset_plan.md) | Codex 工作流规则 | [09 风险与备选方案](09_risk_and_fallback_plan.md) |

## 本文用途

本文件约束后续 Codex 如何修改项目。核心原则是先做教学骨架，再逐步补全算法，保证每一步都能解释、验证和用于面试复盘。

## 核心结论

| 规则 | 说明 |
| --- | --- |
| 当前阶段 | 只允许写文档、表格、目录、模板 |
| 算法阶段 | 先生成 TODO 教学骨架，再逐步实现 |
| 注释要求 | 核心 TODO 使用中文说明输入、输出、推荐 API、验证方式 |
| 外部项目 | 不下载大仓库，不魔改第三方源码 |

## 项目内 Skill 技能库

旧仓库 `Pinocchio_URDF` 中没有标准 `SKILL.md` 技能包，但有 `AGENTS.md` 和 `AGENT.MD` 两份 Codex 协作规则。当前仓库已将它们转换为可提交的项目内 skill：

- `tools/codex_skills/pinocchio-learning/SKILL.md`
- `tools/codex_skills/pinocchio-learning/references/AGENTS_from_Pinocchio_URDF.md`
- `tools/codex_skills/pinocchio-learning/references/AGENT_from_Pinocchio_URDF.md`

后续处理 A 项目 Pinocchio、MuJoCo、FK、Jacobian、IK、PD 学习脚本时，应优先遵守该 skill 中的路径、注释、TODO 和验证规则。

## 总规则

| 编号 | 规则 | 原因 | 验收方式 |
| --- | --- | --- | --- |
| R1 | 不一次性实现完整复杂算法 | 防止学习目标失控，避免黑盒代码 | PR 或提交中只包含当前阶段任务 |
| R2 | 核心算法先保留 TODO 教学骨架 | 让学习者明确要补什么、为什么补 | TODO 注释包含输入、输出、推荐 API、验证方法 |
| R3 | 中文注释说明算法意图 | 服务求职复盘和面试表达 | 关键 TODO 使用中文 |
| R4 | 不下载大型第三方仓库 | 保持仓库轻量 | B/C `external/` 只放说明或 submodule 占位 |
| R5 | 不提交大型模型、视频、日志、权重 | 保持 Git 历史干净 | `.gitignore` 覆盖相关路径 |
| R6 | 每个阶段必须有验收标准 | 避免只写代码不验证 | 文档、脚本、测试或日志能对应检查 |

## TODO 教学骨架格式

后续进入算法阶段时，核心 TODO 建议使用以下格式：

```python
# TODO: 实现这里的机器人控制步骤。
# 要实现什么：说明当前函数要完成的数学或工程功能。
# 求职意义：说明该步骤对应岗位中的哪类能力。
# 推荐 API：列出建议使用的 Pinocchio、MuJoCo、OSQP、NumPy 等 API。
# 输入：说明参数的物理含义和维度。
# 输出：说明返回值的物理含义和维度。
# 验证：说明如何用数值误差、图表、日志或单元测试验证。
```

## 后续开发顺序

| 阶段 | Codex 行为 | 不允许行为 |
| --- | --- | --- |
| 准备阶段 | 写文档、表格、目录、模板 | 写完整 FK/Jacobian/IK/QP/WBC/RL 实现 |
| A 骨架阶段 | 创建函数接口、配置模板、脚本入口、测试骨架 | 一次补全所有算法 |
| A 实现阶段 | 每次只实现一个模块并验证 | 混合改多个无关模块 |
| B 拆解阶段 | 读文档、写架构和源码阅读笔记 | 魔改第三方项目源码 |
| C 拆解阶段 | 分析 obs/action/reward 和运行记录 | 追求大规模训练收敛 |

## 准备任务

| 任务编号 | 任务 | 输出 | 验收标准 |
| --- | --- | --- | --- |
| PREP-010 | 编写 Codex 工作流规则 | 本文件 | 明确不要直接写完整复杂算法 |
| PREP-010 | 固定 TODO 格式 | TODO 教学骨架格式 | 后续代码注释有统一模板 |
| PREP-010 | 固定限制 | 总规则表 | 大仓库和大文件不会进入本仓库 |
