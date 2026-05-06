# Mink Capability vs A Requirements

## 1. 文档目的

本文件用于明确：

- mink 项目具体能做到什么；
- A 项目需要对标实现什么；
- 哪些功能是第一阶段必须实现；
- 哪些功能只做 TODO 或后续扩展；
- 最终可展示成果是什么。

A 项目当前定位是：对标 `kevinzakka/mink` UR5e 示例的教学版 6-DOF 机械臂控制 baseline。

A 项目同时是 simulation-first baseline：当前没有实物 UR5e / 机械臂，因此验证策略优先放在 MuJoCo 仿真、可量化日志、误差曲线、demo video 和后续 sim2sim validation。完整全流程规划见 `A_simulation_only_full_motion_control_plan.md`。sim2sim 是无实物条件下的工程验证策略，不是“真实机器人替代品”。

## 2. mink 能力拆解

mink 的核心能力是：

> 给定机器人当前 configuration 和一组 task-space objectives，在 respecting limits/constraints 的前提下，求一个局部最优 joint velocity。

### MuJoCo Robot Model Loading

mink 基于 MuJoCo 模型工作。UR5e 示例通过 MJCF 资产加载机器人、关节、body、site、actuator 和 keyframe。

### Configuration Abstraction

mink 用 configuration 抽象管理当前机器人状态。它把 `q`、MuJoCo model/data、frame/site pose 查询和更新组织到统一入口中。

### Task Abstraction

mink 用 task 表达任务空间目标，例如末端 frame/site 到目标 pose 的误差。UR5e 示例中重点关注末端执行器跟踪。

### Limit / Constraint Abstraction

mink 用 limit/constraint 表达配置限制、速度限制和可选碰撞约束。这让 IK 不只是最小化误差，也要满足边界条件。

### QP-Based Differential IK

mink 的 differential IK 会把 task objective 和 limits 组织成 QP，求解局部最优 joint velocity，再更新 configuration。

### UR5e Examples

UR5e 示例展示了固定基 6-DOF 机械臂的模型加载、任务定义、目标跟踪和 IK 迭代流程，是 A 项目第一条完整 baseline 的对照对象。

### Actuator Tracking Example

actuator tracking 示例展示了如何把期望关节状态送入 MuJoCo actuator/control，形成可以展示的视频 demo。

### Optional Collision Avoidance

mink 支持 collision avoidance 相关限制。A 项目第一阶段只做 TODO 和概念记录，不实现完整避障。

### Viewer / Target / Mocap-Style Interaction

mink 示例中 viewer target 或 mocap-style target 用于交互式定义任务空间目标。A 项目后续会先做 fixed target，再扩展 mocap-style target。

## 3. A 项目实现边界

### A 项目要做

- 自己写最小模型检查；
- 自己写 site pose 查询；
- 自己写 site Jacobian 检查；
- 自己写 DLS differential IK；
- 自己写最小 QP-IK；
- 自己写 target / mocap-style tracking；
- 自己写 MuJoCo actuator tracking；
- 自己输出报告、日志、轨迹和视频；
- 自己写与 mink 的对比报告。

### A 项目不做或暂缓

- 不完整复制 mink 源码；
- 不直接调用 mink 替代自己的实现；
- 不完整实现 collision avoidance；
- 不追求完整 API 设计；
- 不追求多机器人 examples；
- 不追求库级测试覆盖。

## 4. A00-A10 任务表

| 步骤                                | A 项目目标                   | 对标 mink                      | 需要实现                                                    | 输出物                                | 验收标准                          |
| ----------------------------------- | ---------------------------- | ------------------------------ | ----------------------------------------------------------- | ------------------------------------- | --------------------------------- |
| A00 reference and assets            | 固定上游参考与 UR5e 资产路径 | examples / model assets        | 审计并记录最小参考资产                                      | reference docs / asset README         | 路径、来源、许可证清楚            |
| A01 model inspect                   | 检查 MJCF 模型对象           | MuJoCo model loading           | 读取 model 并列出 `nq/nv/nu`、joint、body、site、actuator | report / summary JSON                 | 能确认末端候选和 actuator 名称    |
| A02 configuration / site pose       | 查询 site/body pose          | `Configuration`              | 从 `q` 更新 data 并读取 pose                              | report / pose JSON                    | pose 随 q 变化合理                |
| A03 site Jacobian check             | 验证速度映射                 | site Jacobian                  | 计算 `mj_jacSite` 并有限差分验证                          | report / error figure / cache         | `J dq` 与有限差分速度一致       |
| A04 DLS differential IK             | 实现无约束 IK baseline       | minimal `solve_ik`           | 用 DLS 从误差求 `dq`                                      | trajectory / log / figure / report    | 误差下降且无 NaN                  |
| A05 task + limit + QP-IK            | 引入 task 与 limits          | `FrameTask` / limits / QP    | 最小 QP-IK 与速度限制                                       | trajectory / constraints log / report | 满足 limit 且误差下降             |
| A06 target / mocap-style tracking   | 管理目标输入                 | viewer target / mocap target   | fixed target，后续 mocap-style target                       | tracking log / report                 | target、site pose、误差可复盘     |
| A07 MuJoCo actuator tracking        | 形成仿真控制展示             | `arm_ur5e_actuators.py`      | 将 `q_des/dq_des` 送入 `data.ctrl`                      | log / figure / video / report         | actuator 维度正确且 tracking 有界 |
| A08 collision avoidance TODO        | 记录避障扩展                 | collision avoidance constraint | TODO 设计，不做完整实现                                     | TODO doc / report section             | 输入、约束、验证思路明确          |
| A09 comparison report               | 和 mink 逐项对照             | example-level comparison       | 汇总 A01-A08 与 mink 差异                                   | comparison report                     | 能说明实现、抽象差距和下一步      |
| A10 demo showcase / video recording | 形成求职展示物               | demo / viewer video            | 整理视频、日志、图和 README 展示                            | demo video / release notes            | GitHub README 或 Release 可展示   |

## 5. 最终 Demo 定义

主 demo：

```text
outputs/videos/A07_ur5e_actuator_tracking_demo.mp4
```

内容包括：

- UR5e 机械臂在 MuJoCo 中跟踪 A04/A05 生成的目标关节轨迹；
- 配套 tracking log；
- 配套 tracking error figure；
- 配套 report；
- README 或 GitHub Release 展示方式。

配套输出建议：

```text
outputs/logs/A07_actuator_tracking.csv
outputs/figures/A07_tracking_error.png
outputs/reports/A07_actuator_tracking_report.md
```

可选 demo：

- `A04_dls_ik_target_tracking_demo.mp4`；
- `A05_qp_ik_joint_limit_demo.mp4`。

## 6. 目录与文件建议

推荐目录：

```text
projects/A_self_baseline/
├── configs/
├── scripts/
├── src/robot_baseline/
├── outputs/
└── docs/
```

建议新增或后续会补的模块：

- `configuration.py`
- `tracking.py`
- `video_recording.py`
- `record_demo_video.py`
- `demo_showcase.md`

## 7. 后续实现顺序

- Step 9A：A01 model inspect 算法规划（历史规划步骤）
- Step 9B：A01 最小可运行 MJCF inspect
- Step 10A：A02 configuration / site pose 算法规划（历史规划步骤）
- Step 10B：A02 最小可运行 site pose
- Step 11A：A03 site Jacobian 算法规划（历史规划步骤）
- Step 11B：A03 Jacobian finite difference
- Step 12A：A04 DLS differential IK 算法规划（历史规划步骤）
- Step 12B：A04 已完成最小 position-mode DLS IK
- Step 13A：A05 task + limit + QP-IK 算法规划（历史规划步骤）
- Step 13B：A05 已完成最小 box-constrained QP-IK
- Step 14：A06 target / mocap-style tracking
- Step 15：A07 actuator tracking
- Step 16：A08 collision avoidance TODO
- Step 17：A09 comparison report
- Step 18：A10 demo showcase / video recording

## 8. Codex 后续任务原则

- 先做 TODO 骨架；
- 再补最小可运行实现；
- 每次只补一个环节；
- 每个 TODO 说明目的、API、输入、输出和验证；
- 不直接调用 mink 替代自己的实现；
- 不把完整 mink 库复制进 A 项目；
- 每步都要有可验证输出。
