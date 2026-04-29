# A Mink Alignment Plan

## 阶段 1：model inspect、configuration/site pose、site Jacobian

- 输入：`configs/robot.yaml`、UR5e `scene.xml`、末端 site 候选名称。
- 输出：模型摘要、site pose 报告、Jacobian 有限差分验证报告。
- 验收标准：能解释 `nq`、`nv`、`nu`、joint、body、site、actuator；能从 `q` 查询 site pose；TODO 中明确 `mj_jacSite` 验证方式。
- 对标 mink 的概念：MuJoCo model loading、`Configuration`、site Jacobian。

## 阶段 2：DLS differential IK

- 输入：当前 site pose、目标 site pose、site Jacobian、阻尼和 gain。
- 输出：`q` 轨迹、误差日志、收敛报告。
- 验收标准：TODO 中写清误差定义、DLS 更新公式、输入输出和验证方法；不实现完整算法。
- 对标 mink 的概念：`solve_ik` 的最小无约束教学版。

## 阶段 3：task + limit + QP-IK

- 输入：FrameTask/PostureTask 风格任务定义、关节位置/速度限制、QP 权重。
- 输出：QP-IK 轨迹、约束日志、求解状态报告。
- 验收标准：TODO 中写清最小 QP 形式和 `dq_min <= dq <= dq_max` 约束；暂不做完整 collision avoidance。
- 对标 mink 的概念：`FrameTask`、`PostureTask`、`ConfigurationLimit`、`VelocityLimit`。

## 阶段 4：target / mocap-style tracking

- 输入：固定 target 或后续 mocap target、当前 site pose、IK 求解入口。
- 输出：target tracking 日志和报告。
- 验收标准：TODO 中说明 fixed target 第一版和 `data.mocap_pos` / `data.mocap_quat` 后续扩展。
- 对标 mink 的概念：viewer target、mocap-style target。

## 阶段 5：MuJoCo actuator tracking

- 输入：A04/A05 生成的 `q_des` 或 `dq_des`、MuJoCo actuator 名称、control range。
- 输出：actuator tracking CSV、误差曲线、可选视频和报告。
- 验收标准：TODO 中明确 `data.ctrl`、`mujoco.mj_step`、actuator 名称和 tracking error 检查。
- 对标 mink 的概念：`arm_ur5e_actuators.py`。

## 阶段 6：collision avoidance TODO

- 输入：A05 QP-IK 结构、碰撞几何、最小距离阈值。
- 输出：collision avoidance TODO 设计说明。
- 验收标准：只记录概念、输入输出和验证思路，不实现完整避障。
- 对标 mink 的概念：collision avoidance constraint。

## 阶段 7：comparison report

- 输入：A01-A08 的报告、日志、轨迹和 TODO 完成情况。
- 输出：A 项目与 mink UR5e 示例的对照报告。
- 验收标准：能说明自己实现链路、mink 抽象、当前差距和下一步补齐顺序。
- 对标 mink 的概念：example-level comparison。
