# A Pipeline Contract

## 1. A 项目总体目标

A 项目是自研机器人运动控制基础系统。目标是用 Pinocchio、MuJoCo 和 OSQP 建立一条从机器人模型到控制原型的可解释学习链路。

当前阶段的重点不是一次性实现完整控制栈，而是把 A01-A07 组织成同一条 pipeline：每一步有明确输入、输出、保存路径和验证方式，后续实现时可以逐步替换 TODO。

## 2. A01-A07 为什么是完整链路

A01-A07 对应机器人运动控制求职中常见的能力链：

```text
A01 URDF 检查
  -> A02 FK frame pose
  -> A03 Jacobian 有限差分验证
  -> A04 DLS-IK
  -> A05 QP-IK with limits
  -> A06 MuJoCo PD tracking
  -> A07 Mini-WBC QP
```

这条链路从模型结构开始，逐步进入运动学、约束优化、仿真闭环和 WBC 结构。每一步都应能独立验证，但不应被当成彼此孤立的 demo。

## 3. A01-A07 的依赖关系

- A01 产生模型摘要、joint/frame 名称清单，是所有后续步骤的基础。
- A02 消费 A01 的模型和 frame 选择，产生目标 frame 位姿报告。
- A03 消费 A02 中确认的 frame，验证 Jacobian 是否能正确描述 frame 速度。
- A04 消费 A03 验证过的 Jacobian，产生 DLS-IK 的 `q` 轨迹和误差曲线。
- A05 消费 A04 的任务定义，把 DLS 更新改造成带关节限制的 QP-IK 问题。
- A06 消费 A04 或 A05 产生的关节轨迹，在 MuJoCo 中做 PD tracking。
- A07 消费 A03-A06 中形成的任务、约束、轨迹和误差概念，过渡到教学版 WBC/QP 结构。

## 4. 每一步输入、输出和保存路径

### A01 - Inspect URDF

- 输入：URDF 路径、package/mesh 搜索路径、是否使用 floating base。
- 输出：`nq`、`nv`、joint 名称、frame 名称、候选脚端/躯干 frame。
- 保存路径：
  - `outputs/reports/A01_inspect_urdf.md`
  - `outputs/cache/A01_model_summary.json`

### A02 - FK Frame Pose

- 输入：A01 确认的 URDF、目标 frame、关节配置 `q`。
- 输出：目标 frame 的位置、旋转、SE3 位姿摘要。
- 保存路径：
  - `outputs/reports/A02_fk_frame_pose.md`
  - `outputs/cache/A02_frame_pose.json`

### A03 - Jacobian FD Check

- 输入：A01/A02 确认的模型、目标 frame、`q`、`dq`、`dt`。
- 输出：解析 Jacobian、有限差分速度、误差范数。
- 保存路径：
  - `outputs/reports/A03_jacobian_fd_check.md`
  - `outputs/figures/A03_jacobian_error.png`
  - `outputs/cache/A03_jacobian_check.json`

### A04 - DLS IK

- 输入：A03 验证过的 Jacobian、目标 frame、目标位置、初始 `q`、阻尼和步长。
- 输出：IK 误差曲线、`q` 轨迹、最终 frame 位姿。
- 保存路径：
  - `outputs/reports/A04_dls_ik.md`
  - `outputs/trajectories/A04_dls_ik_q_traj.csv`
  - `outputs/figures/A04_dls_ik_error.png`

### A05 - QP-IK With Joint Limits

- 输入：A04 的任务定义、A03 的 Jacobian、关节位置/速度上下限、QP 权重。
- 输出：QP 状态、受约束 `dq` 或 `q` 轨迹、约束违反量。
- 保存路径：
  - `outputs/reports/A05_qp_ik_joint_limit.md`
  - `outputs/trajectories/A05_qp_ik_q_traj.csv`
  - `outputs/cache/A05_qp_status.json`

### A06 - MuJoCo PD Tracking

- 输入：MuJoCo XML、A04 或 A05 产生的关节轨迹、PD 增益、仿真时间。
- 输出：跟踪日志、关节位置/速度/力矩曲线、可选视频。
- 保存路径：
  - `outputs/reports/A06_mujoco_pd_tracking.md`
  - `outputs/logs/A06_pd_tracking.csv`
  - `outputs/figures/A06_pd_tracking_error.png`
  - `outputs/videos/A06_pd_tracking.mp4`

### A07 - Mini-WBC QP

- 输入：A03 的 Jacobian 概念、A05 的 QP 约束概念、A06 的仿真反馈概念、任务权重和接触约束占位。
- 输出：WBC QP 变量说明、矩阵维度、任务和约束结构报告。
- 保存路径：
  - `outputs/reports/A07_mini_wbc_qp.md`
  - `outputs/cache/A07_qp_structure.json`

## 5. A04 DLS-IK 和 A05 QP-IK 的关系

A04 是无显式约束的数值 IK 学习入口，重点是理解误差、Jacobian、阻尼和 `pin.integrate`。

A05 在 A04 的基础上引入约束：关节速度限制、关节位置限制、正则项和 OSQP 求解状态。A05 不应重新定义一套孤立问题，而应复用 A04 的目标 frame、目标位置、误差定义和验证方式。

## 6. A06 如何消费 A04/A05 的轨迹

A06 不应只生成一个独立正弦 demo。标准 pipeline 中，A06 的主要输入应该是：

- A04 生成的 DLS-IK 轨迹；
- 或 A05 生成的 QP-IK 轨迹；
- MuJoCo XML；
- PD 增益和仿真时间。

第一版实现可以先使用占位轨迹或单关节轨迹，但接口和文档应保留“消费 IK 轨迹”的位置。

## 7. A07 如何作为 WBC/legged_control 的过渡

A07 是教学版 Mini-WBC QP，不直接复刻 B 项目的完整 WBC。它的作用是把 A 项目中已经建立的概念串起来：

- A03：任务 Jacobian；
- A05：QP 目标项和约束；
- A06：仿真反馈和跟踪误差；
- B 项目：后续阅读 `legged_control` WBC/NMPC 时的概念桥梁。

## 8. 验收标准

- A01-A07 的 README、docstring 和 TODO 都明确自己在 pipeline 中的位置。
- 每一步都说明前置输入、后续输出和保存路径。
- `outputs/README.md` 定义统一输出目录。
- `pipeline_io.py` 提供统一路径和占位写入函数的 TODO 骨架。
- 所有核心算法仍保留 TODO，未实现完整 FK、Jacobian、IK、QP、WBC 或 MuJoCo 控制。
