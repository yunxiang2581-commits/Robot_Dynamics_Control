# A Pipeline Contract

## 1. A 项目当前目标

A 项目当前主线调整为：

> 对标 `kevinzakka/mink` UR5e 示例的教学版 6-DOF 机械臂控制 baseline。

这里的“对标”不是完整复刻 mink 库，也不是直接调用 mink 替代自己的实现。A 项目要做的是逐步复现 mink UR5e 示例背后的核心链路：模型检查、configuration/site pose、site Jacobian、微分 IK、task/limit/QP-IK、target tracking、actuator tracking，以及最终和 mink 的概念对照报告与 demo showcase。

需求总文档见 `mink_capability_vs_A_requirements.md`。

H1 legacy 仍然保留为历史学习参考，不删除、不覆盖，但它不再是当前 A 项目的主线。

## 2. 新 Pipeline 链路

```text
A00 reference and assets
  -> A01 model inspect
  -> A02 configuration / site pose
  -> A03 site Jacobian check
  -> A04 DLS differential IK
  -> A05 task + limit + QP-IK
  -> A06 target / mocap-style tracking
  -> A07 MuJoCo actuator tracking
  -> A08 collision avoidance TODO
  -> A09 comparison report
  -> A10 demo showcase / video recording
```

这条链路以 6-DOF 机械臂为当前学习对象，优先对齐 mink UR5e 示例中的 MuJoCo model、configuration、site、task、limit、QP 和 actuator tracking 概念。

## 2.1 Simulation-first 扩展规划

A 项目当前没有实物 UR5e / 机械臂，因此采用 simulation-first baseline：先在 MuJoCo 中完成可运行、可解释、可量化、可复现的运动控制链路，再用 sim2sim validation 作为无实物条件下的工程验证策略。

完整规划见 `A_simulation_only_full_motion_control_plan.md`。该规划将 A00-A10 扩展到 A18，但不改变当前实现顺序；A03 最小实现已完成，下一步进入 A04 DLS differential IK。

## 3. 每一步契约

### A00 - Reference and Assets

- 输入：`external/mink_upstream/` 上游只读镜像、UR5e/MJCF 模型资产路径。
- 输出：最小参考材料、资产路径约定和本地来源记录。
- 对标 mink 的概念：example、model asset、viewer target、actuator example。
- 验收标准：`reference_mink_ur5e.md` 说明清楚参考范围，模型路径为 `shared/robot_assets/models/mink_universal_robots_ur5e/scene.xml`；对照脚本路径为 `projects/A_self_baseline/external/mink/examples/`。

### A01 - Model Inspect

- 输入：`configs/robot.yaml`、`shared/robot_assets/models/mink_universal_robots_ur5e/scene.xml`，可选 `urdf_path`。
- 输出：`nq`、`nv`、`nu`、joint、body、site、actuator、keyframe 和末端候选对象清单。
- 对标 mink 的概念：MuJoCo model loading、UR5e scene inspect、configuration 的基础模型维度。
- 当前状态：最小可运行实现已完成，已生成 `outputs/reports/A01_model_inspect_report.md` 和 `outputs/cache/A01_model_summary.json`。
- 验收标准：报告列出模型维度和关键 site/body/actuator/keyframe；不依赖旧项目绝对路径。

### A02 - Configuration / Site Pose

- 输入：A01 确认的 MJCF 模型、配置 `q`、末端 site 名称。
- 输出：目标 site/body 的位置、旋转矩阵和可复盘的 JSON/Markdown 摘要。
- 对标 mink 的概念：`mink.Configuration` 中从 `q` 到当前 frame/site pose 的查询能力。
- 当前状态：最小可运行实现已完成，依赖 A01 的 `outputs/cache/A01_model_summary.json` 和已确认的 `attachment_site` / `wrist_3_link`。
- 当前输出：`outputs/cache/A02_site_pose.json` 和 `outputs/reports/A02_site_pose_report.md`。
- 验收标准：能说明 `q -> MuJoCo data -> site pose` 的数据流，并记录目标 site 的 pose。

### A03 - Site Jacobian Check

- 输入：A02 确认的 site、配置 `q`、扰动 `dq` 和有限差分步长。
- 输出：site Jacobian、有限差分速度、误差指标和图像占位。
- 对标 mink 的概念：differential IK 背后的速度映射 `site velocity = J(q) dq`。
- 当前状态：最小可运行实现已完成，依赖 A01/A02 的 model summary 和 site pose。
- 当前输出：`outputs/cache/A03_jacobian_check.json`、`outputs/reports/A03_jacobian_check_report.md`、`outputs/figures/A03_jacobian_fd_error.png`、`outputs/cache/A03_multi_step_trace.json` 和 `outputs/figures/A03_multi_step_linearization_error.png`。
- 当前结果：默认 `dq_source=unit:shoulder_pan`、`dt=1e-6`、`fd_steps=1000`；linear velocity error norm 约为 `2.55e-4`，angular velocity error norm 约为 `8.26e-11`；sweep 图已展示误差随总位移变化的趋势。
- 下一步：进入 A04 DLS differential IK。
- 验收标准：能说明 `site velocity = J(q) dq`，并用 finite difference 验证 MuJoCo site Jacobian 的线速度和角速度映射。

### A04 - DLS Differential IK

- 输入：A03 验证过的 site Jacobian、当前 site pose、目标 site 位置/姿态。
- 输出：`q` 轨迹、误差曲线、收敛报告。
- 对标 mink 的概念：`solve_ik` 的最小无约束教学版。
- 当前状态：TODO learning skeleton 已补充，依赖 A03 已验证的 Jacobian。
- 文档状态：`04_dls_differential_ik.md` 已补充 DLS IK 公式、推导、符号表、物理意义、伪代码、验证标准和常见错误。
- 后续输出：`outputs/trajectories/A04_dls_ik_q_traj.npy`、`outputs/logs/A04_dls_ik_error.csv`、`outputs/figures/A04_dls_ik_error.png` 和 `outputs/reports/A04_dls_ik_report.md`。
- 下一步：A04 最小实现将在 Step 12B 完成。
- 验收标准：TODO 中写清 DLS 数学结构 `dq = J.T @ solve(J @ J.T + lambda I, gain * e)`，但不实现完整算法。

### A05 - Task + Limit + QP-IK

- 输入：A04 的任务误差、A03 的 site Jacobian、关节速度/位置限制、QP 权重。
- 输出：QP-IK 轨迹、约束日志和求解报告。
- 对标 mink 的概念：`FrameTask`、`PostureTask`、`ConfigurationLimit`、`VelocityLimit`。
- 验收标准：TODO 中说明最小 QP 形式和 `scipy.optimize`/`osqp` 可选实现方向；暂不做完整 collision avoidance。

### A06 - Target / Mocap-Style Tracking

- 输入：A01 模型、目标 site、固定 target 或后续 mocap-style target。
- 输出：target tracking 日志和报告。
- 对标 mink 的概念：UR5e viewer target、mocap target 驱动任务空间目标。
- 验收标准：TODO 中说明第一版 fixed target、后续扩展 `data.mocap_pos` 和 `data.mocap_quat`。

### A07 - MuJoCo Actuator Tracking

- 输入：A04/A05 生成的 `q_des` 或 `dq_des`、MuJoCo actuator 名称和控制参数。
- 输出：actuator tracking 日志、误差图、可选视频和报告。
- 对标 mink 的概念：`arm_ur5e_actuators.py` 中把 IK 结果送入 MuJoCo actuator/control 的思路。
- 验收标准：TODO 中明确 `data.ctrl`、`mujoco.mj_step` 和 actuator 名称检查；不承诺完整 humanoid WBC。

### A08 - Collision Avoidance TODO

- 输入：A05 的 QP-IK 结构、后续确认的碰撞几何、最小距离阈值。
- 输出：collision avoidance TODO 设计记录。
- 对标 mink 的概念：collision avoidance constraint。
- 验收标准：只保留 TODO，不实现完整避障；说明它为什么在 QP-IK 之后引入。

### A09 - Comparison Report

- 输入：A01-A08 的报告、日志和轨迹。
- 输出：A 项目与 mink UR5e 示例的对照报告。
- 对标 mink 的概念：把自己实现的模型检查、site pose、IK、tracking 与 mink 示例逐项比较。
- 验收标准：报告能说明“自己实现了什么”“mink 提供了什么抽象”“差距在哪里”“后续如何补齐”。

### A10 - Demo Showcase / Video Recording

- 输入：A04/A05 轨迹、A07 actuator tracking 日志、误差图、视频和 A09 comparison report。
- 输出：demo showcase 文档、README 展示片段、可选 GitHub Release 说明和主 demo 视频。
- 对标 mink 的概念：example demo / viewer video / actuator tracking showcase。
- 验收标准：主视频 `outputs/videos/A07_ur5e_actuator_tracking_demo.mp4` 可展示 UR5e 在 MuJoCo 中跟踪目标轨迹，并配套 logs、figures、reports、trajectories 和 comparison report。

## 4. A04 与 A05 的关系

A04 是最小 DLS differential IK，用来理解误差、Jacobian、阻尼和迭代更新。

A05 在 A04 的任务定义基础上加入 task 权重和 limit 约束，过渡到更接近 mink 的 QP-IK。A05 不应重新写成孤立 demo，而应复用 A04 的目标定义、误差定义和验证方式。

## 5. A06 与 A07 的关系

A06 学习 target / mocap-style tracking 的任务生成方式：目标从哪里来，如何影响 site 误差。

A07 学习 actuator tracking：把 A04/A05 产生的期望关节状态送入 MuJoCo `data.ctrl`，检查 actuator、control range、tracking error 和仿真稳定性。

## 6. H1 Legacy 的新定位

- H1 legacy 是历史学习参考，保留在 `legacy_imported/` 中。
- H1 legacy 不删除，因为它记录了早期 Pinocchio/URDF/FK/Jacobian/IK 学习路径。
- 当前 A 项目主线切换为 UR5e / 6-DOF manipulator / mink-style baseline。
- 新标准入口统一为 `scripts/00_...` 到 `scripts/10_...`，不是 `legacy_imported/`。

## 7. mink 本地参考资产边界

- `external/mink_upstream/` 是上游完整仓库镜像，只读参考，不是 A 项目标准入口。
- A 项目只复制最小 UR5e assets/examples：
  - `shared/robot_assets/models/mink_universal_robots_ur5e/`
  - `projects/A_self_baseline/external/mink/examples/arm_ur5e.py`
  - `projects/A_self_baseline/external/mink/examples/arm_ur5e_actuators.py`
- copied files are reference assets/examples, not our implementation。
- 标准实现仍然在 `projects/A_self_baseline/scripts/` 和 `projects/A_self_baseline/src/robot_baseline/`。

## 8. 验收标准

- README、配置和脚本 docstring 都明确当前主线是 UR5e / mink-style 6-DOF manipulator。
- A01-A10 的输入、输出、mink 对标概念和验收标准清楚。
- 最终展示物包括 reports、trajectories、logs、figures、videos 和 comparison report。
- H1 legacy 被明确标记为历史参考，而不是当前主线。
- 不调用 mink 替代自己的实现。
- 不实现 FK、Jacobian、IK、QP、WBC、MuJoCo 控制算法。
