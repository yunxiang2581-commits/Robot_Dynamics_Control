# A Mink Alignment Plan

需求总文档见 `mink_capability_vs_A_requirements.md`。本计划负责把需求拆成阶段，需求总文档负责定义 mink 能力、A 项目边界和最终展示交付物。

simulation-first 全流程规划见 `A_simulation_only_full_motion_control_plan.md`。当前没有实物 UR5e / 机械臂，因此 A 项目用 MuJoCo 主线和后续 sim2sim validation 作为无实物条件下的工程验证策略；这不改变当前下一步 A03 site Jacobian check。

## 阶段 1：model inspect、configuration/site pose、site Jacobian

- 输入：`configs/robot.yaml`、`shared/robot_assets/models/mink_universal_robots_ur5e/scene.xml`、末端 site 候选名称。
- 输出：模型摘要、site pose 报告、Jacobian 有限差分验证报告。
- 验收标准：能解释 `nq`、`nv`、`nu`、joint、body、site、actuator；能从 `q` 查询 site pose；TODO 中明确 `mj_jacSite` 验证方式。
- 对标 mink 的概念：MuJoCo model loading、`Configuration`、site Jacobian。
- 本地参考：`projects/A_self_baseline/external/mink/examples/arm_ur5e.py`。

A02 当前状态：configuration / site pose 最小可运行实现已完成，依赖 A01 的 model summary 和已确认的 `attachment_site` / `wrist_3_link`。当前输出为 `outputs/cache/A02_site_pose.json` 和 `outputs/reports/A02_site_pose_report.md`。下一步进入 A03 site Jacobian check。

A03 当前状态：site Jacobian check TODO learning skeleton 已补充，依赖 A01/A02 的 model summary 和 site pose，后续输出规划为 `outputs/cache/A03_jacobian_check.json`、`outputs/reports/A03_jacobian_check_report.md` 和 `outputs/figures/A03_jacobian_fd_error.png`。A03 的最小实现将在 Step 11B 完成。

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
- 本地参考：`projects/A_self_baseline/external/mink/examples/arm_ur5e_actuators.py`。

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
- 边界说明：`external/mink_upstream/` 是上游完整仓库镜像，不是 A 项目标准入口；copied files are reference assets/examples, not our implementation。

## 阶段 8：demo showcase / video recording

- 输入：A04/A05 轨迹、A07 actuator tracking 日志、误差图、报告和 A09 comparison report。
- 输出：`outputs/videos/A07_ur5e_actuator_tracking_demo.mp4`、demo showcase 文档、README 或 GitHub Release 展示说明。
- 验收标准：视频能展示 UR5e 机械臂在 MuJoCo 中跟踪目标关节轨迹，并能用日志、误差图和报告解释结果。
- 对标 mink 的概念：example demo、viewer/actuator showcase。
- 求职展示意义：demo video 是最终可展示交付物，用来把模型、IK、QP、tracking 和报告串成一个可理解成果。
