# Step 11P - Simulation-First Full Motion Control Plan

## 1. 为什么新增 simulation-first 全流程规划

A_self_baseline 当前是对标 `kevinzakka/mink` UR5e 示例的教学版 6-DOF 机械臂运动控制 baseline。由于当前没有实物 UR5e 或机械臂，需要明确本项目采用 simulation-first 路线：先在仿真中把模型、运动学、Jacobian、IK、QP-IK、tracking、demo 和评估指标跑通。

这个规划服务于机器人运动控制求职项目展示。它不把仿真包装成真机验证，而是把仿真作为无实物条件下可解释、可复现、可量化的工程验证策略。

## 2. 没有实物机械臂时如何定义有效验证

有效验证不等于上真机。当前项目的有效验证由以下标准组成：

- 可运行：脚本能从配置读取模型并稳定输出文件。
- 可解释：能说明每一步输入、输出、公式和物理意义。
- 可量化：有 tracking error、final pose error、joint limit、constraint violation 等指标。
- 可复现：固定配置、固定目标、固定输出目录，结果可重复生成。
- 可迁移：同一轨迹或目标可以在不同仿真设置中检查。
- 可展示：有视频、误差曲线、benchmark report 和 README 展示说明。

## 3. A00-A18 扩展

Step 11P 将原 A00-A10 主线扩展为 A00-A18：

- A00-A10：第一阶段主线，形成求职展示 baseline。
- A11-A18：第二阶段扩展，增强轨迹、控制、鲁棒性、sim2sim 和 benchmark 能力。

当前实现顺序不变。不要把 A11-A18 插入 A03-A07 之前。

## 4. sim2sim 验证策略

sim2sim 定义为：同一控制任务、同一轨迹或同一目标，在不同仿真模型、不同参数设置或不同仿真后端中验证控制链路是否仍然稳定。

规划分三层：

- MuJoCo-to-MuJoCo 参数扰动版：nominal UR5e 对比 perturbed UR5e，扰动 mass、damping、friction、joint damping、actuator gain。
- MuJoCo-to-Other Simulator：后续可选 PyBullet、Gazebo、Isaac Sim 或另一个 MuJoCo 模型版本。
- Same Controller, Different Model Variant：同一 controller 对比不同 MJCF 或 actuator 配置。

sim2sim 是无实物条件下的工程验证策略，不是“真实机器人替代品”。

## 5. disturbance / robustness 测试

A16 规划测试：

- 目标扰动：target pose 改变、多目标点、阶跃目标。
- 初始位姿扰动：q 初始值偏移、keyframe 之外的初始配置。
- 模型参数扰动：mass scale、damping scale、friction scale、actuator gain scale。
- 传感/控制噪声：q noise、qvel noise、target noise。
- 控制延迟：one-step delay、multi-step delay。
- 限制条件：joint velocity limit、joint position limit、control saturation。

核心指标包括 final pose error、max tracking error、mean tracking error、max joint velocity、max control effort、constraint violation count 和 success rate。

## 6. 最终 demo 和 benchmark 输出

最终展示物是一组可展示结果，不是单个报告。

主 demo：

- `outputs/videos/A07_ur5e_actuator_tracking_demo.mp4`

扩展 demo：

- `outputs/videos/A04_dls_ik_target_tracking_demo.mp4`
- `outputs/videos/A05_qp_ik_joint_limit_demo.mp4`
- `outputs/videos/A16_disturbance_tracking_demo.mp4`
- `outputs/videos/A17_sim2sim_validation_demo.mp4`

最终 benchmark：

- `outputs/reports/A18_final_benchmark_report.md`

README 展示建议：大 MP4 放 GitHub Release，小 GIF 或截图放 `docs/assets/`，README 展示总流程图、误差曲线和视频截图。

## 7. 第一阶段 / 第二阶段边界

第一阶段必须完成：

- A00 reference and assets
- A01 model inspect
- A02 configuration / site pose
- A03 site Jacobian check
- A04 DLS differential IK
- A05 task + limit + QP-IK
- A06 target / mocap-style tracking
- A07 MuJoCo actuator tracking
- A09 comparison report
- A10 demo showcase / video recording

第二阶段扩展：

- A08 collision avoidance TODO / later implementation
- A11 trajectory generation
- A12 joint-space PD / PID tracking
- A13 gravity compensation / inverse dynamics TODO
- A14 operational-space control TODO
- A15 impedance / admittance control TODO
- A16 disturbance and robustness tests
- A17 sim2sim validation
- A18 final benchmark report

## 8. 修改文件清单

- `projects/A_self_baseline/docs/A_simulation_only_full_motion_control_plan.md`
- `projects/A_self_baseline/README.md`
- `projects/A_self_baseline/docs/A_pipeline_contract.md`
- `projects/A_self_baseline/docs/A_mink_alignment_plan.md`
- `projects/A_self_baseline/docs/mink_capability_vs_A_requirements.md`
- `docs/00_project_management/step11P_simulation_first_full_motion_control_plan.md`

## 9. 未实现算法说明

本步骤只创建/修改 Markdown 规划文档，未实现：

- FK。
- Jacobian。
- finite difference。
- IK。
- QP。
- WBC。
- MuJoCo 控制。
- collision avoidance。
- video recording。
- sim2sim 运行逻辑。

## 10. 下一步仍是 A03

尽管本文档规划了 A11-A18，当前下一步仍是：

```text
A03 site Jacobian check
  -> A04 DLS differential IK
  -> A05 task + limit + QP-IK
  -> A06 target / mocap-style tracking
  -> A07 MuJoCo actuator tracking
  -> A10 demo showcase
  -> 再扩展 A11-A18
```

## 11. 验收清单

- [x] 创建 simulation-first 全流程规划文档。
- [x] 明确没有实物机械臂时的验证策略。
- [x] 扩展 A00-A18。
- [x] 加入 sim2sim validation。
- [x] 加入 robustness tests。
- [x] 明确最终 demo / benchmark 输出。
- [x] 未实现算法。
- [x] 未修改 Python 脚本。
- [x] 下一步仍保持 A03。
