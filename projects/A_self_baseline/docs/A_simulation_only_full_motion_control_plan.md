# A Simulation-First Full Motion Control Plan

## 1. 为什么本项目采用 simulation-first

当前项目没有实物 UR5e 或其他机械臂。A 项目的目标不是伪装成已经完成真实机器人验证，而是在无实物条件下构建一条可解释、可复现、可评估的仿真优先运动控制 baseline。

simulation-first 的价值在于：

- 能完整验证模型、运动学、Jacobian、IK、QP-IK、控制器和视频 demo 的工程链路。
- 能保存日志、误差曲线、轨迹文件、配置快照和报告，便于复盘。
- 能反复测试不同目标、不同扰动、不同控制参数和不同模型参数。
- 适合求职展示：面试官可以看到清晰的输入、输出、指标、失败边界和改进路径。
- 适合后续迁移到真实机器人：先把模型、任务、轨迹、控制接口和评估指标拆清楚，再讨论硬件部署。

本项目第一阶段不做真实硬件部署。所有“验证”都限定在仿真和 sim2sim 范围内。

## 2. 没有实物机械臂时如何定义有效验证

有效验证不等于“上真机”。在没有实物机械臂的条件下，有效验证应看是否满足以下标准。

### 2.1 可运行

- 脚本能从配置读取模型并运行。
- 输出稳定文件，例如 JSON cache、Markdown report、CSV log、figure、video。
- 同一命令在相同配置下能重复得到结构一致的结果。

### 2.2 可解释

- 每一步的输入、输出、公式和物理意义清楚。
- 能说明 `q`、site pose、Jacobian、`dq`、`q_des`、tracking error 的含义。
- 能说明当前步骤为什么存在，以及它如何连接到下一步。

### 2.3 可量化

- 有 tracking error。
- 有 final pose error。
- 有 joint velocity / joint limit 检查。
- 有 constraint violation 指标。

### 2.4 可复现

- 固定配置。
- 固定目标。
- 固定输出目录。
- 结果可重复生成。

### 2.5 可迁移

- 同一条轨迹或同一控制目标能在不同仿真设置下验证。
- 能进行 sim2sim 测试，观察控制链路对模型参数变化的敏感性。

### 2.6 可展示

- 有 MuJoCo 视频 demo。
- 有误差曲线。
- 有 benchmark report。
- README 中能说明 demo 做了什么、输入是什么、输出是什么、指标如何解释。

## 3. A00-A18 全流程规划

A00-A10 是第一阶段主线。A11-A18 是第二阶段扩展。不要把 A11-A18 插入当前 A03-A07 的实现顺序中；当前仍应先完成 A03-A07，再做 A10，然后扩展 A11-A18。

| 步骤 | 名称 | 阶段 | 目标 | 输入 | 输出 | 验收标准 | 当前状态 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| A00 | reference and assets | 第一阶段 | 固定 mink UR5e 参考范围和本地资产边界 | `external/mink_upstream`、copied UR5e assets | asset audit、path summary | 路径、来源、许可证、最小复制边界清楚 | 已规划/已完成 |
| A01 | model inspect | 第一阶段 | 检查 MuJoCo 模型维度和对象名称 | `robot.yaml`、`scene.xml` | A01 report、model summary JSON | `nq=6`、`nv=6`、`nu=6`，关键 site/body/actuator/keyframe 命中 | 最小实现已完成 |
| A02 | configuration / site pose | 第一阶段 | 学习 `q -> data -> site/body pose` | A01 summary、`keyframe:home`、target site/body | A02 pose JSON、report | 能查询 `attachment_site` 和 `wrist_3_link` pose | 最小实现已完成 |
| A03 | site Jacobian check | 第一阶段 | 验证 `site velocity = J(q) dq` | A02 pose、target site、`q`、`dq`、`dt` | Jacobian check JSON、report、error figure | `J dq` 与 finite difference velocity 对齐 | 最小实现已完成 |
| A04 | DLS differential IK | 第一阶段 | 实现最小无约束 differential IK | target pose、current pose、J、damping、gain | q trajectory、error log、report | 误差下降，轨迹无 NaN | 最小实现已完成 |
| A05 | task + limit + QP-IK | 第一阶段 | 加入 task、limit 和最小 QP-IK | task、limit、QP weights | QP-IK trajectory、constraint log | 满足速度/位置限制，误差可解释 | 待做 |
| A06 | target / mocap-style tracking | 第一阶段 | 管理 fixed target 和后续 mocap-style target | fixed target、site pose、IK output | target tracking log、report | target、site、误差关系清楚 | 待做 |
| A07 | MuJoCo actuator tracking | 第一阶段 | 把 A04/A05 轨迹送入 MuJoCo actuator | `q_des`/`dq_des`、actuator names | tracking log、error figure、video | actuator 维度正确，tracking error 有界 | 待做 |
| A08 | collision avoidance TODO | 第二阶段 | 记录避障约束扩展方式 | A05 QP-IK、collision geometry | collision avoidance TODO doc | 说明为什么在 QP-IK 后接入 | TODO/later |
| A09 | comparison report | 第一阶段 | 对比自己实现和 mink 抽象 | A01-A08 reports/logs | comparison report | 能说明实现差距和后续补齐顺序 | 待做 |
| A10 | demo showcase / video recording | 第一阶段 | 整理主 demo 和展示材料 | A07 video/log/figure、A09 report | demo showcase doc、video path summary | README/Release 可展示 | 待做 |
| A11 | trajectory generation | 第二阶段 | 生成 joint-space / task-space 轨迹 | waypoints、duration、limits | trajectory file、plot、report | 轨迹连续，速度/加速度有界 | 后续扩展 |
| A12 | joint-space PD / PID tracking | 第二阶段 | 学习关节空间控制器和误差收敛 | q trajectory、MuJoCo model、gains | tracking CSV、error figure、report | tracking error 有界，控制量可解释 | 后续扩展 |
| A13 | gravity compensation / inverse dynamics TODO | 第二阶段 | 规划重力补偿和逆动力学学习路径 | model、q、qvel、qacc target | dynamics TODO report | 清楚区分 kinematics、control、dynamics | TODO |
| A14 | operational-space control TODO | 第二阶段 | 规划任务空间控制和 operational-space error | site pose、Jacobian、mass matrix TODO | OSC TODO report | 输入输出和数学对象清楚 | TODO |
| A15 | impedance / admittance control TODO | 第二阶段 | 规划柔顺控制学习路径 | target pose、measured pose、stiffness/damping | impedance TODO report | 能说明虚拟弹簧阻尼含义 | TODO |
| A16 | disturbance and robustness tests | 第二阶段 | 验证目标、初值、参数、噪声、延迟扰动 | controller、trajectory、disturbance configs | robustness report、CSV、figure、video | success rate 和误差指标可量化 | 后续扩展 |
| A17 | sim2sim validation | 第二阶段 | 在不同仿真设置中验证同一控制链路 | nominal/perturbed model、same controller | sim2sim report、figure、CSV、video | tracking error、pose error、control effort 可比较 | 后续扩展 |
| A18 | final benchmark report | 第二阶段 | 汇总 A00-A18 的求职展示结果 | all reports/logs/figures/videos | final benchmark report | 能完整说明能力边界、结果和后续真机计划 | 后续扩展 |

## 4. sim2sim 验证方案

sim2sim 的定义是：同一控制任务、同一轨迹或同一目标，在不同仿真模型、不同参数设置或不同仿真后端中验证控制链路是否仍然稳定。

sim2sim 不是“真实机器人替代品”。它是在无实物条件下的工程验证策略，用来评估控制链路对模型参数、仿真后端和 actuator 配置变化的敏感性。

### 4.1 MuJoCo-to-MuJoCo 参数扰动版

第一阶段推荐先做 MuJoCo-to-MuJoCo 参数扰动版：

- nominal UR5e。
- perturbed UR5e。
- 修改质量、阻尼、摩擦、关节 damping、actuator gain 等参数。
- 比较 tracking error、final pose error、control effort。

输出建议：

- `outputs/reports/A17_sim2sim_mujoco_param_report.md`
- `outputs/figures/A17_sim2sim_error_compare.png`
- `outputs/logs/A17_sim2sim_tracking_compare.csv`
- `outputs/videos/A17_sim2sim_validation_demo.mp4`

### 4.2 MuJoCo-to-Other Simulator

第二阶段可选：

- PyBullet。
- Gazebo。
- Isaac Sim。
- 或另一个 MuJoCo 模型版本。

第一版不强制做。只有在 A03-A07 主线和 A10 demo 已跑通后，再考虑跨后端 sim2sim。

### 4.3 Same Controller, Different Model Variant

第三层验证使用：

- 同一个 controller。
- 不同 MJCF。
- 不同 actuator 配置。

比较重点是控制效果、稳定性、tracking error、final pose error、control effort 和 constraint violation。

## 5. disturbance / robustness 测试方案

A16 disturbance and robustness tests 应覆盖以下扰动。

### 5.1 目标扰动

- target pose 改变。
- 多目标点。
- 阶跃目标。

### 5.2 初始位姿扰动

- q 初始值偏移。
- keyframe 之外的初始配置。

### 5.3 模型参数扰动

- mass scale。
- damping scale。
- friction scale。
- actuator gain scale。

### 5.4 传感/控制噪声

- q noise。
- qvel noise。
- target noise。

### 5.5 控制延迟

- one-step delay。
- multi-step delay。

### 5.6 限制条件

- joint velocity limit。
- joint position limit。
- control saturation。

A16 输出：

- `outputs/reports/A16_robustness_report.md`
- `outputs/logs/A16_robustness_trials.csv`
- `outputs/figures/A16_robustness_error_compare.png`
- `outputs/videos/A16_disturbance_tracking_demo.mp4`

A16 指标：

- final pose error。
- max tracking error。
- mean tracking error。
- max joint velocity。
- max control effort。
- constraint violation count。
- success rate。

## 6. 最终 demo 和 benchmark 输出

最终展示物不是单个报告，而是一组可展示结果。

### 主 demo

```text
outputs/videos/A07_ur5e_actuator_tracking_demo.mp4
```

主 demo 展示 UR5e 在 MuJoCo 中跟踪 A04/A05 生成的轨迹，重点展示 actuator tracking，并配套误差曲线和日志。

### 扩展 demo

- `outputs/videos/A04_dls_ik_target_tracking_demo.mp4`
- `outputs/videos/A05_qp_ik_joint_limit_demo.mp4`
- `outputs/videos/A16_disturbance_tracking_demo.mp4`
- `outputs/videos/A17_sim2sim_validation_demo.mp4`

### Benchmark 报告

```text
outputs/reports/A18_final_benchmark_report.md
```

A18 benchmark report 应包含：

- A01-A10 主线完成情况。
- A11-A18 扩展完成情况。
- 每个 demo 的输入、输出、误差指标。
- sim2sim 结果。
- 与 mink 的对比。
- 当前局限。
- 后续真机迁移计划。

### README 展示建议

- 大视频不直接提交 Git。
- 小 GIF 或截图可放 `docs/assets/`。
- 大 MP4 可放 GitHub Release。
- README 展示一张总流程图、一张误差曲线、一张视频截图。

## 7. 哪些是第一阶段，哪些是第二阶段

### 第一阶段：必须完成，形成求职展示 baseline

第一阶段包含：

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

第一阶段目标是跑通完整 motion-control demo。重点是可运行、可解释、可展示，不追求高级力控或 sim2sim 完整系统。

### 第二阶段：扩展完整运动控制能力

第二阶段包含：

- A08 collision avoidance TODO / later implementation
- A11 trajectory generation
- A12 joint-space PD / PID tracking
- A13 gravity compensation / inverse dynamics TODO
- A14 operational-space control TODO
- A15 impedance / admittance control TODO
- A16 disturbance and robustness tests
- A17 sim2sim validation
- A18 final benchmark report

第二阶段目标是增强工程可信度。重点是鲁棒性、仿真迁移和 benchmark。在没有实物机械臂时，A16/A17 是替代真机验证的重要部分，但它们仍然不是“真实机器人验证”的替代说法。

## 8. 当前下一步

尽管本文档规划了 A11-A18，但当前实现顺序不变。A04 最小 DLS differential IK 已完成，当前下一步进入 A05：

```text
A05 task + limit + QP-IK
  -> A06 target / mocap-style tracking
  -> A07 MuJoCo actuator tracking
  -> A10 demo showcase
  -> 再扩展 A11-A18
```

不要因为加入 sim2sim 规划而跳过 A04-A07。

A04 当前状态：最小 position-mode DLS differential IK 已完成，依赖 A03 已验证的 Jacobian。脚本语法检查通过，并已生成 `outputs/trajectories/A04_dls_ik_q_traj.npy`、`outputs/logs/A04_dls_ik_error.csv`、`outputs/figures/A04_dls_ik_error.png` 和 `outputs/reports/A04_dls_ik_report.md`。当前 report 显示 `converged=True`、`stop_reason=tolerance_reached`，初始位置误差约为 `0.03`，最终位置误差约为 `8.45e-4`；q trajectory shape 为 `(17, 6)` 且无 NaN。

Step 12A-R 追加了 pose-aware DLS IK 规划，仍不改变当前实现顺序。A04 在保留现有代码的基础上规划 `position` / `pose_6d` 两种任务目标，并在 Markdown 中补充 SO(3) rotation error、6D pose error、`J_task` 和 pose-aware DLS 公式。Step 12C 再扩展 pose_6d mode；A05/A06/A07 后续需要同步 target pose、orientation task 和 trajectory_source。
