# Project B/C/D 优先级

## 1. Project B：MuJoCo MPC

优先级最高。

原因：

- 最贴近当前 MuJoCo baseline。
- 可以直接承接 A 的 task-space tracking。
- 先学 MPC 的 task、cost、rollout、planner。
- 最适合作为第一个 simulation-only runnable simulator。

当前建议：

- 先读 `projects/B_mujoco_mpc_study/docs/01_algorithm_map.md`。
- 再读 `projects/B_mujoco_mpc_study/docs/03_simulation_only_reproduction_plan.md`。
- 后续先规划 `B01_single_joint_mpc_demo`，导出视频和 tracking error。

## 2. Project C：OpenLoong-Dyn-Control

优先级第二。

原因：

- 最贴近人形机器人求职方向。
- 能学习 MPC + WBC + PVT + 步态调度。
- 可以形成简化双足 WBC-QP 仿真器和可视化视频。

当前建议：

- 先画 MPC-WBC-PVT 数据流。
- 再阅读 walking demo 的模块关系。
- 后续优先做 `C01_contact_force_allocation_demo` 和 `C02_simplified_wbc_qp_balance_demo`。

## 3. Project D：legged_control

优先级第三。

原因：

- 控制栈最完整，覆盖四足 NMPC/WBC/状态估计。
- ROS/OCS2 依赖重。
- README 明确项目已经不再维护，不作为长期工程依赖。
- 仍然适合抽象成 simulation-only 四足 contact QP、trot gait 和状态估计 demo。

当前建议：

- 先读 NMPC 问题定义。
- 整理 WBC 决策变量 `qddot`、contact force、tau。
- 后续先做 `D01_quadruped_contact_qp_demo`，不要安装 ROS/OCS2。
