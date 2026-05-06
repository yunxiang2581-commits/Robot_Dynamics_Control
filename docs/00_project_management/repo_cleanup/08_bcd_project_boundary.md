# Project B/C/D 边界说明

## 总原则

Project B/C/D 的共同目标是把复杂开源运动控制项目抽象为 simulation-only 的可运行仿真复现项目。

共同边界：

- 不做实物部署。
- 不做 sim2real 实机测试。
- 不接电机 SDK。
- 不接 CAN / EtherCAT / 串口通信。
- 不做固件。
- 不做真实机器人安全测试。
- 不做真实传感器标定。
- 不实现硬件接口。
- 外部源码仓库只读参考，不合并进主项目。

## Project B

主目录：

```text
projects/B_mujoco_mpc_study/
```

对应外部仓库：

```text
external/open_source_repos/mujoco_mpc/
```

目标：

- MuJoCo MPC runnable simulator。
- video demo。
- metrics。
- README 运行说明。

优先 demo：

```text
B01_single_joint_mpc_demo
B02_two_link_mpc_tracking_demo
B03_rollout_predictive_sampling_demo
```

边界：

- 不直接复现完整 MJPC GUI。
- 不直接合并 MJPC C++ 源码。
- 优先学习 task、residual、rollout、planner 的组织思想。

## Project C

主目录：

```text
projects/C_openloong_dyn_control_study/
```

对应外部仓库：

```text
external/open_source_repos/OpenLoong-Dyn-Control/
```

目标：

- OpenLoong-inspired humanoid MPC/WBC simulation-only demo。
- 简化双足 / 人形控制数据流。
- video demo。
- metrics。

优先 demo：

```text
C01_contact_force_allocation_demo
C02_simplified_wbc_qp_balance_demo
C03_mpc_wbc_pipeline_visual_demo
```

边界：

- 不运行原项目 demo。
- 不做实物样机部署。
- 不直接复现完整 OpenLoong 工程。
- 优先抽象 DataBus、GaitScheduler、FootPlacement、MPC、WBC_QP、PVT 的最小学习模块。

## Project D

主目录：

```text
projects/D_legged_control_study/
```

对应外部仓库：

```text
external/open_source_repos/legged_control/
```

目标：

- legged_control-inspired quadruped NMPC/WBC simulation-only demo。
- 四足 contact QP。
- trot gait schedule visual demo。
- state estimation tracking demo。

优先 demo：

```text
D01_quadruped_contact_qp_demo
D02_trot_gait_schedule_visual_demo
D03_state_estimation_tracking_demo
```

边界：

- 不运行 `legged_hw`。
- 不做 Unitree 实机接口。
- 不做 ros-control hardware interface。
- 不编译 OCS2。
- 不运行 Gazebo demo。
- `external/open_source_repos/legged_control/` 只作为源码阅读参考。
