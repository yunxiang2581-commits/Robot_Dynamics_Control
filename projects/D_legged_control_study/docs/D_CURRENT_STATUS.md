# Project D 当前状态

Project D 的主目录是：

```text
projects/D_legged_control_study/
```

外部源码参考路径是：

```text
external/open_source_repos/legged_control/
```

## 1. 项目定位

D 是当前第三优先级项目，用于在 B/C 的 OpenLoong 人形主线之外，补充腿式机器人通用 NMPC-WBC-contact-state-estimation 知识。

目标是从 legged_control 学习四足机器人 NMPC + WBC + state estimation 控制栈，并抽象为 simulation-only 的简化四足仿真 demo。

当前 BCD 路线为：

```text
B: OpenLoong-oriented humanoid MPC prototype
C: OpenLoong MPC-WBC-PVT full control-chain study
D: legged NMPC-WBC-contact-estimation generalization
```

D 不抢 B/C 的 OpenLoong 人形主线，而是补充 contact schedule、friction cone、contact force QP、NMPC-WBC interface 和 state estimation 等通用腿式控制概念。

## 2. 当前状态

- 项目骨架已完成。
- `docs/`、`notes/`、`simulator/`、`outputs/` 已规划。
- 外部参考仓库已经 clone 到本地并通过 `.gitignore` 隔离。
- 尚未实现 `D01_quadruped_contact_qp_demo`。
- 尚未运行仿真。
- 尚未导出视频 demo。
- 尚未生成 metrics。

## 3. 下一步

D 的下一步不是直接运行完整 ROS / OCS2 工程，而是：

1. 阅读 legged_control 的 NMPC / WBC / state-estimation 结构。
2. 抽象接触计划、摩擦锥约束、WBC-QP 和状态估计模块。
3. 整理 `D01_quadruped_contact_qp_demo` 的数学模型。
4. 后续实现简化四足 contact force QP。

## 4. 边界

- 不运行 `legged_hw`。
- 不做 Unitree 实机接口。
- 不做 `ros-control` hardware interface。
- 不做电机 SDK、CAN、EtherCAT、串口或固件。
- 不修改 `external/open_source_repos/legged_control/`。
- 只做仿真或离线算法复现。
