# Project B 总览

Project B 是 MuJoCo MPC / MJPC 学习线，目标是在 simulation-only 条件下建立一条从小模型 MPC 到后续人形控制原型的理解链：

```text
B01-B03: small-model MPC concept validation
B04: underactuated bridge task
B05-B07: OpenLoong humanoid MPC line
```

当前 Project B 不做实物部署，不接电机 SDK，不做 CAN / EtherCAT / 串口通信，不修改真实机器人安全相关权限。

## 1. 学习目标

Project B 的核心不是复刻一个庞大的 MPC 框架，而是逐步理解这些对象：

- state / control
- rollout
- residual / cost
- horizon
- receding horizon control
- predictive sampling
- CEM / MPPI / iLQG / SQP / direct multiple shooting NMPC
- video / figures / metrics / logs 可复盘输出

每个 demo 都必须满足 Project B 的输出规则：

```text
video + figures + metrics + logs
```

没有可视化输出的任务不算完成；没有 metrics 的视频不算完成；没有复现实验命令的结果不算完成。

## 2. 当前状态

- `B01_single_joint_mpc_demo` 已按 smooth target tracking 标准验收通过。
- `B02_two_link_mpc_tracking_demo` 已实现为可运行的双连杆任务空间 MPC demo。
- B02 已具备 tracking CSV、figures、raw MP4、marked MP4、metrics、logs、benchmark 和 regression 两套基线。
- B03 已重新定义为 `MPC solver ladder demo`，旧的 rollout predictive sampling 文档并入 B03 的 sampling 子专题。
- B04-B07 仍是后续路线，不在当前阶段实现。

B01 参考通过 run：

```text
outputs/runs/B01_single_joint_mpc_demo/20260511_220538/
```

关键指标：

```text
final_error: 0.019126844672575194 rad
mean_tracking_error: 0.06666501475904199 rad
max_torque: 1.88158006947223 Nm
runtime_per_control_step: 0.017442599999582552 s
recorded_frames: 300
```

B02 当前正式结论：

```text
B02-current-10Nm: ACCEPTED / PASSED
```

## 3. 任务路线

### B01: single joint MPC

用单自由度 hinge joint 学习最小 MPC 闭环：

```text
q -> q_target(t)
```

重点是理解 state、torque、rollout、horizon cost 和 receding horizon control。

### B02: two-link task-space tracking

从 B01 的标量角度残差过渡到任务空间末端残差：

```text
p_ee(q) -> p_target(t)
```

重点是 target trajectory、actual end-effector trajectory、tracking error、MPC step cost 和完整输出链路。

### B03: MPC solver ladder

B03 不替代 B02，而是在简单模型上学习不同 solver 的结构和指标：

```text
Random Shooting -> Warm-start Sampling -> CEM -> MPPI -> iLQG-lite -> SQP -> Direct Multiple Shooting NMPC
```

进入 B03 前建议先运行 B02-regression-light 做健康检查。

### B04-B07: 后续路线

- B04：欠驱动非线性平衡过渡任务。
- B05-B07：OpenLoong 人形 MPC 主线，逐步连接到 Project C/D。

## 4. 与 Project A/C/D 的关系

Project A 负责建立机器人模型、路径、FK、Jacobian、IK、viewer 和 actuator 的基础接口理解。

Project B 负责 simulation-only MPC 学习线：

```text
state + future target -> rollout -> horizon cost -> MPC target
```

Project C 后续负责把 MPC target 接到更完整的控制链：

```text
MPC target -> WBC-QP -> PVT / PD -> MuJoCo closed loop
```

Project D 再进一步抽象腿式控制通用化问题。

## 5. 统一输出目录

每个任务的运行结果使用：

```text
outputs/runs/<task_name>/<run_id>/
```

典型结构：

```text
videos/
figures/
metrics/
logs/
```

## 6. 边界与风险

当前阶段不做：

- 不 clone 或修改大型外部仓库作为主实现路径。
- 不运行真实硬件。
- 不把 B03 的 solver 实验写回 B02 benchmark 配置。
- 不把可视化逻辑塞进 controller。
- 不跳过 B02 regression 直接判断 B03 正确。

MJPC / MuJoCo MPC 原项目适合用来阅读 task、residual、planner、rollout、trajectory、agent 等设计，但当前仓库优先做学习型、可解释、可复现的小模型实现。

## 7. 推荐阅读入口

- B01: [B01_SINGLE_JOINT_MPC.md](../B01_single_joint_mpc/B01_SINGLE_JOINT_MPC.md)
- B02: [B02_TWO_LINK_MPC_TRACKING.md](../B02_two_link_mpc_tracking/B02_TWO_LINK_MPC_TRACKING.md)
- B03: [B03_MPC_SOLVER_LADDER.md](../B03_mpc_solver_ladder/B03_MPC_SOLVER_LADDER.md)
