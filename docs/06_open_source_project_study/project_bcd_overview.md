# Project B/C/D 总览

## 总览表

| Project | 开源项目 | 机器人类型 | 核心算法 | 主要学习价值 | 当前是否建议运行 |
|---|---|---|---|---|---|
| Project B | MuJoCo MPC / MJPC | 通用机器人：humanoid、quadruped、manipulation 等 | Predictive Sampling、iLQG、Gradient Descent、multiple shooting、rollout、task residual | 从 A 的单步 QP-IK 推进到 simulation-in-the-loop MPC，并最终做 MuJoCo MPC Simulator | 当前不运行原 MJPC；后续优先做自建 simulation-only demo |
| Project C | OpenLoong-Dyn-Control | 人形 / 双足机器人 | MPC、WBC、GaitScheduler、FootPlacement、StateEstimator、PVT | 学习人形 MPC + WBC + PVT 架构，并最终做简化 Humanoid WBC Simulator | 当前不运行原项目；后续做简化双足 WBC 仿真和视频 |
| Project D | legged_control | 四足机器人 | OCS2 NMPC、WBC-QP、contact schedule、Kalman filter、torque feedforward + PD | 学习四足 NMPC/WBC/状态估计控制栈，并最终做四足简化仿真或离线算法 demo | 当前不运行原项目；项目已不再维护且 ROS/OCS2 依赖重 |

## 已从公开资料确认的信息

- Project B 的 README 确认 MJPC 是 Google DeepMind 基于 MuJoCo 的实时 predictive control 框架。
- Project C 的 README 确认 OpenLoong-Dyn-Control 是人形机器人 MPC + WBC 框架，可部署 MuJoCo。
- Project D 的 README 确认 legged_control 是基于 OCS2 和 ros-control 的 NMPC-WBC 控制栈，并明确不再维护。
- OCS2 文档确认 OCS2 是面向 switched systems optimal control 的 C++ toolbox。

## 待后续源码阅读确认的信息

- B：task residual、planner、agent 的接口细节。
- C：DataBus、MPC、WBC_QP、PVT_Ctr 的具体数据流。
- D：legged_interface、legged_wbc、legged_estimation 的实现细节。

## 仿真交付目标

| Project | Simulator | 视频 demo |
|---|---|---|
| B | MuJoCo MPC Simulator | B01 单关节 MPC、B02 二连杆 tracking、B03 predictive sampling rollout |
| C | OpenLoong-inspired Humanoid WBC Simulator | C01 双足接触力分配、C02 简化 WBC 平衡、C03 MPC-WBC 数据流动画 |
| D | legged_control-inspired Quadruped Simulator | D01 四足 contact QP、D02 trot gait schedule、D03 状态估计 tracking |

## 与当前 A 项目的关系

```text
A_self_baseline: MuJoCo / Pinocchio / FK / Jacobian / QP-IK / tracking
Project B: MPC 的 task / residual / rollout / planner + MuJoCo MPC demo
Project C: 人形 MPC + WBC + PVT 架构 + 简化双足 WBC demo
Project D: 四足 NMPC + WBC + state estimation 控制栈 + 简化四足 demo
```
