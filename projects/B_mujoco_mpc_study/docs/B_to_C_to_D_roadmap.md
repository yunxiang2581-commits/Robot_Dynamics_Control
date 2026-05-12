# B 到 C 到 D 路线图

## 1. 总体目标

BCD 不应被看作三个互不相关的资料夹，而应形成一条从 MPC 概念、人形控制链，到通用腿式控制的学习路线。

```text
B: OpenLoong-oriented humanoid MPC prototype
C: OpenLoong MPC-WBC-PVT full control-chain study
D: legged NMPC-WBC-contact-estimation generalization
```

其中 B 采用“路线 2”：

```text
先最小 demo，再升级到 OpenLoong 人形 MPC。
```

这个路线的优点是最稳：B01-B03 用小模型把 MPC 闭环拆清楚，B04 用小车倒立二阶摆学习欠驱动非线性平衡，B05-B07 才切到 OpenLoong 人形模型。

## 2. Project B：MPC 核心与 OpenLoong 人形原型

B 的任务是把 MPC 的核心概念变成可运行、可解释、可验证的 simulation-only 原型。

### B01-B03：基础概念验证

```text
B01_single_joint_mpc_demo
B02_two_link_mpc_tracking_demo
B03_rollout_predictive_sampling_demo
```

这三步只回答一个问题：

```text
MPC 闭环到底如何从 state、rollout、horizon cost、control selection 组成？
```

它们不是最终成果，而是给 OpenLoong 人形阶段降维。

### B04：欠驱动非线性平衡过渡任务

```text
B04_cart_double_inverted_pendulum_mpc
```

B04 连接 B03 和 OpenLoong 人形站立平衡：

- 小车倒立二阶摆是欠驱动系统。
- 两节摆杆的竖直向上姿态是不稳定平衡点。
- 控制输入只有小车水平力，但任务 residual 同时包含小车位置、摆杆角度、速度和控制力。
- 它比单关节、二连杆更接近人形平衡问题，但还没有 OpenLoong 的高维 floating-base 和复杂接触。

### B05-B07：OpenLoong 人形主线

```text
B05_openloong_model_mpc_setup
B06_openloong_standing_balance_mpc
B07_openloong_weight_shift_or_stepping_mpc
```

B05 先理解 OpenLoong MuJoCo 模型：

- `qpos` / `qvel`。
- actuator。
- body / site。
- pelvis / torso / feet。
- contact candidates。

B06 做第一版人形 MPC 成果：

- 站立平衡。
- pelvis 高度 residual。
- torso 姿态 residual。
- 关节姿态和速度 residual。
- torque cost。
- MP4 与 metrics。

B07 再做更接近运动控制的问题：

- 左右重心转移。
- 或最小小步踏步。
- 简化 contact schedule。
- foot target residual。

## 3. Project C：接上完整 OpenLoong 控制链

C 的任务不是重新发明 B 的 MPC，而是学习 OpenLoong-Dyn-Control 中更完整的分层控制架构：

```text
command -> gait/contact schedule -> MPC target -> WBC-QP -> PVT / PD -> MuJoCo closed loop
```

B 给 C 的输入应尽量清晰：

- desired base state。
- desired pelvis / CoM target。
- desired torso orientation。
- desired foot target。
- optional contact schedule。
- optional desired contact force。

C 重点学习：

- WBC-QP 如何接收 MPC target。
- contact force allocation 如何表达。
- torque / joint command 如何进入 MuJoCo。
- PVT / PD 在闭环中的角色。

## 4. Project D：腿式控制通用化

D 以 legged_control / OCS2 / NMPC / WBC / state estimation 为学习对象。

D 不应抢 B/C 的 OpenLoong 人形主线，而是补充通用腿式控制知识：

- contact schedule。
- friction cone。
- contact force QP。
- NMPC-WBC interface。
- state estimation。

这些知识会帮助反向理解 C 中的接触力、WBC 约束和状态估计。

## 5. 推荐实施顺序

当前推荐顺序是：

```text
1. B01 单关节 MPC TODO skeleton
2. B01 可运行仿真 + metrics
3. B02 二连杆 tracking
4. B03 predictive sampling
5. B04 小车倒立二阶摆 MPC
6. B05 OpenLoong 模型状态摘要
7. B06 OpenLoong 站立平衡 MPC
8. C01 接触力分配 / WBC-QP 数学整理
9. B07 OpenLoong 重心转移或小步踏步
10. D01 四足 contact force QP
```

这样安排的原因是：

- B01-B03 先把 MPC 的数学闭环拆小。
- B04 先引入欠驱动非线性平衡，作为人形站立前的过渡。
- B05-B07 让 B 的最终成果变成人形 MPC。
- C 接住 OpenLoong 的完整 MPC-WBC-PVT 控制链。
- D 把接触、NMPC、WBC、状态估计泛化到腿式机器人。

## 6. 边界

BCD 当前都保持 simulation-only：

- 不做实物部署。
- 不做 sim2real。
- 不接电机 SDK。
- 不接 CAN / EtherCAT / 串口。
- 不涉及固件。
- 不修改外部参考仓库。
- 不编译、不运行外部完整工程 demo。
