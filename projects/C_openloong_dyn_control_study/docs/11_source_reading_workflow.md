# 11 OpenLoong 源码阅读流程

本文档记录 Project C 阅读 OpenLoong-Dyn-Control 源码的推荐流程。

阅读目标不是一次性看懂所有公式，而是先建立清晰的数据流地图：

```text
状态从哪里来
-> DataBus 如何组织 q / dq
-> Pin_KinDyn 写回哪些运动学 / 动力学量
-> GaitScheduler / FootPlacement 给 WBC 准备什么
-> WBC_priority 计算什么
-> PVT_Ctr 最后输出什么
-> DataLogger 记录什么
```

第一轮源码阅读只建立主线，不修改源码，不直接读 `third_party/`，不一开始深挖完整 QP。

## 1. 源码路径

优先阅读已经构建并通过 `wbc_speed_test` runtime smoke 的 R2 worktree：

```text
projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827/worktree/OpenLoong-Dyn-Control/
```

备用只读参考源码：

```text
external/open_source_repos/OpenLoong-Dyn-Control/
```

优先使用 R2 worktree 的原因：

- 它和 C03 / C04 / C05A 的构建、运行和源码追踪证据对应。
- 它已经确认能生成 `wbc_speed_test` 可执行文件。
- 它已经确认 `wbc_speed_test` runtime PASS。

注意：R2 worktree 和 external 官方源码都只作为阅读对象，不在其中改源码。

## 2. 总体阅读顺序

推荐主线：

```text
入口 demo
-> DataBus
-> Pin_KinDyn
-> GaitScheduler
-> FootPlacement
-> WBC_priority
-> PriorityTasks
-> PVT_Ctr
-> DataLogger
-> walk_wbc + MuJoCo
-> MPC
```

对应学习主线：

```text
URDF -> FK -> Jacobian -> dynamics -> WBC -> PVT -> MuJoCo closed loop -> MPC
```

第一轮先读 `wbc_speed_test`，因为它已经跑通，而且没有 MuJoCo viewer 干扰。

第二轮再读 `walk_wbc`，进入真正的 MuJoCo 闭环。

第三轮再读 `walk_mpc_wbc` 和 MPC。

## 3. 阶段 0：先看已有追踪报告

目的：先看地图，再进源码。

先读：

```text
projects/C_openloong_dyn_control_study/outputs/source_trace/C05A_wbc_speed_test_trace_R1/20260530_213038/reports/C05A_summary.md
```

再读：

```text
projects/C_openloong_dyn_control_study/outputs/source_trace/C05A_wbc_speed_test_trace_R1/20260530_213038/tables/wbc_speed_test_call_sequence.csv
```

先记住 `wbc_speed_test` 的主链路：

```text
fixed state
-> DataBus.updateQ
-> Pin_KinDyn
-> JoyStickInterpreter
-> GaitScheduler
-> FootPlacement
-> WBC_priority
-> PVT_Ctr
-> DataLogger
```

完成标准：

```text
能说明 wbc_speed_test 为什么不是完整 MuJoCo 走路 demo。
```

## 4. 阶段 1：入口 demo

文件：

```text
demo/walk_wbc_speed_test.cpp
```

阅读范围：

```text
21-31    main 创建模块
33-72    初始姿态 / IK
74-85    logger 字段注册
89-100   固定 motor_pos / motor_vel / LoopNum
101-130  写固定状态 + Pin_KinDyn
135-154  joystick + gait + foot placement
156-184  WBC 输入和 WBC 计算
193-201  PVT
203-232  DataLogger + stdout
```

第一遍读这个文件时，只跳 `.h` 文件，不跳 `.cpp` 实现。

遇到这些类时，只看接口，然后回到入口：

```text
Pin_KinDyn
DataBus
WBC_priority
GaitScheduler
FootPlacement
PVT_Ctr
DataLogger
```

完成标准：

```text
能手写出 main loop 的 10 个步骤。
```

推荐笔记：

```text
文件：walk_wbc_speed_test.cpp

一句话作用：
固定一组机器人状态，循环 10000 次跑 Pin_KinDyn + Gait + FootPlacement + WBC + PVT，并记录耗时。

输入：
- AzureLoong.urdf
- joint_ctrl_config.json
- 固定 motor_pos / motor_vel
- 固定 rpy / basePos / baseVel / foot force

输出：
- record/datalog.log
- stdout Execution time
- RobotState 中的 WBC / PVT 中间结果

主循环：
1. 写固定状态到 RobotState
2. RobotState.updateQ()
3. Pin_KinDyn 计算 Jacobian / dynamics
4. JoyStickInterpreter 写期望 base command
5. GaitScheduler 写 gait state
6. FootPlacement 写 swing foot target
7. WBC_priority 计算 ddq / tau
8. PVT_Ctr 计算 motor torque
9. DataLogger 写一行
10. printf 打印耗时

暂时不懂：
-
```

## 5. 阶段 2：DataBus

文件：

```text
common/data_bus.h
```

阅读范围：

```text
23-35    当前传感器 / 电机状态
37-41    PVT 控制字段
43-70    q / dq / Jacobian / dynamics
87-124   MPC / WBC 字段
126-155  gait / foot placement 字段
162-199  构造函数
201-236  updateQ()
```

重点理解：

```text
q  = [base position, base quaternion, joint positions]
dq = [base linear velocity, base angular velocity, joint velocities]
```

具体结构：

```text
q[0:3]   base position
q[3:7]   base quaternion
q[7:]    joint positions

dq[0:3]  base linear velocity
dq[3:6]  base angular velocity
dq[6:]   joint velocities
```

因此：

```text
floating base 下 nq != nv
motor count = model_nv - 6
```

完成标准：

```text
能解释 RobotState.updateQ() 怎样从 rpy / basePos / motors_pos_cur 组装 q / dq。
```

## 6. 阶段 3：Pin_KinDyn

文件：

```text
algorithm/pino_kin_dyn.h
algorithm/pino_kin_dyn.cpp
```

阅读顺序：

```text
1. pino_kin_dyn.h: public 字段和函数
2. pino_kin_dyn.cpp:12-77    构造函数
3. pino_kin_dyn.cpp:79-91    dataBusRead
4. pino_kin_dyn.cpp:93-143   dataBusWrite
5. pino_kin_dyn.cpp:146-229  computeJ_dJ
6. pino_kin_dyn.cpp:277-329  computeDyn
```

重点 API：

```cpp
pinocchio::JointModelFreeFlyer
pinocchio::urdf::buildModel
pinocchio::forwardKinematics
pinocchio::jacobianCenterOfMass
pinocchio::computeJointJacobiansTimeVariation
pinocchio::getJointJacobian
pinocchio::crba
pinocchio::computeMinverse
pinocchio::computeCoriolisMatrix
pinocchio::computeGeneralizedGravity
```

读成这个流程：

```text
DataBus.q / dq
-> Pinocchio model / data
-> FK
-> Jacobian
-> M / Minv / C / G / nonlinear term
-> 写回 DataBus
```

完成标准：

```text
能说出 WBC 需要 Pin_KinDyn 提供哪些量。
```

## 7. 阶段 4：GaitScheduler

文件：

```text
algorithm/gait_scheduler.h
algorithm/gait_scheduler.cpp
```

阅读范围：

```text
12-24    构造函数
26-52    dataBusRead
54-77    dataBusWrite
79-211   step
```

第一遍只抓这些变量：

```text
motionState
legState
legStateNext
phi
tSwing
swingStartPos_W
stanceDesPos_W
```

完成标准：

```text
能解释 legState / motionState / phi 这三个变量分别代表什么。
```

## 8. 阶段 5：FootPlacement

文件：

```text
algorithm/foot_placement.h
algorithm/foot_placement.cpp
```

阅读范围：

```text
11-27    dataBusRead
29-35    dataBusWrite
36-121   getSwingPos
123-154  Trajectory
```

输入：

```text
swingStartPos_W
js_vel_des
dq
phi
posHip_W
posST_W
base_pos
tSwing
theta0
rpy
base_omega_W
width_hips
legState
```

输出：

```text
swing_fe_pos_des_W
swing_fe_rpy_des_W
swingDesPosCur_W
swingDesPosFinal_W
```

完成标准：

```text
能解释 FootPlacement 为什么需要 gait phase phi。
```

## 9. 阶段 6：WBC_priority 外壳

文件：

```text
algorithm/wbc_priority.h
algorithm/wbc_priority.cpp
```

第一轮阅读范围：

```text
13-97    构造函数，建立任务列表
99-178   dataBusRead
180-196  dataBusWrite
397-729  computeDdq
198-395  computeTau
```

第一轮只理解：

```text
computeDdq = 任务优先级，算 delta_q / dq / ddq
computeTau = QP，算 contact force correction 和 joint torque
```

WBC 输入：

```text
q / dq
Jacobian
M / Minv / nonlinear term
base desired pose
swing foot desired pose
contact state
Fr_ff
```

WBC 输出：

```text
wbc_delta_q_final
wbc_dq_final
wbc_ddq_final
wbc_tauJointRes
wbc_FrRes
qp_status
```

完成标准：

```text
能说出 computeDdq 和 computeTau 的区别。
```

## 10. 阶段 7：PriorityTasks

文件：

```text
algorithm/priority_tasks.h
algorithm/priority_tasks.cpp
```

阅读范围：

```text
10-14    addTask
35-55    buildPriority
66-108   computeAll
```

重点理解：

```text
高优先级任务先满足，低优先级任务只能在高优先级任务的 null-space 里调整。
```

先记住：

```text
Jpre = J * N
N = null-space projector
```

完成标准：

```text
能解释为什么 WBC 需要 task priority。
```

## 11. 阶段 8：PVT_Ctr

文件：

```text
common/PVT_ctrl.h
common/PVT_ctrl.cpp
```

阅读范围：

```text
11-47    构造函数，读 joint_ctrl_config.json
49-58    dataBusRead
60-63    dataBusWrite
65-76    setJointPD
79-91    calMotorsPVT
```

核心公式：

```text
tau = Kp(pos_des - pos_cur)
    + Kd(vel_des - vel_cur)
    + tau_ff
```

然后：

```text
低通滤波
力矩限幅
gear ratio
写回 DataBus
```

完成标准：

```text
能解释 WBC 输出和 PVT 输出的区别。
```

## 12. 阶段 9：DataLogger

文件：

```text
common/data_logger.h
common/data_logger.cpp
```

阅读范围：

```text
10-20    打开 datalog.log
22-33    addIterm
35-51    finishItermAdding
53-56    startNewLine
58-112   recItermData
114-122  finishLine
```

完成标准：

```text
能解释 datalog 的列为什么和 addIterm 顺序一致。
```

## 13. 阶段 10：真正 MuJoCo 闭环 walk_wbc

等上面读完，再读：

```text
demo/walk_wbc.cpp
sim_interface/MJ_interface.cpp
sim_interface/GLFW_callbacks.cpp
```

比较：

```text
wbc_speed_test:
固定状态 -> WBC / PVT -> log

walk_wbc:
MuJoCo sensor -> DataBus -> WBC / PVT -> torque -> MuJoCo step -> viewer
```

完成标准：

```text
能画出 MuJoCo 闭环数据流。
```

## 14. 阶段 11：MPC

最后再读：

```text
algorithm/mpc.h
algorithm/mpc.cpp
demo/walk_mpc_wbc.cpp
```

不要太早读 MPC，因为它依赖：

```text
当前状态
期望轨迹
contact schedule
gait phase
WBC 接口
```

完成标准：

```text
能解释 MPC 输出如何影响 WBC 的 Fr_ff 或 desired trajectory。
```

## 15. 每日阅读节奏建议

```text
Day 1:
walk_wbc_speed_test.cpp
data_bus.h

Day 2:
pino_kin_dyn.h
pino_kin_dyn.cpp

Day 3:
gait_scheduler.h/cpp
foot_placement.h/cpp

Day 4:
wbc_priority.h
wbc_priority.cpp 第一轮，只读外壳

Day 5:
priority_tasks.h/cpp
wbc_priority.cpp 第二轮，读任务优先级

Day 6:
PVT_ctrl.h/cpp
data_logger.h/cpp

Day 7:
walk_wbc.cpp
MJ_interface.cpp

Day 8:
mpc.h/cpp
walk_mpc_wbc.cpp
```

## 16. 每个文件统一笔记模板

每读一个文件，都按这个模板记录：

```text
文件：

一句话作用：

在主链路中的位置：

输入：
-

输出：
-

关键类 / 函数：
-

DataBus 读：
-

DataBus 写：
-

我理解了：
-

还不懂：
-

下一步要跳到：
-
```

示例：

```text
文件：algorithm/pino_kin_dyn.cpp

一句话作用：
从 DataBus 读 q / dq，用 Pinocchio 计算 FK、Jacobian、动力学项，再写回 DataBus。

输入：
- RobotState.q
- RobotState.dq
- RobotState.ddq

输出：
- J_l / J_r / J_base
- dyn_M / dyn_C / dyn_G / dyn_Non
- pCoM_W
- foot pose
```

## 17. 第一轮不要做的事

第一轮先不要：

```text
1. 不要读 third_party 源码。
2. 不要从 wbc_priority.cpp 第一行硬读到最后。
3. 不要一开始推完整 QP。
4. 不要把 walk_wbc 和 walk_mpc_wbc 混在一起。
5. 不要边读边改源码。
6. 不要急着运行 GUI。
```

第一轮只建立数据流地图。

## 18. 第一轮最小完成目标

读完第一轮后，需要能讲清楚：

```text
机器人状态从哪里来？
DataBus 怎么组织 q / dq？
Pin_KinDyn 写回哪些运动学 / 动力学量？
GaitScheduler / FootPlacement 给 WBC 准备什么？
WBC_priority 算什么？
PVT_Ctr 最后输出什么？
DataLogger 记录什么？
```

如果这些问题能讲清楚，第一阶段源码阅读就完成了。
