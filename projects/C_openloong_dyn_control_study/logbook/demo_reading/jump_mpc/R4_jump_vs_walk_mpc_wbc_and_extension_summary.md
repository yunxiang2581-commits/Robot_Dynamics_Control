# R4: jump_mpc 与 walk_mpc_wbc 对比和扩展启发

## 1. 本轮目标

这一轮不再继续抠 `jump_mpc.cpp` 的每一行，而是做一个横向总结：

```text
jump_mpc 和 walk_mpc_wbc 到底差在哪里？
这些差异对后续做奔跑、边走边跳、跨栏有什么启发？
```

前面已经读完：

```text
R1: jump_mpc 主流程
R2: jump_state 和阶段切换
R3: MPC -> Jacobian -> PVT -> MuJoCo 输出链
```

所以 R4 的作用是把 jump demo 收束成一张工程地图。

## 2. 两个 demo 的总链路

先把两个 demo 的链路并排放在一起。

### 2.1 walk_mpc_wbc

`walk_mpc_wbc` 的主线是：

```text
MuJoCo
-> MJ_Interface
-> Pin_KinDyn
-> JoyStickInterpreter
-> GaitScheduler
-> FootPlacement
-> MPC
-> WBC_priority
-> PVT_Ctr
-> MuJoCo ctrl
```

源码入口可以看：

[walk_mpc_wbc.cpp:40](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/walk_mpc_wbc.cpp:40)

```cpp
GaitScheduler gaitScheduler(0.25, mj_model->opt.timestep);
```

[walk_mpc_wbc.cpp:42](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/walk_mpc_wbc.cpp:42)

```cpp
FootPlacement footPlacement;
```

[walk_mpc_wbc.cpp:164](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/walk_mpc_wbc.cpp:164)

```cpp
MPC_solv.dataBusRead(RobotState);
MPC_solv.cal();
MPC_solv.dataBusWrite(RobotState);
```

[walk_mpc_wbc.cpp:172](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/walk_mpc_wbc.cpp:172)

```cpp
WBC_solv.dataBusRead(RobotState);
WBC_solv.computeDdq(kinDynSolver);
WBC_solv.computeTau();
WBC_solv.dataBusWrite(RobotState);
```

所以 walking 版本是：

```text
步态调度生成接触序列
-> 足步规划生成摆脚目标
-> MPC 规划接触力
-> WBC 统一协调全身任务
```

### 2.2 jump_mpc

`jump_mpc` 的主线更短：

```text
MuJoCo
-> MJ_Interface
-> Pin_KinDyn
-> jump_state 状态机
-> MPC
-> -J_stand^T F
-> PVT_Ctr
-> MuJoCo ctrl
```

关键入口：

[jump_mpc.cpp:84](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:84)

```cpp
uint16_t jump_state = 0;
```

[jump_mpc.cpp:295](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:295)

```cpp
mpc_force.dataBusRead(RobotState);
mpc_force.cal();
mpc_force.dataBusWrite(RobotState);
```

[jump_mpc.cpp:300](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:300)

```cpp
Uje = Jac_stand.transpose() * (-1.0)
    * RobotState.fe_react_tau_cmd.block<nu - 1, 1>(0, 0);
```

所以 jump 版本是：

```text
状态机决定当前阶段
-> 有接触时用 MPC 算接触 wrench
-> 直接用 -J^T F 映射成腿部关节力矩
-> 不经过完整 WBC
```

## 3. 状态生成方式不同

### 3.1 walk_mpc_wbc 是周期步态

walking 的核心是周期性接触：

```text
左支撑 / 右摆动
右支撑 / 左摆动
双支撑过渡
```

所以它需要：

```text
GaitScheduler: 判断当前哪条腿支撑、哪条腿摆动
FootPlacement: 根据速度目标规划下一步落脚点
```

也就是说，walking 的状态来自：

```text
连续步态相位 + 速度指令
```

### 3.2 jump_mpc 是一次性阶段状态机

jump 的核心不是左右换脚，而是：

```text
准备
-> 起跳
-> 上升
-> 下落
-> 落地恢复
```

源码里直接写在 `jump_mpc.cpp`：

[jump_mpc.cpp:166](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:166)

```cpp
if (jump_state == 0) { // Jump
```

[jump_mpc.cpp:203](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:203)

```cpp
} else if (jump_state == 3) { //up
```

[jump_mpc.cpp:232](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:232)

```cpp
} else if (jump_state == 4) { // down
```

[jump_mpc.cpp:267](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:267)

```cpp
} else if (jump_state == 5) {
```

所以 jump 的状态来自：

```text
时间条件 + 速度条件 + 触地力阈值
```

一句话：

```text
walk 是周期调度，jump 是事件驱动。
```

## 4. MPC 使用方式不同

### 4.1 walk_mpc_wbc 中的 MPC

walking 里的 MPC 主要做：

```text
在未来预测窗口内，规划双脚接触 wrench，
让 base 姿态、位置、速度跟踪期望轨迹。
```

MPC 输出写入：

```text
RobotState.Fr_ff
```

然后交给 WBC：

```text
MPC -> Fr_ff -> WBC_priority
```

所以 walking 里的 MPC 更像 WBC 的前馈接触力规划器。

### 4.2 jump_mpc 中的 MPC

jump 里的 MPC 只在关键阶段启用。

起跳阶段启用：

[jump_mpc.cpp:167](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:167)

```cpp
mpc_force.enable();
```

上升和下落阶段关闭：

[jump_mpc.cpp:204](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:204)

```cpp
mpc_force.disable();
```

[jump_mpc.cpp:233](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:233)

```cpp
mpc_force.disable();
```

落地恢复阶段重新启用：

[jump_mpc.cpp:268](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:268)

```cpp
mpc_force.enable();
```

可以先记成：

```text
起跳推地: MPC 开
空中飞行: MPC 关
落地恢复: MPC 开
```

原因是：

```text
MPC 当前优化的是接触力。
空中没有脚底接触，接触力没有实际执行对象。
```

## 5. WBC 层差异

这是两个 demo 最大的工程差异。

### 5.1 walk_mpc_wbc 有完整 WBC

walking 中，MPC 不是直接发关节力矩，而是把结果交给 WBC。

WBC 会综合：

```text
base 姿态任务
base 位置任务
摆脚任务
接触约束
动力学约束
关节力矩
```

所以它是更完整的全身协调层。

可以理解成：

```text
MPC 决定地面对机器人该怎么推
WBC 决定全身关节应该怎么配合
```

### 5.2 jump_mpc 没有完整 WBC

jump 中没有走：

```text
MPC -> WBC_priority -> wbc_tauJointRes
```

而是直接：

```text
MPC -> fe_react_tau_cmd -> -J_stand^T F -> motors_tor_des
```

也就是：

```text
tau_leg = -J_stand^T F_foot
```

这更像一个直接力控制 demo。

优点：

```text
链路短，容易理解，起跳推力很直接。
```

缺点：

```text
没有完整处理全身任务优先级，也没有像 walking 那样系统地协调上身、双脚和接触约束。
```

## 6. 接触状态处理不同

### 6.1 walking 的接触状态来自步态调度

walking 中，接触状态由 `GaitScheduler` 维护。

它关心的是：

```text
当前哪只脚支撑
哪只脚摆动
相位走到哪里
下一步什么时候切换
```

所以接触是可预测、周期性的。

### 6.2 jump 的接触状态来自阶段和触地检测

jump 里真正重要的切换是落地检测：

[jump_mpc.cpp:254](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:254)

```cpp
if (FLest(2) > 1000 && FRest(2) > 1000) {
    jump_state = 5;
}
```

这里的意思是：

```text
如果左右脚估计出的竖直接触力都很大，
说明机器人已经触地，
于是进入落地恢复阶段。
```

所以 jump 的接触是事件触发的：

```text
没有落地 -> 继续下落阶段
检测到落地 -> 进入恢复阶段
```

## 7. 对奔跑 demo 的启发

奔跑不是简单地把 walking 速度调大。

奔跑至少要比 walking 多处理：

```text
飞行相
更短接触时间
更大的冲击力
更激进的落脚点规划
更强的 CoM 动量变化
```

所以后续如果做 running，不能只改 `GaitScheduler` 的周期参数。

更合理的路线是：

```text
1. 保留 walk_mpc_wbc 的模块化链路
2. 扩展 GaitScheduler，让它支持 flight phase
3. 扩展 FootPlacement，让落脚点考虑飞行时间和 CoM 速度
4. MPC 接触序列允许双脚都不接触
5. 落地阶段增加冲击缓冲和力矩限幅
```

也就是：

```text
running = walking 的周期框架 + jump 的飞行/落地事件处理
```

## 8. 对边走边跳 demo 的启发

边走边跳不是单独的 walking，也不是单独的 jump。

它更像：

```text
正常 walking
-> 检测到跳跃触发
-> 切换到 jump module
-> 起跳
-> 飞行
-> 落地
-> 回到 walking gait
```

因此需要一个更高层状态机：

```text
LOCOMOTION_WALK
LOCOMOTION_JUMP_PREPARE
LOCOMOTION_JUMP_TAKEOFF
LOCOMOTION_FLIGHT
LOCOMOTION_LANDING
LOCOMOTION_RECOVER
LOCOMOTION_WALK
```

这时 `jump_mpc.cpp` 里硬编码的 `jump_state` 最好不要继续塞在 demo 主循环里。

更好的工程结构是：

```text
JumpStateMachine
-> 生成 jump 阶段目标
-> 写入 DataBus
-> 下层仍复用 MPC / WBC / PVT
```

一句话：

```text
边走边跳的关键不是 MPC 公式本身，
而是 walking gait 和 jump state machine 怎么衔接。
```

## 9. 对跨栏 demo 的启发

跨栏比边走边跳又多一个问题：

```text
脚必须越过障碍物。
```

所以它不只是跳高，还要规划：

```text
起跳点
最高点
跨越障碍时的脚尖高度
落地点
落地恢复
```

可以拆成三层：

```text
1. 障碍物感知 / 手动给定障碍物位置
2. CoM 和足端轨迹规划
3. MPC/WBC 执行轨迹
```

第一版可以先不做视觉感知，直接给定：

```text
obstacle_x
obstacle_height
landing_x
```

然后生成足端轨迹：

```text
p_foot_z(t) > obstacle_height + safety_margin
```

再让 MPC/WBC 跟踪。

所以跨栏 demo 的最小可行版本可以是：

```text
walk_mpc_wbc 的行走框架
+ jump_mpc 的起跳/落地阶段
+ 一个手写的 obstacle-aware foot trajectory
```

## 10. OpenLoong 里应该保留什么

后续扩展时，OpenLoong 不要大改一锅粥。

建议保留主线模块：

```text
MJ_Interface
Pin_KinDyn
DataBus
MPC
WBC_priority
GaitScheduler
FootPlacement
PVT_Ctr
```

其中：

```text
walk_mpc_wbc = 模块化 locomotion 主线
jump_mpc     = 跳跃状态机和直接力控制参考
```

更适合的做法是新建 demo，而不是改坏原 demo：

```text
demo/run_mpc_wbc.cpp
demo/walk_jump_mpc_wbc.cpp
demo/hurdle_mpc_wbc.cpp
```

这样原来的三个 demo 可以继续作为对照：

```text
walk_wbc       看纯 WBC
walk_mpc_wbc   看 MPC + WBC 行走
jump_mpc       看状态机跳跃
```

## 11. 后续外部仓库怎么读

外部仓库不要一开始就全量移植。

建议只把它们当成参考答案：

```text
Learning_MPC_Jumping
  看跳跃阶段如何组织，以及学习残差如何补 MPC 模型误差。

Quadruped-PyMPC
  看单刚体 MPC、接触力约束和 Python 原型结构。

olympus_mpc_demo
  看空中姿态控制和落地准备。
```

读外部仓库时，每个仓库只回答三个问题：

```text
1. 它的状态机怎么设计？
2. 它的 MPC 优化变量是什么？
3. 它在飞行相和落地相分别控制什么？
```

不要一开始追求全部跑通。

## 12. 一句话总结

`walk_mpc_wbc` 是一个模块化的周期行走控制框架：

```text
GaitScheduler + FootPlacement + MPC + WBC
```

`jump_mpc` 是一个状态机驱动的跳跃 demo：

```text
jump_state + MPC + -J^T F + PVT
```

后续做奔跑、边走边跳、跨栏，最自然的路线不是从零写新控制器，而是：

```text
以 walk_mpc_wbc 作为主框架，
吸收 jump_mpc 的起跳、飞行、落地阶段逻辑，
再逐步补上飞行相姿态控制、落脚点规划和落地恢复。
```

## 13. 面试一句话

可以这样讲：

```text
我对比了 OpenLoong 的 walk_mpc_wbc 和 jump_mpc。前者是模块化的行走链路，通过 GaitScheduler、FootPlacement、MPC 和 WBC 实现周期步态；后者是硬编码状态机驱动的跳跃 demo，只在起跳和落地恢复阶段启用 MPC，并通过 -J^T F 直接把足端 wrench 映射为腿部关节力矩。这个对比说明，如果要扩展奔跑、边走边跳或跨栏，应该以 walk_mpc_wbc 的模块化架构为主线，再引入 jump_mpc 的飞行相和落地事件处理。
```
