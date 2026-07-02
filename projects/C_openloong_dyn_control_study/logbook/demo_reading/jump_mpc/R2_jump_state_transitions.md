# R2: jump_state 和阶段切换

## 1. 本轮目标

本轮只读 `jump_mpc.cpp` 里的跳跃状态机，不展开 MPC 内部、不展开 PVT 内部。

核心问题是：

```text
jump_mpc 是怎样从准备站立，切到起跳、空中、下落、落地恢复的？
```

源码范围：

- 时间分段入口：[jump_mpc.cpp:129](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:129)
- 跳跃主状态机入口：[jump_mpc.cpp:161](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:161)
- `jump_state == 0` 起跳：[jump_mpc.cpp:166](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:166)
- `jump_state == 3` 空中上升：[jump_mpc.cpp:203](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:203)
- `jump_state == 4` 下落准备：[jump_mpc.cpp:232](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:232)
- `jump_state == 5` 落地恢复：[jump_mpc.cpp:267](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:267)

## 2. 本轮边界

- 只解释 `simTime`、`prepareTime`、`startJumpingTime`、`jump_state` 的阶段切换。
- 只说明每个阶段写入哪些目标量，例如 `js_pos_des`、`js_vel_des`、`motors_pos_des`。
- MPC 在本轮只看 `enable()` / `disable()`，不展开 `MPC::cal()`。
- `Jac_stand`、`Uje = -J^T F` 放到 R4 细读。
- PVT 和 `mj_interface.setMotorsTorque()` 放到 R5 细读。

## 3. 状态机是写在 demo 主循环里的

`jump_state` 定义在：

[jump_mpc.cpp:84](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:84)

```cpp
uint16_t jump_state = 0;
```

这不是一个单独类，也不是类似 `GaitScheduler` 的模块。

当前源码没有：

```cpp
JumpScheduler jumpScheduler;
jumpScheduler.step();
jumpScheduler.dataBusWrite(RobotState);
```

而是直接在 `jump_mpc.cpp` 主循环里用 `if / else if` 写死：

```text
时间分段
-> jump_state 分段
-> 写 RobotState 目标量
```

所以这个 jump demo 更像一个状态机脚本，而不是一个通用跳跃控制框架。

## 4. 两层分段结构

源码先按时间分成三大段：

[jump_mpc.cpp:129](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:129)

```cpp
if (simTime <= prepareTime) {
    ...
} else if (simTime < startJumpingTime && simTime > prepareTime) {
    ...
} else if (simTime >= startJumpingTime) {
    ...
}
```

其中参数来自：

[jump_mpc.cpp:82](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:82)

```cpp
double startJumpingTime = 8.5;
double prepareTime = 3;
```

所以第一层时间轴是：

```text
0 ~ 3s       准备站立
3 ~ 8.5s     起跳前调整姿态
8.5s 以后    进入跳跃状态机
```

真正的跳跃状态机只发生在：

```cpp
else if (simTime >= startJumpingTime)
```

内部。

## 5. 外层阶段一：准备站立

代码位置：[jump_mpc.cpp:129](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:129)

```cpp
if (simTime <= prepareTime) {
    fe_l_pos_L_des = RobotState.fe_l_pos_L;
    fe_r_pos_L_des = RobotState.fe_r_pos_L;
    fe_l_pos_W_des = RobotState.base_rot * fe_l_pos_L_des;
    fe_r_pos_W_des = RobotState.base_rot * fe_r_pos_L_des;
    RobotState.motors_pos_des = eigen2std(resLeg.jointPosRes + resHand.jointPosRes);
    RobotState.motors_vel_des.assign(model_nv - 6, 0);
    RobotState.motors_tor_des.assign(model_nv - 6, 0);
}
```

这一段对应：

```text
0 ~ 3s
```

目的很简单：

```text
先保持初始 IK 姿态，让机器人站稳。
```

关键操作：

```cpp
fe_l_pos_L_des = RobotState.fe_l_pos_L;
fe_r_pos_L_des = RobotState.fe_r_pos_L;
```

意思是：

```text
脚当前在哪里，期望脚位置就设在哪里。
```

然后写入电机目标：

```cpp
RobotState.motors_pos_des = eigen2std(resLeg.jointPosRes + resHand.jointPosRes);
RobotState.motors_vel_des.assign(model_nv - 6, 0);
RobotState.motors_tor_des.assign(model_nv - 6, 0);
```

也就是：

```text
q_des      = 初始 IK 结果
dq_des     = 0
tau_des    = 0
```

这个阶段没有使用 `jump_state`，也没有主动打开 MPC。

## 6. 外层阶段二：起跳前姿态调整

代码位置：[jump_mpc.cpp:137](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:137)

```cpp
else if (simTime < startJumpingTime && simTime > prepareTime) {
```

这一段对应：

```text
3s ~ 8.5s
```

主要目的：

```text
慢慢改变脚相对 base 的 z 方向目标，进入起跳前站姿。
```

关键代码：

[jump_mpc.cpp:138](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:138)

```cpp
fe_l_pos_L_des(2) = Ramp(fe_l_pos_L_des(2), stand_z, 0.1 * dt);
fe_r_pos_L_des(2) = Ramp(fe_r_pos_L_des(2), stand_z, 0.1 * dt);
```

其中：

[jump_mpc.cpp:85](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:85)

```cpp
double stand_z = -0.8;
```

所以脚相对 base 的 z 方向目标逐渐变为：

```text
z_foot_des = -0.8
```

`Ramp()` 的作用是限速，避免目标突变：

```text
z_next = Ramp(z_current, z_target, step)
```

脚目标变了以后，重新做 IK：

[jump_mpc.cpp:141](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:141)

```cpp
auto resLeg = kinDynSolver.computeInK_Leg(...);
auto resHand = kinDynSolver.computeInK_Hand(...);
```

然后继续用位置控制跟踪新的 IK 结果：

```cpp
RobotState.motors_pos_des = eigen2std(resLeg.jointPosRes + resHand.jointPosRes);
RobotState.motors_vel_des.assign(model_nv - 6, 0);
RobotState.motors_tor_des.assign(model_nv - 6, 0);
```

这一阶段还会把当前 base 状态写成跳跃参考：

[jump_mpc.cpp:152](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:152)

```cpp
RobotState.js_eul_des(j) = RobotState.base_rpy(j);
RobotState.js_pos_des(j) = RobotState.base_pos(j);
RobotState.js_omega_des(j) = RobotState.base_omega_W(j);
RobotState.js_vel_des(j) = RobotState.dq(j);
RobotState.legState = DataBus::DSt;
```

这里的 `js_` 可以理解为 jump state 使用的 base 期望状态：

```text
js_eul_des    期望姿态
js_pos_des    期望位置
js_omega_des  期望角速度
js_vel_des    期望线速度
```

`DataBus::DSt` 表示 double stance，也就是双脚支撑。

## 7. 外层阶段三：进入跳跃状态机

代码位置：[jump_mpc.cpp:161](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:161)

```cpp
else if (simTime >= startJumpingTime) {
```

进入这一段后，先计算跳跃目标速度：

[jump_mpc.cpp:162](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:162)

```cpp
double jump_vel_des[3] = {0.0, 0.0, sqrt(2.0 * 9.8 * jump_z)};
```

其中：

[jump_mpc.cpp:86](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:86)

```cpp
double jump_z = 0.2;
```

使用的是抛体公式：

```text
v_z = sqrt(2 * g * h)
```

所以：

```text
v_z = sqrt(2 * 9.8 * 0.2) ~= 1.98 m/s
```

这就是起跳阶段希望 base 获得的向上速度。

然后计算起跳加速时间：

[jump_mpc.cpp:163](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:163)

```cpp
double jump_acc_t = 2.0 * (0.9 + stand_z) / (jump_vel_des[2]);
```

因为：

```text
stand_z = -0.8
```

所以：

```text
jump_acc_t ~= 2 * 0.1 / 1.98 ~= 0.1s
```

也就是说，起跳推力阶段大约持续 0.1 秒。

## 8. jump_state == 0：起跳加速

代码位置：[jump_mpc.cpp:166](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:166)

```cpp
if (jump_state == 0) {// Jump
```

这一阶段的目标是：

```text
双脚仍然接触地面，通过 MPC 规划接触力，把 base 向上推。
```

第一步启用 MPC：

[jump_mpc.cpp:167](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:167)

```cpp
mpc_force.enable();
```

然后设置起跳阶段的 MPC 权重：

[jump_mpc.cpp:170](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:170)

```cpp
L_diag <<
       50.0, 50.0, 1.0,      // eul
        50.0, 50.0, 200.0,   // pCoM
        0.1, 0.1 , 0.1,      // w
        0.01, 0.1, 20.0;     // vCoM
```

这里对 `pCoM z` 和 `vCoM z` 给了较大权重：

```text
p_z weight = 200
v_z weight = 20
```

说明起跳阶段最关心的是竖直位置和竖直速度。

接着设置姿态目标：

[jump_mpc.cpp:182](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:182)

```cpp
RobotState.js_eul_des(0) = jump_eul_des[0];
RobotState.js_eul_des(1) = jump_eul_des[1];
RobotState.js_eul_des(2) = jump_eul_des[2];
```

当前 `jump_eul_des` 等价于：

```text
roll_des  = 0
pitch_des = 0
yaw_des   = 0
```

然后生成起跳速度目标：

[jump_mpc.cpp:186](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:186)

```cpp
RobotState.js_vel_des(0) = Ramp(...);
RobotState.js_vel_des(1) = 0.0;
RobotState.js_vel_des(2) = Ramp(RobotState.js_vel_des(2), jump_vel_des[2],
                                fabs(jump_vel_des[2] / jump_acc_t * dt));
```

也就是：

```text
v_x_des -> 0
v_y_des = 0
v_z_des -> sqrt(2gh)
```

之后积分得到高度目标：

[jump_mpc.cpp:192](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:192)

```cpp
RobotState.js_pos_des(2) = RobotState.js_pos_des(2) + RobotState.js_vel_des(2) * dt;
```

对应：

```text
z_des_next = z_des_current + v_z_des * dt
```

状态切换条件：

[jump_mpc.cpp:197](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:197)

```cpp
if (simTime > startJumpingTime + jump_acc_t) {
    jump_state = 3;
    mpc_force.disable();
    RobotState.pfeW0.block<3, 1>(0, 0) = fe_l_pos_W_des;
    RobotState.pfeW0.block<3, 1>(3, 0) = fe_r_pos_W_des;
}
```

也就是：

```text
起跳推力时间结束 -> jump_state: 0 -> 3
```

同时关闭 MPC，因为下一阶段是空中阶段。

## 9. jump_state == 3：空中上升

代码位置：[jump_mpc.cpp:203](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:203)

```cpp
else if (jump_state == 3) { //up
```

这一阶段的目标是：

```text
MPC 关闭，靠 IK / PVT 调整腿型，为后续落地准备。
```

源码再次关闭 MPC：

[jump_mpc.cpp:204](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:204)

```cpp
mpc_force.disable();
```

然后调整脚的 z 方向目标：

[jump_mpc.cpp:205](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:205)

```cpp
fe_l_pos_W_des[2] = Ramp(fe_l_pos_W_des[2], stand_z, 5.0 * dt);
fe_r_pos_W_des[2] = Ramp(fe_r_pos_W_des[2], stand_z, 5.0 * dt);
```

再把世界系脚目标转换回 base 坐标系，供 IK 使用：

[jump_mpc.cpp:208](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:208)

```cpp
fe_l_pos_L_des = RobotState.base_rot.inverse() * fe_l_pos_W_des;
fe_r_pos_L_des = RobotState.base_rot.inverse() * fe_r_pos_W_des;
```

然后重新做 IK，并写入电机位置目标：

```cpp
RobotState.motors_pos_des = eigen2std(IKRes);
RobotState.motors_vel_des.assign(model_nv - 6, 0);
RobotState.motors_tor_des.assign(model_nv - 6, 0);
```

状态切换条件：

[jump_mpc.cpp:221](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:221)

```cpp
if (RobotState.dq(2) < 0.1) {
    jump_state = 4;
    RobotState.pfeW0.block<3, 1>(0, 0) = RobotState.base_rot * RobotState.fe_l_pos_L;
    RobotState.pfeW0.block<3, 1>(3, 0) = RobotState.base_rot * RobotState.fe_r_pos_L;
}
```

`RobotState.dq(2)` 可以理解为 base 的竖直速度。

当：

```text
v_z < 0.1
```

说明机器人接近最高点，于是：

```text
jump_state: 3 -> 4
```

进入下落准备阶段。

## 10. jump_state == 4：下落准备和触地检测

代码位置：[jump_mpc.cpp:232](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:232)

```cpp
else if (jump_state == 4) { // down
```

这一阶段的目标是：

```text
下落前把脚往前放一点，并等待触地检测。
```

MPC 仍然关闭：

[jump_mpc.cpp:233](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:233)

```cpp
mpc_force.disable();
```

脚的 x 方向目标向前移动：

[jump_mpc.cpp:234](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:234)

```cpp
fe_l_pos_W_des[0] = Ramp(fe_l_pos_W_des[0], RobotState.pfeW0[0] + 0.2, fabs(10.0 * dt));
fe_r_pos_W_des[0] = Ramp(fe_r_pos_W_des[0], RobotState.pfeW0[3] + 0.2, fabs(10.0 * dt));
```

也就是：

```text
x_foot_des = x_foot_0 + 0.2m
```

这里可以理解为落地前向前摆脚。

触地检测在：

[jump_mpc.cpp:254](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:254)

```cpp
if (FLest(2) > 1000 && FRest(2) > 1000) {
    jump_state = 5;
    RobotState.pfeW0.block<3, 1>(0, 0) = RobotState.base_rot * RobotState.fe_l_pos_L;
    RobotState.pfeW0.block<3, 1>(3, 0) = RobotState.base_rot * RobotState.fe_r_pos_L;
    RobotState.js_pos_des(0) = RobotState.base_pos(0);
    RobotState.js_pos_des(1) = RobotState.base_pos(1);
    RobotState.js_pos_des(2) = RobotState.base_pos(2);
}
```

`FLest(2)` 和 `FRest(2)` 是左右脚估计竖直反力。

判断条件是：

```text
F_Lz > 1000
F_Rz > 1000
```

满足后认为双脚已经触地：

```text
jump_state: 4 -> 5
```

并把当前 base 位置写成恢复阶段的初始目标，避免恢复目标突然跳变。

## 11. jump_state == 5：落地恢复

代码位置：[jump_mpc.cpp:267](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:267)

```cpp
else if (jump_state == 5) {
```

这一阶段的目标是：

```text
重新进入双脚支撑，打开 MPC，把身体恢复到站立高度和零速度状态。
```

首先重新启用 MPC：

[jump_mpc.cpp:268](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:268)

```cpp
mpc_force.enable();
```

因为落地后双脚重新接触地面，MPC 计算接触力又有物理意义。

恢复阶段使用另一套 MPC 权重：

[jump_mpc.cpp:272](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:272)

```cpp
L_diag <<2.0, 10.0, 1.0,        // eul
        100.0, 100.0, 200.0,    // pCoM
        1e-4, 1e-4, 1e-4,       // w
        0.5, 0.01, 0.5;         // vCoM
```

这里 `pCoM` 权重比较大，说明落地后更重视身体位置恢复，尤其是高度。

恢复目标：

[jump_mpc.cpp:283](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:283)

```cpp
double tt = 0.4;
RobotState.js_pos_des(2) = Ramp(RobotState.js_pos_des(2), 1.08, 0.1 * dt);
RobotState.js_eul_des.setZero();
RobotState.js_omega_des.setZero();
RobotState.js_vel_des.setZero();
```

含义是：

```text
z_des      -> 1.08
eul_des    -> 0
omega_des  -> 0
vel_des    -> 0
```

也就是让机器人重新站稳。

`tt = 0.4` 和 `count`：

[jump_mpc.cpp:289](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:289)

```cpp
if (count < tt / dt)
    count = count + 1.0;
```

当前源码里 `count` 只递增，没有继续参与新的状态切换或控制公式，所以它更像是预留的恢复计时变量。

## 12. 状态转移总表

| 阶段 | 触发条件 | 主要动作 | MPC |
| --- | --- | --- | --- |
| 准备站立 | `simTime <= prepareTime` | 保持初始 IK 姿态 | 未主动启用 |
| 起跳前调整 | `prepareTime < simTime < startJumpingTime` | `fe_*_pos_L_des(2) -> stand_z`，重新 IK | 未主动启用 |
| `jump_state == 0` 起跳 | `simTime >= startJumpingTime` 且状态为 0 | 生成 `v_z_des` 和 `z_des`，设置起跳 MPC 权重 | 启用 |
| `0 -> 3` | `simTime > startJumpingTime + jump_acc_t` | 记录脚位置，进入空中 | 关闭 |
| `jump_state == 3` 上升 | 状态为 3 | 空中调整脚 z 目标，重新 IK | 关闭 |
| `3 -> 4` | `RobotState.dq(2) < 0.1` | 接近最高点，进入下落 | 关闭 |
| `jump_state == 4` 下落 | 状态为 4 | 脚 x 方向前移 0.2m，等待触地 | 关闭 |
| `4 -> 5` | `FLest(2) > 1000 && FRest(2) > 1000` | 认为双脚触地，记录当前 base 位置 | 关闭到即将启用 |
| `jump_state == 5` 恢复 | 状态为 5 | `z_des -> 1.08`，姿态/速度目标清零 | 启用 |

## 13. 一句话总结

`jump_mpc` 的跳跃状态机是写死在 demo 主循环里的两层逻辑：

```text
先按 simTime 分成准备、起跳前、正式跳跃；
正式跳跃后再用 jump_state 做 0 -> 3 -> 4 -> 5 的阶段切换。
```

它的核心切换依据是：

```text
时间到 8.5s     -> 开始起跳
起跳加速时间结束 -> 进入空中
竖直速度接近 0   -> 进入下落
双脚竖直反力足够大 -> 认为落地
```

MPC 的启停原则也很清楚：

```text
有稳定接触并需要推地/恢复时，MPC 启用；
空中或未稳定接触时，MPC 关闭。
```

## 14. 下一轮阅读

R3 建议专门读：

```text
MPC 何时介入、输出如何使用
```

重点源码：

- 起跳阶段 `mpc_force.enable()` 和 `set_weight()`：[jump_mpc.cpp:167](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:167)
- 落地恢复 `mpc_force.enable()` 和恢复权重：[jump_mpc.cpp:268](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:268)
- 每周期统一调用 MPC：[jump_mpc.cpp:294](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:294)

