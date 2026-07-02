# R3 fast: MPC 到电机输出快速阅读

## 1. 本轮目标

本轮把原计划里的 R3、R4、R5 合并快速读完：

```text
R3: MPC 何时介入、输出如何使用
R4: Jacobian / wrench -> 关节力矩
R5: PVT 和最终 ctrl 写回
```

这三部分在 R1 主流程里已经扫过，本轮只固化一条最重要的数据链：

```text
MPC
-> fe_react_tau_cmd
-> -J_stand^T F
-> motors_tor_des
-> PVT_Ctr
-> motors_tor_out
-> MuJoCo ctrl
```

## 2. 本轮边界

- 不展开 `MPC::cal()` 内部 QP。
- 不重新细读 `jump_state`，R2 已经完成。
- 不展开 `PVT_Ctr::calMotorsPVT()` 内部实现。
- 只关注 `jump_mpc.cpp` 顶层 demo 如何把 MPC 输出接到电机力矩。

## 3. MPC 什么时候启用

MPC 的启停由 `jump_state` 决定。

起跳阶段打开 MPC：

[jump_mpc.cpp:167](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:167)

```cpp
mpc_force.enable();
```

起跳结束后关闭：

[jump_mpc.cpp:197](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:197)

```cpp
if (simTime > startJumpingTime + jump_acc_t) {
    jump_state = 3;
    mpc_force.disable();
}
```

空中上升阶段继续关闭：

[jump_mpc.cpp:203](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:203)

```cpp
else if (jump_state == 3) { //up
    mpc_force.disable();
```

下落阶段也关闭：

[jump_mpc.cpp:232](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:232)

```cpp
else if (jump_state == 4) { // down
    mpc_force.disable();
```

落地恢复阶段重新打开：

[jump_mpc.cpp:267](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:267)

```cpp
else if (jump_state == 5) {
    mpc_force.enable();
```

所以可以先记成：

```text
起跳推地：MPC 开
空中上升：MPC 关
下落摆腿：MPC 关
落地恢复：MPC 开
```

原因也很直接：

```text
只有脚和地面存在有效接触时，MPC 规划的足端接触力才有物理意义。
```

## 4. 每周期统一调用 MPC

虽然 MPC 有启停状态，但主循环每个控制周期都会调用：

[jump_mpc.cpp:294](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:294)

```cpp
mpc_force.dataBusRead(RobotState);
mpc_force.cal();
mpc_force.dataBusWrite(RobotState);
```

含义是：

```text
dataBusRead  从 RobotState 读当前状态和期望状态
cal          求解 MPC
dataBusWrite 把结果写回 RobotState
```

真正是否使用 MPC 输出，由下面这个判断决定：

[jump_mpc.cpp:298](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:298)

```cpp
if (mpc_force.get_ENA()) {
```

如果 MPC 没启用，就清零足端力命令：

[jump_mpc.cpp:314](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:314)

```cpp
else {
    RobotState.fe_react_tau_cmd.setZero();
}
```

## 5. MPC 输出是什么

在 `jump_mpc` 这条链里，后面实际使用的是：

```cpp
RobotState.fe_react_tau_cmd.block<nu - 1, 1>(0, 0)
```

因为：

```text
nu = 13
nu - 1 = 12
```

所以这里取 12 维足端 wrench：

```text
F_foot =
[f_L, tau_L, f_R, tau_R]^T
```

展开就是：

```text
F_foot =
[fLx, fLy, fLz, tauLx, tauLy, tauLz,
 fRx, fRy, fRz, tauRx, tauRy, tauRz]^T
```

这 12 维是后面映射到腿部关节力矩的输入。

## 6. 构造 Jac_stand

`Jac_stand` 在主循环前半段构造：

[jump_mpc.cpp:109](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:109)

```cpp
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Jac_stand(12, 12);
Jac_stand.setZero();
Jac_stand.block(0, 0, 6, 12) = RobotState.J_l.block(0, model_nv-12, 6, 12);
Jac_stand.block(6, 0, 6, 12) = RobotState.J_r.block(0, model_nv-12, 6, 12);
```

它把左右脚 Jacobian 的最后 12 个腿部关节列取出来：

```text
J_stand =
[J_L_leg
 J_R_leg]
```

维度是：

```text
J_stand in R^{12 x 12}
```

为什么取最后 12 列：

```text
model_nv 前 6 维是 floating base
最后 12 维对应左右腿关节
```

这里不使用全身所有关节，而是只把足端 wrench 映射到腿部 12 个关节。

## 7. 核心公式：wrench -> 关节力矩

核心代码：

[jump_mpc.cpp:300](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:300)

```cpp
Uje = Jac_stand.transpose() * (-1.0)
    * RobotState.fe_react_tau_cmd.block<nu - 1, 1>(0, 0);
```

对应公式：

```text
U_je = - J_stand^T F_foot
```

维度关系：

```text
J_stand      in R^{12 x 12}
J_stand^T    in R^{12 x 12}
F_foot       in R^{12}
U_je         in R^{12}
```

其中：

```text
U_je = 左右腿 12 个关节的力矩命令
```

负号表示源码中 `fe_react_tau_cmd` 的接触力方向和关节需要施加的等效力矩方向相反。

用虚功关系可以先记：

```text
tau = J^T F
```

当前源码实际使用：

```text
tau = -J^T F
```

## 8. 关节力矩限幅

映射得到 `Uje` 以后，源码先限幅：

[jump_mpc.cpp:301](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:301)

```cpp
double jTor_max[6] = {400.0, 100.0, 400.0, 400.0, 80.0, 20.0};
double jTor_min[6] = {-400.0, -100.0, -400.0, -400.0, -80.0, -20.0};
```

[jump_mpc.cpp:304](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:304)

```cpp
for (int i = 0; i < 6; i++) {
    Limit(Uje(i), jTor_max[i], jTor_min[i]);
    Limit(Uje(i + 6), jTor_max[i], jTor_min[i]);
}
```

左腿和右腿各 6 个关节，共用同一组力矩上下限。

限幅的原因：

```text
MPC 输出的接触力经过 -J^T F 后可能导致过大的关节力矩。
直接限幅可以避免电机命令不现实，也避免仿真发散。
```

## 9. 写入最后 12 个腿部关节

限幅后，把 `Uje` 写入腿部关节力矩目标：

[jump_mpc.cpp:309](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:309)

```cpp
for (int i = 0; i < 12; i++) {
    pvtCtr.disablePV(model_nv-6-12 + i);
    RobotState.motors_tor_des[model_nv-6-12 + i] = Uje(i);
}
```

这里有两个动作。

第一，关闭最后 12 个腿部关节的 PV 控制：

```cpp
pvtCtr.disablePV(model_nv-6-12 + i);
```

第二，写入力矩目标：

```cpp
RobotState.motors_tor_des[...] = Uje(i);
```

也就是说：

```text
这些腿部关节不再主要靠 q_des / dq_des 的 PV 跟踪，
而是直接使用 MPC 映射出来的 torque。
```

这是 `jump_mpc` 和 `walk_mpc_wbc` 最大的输出层区别之一。

## 10. PVT 计算最终电机输出

PVT 读取 `RobotState`：

[jump_mpc.cpp:319](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:319)

```cpp
pvtCtr.dataBusRead(RobotState);
```

然后计算电机输出：

[jump_mpc.cpp:320](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:320)

```cpp
if (simTime <= startJumpingTime) {
    pvtCtr.calMotorsPVT(110.0 / 1000.0 / 180.0 * 3.1415);
} else {
    pvtCtr.calMotorsPVT();
}
```

起跳前使用带参数版本，动作更柔和。

起跳后使用默认版本，允许更直接地输出力矩。

PVT 写回：

[jump_mpc.cpp:325](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:325)

```cpp
pvtCtr.dataBusWrite(RobotState);
```

最终结果进入：

```text
RobotState.motors_tor_out
```

可以粗略理解：

```text
motors_tor_out =
Kp * (q_des - q)
+ Kd * (dq_des - dq)
+ motors_tor_des
```

对于已经 `disablePV()` 的腿部关节，PV 项被关闭或削弱，`motors_tor_des` 里的 `Uje` 会成为主要输出来源。

## 11. 写回 MuJoCo

最终写回仿真：

[jump_mpc.cpp:328](/d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control/demo/jump_mpc.cpp:328)

```cpp
mj_interface.setMotorsTorque(RobotState.motors_tor_out);
```

这一步把最终电机力矩写入 MuJoCo actuator。

下一次：

```cpp
mj_step(mj_model, mj_data);
```

这些力矩就会作用到机器人身上。

## 12. 完整输出链路

这一轮的完整链路可以写成：

```text
RobotState 当前状态和 jump_state 目标
-> mpc_force.dataBusRead(RobotState)
-> mpc_force.cal()
-> mpc_force.dataBusWrite(RobotState)
-> RobotState.fe_react_tau_cmd[0:12]
-> Uje = -J_stand^T F_foot
-> Limit(Uje)
-> RobotState.motors_tor_des[last 12] = Uje
-> pvtCtr.calMotorsPVT()
-> RobotState.motors_tor_out
-> mj_interface.setMotorsTorque(...)
-> MuJoCo
```

一句话总结：

```text
jump_mpc 没有把 MPC 输出交给 WBC，
而是直接用接触 Jacobian 把足端 wrench 映射成腿部关节力矩，
再通过 PVT 输出到 MuJoCo。
```

## 13. 和 walk_mpc_wbc 的一句话区别

`walk_mpc_wbc` 的链路更像：

```text
MPC -> Fr_ff -> WBC_priority -> tau -> PVT
```

`jump_mpc` 的链路更像：

```text
MPC -> fe_react_tau_cmd -> -J^T F -> motors_tor_des -> PVT
```

所以 `jump_mpc` 更直接，也更像一个 demo 原型。

## 14. 本轮结论

R3-R5 可以合并理解：

```text
R3: MPC 只在起跳和落地恢复时启用。
R4: MPC 输出的足端 wrench 通过 -J_stand^T F 映射成腿部 12 维关节力矩。
R5: PVT 把 motors_tor_des 整形成 motors_tor_out，最后写入 MuJoCo。
```

下一轮可以进入 R6：

```text
和 walk_mpc_wbc 的差异总结，以及对后续奔跑 / 边走边跳 / 跨栏扩展的启发。
```

