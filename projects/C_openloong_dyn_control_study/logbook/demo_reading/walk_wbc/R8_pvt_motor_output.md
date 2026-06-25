# walk_wbc 第八轮阅读笔记

## 1. 本轮目标

第八轮开始回答这个问题：

$$
\text{WBC 输出的 } q_{des},\ \dot q_{des},\ \tau_{ff}
\text{ 如何变成 MuJoCo 真正执行的电机力矩？}
$$

本轮关注的源码位置：

```text
demo/walk_wbc.cpp
common/PVT_ctrl.cpp
common/PVT_ctrl.h
sim_interface/MJ_interface.cpp
common/data_bus.h
```

R7 已经完成：

$$
\ddot q_{\text{kin}},\ F_{\text{ff}}
\rightarrow
QP
\rightarrow
\tau_{\text{joint}}
$$

R8 继续看：

$$
\Delta q_{\text{wbc}},\ \dot q_{\text{wbc}},\ \tau_{\text{joint}}
\rightarrow
\tau_{\text{motor}}
$$

也就是把全身控制层的结果送到关节 PVT 控制器，再写回仿真器。

## 2. R7 到 R8 的接口

`WBC_priority::dataBusWrite()` 写回 `DataBus` 后，下游主要使用三个量：

$$
\Delta q_{\text{wbc}}
=
\text{wbc\_delta\_q\_final}
$$

$$
\dot q_{\text{wbc}}
=
\text{wbc\_dq\_final}
$$

$$
\tau_{\text{ff}}
=
\text{wbc\_tauJointRes}
$$

其中：

- \(\Delta q_{\text{wbc}}\)：R6 运动学任务层给出的关节位置修正来源。
- \(\dot q_{\text{wbc}}\)：R6 运动学任务层给出的期望关节速度。
- \(\tau_{\text{ff}}\)：R7 根据动力学和接触 wrench 反算出的关节前馈力矩。

注意这里的 \(\tau_{\text{ff}}\) 已经是关节维度，不包含 floating-base 前 6 维。

## 3. 主数据流

R8 当前读到的主链路是：

```text
WBC_solv.dataBusWrite()
-> RobotState.wbc_delta_q_final
-> RobotState.wbc_dq_final
-> RobotState.wbc_tauJointRes
-> walk_wbc.cpp 生成 motors_pos_des / motors_vel_des / motors_tor_des
-> PVT_Ctr::dataBusRead()
-> PVT_Ctr::calMotorsPVT()
-> PVT_Ctr::dataBusWrite()
-> RobotState.motors_tor_out
-> MJ_Interface::setMotorsTorque()
-> mj_data->ctrl
```

这一层不再解 WBC，也不重新求接触力。它做的是关节执行层整理：

$$
\text{WBC 全身目标}
\rightarrow
\text{关节期望位置、速度、前馈力矩}
\rightarrow
\text{PVT 反馈修正}
\rightarrow
\text{电机侧输出力矩}
$$

## 4. `walk_wbc.cpp` 中生成关节命令

走路阶段，源码先用 WBC 的 \(\Delta q\) 积分得到期望构型：

$$
q_{\text{des}}
=
\operatorname{integrateDIY}
\left(
q,\ \Delta q_{\text{wbc}}
\right)
$$

因为完整构型 \(q\) 的前 7 维是 floating-base 位姿：

$$
q =
\begin{bmatrix}
p_b\\
Q_b\\
q_j
\end{bmatrix}
$$

所以下游电机只取关节部分：

$$
q_{j,\text{des}}
=
q_{\text{des}}[7:]
$$

对应写入：

```text
motors_pos_des
```

速度命令来自：

$$
\dot q_{j,\text{des}}
=
\dot q_{\text{wbc}}
$$

对应写入：

```text
motors_vel_des
```

力矩前馈来自：

$$
\tau_{\text{ff}}
=
\tau_{\text{joint}}
=
\text{wbc\_tauJointRes}
$$

对应写入：

```text
motors_tor_des
```

因此 R8 入口处形成了三类关节命令：

$$
\left(
q_{j,\text{des}},
\dot q_{j,\text{des}},
\tau_{\text{ff}}
\right)
$$

## 5. `PVT_Ctr::dataBusRead()` 读入什么

`PVT_Ctr` 从 `DataBus` 读两类数据。

第一类是当前关节状态：

$$
q_{j}
=
\text{motors\_pos\_cur}
$$

$$
\dot q_{j}
=
\text{motors\_vel\_cur}
$$

第二类是期望关节命令：

$$
q_{j,\text{des}}
=
\text{motors\_pos\_des}
$$

$$
\dot q_{j,\text{des}}
=
\text{motors\_vel\_des}
$$

$$
\tau_{\text{ff}}
=
\text{motors\_tor\_des}
$$

所以 `PVT_Ctr` 的输入不是 floating-base 状态，而是关节层的：

$$
\left(
q_j,\ \dot q_j,\ q_{j,\text{des}},\ \dot q_{j,\text{des}},\ \tau_{\text{ff}}
\right)
$$

## 6. PVT 控制律

对第 \(i\) 个关节，PVT 先计算位置和速度反馈项：

$$
\tau_{PD,i}
=
K_{p,i}
\left(
q_{des,i}-q_i
\right)
+
K_{d,i}
\left(
\dot q_{des,i}-\dot q_i
\right)
$$

然后经过一阶低通滤波：

$$
\bar{\tau}_{PD,i}
=
LPF
\left(
\tau_{PD,i}
\right)
$$

再加上 WBC/R7 给出的前馈力矩：

$$
\tau_{link,i}
=
\bar{\tau}_{PD,i}
+
\tau_{ff,i}
$$

因此这一层的核心可以记成：

$$
\text{最终关节侧力矩}
=
\text{反馈修正}
+
\text{WBC 前馈力矩}
$$

也就是：

$$
\tau_{link}
=
LPF
\left[
K_p(q_{des}-q)
+
K_d(\dot q_{des}-\dot q)
\right]
+
\tau_{ff}
$$

## 7. 力矩限幅与减速比

PVT 算出关节侧力矩后，会先做限幅：

$$
\left|
\tau_{link,i}
\right|
\le
\tau_{\max,i}
$$

然后根据减速比转换成电机侧力矩：

$$
\tau_{motor,i}
=
\frac{\tau_{link,i}}{gear_i}
$$

源码里同时保存两种力矩：

```text
motor_tor_out_link   -> 关节侧力矩
motor_tor_out_motor  -> 电机侧力矩
```

写回 `DataBus` 时：

```text
motors_tor_cur = motor_tor_out_link
motors_tor_out = motor_tor_out_motor
```

所以需要区分：

$$
\tau_{link}
\ne
\tau_{motor}
$$

`motors_tor_out` 才是后面写入 MuJoCo 的量。

## 8. 启动阶段的位置步长限制

源码在仿真前几秒调用带参数的版本：

```text
calMotorsPVT(deltaP_Lim)
```

这时不会直接使用完整的位置跳变，而是限制每次期望位置变化：

$$
\Delta q_{des,i}
=
q_{des,i}-q_{des,i}^{old}
$$

若变化过大，则截断：

$$
\left|
\Delta q_{des,i}
\right|
\le
\Delta q_{\max}
$$

再得到本周期实际使用的期望位置：

$$
q_{des,i}^{use}
=
q_{des,i}^{old}
+
\operatorname{clip}
\left(
\Delta q_{des,i},
-\Delta q_{\max},
\Delta q_{\max}
\right)
$$

这一步的意义是防止启动时位置命令突变过大，导致 PD 反馈瞬间产生过大的力矩。

## 9. 写入 MuJoCo

`PVT_Ctr::dataBusWrite()` 写回：

$$
\tau_{motor}
\rightarrow
\text{RobotState.motors\_tor\_out}
$$

随后 `MJ_Interface::setMotorsTorque()` 执行：

$$
\text{mj\_data->ctrl}[i]
=
\tau_{motor,i}
$$

所以仿真器真正执行的是：

$$
\tau_{motor}
=
\text{motors\_tor\_out}
$$

不是直接执行 R7 的：

$$
\tau_{\text{ff}}
=
\text{wbc\_tauJointRes}
$$

R7 的力矩只是 PVT 控制律中的前馈项。

## 10. 构造函数 `PVT_Ctr::PVT_Ctr()`

`PVT_Ctr` 构造函数在主循环开始前执行一次：

```text
PVT_Ctr pvtCtr(mj_model->opt.timestep, "../common/joint_ctrl_config.json")
```

它的输入是：

$$
T_s=\text{timeStepIn}
$$

以及关节控制参数文件：

```text
common/joint_ctrl_config.json
```

构造函数先根据 `motorName` 得到关节数：

$$
n_j=\text{jointNum}
$$

然后为每个关节初始化：

$$
K_p,\ K_d,\ \tau_{\max},\ gear,\ PV,\ LPF
\in\mathbb{R}^{n_j}
$$

其中：

```text
pvt_Kp / pvt_Kd    PD 增益
maxTor             关节侧最大力矩
gear               减速比
PV_enable          是否启用位置/速度反馈
tau_out_lpf        每个关节一个一阶低通滤波器
```

随后按关节名从 JSON 读取参数：

$$
K_{p,i},\quad K_{d,i},\quad \tau_{\max,i},\quad gear_i,\quad f_{c,i}
$$

并设置低通滤波器：

$$
\alpha_i
=
\frac{T_s}{T_s+\frac{1}{2\pi f_{c,i}}}
$$

低通滤波器后面用于：

$$
\bar{\tau}_{PD,i}(k)
=
(1-\alpha_i)\bar{\tau}_{PD,i}(k-1)
+
\alpha_i\tau_{PD,i}(k)
$$

注意：构造函数也读取了 `maxVel / maxPos / minPos`，但当前 `calMotorsPVT()` 中没有真正使用这些量做速度或位置限幅。

## 11. `dataBusRead()` 和 `dataBusWrite()`

`dataBusRead()` 是 PVT 的输入入口：

$$
\text{DataBus}
\rightarrow
\text{PVT\_Ctr 内部变量}
$$

它读入当前反馈状态：

$$
q_{cur}
=
\text{motors\_pos\_cur}
$$

$$
\dot q_{cur}
=
\text{motors\_vel\_cur}
$$

也读入期望命令：

$$
q_{des}
=
\text{motors\_pos\_des}
$$

$$
\dot q_{des}
=
\text{motors\_vel\_des}
$$

$$
\tau_{ff}
=
\text{motors\_tor\_des}
$$

`dataBusWrite()` 是 PVT 的输出出口：

$$
\text{motors\_tor\_cur}
=
\tau_{link}
$$

$$
\text{motors\_tor\_out}
=
\tau_{motor}
$$

这里的两个力矩含义不同：

```text
motors_tor_cur  = 关节侧 / link-side 力矩
motors_tor_out  = 电机侧 / motor-side 力矩
```

后面真正写入 MuJoCo 的是：

$$
\text{motors\_tor\_out}
$$

## 12. `setJointPD()`

`setJointPD()` 根据关节名找到对应下标：

$$
\text{jointName}\rightarrow i
$$

然后覆盖该关节的：

$$
K_{p,i},\quad K_{d,i}
$$

主循环中，`simTime > 3` 后会对腿部关节调用 `setJointPD()`，所以实际使用的增益是：

$$
K_{p,i}^{use}
=
\begin{cases}
K_{p,i}^{setJointPD}, & \text{腿部关节被主循环覆盖}\\
K_{p,i}^{json}, & \text{其它关节}
\end{cases}
$$

$$
K_{d,i}^{use}
=
\begin{cases}
K_{d,i}^{setJointPD}, & \text{腿部关节被主循环覆盖}\\
K_{d,i}^{json}, & \text{其它关节}
\end{cases}
$$

但是：

$$
\tau_{\max,i},\quad gear_i,\quad f_{c,i}
$$

仍然来自 JSON。

实现细节：如果关节名没找到，源码会打印 `NOT found`，但随后仍可能使用 `id=-1` 访问数组。这是一个潜在安全风险；当前主流程传入的关节名都在 `motorName` 中。

## 13. 普通版 `calMotorsPVT()`

普通版 PVT 对每个关节独立计算输出。

第一步，计算 PD 反馈力矩：

$$
\tau_{PD,i}
=
PV_i
\left[
K_{p,i}(q_{des,i}-q_i)
+
K_{d,i}(\dot q_{des,i}-\dot q_i)
\right]
$$

第二步，低通滤波：

$$
\bar{\tau}_{PD,i}(k)
=
(1-\alpha_i)\bar{\tau}_{PD,i}(k-1)
+
\alpha_i\tau_{PD,i}(k)
$$

第三步，加 WBC 前馈力矩：

$$
\tau_{raw,i}
=
\bar{\tau}_{PD,i}
+
\tau_{ff,i}
$$

其中：

$$
\tau_{ff}
=
\text{wbc\_tauJointRes}
$$

第四步，关节侧力矩限幅：

$$
\tau_{link,i}
=
\operatorname{sat}_{\tau_{\max,i}}
\left(
\tau_{raw,i}
\right)
$$

第五步，通过减速比换成电机侧力矩：

$$
\tau_{motor,i}
=
\frac{\tau_{link,i}}{gear_i}
$$

因此普通版完整控制律为：

$$
\tau_{motor,i}
=
\frac{
\operatorname{sat}_{\tau_{\max,i}}
\left(
LPF_i
\left\{
PV_i
\left[
K_{p,i}(q_{des,i}-q_i)
+
K_{d,i}(\dot q_{des,i}-\dot q_i)
\right]
\right\}
+
\tau_{ff,i}
\right)
}{gear_i}
$$

## 14. 启动限幅版 `calMotorsPVT(deltaP_Lim)`

启动阶段调用：

```text
calMotorsPVT(100.0/1000.0/180.0*3.1415)
```

这个数约为：

$$
\Delta q_{\max}
\approx
0.001745\ \text{rad}
\approx
0.1^\circ
$$

这一版先限制每个控制周期的位置命令变化：

$$
\Delta q_{des,i}
=
q_{des,i}^{cmd}
-
q_{des,i}^{old}
$$

$$
\Delta q_{des,i}^{use}
=
\operatorname{clip}
\left(
\Delta q_{des,i},
-\Delta q_{\max},
\Delta q_{\max}
\right)
$$

$$
q_{des,i}^{use}
=
q_{des,i}^{old}
+
\Delta q_{des,i}^{use}
$$

然后用受限后的位置目标计算 PD：

$$
\tau_{PD,i}
=
PV_i
\left[
K_{p,i}(q_{des,i}^{use}-q_i)
+
K_{d,i}(\dot q_{des,i}-\dot q_i)
\right]
$$

后续仍然是：

$$
\tau_{motor,i}
=
\frac{
\operatorname{sat}_{\tau_{\max,i}}
\left(
LPF_i(\tau_{PD,i})
+
\tau_{ff,i}
\right)
}{gear_i}
$$

这一步的作用是避免启动阶段位置命令突然跳变，使 PD 反馈瞬间产生过大的力矩。

## 15. `gear` 的物理意义

`gear` 是减速比。PVT 先计算关节侧力矩：

$$
\tau_{link,i}
$$

再换算到电机侧：

$$
\tau_{motor,i}
=
\frac{\tau_{link,i}}{gear_i}
$$

理想减速器近似满足：

$$
\tau_{link,i}
\approx
gear_i\tau_{motor,i}
$$

所以若某个关节需要：

$$
\tau_{link}=160\ \text{Nm}
$$

且：

$$
gear=16
$$

则电机侧输出约为：

$$
\tau_{motor}=10\ \text{Nm}
$$

当前源码约定：

```text
motor_tor_out_link  = tauDes
motor_tor_out_motor = tauDes / gear
motors_tor_out      = motor_tor_out_motor
```

所以最终写入 MuJoCo 的是：

$$
\tau_{motor}
$$

## 16. `sign()` / `enablePV()` / `disablePV()`

`sign()` 用于保留方向、限制幅值：

$$
\operatorname{sign}(x)
=
\begin{cases}
1, & x\ge0\\
-1, & x<0
\end{cases}
$$

它用于：

```text
位置步长限制
力矩限幅
```

`PV_enable` 是 PD 反馈项开关：

$$
PV_i=1
$$

表示启用：

$$
K_{p,i}(q_{des,i}-q_i)
+
K_{d,i}(\dot q_{des,i}-\dot q_i)
$$

若：

$$
PV_i=0
$$

则 PVT 近似退化为：

$$
\tau_{link,i}\approx\tau_{ff,i}
$$

也就是只使用 WBC 前馈力矩。

## 17. 写入 MuJoCo

主循环最后：

```text
mj_interface.setMotorsTorque(RobotState.motors_tor_out)
```

`MJ_Interface::setMotorsTorque()` 执行：

$$
\text{mj\_data->ctrl}[i]
=
\text{motors\_tor\_out}[i]
$$

而：

$$
\text{motors\_tor\_out}
=
\tau_{motor}
$$

所以最终：

$$
\text{mj\_data->ctrl}
=
\tau_{motor}
$$

这一步完成：

$$
\text{controller output}
\rightarrow
\text{simulator input}
$$

## 18. 维度和索引注意点

位置命令从完整构型中取关节部分：

$$
q_{j,des}
=
\begin{bmatrix}
0_{n_j\times7} & I_{n_j}
\end{bmatrix}
q_{des}
$$

速度命令源码写的是：

```text
motors_vel_des = wbc_dq_final
```

但 `wbc_dq_final` 在 WBC 内部来自 `dq_final_kin`，源码初始化为 `model_nv` 维。更严格地说，关节速度选择应类似：

$$
\dot q_{j,des}
=
\begin{bmatrix}
0_{n_j\times6} & I_{n_j}
\end{bmatrix}
\dot q_{\text{wbc}}
$$

也就是取：

```text
wbc_dq_final[6:]
```

当前源码没有显式 `block(6,...)`，这可以作为 R9 总复盘时的维度/索引一致性检查点。

另一个关键点是 `motorName` 顺序必须和 MuJoCo / DataBus 的关节顺序一致，否则会出现参数、反馈和输出力矩错位。

## 19. R8 最终结论

R8 可以压缩成一句话：

```text
PVT_Ctr 不重新求 WBC，而是把 WBC 的关节位置、速度和前馈力矩，
与当前关节状态做 PD 反馈融合，经过滤波、限幅和 gear 换算，
得到最终写入 MuJoCo ctrl 的电机侧力矩。
```

对应数学链路是：

$$
\Delta q_{\text{wbc}},\ \dot q_{\text{wbc}},\ \tau_{\text{ff}}
\rightarrow
q_{des},\ \dot q_{des},\ \tau_{\text{ff}}
\rightarrow
\tau_{link}
\rightarrow
\tau_{motor}
\rightarrow
\text{mj\_data->ctrl}
$$

其中最关键的控制律是：

$$
\tau_{motor,i}
=
\frac{
\operatorname{sat}
\left(
LPF
\left[
K_{p,i}(q_{des,i}-q_i)
+
K_{d,i}(\dot q_{des,i}-\dot q_i)
\right]
+
\tau_{ff,i}
\right)
}{gear_i}
$$

因此 R8 已完成：

```text
WBC output
-> PVT joint command
-> link-side torque
-> motor-side torque
-> mj_data->ctrl
```

下一轮进入 R9：闭环总复盘。
