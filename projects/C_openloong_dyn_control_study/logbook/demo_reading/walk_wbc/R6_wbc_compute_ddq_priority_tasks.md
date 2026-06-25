# walk_wbc 第六轮阅读笔记

## 1. 本轮目标

第六轮回答一个问题：

$$
\text{walking / standing 任务目标如何变成全身的 } \Delta q,\ \dot q,\ \ddot q\text{？}
$$

本轮关注的是 WBC 的运动学任务求解层：

- `WBC_priority::dataBusRead()`
- `WBC_priority::computeDdq()`
- `PriorityTasks::computeAll()`

暂时不展开：

- `WBC_priority::computeTau()`
- 接触力 QP
- 摩擦锥
- 关节力矩求解

这些放到 R7。

## 2. 本轮主链位置

在 `walk_wbc.cpp` 主循环中，WBC 调用顺序是：

```cpp
WBC_solv.dataBusRead(RobotState);
WBC_solv.computeDdq(kinDynSolver);
WBC_solv.computeTau();
WBC_solv.dataBusWrite(RobotState);
```

第六轮只读前两步中的运动学部分：

$$
\text{DataBus}
\rightarrow
\text{WBC task}
\rightarrow
\Delta q_{\text{final}},\ \dot q_{\text{final}},\ \ddot q_{\text{final}}
$$

`computeDdq()` 不是直接求力矩，而是先求全身应该怎么动。

## 3. dataBusRead：WBC 读取哪些输入

`WBC_priority::dataBusRead()` 把 `RobotState` 中的数据分成三类读进 WBC。

第一类：R5 生成的任务目标。

- `base_pos_des`
- `base_rpy_des`
- `swing_fe_pos_des_W`
- `swing_fe_rpy_des_W`
- `stance_fe_pos_cur_W`
- `stanceDesPos_W`
- `legState`
- `motionState`

第二类：R4 生成的运动学 / 动力学量。

- `J_base / dJ_base`
- `J_l / J_r`
- `dJ_l / dJ_r`
- `Jcom_W`
- `dyn_M / dyn_M_inv / dyn_Non`
- `q / dq`

第三类：demo 给 WBC 的高层 base 参考。

- `des_delta_q`
- `des_dq`
- `des_ddq`

其中最关键的是根据 `legState` 选择支撑脚和摆动脚。

如果当前是 `LSt`：

$$
J_c = J_l,\qquad J_{sw}=J_r
$$

表示左脚支撑、右脚摆动。

如果当前是 `RSt`：

$$
J_c = J_r,\qquad J_{sw}=J_l
$$

表示右脚支撑、左脚摆动。

所以 R5 的 `GaitScheduler` 决定“哪只脚支撑”，R6 的 WBC 根据这个状态选择：

$$
J_c = \text{contact Jacobian}
$$

$$
J_{sw} = \text{swing foot Jacobian}
$$

## 4. computeDdq 的核心数学

每个 WBC 任务都可以抽象成任务空间变量：

$$
x = f(q)
$$

任务速度满足：

$$
\dot{x} = J(q)\dot{q}
$$

任务加速度满足：

$$
\ddot{x} = J(q)\ddot{q} + \dot{J}(q,\dot q)\dot{q}
$$

每个任务先构造一个期望任务加速度：

$$
\ddot{x}_{cmd}
=
\ddot{x}_{des}
+K_p e
+K_d \dot e
$$

对应代码字段是：

- `errX`：任务误差 \(e\)
- `derrX`：任务速度误差 \(\dot e\)
- `ddxDes`：任务期望加速度 \(\ddot{x}_{des}\)
- `kp`：比例增益 \(K_p\)
- `kd`：阻尼增益 \(K_d\)
- `J`：任务 Jacobian
- `dJ`：任务 Jacobian 的时间导数

希望任务满足：

$$
\ddot{x} = \ddot{x}_{cmd}
$$

代入任务加速度关系：

$$
J\ddot{q}+\dot{J}\dot{q}
=
\ddot{x}_{cmd}
$$

整理得到加速度层 WBC 的基本方程：

$$
J\ddot{q}
=
\ddot{x}_{cmd}-\dot{J}\dot{q}
$$

如果只有一个任务，可以用伪逆求：

$$
\ddot{q}
=
J^\#\left(\ddot{x}_{cmd}-\dot{J}\dot{q}\right)
$$

当前代码使用的是动力学加权伪逆：

$$
J^\#_{\text{dyn}}
=
\text{dyn\_pseudoInv}(J,\ M^{-1})
$$

## 5. Walk 模式任务链

`computeDdq()` 里 walking 模式实际启用的优先级链是：

$$
\text{static\_Contact}
\rightarrow
\text{PosRot}
\rightarrow
\text{SwingLeg}
\rightarrow
\text{RedundantJoints}
\rightarrow
\text{HandTrackJoints}
$$

源码中也注册并填写了 `Roll_Pitch_Yaw_Pz` 和 `PxPy`，但它们没有放进 `taskOrder_walk`，所以当前不会参与最终 `computeAll()`。

### 5.1 static_Contact：支撑脚静止

walking 中第一优先级是支撑脚接触约束。

支撑脚速度：

$$
v_c = J_c\dot q
$$

支撑脚加速度：

$$
a_c = J_c\ddot q+\dot J_c\dot q
$$

支撑脚静止要求：

$$
a_c = 0
$$

所以约束写成：

$$
J_c\ddot q+\dot J_c\dot q=0
$$

整理为：

$$
J_c\ddot q=-\dot J_c\dot q
$$

这就是 `static_Contact` 中 `errX = 0`、`ddxDes = 0`、`kp = 0`、`kd = 0`，但 `J = Jc`、`dJ = dJc` 的原因。

它不是普通的位置跟踪任务，而是最高优先级的接触约束。

### 5.2 PosRot：base 位置和姿态

`PosRot` 控制 base 的 6D 位姿：

$$
x_{base}
=
\begin{bmatrix}
p_{base}\\
R_{base}
\end{bmatrix}
$$

位置误差：

$$
e_p = p_{base}^{des}-p_{base}^{cur}
$$

代码中：

$$
p_{base}^{cur}=q(0:2)
$$

姿态误差不是直接用欧拉角相减，而是：

$$
R_{err}=R_{cur}^{T}R_{des}
$$

再将相对旋转转成轴角向量：

$$
e_R=\theta u
$$

代码中由 `diffRot(base_rot, desRot)` 完成。

完整误差为：

$$
e_{PosRot}
=
\begin{bmatrix}
e_p\\
e_R
\end{bmatrix}
$$

任务加速度关系：

$$
\ddot{x}_{base}
=
J_{base}\ddot q+\dot J_{base}\dot q
$$

所以 `PosRot` 给 WBC 的方程是：

$$
J_{base}\ddot q
=
\ddot{x}_{base,cmd}-\dot J_{base}\dot q
$$

它在 walking 中排第二，意思是：

$$
\text{先保证支撑脚不动，再控制身体 base。}
$$

### 5.3 SwingLeg：摆动脚跟踪

`SwingLeg` 是 R5 和 R6 的关键接口。

R5 `FootPlacement` 写入：

$$
p_{sw}^{des}=\text{swing\_fe\_pos\_des\_W}
$$

$$
rpy_{sw}^{des}=\text{swing\_fe\_rpy\_des\_W}
$$

R6 读取后构造摆动脚位置误差：

$$
e_{p,sw}
=
p_{sw}^{des}-p_{sw}^{cur}
$$

摆动脚姿态误差：

$$
e_{R,sw}
=
\text{diffRot}(R_{sw}^{cur},R_{sw}^{des})
$$

完整误差：

$$
e_{sw}
=
\begin{bmatrix}
e_{p,sw}\\
e_{R,sw}
\end{bmatrix}
$$

摆动脚加速度关系：

$$
\ddot{x}_{sw}
=
J_{sw}\ddot q+\dot J_{sw}\dot q
$$

所以任务方程是：

$$
J_{sw}\ddot q
=
\ddot{x}_{sw,cmd}-\dot J_{sw}\dot q
$$

其中 `Jsw` 由 `legState` 选择：

$$
\text{LSt}: J_{sw}=J_r
$$

$$
\text{RSt}: J_{sw}=J_l
$$

源码还把腰部三列从摆动脚任务中排除：

$$
J_{sw}(:,22:24)=0
$$

其意图是让摆动脚跟踪主要由腿部完成，而不是通过腰部关节代偿。

### 5.4 RedundantJoints：冗余关节回零

该任务把若干冗余关节拉向 0。

目标：

$$
q_{red}^{des}=0
$$

误差：

$$
e_{red}=q_{red}^{des}-q_{red}=-q_{red}
$$

它使用的 `J` 不是几何 Jacobian，而是选择矩阵：

$$
\ddot q_{red}=J_{red}\ddot q
$$

该任务低于 `SwingLeg`，所以只能在不破坏支撑脚、base、摆动脚任务的前提下整理姿态。

### 5.5 HandTrackJoints：手臂关节跟踪

手臂任务构造 14 维目标：

$$
q_{arm}^{des}\in \mathbb{R}^{14}
$$

误差：

$$
e_{arm}=q_{arm}^{des}-q_{arm}^{cur}
$$

Jacobian 也是选择矩阵：

$$
\ddot q_{arm}=J_{arm}\ddot q
$$

其中：

$$
J_{arm}
=
\begin{bmatrix}
0_{14\times 6} & I_{14\times 14} & 0
\end{bmatrix}
$$

它是 walking 中最低优先级任务。

## 6. Stand 模式任务链

standing 模式实际启用的优先级链是：

$$
\text{static\_Contact}
\rightarrow
\text{CoMXY\_HipRPY}
\rightarrow
\text{Pz}
\rightarrow
\text{HandTrackJoints}
\rightarrow
\text{HeadRP}
$$

### 6.1 static_Contact：双脚接触

stand 中 `static_Contact` 是 12 维：

$$
J_{fe}
=
\begin{bmatrix}
J_l\\
J_r
\end{bmatrix}
$$

双脚静止约束：

$$
J_{fe}\ddot q+\dot J_{fe}\dot q=0
$$

### 6.2 CoMXY_HipRPY：质心 xy + 髋部姿态

质心水平位置误差：

$$
e_{com,xy}
=
p_{com,xy}^{des}-p_{com,xy}^{cur}
$$

髋部姿态误差：

$$
e_{hip,R}
=
\text{diffRot}(R_{hip}^{cur},R_{des})
$$

任务误差：

$$
e
=
\begin{bmatrix}
e_{com,xy}\\
e_{hip,R}
\end{bmatrix}
$$

任务 Jacobian：

$$
J
=
\begin{bmatrix}
J_{com,xy}\\
S_{rpy}J_{hip}
\end{bmatrix}
$$

所以 stand 更强调：

$$
\text{双脚固定 + 质心水平平衡 + 髋部姿态稳定}
$$

### 6.3 Pz：base 高度

高度误差：

$$
e_z=z_{base}^{des}-z_{base}^{cur}
$$

任务 Jacobian 从 `J_base` 中抽取 z 方向：

$$
J_z=S_zJ_{base}
$$

### 6.4 HandTrackJoints / HeadRP

站立时手臂跟踪固定姿态：

$$
e_{arm}=q_{arm}^{des}-q_{arm}^{cur}
$$

头部 roll / pitch 任务用于维持头部姿态：

$$
e_{head}
=
\begin{bmatrix}
0-q_{head,roll}\\
\theta_{base,pitch}-q_{head,pitch}
\end{bmatrix}
$$

## 7. PriorityTasks::computeAll()

`computeDdq()` 前面只是填任务表，真正逐层求解发生在：

```cpp
PriorityTasks::computeAll()
```

### 7.1 第一层任务

最高优先级任务没有父任务，所以：

$$
N_0=I
$$

投影后的 Jacobian：

$$
J_0^{pre}=J_0N_0=J_0
$$

位置增量层：

$$
\Delta q_0
=
\Delta q_{des}
+(J_0^{pre})^\# e_0
$$

加速度层：

$$
\ddot q_0
=
\ddot q_{des}
+(J_0^{pre})_{\text{dyn}}^\#
\left(
\ddot x_{cmd,0}-\dot J_0\dot q
\right)
$$

第一层可以使用所有自由度，因为没有更高优先级任务需要避让。

### 7.2 低优先级任务

第 \(i\) 层任务必须在上一层任务的零空间中求解。

上一层的零空间投影：

$$
N_i
=
N_{i-1}
\left(
I-(J_{i-1}^{pre})^\#J_{i-1}^{pre}
\right)
$$

如果只看第一层任务之后的零空间，可以写成：

$$
N_1 = I-J_0^\#J_0
$$

如果按“第 1 个任务、下一层是第 2 个任务”的编号习惯，也常写成：

$$
N_1 = I-J_1^\#J_1
$$

核心意思不变：后续任务只能使用高优先级任务的零空间。

当前任务在剩余自由度中的有效 Jacobian 是：

$$
J_i^{pre}=J_iN_i
$$

位置增量递推：

$$
\Delta q_i
=
\Delta q_{i-1}
+(J_i^{pre})^\#
\left(
e_i-J_i\Delta q_{i-1}
\right)
$$

速度递推：

$$
\dot q_i
=
\dot q_{i-1}
+(J_i^{pre})^\#
\left(
\dot x_i^{des}-J_i\dot q_{i-1}
\right)
$$

加速度递推：

$$
\ddot q_i
=
\ddot q_{i-1}
+(J_i^{pre})_{\text{dyn}}^\#
\left(
\ddot x_{cmd,i}
-\dot J_i\dot q
-J_i\ddot q_{i-1}
\right)
$$

这就是“逐层求解”：

$$
\text{当前层解}
=
\text{上一层解}
+
\text{不破坏上一层任务的零空间修正}
$$

### 7.3 最终输出

循环沿着 `childId` 走到最后一个任务。

最后一个任务的结果已经包含前面所有高优先级任务的解，所以：

$$
\Delta q_{\text{out}}=\Delta q_{\text{last}}
$$

$$
\dot q_{\text{out}}=\dot q_{\text{last}}
$$

$$
\ddot q_{\text{out}}=\ddot q_{\text{last}}
$$

回到 `WBC_priority::computeDdq()` 后保存为：

$$
\Delta q_{\text{final,kin}}
,\quad
\dot q_{\text{final,kin}}
,\quad
\ddot q_{\text{final,kin}}
$$

## 8. R6 总结

R6 读完后，完整链路是：

$$
\text{R4: } J,\dot J,M,M^{-1},q,\dot q
$$

$$
\text{R5: } \text{base 目标、摆动脚目标、legState}
$$

$$
\Downarrow
$$

$$
\text{WBC\_priority::dataBusRead()}
$$

$$
\Downarrow
$$

$$
\text{WBC\_priority::computeDdq()}
$$

$$
\Downarrow
$$

$$
\text{PriorityTasks::computeAll()}
$$

$$
\Downarrow
$$

$$
\Delta q_{\text{final,kin}},
\quad
\dot q_{\text{final,kin}},
\quad
\ddot q_{\text{final,kin}}
$$

一句话总结：

$$
\text{R6 完成的是“任务目标到加速度层 WBC 解”的链路。}
$$

下一轮 R7 再进入：

$$
\ddot q_{\text{final,kin}}
\rightarrow
\text{接触力}
\rightarrow
\text{关节力矩}
$$
