# walk_wbc 第四轮阅读笔记

## 1. 本轮目标

第四轮只回答一个问题：

```text
RobotState.q / dq / base_rot
进入 Pin_KinDyn 之后，
是怎样变成 Jacobian、dJ、末端位姿和动力学项的？
```

本轮重点是：

- `Pin_KinDyn::dataBusRead(const DataBus &robotState)`
- `Pin_KinDyn::computeJ_dJ()`
- `Pin_KinDyn::computeDyn()`
- `Pin_KinDyn::dataBusWrite(DataBus &robotState)`

## 2. 本轮边界

本轮只读：

- `external/open_source_repos/OpenLoong-Dyn-Control/algorithm/pino_kin_dyn.h`
- `external/open_source_repos/OpenLoong-Dyn-Control/algorithm/pino_kin_dyn.cpp`
- `walk_wbc.cpp` 中 `Pin_KinDyn` 的调用位置

暂时不展开：

- `GaitScheduler`
- `FootPlacement`
- `WBC_priority`
- `PVT_Ctr`

## 3. 本轮主链

`walk_wbc.cpp` 主循环里的相关调用是：

```cpp
kinDynSolver.dataBusRead(RobotState);
kinDynSolver.computeJ_dJ();
kinDynSolver.computeDyn();
kinDynSolver.dataBusWrite(RobotState);
```

这一段在主流程中的位置是：

```text
MuJoCo 原始状态
-> StateEst 修正 base 状态
-> Pin_KinDyn 计算运动学 / 动力学对象
-> WBC / gait / foot placement 继续读取这些结果
```

所以第四轮的定位很明确：

```text
Pin_KinDyn 不是决定“怎么走”，
而是负责把当前机器人状态转换成控制器需要的模型量。
```

## 4. 第四轮输入与输出

### 4.1 输入

第四轮最核心的输入是：

- `RobotState.q`
- `RobotState.dq`
- `RobotState.base_rot`

其中：

- `q`：floating-base 广义位置
- `dq`：floating-base 广义速度
- `base_rot`：base 旋转矩阵

### 4.2 输出

第四轮最核心的输出是：

- `J_l / J_r / J_base / J_hd_l / J_hd_r / J_hip_link`
- `dJ_l / dJ_r / dJ_base / dJ_hd_l / dJ_hd_r`
- `fe_* / hd_* / base / hip_link` 的世界系位姿
- `fe_*_pos_L / fe_*_vel_L / hip_*_pos_L / hd_*_pos_L`
- `dyn_M / dyn_M_inv / dyn_C / dyn_G / dyn_Non`
- `dyn_Ag / dyn_dAg`
- `CoM_pos / Jcom`

## 5. dataBusRead()：把工程总线状态改写成 Pinocchio 可用状态

对应实现：

```cpp
q = robotState.q;
dq = robotState.dq;
dq.block(0, 0, 3, 1) = robotState.base_rot.transpose() * dq.block(0, 0, 3, 1);
dq.block(3, 0, 3, 1) = robotState.base_rot.transpose() * dq.block(3, 0, 3, 1);
ddq = robotState.ddq;
```

### 5.1 为什么 `q` 直接抄，`dq` 还要变换

原因是 Pinocchio 对 floating-base 的约定是：

```text
q = [world中的 base 位置, world中的 base 姿态, 关节角]
v = [body中的 base 线速度, body中的 base 角速度, 关节速度]
```

而工程里的 `RobotState.dq` 上游通常按 world 风格理解。

所以 `dataBusRead()` 里必须做：

```text
world base velocity -> body base velocity
```

用的就是：

```cpp
base_rot.transpose()
```

因为：

```text
base_rot: body -> world
base_rot.transpose(): world -> body
```

### 5.2 本轮结论

`dataBusRead()` 不是在计算 Jacobian，而是在做一层状态表达转换：

```text
RobotState 状态
-> Pinocchio floating-base 约定下的 q / dq / ddq
```

## 6. computeJ_dJ()：运动学主计算函数

`computeJ_dJ()` 的职责是：

```text
根据当前 q / dq，
计算脚、手、base、hip_link 的 Jacobian、dJ、位姿、
并补出 body frame 下的末端几何量和脚线速度。
```

### 6.1 第 1 段：刷新 Pinocchio 当前时刻的运动学缓存

```cpp
pinocchio::forwardKinematics(model_biped, data_biped, q);
pinocchio::jacobianCenterOfMass(model_biped, data_biped, q, true);
pinocchio::computeJointJacobiansTimeVariation(model_biped, data_biped, q, dq);
pinocchio::updateGlobalPlacements(model_biped, data_biped);
```

这四句分别做：

- `forwardKinematics`：更新当前位姿
- `jacobianCenterOfMass`：准备 `Jcom`
- `computeJointJacobiansTimeVariation`：准备 `data.J` 与 `data.dJ`
- `updateGlobalPlacements`：更新可直接读取的世界位姿 `oMi[...]`

这里不是“只算一次 forwardKinematics”，而是连续调用几种运动学算法，共同刷新 `data_biped` 缓存。

### 6.2 第 2 段：提取主要 Jacobian

```cpp
getJointJacobian(... r_ankle_joint ..., J_r);
getJointJacobian(... l_ankle_joint ..., J_l);
getJointJacobian(... r_hand_joint ..., J_hd_r);
getJointJacobian(... l_hand_joint ..., J_hd_l);
getJointJacobian(... base_joint ..., J_base);
getJointJacobian(... waist_yaw_joint ..., J_hip_link);
```

这里提取的是：

- 双脚 Jacobian
- 双手 Jacobian
- base Jacobian
- hip_link Jacobian

参考系使用：

```cpp
pinocchio::LOCAL_WORLD_ALIGNED
```

可先理解为：

```text
参考点在该 joint，
但表达方向与世界坐标轴对齐
```

### 6.3 第 3 段：提取 Jacobian 时间导数

```cpp
getJointJacobianTimeVariation(... r_ankle_joint ..., dJ_r);
getJointJacobianTimeVariation(... l_ankle_joint ..., dJ_l);
getJointJacobianTimeVariation(... r_hand_joint ..., dJ_hd_r);
getJointJacobianTimeVariation(... l_hand_joint ..., dJ_hd_l);
getJointJacobianTimeVariation(... base_joint ..., dJ_base);
```

这些量后面常用于：

```text
末端加速度 = J ddq + dJ dq
```

Pinocchio 这里不是差分算 `dJ`，而是：

1. 先用 `computeJointJacobiansTimeVariation(...)` 解析计算全模型 `data.dJ`
2. 再用 `getJointJacobianTimeVariation(...)` 提取目标 joint 的 `dJ`

### 6.4 第 4 段：读取世界系下的脚 / 手 / base / hip_link 位姿

```cpp
fe_l_pos = data_biped.oMi[l_ankle_joint].translation();
fe_l_rot = data_biped.oMi[l_ankle_joint].rotation();
...
base_pos = data_biped.oMi[base_joint].translation();
base_rot = data_biped.oMi[base_joint].rotation();
...
hip_link_pos = data_biped.oMi[waist_yaw_joint].translation();
hip_link_rot = data_biped.oMi[waist_yaw_joint].rotation();
Jcom = data_biped.Jcom;
```

这里是在“读结果”，不是重新计算。

`data_biped.oMi[joint]` 可以理解成：

```text
joint 当前在世界系下的位姿
```

因此得到：

- 双脚世界位姿
- 双手世界位姿
- base 世界位姿
- `waist_yaw_joint` 代表的 `hip_link` 位姿
- `Jcom`

### 6.5 第 5 段：把 Jacobian 改写到工程统一使用的速度表达

```cpp
Mpj = Identity;
Mpj.block(0, 0, 3, 3) = base_rot.transpose();
Mpj.block(3, 3, 3, 3) = base_rot.transpose();

J_l = J_l * Mpj;
J_r = J_r * Mpj;
J_base = J_base * Mpj;
dJ_l = dJ_l * Mpj;
dJ_r = dJ_r * Mpj;
...
Jcom = Jcom * Mpj;
```

这段的背景是：

- Pinocchio 内部使用的是 `dq_local`
- 工程后续更习惯按 `RobotState.dq` 的 world 风格来使用

如果：

```text
dq_local = Mpj * dq_world
```

那么速度关系：

```text
v = J_pin * dq_local
```

就可以改写为：

```text
v = (J_pin * Mpj) * dq_world
```

所以这段本质是：

```text
把 Jacobian 的输入变量定义，
从 Pinocchio 内部的 dq_local
改写成工程后续使用的 dq_world_style
```

### 6.6 第 6 段：为什么还要单独算 fixed-base

后半段代码：

```cpp
q_fixed = q.block(7, 0, model_biped_fixed.nv, 1);
dq_fixed = dq.block(6, 0, model_biped_fixed.nv, 1);
...
fe_l_pos_body = ...
fe_r_pos_body = ...
...
fe_l_vel_body = (J_l_body * dq_fixed).block(0, 0, 3, 1);
fe_r_vel_body = (J_r_body * dq_fixed).block(0, 0, 3, 1);
```

这里换成 `model_biped_fixed` 的原因是：

```text
前半段要 world 量，
后半段要 body / baselink 相对量。
```

前半段回答：

- 机器人在世界里脚、手、base 在哪
- 世界系 Jacobian 是什么

后半段回答：

- 脚相对 body 在哪
- 手相对 body 在哪
- 脚相对 body 的线速度多大

所以：

- `floating-base`：适合算世界中的全身运动
- `fixed-base`：适合算相对 body 的内部几何关系

### 6.7 第 7 段：绝对运动与相对运动的作用

#### 绝对运动 / 世界量

典型量：

- `fe_l_pos_W / fe_r_pos_W`
- `hip_l_pos_W / hip_r_pos_W`
- `base_pos`
- `J_l / J_r / Jcom_W`

主要用途：

- `GaitScheduler` 记录支撑脚、摆脚起点
- `FootPlacement` 规划世界系摆脚落点
- `WBC_priority` 读取世界系脚位姿和 CoM 任务量

#### 相对运动 / body 量

典型量：

- `fe_l_pos_L / fe_r_pos_L`
- `fe_l_vel_L / fe_r_vel_L`
- `hip_l_pos_L / hip_r_pos_L`

主要用途：

- `StateEst` 用脚相对 body 的位置、速度做估计测量
- IK / 局部姿态目标表达更自然

一句话总结：

```text
绝对量服务于“机器人在世界里怎么运动”，
相对量服务于“肢体相对身体怎么运动”。
```

## 7. computeDyn()：动力学主计算函数

`computeDyn()` 的目标是围绕动力学主方程：

```text
M(q) ddq + C(q,dq) dq + G(q) = tau + J^T F
```

计算：

- `dyn_M`
- `dyn_M_inv`
- `dyn_C`
- `dyn_G`
- `dyn_Non`
- `dyn_Ag`
- `dyn_dAg`
- `CoM_pos`
- `inertia`

### 7.1 质量矩阵 `dyn_M`

```cpp
pinocchio::crba(model_biped, data_biped, q);
dyn_M = data_biped.M;
```

Pinocchio 用的是：

- `CRBA = Composite Rigid Body Algorithm`

它根据当前 `q` 解析计算：

```text
M(q)
```

动能写成：

```text
T = 1/2 dq^T M(q) dq
```

因此 `M(q)` 就是动能二次型的系数矩阵。

### 7.2 逆质量矩阵 `dyn_M_inv`

```cpp
pinocchio::computeMinverse(model_biped, data_biped, q);
dyn_M_inv = data_biped.Minv;
```

Pinocchio 用的是：

- `Articulated Body formulation`

目标是直接计算：

```text
M(q)^{-1}
```

不是先算 `M` 再做普通矩阵求逆。

### 7.3 科氏/离心矩阵 `dyn_C`

```cpp
pinocchio::computeCoriolisMatrix(model_biped, data_biped, q, dq);
dyn_C = data_biped.C;
```

这里计算的是：

```text
C(q,dq)
```

真正进入动力学方程的速度项是：

```text
C(q,dq) dq
```

Pinocchio 内部通过树形前向+后向递推解析构造 `C`。

### 7.4 重力项 `dyn_G`

```cpp
pinocchio::computeGeneralizedGravity(model_biped, data_biped, q);
dyn_G = data_biped.g;
```

这里计算的是：

```text
G(q)
```

Pinocchio 注释明确指出：

```text
computeGeneralizedGravity(q)
等价于
rnea(q, v=0, a=0)
```

也就是 RNEA 的一个特例。

### 7.5 质心动量矩阵 `dyn_Ag` 和导数 `dyn_dAg`

```cpp
pinocchio::dccrba(model_biped, data_biped, q, dq);
pinocchio::computeCentroidalMomentum(model_biped, data_biped, q, dq);
dyn_Ag = data_biped.Ag;
dyn_dAg = data_biped.dAg;
```

质心动量满足：

```text
h = Ag(q) dq
```

对时间求导：

```text
dh/dt = Ag(q) ddq + dAg(q,dq) dq
```

这里的：

- `Ag`：质心动量矩阵
- `dAg`：它的时间导数

Pinocchio 在 `dccrba(...)` 中会一并准备：

- `data.J`
- `data.dJ`
- `data.Ag`
- `data.dAg`

### 7.6 非线性项 `dyn_Non`

```cpp
dyn_Non = dyn_C * dq + dyn_G;
```

这是工程里自己拼出来的，不是 Pinocchio 单独返回的函数。

定义为：

```text
Non(q,dq) = C(q,dq) dq + G(q)
```

于是动力学方程可写成：

```text
M(q) ddq + Non(q,dq) = tau + J^T F
```

### 7.7 整体惯量 `inertia`

```cpp
pinocchio::ccrba(model_biped, data_biped, q, dq);
inertia = data_biped.Ig.inertia().matrix();
```

这里取的是：

```text
整机相对质心的整体转动惯量矩阵
```

它不是某个单杆的惯量，而是整台机器人当前姿态下的整体结果。

### 7.8 质心位置 `CoM_pos`

```cpp
CoM_pos = data_biped.com[0];
```

表示：

```text
p_com(q)
```

即当前机器人质心在世界系中的位置。

### 7.9 动力学项的坐标表达转换

```cpp
Mpj.block(0, 0, 3, 3) = base_rot.transpose();
Mpj.block(3, 3, 3, 3) = base_rot.transpose();
Mpj_inv.block(0, 0, 3, 3) = base_rot;
Mpj_inv.block(3, 3, 3, 3) = base_rot;

dyn_M = Mpj_inv * dyn_M * Mpj;
dyn_M_inv = Mpj_inv * dyn_M_inv * Mpj;
dyn_C = Mpj_inv * dyn_C * Mpj;
dyn_G = Mpj_inv * dyn_G;
dyn_Non = Mpj_inv * dyn_Non;
```

这段和 `computeJ_dJ()` 里的 `Mpj` 逻辑相同：

- Pinocchio 内部 base 速度是 local/body 表达
- 工程后续更习惯按 `RobotState.dq` 的 world 风格使用

因此要把动力学对象也改写到统一速度定义下。

## 8. dataBusWrite()：把 `Pin_KinDyn` 内部结果发布回 `RobotState`

`dataBusWrite()` 不是再做计算，而是把已经算好的结果写回总线。

按本轮关心内容，它主要写回：

- `J_l / J_r / J_base / J_hd_l / J_hd_r / J_hip_link`
- `dJ_l / dJ_r / dJ_base / dJ_hd_l / dJ_hd_r`
- `fe_*_pos_W / fe_*_rot_W`
- `fe_*_pos_L / fe_*_rot_L / fe_*_vel_L`
- `hip_*_pos_W / hip_*_pos_L`
- `hd_*_pos_W / hd_*_rot_W`
- `hd_*_pos_L / hd_*_rot_L`
- `hip_link_pos / hip_link_rot`
- `dyn_M / dyn_M_inv / dyn_C / dyn_G / dyn_Ag / dyn_dAg / dyn_Non`
- `pCoM_W / Jcom_W`

一句话总结：

```text
Pin_KinDyn 内部缓存
-> DataBus 公共总线
```

## 9. 第四轮总收口

第四轮的核心不是“会用 Pinocchio API”，而是搞清楚 `Pin_KinDyn` 在 `walk_wbc` 里的职责：

```text
StateEst 输出的 q / dq / base_rot
-> Pin_KinDyn 计算 Jacobian、dJ、world/body 位姿、M/C/G/CoM 等模型量
-> 结果写回 RobotState
-> WBC / gait / foot placement 再继续读取
```

因此，本轮最关键的结论可以压成两句：

1. `computeJ_dJ()` 负责把当前状态变成控制器需要的运动学描述。
2. `computeDyn()` 负责把当前状态变成控制器需要的动力学描述。

## 10. 输入 / 中间对象 / 输出 / 下游总表

这张表用于后续复习。读第五轮之前，先确认自己能沿着这一列从左到右说清楚。

| 输入字段 | 读取位置 | 中间计算 | 写回字段 | 下游使用者 |
| --- | --- | --- | --- | --- |
| `RobotState.q` | `dataBusRead()` | `forwardKinematics(model_biped, data_biped, q)` | `fe_l_pos_W`、`fe_r_pos_W`、`base_pos`、`hip_link_pos`、手部位姿 | `StateEst`、`GaitScheduler`、`FootPlacement`、`WBC_priority` |
| `RobotState.dq` | `dataBusRead()` | base 前 6 维先通过 `base_rot.transpose()` 从 world 表达转成 Pinocchio local/body 表达 | `dq` 内部缓存 | `computeJ_dJ()`、`computeDyn()` |
| `RobotState.base_rot` | `dataBusRead()` | 坐标变换矩阵 `Mpj` / `Mpj_inv` | 变换后的 `J`、`dJ`、`dyn_*` | `WBC_priority` |
| floating-base model | 构造函数 | `computeJointJacobiansTimeVariation()`、`getJointJacobian()` | `J_l`、`J_r`、`J_base`、`J_hd_l`、`J_hd_r`、`J_hip_link` | `WBC_priority::computeDdq()`、`computeTau()` |
| floating-base model | `computeJ_dJ()` | `getJointJacobianTimeVariation()` | `dJ_l`、`dJ_r`、`dJ_base`、`dJ_hd_l`、`dJ_hd_r` | 任务空间加速度项 `J ddq + dJ dq` |
| floating-base model | `computeJ_dJ()` | `jacobianCenterOfMass()` | `Jcom_W` | CoM / momentum 相关任务或约束 |
| fixed-base model | `computeJ_dJ()` | 只取 `q[7:]`、`dq[6:]` 做固定基 FK/Jacobian | `fe_l_pos_L`、`fe_r_pos_L`、`fe_l_vel_L`、`fe_r_vel_L` | `StateEst`、局部几何目标、后续落脚点模块 |
| `q / dq` | `computeDyn()` | `crba()` | `dyn_M` | WBC 动力学等式 |
| `q` | `computeDyn()` | `computeMinverse()` | `dyn_M_inv` | WBC / QP 中的质量矩阵逆相关计算 |
| `q / dq` | `computeDyn()` | `computeCoriolisMatrix()` | `dyn_C` | 非线性项构造 |
| `q` | `computeDyn()` | `computeGeneralizedGravity()` | `dyn_G` | 非线性项构造 |
| `dyn_C / dq / dyn_G` | `computeDyn()` | `dyn_C * dq + dyn_G` | `dyn_Non` | WBC 动力学方程右侧补偿 |
| `q / dq` | `computeDyn()` | `dccrba()`、`computeCentroidalMomentum()`、`ccrba()` | `dyn_Ag`、`dyn_dAg`、`inertia`、`pCoM_W` | 质心 / 角动量 / 惯量相关任务 |

最简链条：

```text
q / dq / base_rot
-> Pinocchio FK / Jacobian / dynamics
-> J / dJ / pose / dyn_*
-> RobotState
-> gait / foot placement / WBC
```

## 11. 坐标系注意事项

第四轮最容易忘的是坐标表达，而不是 API 名字。

### 11.1 `q` 和 `dq` 的表达不同

`q` 可以直接从 `RobotState.q` 复制：

```text
q = [world base position, world base quaternion, joint positions]
```

但是 Pinocchio 的 floating-base 速度约定更接近：

```text
v = [local/body base linear velocity, local/body base angular velocity, joint velocities]
```

所以 `dataBusRead()` 里有：

```cpp
dq.block(0, 0, 3, 1) = robotState.base_rot.transpose() * dq.block(0, 0, 3, 1);
dq.block(3, 0, 3, 1) = robotState.base_rot.transpose() * dq.block(3, 0, 3, 1);
```

读法：

```text
RobotState 中的 base velocity
-> world 表达
-> 转成 Pinocchio 需要的 body/local 表达
```

### 11.2 Jacobian 和动力学项又被改写回工程表达

`computeJ_dJ()` 里：

```cpp
J_l = J_l * Mpj;
J_r = J_r * Mpj;
J_base = J_base * Mpj;
Jcom = Jcom * Mpj;
```

`computeDyn()` 里：

```cpp
dyn_M = Mpj_inv * dyn_M * Mpj;
dyn_C = Mpj_inv * dyn_C * Mpj;
dyn_G = Mpj_inv * dyn_G;
dyn_Non = Mpj_inv * dyn_Non;
```

读法：

```text
Pinocchio 内部使用 local/body base velocity 计算；
OpenLoong 后续 WBC 更希望用工程统一的 base 表达；
所以输出 Jacobian 和 dynamics 前又做一次 base block 坐标变换。
```

### 11.3 后续读 WBC 时要保留这个风险点

后续进入 `WBC_priority` 时，凡是看到这些式子都要回想第四轮：

```text
J * dq
J * ddq + dJ * dq
dyn_M * ddq + dyn_Non
J_c^T * F
```

需要确认：

```text
这里使用的 dq / ddq / force / Jacobian 是否处在同一套表达约定下？
```

这不是当前要解决的 bug，而是读 WBC 时必须持续检查的坐标系风险。

## 12. R4 -> R5 接口

第四轮结束时，`RobotState` 中已经有了下一轮需要的模型量。

### 12.1 R5 会用到的世界系量

这些量更可能给 `GaitScheduler`、`FootPlacement` 和 `WBC_priority` 使用：

- `fe_l_pos_W`
- `fe_r_pos_W`
- `fe_l_rot_W`
- `fe_r_rot_W`
- `hip_l_pos_W`
- `hip_r_pos_W`
- `hip_link_pos`
- `base_pos`
- `pCoM_W`
- `J_l`
- `J_r`
- `J_base`
- `Jcom_W`

### 12.2 R5 会用到的 body/local 量

这些量更可能给 `StateEst`、局部落脚几何和相对目标表达使用：

- `fe_l_pos_L`
- `fe_r_pos_L`
- `fe_l_rot_L`
- `fe_r_rot_L`
- `fe_l_vel_L`
- `fe_r_vel_L`
- `hip_l_pos_L`
- `hip_r_pos_L`

### 12.3 WBC 会用到的动力学量

这些量会在后续读 `WBC_priority` 时重点出现：

- `dyn_M`
- `dyn_M_inv`
- `dyn_C`
- `dyn_G`
- `dyn_Non`
- `dyn_Ag`
- `dyn_dAg`
- `inertia`

### 12.4 第五轮入口

R5 不再问“模型量怎么算”，而是问：

```text
当前模型量已经在 RobotState 里了，
JoyStickInterpreter / GaitScheduler / FootPlacement
如何把期望速度变成 motionState、legState、接触状态和摆动脚目标？
```

第五轮建议阅读顺序：

```text
walk_wbc.cpp 中 jsInterp / gaitScheduler / footPlacement 调用块
-> joystick_interpreter.cpp
-> gait_scheduler.cpp
-> foot_placement.cpp
```

## 13. 第四轮完成标准

第四轮可以结束，当你能独立说出下面这段话：

```text
Pin_KinDyn 接收 StateEst 修正后的 floating-base q/dq。
进入 Pinocchio 前，base velocity 会从 world 表达转成 local/body 表达。
computeJ_dJ() 计算脚、手、base、hip_link、CoM 的 Jacobian、dJ 和位姿；
fixed-base model 额外补出 body frame 下的脚/手/髋几何量。
computeDyn() 计算 M、M^{-1}、C、G、Non、Ag、dAg、CoM 和惯量。
随后 dataBusWrite() 把这些模型量发布回 RobotState，
供 StateEst 接触力更新、步态/落脚点规划和 WBC 使用。
```

按这个标准，第四轮已经完成，可以进入第五轮。
