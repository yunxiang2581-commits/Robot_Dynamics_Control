# walk_wbc 第二轮阅读笔记

## 1. 本轮目标

- 只读 `MuJoCo state -> MJ_Interface -> DataBus::updateQ()` 这一段。
- 回答一个核心问题：

```text
MuJoCo 的 qpos/qvel/sensordata
到底怎么变成 RobotState.q / RobotState.dq
```

## 2. 本轮边界

先不读这些模块内部：

- `StateEst`
- `Pin_KinDyn`
- `GaitScheduler`
- `WBC_priority`
- `PVT_Ctr`

本轮只读 3 个位置：

1. `demo/walk_wbc.cpp`
2. `sim_interface/MJ_interface.cpp`
3. `common/data_bus.h` 中的 `DataBus::updateQ()`

## 3. 本轮主链

本轮只关心这条链：

```text
MuJoCo state
-> MJ_Interface::updateSensorValues()
-> MJ_Interface::dataBusWrite()
-> DataBus::updateQ()
-> 初始 q / dq
```

## 4. walk_wbc 调用入口

在 `walk_wbc.cpp` 主循环中，相关入口是：

```cpp
mj_step(mj_model, mj_data);
mj_interface.updateSensorValues();
mj_interface.dataBusWrite(RobotState);
```

- `mj_step()`：上一轮 `ctrl` 作用到 MuJoCo，仿真推进一步
- `updateSensorValues()`：从 `mj_data` 读取当前状态，存入 `MJ_Interface` 内部缓存
- `dataBusWrite()`：把内部缓存写到 `RobotState`，并调用 `updateQ()`

关键时序：

```text
上一轮 torque
-> mj_step
-> 当前 MuJoCo state
-> 读取 state
-> 写入 RobotState
```

## 5. MJ_Interface 构造函数中的索引映射

在 `MJ_Interface` 构造函数里，最关键的是这几句：

```cpp
tmpId = mj_name2id(mj_model, mjOBJ_JOINT, JointName[i].c_str());
jntId_qpos[i] = mj_model->jnt_qposadr[tmpId];
jntId_qvel[i] = mj_model->jnt_dofadr[tmpId];
```

这说明：

```text
JointName[i]
-> MuJoCo joint id
-> qpos 中的位置索引
-> qvel 中的速度索引
```

所以后面读取关节时：

```cpp
motor_pos[i] = mj_data->qpos[jntId_qpos[i]];
motor_vel[i] = mj_data->qvel[jntId_qvel[i]];
```

不是直接顺序扫 `qpos/qvel`，而是按 `JointName` 精确映射。

```text
motor_pos / motor_vel 的顺序由 JointName 决定，
不直接等于 mj_data->qpos / qvel 的原始顺序。
```

另外，这里的“控制顺序关节向量”来源也已经明确：

- `JointName` 定义在 `sim_interface/MJ_interface.h`
- 它先列出左臂、右臂、头、腰、左腿、右腿
- `MJ_Interface` 后续所有 `motor_pos[i] / motor_vel[i] / ctrl[i]` 都按这个顺序组织

同一个头文件里还定义了传感器名字：

- `orientationSensorName = "baselink-quat"`
- `velSensorName = "baselink-velocity"`
- `gyroSensorName = "baselink-gyro"`
- `accSensorName = "baselink-baseAcc"`

这几个字符串本身不做计算，它们只是给 `mj_name2id()` 提供名字，先换成 MuJoCo 内部 id，再去读 `sensordata`。

## 6. updateSensorValues() 读取了什么

### 6.1 关节状态

代码：

```cpp
for (int i = 0; i < jointNum; i++)
{
    motor_pos_Old[i] = motor_pos[i];
    motor_pos[i] = mj_data->qpos[jntId_qpos[i]];
    motor_vel[i] = mj_data->qvel[jntId_qvel[i]];
}
```

这一段会更新：

- `motor_pos_Old`
- `motor_pos`
- `motor_vel`

- `motor_pos[i]`：第 `i` 个受控关节的当前位置
- `motor_vel[i]`：第 `i` 个受控关节的当前速度

后面它们会进入：

- `RobotState.motors_pos_cur`
- `RobotState.motors_vel_cur`

再映射到：

- `q[7:]`
- `dq[6:]`

### 6.2 base 姿态 quaternion 和 rpy

代码：

```cpp
for (int i = 0; i < 4; i++)
    baseQuat[i] = mj_data->sensordata[mj_model->sensor_adr[orientataionSensorId] + i];
double tmp = baseQuat[0];
baseQuat[0] = baseQuat[1];
baseQuat[1] = baseQuat[2];
baseQuat[2] = baseQuat[3];
baseQuat[3] = tmp;
```

来源：

```text
sensordata["baselink-quat"]
```

这里说明：

- MuJoCo quaternion 顺序：`[w, x, y, z]`
- 代码内部改成：`[x, y, z, w]`

也就是这段代码：

```cpp
double tmp = baseQuat[0];
baseQuat[0] = baseQuat[1];
baseQuat[1] = baseQuat[2];
baseQuat[2] = baseQuat[3];
baseQuat[3] = tmp;
```

它不是做数学变换，而只是做分量重排，让后面的四元数转欧拉角公式可以按内部约定使用。

后面再计算：

```cpp
rpy[0] = roll
rpy[1] = pitch
rpy[2] = yaw
```

并对 yaw 做 unwrap，避免在 `pi/-pi` 邻域发生跳变。

具体就是：

```cpp
if ((rpy[2] - yaw_simgle) > 3.1415926*0.5)
    yaw_N -= 1.0;
else if ((rpy[2] - yaw_simgle) < -3.1415926*0.5)
    yaw_N += 1.0;
yaw_simgle = rpy[2];
rpy[2] = yaw_simgle + yaw_N * 2.0 * 3.1415926;
```

作用是把原本会在 `[-pi, pi]` 来回跳的 yaw 角，改成连续累计的 yaw，便于控制器做微分和跟踪。

```text
MuJoCo quaternion
-> baseQuat
-> rpy
```

### 6.3 basePos / baseAcc / baseAngVel / baseLinVel

代码：

```cpp
double posOld = basePos[i];
basePos[i] = mj_data->xpos[3 * baseBodyId + i];
baseAcc[i] = mj_data->sensordata[mj_model->sensor_adr[accSensorId] + i];
baseAngVel[i] = mj_data->sensordata[mj_model->sensor_adr[gyroSensorId] + i];
baseLinVel[i] = (basePos[i] - posOld) / (mj_model->opt.timestep);
```

得到：

- `basePos`：来自 `mj_data->xpos[base_link]`
- `baseAcc`：来自 `baselink-baseAcc`
- `baseAngVel`：来自 `baselink-gyro`
- `baseLinVel`：由 `basePos` 差分得到

这里要特别记一个坑：

```text
这些量在 MJ_Interface 中读出来了，
但不是每个量都会直接写入 DataBus。
```

## 7. dataBusWrite() 到底写了什么

`MJ_Interface::dataBusWrite()` 中，明确写入了：

```cpp
busIn.motors_pos_cur = motor_pos;
busIn.motors_vel_cur = motor_vel;
busIn.rpy = rpy;
busIn.baseAcc = baseAcc;
busIn.baseAngVel = baseAngVel;
```

还写入了：

- `fL`
- `fR`

但是这几行被注释掉了：

```cpp
// busIn.basePos[0] = basePos[0];
// busIn.basePos[1] = basePos[1];
// busIn.basePos[2] = basePos[2];
// busIn.baseLinVel[0] = baseLinVel[0];
// busIn.baseLinVel[1] = baseLinVel[1];
// busIn.baseLinVel[2] = baseLinVel[2];
```

然后最后执行：

```cpp
busIn.updateQ();
```

```text
MJ_Interface 把关节状态、rpy、IMU 加速度、IMU 角速度写入 DataBus。
但 base position 和 base linear velocity 没有直接写入 DataBus。
```

## 8. DataBus::updateQ() 如何组装 q / dq

### 8.1 先处理 base 角速度

代码：

```cpp
base_omega_W << baseAngVel[0], baseAngVel[1], baseAngVel[2];
auto Rcur = eul2Rot(rpy[0], rpy[1], rpy[2]);
base_omega_W = Rcur * base_omega_W;
```

含义：

```text
baseAngVel 先按机体系读入，
再通过当前姿态旋转到世界系，
得到 base_omega_W。
```

### 8.2 q 的映射

代码：

```cpp
auto quatNow = eul2quat(rpy[0], rpy[1], rpy[2]);
q(0) = basePos[0];
q(1) = basePos[1];
q(2) = basePos[2];
q(3) = quatNow.x();
q(4) = quatNow.y();
q(5) = quatNow.z();
q(6) = quatNow.w();
for (int i = 0; i < model_nv - 6; i++)
    q(i + 7) = motors_pos_cur[i];
```

映射关系：

```text
q[0] = basePos.x
q[1] = basePos.y
q[2] = basePos.z

q[3] = quaternion.x
q[4] = quaternion.y
q[5] = quaternion.z
q[6] = quaternion.w

q[7 + i] = motors_pos_cur[i]
```

### 8.3 dq 的映射

代码：

```cpp
Eigen::Vector3d vCoM_W;
vCoM_W << baseLinVel[0], baseLinVel[1], baseLinVel[2];
dq.block<3, 1>(0, 0) = vCoM_W;
dq.block<3, 1>(3, 0) << base_omega_W[0], base_omega_W[1], base_omega_W[2];
for (int i = 0; i < model_nv - 6; i++)
{
    dq(i + 6) = motors_vel_cur[i];
}
```

映射关系：

```text
dq[0] = baseLinVel.x
dq[1] = baseLinVel.y
dq[2] = baseLinVel.z

dq[3] = base_omega_W.x
dq[4] = base_omega_W.y
dq[5] = base_omega_W.z

dq[6 + i] = motors_vel_cur[i]
```

```text
DataBus::updateQ() 负责把关节状态、rpy、base 观测整理成 floating-base q / dq。
```

## 9. 第二轮最终图

```text
mj_data->qpos
-> motor_pos[i]
-> RobotState.motors_pos_cur[i]
-> q[7 + i]

mj_data->qvel
-> motor_vel[i]
-> RobotState.motors_vel_cur[i]
-> dq[6 + i]

mj_data->sensordata["baselink-quat"]
-> baseQuat
-> rpy
-> quaternion(rpy)
-> q[3:6]

mj_data->sensordata["baselink-gyro"]
-> baseAngVel
-> Rcur * baseAngVel
-> dq[3:5]

mj_data->sensordata["baselink-baseAcc"]
-> baseAcc
-> RobotState.baseAcc
-> 后续 StateEst 使用

mj_data->xpos[base_link]
-> basePos
-> baseLinVel by finite difference
-> 当前没有直接写入 DataBus
```

## 10. 第二轮核心结论

第二轮读完后，应明确：

```text
MJ_Interface 负责从 MuJoCo 读取关节、IMU 和 base 观测；
DataBus::updateQ() 负责把这些观测整理成 floating-base q / dq；
但 base position / base linear velocity 在 dataBusWrite() 中没有直接写入，
所以 q[0:2] 和 dq[0:2] 的最终可信来源要到下一轮 StateEst 中继续确认。
```

## 11. 当前仍未完全确认的点

- `f3d -> fL/fR` 的真正来源，这一轮还没在 `MJ_Interface.cpp` 当前范围内读到。
- `basePos / baseLinVel` 虽然在 `MJ_Interface` 内部被更新，但由于没有直接写入 `DataBus`，它们在后续 `q[0:2]` / `dq[0:2]` 中的最终可信来源还不能在这一轮定论。

## 12. 下一轮建议

第三轮再读：

- `StateEst::set()`
- `StateEst::update()`
- `StateEst::get()`

目标是回答：

```text
为什么 basePos / baseLinVel 没直接用 MuJoCo，
而是要经过状态估计修正。
```
