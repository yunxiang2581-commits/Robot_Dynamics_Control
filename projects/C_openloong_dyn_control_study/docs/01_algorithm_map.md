# Project C 算法地图

## 主线数据流

```text
Joystick / Command
-> GaitScheduler
-> FootPlacement
-> StateEstimator
-> MPC
-> WBC_QP
-> PVT_Ctr
-> MuJoCo / Robot
-> Sensor feedback
```

## 核心算法

### 人形浮动基动力学

人形机器人不是固定基机械臂。pelvis / torso 在空间中运动，因此广义坐标通常包含 floating base。学习时要特别关注：

- base 位姿和速度如何表示。
- 接触约束如何限制浮动基运动。
- CoM 与接触力如何影响整体动力学。

### 质心 / 接触力 MPC

MPC 负责在未来 horizon 内规划身体状态、CoM 趋势和接触力。它把行走目标转成可执行的动力学目标。

### Whole-Body Control

WBC 将 MPC 的高层输出转换为全身关节级任务，通常需要同时满足：

- 浮动基动力学。
- 接触约束。
- 足端任务。
- 躯干姿态任务。
- 关节限制或力矩限制。

### 优先级任务控制

人形 WBC 中不同任务重要性不同，例如支撑脚接触约束通常高于摆动腿跟踪。后续源码阅读要确认项目如何表达任务优先级。

### 接触约束

双足运动的核心是支撑脚不能滑动、接触力要合理、摆动脚要避障或按轨迹落脚。

### 摆动腿轨迹

FootPlacement 给出落脚目标，摆动腿轨迹负责生成从当前足端到目标足端的平滑路径。

### 状态估计

StateEstimator 用传感器或仿真状态估计 base、关节、速度、接触等状态，为 MPC 和 WBC 提供当前状态输入。

### PVT 低层控制

PVT 通常表示 position / velocity / torque 组合控制，用于把 WBC 输出转成关节执行命令。

## 待后续源码阅读确认的信息

- MPC 是否使用 LIPM、centroidal dynamics 或其他简化模型。
- WBC_QP 决策变量是否包含 `qddot`、contact force、tau。
- 优先级任务是 strict hierarchy 还是权重加权。
- PVT 控制参数如何设置。
