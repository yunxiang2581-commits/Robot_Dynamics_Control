# walk_wbc 第一轮阅读笔记

## 1. 本轮目标

- 只读 `external/open_source_repos/OpenLoong-Dyn-Control/demo/walk_wbc.cpp` 主流程。
- 回答一个核心问题：

```text
walk_wbc 每个仿真步是怎样从 MuJoCo 状态走到 WBC/PVT 力矩，再写回 MuJoCo 的？
```

## 2. 本轮边界

- 只读 `walk_wbc.cpp`。
- 不展开 `MJ_Interface`、`StateEst`、`Pin_KinDyn`、`WBC_priority`、`PVT_Ctr` 类内部实现。
- 不运行 GUI，不修改 `external/` 源码。

## 3. 文件角色

`walk_wbc.cpp` 不是单个算法实现文件，而是顶层控制装配入口。

它负责把这些模块串起来：

- MuJoCo 仿真
- 状态估计
- Pinocchio 运动学/动力学
- 步态调度
- 摆脚落脚点规划
- WBC
- PVT/PD 关节控制
- 日志与可视化

一句话总结：

```text
MuJoCo step -> 状态读入 -> 状态估计 -> 运动学/动力学更新 -> walking 任务生成 -> WBC -> PVT -> torque 写回 MuJoCo
```

## 3.1 主流程摘要

`walk_wbc` 先加载 MuJoCo 场景 XML 和 Pinocchio 使用的 URDF。  
`main()` 中创建 `MJ_Interface`、`DataBus(RobotState)`、`Pin_KinDyn`、`StateEst`、`GaitScheduler`、`FootPlacement`、`WBC_priority`、`PVT_Ctr` 等模块，组成完整闭环。

在每个仿真 timestep 中，MuJoCo 先执行 `mj_step()` 推进一步；随后 `MJ_Interface` 从 `mj_data` 中读取状态和传感器信息并写入 `RobotState`。  
`StateEst` 基于观测修正 base 状态；`Pin_KinDyn` 基于当前 `q/dq` 计算 Jacobian、`dJ` 和动力学项。

之后 `JoyStickInterpreter`、`GaitScheduler`、`FootPlacement` 生成 walking 相关任务参考；`WBC_priority` 根据当前状态、任务参考和模型量求解全身层结果，包括期望广义运动、关节力矩结果以及接触力结果。  
`PVT_Ctr` 再把期望关节 `pos/vel/torque` 整形成最终电机力矩 `motors_tor_out`。  
最后 `MJ_Interface` 将 `motors_tor_out` 写入 `mj_data->ctrl`，并在下一次 `mj_step()` 时真正作用到仿真。

## 4. 文件级初始化

在 `main()` 外已经完成：

- `mj_loadXML("../models/scene_board.xml", ...)`
- `mj_makeData(mj_model)`

当前理解：

- `mj_model`：MuJoCo 模型结构
- `mj_data`：MuJoCo 当前仿真状态

这说明程序进入 `main()` 前，场景已经加载完成。

## 5. main() 中创建的模块

`main()` 一开始实例化了：

- `UIctr`
- `MJ_Interface`
- `Pin_KinDyn`
- `DataBus RobotState`
- `WBC_priority`
- `GaitScheduler`
- `PVT_Ctr`
- `FootPlacement`
- `JoyStickInterpreter`
- `DataLogger`
- `StateEst`

当前理解：

- `RobotState` 是这份 demo 的状态总线。
- 大多数模块都遵循：

```text
dataBusRead(RobotState) -> 内部计算 -> dataBusWrite(RobotState)
```

## 6. 初始化姿态

主循环开始前，代码先完成：

- 双脚目标位置/姿态设定
- 双臂目标关节角设定
- 腿部 IK：`computeInK_Leg(...)`
- 构造全身初始配置 `qIniDes`
- `WBC_solv.setQini(qIniDes, RobotState.q)`

当前理解：

- 这一步不是 walking 主循环，而是在给系统准备一个合理的起步姿态参考。

## 7. datalog 注册

当前 demo 注册了这些日志字段：

- `simTime`
- `motors_pos_cur`
- `motors_vel_cur`
- `rpy`
- `fL`
- `fR`
- `basePos`
- `baseLinVel`
- `baseAcc`
- `baseAngVel`

当前理解：

- 当前日志更偏“基础状态观测”。
- 尚未记录 `qp_status`、`wbc_tauJointRes`、`wbc_FrRes` 这类 WBC 内部结果。

## 8. 双层循环结构

主循环是双层结构：

- 外层：窗口未关闭时持续运行
- 内层：每个显示帧内执行多个 MuJoCo 小步

关键时间参数：

- `simEndTime = 30`
- `startSteppingTime = 3`
- `startWalkingTime = 5`

当前理解：

- 0 到 3 秒：更像初始站立/缓启动
- 3 到 5 秒：进入 stepping/walking 相关模块
- 5 秒后：开始给前进速度命令

## 9. 每个仿真步的主调用链

### 9.1 MuJoCo 先推进一步

- `mj_step(mj_model, mj_data);`

当前理解：

- 当前步使用的是上一轮已经写入的控制量。
- 这是标准离散闭环时序。

### 9.2 从 MuJoCo 读状态到 RobotState

- `mj_interface.updateSensorValues();`
- `mj_interface.dataBusWrite(RobotState);`

当前理解：

- MuJoCo 原始状态和传感器数据被转写到 `RobotState`。

### 9.3 状态估计

- `StateModule.init(...)`
- `StateModule.set(...)`
- `StateModule.update()`
- `StateModule.get(...)`

当前理解：

- 估计器会修正或补全 base 状态。

### 9.4 Pinocchio 运动学/动力学更新

- `kinDynSolver.dataBusRead(RobotState);`
- `kinDynSolver.computeJ_dJ();`
- `kinDynSolver.computeDyn();`
- `kinDynSolver.dataBusWrite(RobotState);`

当前理解：

- 这里是在为 WBC 准备 Jacobian、`dJ`、动力学项等模型量。

### 9.5 接触/力相关状态更新

- `StateModule.setF(...)`
- `StateModule.updateF()`
- `StateModule.getF(...)`

当前理解：

- 这一步和足端受力或接触状态有关。
- 精确细节需要下一轮读 `StateEst`。

### 9.6 walking 任务生成

- `jsInterp`
- `gaitScheduler`
- `footPlacement`

当前理解：

- `jsInterp`：生成 base 参考和速度命令
- `gaitScheduler`：决定当前步态相位与支撑/摆动状态
- `footPlacement`：生成摆动脚目标

### 9.7 WBC 输入构造

代码先清零：

- `des_ddq`
- `des_dq`
- `des_delta_q`

再设置：

- `Fr_ff`

当前理解：

- WBC 的高层输入是 base 的位置增量、速度、加速度参考，以及双脚前馈支撑力。
- 真正进入前向 walking 后，只改 base 的 `x/y/yaw` 相关分量。

### 9.8 WBC 求解

- `WBC_solv.dataBusRead(RobotState);`
- `WBC_solv.computeDdq(kinDynSolver);`
- `WBC_solv.computeTau();`
- `WBC_solv.dataBusWrite(RobotState);`

当前理解：

- `computeDdq()`：先求全身运动层结果
- `computeTau()`：再求实现该结果所需的关节力矩

### 9.9 WBC 输出转成关节命令

启动阶段：

- 使用初始 IK 姿态作为关节位置目标

stepping 之后：

- `integrateDIY(RobotState.q, RobotState.wbc_delta_q_final)`
- 提取关节部分为 `motors_pos_des`
- 使用 `wbc_dq_final` 作为 `motors_vel_des`
- 使用 `wbc_tauJointRes` 作为 `motors_tor_des`

当前理解：

- WBC 输出不是直接送执行器，而是先落成关节层目标。

### 9.10 PVT/PD 关节控制

- `pvtCtr.dataBusRead(RobotState);`
- 启动阶段使用 `calMotorsPVT(...)`
- 之后为双腿各关节设置 PD
- `pvtCtr.calMotorsPVT();`
- `pvtCtr.dataBusWrite(RobotState);`

当前理解：

- `PVT_Ctr` 把关节级位置/速度/力矩参考整形成最终电机输出力矩。

### 9.11 最终写回 MuJoCo

- `mj_interface.setMotorsTorque(RobotState.motors_tor_out);`

当前理解：

- 这是整个闭环真正作用到仿真的位置。

## 10. 第一轮阅读结论

`walk_wbc.cpp` 的核心不是某个公式，而是一条清晰的控制流水线：

```text
上一轮 torque -> mj_step
-> 读 MuJoCo 状态
-> 状态估计
-> 运动学/动力学更新
-> walking 任务生成
-> WBC
-> PVT/PD
-> 新 torque 写回 MuJoCo
```

## 11. 当前仍未展开的问题

- `MJ_Interface::dataBusWrite()` 具体把哪些 MuJoCo 量写入了 `RobotState`
- `StateEst` 如何估计 `basePos`、`baseLinVel`、`fL/fR`
- `computeInK_Leg()` 返回的 `jointPosRes` 覆盖哪些关节
- `WBC_priority::computeDdq()` / `computeTau()` 内部到底是解析法、QP 还是其他优先级求解
- `integrateDIY()` 如何处理浮动基配置积分

## 12. 建议的第二轮阅读顺序

建议先按下面顺序进入类内部：

1. `MJ_Interface`
2. `DataBus`
3. `StateEst`
4. `Pin_KinDyn`
5. `WBC_priority`
6. `PVT_Ctr`

这样能先把数据字段和状态流读清楚，再回来看 WBC 细节。
