# OpenLoong-Dyn-Control Learning Map

## 1. 先读哪些文件

推荐先按下面顺序读，不要从 `algorithm/mpc.cpp` 或 `wbc_priority.cpp` 直接开啃。那些文件有价值，但没有 DataBus 和 demo 主循环上下文时，会像拆一台还通着电的机器。

| 顺序 | 文件 | 看什么 | 看完应该知道什么 |
|---|---|---|---|
| 1 | `external/open_source_repos/OpenLoong-Dyn-Control/README.md` | 官方目标、依赖、demo、关键参数说明 | 这是基于 MPC + WBC 的 humanoid MuJoCo 控制框架。 |
| 2 | `external/open_source_repos/OpenLoong-Dyn-Control/README-zh.md` | 中文说明、术语、开发指南 | MPC/WBC/PVT/gait 参数在哪里。 |
| 3 | `projects/C_openloong_dyn_control_study/docs/C_CURRENT_STATUS.md` | Project C 当前状态 | Project C 是学习封装和未来 simulator 容器，不是已完成实现。 |
| 4 | `external/open_source_repos/OpenLoong-Dyn-Control/demo/walk_wbc.cpp` | WBC-only 主循环 | 不含 MPC 时，MuJoCo -> DataBus -> WBC -> PVT -> MuJoCo 怎么闭环。 |
| 5 | `external/open_source_repos/OpenLoong-Dyn-Control/demo/walk_mpc_wbc.cpp` | MPC+WBC 主循环 | MPC 以 `dt_200Hz=0.005` 插入 WBC 前，输出 `Fr_ff/X_cal/dX_cal`。 |
| 6 | `external/open_source_repos/OpenLoong-Dyn-Control/common/data_bus.h` | 所有模块共享字段 | 谁读什么、谁写什么。 |
| 7 | `external/open_source_repos/OpenLoong-Dyn-Control/sim_interface/MJ_interface.h` | MuJoCo sensor/joint/motor 名称 | XML 中的 joint/motor/sensor 如何进入 DataBus。 |
| 8 | `external/open_source_repos/OpenLoong-Dyn-Control/models/AzureLoong.xml` | MuJoCo model | `timestep=0.001`、`freejoint`、actuator、sensor。 |
| 9 | `external/open_source_repos/OpenLoong-Dyn-Control/models/AzureLoong.urdf` | Pinocchio model | 31 个关节和 link 树。 |
| 10 | `external/open_source_repos/OpenLoong-Dyn-Control/common/joint_ctrl_config.json` | 关节 PVT/limit 参数 | PVT 输出约束来自哪里。 |

## 2. 关键类 / 函数

| 模块 | 类 / 函数 | 文件 | 作用 |
|---|---|---|---|
| MuJoCo interface | `MJ_Interface::updateSensorValues` | `sim_interface/MJ_interface.*` | 从 MuJoCo 读取 base、IMU、joint、foot force。 |
| MuJoCo interface | `MJ_Interface::dataBusWrite` | `sim_interface/MJ_interface.*` | 把仿真状态写入 DataBus。 |
| MuJoCo interface | `MJ_Interface::setMotorsTorque` | `sim_interface/MJ_interface.*` | 把 PVT 输出的 torque 写回 actuator。 |
| State bus | `DataBus` | `common/data_bus.h` | 模块间共享状态总线。 |
| Pinocchio | `Pin_KinDyn::computeJ_dJ` | `algorithm/pino_kin_dyn.*` | 计算足端、手端、base、CoM Jacobian 和 dJ。 |
| Pinocchio | `Pin_KinDyn::computeDyn` | `algorithm/pino_kin_dyn.*` | 计算质量矩阵、重力、非线性项、centroidal 相关量。 |
| Gait | `GaitScheduler::step` | `algorithm/gait_scheduler.*` | 更新支撑腿/摆动腿和相位。 |
| Foot placement | `FootPlacement::getSwingPos` | `algorithm/foot_placement.*` | 生成摆动脚目标位置。 |
| MPC | `MPC::cal` | `algorithm/mpc.*` | 求解 horizon QP，输出接触力和状态预测。 |
| WBC | `WBC_priority::computeDdq` | `algorithm/wbc_priority.*` | 基于任务和约束求解 qddot/contact force 增量。 |
| WBC | `WBC_priority::computeTau` | `algorithm/wbc_priority.*` | 用动力学方程计算关节力矩。 |
| PVT | `PVT_Ctr::calMotorsPVT` | `common/PVT_ctrl.*` | 把 desired P/V/T 转成最终 motor torque。 |
| Logging | `DataLogger::recItermData` | `common/data_logger.*` | 写 `record/datalog.log`。 |

## 3. 控制链路

### WBC-only 链路

```text
MuJoCo mj_step
    -> MJ_Interface reads sensors
    -> DataBus.updateQ
    -> StateEst estimates base/contact
    -> Pin_KinDyn computes Jacobian + dynamics
    -> JoyStickInterpreter desired base command
    -> GaitScheduler leg state
    -> FootPlacement swing foot target
    -> manual desired ddq/dq/delta_q + fixed Fr_ff
    -> WBC_priority computeDdq + computeTau
    -> PVT_Ctr final torque
    -> MJ_Interface setMotorsTorque
    -> DataLogger
```

### MPC + WBC 链路

```text
MuJoCo mj_step
    -> MJ_Interface
    -> Pin_KinDyn
    -> JoyStickInterpreter
    -> GaitScheduler
    -> FootPlacement
    -> MPC every 0.005 s
    -> WBC_priority
    -> PVT_Ctr
    -> MuJoCo torque
    -> DataLogger
```

## 4. 数学模块

| 模块 | 数学对象 | 文件 | 学习重点 |
|---|---|---|---|
| Floating-base kinematics | `q`, `dq`, Jacobian, frame placement | `pino_kin_dyn.*`, `data_bus.h` | `nq = nv + 1` 的 free-flyer quaternion 差异；q 前 7 维、dq 前 6 维。 |
| Dynamics | `M`, `C`, `G`, `Non`, centroidal momentum | `pino_kin_dyn.*` | CRBA、gravity、Coriolis、centroidal 量如何服务 WBC。 |
| MPC | horizon state/input QP | `mpc.*` | `mpc_N=10`, `nx=12`, `nu=13`, qpOASES 输出接触力。 |
| WBC-QP | acceleration/contact-force QP | `wbc_priority.*`, `priority_tasks.*` | decision size 18，constraints 22，动力学等式 + 摩擦/接触不等式。 |
| PVT | PD + torque feedforward + limits/filter | `PVT_ctrl.*`, `joint_ctrl_config.json` | desired position/velocity/torque 如何组合成 actuator torque。 |
| State estimation | Kalman/trust region/contact estimate | `StateEst.*` | base/CoM/foot 状态估计如何改善 WBC 输入。 |

## 5. 仿真模块

| 模块 | 文件 | 注意点 |
|---|---|---|
| Model loading | `mj_loadXML("../models/scene*.xml")` in demo | 必须从 `build/` 目录运行，否则相对路径错。 |
| Viewer | `GLFW_callbacks.*` | 大部分 demo 会打开 GLFW 窗口。 |
| Sensors | `models/AzureLoong.xml`, `MJ_interface.h` | sensor 名包括 `baselink-quat`, `baselink-velocity`, `baselink-gyro`, `baselink-baseAcc`。 |
| Actuators | `models/AzureLoong.xml` | 31 个 motor，gear/ctrlrange 影响 torque 命令。 |
| Logging | `record/datalog.log` | 官方日志写到 external 仓库 `record/`，复现时应复制到 Project C outputs。 |

## 6. 复现路线

1. C00: 完成 architecture map，明确官方源码和 Project C 封装边界。
2. C01: 只做模型路径审查和 model loading smoke，不跑长仿真。
3. C02: 做 joint / motor / actuator / sensor mapping 表。
4. C03: 尝试构建后运行 `wbc_speed_test`，作为非 viewer WBC/PVT smoke。
5. C04: 运行 `walk_wbc`，保存日志和截图。
6. C05: 运行 `walk_mpc_wbc`，比较 MPC 增加的字段。
7. C06: 解析 `record/datalog.log`，输出 metrics/figures。
8. C07: 录短视频或导出动画。
9. C08: 写最终复现报告和简历项目描述。

## 7. 需要补的基础知识

- Floating base: free-flyer `q` 7 维、`dq` 6 维，为什么 `nq != nv`。
- Pinocchio: `JointModelFreeFlyer`、`forwardKinematics`、`computeJointJacobiansTimeVariation`、`crba`、`computeGeneralizedGravity`。
- MuJoCo XML: `freejoint`、`actuator/motor`、`sensor`、`site`、`ctrlrange`、`gear`。
- QP: qpOASES 的 Hessian、constraint matrix、lb/ub、status。
- WBC: floating-base dynamics equality, contact/friction constraints, task hierarchy。
- MPC: state horizon、contact force input、contact schedule、QP warm start/status。
- PVT/PD: desired position + desired velocity + feedforward torque + limits/filter。

## 8. 和 Project B 的关系

Project B 更偏 MPC 求解器和 OpenLoong-oriented humanoid MPC prototype。Project C 应承接 B 中形成的：

- desired base state
- desired pelvis / CoM target
- torso orientation target
- foot target
- contact schedule
- optional desired contact force

Project C 的新任务是把这些高层目标接到 WBC-QP、PVT/PD 和 MuJoCo 闭环。也就是说，B 更像“高层 MPC target 怎么来”，C 更像“target 如何进入完整 humanoid control stack”。
