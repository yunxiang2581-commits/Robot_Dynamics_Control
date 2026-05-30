# C OpenLoong Architecture Map

## 1. Project C vs Official Source

```text
projects/C_openloong_dyn_control_study/
    docs / notes / simulator skeleton / outputs
    role: learning wrapper, audit reports, future simplified simulator

external/open_source_repos/OpenLoong-Dyn-Control/
    C++ source / CMake / models / third_party / demos
    role: official upstream implementation, read-only reference
```

Project C 当前没有实现 OpenLoong 控制链。真实实现都在官方 external 仓库。

## 2. Official Directory Roles

| 目录 | 作用 | 关键文件 |
|---|---|---|
| `demo/` | 可执行 demo 入口 | `walk_wbc.cpp`, `walk_mpc_wbc.cpp`, `jump_mpc.cpp`, `walk_wbc_speed_test.cpp` |
| `algorithm/` | 控制与动力学算法 | `pino_kin_dyn.*`, `mpc.*`, `wbc_priority.*`, `priority_tasks.*`, `gait_scheduler.*`, `foot_placement.*`, `StateEst.*` |
| `common/` | 共享数据与低层控制 | `data_bus.h`, `PVT_ctrl.*`, `joint_ctrl_config.json`, `data_logger.*` |
| `sim_interface/` | MuJoCo/GLFW 接口 | `MJ_interface.*`, `GLFW_callbacks.*` |
| `math/` | 数学工具和轨迹工具 | `useful_math.*`, `bezier_1D.*`, `ramp_trajectory.*`, `LPF_fst.*` |
| `models/` | URDF/MuJoCo XML/mesh | `AzureLoong.urdf`, `AzureLoong.xml`, `scene.xml`, `scene_board.xml`, `meshes/*.STL` |
| `third_party/` | vendored dependencies | MuJoCo, Pinocchio, Eigen, GLFW, qpOASES, JsonCpp, Quill, urdfdom |
| `record/` | 官方运行日志输出 | `datalog.log` generated at runtime, helper txt/m scripts |

## 3. Module IO Map

| 模块 | 文件 | 输入 | 输出 | 下游 |
|---|---|---|---|---|
| MuJoCo model | `models/scene*.xml`, `AzureLoong.xml` | XML, mesh, timestep, actuator/sensor config | `mjModel`, `mjData` | `MJ_Interface`, demo loop |
| MJ interface | `MJ_interface.*` | `mjModel`, `mjData`, sensor/joint/motor names | base state, joint state, foot force, actuator torque command | `DataBus`, MuJoCo |
| DataBus | `data_bus.h` | all module writes | shared fields | all modules |
| StateEst | `StateEst.*` | sensor/base/foot/contact data | estimated base/foot/contact data | WBC, planner |
| Pin_KinDyn | `pino_kin_dyn.*` | URDF, `q`, `dq` | Jacobian, dJ, dynamics, CoM, IK | WBC, MPC, planner |
| JoyStickInterpreter | `joystick_interpreter.*` | desired vx/wz/time | desired base pos/vel/yaw | GaitScheduler, MPC/WBC desired state |
| GaitScheduler | `gait_scheduler.*` | state, contact, timing | `legState`, phase, contact schedule | FootPlacement, MPC, WBC |
| FootPlacement | `foot_placement.*` | leg state, base/foot state, desired velocity | swing foot target | WBC |
| MPC | `mpc.*` | current state, desired horizon, leg state | `Fr_ff`, `X_cal`, `dX_cal`, QP status | WBC, logger |
| WBC | `wbc_priority.*`, `priority_tasks.*` | dynamics, Jacobian, desired state, contact force | `delta_q`, `dq`, `ddq`, `tau`, contact force | PVT, logger |
| PVT | `PVT_ctrl.*`, `joint_ctrl_config.json` | desired P/V/T, current joint state, gains/limits | final motor torque | MJ_Interface |
| Logger | `data_logger.*` | selected DataBus fields | `record/datalog.log` | reproduction evidence |

## 4. Demo-Specific Architecture

### `walk_wbc`

```text
scene_board.xml + AzureLoong.urdf
    -> MJ_Interface
    -> DataBus
    -> StateEst
    -> Pin_KinDyn
    -> JoyStickInterpreter
    -> GaitScheduler
    -> FootPlacement
    -> WBC_priority with fixed/manual Fr_ff
    -> PVT_Ctr
    -> MuJoCo torque
    -> DataLogger
```

Use when the goal is to validate WBC/PVT/MuJoCo before introducing MPC.

### `walk_mpc_wbc`

```text
scene.xml + AzureLoong.urdf
    -> MJ_Interface
    -> DataBus
    -> Pin_KinDyn
    -> JoyStickInterpreter
    -> GaitScheduler
    -> FootPlacement
    -> MPC at 200 Hz
    -> WBC_priority
    -> PVT_Ctr
    -> MuJoCo torque
    -> DataLogger
```

Use after `walk_wbc` works. It adds MPC QP status, `X_cal`, `dX_cal`, `Xd`, and `Fr_ff` evidence.

### `wbc_speed_test`

```text
hard-coded fake state
    -> DataBus.updateQ
    -> Pin_KinDyn
    -> JoyStickInterpreter / GaitScheduler / FootPlacement
    -> WBC_priority
    -> PVT_Ctr
    -> DataLogger runtime column
```

This is the closest official non-viewer smoke candidate after build. It still requires official C++ build and relative model/config paths.

## 5. Critical Names and Dimensions

| Item | Value / Location | Why it matters |
|---|---|---|
| MuJoCo timestep | `models/AzureLoong.xml`, `timestep="0.001"` | Base simulation/control step. |
| MPC period | `demo/walk_mpc_wbc.cpp`, `dt_200Hz=0.005` | MPC runs every 5 simulation steps. |
| MPC horizon | `algorithm/mpc.h`, `mpc_N=10` | QP horizon length. |
| MPC state dim | `algorithm/mpc.h`, `nx=12` | eul, pos, omega, velocity state. |
| MPC input dim | `algorithm/mpc.h`, `nu=13` | contact-force/input vector dimension. |
| WBC QP decision | demo constructor `18` | acceleration/contact-force correction variables. |
| WBC QP constraints | demo constructor `22` | floating-base dynamics + contact/friction constraints. |
| Friction coefficient | demo constructor `0.7` | WBC contact cone/friction constraint. |
| Joint count | 31 motors in `MJ_interface.h` / `PVT_ctrl.h` / XML | Must align across XML, URDF, JSON, C++ lists. |
| Floating base | MuJoCo `freejoint`, Pinocchio `JointModelFreeFlyer` | `q` and `dq` offsets differ: q has quaternion. |

## 6. Read-Only Risk Points

- External source has many dirty files, sampled diff indicates line-ending changes. Preserve as evidence, do not reset during audit.
- `external/open_source_repos/OpenLoong-Dyn-Control/.git/index.lock` exists. Do not remove during audit.
- Official demo paths are relative: run from `build/`, not repo root.
- Official logs go to `external/.../record/datalog.log`; Project C should copy evidence out later instead of editing official logger first.
- Most official demos open GLFW viewer; use `wbc_speed_test` for non-viewer smoke after build.
