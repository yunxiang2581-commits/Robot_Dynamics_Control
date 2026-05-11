# A 项目补充实现路径

本文档用于把当前四接口骨架继续补成可验证的学习实现。当前主线仍然是：

```text
URDF -> FK -> Jacobian -> IK -> dynamics -> control
```

当前 A 项目已经收口为两层：

```text
Foundation validation layer:
  A00 reference and assets
  A01 model inspect
  A02 configuration / site pose
  A03 site Jacobian check

Unified motion interface layer:
  Target interface
  IK interface
  Viewer interface
  Actuator interface
```

A04/A05/A06/A07 当前是 wrapper + TODO learning skeleton，不是完整算法实现。后续补实现时应优先保持小步、可验证、可回滚。

## 1. 总体顺序

推荐实现顺序：

```text
R1  TargetDefinition load/save/validate
R2  A04/A05 IkRequest 配置读取
R3  IK 纯数学小函数
R4  A04 DLS position-only 最小恢复
R5  A05 QP-IK position-only 最小恢复
R6  A06 fixed target 生成
R7  A06 pose sequence 生成
R8  A06 mocap/viewer capability 验证
R9  A07 actuator tracking 最小实现
R10 demo / video 展示层
```

不要先写 viewer、video 或 actuator。先把 target schema、IK request 和最小 IK 数学跑通。

## 2. R1: TargetDefinition load/save/validate

目标文件：

```text
projects/A_self_baseline/src/robot_baseline/target_interface.py
projects/A_self_baseline/src/robot_baseline/motion_types.py
```

优先补：

```text
parse_vector_optional()
load_target_definition()
save_target_definition()
target_from_offset()
```

输入：

```text
motion_task.yaml
A02_site_pose.json
A06_target_definition.json，可选
```

输出：

```text
TargetDefinition
```

验证标准：

```text
position shape = (3,)
quat_wxyz shape = (4,)
rotation_matrix shape = (3, 3)
JSON 能保存再读回
```

常见错误：

```text
quat wxyz / xyzw 顺序混淆
position 写成字符串后没有转成 float array
target frame 没记录
把 RPY 当成内部误差表示
```

## 3. R2: A04/A05 IkRequest 配置读取

目标文件：

```text
projects/A_self_baseline/scripts/04_ik_dls_wrapper.py
projects/A_self_baseline/scripts/05_ik_qp_wrapper.py
projects/A_self_baseline/configs/motion_task.yaml
```

要补什么：

```text
从 motion_task.yaml 读取 ik / limits / target 配置
填充 IkRequest.weights
填充 IkRequest.limits
填充 IkRequest.solver_config
保留 CLI 少量覆盖 solver_type / task_mode / target_definition
```

为什么：

A04/A05 不应该各自维护一套参数来源。统一配置后，后续 DLS 和 QP-IK 的差异只体现在 solver_type 和 limits。

验证标准：

```text
IkRequest.solver_type 正确
IkRequest.task_mode 正确
site_name = attachment_site
weights / limits / solver_config 不为空
```

## 4. R3: IK 纯数学小函数

目标文件：

```text
projects/A_self_baseline/src/robot_baseline/ik_interface.py
```

优先补：

```text
build_qp_objective()
build_velocity_bounds()
merge_bounds()
solve_box_qp()  # 第一版只支持 scipy
```

QP 目标函数：

```text
minimize ||J_task dq - gain e_task||^2 + regularization ||dq||^2
```

标准二次型：

```text
minimize 1/2 dq^T H dq + c^T dq
H = J_task^T J_task + regularization I
c = -J_task^T gain e_task
```

验证标准：

```text
H shape = (nv, nv)
c shape = (nv,)
H 对称
dq shape = (nv,)
lower <= dq <= upper
```

常见错误：

```text
把 OSQP 里的 q 变量和机器人 q 混淆
H 没有正则导致数值不稳定
bounds shape 和 nv 不一致
```

## 5. R4: A04 DLS position-only 最小恢复

目标文件：

```text
projects/A_self_baseline/src/robot_baseline/ik_interface.py
projects/A_self_baseline/scripts/04_ik_dls_wrapper.py
```

数学逻辑：

```text
e_pos = p_target - p_current
dq = J^T (J J^T + lambda^2 I)^-1 gain e_pos
q_next = integrate(q, dq, dt)
```

推荐 API：

```text
mujoco.MjModel.from_xml_path
mujoco.MjData
mujoco.mj_forward
mujoco.mj_jacSite
mujoco.mj_integratePos
```

输出：

```text
outputs/trajectories/A04_dls_ik_q_traj.npy
outputs/logs/A04_dls_ik_error.csv
outputs/figures/A04_dls_ik_error.png
outputs/reports/A04_dls_ik_report.md
```

验证标准：

```text
position_error_norm 下降
q_traj shape = (N, 6)
dq_norm 不爆炸
不写 data.ctrl
```

## 6. R5: A05 QP-IK position-only 最小恢复

目标文件：

```text
projects/A_self_baseline/src/robot_baseline/ik_interface.py
projects/A_self_baseline/scripts/05_ik_qp_wrapper.py
```

任务结构：

```text
FrameTask(position-only)
VelocityLimit
JointPositionLimit
SciPy box QP
```

约束：

```text
dq_min <= dq <= dq_max
q_min <= q + dq dt <= q_max
```

position limit 转换：

```text
(q_min - q) / dt <= dq <= (q_max - q) / dt
```

输出：

```text
outputs/cache/A05_target_definition.json
outputs/trajectories/A05_qp_ik_q_traj.npy
outputs/trajectories/A05_qp_ik_dq_traj.npy
outputs/logs/A05_qp_ik_error.csv
outputs/logs/A05_qp_ik_constraints.csv
outputs/figures/A05_qp_ik_error.png
outputs/reports/A05_qp_ik_report.md
```

验证标准：

```text
position_error_norm 下降
max_constraint_violation 接近 0
q_traj shape = (N, 6)
dq_traj shape = (N, 6)
```

## 7. R6: A06 fixed target

目标文件：

```text
projects/A_self_baseline/scripts/06_target_viewer_wrapper.py
projects/A_self_baseline/src/robot_baseline/target_interface.py
```

目标：

```text
生成 A06_target_definition.json
支持 fixed_pose
支持 offset_from_current
```

公式：

```text
p_target = p_current + delta_p
```

验证标准：

```text
target_position shape = (3,)
target_quat_wxyz shape = (4,)
target_position - current_position 与 offset 一致
A05 能读取该 target_definition
```

## 8. R7: A06 pose sequence

目标文件：

```text
projects/A_self_baseline/scripts/06_target_viewer_wrapper.py
projects/A_self_baseline/src/robot_baseline/target_interface.py
```

位置插值：

```text
p_i = p_start + alpha_i (p_target - p_start)
alpha_i = i / (N - 1)
```

第一版姿态：

```text
orientation = keep_current
```

验证标准：

```text
waypoints 数量等于 num_waypoints
首点和末点正确
每个 waypoint 都有 frame / timestamp / position / quat
```

## 9. R8: A06 mocap/viewer capability

目标文件：

```text
projects/A_self_baseline/src/robot_baseline/viewer_interface.py
projects/A_self_baseline/scripts/06_target_viewer_wrapper.py
```

分步顺序：

```text
先做 mocap_placeholder 检查
再规划派生 MJCF
再做 keyboard target movement
再验证 mouse drag target
最后才做 kinematic IK follow
```

需要验证：

```text
model.nmocap
data.mocap_pos.shape = (model.nmocap, 3)
data.mocap_quat.shape = (model.nmocap, 4)
```

边界：

```text
A06 不写 data.ctrl
A06 不做 actuator tracking
A06 不录 video
```

## 10. R9: A07 actuator tracking

目标文件：

```text
projects/A_self_baseline/scripts/07_actuator_wrapper.py
projects/A_self_baseline/src/robot_baseline/actuator_interface.py
```

输入：

```text
A04 或 A05 生成的 q_traj
trajectory_source
MuJoCo model actuator 信息
```

未来控制数据流：

```text
q_des -> actuator mapping -> data.ctrl -> mujoco.mj_step
```

PD 形式：

```text
u = Kp(q_des - q) + Kd(dq_des - dq)
```

注意：

```text
position actuator 的 ctrl 不等于 torque
torque actuator 才适合直接理解成力矩输入
A07 是唯一允许写 data.ctrl 的模块
```

验证标准：

```text
q_traj shape = (N, nq)
model.nu 正确
ctrl shape = (nu,)
ctrl 不越界
tracking error 可记录
```

## 11. R10: demo / video 展示层

目标：

```text
只做展示，不混入 IK / target / actuator 核心逻辑
```

输入：

```text
trajectory_source
A07 tracking log
MuJoCo scene
```

输出：

```text
outputs/videos/
outputs/reports/
```

验证标准：

```text
视频文件存在
轨迹来源记录清楚
报告说明 target_source / trajectory_source / control_mode
```

## 12. 模块边界总结

```text
A04: DLS IK，不处理 QP，不写 data.ctrl
A05: QP-IK，不启动 viewer，不写 data.ctrl
A06: target manager / viewer target，不做 actuator tracking
A07: actuator executor，未来才写 data.ctrl
A10: demo 展示，不放核心算法
```

## 13. 当前最小下一步

建议下一步只做：

```text
R1: target_interface.py
  - parse_vector_optional()
  - load_target_definition()
  - save_target_definition()
```

不要先补 QP solver。先把 target schema 的读写和验证打稳，后续 A04/A05/A06 才能复用同一条目标数据流。
