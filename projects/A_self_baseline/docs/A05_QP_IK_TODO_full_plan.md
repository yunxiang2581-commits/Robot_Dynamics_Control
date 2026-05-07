# A05 QP IK TODO Full Plan

## 当前定位

A05 是 IK interface 的 QP-IK wrapper，入口为 `scripts/05_ik_qp_wrapper.py`。当前是完整功能版 TODO skeleton，不实现 solver。

## 对标 mink 的概念

对标 `FrameTask / PostureTask / VelocityLimit / ConfigurationLimit / solve_ik`。

## 完整功能列表

- 读取 target definition。
- FrameTask。
- PostureTask。
- QP objective。
- VelocityLimit。
- JointPositionLimit / ConfigurationLimit。
- SciPy backend。
- OSQP backend。
- QP-IK outer loop。
- constraint log。
- q/dq trajectory、error log、figure、report。

## TODO 任务清单

1. A05-1 读取统一配置和 target definition。
2. A05-2 FrameTask。
3. A05-3 PostureTask。
4. A05-4 QP 目标函数。
5. A05-5 VelocityLimit。
6. A05-6 JointPositionLimit / ConfigurationLimit。
7. A05-7 SciPy solver。
8. A05-8 OSQP solver。
9. A05-9 QP-IK 外层循环。
10. A05-10 constraint violation 日志。
11. A05-11 输出完整产物。
12. A05-12 A05 边界。

## 数学公式

```text
e_pos = p_target - p_current
R_err = R_target R_current^T
e_rot = log(R_err)

J_task = stack([J_frame, w_posture I])
v_task = concat([gain_frame e_frame, gain_posture w_posture e_posture])

min ||J_task dq - v_task||^2 + regularization ||dq||^2
H = J_task^T J_task + regularization I
c = -J_task^T v_task

dq_min <= dq <= dq_max
(q_min - q) / dt <= dq <= (q_max - q) / dt
```

## 符号表

| 符号 | 含义 | 维度 | 单位 |
|---|---|---|---|
| dq | 优化变量，关节速度 | (nv,) | rad/s |
| J_frame | frame task Jacobian | (3或6,nv) | - |
| e_frame | frame task error | (3或6,) | m/rad |
| H | QP Hessian | (nv,nv) | - |
| c | QP linear term | (nv,) | - |
| lower/upper | box bounds | (nv,) | rad/s |
| q_osqp | OSQP 线性项变量名 | (nv,) | - |

## 输入输出

输入：`motion_task.yaml`、A06/A05 target definition、A02 current pose、MuJoCo model。

未来输出：`A05_target_definition.json`、`A05_qp_ik_q_traj.npy`、`A05_qp_ik_dq_traj.npy`、error/constraint CSV、figure、report。

## 验证标准

- `H.shape == (nv,nv)`。
- `dq.shape == (nv,)`。
- lower/upper shape=(nv,)。
- `max_constraint_violation ≈ 0`。
- error norm 下降。
- 不写 `data.ctrl`。

## 常见错误

- OSQP 的 `q` 和机器人 `q` 混淆。
- q/nq 与 dq/nv 混淆。
- position limit 推导忘记除 dt。
- solver success 后不检查 bounds。
- 在 A05 中启动 viewer 或 actuator。

## 前后关系

A06 定义 target；A05 求带约束 q/dq trajectory；A07 后续消费 trajectory。

## 当前不实现内容

不实现 SciPy/OSQP solver，不生成 outputs，不调用 mink，不写 `data.ctrl`。

## Step R-G 骨架格式

Python wrapper 只保留函数级 TODO skeleton：

- `build_qp_request`
- `load_qp_inputs`
- `resolve_qp_target`
- `build_qp_task_plan`
- `run_qp_trajectory`
- `write_qp_outputs`

完整 QP 公式、solver 细节、符号表、验证标准和常见错误保留在本文档中。Python 文件不再承载超长 TODO 字符串列表，也不在 `main()` 中打印大段 TODO。

## 后续顺序

R1 TargetDefinition；R2 A05 读取 target_definition_json；R3 QP backend；R9 A07 actuator tracking。
