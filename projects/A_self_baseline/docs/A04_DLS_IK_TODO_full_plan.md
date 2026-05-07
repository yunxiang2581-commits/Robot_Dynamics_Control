# A04 DLS IK TODO Full Plan

## 当前定位

A04 是 IK interface 的 DLS learning wrapper，入口为 `scripts/04_ik_dls_wrapper.py`。当前只生成完整功能版 TODO skeleton，不实现完整 DLS。

## 对标 mink 的概念

对标 mink 数据流中的 `FrameTask -> solve_ik`，但 A04 是无约束教学版，不处理 limits。

## 完整功能列表

- 读取 robot / motion task / A01-A03 前置产物。
- 解析 TargetDefinition。
- 加载 MuJoCo model/data。
- position-only DLS。
- pose_6d DLS。
- damping 数值稳定性检查。
- q integration。
- 终止条件。
- q trajectory、error log、figure、report。

## TODO 任务清单

1. A04-1 读取统一配置和前置产物。
2. A04-2 解析 TargetDefinition。
3. A04-3 加载 MuJoCo model/data。
4. A04-4 position-only DLS task。
5. A04-5 pose_6d DLS task。
6. A04-6 DLS 数值稳定性。
7. A04-7 积分更新 q。
8. A04-8 迭代终止条件。
9. A04-9 输出 q_traj / error log / figure / report。
10. A04-10 A04 边界。

## 数学公式

```text
e_pos = p_target - p_current
v_task = gain e_pos
dq = J^T (J J^T + lambda^2 I)^(-1) v_task

R_err = R_target R_current^T
e_rot = log(R_err)
e_task = [w_pos e_pos; w_rot e_rot]
J_task = [w_pos J_pos; w_rot J_rot]
q_next = integrate(q, dq, dt)
```

## 符号表

| 符号 | 含义 | 维度 | 单位 |
|---|---|---|---|
| q | 关节位置 | (nq,) | rad |
| dq | 关节速度 | (nv,) | rad/s |
| p_current | 当前 site 位置 | (3,) | m |
| p_target | 目标位置 | (3,) | m |
| R_current | 当前姿态 | (3,3) | - |
| R_target | 目标姿态 | (3,3) | - |
| J_pos | 位置 Jacobian | (3,nv) | - |
| J_rot | 角速度 Jacobian | (3,nv) | - |
| lambda | damping | scalar | - |

## 输入输出

输入：`robot.yaml`、`motion_task.yaml`、A01/A02/A03 cache、TargetDefinition。

未来输出：`A04_dls_ik_q_traj.npy`、`A04_dls_ik_error.csv`、`A04_dls_ik_error.png`、`A04_dls_ik_report.md`。

## 验证标准

- A01-A03 前置产物可读取。
- `site=attachment_site` 可验证。
- position error 下降。
- pose_6d 下 orientation error 下降。
- `dq_norm` 无爆炸。
- 不进入 QP / actuator / viewer。

## 常见错误

- quaternion wxyz / xyzw 混淆。
- RPY 直接相减。
- damping 太小导致奇异附近速度爆炸。
- dq 速度和 q 增量定义混用。
- 在 A04 中写 `data.ctrl`。

## 前后关系

A03 提供 Jacobian 验证基础；A04 生成无约束 IK 对照；A05 处理 QP limits；A07 后续消费 trajectory。

## 当前不实现内容

不实现完整 DLS，不生成 outputs，不调用 mink，不启动 viewer，不写 `data.ctrl`。

## Step R-G 骨架格式

Python wrapper 只保留函数级 TODO skeleton：

- `build_dls_request`
- `load_dls_inputs`
- `resolve_dls_target`
- `run_dls_trajectory`
- `write_dls_outputs`

完整公式、符号表、输入输出、验证标准和常见错误保留在本文档中。Python 文件不再承载超长 TODO 字符串列表，也不在 `main()` 中打印大段 TODO。

## 后续顺序

R1 TargetDefinition load/save/validate；R2 恢复 DLS 单步；R3 恢复 DLS trajectory loop。
