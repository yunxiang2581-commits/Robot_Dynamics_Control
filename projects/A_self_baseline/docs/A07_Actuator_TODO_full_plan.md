# A07 Actuator TODO Full Plan

## 当前定位

A07 是 Actuator interface wrapper，入口为 `scripts/07_actuator_wrapper.py`。当前只规划完整 actuator TODO，不执行控制。

## 对标 mink 的概念

对标 mink 数据流末端：`viewer.sync -> data.ctrl / mj_step`。A07 是本项目未来唯一允许写 `data.ctrl` 的模块。

## 完整功能列表

- 读取 trajectory_source。
- MuJoCo model / actuator inspect。
- position actuator tracking。
- PD tracking。
- control range safety。
- MuJoCo control loop。
- tracking log。
- tracking figure。
- video demo placeholder。
- report。
- A07 边界。

## TODO 任务清单

1. A07-1 读取 trajectory_source。
2. A07-2 加载 MuJoCo model / actuator inspect。
3. A07-3 position actuator tracking。
4. A07-4 PD tracking。
5. A07-5 control range safety。
6. A07-6 MuJoCo control loop。
7. A07-7 tracking log。
8. A07-8 tracking figure。
9. A07-9 video demo placeholder。
10. A07-10 report。
11. A07-11 A07 边界。

## 数学公式

```text
q_error = q_des - q
dq_error = dq_des - dq
u = Kp q_error + Kd dq_error
ctrl_min <= ctrl <= ctrl_max
```

## 符号表

| 符号 | 含义 | 维度 | 单位 |
|---|---|---|---|
| q_traj | 期望轨迹 | (N,nq) | rad |
| q_des | 当前期望关节位置 | (nq,) | rad |
| q | 当前仿真关节位置 | (nq,) | rad |
| dq_des | 当前期望关节速度 | (nv,) | rad/s |
| dq | 当前仿真关节速度 | (nv,) | rad/s |
| ctrl | actuator command | (nu,) | actuator-specific |
| ctrlrange | actuator 命令范围 | (nu,2) | actuator-specific |

## 输入输出

输入：A04/A05 q trajectory、A06 trajectory_source metadata、MuJoCo model actuator 信息。

未来输出：A07 tracking CSV、tracking figure、report、video placeholder。

## 验证标准

- `q_traj.shape == (N,nq)`。
- `ctrl.shape == (model.nu,)`。
- ctrl 无 NaN。
- max_ctrl_violation 可解释。
- tracking error 可计算。

## 常见错误

- q_traj 与 ctrl 维度混用。
- position actuator 和 torque actuator 混用。
- clip 后不记录 saturation。
- A07 重新定义 target 或重新求 IK。
- 在 A06 写 `data.ctrl`。

## 前后关系

A04/A05 产生 q trajectory；A06 记录 target/trajectory metadata；A07 消费 trajectory；A10 整理最终 video showcase。

## 当前不实现内容

不写 `data.ctrl`，不调用 `mj_step` 控制循环，不启动 viewer，不生成 video。

## Step R-G 骨架格式

Python wrapper 只保留函数级 TODO skeleton：

- `build_actuator_request`
- `load_tracking_trajectory`
- `inspect_actuators`
- `plan_position_actuator_tracking`
- `plan_pd_tracking`
- `plan_control_loop`
- `write_actuator_outputs`

完整 actuator tracking 公式、符号表、验证标准和常见错误保留在本文档中。Python 文件不再承载超长 TODO 字符串列表，也不在 `main()` 中打印大段 TODO。

## 后续顺序

R9 A07 actuator tracking；R10 A10 video demo。
