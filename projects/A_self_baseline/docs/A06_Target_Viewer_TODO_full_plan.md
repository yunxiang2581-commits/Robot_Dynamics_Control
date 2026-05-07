# A06 Target Viewer TODO Full Plan

## 当前定位

A06 是 Target interface + Viewer interface wrapper，入口为 `scripts/06_target_viewer_wrapper.py`。当前只规划完整功能 TODO，不启动 viewer。

## 对标 mink 的概念

对标 mink 的 mocap target、viewer target、target pose feeds FrameTask。

## 完整功能列表

- TargetDefinition 生成。
- fixed_pose。
- pose_sequence。
- mocap_placeholder。
- interactive viewer TODO。
- 派生 MJCF 规划。
- mouse drag target。
- keyboard movement。
- target pose feeds IK。
- kinematic IK follow。
- target logs / reports / preview。

## TODO 任务清单

1. A06-1 TargetDefinition 生成。
2. A06-2 fixed_pose。
3. A06-3 pose_sequence。
4. A06-4 mocap_placeholder。
5. A06-5 interactive viewer TODO。
6. A06-6 派生 MJCF 规划。
7. A06-7 mouse drag target。
8. A06-8 keyboard movement。
9. A06-9 target pose feeds IK。
10. A06-10 kinematic IK follow TODO。
11. A06-11 A06 输出规划。
12. A06-12 A06 边界。

## 数学公式

```text
p_target = p_current + Delta p
p_i = p_start + alpha_i (p_target - p_start)
alpha_i = i / (N - 1)

e_pos = p_target - p_current
R_err = R_target R_current^T
e_rot = log(R_err)
```

## 符号表

| 符号 | 含义 | 维度 | 单位 |
|---|---|---|---|
| p_target | 目标位置 | (3,) | m |
| R_target | 目标姿态 | (3,3) | - |
| quat_wxyz | 目标四元数 | (4,) | - |
| waypoints | target 序列 | (N,pose) | - |
| model.nmocap | mocap body 数量 | scalar | - |
| data.mocap_pos | mocap 位置数组 | (nmocap,3) | m |
| data.mocap_quat | mocap 姿态数组 | (nmocap,4) | wxyz |
| keyboard_step | 键盘步长 | scalar | m |

## 输入输出

输入：A02 current pose、motion_task.yaml、viewer target 配置、未来派生 MJCF。

未来输出：A06 target definition、tracking CSV、path preview、target report、interactive trace/report。

## 验证标准

- fixed target shape 正确。
- waypoints 数量正确。
- nmocap shape check 正确。
- mouse drag 后 target pose 变化。
- keyboard W/S/A/D/Q/E 可移动 target。
- A06 不写 `data.ctrl`。

## 常见错误

- 修改原始 scene.xml。
- `model.nmocap==0` 时访问 mocap arrays。
- 忘记 `viewer.sync()`。
- 不使用 `viewer.lock()` 修改 data。
- target pose 当 actuator control。
- 在 A06 中重新实现 IK。

## 前后关系

A06 生成 TargetDefinition；A04/A05 消费 target 求 IK；A07 消费 trajectory 执行 actuator。

## 当前不实现内容

不创建 MJCF，不启动 viewer，不做 mouse/keyboard，不做 IK follow，不写 `data.ctrl`。

## Step R-G 骨架格式

Python wrapper 只保留函数级 TODO skeleton：

- `build_target_request`
- `build_fixed_target`
- `build_pose_sequence`
- `plan_mocap_placeholder`
- `plan_interactive_viewer`
- `write_target_outputs`

完整 target/viewer/mocap 说明、公式、符号表、验证标准和常见错误保留在本文档中。Python 文件不再承载超长 TODO 字符串列表，也不在 `main()` 中打印大段 TODO。

## 后续顺序

R1 TargetDefinition；R3 fixed_pose/pose_sequence；R4 mocap capability；R5 derived MJCF；R6 keyboard；R7 mouse；R8 kinematic IK follow。
