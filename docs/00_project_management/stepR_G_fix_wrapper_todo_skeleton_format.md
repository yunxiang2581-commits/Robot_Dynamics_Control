# Step R-G Fix Wrapper TODO Skeleton Format

## 1. 为什么当前骨架不合适

Step R-F 把完整 TODO 写进了 Python 字符串列表，例如 `A04_TODO_ITEMS`、`A05_TODO_ITEMS`、`A06_TODO_ITEMS`、`A07_TODO_ITEMS`。这种格式的问题是：

- Python 文件像 Markdown，不像可逐步补实现的代码骨架。
- `main()` 会打印大段 TODO，运行时噪声过大。
- 后续 Codex 不容易按函数名逐个补实现。
- 公式、符号表、验证标准和常见错误更适合放在 Markdown。

## 2. 删除或停用的超长 TODO 列表

- 删除 `A04_TODO_ITEMS`。
- 删除 `A05_TODO_ITEMS`。
- 删除 `A06_TODO_ITEMS`。
- 删除 `A07_TODO_ITEMS`。
- 删除 `main()` 中对应的 `for item in ... logging.info(item)`。

## 3. 新增函数级 TODO

A04:

- `build_dls_request`
- `load_dls_inputs`
- `resolve_dls_target`
- `run_dls_trajectory`
- `write_dls_outputs`

A05:

- `build_qp_request`
- `load_qp_inputs`
- `resolve_qp_target`
- `build_qp_task_plan`
- `run_qp_trajectory`
- `write_qp_outputs`

A06:

- `build_target_request`
- `build_fixed_target`
- `build_pose_sequence`
- `plan_mocap_placeholder`
- `plan_interactive_viewer`
- `write_target_outputs`

A07:

- `build_actuator_request`
- `load_tracking_trajectory`
- `inspect_actuators`
- `plan_position_actuator_tracking`
- `plan_pd_tracking`
- `plan_control_loop`
- `write_actuator_outputs`

## 4. 完整说明迁移到 Markdown

完整功能说明保留在：

- `projects/A_self_baseline/docs/A04_DLS_IK_TODO_full_plan.md`
- `projects/A_self_baseline/docs/A05_QP_IK_TODO_full_plan.md`
- `projects/A_self_baseline/docs/A06_Target_Viewer_TODO_full_plan.md`
- `projects/A_self_baseline/docs/A07_Actuator_TODO_full_plan.md`

Python 文件只保留函数级 TODO docstring 和 `NotImplementedError`。

## 5. 当前未实现内容

- DLS IK。
- QP-IK。
- TargetDefinition 真实生成。
- viewer / mocap target。
- keyboard / mouse interaction。
- kinematic IK follow。
- actuator tracking。
- video generation。

## 6. 验收清单

- [x] 删除 A04_TODO_ITEMS / A05_TODO_ITEMS / A06_TODO_ITEMS / A07_TODO_ITEMS 或不再使用。
- [x] main() 不再打印大段 TODO。
- [x] A04 改为函数级 DLS TODO。
- [x] A05 改为函数级 QP-IK TODO。
- [x] A06 改为函数级 Target + Viewer TODO。
- [x] A07 改为函数级 Actuator TODO。
- [x] 完整说明已迁移到 Markdown。
- [x] 未实现算法。
- [x] 未启动 viewer。
- [x] 未写 data.ctrl。
- [x] 未生成 video。
- [ ] py_compile 通过。
