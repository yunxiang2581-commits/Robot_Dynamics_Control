# Step B03-R4C-1F: State Tracking Smoke Figures

## 本步目标

B03-R4C-1F 的目标是在 R4C-1E 已经输出 CSV/NPZ 的基础上，补齐最小可视化：

```text
predicted_states + x_refs -> state tracking figure
cost_history -> cost history figure
```

本步仍然只做短 horizon state tracking smoke，不做 task-space tracking，不生成正式 MP4，不修改 B02 controller 或 B02 benchmark/regression 配置。

## 本步新增输出

脚本现在会在 figures 目录写入：

```text
outputs/figures/B03_R4C_two_link_state_trajectory_todo.png
outputs/figures/B03_R4C_ilqr_cost_history_todo.png
```

其中：

- `B03_R4C_two_link_state_trajectory_todo.png`：绘制 `q1/q2` 的 predicted vs reference；
- `B03_R4C_ilqr_cost_history_todo.png`：绘制 iLQR-lite cost history。

## 本步实现内容

更新 `plot_trajectory(...)`：

1. 检查 `solution.predicted_states` 存在；
2. 检查 `x_ref` 与预测状态行数匹配；
3. 绘制两个子图：`q1` 和 `q2`；
4. 每个子图显示 reference 与 predicted；
5. 标注最终关节误差；
6. 保存 PNG。

更新 `plot_cost_history(...)`：

1. 从 `solution.metadata["cost_history"]` 读取 cost；
2. 绘制 iteration -> total cost 曲线；
3. 标注 final cost；
4. 保存 PNG。

## 验证命令

```bash
D:\anaconda\envs\mujoco_py311\python.exe -m py_compile \
  projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqr_lite_two_link_smoke.py

D:\anaconda\envs\mujoco_py311\python.exe -m pytest -q \
  projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_smoke_skeleton.py \
  projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_state_adapter.py \
  projects/B_mujoco_mpc_study/tests/test_B03_ilqg_solver.py
```

真实 MuJoCo smoke：

```bash
D:\anaconda\envs\mujoco_py311\python.exe \
  projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqr_lite_two_link_smoke.py \
  --output-dir outputs/pytest_tmp/B03_R4C1F_real_smoke \
  --log-level INFO
```

示例验证结果：

```text
B03_R4C_ilqr_cost_history_todo.png          42281 bytes, 1176x651
B03_R4C_two_link_state_trajectory_todo.png  66404 bytes, 1606x591
success True
horizon 32
best_cost 1.5449372487697226e-07
```

## 当前仍保留的 TODO

- 文件名仍带 `_todo`，因为 R4C 还处于 smoke 学习线；
- 只画 `q1/q2`，还没有画 `dq1/dq2`；
- 还没有 task-space end-effector tracking；
- 还没有 CEM/MPPI warm-start；
- 还没有 long benchmark；
- 还没有正式 MP4。

## 下一步 B03-R4C-1G

建议下一步补一个更完整但仍轻量的 smoke report：

1. 汇总 metrics CSV；
2. 列出生成的 figures/cache 路径；
3. 写入 `reports/B03_R4C_ilqr_two_link_smoke_report.md`；
4. 把 R4C 当前状态同步回 B03 文档。
