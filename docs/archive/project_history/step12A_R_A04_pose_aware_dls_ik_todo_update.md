# Step 12A-R A04 Pose-Aware DLS IK TODO Update

当前状态更新：A04 已完成最小 position-mode DLS differential IK。本文档记录的是 pose-aware 接口规划的历史补充。

## 1. 为什么 A04 要从 position-only 规划扩展到 pose-aware

A04 的第一版学习目标仍然是 position-only DLS differential IK。这个版本足够验证 A03 已确认的 `site velocity = J(q) dq`，并学习误差、Jacobian、damping、gain 和迭代更新之间的关系。

但机械臂末端任务通常不只是到达某个点，还需要以正确姿态到达。抓取、插入、对接和工具操作都依赖 orientation。A05 对标 mink 的 `FrameTask`，而 `FrameTask` 是 position + orientation 的 pose task，因此 A04 需要提前把 pose-aware 接口和算法边界规划清楚。

## 2. 本次保留了哪些已有 A04 代码

本次保留了 `projects/A_self_baseline/scripts/04_dls_differential_ik.py` 中已有的：

- 顶部 A04 docstring。
- `argparse` 参数结构。
- logging 配置。
- `main()` 入口。
- A01 / A02 / A03 输入路径参数。
- A04 future output 路径规划。
- TODO 1-14 教学注释。
- `NotImplementedError` 边界。

当前 A04 已完成最小 position-mode DLS IK；pose_6d 仍作为后续扩展边界保留。

## 3. 本次只做了哪些增量修改

本次只做 pose-aware 规划增量：

- 追加 `position` / `pose_6d` task mode 参数。
- 追加 target orientation mode 参数。
- 追加 position / orientation weight 参数。
- 在脚本 docstring 中追加 pose-aware DLS 公式。
- 在脚本注释中追加 target orientation、orientation error、task error、task Jacobian 和 error log TODO。
- 在 A04 Markdown 中追加 SO(3) rotation error、6D pose error、6D task Jacobian 和 pose-aware DLS 说明。
- 追加 README / pipeline 文档状态说明。

## 4. 新增 task-mode / orientation 参数

新增 CLI 规划参数：

- `--task-mode`
  - `position`: 只使用 `e_pos` 和 `J_pos`。
  - `pose_6d`: 未来组合 `e_pos/e_rot` 和 `J_pos/J_rot`。
- `--target-position-offset`
  - 默认 `"0.05,0.00,0.00"`。
- `--target-orientation-mode`
  - `keep_current`: `R_target = R_current`。
  - `fixed_rpy`: 未来从 roll/pitch/yaw 得到目标姿态。
  - `fixed_quat`: 未来从 quaternion 得到目标姿态。
- `--position-weight`
  - 默认 `1.0`。
- `--orientation-weight`
  - 默认 `0.2`。

orientation weight 必须存在，因为 position 单位是 m，rotation 单位是 rad，不能不加权直接拼接。

## 5. Markdown 中新增的旋转算法细节

`projects/A_self_baseline/docs/04_dls_differential_ik.md` 新增了：

- 为什么 A04 要考虑旋转。
- 为什么不能用欧拉角直接相减。
- SO(3) rotation error：
  - `R_err = R_target R_current^T`
  - `e_rot = log(R_err)`
- 6D pose error：
  - `e_task = [w_pos e_pos; w_rot e_rot]`
- 6D task Jacobian：
  - `J_task = [w_pos J_pos; w_rot J_rot]`
- pose-aware DLS：
  - `dq = J_task.T @ solve(J_task J_task.T + lambda I, gain e_task)`
- pose-aware 符号表、伪代码、验证标准和常见错误。

## 6. A04 对 A05/A06/A07 的后续接口影响

A05 后续需要接收更清晰的 task 定义：

- `task_mode`
- target position
- target orientation
- position / orientation weights
- `J_task`
- `e_task`

A06 后续需要同步 target pose 来源：

- fixed target position。
- future mocap target position。
- future target orientation。

A07 后续仍然只消费 A04/A05 生成的 `q_traj` 或 `q_des`，但 report 中应说明该轨迹来自 position mode 还是 pose_6d mode。

## 7. 未实现算法说明

本步骤没有实现：

- pose_6d IK。
- SO(3) log map。
- QP-IK。
- actuator tracking。
- MuJoCo 控制。
- collision avoidance。
- video recording。
- mink 调用替代自己的实现。

## 8. 下一步建议

建议继续按顺序推进：

1. Step 12B：A04 position mode 最小实现已完成并验证。
2. Step 12C：在 position mode 稳定后扩展 pose_6d mode。
3. A05：在 A04 task 定义基础上加入 task + limit + QP-IK。
4. A06/A07：同步 target pose / orientation task / trajectory_source。

## 9. 验收清单

- [x] 保留已有 A04 代码。
- [x] 未整体重写 `04_dls_differential_ik.py`。
- [x] A04 TODO / 接口支持 position / pose_6d 两种规划。
- [x] 增加 orientation error TODO。
- [x] 增加 J_rot / J_task TODO。
- [x] Markdown 讲解 SO(3) rotation error。
- [x] Markdown 讲解 6D pose DLS。
- [x] 未实现 QP。
- [x] 未实现 actuator tracking。
- [x] 未修改 external/mink_upstream。
- [x] 未修改 legacy_imported。
- [x] py_compile 通过。
