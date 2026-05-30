# Step R-C Deletion Inventory Before Refactor

## 1. 删除原则

本次 Step R-C 是结构重构与状态收口，不做完整 IK、viewer 或 actuator tracking 实现。删除只允许发生在明确白名单内，用来清理旧 Step 执行记录、旧 A04/A05 运行产物，以及已经被四接口文档替代的旧 A04/A05 单脚本文档。

允许删除的类别：

- 旧 Step 执行记录 Markdown：`docs/00_project_management/step*.md`、`Step*.md`、`*step*.md`、`*Step*.md`。
- 旧 A04-A07 运行产物：`outputs/trajectories/A04_*.npy`、`A05_*.npy`，`outputs/reports/A04_*.md`、`A05_*.md`，以及同类 logs/figures/cache 中的 A04-A07 文件。
- 与旧 A04-A07 执行路线强绑定、且会被新四接口文档替代的 A 项目 Markdown。

## 2. 保留原则

必须保留基础验证层和主线资产：

- A00-A03 脚本和对应 A01-A03 输出。
- `projects/A_self_baseline/src/robot_baseline/`。
- `projects/A_self_baseline/configs/`。
- `shared/robot_assets/`。
- `projects/A_self_baseline/external/mink/`。
- `external/mink_upstream/`。
- `legacy_imported/`。
- README 与项目管理稳定文档。

## 3. 删除候选

本次审计发现的删除候选如下。

旧 Step 执行记录：

- `docs/00_project_management/step14A_A06_target_mocap_tracking_todo_skeleton.md`
- `docs/00_project_management/stepR_A_mink_style_unified_motion_interface_refactor.md`

旧 A04/A05 运行产物：

- `projects/A_self_baseline/outputs/trajectories/A04_dls_ik_q_traj.npy`
- `projects/A_self_baseline/outputs/trajectories/A05_qp_ik_q_traj.npy`
- `projects/A_self_baseline/outputs/reports/A04_dls_ik_report.md`
- `projects/A_self_baseline/outputs/reports/A05_qp_ik_report.md`

旧 A04/A05 单脚本文档：

- `projects/A_self_baseline/docs/04_dls_differential_ik.md`
- `projects/A_self_baseline/docs/05_task_limit_qp_ik.md`

未发现但属于白名单模式的候选：

- `projects/A_self_baseline/docs/06_target_mocap_tracking.md`
- `projects/A_self_baseline/docs/07_mujoco_actuator_tracking.md`
- `projects/A_self_baseline/outputs/logs/A04_*.csv`
- `projects/A_self_baseline/outputs/logs/A05_*.csv`
- `projects/A_self_baseline/outputs/logs/A06_*.csv`
- `projects/A_self_baseline/outputs/logs/A07_*.csv`
- `projects/A_self_baseline/outputs/figures/A04_*.png`
- `projects/A_self_baseline/outputs/figures/A05_*.png`
- `projects/A_self_baseline/outputs/figures/A06_*.png`
- `projects/A_self_baseline/outputs/figures/A07_*.png`
- `projects/A_self_baseline/outputs/cache/A04_*.json`
- `projects/A_self_baseline/outputs/cache/A05_*.json`
- `projects/A_self_baseline/outputs/cache/A06_*.json`
- `projects/A_self_baseline/outputs/cache/A07_*.json`

## 4. 实际删除白名单

本次只删除以下已确认存在的文件：

- `docs/00_project_management/step14A_A06_target_mocap_tracking_todo_skeleton.md`
- `docs/00_project_management/stepR_A_mink_style_unified_motion_interface_refactor.md`
- `projects/A_self_baseline/outputs/trajectories/A04_dls_ik_q_traj.npy`
- `projects/A_self_baseline/outputs/trajectories/A05_qp_ik_q_traj.npy`
- `projects/A_self_baseline/outputs/reports/A04_dls_ik_report.md`
- `projects/A_self_baseline/outputs/reports/A05_qp_ik_report.md`
- `projects/A_self_baseline/docs/04_dls_differential_ik.md`
- `projects/A_self_baseline/docs/05_task_limit_qp_ik.md`

## 5. 明确不删除清单

以下文件和目录禁止删除：

- `README.md`
- `projects/A_self_baseline/README.md`
- `projects/A_self_baseline/scripts/00_reference_and_assets.py`
- `projects/A_self_baseline/scripts/01_model_inspect.py`
- `projects/A_self_baseline/scripts/02_configuration_site_pose.py`
- `projects/A_self_baseline/scripts/03_site_jacobian_check.py`
- `projects/A_self_baseline/scripts/04_ik_dls_wrapper.py`
- `projects/A_self_baseline/scripts/05_ik_qp_wrapper.py`
- `projects/A_self_baseline/scripts/06_target_viewer_wrapper.py`
- `projects/A_self_baseline/scripts/07_actuator_wrapper.py`
- `projects/A_self_baseline/src/robot_baseline/`
- `projects/A_self_baseline/configs/`
- `shared/robot_assets/`
- `external/mink_upstream/`
- `projects/A_self_baseline/external/mink/`
- `legacy_imported/`
- `.gitignore`
- `docs/00_project_management/PROJECT_STATUS.md`
- `docs/00_project_management/PROJECT_STRUCTURE.md`
- `docs/00_project_management/PROJECT_ROADMAP.md`
- `docs/00_project_management/DOC_PLACEMENT_RULES.md`
- `docs/00_project_management/DOC_MIGRATION_CANDIDATES.md`
- `docs/00_project_management/stepR_C_deletion_inventory_before_refactor.md`
- `docs/00_project_management/stepR_C_complete_refactor_report.md`

## 6. 删除后如何验证

删除后运行：

```bash
git status --short
```

并确认：

- 删除项只来自实际删除白名单。
- A01-A03 脚本与 A01-A03 输出仍存在。
- `shared/robot_assets/`、`external/mink_upstream/`、`projects/A_self_baseline/external/mink/` 未修改。
- 没有删除整个 `docs/`、`outputs/` 或 `configs/` 目录。

## 7. 风险说明

- 删除 A04/A05 旧运行产物后，A04/A05 的旧最小实现结果不再作为当前主线证据；新主线以四接口 TODO skeleton 为准。
- 如果后续需要复查旧结果，可以从 git 历史恢复。
- 本次不会删除 A01-A03 输出，因为它们是 foundation validation layer 的验证证据。

## 8. 删除执行结果

已执行逐项白名单删除。

删除结果：

- 已删除 `docs/00_project_management/step14A_A06_target_mocap_tracking_todo_skeleton.md`
- 已删除 `docs/00_project_management/stepR_A_mink_style_unified_motion_interface_refactor.md`
- 已删除 `projects/A_self_baseline/outputs/trajectories/A04_dls_ik_q_traj.npy`
- 已删除 `projects/A_self_baseline/outputs/trajectories/A05_qp_ik_q_traj.npy`
- 已删除 `projects/A_self_baseline/outputs/reports/A04_dls_ik_report.md`
- 已删除 `projects/A_self_baseline/outputs/reports/A05_qp_ik_report.md`
- 已删除 `projects/A_self_baseline/docs/04_dls_differential_ik.md`
- 已删除 `projects/A_self_baseline/docs/05_task_limit_qp_ik.md`

删除后 `git status --short` 显示的删除项与本白名单一致。A01-A03 脚本、A01-A03 输出、`shared/robot_assets/`、`external/mink_upstream/`、`projects/A_self_baseline/external/mink/` 均未进入删除白名单。
