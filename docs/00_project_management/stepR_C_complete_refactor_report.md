# Step R-C Complete Refactor Report

## 1. 目标

Step R-C 将 A 项目收口为 foundation validation layer + unified motion interface layer。

Foundation validation layer 保留：

- A00 reference and assets。
- A01 model inspect。
- A02 configuration / site pose。
- A03 site Jacobian check。

Unified motion interface layer 新增：

- Target interface。
- IK interface。
- Viewer interface。
- Actuator interface。

## 2. 参考 mink，但不调用 mink

mink example 的数据流是：

```text
mocap target -> FrameTask/PostureTask -> limits -> solve_ik -> viewer -> data.ctrl
```

A 项目只学习这个结构，不调用 mink 替代自己的实现。

## 3. 本次新增文件

- `projects/A_self_baseline/src/robot_baseline/motion_types.py`
- `projects/A_self_baseline/src/robot_baseline/target_interface.py`
- `projects/A_self_baseline/src/robot_baseline/ik_interface.py`
- `projects/A_self_baseline/src/robot_baseline/trajectory_io.py`
- `projects/A_self_baseline/src/robot_baseline/viewer_interface.py`
- `projects/A_self_baseline/src/robot_baseline/actuator_interface.py`

## 4. A04-A07 映射

- A04 = `solver_type=dls` 的 IK wrapper。
- A05 = `solver_type=qp_scipy` / `qp_osqp` 的 IK wrapper。
- A06 = Target interface + Viewer interface wrapper。
- A07 = Actuator interface wrapper。

## 5. 清理内容

删除了旧 Step 执行记录、旧 A04/A05 运行产物、旧 A04/A05 单脚本文档。删除清单见：

- `docs/00_project_management/stepR_C_deletion_inventory_before_refactor.md`

## 6. 当前只保留 TODO 的功能

- TargetDefinition 完整 load/save/validate。
- A05 target_definition_json 回归。
- A06 fixed target 最小实现。
- A06 派生 MJCF。
- A06 keyboard target movement。
- A06 mouse drag target validation。
- A06 kinematic IK follow。
- A07 actuator tracking。
- video demo。

## 7. 下一步

R1: 完成 TargetDefinition 的 load/save/validate。

## 8. 禁止事项确认

- 未调用 mink。
- 未修改 external/mink_upstream。
- 未修改 legacy_imported。
- 未修改 shared/robot_assets 中的原始 UR5e scene.xml。
- 未创建派生 MJCF。
- 未启动 viewer。
- 未写 data.ctrl。
- 未运行 actuator control loop。
- 未录制 video。
- 未执行 git add / commit / push。
