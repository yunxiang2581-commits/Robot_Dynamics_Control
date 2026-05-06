# Step 9C - Rename A Scripts To Mink-Style Pipeline

## 1. 为什么统一脚本命名

A 项目当前主线已经从早期 H1/URDF 学习入口，转为对标 mink UR5e 示例的 6-DOF manipulator 教学 baseline。旧入口名称仍带有 `inspect_urdf`、`fk_frame_pose`、`mini_wbc` 等历史含义，容易让后续实现范围发散。

Step 9C 的目标是把 `scripts/` 统一成 A00-A10 mink-style pipeline，让每个入口只表达当前步骤的学习职责、输入输出和未来 TODO，不提前实现 FK、Jacobian、IK、QP、WBC、MuJoCo 控制或视频录制。

## 2. 旧脚本名到新脚本名映射

| 旧入口 | 新入口 | 新定位 |
|---|---|---|
| `01_inspect_urdf.py` | `01_model_inspect.py` | A01 model inspect |
| `02_fk_frame_pose.py` | `02_configuration_site_pose.py` | A02 configuration / site pose |
| `03_jacobian_fd_check.py` | `03_site_jacobian_check.py` | A03 site Jacobian check |
| `04_dls_ik_demo.py` | `04_dls_differential_ik.py` | A04 DLS differential IK |
| `05_qp_ik_joint_limit_demo.py` | `05_task_limit_qp_ik.py` | A05 task + limit + QP-IK |
| `06_mujoco_pd_tracking.py` | `06_target_mocap_tracking.py` | A06 target / mocap-style tracking |
| `07_mini_wbc_qp_demo.py` | `07_mujoco_actuator_tracking.py` | A07 MuJoCo actuator tracking |

新增入口：

- `00_reference_and_assets.py`
- `08_collision_avoidance_todo.py`
- `09_comparison_report.py`
- `10_demo_showcase_video.py`

## 3. A00-A10 Pipeline

```text
A00 reference and assets
  -> A01 model inspect
  -> A02 configuration / site pose
  -> A03 site Jacobian check
  -> A04 DLS differential IK
  -> A05 task + limit + QP-IK
  -> A06 target / mocap-style tracking
  -> A07 MuJoCo actuator tracking
  -> A08 collision avoidance TODO
  -> A09 comparison report
  -> A10 demo showcase / video recording
```

## 4. pinocchio-learning Skill 约束如何体现

已确认本仓库存在本地 skill：

```text
tools/codex_skills/pinocchio-learning/SKILL.md
```

脚本 TODO 按该 skill 和 AGENTS.md 约束组织：

- 使用 `pathlib.Path` 和 `__file__` 定位项目路径。
- 保留 `argparse`、`logging`、清晰 `main()` 和入口保护。
- 核心算法位置只写中文教学 TODO，不写完整实现。
- TODO 说明要实现什么、为什么需要、对标 mink/机器人概念、推荐 API、输入、输出和验证方式。
- 关键步骤保留 `NotImplementedError`，避免误以为已有完整实现。
- A04 已完成最小 position-mode DLS differential IK。
- A05 已完成最小 box-constrained QP-IK。
- A08 说明 collision avoidance 为什么放在 QP-IK 之后。

## 5. 修改文件清单

脚本：

- `projects/A_self_baseline/scripts/00_reference_and_assets.py`
- `projects/A_self_baseline/scripts/01_model_inspect.py`
- `projects/A_self_baseline/scripts/02_configuration_site_pose.py`
- `projects/A_self_baseline/scripts/03_site_jacobian_check.py`
- `projects/A_self_baseline/scripts/04_dls_differential_ik.py`
- `projects/A_self_baseline/scripts/05_task_limit_qp_ik.py`
- `projects/A_self_baseline/scripts/06_target_mocap_tracking.py`
- `projects/A_self_baseline/scripts/07_mujoco_actuator_tracking.py`
- `projects/A_self_baseline/scripts/08_collision_avoidance_todo.py`
- `projects/A_self_baseline/scripts/09_comparison_report.py`
- `projects/A_self_baseline/scripts/10_demo_showcase_video.py`

文档：

- `projects/A_self_baseline/README.md`
- `projects/A_self_baseline/docs/A_pipeline_contract.md`
- `projects/A_self_baseline/docs/A_mink_alignment_plan.md`
- `projects/A_self_baseline/docs/mink_capability_vs_A_requirements.md`
- `projects/A_self_baseline/docs/reference_mink_ur5e.md`
- `projects/A_self_baseline/docs/01_model_inspect.md`
- `docs/00_project_management/step8_11_define_mink_capability_and_A_requirements.md`
- `docs/00_project_management/step9C_rename_A_scripts_to_mink_pipeline.md`

## 6. 未实现算法说明

本步骤未实现：

- FK
- Jacobian
- IK
- QP
- WBC
- MuJoCo 控制
- collision avoidance
- video recording

本步骤也未调用 mink 替代自己的实现，未修改 `external/mink_upstream`，未修改 `legacy_imported`，未执行 `git add`、`git commit` 或 `git push`。

## 7. 下一步

后续 Step 9B 已完成 A01 model inspect 最小实现；当前 A01-A05 最小主线已完成。

- 读取 `configs/robot.yaml`。
- 解析 `scene.xml` 路径。
- 使用 `mujoco.MjModel.from_xml_path` 加载模型。
- 使用 `mj_id2name` 枚举 joint/body/site/actuator/keyframe。
- 输出 `reports/A01_model_inspect_report.md` 和 `cache/A01_model_summary.json`。

## 8. 验收清单

- [x] 旧 A01-A07 入口已重命名。
- [x] A00/A08/A09/A10 TODO skeleton 已新增。
- [x] A00-A10 pipeline 已写入 README 和核心文档。
- [x] 当时所有标准脚本为 TODO learning skeleton；当前 A01-A05 已推进到最小实现。
- [x] 所有核心 TODO 为中文教学型 TODO。
- [x] 未实现算法。
- [x] 未修改 `external/mink_upstream`。
- [x] 未修改 `legacy_imported`。
- [x] 后续已进入并完成 A01 model inspect 最小实现。
