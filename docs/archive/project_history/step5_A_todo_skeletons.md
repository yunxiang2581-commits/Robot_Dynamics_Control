# Step 5 A TODO Skeletons

## 1. Step 5 Goal

Create standard TODO learning script skeletons, module skeletons, configuration templates, and documentation for `projects/A_self_baseline/`.

This step prepares the A project learning path without implementing robot control algorithms.

## 2. New Script List

```text
projects/A_self_baseline/scripts/01_model_inspect.py
projects/A_self_baseline/scripts/02_configuration_site_pose.py
projects/A_self_baseline/scripts/03_site_jacobian_check.py
projects/A_self_baseline/scripts/04_dls_differential_ik.py
projects/A_self_baseline/scripts/05_task_limit_qp_ik.py
projects/A_self_baseline/scripts/06_target_mocap_tracking.py
projects/A_self_baseline/scripts/07_mujoco_actuator_tracking.py
```

## 3. New Module List

```text
projects/A_self_baseline/src/robot_baseline/__init__.py
projects/A_self_baseline/src/robot_baseline/model_loader.py
projects/A_self_baseline/src/robot_baseline/kinematics.py
projects/A_self_baseline/src/robot_baseline/jacobian_check.py
projects/A_self_baseline/src/robot_baseline/ik.py
projects/A_self_baseline/src/robot_baseline/qp_ik.py
projects/A_self_baseline/src/robot_baseline/pd_controller.py
projects/A_self_baseline/src/robot_baseline/mini_wbc.py
projects/A_self_baseline/src/robot_baseline/metrics.py
```

## 4. New Config List

```text
projects/A_self_baseline/configs/robot.yaml
projects/A_self_baseline/configs/ik.yaml
projects/A_self_baseline/configs/qp_ik.yaml
projects/A_self_baseline/configs/mujoco_pd.yaml
projects/A_self_baseline/configs/mini_wbc.yaml
```

## 5. New Document List

```text
projects/A_self_baseline/docs/legacy_script_mapping.md
projects/A_self_baseline/docs/self_baseline_learning_scripts_plan.md
docs/00_project_management/step5_A_todo_skeletons.md
```

## 6. Algorithms Not Implemented

The new files intentionally do not implement full FK, Jacobian, IK, QP-IK, MuJoCo PD control, Mini-WBC, or RL logic.

Core computation blocks raise `NotImplementedError` and contain Chinese TODO teaching notes describing the implementation target, job relevance, recommended APIs, inputs, outputs, and validation method.

## 7. Acceptance Checklist

- [x] Seven standard A learning scripts were created.
- [x] Each script has a docstring, argparse, logging, `main()`, and entry guard.
- [x] Each script contains Chinese TODO teaching notes.
- [x] `src/robot_baseline/` module skeletons were created with typed function signatures.
- [x] Config templates were created without old absolute paths.
- [x] Legacy mapping and learning plan documents were created.
- [x] A README was updated with scripts, modules, configs, status, legacy role, and implementation order.
- [x] No legacy script was modified.
- [x] No algorithm was implemented.
