# Step 3 Migrate A Assets

## 1. Step 3 Goal

Migrate lightweight, reusable assets from `/home/ubuntu/robot_proj/Pinocchio_URDF` into the A self-developed baseline container:

`/home/ubuntu/Robot_Dynamics_Control/projects/A_self_baseline/`

This step keeps all imported legacy scripts under `legacy_imported/` so they can be reviewed before becoming standard A project scripts. No old script was refactored and no FK, Jacobian, IK, QP, WBC, or RL algorithm was implemented.

## 2. Files Actually Migrated

Legacy scripts copied to `projects/A_self_baseline/scripts/legacy_imported/`:

```text
check_docker_env.py
fk_h1.py
fk_h1_left_knee_perturb.py
ik_h1_left_foot_step1_target.py
ik_h1_left_foot_step2_jpos.py
ik_h1_left_foot_step3_one_step.py
ik_h1_left_foot_step4_apply_update.py
ik_h1_left_foot_step5_loop.py
jacobian_h1.py
jacobian_h1_check.py
mujoco_h1_left_knee_perturb.py
mujoco_h1_sim_learning.py
mujoco_playback_ik_traj.py
task1_inspect_humanoid_model.py
```

Legacy source placeholder copied to `projects/A_self_baseline/src/legacy_imported/`:

```text
.gitkeep
```

Legacy controllers copied to `projects/A_self_baseline/controllers/legacy_imported/`:

```text
__init__.py
```

Legacy experiments copied to `projects/A_self_baseline/experiments/legacy_imported/`:

```text
__init__.py
```

Legacy notes copied to `projects/A_self_baseline/notes/legacy_imported/`:

```text
robot_script_builder_v1.md
```

Legacy project documents copied to `projects/A_self_baseline/docs/legacy_imported/`:

```text
README_from_Pinocchio_URDF.md
AGENTS_from_Pinocchio_URDF.md
AGENT_from_Pinocchio_URDF.md
```

Environment files copied to `shared/env/`:

```text
pinocchio_urdf_environment.yml
pinocchio_urdf_requirements.txt
pinocchio_urdf_docker/.dockerignore
pinocchio_urdf_docker/Dockerfile
pinocchio_urdf_docker/docker-compose.yml
```

Small non-timestamp output samples copied to `projects/A_self_baseline/outputs/legacy_samples/`:

```text
fk/h1_fk_report.txt
ik/h1_left_foot_step1_target.txt
ik/h1_left_foot_step2_jpos.txt
ik/h1_left_foot_step3_one_step.txt
ik/h1_left_foot_step4_apply_update.txt
ik/h1_left_foot_step5_loop.txt
jacobian/h1_left_foot_jacobian.txt
jacobian/h1_left_foot_jacobian_check.png
jacobian/h1_left_foot_jacobian_check.txt
```

## 3. Skipped Directories

The following old project directories were intentionally not copied:

- `.git/`: repository metadata.
- `.codex/`: local agent/tool state.
- `.vscode/`: editor-local settings.
- `unitree_ros/`: external upstream repository, not A self-developed baseline source.
- `scripts/__pycache__/`: Python cache.
- `outputs/` timestamped run directories: generated experiment results.

The following requested source areas did not exist in the old project at migration time:

- `/home/ubuntu/robot_proj/Pinocchio_URDF/configs`
- `/home/ubuntu/robot_proj/Pinocchio_URDF/docs`

The following old project areas had no matching files for the requested copy rule:

- `tests/`: no `.py` files found.
- `envs/`: no `.xml` or `.md` files found.
- `models/`: no files found during this step.

## 4. Files Skipped Because They Were Large, Generated, Or Unclear

Generated data skipped:

```text
outputs/ik/h1_left_foot_step5_errors.csv
outputs/ik/h1_left_foot_step5_q_traj.npy
outputs/mujoco_left_foot_ik_loop/*/error_history.csv
outputs/mujoco_left_knee_perturb/*/timeseries.csv
```

Timestamped output samples skipped pending manual confirmation:

```text
outputs/fk/20260421_171542/
outputs/fk/20260421_171917/
outputs/fk/20260421_180804/
outputs/mujoco_left_foot_ik_loop/20260423_190016/
outputs/mujoco_left_foot_ik_loop/20260423_190046/
outputs/mujoco_left_foot_ik_loop/20260423_190106/
outputs/mujoco_left_foot_ik_loop/20260424_154026/
outputs/mujoco_left_foot_ik_loop/20260424_154027/
outputs/mujoco_left_foot_ik_loop/20260424_154044/
outputs/mujoco_left_foot_ik_loop/20260424_154319/
outputs/mujoco_left_knee_perturb/20260421_172356/
outputs/mujoco_left_knee_perturb/20260421_172834/
outputs/mujoco_left_knee_perturb/20260421_172957/
outputs/mujoco_left_knee_perturb/20260421_173101/
outputs/mujoco_left_knee_perturb/20260421_173231/
outputs/mujoco_left_knee_perturb/20260421_175219/
outputs/mujoco_left_knee_perturb/20260421_175349/
outputs/mujoco_left_knee_perturb/20260421_180019/
outputs/mujoco_left_knee_perturb/20260427_154729/
outputs/task1_h1_model_check/20260420_142338/
outputs/task1_h1_model_check/20260421_162511/
outputs/task1_inspect_humanoid_model/20260418_183056/
```

No model files were found under `models/`, so no model file was copied to `shared/robot_assets/models/`.

## 5. File Conflicts

No target file conflicts were detected during this migration. The copy process used a non-overwrite policy; if a target path already existed, it would have been recorded as a conflict instead of overwritten.

## 6. Why unitree_ros Was Not Migrated As A Whole

`unitree_ros/` is a large external upstream project with its own repository metadata, ROS packages, robot descriptions, controllers, Gazebo plugins, and bridge code. It is not the A self-developed baseline implementation.

Migrating it wholesale would mix external reproduction material with the self-developed baseline and would make review, licensing, dependency management, and repository size harder to control. It should remain an external reference unless a later step explicitly decides how to reference or vendor selected files.

## 7. How Legacy Scripts Should Be Refactored Later

Later steps should treat `projects/A_self_baseline/scripts/legacy_imported/` as read-only source material first:

1. Review each script's learning purpose and dependencies.
2. Decide whether it belongs to URDF inspection, FK, Jacobian, IK, MuJoCo playback, or environment checking.
3. Create a clean standard A script outside `legacy_imported/`.
4. Preserve useful comments and teaching output.
5. Update paths to the A project layout.
6. Add tests or reproducible text checks where useful.
7. Only then retire or archive the corresponding legacy script.

## 8. Acceptance Checklist

- [x] Legacy scripts were copied under `projects/A_self_baseline/scripts/legacy_imported/`.
- [x] A project standard script directories were not mixed with migrated script files.
- [x] Old `README.md`, `AGENTS.md`, and `AGENT.MD` were copied into A legacy docs with renamed filenames.
- [x] Environment files were copied under `shared/env/`.
- [x] Docker-related files were copied under `shared/env/pinocchio_urdf_docker/`.
- [x] No `.git/`, `.codex/`, `.vscode/`, `unitree_ros/`, `third_party/`, cache, virtual environment, videos, logs, weights, or training results were copied.
- [x] No old file was refactored.
- [x] No robot control algorithm was implemented.
- [x] Skipped and unclear files are recorded for manual confirmation.
