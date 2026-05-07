# Step 1 Audit Old And New Projects

## 1. Paths

- Old project path: `/home/ubuntu/robot_proj/Pinocchio_URDF`
- New project path: `/home/ubuntu/Robot_Dynamics_Control`
- Path check result: both paths exist.
- Audit working directory: `/home/ubuntu/Robot_Dynamics_Control`

## 2. Git Status At Audit Start

Current Git status recorded before creating this audit document:

```text
 M .gitignore
```

This audit step does not migrate, copy, move, delete, or refactor files.

## 3. Old Project Directory Overview

Old project first-level entries:

```text
.codex
.dockerignore
.git
.gitignore
.vscode
AGENT.MD
AGENTS.md
Dockerfile
README.md
controllers
docker-compose.yml
environment.yml
envs
experiments
main.py
models
notes
outputs
requirements.txt
scripts
src
tests
unitree_ros
```

Old project second-level overview:

```text
.vscode/settings.json
controllers/__init__.py
experiments/__init__.py
models/meshes
models/mjcf
models/urdf
notes/robot_script_builder_v1.md
outputs/.gitkeep
outputs/fk
outputs/ik
outputs/jacobian
outputs/mujoco_left_foot_ik_loop
outputs/mujoco_left_knee_perturb
outputs/task1_h1_model_check
outputs/task1_inspect_humanoid_model
scripts/__pycache__
scripts/check_docker_env.py
scripts/fk_h1.py
scripts/fk_h1_left_knee_perturb.py
scripts/ik_h1_left_foot_step1_target.py
scripts/ik_h1_left_foot_step2_jpos.py
scripts/ik_h1_left_foot_step3_one_step.py
scripts/ik_h1_left_foot_step4_apply_update.py
scripts/ik_h1_left_foot_step5_loop.py
scripts/jacobian_h1.py
scripts/jacobian_h1_check.py
scripts/mujoco_h1_left_knee_perturb.py
scripts/mujoco_h1_sim_learning.py
scripts/mujoco_playback_ik_traj.py
scripts/task1_inspect_humanoid_model.py
src/.gitkeep
unitree_ros/.git
unitree_ros/.gitignore
unitree_ros/.gitmodules
unitree_ros/LICENSE
unitree_ros/README.md
unitree_ros/robots
unitree_ros/unitree_controller
unitree_ros/unitree_gazebo
unitree_ros/unitree_legged_control
unitree_ros/unitree_ros_to_real
```

## 4. New Project Directory Overview

New project first-level entries:

```text
.codex
.git
.gitignore
.idea
.vscode
README.md
configs
controllers
debug.log
docs
dynamics
envs
experiments
exports
external
init_project.py
main.py
notes
outputs
plots
requirements.txt
scripts
src
test_mujoco.py
tests
utils
```

New project second-level overview:

```text
.idea/.gitignore
.idea/.name
.idea/Robot Dynamics Control.iml
.idea/inspectionProfiles
.idea/misc.xml
.idea/modules.xml
.idea/vcs.xml
.idea/workspace.xml
.vscode/settings.json
configs/README.md
controllers/__pycache__
controllers/pd_controller.py
docs/00_preparation
docs/00_project_management
docs/01_self_baseline
docs/02_legged_control
docs/03_unitree_rl_mjlab
docs/04_compare
docs/interview
envs/single_joint.xml
envs/two_link_arm.xml
experiments/exp_pd_joint_control.py
exports/word
external/legged_control_README.md
external/unitree_rl_mjlab_README.md
notes/day1_setup.md
outputs/README.md
outputs/exp_pd_joint_control
outputs/figures
outputs/logs
outputs/reports
outputs/videos
plots/pd_joint_position.png
plots/pd_joint_torque.png
scripts/README.md
scripts/export_md_to_docx.ps1
scripts/export_md_to_docx.sh
src/robot_baseline
tests/README.md
tests/test_mujoco_basic.py
```

## 5. Old Project Content That May Belong To A Self-Developed Baseline

Potential A baseline candidates, pending manual confirmation:

- `scripts/fk_h1.py`
- `scripts/fk_h1_left_knee_perturb.py`
- `scripts/jacobian_h1.py`
- `scripts/jacobian_h1_check.py`
- `scripts/ik_h1_left_foot_step1_target.py`
- `scripts/ik_h1_left_foot_step2_jpos.py`
- `scripts/ik_h1_left_foot_step3_one_step.py`
- `scripts/ik_h1_left_foot_step4_apply_update.py`
- `scripts/ik_h1_left_foot_step5_loop.py`
- `scripts/mujoco_h1_left_knee_perturb.py`
- `scripts/mujoco_h1_sim_learning.py`
- `scripts/mujoco_playback_ik_traj.py`
- `scripts/task1_inspect_humanoid_model.py`
- `scripts/check_docker_env.py`
- `controllers/`
- `envs/`
- `models/meshes`, `models/mjcf`, `models/urdf`
- `notes/robot_script_builder_v1.md`
- `README.md`, `requirements.txt`, `environment.yml`, `Dockerfile`, `docker-compose.yml`

The `models/` subdirectories exist, but no model files were found in the scan depth used for this audit. Their exact migration value is pending manual confirmation.

## 6. Old Project Content That Should Not Be Migrated Directly

The following should not be copied into the new project as source assets during migration:

- `.git/`: old repository metadata.
- `.codex/`: local agent/tool state.
- `.vscode/`: editor-local configuration unless manually converted into documented setup guidance.
- `scripts/__pycache__/`: Python cache files.
- `outputs/`: generated FK, Jacobian, IK, MuJoCo playback, and model inspection outputs.
- `unitree_ros/.git`: nested repository metadata.
- `unitree_ros/`: external upstream project content; should be referenced or vendored only after a separate decision.
- `Dockerfile`, `docker-compose.yml`, `environment.yml`: pending manual confirmation before migration because they may be environment setup assets rather than A baseline implementation.
- Any generated `.csv`, `.npy`, image, video, log, or timestamped output under `outputs/`.

## 7. Existing Preparation Documents In New Project

Preparation and planning documents already present:

```text
docs/00_preparation/00_preparation_overview.md
docs/00_preparation/01_job_target_and_skill_matrix.md
docs/00_preparation/02_three_project_scope.md
docs/00_preparation/03_preparation_task_table.md
docs/00_preparation/04_environment_requirements.md
docs/00_preparation/05_repository_structure_plan.md
docs/00_preparation/06_external_project_plan.md
docs/00_preparation/07_robot_model_and_asset_plan.md
docs/00_preparation/08_codex_workflow_rules.md
docs/00_preparation/09_risk_and_fallback_plan.md
docs/00_preparation/10_preparation_acceptance_checklist.md
docs/00_project_management/step0_5_pre_merge_git_cleanup.md
docs/01_self_baseline/README.md
docs/02_legged_control/README.md
docs/03_unitree_rl_mjlab/README.md
docs/04_compare/README.md
docs/interview/README.md
```

## 8. Existing New Project Source, Script, Config, Experiment, And Output Directories

Source and package areas:

- `controllers/pd_controller.py`
- `src/robot_baseline/README.md`
- `src/robot_baseline/__init__.py`
- `dynamics/` pending manual confirmation
- `utils/` pending manual confirmation

Scripts:

- `scripts/README.md`
- `scripts/export_md_to_docx.ps1`
- `scripts/export_md_to_docx.sh`

Configuration and model assets:

- `configs/README.md`
- `envs/single_joint.xml`
- `envs/two_link_arm.xml`

Experiments and tests:

- `experiments/exp_pd_joint_control.py`
- `tests/test_mujoco_basic.py`
- `tests/README.md`
- `test_mujoco.py` pending manual confirmation

External reference and export folders:

- `external/legged_control_README.md`
- `external/unitree_rl_mjlab_README.md`
- `exports/word/README.md`

Outputs:

- `outputs/README.md`
- `outputs/figures/README.md`
- `outputs/reports/README.md`
- `outputs/logs/README.md`
- `outputs/videos/README.md`
- `outputs/exp_pd_joint_control/` contains generated run results and should not be migrated as source.
- `plots/` contains generated images and is pending manual confirmation.

## 9. Pending Manual Confirmation

These directories or files require human decision before any migration:

- Whether old `scripts/fk_*`, `scripts/jacobian_*`, and `scripts/ik_*` should become A baseline learning scripts or remain only historical references.
- Whether old `models/meshes`, `models/mjcf`, and `models/urdf` are intentionally empty, external symlinks, or missing assets.
- Whether old `Dockerfile`, `docker-compose.yml`, `environment.yml`, and `requirements.txt` should be migrated into A baseline or only documented.
- Whether old `unitree_ros/` should remain an external reference only.
- Whether new `dynamics/`, `utils/`, `test_mujoco.py`, and root `plots/` are intentional project assets or local leftovers.
- Whether new `debug.log`, `.idea/`, `.vscode/`, and `.codex/` should remain ignored local state.

## 10. Next Step Suggestions

Before Step 2:

- Confirm the exact target naming and boundaries for A, B, and C projects.
- Confirm which old scripts are part of A self-developed baseline.
- Confirm whether any old model assets are missing or stored outside `models/`.
- Confirm that `outputs/` and timestamped experiment results are excluded from migration.
- Confirm whether external projects such as `unitree_ros` should be referenced by documentation instead of copied.
- Keep the current Git checkpoint clean before creating any new monorepo structure.
