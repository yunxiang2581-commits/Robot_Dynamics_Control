# Step 3 Migration TODO

## Model Files To Confirm

- `models/meshes/`: directory exists in the old project, but no files were found during this migration.
- `models/mjcf/`: directory exists in the old project, but no files were found during this migration.
- `models/urdf/`: directory exists in the old project, but no files were found during this migration.

Manual confirmation needed:

- Are model files stored outside `/home/ubuntu/robot_proj/Pinocchio_URDF/models/`?
- Are the model directories intentionally empty?
- Should selected robot descriptions be sourced later from `unitree_ros/robots/` or another external asset location?

## Output Files To Confirm

Generated data skipped:

```text
outputs/ik/h1_left_foot_step5_errors.csv
outputs/ik/h1_left_foot_step5_q_traj.npy
outputs/mujoco_left_foot_ik_loop/*/error_history.csv
outputs/mujoco_left_knee_perturb/*/timeseries.csv
```

Timestamped output directories skipped:

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

Manual confirmation needed:

- Which plots or reports are useful as small curated examples?
- Should any skipped `.txt`, `.md`, or `.png` files be copied later as documentation evidence?
- Should CSV/NPY files remain local-only generated artifacts?

## Environment Files To Confirm

Copied for review:

```text
shared/env/pinocchio_urdf_environment.yml
shared/env/pinocchio_urdf_requirements.txt
shared/env/pinocchio_urdf_docker/.dockerignore
shared/env/pinocchio_urdf_docker/Dockerfile
shared/env/pinocchio_urdf_docker/docker-compose.yml
```

Manual confirmation needed:

- Should A use Conda, pip, Docker, or a combination?
- Are the old Docker files still valid for the new repository layout?
- Should these environment files stay in `shared/env/` or be copied into A after cleanup?

## Legacy Script Purpose To Confirm

Legacy scripts copied for review:

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

Manual confirmation needed:

- Which scripts become A standard TODO learning scripts?
- Which scripts are historical references only?
- Which scripts depend on missing robot model paths?
- Which scripts should be split into URDF inspection, FK, Jacobian, IK, MuJoCo playback, and environment check categories?
