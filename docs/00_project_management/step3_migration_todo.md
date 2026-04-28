# Step 3 Migration TODO

## Model Files To Confirm

Resolved:

- Selected H1 assets are now available under `shared/robot_assets/models/h1_description/`.
- Confirmed files include:
  - `urdf/h1.urdf`
  - `urdf/h1_with_hand.urdf`
  - `mjcf/h1.xml`
  - `mjcf/h1_with_hand.xml`
  - `mjcf/scene.xml`
  - `mjcf/scene_with_hand_bright.xml`
  - `meshes/` with 98 `.STL` and `.dae` files
  - `package.xml`

Remaining manual confirmation:

- Whether A project should standardize on `h1.urdf` or `h1_with_hand.urdf` as the default Pinocchio model.
- Whether `launch/`, `doc/`, `README.md`, and `CMakeLists.txt` should stay in Git or be treated as optional reference-only files later.
- Whether other robot packages from the old `unitree_ros/robots/` tree need selective import later.

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
