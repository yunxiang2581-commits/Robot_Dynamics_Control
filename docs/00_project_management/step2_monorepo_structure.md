# Step 2 Monorepo Structure

## 1. Step 2 Goal

Create independent directory containers for the three planned project tracks and shared utility areas. This step only establishes the monorepo skeleton and README placeholders.

No legacy files are migrated in this step.

## 2. Why Use A/B/C Project Structure

The repository will contain three different learning and implementation tracks. Keeping them under `projects/` makes their boundaries explicit and avoids mixing self-developed baseline code with external project reproduction notes or reinforcement learning experiments.

The structure also makes later migration review easier because each file can be assigned to one project track before it is copied or rewritten.

## 3. A/B/C Project Boundaries

- `projects/A_self_baseline/`: self-developed robot motion control baseline. Target stack: Pinocchio, MuJoCo, and OSQP. Future topics include URDF, FK, Jacobian, IK, QP-IK, MuJoCo PD, and Mini-WBC.
- `projects/B_legged_control_study/`: reproduction and breakdown workspace for `qiayuanl/legged_control`. Focus areas include NMPC, WBC, QP, state estimation, and ROS control.
- `projects/C_unitree_rl_mjlab_study/`: reproduction and breakdown workspace for `unitreerobotics/unitree_rl_mjlab`. Focus areas include MuJoCo, PPO, Train, Play, Sim2Real, observation, action, and reward.

Code, notes, outputs, and external references should be placed in the matching project container.

## 4. Role Of shared/

`shared/` is reserved for assets and helpers that are intentionally shared across projects:

- `shared/robot_assets/`: common robot descriptions, meshes, or model references after manual review.
- `shared/env/`: shared environment notes or setup fragments.
- `shared/scripts/`: reusable helper scripts that are not owned by a single project.
- `shared/templates/`: templates for notes, reports, scripts, or experiment records.

Shared content should be added only when it is useful across more than one project.

## 5. Role Of tools/

`tools/` is reserved for repository-level utility code:

- `tools/migration/`: migration audit, validation, and one-time repository organization helpers.
- `tools/export/`: export helpers for reports, documents, or generated review artifacts.

Project-specific learning scripts should stay inside the relevant `projects/*/scripts/` directory.

## 6. No Legacy Migration In This Step

This step does not copy old project files from `/home/ubuntu/robot_proj/Pinocchio_URDF`.

It does not move existing root-level files from `/home/ubuntu/Robot_Dynamics_Control`.

It does not implement FK, Jacobian, IK, QP, WBC, RL, or any robot control algorithm.

## 7. Next Step

Step 3 should migrate only the manually confirmed A project assets from the old project into `projects/A_self_baseline/`.

Before Step 3, confirm which old scripts, notes, robot assets, and environment files are part of the self-developed baseline.

## 8. Acceptance Checklist

- [ ] `projects/A_self_baseline/` exists with source, script, config, controller, dynamics, env, experiment, docs, notes, test, plot, and output containers.
- [ ] `projects/B_legged_control_study/` exists with docs, scripts, configs, external, reproduce logs, and output containers.
- [ ] `projects/C_unitree_rl_mjlab_study/` exists with docs, scripts, configs, external, reproduce logs, and output containers.
- [ ] `shared/` exists with robot asset, environment, script, and template containers.
- [ ] `tools/` exists with migration and export containers.
- [ ] README files describe the purpose and boundaries of A, B, and C.
- [ ] No old project files were migrated.
- [ ] No existing root-level files were moved or deleted.
