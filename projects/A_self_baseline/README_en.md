# A Self Baseline

> Chinese version: [README.md](README.md)

This is Track A, the self-developed robot motion control baseline.

The planned stack is Pinocchio, MuJoCo, and OSQP. The learning path will cover URDF loading, FK, Jacobian, IK, QP-IK, MuJoCo PD tracking, and a teaching Mini-WBC pipeline.

The current state is a TODO teaching skeleton. The repository already contains standard scripts, module interfaces, config templates, and imported reference assets, but it does not contain complete algorithm implementations.

## Documentation Entry

- `docs/README.md`: A-project documentation index.
- `docs/A_pipeline_contract.md`: A01-A07 pipeline contract.
- `docs/legacy_script_mapping.md`: mapping from legacy scripts to standard A scripts.
- `docs/self_baseline_learning_scripts_plan.md`: learning-script plan.
- `docs/legacy_imported/`: imported legacy documents from the old project.

The main documentation directory for A is `projects/A_self_baseline/docs/`.

## A01-A07 Pipeline

```text
A01 inspect URDF
  -> A02 FK frame pose
  -> A03 Jacobian FD check
  -> A04 DLS-IK
  -> A05 QP-IK with joint limits
  -> A06 MuJoCo PD tracking
  -> A07 Mini-WBC QP
```

These scripts are pipeline components, not isolated demos.

## Inputs And Outputs

- A01: model summary, joint list, frame list.
- A02: FK pose report for a target frame.
- A03: Jacobian finite-difference validation report.
- A04: DLS-IK error curve and `q` trajectory.
- A05: constrained QP-IK status and trajectory.
- A06: MuJoCo PD tracking logs, figures, and optional video.
- A07: Mini-WBC QP structure report.

See `outputs/README.md` for output-directory conventions.

## Execution Order

The current scripts still stop at `NotImplementedError`. When implementation starts, the intended order is:

```bash
python projects/A_self_baseline/scripts/01_model_inspect.py --urdf shared/robot_assets/models/h1_description/urdf/h1_with_hand.urdf
python projects/A_self_baseline/scripts/02_configuration_site_pose.py --urdf shared/robot_assets/models/h1_description/urdf/h1_with_hand.urdf --frame left_foot
python projects/A_self_baseline/scripts/03_site_jacobian_check.py --urdf shared/robot_assets/models/h1_description/urdf/h1_with_hand.urdf --frame left_foot
python projects/A_self_baseline/scripts/04_dls_differential_ik.py --urdf shared/robot_assets/models/h1_description/urdf/h1_with_hand.urdf --frame left_foot
python projects/A_self_baseline/scripts/05_task_limit_qp_ik.py --urdf shared/robot_assets/models/h1_description/urdf/h1_with_hand.urdf --frame left_foot
python projects/A_self_baseline/scripts/06_target_mocap_tracking.py --model-xml shared/robot_assets/models/h1_description/mjcf/scene_with_hand_bright.xml
python projects/A_self_baseline/scripts/07_mujoco_actuator_tracking.py --config projects/A_self_baseline/configs/mini_wbc.yaml
```

`scripts/` contains CLI entry points. `src/robot_baseline/` contains reusable components.

## Standard Modules

- `model_loader.py`
- `kinematics.py`
- `jacobian_check.py`
- `ik.py`
- `qp_ik.py`
- `pd_controller.py`
- `mini_wbc.py`
- `metrics.py`
- `pipeline_io.py`

## Imported Material

- `legacy_imported/`: old-project reference scripts and docs. Read-only reference, not the standard entry point.
- `root_imported/`, `root_imported_src/`, `root_imported_utils/`: A-project assets relocated from the early root-level structure of this repo.

## Recommended Implementation Order

1. Inspect URDF and confirm model, joint, and frame names.
2. Implement FK frame pose.
3. Implement Jacobian and finite-difference validation.
4. Implement DLS-IK.
5. Implement constrained QP-IK.
6. Implement MuJoCo PD tracking.
7. Implement Mini-WBC QP structure.
