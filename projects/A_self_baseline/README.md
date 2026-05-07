# A Self Baseline

Project A is the self-written simulation-only robotics control baseline. It studies the structure behind the mink UR5e examples without calling mink as a replacement implementation.

## Current Layering

Foundation validation layer:

- `scripts/00_reference_and_assets.py`
- `scripts/01_model_inspect.py`
- `scripts/02_configuration_site_pose.py`
- `scripts/03_site_jacobian_check.py`

Unified motion interface layer:

- Target interface: `src/robot_baseline/target_interface.py`
- IK interface: `src/robot_baseline/ik_interface.py`
- Viewer interface: `src/robot_baseline/viewer_interface.py`
- Actuator interface: `src/robot_baseline/actuator_interface.py`

Shared schema:

- `src/robot_baseline/motion_types.py`

## A04-A07 Mapping

- A04 DLS differential IK is now a thin wrapper for IK interface with `solver_type=dls`.
- A05 QP-IK is now a thin wrapper for IK interface with `solver_type=qp_scipy` or `qp_osqp`.
- A06 target / mocap-style tracking is now a thin wrapper for Target interface + Viewer interface.
- A07 MuJoCo actuator tracking is now a thin wrapper for Actuator interface.

## Current Status

- A01-A03 remain the foundation validation layer and are not downgraded.
- A04-A07 are TODO learning skeletons around the unified interfaces.
- Old A04/A05 execution artifacts were removed from the current main line.
- `configs/motion_task.yaml` is the new unified task configuration template.

## Current Non-Goals

- No full DLS or QP-IK implementation in Step R-C.
- No real viewer drag or keyboard target movement.
- No derived MJCF creation.
- No `data.ctrl` writes.
- No MuJoCo actuator control loop.
- No video recording.
- No direct mink replacement call.

## Documentation

- `docs/A_four_interface_refactor_plan.md`
- `docs/A_target_interface.md`
- `docs/A_ik_interface.md`
- `docs/A_viewer_interface.md`
- `docs/A_actuator_interface.md`
- `docs/A_pipeline_contract.md`
- `docs/A_mink_alignment_plan.md`
- `docs/A_simulation_only_full_motion_control_plan.md`

## Next Step

R1: finish `TargetDefinition` load/save/validate, then let A06 write `A06_target_definition.json` and A05 read it.

## Step R-F Full TODO Skeleton

A04/A05/A06/A07 are complete-scope TODO learning skeletons, not simplified placeholders.

Current entries:

- A04: `scripts/04_ik_dls_wrapper.py`
- A05: `scripts/05_ik_qp_wrapper.py`
- A06: `scripts/06_target_viewer_wrapper.py`
- A07: `scripts/07_actuator_wrapper.py`

Implementation order:

1. R1 TargetDefinition load/save/validate.
2. R2 A05 target_definition_json.
3. R3 A06 fixed_pose / pose_sequence.
4. R4 A06 mocap capability.
5. R5 A06 derived MJCF.
6. R6 A06 keyboard movement.
7. R7 A06 mouse drag.
8. R8 A06 kinematic IK follow.
9. R9 A07 actuator tracking.
10. R10 A10 video demo.
