# Robot Dynamics Control

## Current Status

`Robot_Dynamics_Control` is a simulation-only robotics motion-control learning repository. The current main line is Project A, which builds a readable baseline before later B/C/D simulation studies.

Project A is now split into two layers:

- Foundation validation layer: A00-A03.
- Unified motion interface layer: A04-A07.

## Project A: Current Architecture

Foundation validation layer:

- A00 reference and assets.
- A01 model inspect.
- A02 configuration / site pose.
- A03 site Jacobian check.

Unified motion interface layer:

- Target interface: where the target comes from.
- IK interface: how a target becomes `q_traj`.
- Viewer interface: how viewer / mocap target will be planned.
- Actuator interface: how `q_traj` will later enter MuJoCo actuator tracking.

A04-A07 are now thin TODO learning wrappers around those interfaces:

- A04 = DLS IK wrapper, `solver_type=dls`.
- A05 = QP-IK wrapper, `solver_type=qp_scipy` / `qp_osqp`.
- A06 = Target + Viewer wrapper.
- A07 = Actuator wrapper.

## Current Completion

- A01-A03 validation outputs are retained.
- Four-interface schema and TODO skeletons are created.
- Old A04/A05 run artifacts and outdated step records were cleaned.
- Documentation was refreshed around the new interface architecture.

## Not Yet Implemented

- Full TargetDefinition load/save/validate.
- A05 `target_definition_json` regression.
- A06 fixed target minimal implementation.
- A06 derived MJCF, keyboard target movement, mouse drag validation, and kinematic IK follow.
- A07 actuator tracking.
- Video demo.

## Next Step

R1: finish `TargetDefinition` load/save/validate.

## Project A Wrapper Entries

A04/A05/A06/A07 are now complete-scope TODO learning skeletons:

- A04: `projects/A_self_baseline/scripts/04_ik_dls_wrapper.py`
- A05: `projects/A_self_baseline/scripts/05_ik_qp_wrapper.py`
- A06: `projects/A_self_baseline/scripts/06_target_viewer_wrapper.py`
- A07: `projects/A_self_baseline/scripts/07_actuator_wrapper.py`

The TODO scope is full: DLS, QP-IK, target/viewer, and actuator routes are all planned, but not implemented in this step.

## Boundaries

- Do not call mink as a replacement implementation.
- Do not modify `external/mink_upstream/`.
- Do not modify original robot assets such as the UR5e `scene.xml`.
- Do not write `data.ctrl` outside A07.
- Do not run real hardware.
- Do not record video outside the planned demo step.

## Entry Points

- [Project status](docs/00_project_management/PROJECT_STATUS.md)
- [Project structure](docs/00_project_management/PROJECT_STRUCTURE.md)
- [Project roadmap](docs/00_project_management/PROJECT_ROADMAP.md)
- [A project README](projects/A_self_baseline/README.md)
- [A four-interface plan](projects/A_self_baseline/docs/A_four_interface_refactor_plan.md)
