# Project Status

## Current Main Line

`Robot_Dynamics_Control` is a simulation-only robotics motion-control learning repository.

Current focus: Project A has been refactored into a four-interface TODO skeleton while preserving A01-A03 foundation validation.

## Project A Status

Foundation validation layer retained:

- A00 reference and assets.
- A01 model inspect.
- A02 configuration / site pose.
- A03 site Jacobian check.

Unified motion interface layer:

- Target interface: where the target comes from.
- IK interface: how target becomes `q_traj`.
- Viewer interface: how viewer / mocap target is planned.
- Actuator interface: how `q_traj` later enters MuJoCo actuator tracking.

## Completed in Step R-C

- Added shared schema in `motion_types.py`.
- Added Target / IK / Trajectory IO / Viewer / Actuator interface skeletons.
- Refactored A04-A07 into thin wrappers.
- Added `configs/motion_task.yaml` as unified task template.
- Cleaned old A04/A05 execution artifacts and outdated step records.
- Updated project status and A documentation.

## Not Completed

- TargetDefinition load/save/validate full implementation.
- A05 target_definition_json integration.
- A06 fixed target minimal implementation.
- A06 derived MJCF.
- A06 keyboard target movement.
- A06 mouse drag target validation.
- A06 kinematic IK follow.
- A07 actuator tracking.
- Video demo.

## Next Step

R1: finish TargetDefinition load/save/validate.

## Boundaries

- Do not call mink as replacement implementation.
- Do not modify `external/mink_upstream/`.
- Do not write `data.ctrl` outside A07.
- Do not record video outside the video demo step.
- Do not run real hardware.
