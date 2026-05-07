# A Pipeline Contract

## 1. Current Pipeline

```text
A00 reference/assets
-> A01 model inspect
-> A02 configuration/site pose
-> A03 site Jacobian check
-> A04/A05 IK interface wrappers
-> A06 Target + Viewer interface wrapper
-> A07 Actuator interface wrapper
-> A08-A10 later TODO steps
```

## 2. Foundation Validation Layer

A01-A03 are retained as completed validation steps:

- A01 confirms MuJoCo model dimensions and object names.
- A02 confirms `attachment_site` / `wrist_3_link` pose.
- A03 confirms site Jacobian by finite difference.

These files and outputs are not deleted or downgraded.

## 3. Unified Motion Interface Layer

| Layer | Contract |
|---|---|
| Target interface | Produces or loads `TargetDefinition` |
| IK interface | Consumes `IkRequest`, returns `IkResult` |
| Viewer interface | Plans viewer / mocap target data flow |
| Actuator interface | Plans A07 actuator tracking |

## 4. A04/A05 Contract

A04 and A05 no longer own separate target logic.

- A04 uses `solver_type=dls`.
- A05 uses `solver_type=qp_scipy` / `qp_osqp`.
- Both should consume `TargetDefinition` through `IkRequest`.

## 5. A06 Contract

A06 defines targets and plans viewer target behavior:

- fixed target.
- pose sequence.
- mocap placeholder.
- interactive viewer TODO.

A06 does not solve IK, does not write `data.ctrl`, and does not run actuator tracking.

## 6. A07 Contract

A07 consumes `trajectory_source` and is the only layer that may later write `data.ctrl` and call `mujoco.mj_step` in a control loop.

## 7. Current TODOs

- R1: TargetDefinition load/save/validate.
- R2: A05 target_definition_json.
- R3: A06 fixed_pose / pose_sequence.
- R4: A06 mocap capability.
- R5: A06 derived MJCF.
- R6: A06 keyboard movement.
- R7: A06 mouse drag.
- R8: A06 kinematic IK follow.
- R9: A07 actuator tracking.
- R10: A10 video demo.

## 8. Step R-F Wrapper Entries

A04/A05/A06/A07 当前是完整功能版 TODO learning skeleton，不是简化版。

| Step | Entry | Interface |
|---|---|---|
| A04 | `scripts/04_ik_dls_wrapper.py` | IK interface / DLS |
| A05 | `scripts/05_ik_qp_wrapper.py` | IK interface / QP |
| A06 | `scripts/06_target_viewer_wrapper.py` | Target + Viewer interface |
| A07 | `scripts/07_actuator_wrapper.py` | Actuator interface |
