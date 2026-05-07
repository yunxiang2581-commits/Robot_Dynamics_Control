# Reference: mink UR5e

This document is now an index for the mink UR5e reference material.

## Reference Files

- `projects/A_self_baseline/external/mink/examples/arm_ur5e.py`
- `projects/A_self_baseline/external/mink/examples/arm_ur5e_actuators.py`
- `projects/A_self_baseline/external/mink/README.md`
- `projects/A_self_baseline/external/mink/LICENSE`

## What We Learn

The useful structure is:

```text
mocap target -> FrameTask/PostureTask -> limits -> solve_ik -> viewer -> data.ctrl
```

Project A maps this to:

- Target interface.
- IK interface.
- Viewer interface.
- Actuator interface.

## Boundary

The reference files are for reading and comparison. Project A does not call mink to replace its own implementation.
