# Self Baseline Learning Scripts Plan

This plan is updated after Step R-C.

## Current Script Roles

- A00-A03: foundation validation scripts.
- A04: DLS IK wrapper for IK interface.
- A05: QP-IK wrapper for IK interface.
- A06: Target + Viewer wrapper.
- A07: Actuator wrapper.
- A08-A10: later TODO topics.

## Current Rule

Do not add more duplicated implementation logic into A04/A05/A06/A07. Shared behavior should move into:

- `target_interface.py`
- `ik_interface.py`
- `viewer_interface.py`
- `actuator_interface.py`
- `trajectory_io.py`

## Next Script Work

R1 updates Target interface first, then A06/A05 wrappers can consume the unified schema.
