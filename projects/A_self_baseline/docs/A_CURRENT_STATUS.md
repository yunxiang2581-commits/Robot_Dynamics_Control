# A Current Status

Project A current state after Step R-C:

- A01-A03 are retained as foundation validation.
- A04-A07 are thin wrappers around unified interfaces.
- The old A04/A05 run artifacts are no longer current main-line evidence.
- New implementation work starts from R1: TargetDefinition load/save/validate.

Current interface files:

- `src/robot_baseline/motion_types.py`
- `src/robot_baseline/target_interface.py`
- `src/robot_baseline/ik_interface.py`
- `src/robot_baseline/trajectory_io.py`
- `src/robot_baseline/viewer_interface.py`
- `src/robot_baseline/actuator_interface.py`

Current prohibitions:

- Do not call mink as replacement implementation.
- Do not modify upstream mirrors or original robot assets.
- Do not write `data.ctrl` outside A07.
- Do not start viewer or record video in TODO skeleton steps.
