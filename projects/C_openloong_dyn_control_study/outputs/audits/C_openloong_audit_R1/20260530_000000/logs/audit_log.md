# C-OPENLOONG-AUDIT-R1 Audit Log

## Scope

Task: OpenLoong-Dyn-Control open-source project architecture audit and learning map.

Allowed output root:

```text
projects/C_openloong_dyn_control_study/outputs/audits/C_openloong_audit_R1/20260530_000000/
```

## Commands Run

- `git status --short`
- `git diff --stat`
- `find projects/C_openloong_dyn_control_study ...`
- `rg ... projects/C_openloong_dyn_control_study docs`
- `find external ... openloong ...`
- `git --no-optional-locks -C external/open_source_repos/OpenLoong-Dyn-Control rev-parse --short HEAD`
- `git --no-optional-locks -C external/open_source_repos/OpenLoong-Dyn-Control status --short --branch`
- `sed -n ...` on Project C docs and official OpenLoong README/CMake/demo/header files
- `python --version`
- `pytest --version`
- `cmake --version`
- `g++ --version`
- `make --version`
- `lsb_release -a`

Full command list is preserved in:

```text
commands/audit_commands.sh
```

## Evidence Summary

- Main repo dirty tree: yes.
- Project C source implementation: none found.
- Project C config/model/test files: none found.
- Project C docs: present and substantial.
- Official source exists: `external/open_source_repos/OpenLoong-Dyn-Control/`.
- Official commit: `4dd7a7e4`.
- Official source status: many `M` entries; previous sampling indicates line-ending changes are likely.
- Official build system: CMake.
- Official primary demos: `walk_wbc`, `walk_mpc_wbc`, `jump_mpc`, `float_control`, `wbc_speed_test`, joystick and staircase variants.
- Safest official post-build smoke candidate: `wbc_speed_test`, because it does not open GLFW viewer.
- First complete closed-loop reproduction candidate: `walk_wbc`.
- Current environment: Ubuntu 25.10, Python 3.13.12, pytest 9.0.3, g++ 15.2.0, make 4.4.1.
- Current missing build dependency: `cmake`.

## Safety Boundary Confirmation

- No source code modified.
- No external source modified.
- No Project A/B/D modified.
- No `shared/robot_assets/vendor` modified.
- No long MuJoCo run.
- No MP4 generated.
- No `git add`.
- No `git commit`.
- No `git push`.

## Generated Files

- `reports/C_openloong_audit_R1_report.md`
- `reports/C_openloong_learning_map.md`
- `reports/C_openloong_architecture_map.md`
- `reports/C_openloong_reproduction_roadmap.md`
- `commands/audit_commands.sh`
- `logs/audit_log.md`
