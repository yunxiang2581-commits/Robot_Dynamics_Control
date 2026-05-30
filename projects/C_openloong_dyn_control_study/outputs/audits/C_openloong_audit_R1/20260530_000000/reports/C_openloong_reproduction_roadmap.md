# C OpenLoong Reproduction Roadmap

This roadmap is intentionally staged. Do not start from `walk_mpc_wbc`; that bundles too many failure modes into one run.

| Step | Goal | Input | Output | Verification | Risk |
|---|---|---|---|---|---|
| C00 | Finish architecture map | Project C docs + official source | audit report, learning map, architecture map | Can explain entrypoints, modules, resources, dependencies | Confusing Project C skeleton with official implementation |
| C01 | Model path audit | `models/scene_board.xml`, `models/scene.xml`, `models/AzureLoong.urdf`, `models/meshes/` | model/resource path table | All includes and mesh paths can be resolved conceptually | Relative path assumptions |
| C02 | Joint/motor/sensor mapping audit | `AzureLoong.xml`, `AzureLoong.urdf`, `MJ_interface.h`, `PVT_ctrl.h`, `joint_ctrl_config.json` | mapping table | 31 motor/joint names aligned across files | floating-base offset, typo mismatch |
| C03 | Dependency installation check | README apt deps | environment log | `cmake`, `gcc-11`, `g++-11`, `make` available | current Ubuntu 25.10 differs from official Ubuntu 22.04 |
| C04 | CMake configure/build evidence | official source, CMakeLists | configure/build logs | binaries exist: `walk_wbc`, `walk_mpc_wbc`, `wbc_speed_test` | ABI/compiler/static lib mismatch |
| C05 | Non-viewer WBC smoke | `wbc_speed_test` | runtime log, `record/datalog.log` | completes without GLFW viewer, logs runtime column | 10000-loop runtime, QP failures, relative paths |
| C06 | WBC walking baseline | `walk_wbc` | console log, datalog, screenshot | reaches stepping/walking phase around 3-5s | OpenGL/GLFW/window, simulation stability |
| C07 | MPC+WBC walking baseline | `walk_mpc_wbc` | console log, datalog, screenshot | MPC QP and WBC QP status observed, stable segment captured | MPC QP instability, harder failure localization |
| C08 | Metrics and figures | copied `datalog.log` | CSV/plots | base/rpy/foot force/joint torque curves readable | log parsing and column mapping |
| C09 | Final reproduction report | build/run logs, screenshots, metrics | final report + resume bullets | evidence directory complete and reproducible | overclaiming if demo only partially runs |

## Recommended Immediate Next Step

Do C01/C02 before installing or compiling anything.

Reason: the official project relies on name-based lookup plus some index-based assumptions. The model path and joint mapping table will make later build/run failures easier to interpret.

## Build Gate

Do not attempt official build until these are true:

```text
cmake available
compiler plan chosen: official g++-11 or recorded risk build with g++-15
OpenGL/GLFW display plan documented
official source dirty status recorded
evidence output directory chosen
```

## First Runnable Candidate After Build

1. `wbc_speed_test`: preferred non-viewer algorithm smoke.
2. `walk_wbc`: first complete MuJoCo WBC closed-loop demo.
3. `walk_mpc_wbc`: second complete demo after WBC-only is stable.
