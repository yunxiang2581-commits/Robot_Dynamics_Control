# Project Structure

## Repository Role

This repository is a simulation-only robotics control learning workspace.

## Important Directories

```text
projects/A_self_baseline/
  configs/
    robot.yaml
    motion_task.yaml
  scripts/
    00_reference_and_assets.py
    01_model_inspect.py
    02_configuration_site_pose.py
    03_site_jacobian_check.py
    04_ik_dls_wrapper.py
    05_ik_qp_wrapper.py
    06_target_viewer_wrapper.py
    07_actuator_wrapper.py
  src/robot_baseline/
    model_loader.py
    motion_types.py
    target_interface.py
    ik_interface.py
    trajectory_io.py
    viewer_interface.py
    actuator_interface.py
  docs/
    A_four_interface_refactor_plan.md
    A_target_interface.md
    A_ik_interface.md
    A_viewer_interface.md
    A_actuator_interface.md
```

## Layering

- A00-A03: foundation validation layer.
- A04-A07: unified motion interface wrappers.
- `src/robot_baseline`: reusable schema and interfaces.
- `shared/robot_assets`: shared model assets; original UR5e scene.xml is not modified.
- `external/mink_upstream`: read-only upstream mirror.

## Documentation Placement

- Overall project status: `docs/00_project_management/PROJECT_STATUS.md`.
- A interface docs: `projects/A_self_baseline/docs/`.
- Temporary step execution reports: keep only current reports needed for active refactors.
