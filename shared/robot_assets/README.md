# Shared Robot Assets

This directory stores robot model assets shared across the monorepo.

## Current Asset Record

The current confirmed asset set is:

```text
shared/robot_assets/models/h1_description/
```

It currently contains:

- `urdf/h1.urdf`
- `urdf/h1_with_hand.urdf`
- `mjcf/h1.xml`
- `mjcf/h1_with_hand.xml`
- `mjcf/scene.xml`
- `mjcf/scene_with_hand_bright.xml`
- `meshes/` with 98 mesh files in `.STL` and `.dae`
- `package.xml`
- `README.md`
- `CMakeLists.txt`
- `doc/H1.png`
- `launch/` ROS display and Gazebo launch files

## Recommended Usage In A Project

For A01-A05 Pinocchio learning:

- Prefer `models/h1_description/urdf/h1_with_hand.urdf`
- Use `../../shared/robot_assets/models` as `package_dirs`

For A06 MuJoCo PD tracking:

- Prefer `models/h1_description/mjcf/scene_with_hand_bright.xml`

## Path Rules

Keep the `h1_description/` package layout intact.

Reasons:

- The URDF uses `package://h1_description/meshes/...`
- The MuJoCo XML uses `meshdir="../meshes"`
- `scene_with_hand_bright.xml` includes `h1_with_hand.xml`

Do not flatten these files into a single directory unless the URDF and MJCF paths are rewritten consistently.

## Git Guidance

Small model descriptors can stay in Git. Large generated outputs, logs, videos, checkpoints, and external training results should not be stored here.
