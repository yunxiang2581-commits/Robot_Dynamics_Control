# A Mink Alignment Plan

## 1. Reference Flow

mink UR5e examples can be understood as:

```text
mocap target -> FrameTask/PostureTask -> limits -> solve_ik -> viewer -> data.ctrl
```

A 项目不调用 mink 替代自己的实现，而是学习这个结构并拆成自己的四接口骨架。

## 2. Mapping

| mink concept | A project interface |
|---|---|
| mocap target / viewer target | Viewer interface + TargetDefinition |
| FrameTask | IK interface frame task TODO |
| PostureTask | IK interface posture task TODO |
| limits | IK interface bounds TODO |
| solve_ik | IK interface solver TODO |
| viewer | Viewer interface TODO |
| data.ctrl | Actuator interface / A07 only |

## 3. Boundary

- `projects/A_self_baseline/external/mink/` is reference material.
- `external/mink_upstream/` is read-only upstream mirror.
- A 项目不会直接调用 mink 完成自己的 IK 或 actuator tracking。

## 4. Current Status

- A01-A03 validation remains intact.
- A04-A07 are thin wrappers over four interfaces.
- Four-interface schema is in `motion_types.py`.
- Full solver/viewer/actuator behavior remains TODO.

## 5. Next

R1: complete TargetDefinition load/save/validate.
