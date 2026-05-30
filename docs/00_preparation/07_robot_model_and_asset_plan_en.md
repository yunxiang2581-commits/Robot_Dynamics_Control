# Robot Model And Asset Plan

> Chinese version: [07_robot_model_and_asset_plan.md](07_robot_model_and_asset_plan.md)

## Reading Navigation

| Previous | Current | Next |
| --- | --- | --- |
| [06 External project plan](06_external_project_plan_en.md) | Robot model and asset plan | [08 Codex workflow rules](08_codex_workflow_rules_en.md) |

## Purpose

This document defines where URDF, MJCF, mesh files, videos, logs, and model weights belong, and how Git should treat them.

## Key Takeaways

| Asset | Policy |
| --- | --- |
| Small text assets | may be committed, for example lightweight URDF, configs, Markdown reports |
| Large binary assets | do not commit by default, for example large meshes, videos, training weights |
| Experiment outputs | curated plots may be committed, logs and videos are ignored by default |
| Third-party source | do not copy into this repository by default |

## Asset Rules

| Type | Use | Recommended location | Git policy |
| --- | --- | --- | --- |
| URDF | Pinocchio model loading for A | `shared/robot_assets/models/` or external path | small examples may be committed |
| MJCF | MuJoCo simulation and RL | `shared/robot_assets/models/` or external path | small examples may be committed |
| Mesh | visual/collision assets | external data dir or shared model package | usually do not commit large assets |
| Figures | results | `projects/*/outputs/` or root `outputs/figures/` | curated small figures may be committed |
| Videos | demo outputs | `projects/*/outputs/videos/` | ignored by default |
| Logs | build and training records | `projects/*/outputs/logs/` | ignored by default |
| Weights | RL checkpoints | `projects/*/outputs/checkpoints/` | do not commit |
| Reports | experiment summaries | `projects/*/outputs/reports/` | may be committed |

## Current Confirmed Assets

The repository now contains:

```text
shared/robot_assets/models/h1_description/
```

Confirmed contents:

- `urdf/h1.urdf`
- `urdf/h1_with_hand.urdf`
- `mjcf/h1.xml`
- `mjcf/h1_with_hand.xml`
- `mjcf/scene.xml`
- `mjcf/scene_with_hand_bright.xml`
- `meshes/`
- `package.xml`

Current recommended defaults for A:

- Pinocchio: `shared/robot_assets/models/h1_description/urdf/h1_with_hand.urdf`
- MuJoCo: `shared/robot_assets/models/h1_description/mjcf/scene_with_hand_bright.xml`

## Ignore Strategy

- `outputs/videos/*`
- `outputs/logs/*`
- `outputs/checkpoints/`
- `projects/*/external/*/src/`
- `third_party/`
- `build/`, `install/`, `log/`
