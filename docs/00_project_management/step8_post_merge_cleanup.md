# Step 8 Post-Merge Cleanup

## Step 8 Goal

Perform a lightweight post-merge cleanup after the monorepo migration. This step checks root-level residue, removes only confirmed empty root directories, and updates documentation for retained root files.

This step does not implement algorithms, does not migrate the old project, and does not modify A/B/C `legacy_imported` content.

## Root scripts/ Check Result

Root `scripts/` existed and contained no files or subdirectories beyond the directory itself:

```text
scripts
```

Because it was confirmed empty, it was removed with `rmdir scripts`.

Export scripts now live under:

```text
tools/export/
```

## Root external/ Check Result

Root `external/` existed and contained no files or subdirectories beyond the directory itself:

```text
external
```

Because it was confirmed empty, it was removed with `rmdir external`.

B/C external notes now live under:

```text
projects/B_legged_control_study/external/
projects/C_unitree_rl_mjlab_study/external/
```

## requirements.txt Retention

Root `requirements.txt` is retained as the current overall project environment entry. It contains the lightweight baseline dependencies currently visible at root:

```text
mujoco
numpy
matplotlib
scipy
```

Later steps may split environment files into `shared/env/` or project-specific environment files after A/B/C dependencies are clearer.

## outputs/ Retention

Root `outputs/` is retained as a temporary overall output index. It contains README files and historical PD experiment output directories.

Project-level outputs should be written under:

```text
projects/*/outputs/
```

Large or generated root outputs remain ignored by `.gitignore` and should not be committed unless manually curated.

## Current Recommended Working Style

- Use `/home/ubuntu/Robot_Dynamics_Control` as the only active working directory.
- Put A self baseline work under `projects/A_self_baseline/`.
- Put B study notes and reproduction artifacts under `projects/B_legged_control_study/`.
- Put C study notes and reproduction artifacts under `projects/C_unitree_rl_mjlab_study/`.
- Put shared robot assets and environment references under `shared/`.
- Put repository-level utilities under `tools/`.

## Next Step

Enter A project implementation from:

```text
projects/A_self_baseline/scripts/01_inspect_urdf.py
```

The first implementation step should load and inspect a URDF model, then report `nq`, `nv`, joints, and frames.
