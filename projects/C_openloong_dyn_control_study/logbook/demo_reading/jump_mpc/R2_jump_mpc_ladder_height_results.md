# R2 jump_mpc ladder height notes

Date: 2026-07-01

Scope: Project C experimental OpenLoong copy only. Original OpenLoong source is not changed.

## Goal

Raise `jump_mpc` height gradually instead of forcing a single 0.5 m jump in one step.

## Inputs

- Demo: `jump_mpc`
- Experimental worktree:
  `projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_jump_mpc_basepos_exp/20260701_001306/worktree/OpenLoong-Dyn-Control`
- Visual launcher:
  `projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/commands/run_jump_mpc_ladder_visual.ps1`
- Container launcher:
  `projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/commands/container_run_jump_mpc_ladder_visual.sh`
- Batch scan launcher:
  `projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/commands/run_jump_mpc_ladder_scan.ps1`
- Batch scan container script:
  `projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/commands/container_run_jump_mpc_ladder_once.sh`

## Control Logic

The original `jump_z` value was not a direct actual-height command. It is converted to a target vertical velocity:

```cpp
jump_vel_des[2] = sqrt(2.0 * 9.8 * jump_z);
```

The observed jump height is strongly affected by the push-off duration `jump_acc_t`, torque/force saturation, and landing recovery.

The Project C experimental copy adds runtime environment switches:

- `OPENLOONG_JUMP_Z`
- `OPENLOONG_JUMP_ACC_T`
- `OPENLOONG_ANKLE_PITCH_COMP_GAIN`
- `OPENLOONG_LANDING_X_OFFSET`

It also adds diagnostic log columns:

- `js_pos_des`
- `js_vel_des`
- `base_pos_des`
- `Xd0`
- `dX_cal`
- `jump_debug`

## Current Ladder Results

All height deltas are measured from `gpsVal_z` at about `t=8.5s`.

Latest matrix scan:

`projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_jump_mpc_basepos_exp/ladder_scan_jumpz0p2_0p3_0p4_0p5_20260701_052526/summary.csv`

This scan varies both target `jump_z` and push-off duration `jump_acc_t`.
Each run reached `t=13.005s`; `DockerExitCode=0` for all rows, so unstable rows are controller/landing outcomes rather than container crashes.

| jump_z | jump_acc_t | actual jump delta | max abs RPY | min z after jump | result |
|---:|---:|---:|---:|---:|---|
| 0.2 | 0.075 | 0.181 m | 0.186 rad | 0.839 m | stable |
| 0.2 | 0.085 | 0.193 m | 0.197 rad | 0.838 m | stable |
| 0.2 | 0.095 | 0.205 m | 0.218 rad | 0.838 m | stable |
| 0.3 | 0.075 | 0.190 m | 0.268 rad | 0.848 m | stable |
| 0.3 | 0.085 | 0.246 m | 0.353 rad | 0.843 m | stable |
| 0.3 | 0.095 | 0.286 m | 3.874 rad | 0.117 m | falls |
| 0.4 | 0.075 | 0.192 m | 0.272 rad | 0.848 m | stable |
| 0.4 | 0.085 | 0.251 m | 0.373 rad | 0.844 m | stable |
| 0.4 | 0.095 | 0.321 m | 7.281 rad | 0.108 m | falls |
| 0.5 | 0.075 | 0.194 m | 0.273 rad | 0.848 m | stable |
| 0.5 | 0.085 | 0.254 m | 0.376 rad | 0.845 m | stable |
| 0.5 | 0.095 | 0.326 m | 3.338 rad | 0.124 m | falls |

Matrix conclusion:

```text
Current stable envelope is around actual jump delta 0.25 m.
The 0.095 s push-off duration is safe only for low target jump_z=0.2.
For jump_z >= 0.3, acc_t=0.095 reaches roughly 0.29-0.33 m but falls on landing.
The practical visual setting remains jump_z=0.5, jump_acc_t=0.085.
```

Latest automated scan:

`projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_jump_mpc_basepos_exp/ladder_scan_jumpz0p5_20260701_050925/summary.csv`

| jump_z | jump_acc_t | actual jump delta | result |
|---:|---:|---:|---|
| 0.5 | 0.0639 | ~0.154 m | stable, but visually too low |
| 0.5 | 0.075 | 0.194 m | stable |
| 0.5 | 0.085 | 0.254 m | stable, current visual default |
| 0.5 | 0.090 | 0.289 m | borderline; max abs RPY is 0.558 rad |
| 0.5 | 0.095 | 0.326 m | falls; min z drops to 0.123 m |
| 0.5 | 0.10 | ~0.366 m | falls |
| 0.5 | 0.14 | ~0.524 m | reaches height, but falls on landing |

Scan command:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File D:\project\Robot_Dynamics_Control\projects\C_openloong_dyn_control_study\tools\openloong_demo_runner\commands\run_jump_mpc_ladder_scan.ps1 -JumpAccT '0.075,0.085,0.090,0.095' -TimeoutSeconds 95
```

## Current Visual Run

Command:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File D:\project\Robot_Dynamics_Control\projects\C_openloong_dyn_control_study\tools\openloong_demo_runner\commands\run_jump_mpc_ladder_visual.ps1 -JumpZ 0.5 -JumpAccT 0.085 -Port 6087
```

URL:

```text
http://127.0.0.1:6087/vnc.html?autoconnect=true&resize=remote
```

Stop:

```powershell
wsl.exe -d Ubuntu-22.04-ProjectC -- docker rm -f projectc_jump_mpc_ladder_visual
```

## Risk

Trying to reach an actual 0.5 m jump by increasing only `jump_acc_t` causes landing instability. The data suggests the next real fix should focus on landing recovery, not just higher push-off speed.
