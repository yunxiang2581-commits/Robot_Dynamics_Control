# WSLg COPY MODE Repair Project Skill Design

## Goal

Create a project-local Codex Skill that repairs the Windows-side WSLg session when native MuJoCo, GLFW, or OpenGL windows show `[WARN:COPY MODE]`, remain gray, or fail to present frames while the simulation continues running.

The repair must affect only the display environment. It must not change MuJoCo models, IK, WBC, MPC, FSM, controller parameters, Docker socket permissions, or simulation math.

## Location

Create the Skill under the repository's existing project Skill convention:

```text
tools/codex_skills/wslg-copy-mode-repair/
├── SKILL.md
├── agents/openai.yaml
└── scripts/repair_wslg_copy_mode.ps1
```

Add the PowerShell behavior test beside the existing OpenLoong runner tests:

```text
projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/tests/
└── test_wslg_copy_mode_repair_skill.ps1
```

## Trigger Conditions

The Skill should trigger for project requests involving any of these symptoms:

- WSLg or MuJoCo window title contains `[WARN:COPY MODE]`.
- MuJoCo, GLFW, or `glxgears` creates a gray, white, black, or stale native window.
- Simulation time and logs advance while the native window does not repaint.
- `/mnt/wslg/weston.log` reports `rdp_allocate_shared_memory` with `Input/output error`.
- Weston reports `use_gfxredir = 0`.
- The user asks to restore or repair native WSLg simulation display in this repository.

## Skill Workflow

1. Run `git status --short` before changing project files.
2. Explain that the input is the Windows/WSLg session and the output is a restored native GUI path; control mathematics do not change.
3. Confirm the symptom with the requested application or a minimal OpenGL program.
4. Read Weston evidence only through the normal user distribution path `/mnt/wslg/weston.log`.
5. Never use `wsl --system` as a verification step because it can rebuild or disturb the WSLg system session.
6. Warn that repair closes every running WSL distribution and Docker container.
7. Run the bundled PowerShell script after user confirmation.
8. Launch the requested native WSLg simulation directly.
9. Verify that the Windows title no longer contains `[WARN:COPY MODE]` and that the rendered scene is nonblank.
10. Run `git diff --stat` after project edits.

## Repair Script

`repair_wslg_copy_mode.ps1` will provide:

- `-DryRun`: print the planned operations without changing system state.
- `-Force`: skip the interactive shutdown confirmation.
- `-WslExe`: injectable WSL executable path for deterministic tests.
- `-ServiceName`: default to `WslService`.
- `-MinimumWslVersion`: default to `2.7.10`.

Normal execution will:

1. Confirm Windows and administrator privileges.
2. Read and report the installed WSL version.
3. Stop with a clear error when WSL is older than the configured minimum.
4. Prompt before terminating WSL unless `-Force` is supplied.
5. Execute `wsl.exe --shutdown`.
6. Restart `WslService`.
7. Wait until the service is running.
8. Print a concise completion message instructing the caller to launch the native simulation directly.

The script will not start a browser, noVNC, VNC server, container, or simulation. It will not modify `/mnt/shared_memory`, `/etc/fstab`, `/var/run/docker.sock`, WSL configuration files, or repository source code.

## Failure Handling

- Missing administrator privileges: stop with an instruction to use an elevated PowerShell session.
- WSL executable missing: stop before shutting down anything.
- WSL version cannot be parsed: stop and report the raw version output.
- WSL version too old: stop and require an explicit WSL update before repair.
- `wsl --shutdown` fails: do not restart unrelated services; report the exit code.
- `WslService` is missing or does not return to `Running`: stop with the service status.
- COPY MODE remains after repair: recommend a Windows restart and collect fresh normal-distro Weston logs. Do not fall back to noVNC unless the user explicitly requests it.

## Testing

Follow TDD:

1. Add a failing PowerShell test before creating the Skill.
2. Verify the test fails because the Skill and script do not exist.
3. Initialize the Skill with the official `init_skill.py`.
4. Implement the minimum Skill and script.
5. Run the same test and verify it passes.

The automated test will verify:

- Required Skill files exist.
- `SKILL.md` has valid `name` and `description` frontmatter.
- Trigger keywords include COPY MODE, MuJoCo, WSLg, `use_gfxredir`, and `rdp_allocate_shared_memory`.
- `-DryRun` reports `wsl.exe --shutdown` and `Restart-Service WslService`.
- Dry-run output contains no `--system`, noVNC, browser launch, Docker socket permission changes, `fstab` edits, or shared-memory mounts.
- The PowerShell parser accepts the repair script.
- Existing project Skill validation passes with `quick_validate.py`.

## Verification

Implementation is complete only when:

- The new test passes.
- `quick_validate.py` passes.
- `agents/openai.yaml` matches the Skill metadata.
- The script's dry-run output is correct.
- Existing OpenLoong WSLg launcher tests still pass.
- `git diff --check` and `git diff --stat` complete successfully.
- No automatic `git push` is performed.
