# OpenLoong Demo Runner Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a reusable Docker demo runner for OpenLoong with one shared host entrypoint, one shared container entrypoint, and generated per-demo wrapper scripts.

**Architecture:** Keep all Docker/X11 logic in one host script and one container script. Keep demo selection in an explicit mapping table, then generate thin wrappers that delegate to the shared host script.

**Tech Stack:** Bash, Docker, X11, existing OpenLoong build artifacts

---

### Task 1: Add a minimal shell-level interface test

**Files:**
- Create: `projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/tests/test_openloong_demo_runner.sh`

- [ ] **Step 1: Write the failing test**

```bash
#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
RUNNER="$SCRIPT_DIR/../commands/run_openloong_demo_host.sh"

bash "$RUNNER" --list
```

- [ ] **Step 2: Run test to verify it fails**

Run: `bash projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/tests/test_openloong_demo_runner.sh`
Expected: FAIL because `run_openloong_demo_host.sh` does not exist yet

- [ ] **Step 3: Expand the test after implementation**

```bash
bash "$RUNNER" --list | grep -q '^walk_wbc$'
bash "$RUNNER" --list | grep -q '^walk_wbc_staircase$'
bash "$RUNNER" --dry-run walk_wbc | grep -q 'Executable: walk_wbc'
if bash "$RUNNER" does_not_exist >/dev/null 2>&1; then
    echo "invalid demo unexpectedly succeeded" >&2
    exit 1
fi
```

### Task 2: Implement the shared runner scripts

**Files:**
- Create: `projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/commands/run_openloong_demo_host.sh`
- Create: `projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/commands/container_run_openloong_demo.sh`

- [ ] **Step 1: Add the host-side interface**

Expose:

- `--list`
- `--dry-run`
- `<demo_name>`

and keep an explicit `demo -> executable` mapping table.

- [ ] **Step 2: Add the container-side executor**

Accept executable name and demo name as positional arguments, record precheck logs, then execute the target binary from the build directory.

### Task 3: Add wrapper generation

**Files:**
- Create: `projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/commands/generate_demo_wrapper_scripts.sh`
- Create: `projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/commands/generated/`

- [ ] **Step 1: Generate one thin wrapper per demo**

Each generated script should look like:

```bash
#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
bash "$SCRIPT_DIR/../run_openloong_demo_host.sh" walk_wbc "$@"
```

- [ ] **Step 2: Commit generated wrappers**

Generate wrappers for:

- `walk_wbc`
- `walk_wbc_staircase`
- `walk_mpc_wbc`
- `walk_mpc_wbc_joystick`
- `walk_wbc_joystick`
- `jump_mpc`
- `float_control`
- `walk_wbc_speed_test`

### Task 4: Verify

**Files:**
- Test: `projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/tests/test_openloong_demo_runner.sh`

- [ ] **Step 1: Run syntax and interface checks**

Run:

```bash
bash projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/tests/test_openloong_demo_runner.sh
bash -n projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/commands/run_openloong_demo_host.sh
bash -n projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/commands/container_run_openloong_demo.sh
bash -n projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/commands/generate_demo_wrapper_scripts.sh
```

Expected: all commands pass

- [ ] **Step 2: Inspect git diff summary**

Run: `git diff --stat`
Expected: only the new demo-runner files and docs appear

