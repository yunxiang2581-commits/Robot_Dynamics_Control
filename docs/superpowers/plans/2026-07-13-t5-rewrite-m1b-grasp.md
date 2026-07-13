# T5 Rewrite — Plan M1b: ApproachUnder + Grasp/Weld + Verify-Lift

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> Design spec: `docs/superpowers/specs/2026-07-13-t5-pick-place-rewrite-m1b-design.md`. This is **part 2 of 2** of milestone M1 (M1a = walk→crouch, done+committed `cb319c37`). M1c = payload stand-up.

**Goal:** Append two phases after `CrouchPhase` so the robot descends under the basket handle (pose6), does a gentle contact-seek grasp with a force-budget freeze, welds on a stable under-contact, and lifts ~3 cm to confirm the basket is caught — without toppling or flicking the 0.27 kg basket.

**Architecture:** `StateCommand` (existing struct, `demo/t5_pick_place_fsm.h:111`) stays the sole contract; the main loop at `walk_mpc_wbc_t5_pick_place.cpp:2438` consumes it. Two new `Phase` modules (`ApproachUnderPhase`, `GraspPhase`) are appended to the M1a `Scheduler` phase list behind `OPENLOONG_T5_NEW_FSM`. Each emits hand commands only through `HandCommandBuilder` and waypoints only through `HandleGeometry`; all params live in `T5Config`.

**Tech Stack:** C++17, Eigen, standalone `int main()` assert tests (no ctest), build under WSL2+Docker (`openloong-ubuntu22-build:local`). Work branch: `codex/t5-rewrite-m1`, worktree `C:/Users/Administrator/.config/superpowers/worktrees/OpenLoong-Dyn-Control/t5-m3-source`. `tests/` is gitignored → `git add -f`.

---

## File Structure (M1b)

- Modify `demo/t5_config.h` — extend `GraspCfg` with `contact_seek_z_rate`, `under_contact_stable_n`, `pos_err_tol`, `verify_lift_z`.
- Create `demo/t5_phases_approach_under.h` — `ApproachUnderPhase` (pose6, staged CenterAxis→LowerUnder→ApproachUnder).
- Create `demo/t5_phases_grasp.h` — `GraspPhase` (position3+wrist-reg, contact-seek + force-budget freeze + weld gating + verify-lift).
- Create tests `tests/t5_approach_under_phase_test.cpp`, `tests/t5_grasp_phase_test.cpp`.
- Modify `CMakeLists.txt` — register the 2 new test executables.
- Modify `demo/walk_mpc_wbc_t5_pick_place.cpp` — append the 2 phases to the M1a `t5_new_fsm` phase list (after `CrouchPhase`).

Header-only modules in namespace `T5M1`, like the M1a phases.

---

## Verify-before-code notes (check these exact names against the tree before writing each phase body)

1. **Contact fields:** M1a `CrouchPhase.checkAbort` reads `ctx.robot.t5_basket_handle_contacts`. DataBus (`common/data_bus.h:74-80`) has `t5_basket_handle_under_contacts`, `t5_basket_handle_side_contacts`, `t5_basket_handle_top_non_under_contacts`, `t5_basket_handle_force_norm_max`, `t5_basket_handle_under_force_norm_max`. **The runtime DIAG also prints `robot_handle_*` variants** (demo-computed, robot-handle-only per G2 `c33bef7e`). Before writing GraspPhase, `grep -n "robot_handle_under_contacts" demo/*.cpp common/*.h` — if there is a DataBus `t5_robot_handle_under_contacts` field, prefer it for the weld gate (cleaner). If only `t5_basket_handle_*` is on DataBus, use that (M1a already does). This plan's code uses `t5_basket_handle_*`; swap to the robot-handle field if it exists.
2. **Weld request:** `StateCommand.weld_request_valid` + `weld_active` (`t5_pick_place_fsm.h:128-129`). The old FSM sets them via a helper at `t5_pick_place_fsm.cpp:831-832` (`cmd.weld_request_valid = true; cmd.weld_active = active;`). Confirm the main loop honors `weld_request_valid`/`weld_active` under the new FSM (it should — same contract). If the main loop gates weld on `t5_state` (old FSM state), note it and set the fields anyway; validate in the sim gate.
3. **`crouchHoverTargetW` / `HandleGeometry`:** `HandleGeometry::fromRobot(robot)` returns `{frame, valid}`; `underPoint(x_back, z_down)` gives a point under the handle center along frame axes (`demo/t5_handle_geometry.h`). Confirm `frame.handle_center_W`, `frame.x_H_W`, `frame.z_H_W` member names before use.
4. **`weld_solref`:** M1b softens the weld to `0.05`. Confirm whether solref is read from env (`OPENLOONG_FSM_GRASP` weld var) or the scene XML. Prefer driving it from cfg/env; only edit `scene_basket.xml` if there is no env hook. This is a sim-gate concern, not a unit-test one.

---

### Task 1: Extend `GraspCfg`

**Files:** Modify `demo/t5_config.h`; Test `tests/t5_config_test.cpp` (extend existing).

- [ ] **Step 1: Add the failing assertion** to `tests/t5_config_test.cpp` (before the final `std::cout`):

```cpp
    // M1b: grasp seek/weld/verify-lift invariants.
    if (!(fast.grasp.contact_seek_z_rate > 0.0) ||
        !(fast.grasp.under_contact_stable_n > 0) ||
        !(fast.grasp.pos_err_tol > 0.0) ||
        !(fast.grasp.verify_lift_z > 0.0) ||
        !(fast.grasp.weld_solref > 0.0)) {
        std::cerr << "grasp seek/weld/verify-lift cfg must be positive\n";
        return 1;
    }
```

- [ ] **Step 2: Run to verify it fails to compile** (fields don't exist yet):

```bash
WT=/mnt/c/Users/Administrator/.config/superpowers/worktrees/OpenLoong-Dyn-Control/t5-m3-source
wsl.exe bash -lc "docker run --rm -v $WT:/work openloong-ubuntu22-build:local bash -lc 'cd /work && cmake --build build --target t5_config_test 2>&1 | tail -5'"
```
Expected: compile error — `contact_seek_z_rate` is not a member of `GraspCfg`.

- [ ] **Step 3: Extend `GraspCfg`** in `demo/t5_config.h` (replace the existing struct):

```cpp
struct GraspCfg {
    double force_budget_n = 18.0;      // tau_safe: freeze-on-contact threshold
    double hard_abort_n = 80.0;
    double wrist_posture_alpha = 0.7;  // strong reg holding the good wrist config
    double weld_solref = 0.05;         // softened weld (old rigid = 0.005)
    double contact_seek_z_rate = 0.004; // m/tick: gentle downward seek to contact
    int    under_contact_stable_n = 20; // weld gate: under-contact stable ticks
    double pos_err_tol = 0.03;          // weld gate: hand position error tolerance (m)
    double verify_lift_z = 0.03;        // post-weld verification lift height (m)
};
```

- [ ] **Step 4: Build + run** — expect PASS:

```bash
WT=/mnt/c/Users/Administrator/.config/superpowers/worktrees/OpenLoong-Dyn-Control/t5-m3-source
wsl.exe bash -lc "docker run --rm -v $WT:/work openloong-ubuntu22-build:local bash -lc 'cd /work && cmake --build build --target t5_config_test -j\$(nproc) && ./build/t5_config_test'"
```
Expected: `t5_config_test passed`.

- [ ] **Step 5: Commit**

```bash
git -C "$WT" add demo/t5_config.h
git -C "$WT" add -f tests/t5_config_test.cpp
git -C "$WT" commit -m "feat(t5-rewrite): extend GraspCfg for M1b seek/weld/verify-lift"
```

---

### Task 2: `ApproachUnderPhase` (pose6, staged descent under the handle)

**Files:** Create `demo/t5_phases_approach_under.h`; Test `tests/t5_approach_under_phase_test.cpp`.

- [ ] **Step 1: Write the failing test** `tests/t5_approach_under_phase_test.cpp`:

```cpp
#include "../demo/t5_phases_approach_under.h"

#include <iostream>

using namespace T5M1;

static DataBus underRobot(int side, int top)
{
    DataBus r(37);
    r.base_pos = Eigen::Vector3d(0.9, -0.06, 0.90);
    r.hd_r_pos_W = Eigen::Vector3d(0.9, -0.40, 0.90);
    r.hd_r_rot_W = Eigen::Matrix3d::Identity();
    r.t5_handle_canonical_valid = true;
    r.t5_handle_canonical_pos_W = Eigen::Vector3d(1.0, -0.32, 0.68);
    r.t5_handle_frame_y_W = Eigen::Vector3d(0, 1, 0);
    r.t5_handle_frame_z_W = Eigen::Vector3d(0, 0, 1);
    r.t5_basket_handle_side_contacts = side;
    r.t5_basket_handle_top_non_under_contacts = top;
    return r;
}

int main()
{
    T5Config cfg; PhaseState st; PickPlaceTarget tgt;

    // (a) side contact during approach -> abort with approach_under_side_contact
    {
        DataBus r = underRobot(/*side=*/1, /*top=*/0);
        Ctx ctx{r, st, tgt, cfg, 5.0};
        ApproachUnderPhase ph; ph.onEnter(ctx);
        auto ab = ph.checkAbort(ctx);
        if (!ab || ab->reason.find("side") == std::string::npos) {
            std::cerr << "side contact must abort approach_under\n"; return 1;
        }
    }
    // (b) hover command is pose6 (mode 2), no contact
    {
        DataBus r = underRobot(0, 0);
        Ctx ctx{r, st, tgt, cfg, 5.0};
        ApproachUnderPhase ph; ph.onEnter(ctx);
        StateCommand c = ph.step(ctx);
        if (c.hand_task_mode != 2 || !c.hand_des_valid) {
            std::cerr << "approach_under must command pose6 (mode 2)\n"; return 1;
        }
        // still crouched (leg-IK crouch overlay preserved)
        if (!c.use_leg_ik_crouch) {
            std::cerr << "approach_under must keep the crouch stance\n"; return 1;
        }
    }
    std::cout << "t5_approach_under_phase_test passed\n";
    return 0;
}
```

- [ ] **Step 2: Verify-before-code** — confirm the DataBus handle-frame members and `HandleGeometry` API:

```bash
WT=/mnt/c/Users/Administrator/.config/superpowers/worktrees/OpenLoong-Dyn-Control/t5-m3-source
grep -n "t5_handle_canonical_valid\|t5_handle_canonical_pos_W\|t5_handle_frame_y_W\|t5_handle_frame_z_W" "$WT/common/data_bus.h"
grep -n "underPoint\|fromRobot\|handle_center_W\|x_H_W\|z_H_W" "$WT/demo/t5_handle_geometry.h"
```
Expected: all names resolve. If a member differs (e.g. canonical-rot name), adjust the test's `underRobot` and the phase body to match.

- [ ] **Step 3: Create `demo/t5_phases_approach_under.h`**:

```cpp
#pragma once

// T5 rewrite (M1b) — ApproachUnderPhase: from the stable crouch hover, descend
// the hand UNDER the basket handle in staged waypoints (CenterAxis -> LowerUnder
// -> ApproachUnder) computed from the single HandleGeometry. Orientation is
// LOCKED in pose6 (mode 2) using the crouch-captured hand rotation, far from the
// handle where pose6 is safe (M1a proved it stable). Any side/top handle contact
// during the descent is an abort -> Scheduler routes to SafeHold.

#include "t5_fsm_helpers.h"        // configureLegIkCrouchCommand, readEnvDouble
#include "t5_hand_command_builder.h"
#include "t5_handle_geometry.h"
#include "t5_phase.h"

namespace T5M1 {

class ApproachUnderPhase : public Phase {
public:
    const char *name() const override { return "ApproachUnder"; }

    void onEnter(Ctx &ctx) override {
        ctx.state.entry_time = ctx.simTime;
        ctx.state.entry_time_valid = true;
        // Lock the crouch-captured hand orientation for the whole descent.
        if (!ctx.state.captured_right_hand_rot_valid &&
            ctx.robot.hd_r_rot_W.allFinite()) {
            ctx.state.captured_right_hand_rot_W = ctx.robot.hd_r_rot_W;
            ctx.state.captured_right_hand_rot_valid = true;
        }
    }

    std::optional<Abort> checkAbort(const Ctx &ctx) const override {
        if (ctx.robot.t5_basket_handle_side_contacts > 0) {
            return Abort{"approach_under_side_contact"};
        }
        if (ctx.robot.t5_basket_handle_top_non_under_contacts > 0) {
            return Abort{"approach_under_top_contact"};
        }
        HandleGeometry g = HandleGeometry::fromRobot(ctx.robot);
        if (!g.valid) {
            return Abort{"approach_under_invalid_pose"};
        }
        return std::nullopt;
    }

    StateCommand step(Ctx &ctx) override {
        HandleGeometry g = HandleGeometry::fromRobot(ctx.robot);
        // Staged target: CenterAxis (over handle center) -> LowerUnder (drop to
        // handle-under height) -> ApproachUnder (advance to the under front
        // point). Stage by elapsed time within the phase, cfg-driven.
        const double t = ctx.state.entry_time_valid
                             ? ctx.simTime - ctx.state.entry_time : 0.0;
        const double t_center = ctx.cfg.approach.center_axis_s;
        const double t_lower  = t_center + ctx.cfg.approach.lower_under_s;
        Eigen::Vector3d target_W;
        const char *reason;
        if (t < t_center) {
            target_W = g.valid ? g.underPoint(0.0, 0.0)     // over handle center
                               : ctx.robot.hd_r_pos_W;
            reason = "approach_center_axis";
        } else if (t < t_lower) {
            target_W = g.valid ? g.underPoint(0.0, ctx.cfg.approach.under_x_offset)
                               : ctx.robot.hd_r_pos_W;
            reason = "approach_lower_under";
        } else {
            target_W = g.valid ? g.underPoint(ctx.cfg.approach.under_x_offset,
                                              ctx.cfg.approach.under_x_offset)
                               : ctx.robot.hd_r_pos_W;
            reason = "approach_under";
        }

        HandGoal goal;
        goal.mode = ConstraintMode::Pose6HandleFrame;   // lock orientation
        goal.pos_W = target_W;
        goal.rot_W = ctx.state.captured_right_hand_rot_valid
                         ? ctx.state.captured_right_hand_rot_W
                         : palmUpRotationW();
        goal.reason = reason;
        goal.has_freeze_pos = false;
        StateCommand cmd = buildHandCommand(goal, /*contact_force_n=*/0.0,
                                            /*force_budget_n=*/0.0);
        // Keep the crouch stance underneath (base_z ~0.90), same overlay as
        // CrouchPhase.
        const double fe_z = readEnvDouble("OPENLOONG_FSM_CROUCH_FE_Z",
                                          ctx.cfg.crouch.fe_z_des);
        configureLegIkCrouchCommand(cmd, fe_z, reason);
        return cmd;
    }

    bool done(const Ctx &ctx) const override {
        // Done once the full stage schedule has elapsed and the hand is at the
        // under front point within tolerance (position error small).
        if (!ctx.state.entry_time_valid) return false;
        const double t = ctx.simTime - ctx.state.entry_time;
        const double t_total = ctx.cfg.approach.center_axis_s +
                               ctx.cfg.approach.lower_under_s +
                               ctx.cfg.approach.approach_under_s;
        if (t < t_total) return false;
        HandleGeometry g = HandleGeometry::fromRobot(ctx.robot);
        if (!g.valid) return false;
        const Eigen::Vector3d under =
            g.underPoint(ctx.cfg.approach.under_x_offset,
                         ctx.cfg.approach.under_x_offset);
        return (ctx.robot.hd_r_pos_W - under).norm() <= ctx.cfg.grasp.pos_err_tol;
    }
};

}  // namespace T5M1
```

- [ ] **Step 4: Build + run the test** — expect PASS:

```bash
WT=/mnt/c/Users/Administrator/.config/superpowers/worktrees/OpenLoong-Dyn-Control/t5-m3-source
# add to CMake first (Task 4 registers it); for now compile ad hoc against fsm.cpp+core
wsl.exe bash -lc "docker run --rm -v $WT:/work openloong-ubuntu22-build:local bash -lc 'cd /work && g++-11 -std=c++17 -I. -Ithird_party/eigen tests/t5_approach_under_phase_test.cpp demo/t5_pick_place_fsm.cpp -o /tmp/aut -lpthread 2>&1 | tail -8 && /tmp/aut'"
```
Expected: `t5_approach_under_phase_test passed`. (If ad-hoc include paths differ, defer the run to Task 4 after CMake registration; the phase must at least compile.)

- [ ] **Step 5: Commit**

```bash
git -C "$WT" add demo/t5_phases_approach_under.h
git -C "$WT" add -f tests/t5_approach_under_phase_test.cpp
git -C "$WT" commit -m "feat(t5-rewrite): ApproachUnderPhase pose6 staged descent under handle"
```

---

### Task 3: `GraspPhase` (contact-seek + force-budget freeze + weld gating + verify-lift)

**Files:** Create `demo/t5_phases_grasp.h`; Test `tests/t5_grasp_phase_test.cpp`.

- [ ] **Step 1: Write the failing test** `tests/t5_grasp_phase_test.cpp`:

```cpp
#include "../demo/t5_phases_grasp.h"

#include <iostream>

using namespace T5M1;

static DataBus graspRobot(int under, int side, int top, double force)
{
    DataBus r(37);
    r.base_pos = Eigen::Vector3d(0.9, -0.06, 0.90);
    r.hd_r_pos_W = Eigen::Vector3d(1.0, -0.32, 0.66);
    r.hd_r_rot_W = Eigen::Matrix3d::Identity();
    r.t5_handle_canonical_valid = true;
    r.t5_handle_canonical_pos_W = Eigen::Vector3d(1.0, -0.32, 0.68);
    r.t5_handle_frame_y_W = Eigen::Vector3d(0, 1, 0);
    r.t5_handle_frame_z_W = Eigen::Vector3d(0, 0, 1);
    r.t5_basket_handle_under_contacts = under;
    r.t5_basket_handle_side_contacts = side;
    r.t5_basket_handle_top_non_under_contacts = top;
    r.t5_basket_handle_force_norm_max = force;
    return r;
}

int main()
{
    T5Config cfg; PhaseState st; PickPlaceTarget tgt;

    // (a) grasp uses position3 + strong wrist reg (mode 1)
    {
        DataBus r = graspRobot(0, 0, 0, 0.0);
        Ctx ctx{r, st, tgt, cfg, 5.0};
        GraspPhase ph; ph.onEnter(ctx);
        StateCommand c = ph.step(ctx);
        if (c.hand_task_mode != 1 || !c.right_arm_posture_target_valid ||
            c.right_arm_posture_alpha < 0.69) {
            std::cerr << "grasp must be mode 1 + strong wrist reg\n"; return 1;
        }
    }
    // (b) force >= budget freezes the commanded hand position (no over-press)
    {
        DataBus r = graspRobot(0, 0, 0, /*force=*/25.0);  // > budget 18
        r.hd_r_pos_W = Eigen::Vector3d(0.90, -0.40, 0.66);
        Ctx ctx{r, st, tgt, cfg, 5.0};
        GraspPhase ph; ph.onEnter(ctx);
        StateCommand c = ph.step(ctx);
        if ((c.hand_pos_des_W - r.hd_r_pos_W).norm() > 1e-9) {
            std::cerr << "force >= budget must freeze commanded position\n"; return 1;
        }
    }
    // (c) top contact must NOT weld
    {
        DataBus r = graspRobot(/*under=*/0, 0, /*top=*/1, 10.0);
        Ctx ctx{r, st, tgt, cfg, 5.0};
        GraspPhase ph; ph.onEnter(ctx);
        ph.step(ctx);
        if (ph.weldRequested()) {
            std::cerr << "top contact must not weld\n"; return 1;
        }
    }
    // (d) stable under-contact in budget -> weld requested after N ticks
    {
        DataBus r = graspRobot(/*under=*/1, 0, 0, /*force=*/10.0);
        r.hd_r_pos_W = r.t5_handle_canonical_pos_W;  // at handle -> pos err ~0
        Ctx ctx{r, st, tgt, cfg, 5.0};
        GraspPhase ph; ph.onEnter(ctx);
        for (int i = 0; i < cfg.grasp.under_contact_stable_n + 1; ++i) {
            ctx.simTime = 5.0 + 0.01 * i;
            StateCommand c = ph.step(ctx);
            (void)c;
        }
        if (!ph.weldRequested()) {
            std::cerr << "stable under-contact in budget must request weld\n"; return 1;
        }
    }
    // (e) force >= hard abort -> abort
    {
        DataBus r = graspRobot(0, 0, 0, /*force=*/90.0);  // > hard_abort 80
        Ctx ctx{r, st, tgt, cfg, 5.0};
        GraspPhase ph; ph.onEnter(ctx);
        auto ab = ph.checkAbort(ctx);
        if (!ab || ab->reason.find("force") == std::string::npos) {
            std::cerr << "force over hard abort must abort\n"; return 1;
        }
    }
    std::cout << "t5_grasp_phase_test passed\n";
    return 0;
}
```

- [ ] **Step 2: Verify-before-code** — confirm the contact/force/weld field names (see verify-before-code note 1 & 2):

```bash
WT=/mnt/c/Users/Administrator/.config/superpowers/worktrees/OpenLoong-Dyn-Control/t5-m3-source
grep -n "t5_basket_handle_under_contacts\|t5_basket_handle_force_norm_max\|t5_robot_handle" "$WT/common/data_bus.h"
grep -n "weld_request_valid\|weld_active" "$WT/demo/t5_pick_place_fsm.h"
```
Expected: contact/force fields exist on DataBus; `weld_request_valid`/`weld_active` on `StateCommand`. If a `t5_robot_handle_under_contacts` field exists, use it in place of `t5_basket_handle_under_contacts` for the weld gate.

- [ ] **Step 3: Create `demo/t5_phases_grasp.h`**:

```cpp
#pragma once

// T5 rewrite (M1b) — GraspPhase: gentle contact-seek grasp of the basket handle.
// Constraint switches to position3 + STRONG wrist regularization (mode 1) to
// avoid G1's 53.7deg finger-axis bend when the finger axis meets the bar. The
// hand seeks DOWNWARD at a small rate; HandCommandBuilder's force-budget freeze
// holds position once contact force >= tau_safe, so it never over-presses or
// flicks the light basket. A dynamic-weld is requested only on a STABLE
// under-surface contact within budget and position tolerance. After welding, a
// small verify-lift confirms the basket is caught. Any hard-abort force / lost
// contact / top-or-side contact routes to SafeHold.

#include "t5_fsm_helpers.h"
#include "t5_hand_command_builder.h"
#include "t5_handle_geometry.h"
#include "t5_phase.h"

namespace T5M1 {

class GraspPhase : public Phase {
public:
    const char *name() const override { return "Grasp"; }
    bool weldRequested() const { return weld_requested_; }

    void onEnter(Ctx &ctx) override {
        ctx.state.entry_time = ctx.simTime;
        ctx.state.entry_time_valid = true;
        seek_pos_valid_ = false;
        under_stable_count_ = 0;
        weld_requested_ = false;
        lift_started_ = false;
        // Capture the good wrist posture to hold via strong reg.
        if (ctx.robot.q.size() >= 7) {
            captured_arm_q_ = ctx.robot.q.tail(7);   // NOTE: verify right-arm q slice
            captured_arm_q_valid_ = true;
        }
    }

    std::optional<Abort> checkAbort(const Ctx &ctx) const override {
        if (ctx.robot.t5_basket_handle_force_norm_max >= ctx.cfg.grasp.hard_abort_n) {
            return Abort{"grasp_force_abort"};
        }
        if (ctx.robot.t5_basket_handle_top_non_under_contacts > 0) {
            return Abort{"grasp_top_contact"};
        }
        if (ctx.robot.t5_basket_handle_side_contacts > 0) {
            return Abort{"grasp_side_contact"};
        }
        if (weld_requested_ &&
            ctx.robot.t5_basket_handle_under_contacts == 0) {
            return Abort{"grasp_lost_contact"};
        }
        return std::nullopt;
    }

    StateCommand step(Ctx &ctx) override {
        const double force = ctx.robot.t5_basket_handle_force_norm_max;
        const bool in_contact = force >= ctx.cfg.grasp.force_budget_n;

        // Seek target: descend from the current hand pos at a small z-rate until
        // contact freezes it. Once welding, hold + lift by verify_lift_z.
        if (!seek_pos_valid_) {
            seek_pos_ = ctx.robot.hd_r_pos_W;
            seek_pos_valid_ = true;
        }
        if (weld_requested_) {
            if (!lift_started_) { lift_base_ = seek_pos_; lift_started_ = true; }
            seek_pos_ = lift_base_ + Eigen::Vector3d(0, 0, ctx.cfg.grasp.verify_lift_z);
        } else if (!in_contact) {
            seek_pos_.z() -= ctx.cfg.grasp.contact_seek_z_rate;   // gentle down
        }
        // else: in contact but not yet welded -> hold (freeze handles it).

        HandGoal goal;
        goal.mode = ConstraintMode::Position3WithWristReg;   // avoid finger bend
        goal.pos_W = seek_pos_;
        goal.rot_W = ctx.robot.hd_r_rot_W;
        goal.posture_target = captured_arm_q_valid_
                                  ? captured_arm_q_
                                  : Eigen::Matrix<double, 7, 1>::Zero();
        goal.posture_valid = captured_arm_q_valid_;
        goal.posture_alpha = ctx.cfg.grasp.wrist_posture_alpha;
        goal.reason = weld_requested_ ? "grasp_verify_lift" : "grasp_seek";
        goal.freeze_pos_on_budget = ctx.robot.hd_r_pos_W;   // freeze where we are
        goal.has_freeze_pos = true;
        StateCommand cmd = buildHandCommand(goal, force, ctx.cfg.grasp.force_budget_n);

        // Weld gate: stable under-contact, in budget, hand near handle.
        const bool under = ctx.robot.t5_basket_handle_under_contacts > 0;
        const bool no_illegal = ctx.robot.t5_basket_handle_side_contacts == 0 &&
                                ctx.robot.t5_basket_handle_top_non_under_contacts == 0;
        HandleGeometry g = HandleGeometry::fromRobot(ctx.robot);
        const bool near_handle =
            g.valid &&
            (ctx.robot.hd_r_pos_W - g.frame.handle_center_W).norm()
                <= ctx.cfg.grasp.pos_err_tol;
        const bool in_budget = force > 0.0 && force < ctx.cfg.grasp.hard_abort_n;
        if (!weld_requested_ && under && no_illegal && near_handle && in_budget) {
            ++under_stable_count_;
            if (under_stable_count_ >= ctx.cfg.grasp.under_contact_stable_n) {
                weld_requested_ = true;
            }
        } else if (!weld_requested_) {
            under_stable_count_ = 0;
        }
        cmd.weld_request_valid = weld_requested_;
        cmd.weld_active = weld_requested_;

        // Keep the crouch stance underneath.
        const double fe_z = readEnvDouble("OPENLOONG_FSM_CROUCH_FE_Z",
                                          ctx.cfg.crouch.fe_z_des);
        configureLegIkCrouchCommand(cmd, fe_z, goal.reason);
        cmd.weld_request_valid = weld_requested_;   // re-assert (leg helper may clear reason only)
        cmd.weld_active = weld_requested_;
        return cmd;
    }

    bool done(const Ctx &ctx) const override {
        // Done once welded AND the verify-lift has been commanded to height (the
        // sim gate asserts the basket actually followed).
        if (!weld_requested_ || !lift_started_) return false;
        const double lifted = seek_pos_.z() - lift_base_.z();
        return lifted >= ctx.cfg.grasp.verify_lift_z - 1e-6;
    }

private:
    Eigen::Vector3d seek_pos_ = Eigen::Vector3d::Zero();
    bool seek_pos_valid_ = false;
    Eigen::Vector3d lift_base_ = Eigen::Vector3d::Zero();
    bool lift_started_ = false;
    int under_stable_count_ = 0;
    bool weld_requested_ = false;
    Eigen::Matrix<double, 7, 1> captured_arm_q_ =
        Eigen::Matrix<double, 7, 1>::Zero();
    bool captured_arm_q_valid_ = false;
};

}  // namespace T5M1
```

*(Verify-before-code: `ctx.robot.q.tail(7)` assumes the right-arm joints are the last 7 of `q`. Confirm the right-arm q slice against how M1a's CrouchPhase / the old FSM reads `captured_right_arm_q` (`t5_context.h` has a `captured_right_arm_q` field). If the right arm is a different slice, use that index. If unsure, reuse M1a's posture-capture helper.)*

- [ ] **Step 4: Build + run the test** — expect PASS (after CMake in Task 4, or ad hoc):

```bash
WT=/mnt/c/Users/Administrator/.config/superpowers/worktrees/OpenLoong-Dyn-Control/t5-m3-source
wsl.exe bash -lc "docker run --rm -v $WT:/work openloong-ubuntu22-build:local bash -lc 'cd /work && cmake --build build --target t5_grasp_phase_test -j\$(nproc) && ./build/t5_grasp_phase_test'"
```
Expected: `t5_grasp_phase_test passed`.

- [ ] **Step 5: Commit**

```bash
git -C "$WT" add demo/t5_phases_grasp.h
git -C "$WT" add -f tests/t5_grasp_phase_test.cpp
git -C "$WT" commit -m "feat(t5-rewrite): GraspPhase contact-seek + force-budget weld + verify-lift"
```

---

### Task 4: Register tests + wire the two phases into the phase list

**Files:** Modify `CMakeLists.txt`; Modify `demo/walk_mpc_wbc_t5_pick_place.cpp`.

- [ ] **Step 1: Register the 2 new tests** in `CMakeLists.txt` (next to the M1a `t5_crouch_phase_test`):

```cmake
add_executable(t5_approach_under_phase_test tests/t5_approach_under_phase_test.cpp demo/t5_pick_place_fsm.cpp)
add_executable(t5_grasp_phase_test tests/t5_grasp_phase_test.cpp demo/t5_pick_place_fsm.cpp)
target_link_libraries(t5_approach_under_phase_test core)
target_link_libraries(t5_grasp_phase_test core)
```

- [ ] **Step 2: Append the phases to the new-FSM phase list.** In `demo/walk_mpc_wbc_t5_pick_place.cpp`, find the M1a phase-list build (grep `make_unique<T5M1::CrouchPhase>`) and add after it, and add the two includes at the top:

```cpp
// top of file, with the other T5M1 phase includes:
#include "t5_phases_approach_under.h"
#include "t5_phases_grasp.h"
```
```cpp
    phases.push_back(std::make_unique<T5M1::CrouchPhase>());
    phases.push_back(std::make_unique<T5M1::ApproachUnderPhase>());   // M1b
    phases.push_back(std::make_unique<T5M1::GraspPhase>());           // M1b
```

- [ ] **Step 3: Build everything + run all M1a/M1b unit tests** — expect all PASS:

```bash
WT=/mnt/c/Users/Administrator/.config/superpowers/worktrees/OpenLoong-Dyn-Control/t5-m3-source
wsl.exe bash -lc "docker run --rm -v $WT:/work openloong-ubuntu22-build:local bash -lc 'cd /work && cmake -S . -B build -DCMAKE_C_COMPILER=gcc-11 -DCMAKE_CXX_COMPILER=g++-11 >/dev/null && cmake --build build --target t5_config_test t5_approach_under_phase_test t5_grasp_phase_test t5_scheduler_test t5_locomotion_phase_test t5_crouch_phase_test walk_mpc_wbc_t5_pick_place -j\$(nproc) 2>&1 | tail -4 && cd build && ./t5_config_test && ./t5_approach_under_phase_test && ./t5_grasp_phase_test && ./t5_scheduler_test && ./t5_locomotion_phase_test && ./t5_crouch_phase_test'"
```
Expected: each prints `... passed`; the demo target links clean.

- [ ] **Step 4: Commit**

```bash
git -C "$WT" add CMakeLists.txt demo/walk_mpc_wbc_t5_pick_place.cpp
git -C "$WT" commit -m "build(t5-rewrite): register M1b tests + wire ApproachUnder/Grasp into scheduler"
```

---

### Task 5: Sim gate — grasp/weld/verify-lift on the new FSM

**Requires WSL2+Docker.** Uses the config `T5_pick_place_newfsm_m1a.env` (already forwards `OPENLOONG_T5_NEW_FSM=1`; the runner unsets inherited `OPENLOONG_*` and sources `--config`, so keep the flag in that file). Runner: `projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/commands/run_t5_pick_place_visual_wslg.sh --config <that env> --source-repo <worktree> --smoke-seconds 90`.

- [ ] **Step 1:** Bump `--smoke-seconds` to ~90 (grasp adds time past the ~20 s crouch). Run the sim.

- [ ] **Step 2: Acceptance (M1b subset):** from the runtime log `.../logs/02_*runtime.log`:
  - Phase sequence reaches `approach_center_axis → approach_lower_under → approach_under → grasp_seek → grasp_verify_lift` (grep `reason=`).
  - **No `side_contact`/`top_contact` during ApproachUnder** (all `t5_basket_handle_side_contacts`/`_top_non_under_contacts` == 0 while reason is `approach_*`).
  - **Grasp peak force `t5_basket_handle_force_norm_max` < `hard_abort_n` (80 N)**; no `grasp_force_abort`; the basket is not flicked away (`t5_handle_canonical_valid` stays true).
  - **Weld established on under-contact** (`weld_active=1` appears while `t5_basket_handle_under_contacts>0` and side/top == 0).
  - **Verify-lift:** during `grasp_verify_lift`, the hand rises ~`verify_lift_z` and the basket follows (handle canonical z rises with the hand; relative hand↔handle displacement stays within tol).
  - **Base stays crouched-stable** the whole time (`base_z` ≈ 0.90, `|roll|`,`|pitch|` small — same check M1a passed).

- [ ] **Step 3: Record** the run + log to `outputs/t5_m1b_grasp_<date>/` (GATE_SUMMARY.md + runtime.log + a pose/force timeline), mirroring `outputs/t5_m1a_walkcrouch_20260713/`.

- [ ] **Step 4:** If a gate fails, debug with the same static-first method that fixed M1a (compare against the old FSM's grasp branch `t5_pick_place_fsm.cpp` GRASP_LOCK region; the known risks are force spike / flick and weld-gate misclassification). Do NOT tune blindly — read the old grasp branch and match it.

---

## Self-Review

- **Spec coverage:** §4.1 ApproachUnder (pose6 staged CenterAxis→LowerUnder→ApproachUnder, side/top abort) → Task 2. §4.2 Grasp (position3+wrist-reg, contact-seek, force-budget freeze, weld gating on under-contact, verify-lift, hard-abort) → Task 3. §4.3 SafeHold reuse → inherited from M1a (abort routing). §5 Config → Task 1. §6 unit tests → Tasks 1,2,3; sim gate → Task 5. §7 file structure → Tasks 2,3,4. **Deferred to M1c (correctly out of this plan):** payload feedforward, full stand-up balance.
- **Placeholder scan:** No TBD/TODO. Concrete numbers flagged as M1b starting points (Task 1 note); tests pin only invariants. Verify-before-code notes name the exact greps to run for the 4 uncertain interface points (contact-field naming, weld-request honoring, HandleGeometry members, right-arm q slice) — these are checks, not placeholders.
- **Type consistency:** `HandGoal{pos_W,rot_W,mode,posture_target,posture_valid,posture_alpha,reason,freeze_pos_on_budget,has_freeze_pos}` and `buildHandCommand(goal, contact_force_n, force_budget_n)` match M1a `t5_hand_command_builder.h`. `ConstraintMode::{Pose6HandleFrame,Position3WithWristReg}`, `configureLegIkCrouchCommand(cmd, fe_z, reason)`, `HandleGeometry::fromRobot`/`underPoint`/`frame.handle_center_W`, `StateCommand.{hand_task_mode,weld_request_valid,weld_active,use_leg_ik_crouch,right_arm_posture_*}`, `GraspCfg.{force_budget_n,hard_abort_n,wrist_posture_alpha,weld_solref,contact_seek_z_rate,under_contact_stable_n,pos_err_tol,verify_lift_z}`, `ApproachCfg.{center_axis_s,lower_under_s,approach_under_s,under_x_offset}` all defined in Task 1 or M1a and used consistently. `weldRequested()` accessor is on `GraspPhase` and used by its test. `T5M1` namespace throughout.
