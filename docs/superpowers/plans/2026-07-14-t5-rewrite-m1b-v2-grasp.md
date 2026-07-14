# T5 Rewrite — Plan M1b-v2: Under-Handle Grasp (port old FSM caging/axis-lift geometry)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> Design spec: `docs/superpowers/specs/2026-07-14-t5-pick-place-rewrite-m1b-v2-design.md`. Supersedes the ApproachUnder/Grasp/Crouch-posture parts of the v1 plan (`2026-07-13-t5-rewrite-m1b-grasp.md`). M1a is done+committed (`cb319c37`); M1b v1 phases exist and are wired (WIP `e6d5fc3c`) — this plan MODIFIES them.

**Goal:** Get the right hand UNDER the basket handle and grasp it — by holding the arm in an elbow-out/palm-up posture during the crouch (so the forearm never rests on the bar top), then a 3-segment axis_lift approach (descend-behind → insert-under-along-axis → hook-lift), welding only when the caging gate confirms the forearm is legally under the bar.

**Architecture:** `StateCommand` stays the sole contract. Route A (analytic axis_lift). Three modified `T5M1` phases behind `OPENLOONG_T5_NEW_FSM`: `CrouchPhase` (adds joint-space posture reg toward the fixed grasp pose, removes the failed v1 retract experiments), `ApproachUnderPhase` (3-segment axis waypoints via `HandleGeometry`), `GraspPhase` (hook-lift + `t5_caging_ready`-gated weld + verify-lift). All offsets/posture are the old FSM's proven values.

**Tech Stack:** C++17, Eigen, standalone `int main()` assert tests, WSL2+Docker (`openloong-ubuntu22-build:local`). Branch `codex/t5-rewrite-m1`, worktree `C:/Users/Administrator/.config/superpowers/worktrees/OpenLoong-Dyn-Control/t5-m3-source`. `tests/` gitignored → `git add -f`.

---

## Confirmed interfaces (verified in-tree, use these exact names)

- DataBus (`common/data_bus.h`): `t5_caging_ready` (:86), `t5_caging_no_illegal_contact` (:120), `t5_caging_r07_below_handle` (:122), `t5_caging_axis_within_segment` (:121), `t5_caging_lateral_aligned` (:123), `t5_caging_top_handle_contact` (:119), `t5_basket_handle_side_contacts`, `t5_basket_handle_top_non_under_contacts`, `t5_basket_handle_under_contacts`, `t5_basket_handle_force_norm_max`, `t5_handle_canonical_valid`, `t5_handle_canonical_pos_W`, `t5_handle_frame_y_W`, `t5_handle_frame_z_W`, `base_pos`, `hd_r_pos_W`, `hd_r_rot_W`, `q`.
- `StateCommand` (`demo/t5_pick_place_fsm.h`): `right_arm_posture_target` (Matrix<double,7,1>, :146), `right_arm_posture_alpha` (:142), `right_arm_posture_target_valid` (:145); `weld_request_valid`/`weld_active` (:128-129); `hand_task_mode`, `hand_des_valid`, `use_leg_ik_crouch`. The main loop applies `right_arm_posture_target` unconditionally (walk_mpc:125-126), independent of the hand task — so setting it during the crouch is a pure joint-space reg (no Cartesian task, no descent destabilization).
- `HandGoal`/`buildHandCommand` (`demo/t5_hand_command_builder.h`), `HandleGeometry::fromRobot`/`frame.{handle_center_W,x_H_W,y_H_W,z_H_W}` (`demo/t5_handle_geometry.h`), `configureLegIkCrouchCommand`/`readEnvDouble`/`palmUpRotationW` (`demo/t5_fsm_helpers.h`), `ConstraintMode::{Pose6HandleFrame,Position3WithWristReg}`.

---

### Task 1: `t5FixedRightArmPostureQ()` helper + config v2 fields

**Files:** Create `demo/t5_fixed_arm_posture.h`; Modify `demo/t5_config.h`; Test `tests/t5_config_test.cpp`.

- [ ] **Step 1: Create `demo/t5_fixed_arm_posture.h`**

```cpp
#pragma once

#include <Eigen/Dense>

namespace T5M1 {

// The old FSM's known-good grasp posture (right arm 7 DoF), captured from the
// visual grasp pose at simTime ~= 42.330 s (hand_track_task.cpp:113-120 /
// walk_mpc_wbc_t5_pick_place.cpp:131-138). Palm-up with the elbow swung OUT
// (joints 2,4 near +/-pi) so the forearm clears the handle top. CrouchPhase
// regularizes the right arm toward this during the crouch so the forearm never
// rests on the bar.
inline Eigen::Matrix<double, 7, 1> t5FixedRightArmPostureQ()
{
    Eigen::Matrix<double, 7, 1> q;
    q << 0.202922, -1.324359, -3.045075, 0.134528,
         -2.989420, -0.118958, 1.056396;
    return q;
}

}  // namespace T5M1
```

- [ ] **Step 2: Add failing assertions** to `tests/t5_config_test.cpp` before the final `std::cout`:

```cpp
    // M1b-v2: axis_lift waypoint + hook invariants.
    if (!(fast.approach.descend_s > 0.0) || !(fast.approach.insert_s > 0.0) ||
        !(fast.approach.axis_contact_offset > 0.0) ||
        !(fast.approach.axis_pre_offset > 0.0) ||
        !(fast.grasp.hook_lift_z > 0.0) || !(fast.grasp.hook_s > 0.0)) {
        std::cerr << "m1b-v2 axis/hook cfg must be positive\n";
        return 1;
    }
```

- [ ] **Step 3: Replace `ApproachCfg` and extend `GraspCfg`** in `demo/t5_config.h`:

```cpp
struct ApproachCfg {
    double hover_s = 0.5;            // reach to hover from crouch
    double descend_s = 0.40;         // descend behind the bar (GRASP_DESCEND_DURATION_S)
    double insert_s = 0.50;          // slide under along axis (GRASP_INSERT_DURATION_S)
    double pre_grasp_x = 0.20;       // hover back-off in x (AXIS_PRE_GRASP_X_OFFSET)
    double above_z = 0.06;           // hover above under-bar depth (AXIS_PRE_GRASP_ABOVE_Z)
    double under_z = 0.02;           // graspTargetW depth below bar (GRASP_UNDER_Z)
    double insert_x_offset = 0.08;   // graspTargetW back-off in x (GRASP_INSERT_X_OFFSET)
    double axis_contact_offset = 0.075;  // along-axis to under-contact (GRASP_AXIS_CONTACT_OFFSET mag)
    double axis_pre_offset = 0.04;       // along-axis back-off to descend start (GRASP_AXIS_PRE_OFFSET)
};
```
And add to `GraspCfg` (keep the existing fields from v1 Task 1):
```cpp
    double hook_lift_z = 0.03;       // hook up under the bar (GRASP_HOOK_LIFT_Z)
    double hook_s = 0.50;            // hook-lift segment duration
```

- [ ] **Step 4: Build + run** (WSL+Docker):
```bash
WT=/mnt/c/Users/Administrator/.config/superpowers/worktrees/OpenLoong-Dyn-Control/t5-m3-source
wsl.exe bash -lc "docker run --rm -v $WT:/work openloong-ubuntu22-build:local bash -lc 'cd /work && cmake --build build --target t5_config_test -j\$(nproc) 2>&1 | tail -3 && ./build/t5_config_test'"
```
Expected: `t5_config_test passed`.

- [ ] **Step 5: Commit**
```bash
git -C "$WT" add demo/t5_fixed_arm_posture.h demo/t5_config.h
git -C "$WT" add -f tests/t5_config_test.cpp
git -C "$WT" commit -m "feat(t5-rewrite): M1b-v2 fixed arm posture + axis_lift/hook config"
```

---

### Task 2: `CrouchPhase` — regularize arm toward the fixed posture (G1); drop v1 experiments

**Files:** Modify `demo/t5_phases_crouch.h`; Test `tests/t5_crouch_phase_test.cpp`.

- [ ] **Step 1: Update the crouch test.** Replace the contact-abort test block's expectations so that: (a) sustained side contact aborts; (b) a normal crouch step sets a strong posture reg toward the fixed pose. Overwrite `tests/t5_crouch_phase_test.cpp` body's case (b) and add a new case (c):

```cpp
    // (c) crouch commands a strong posture reg toward the fixed elbow-out pose,
    // so the forearm swings clear of the handle top (G1). Joint-space only.
    {
        DataBus r = crouchRobot(0.95, /*side=*/0, /*top=*/0);
        Ctx ctx{r, st, tgt, cfg, 3.2};
        CrouchPhase ph;
        ph.onEnter(ctx);
        StateCommand c = ph.step(ctx);
        if (!c.right_arm_posture_target_valid || c.right_arm_posture_alpha < 0.9) {
            std::cerr << "crouch must reg arm toward fixed posture (alpha>=0.9)\n";
            return 1;
        }
        const Eigen::Matrix<double,7,1> want = T5M1::t5FixedRightArmPostureQ();
        if ((c.right_arm_posture_target - want).norm() > 1e-9) {
            std::cerr << "crouch posture target must be t5FixedRightArmPostureQ\n";
            return 1;
        }
    }
```
Add `#include "../demo/t5_fixed_arm_posture.h"` at the top of the test.

- [ ] **Step 2: Modify `demo/t5_phases_crouch.h`.** (a) add `#include "t5_fixed_arm_posture.h"`. (b) In `step()`, after building `cmd` (both hover and descend branches) and before/after the leg overlay, set the posture reg unconditionally:

```cpp
        // G1: hold the right arm in the elbow-out/palm-up fixed grasp posture via
        // joint-space posture reg throughout the crouch, so the forearm swings to
        // the side and never rests on the handle top (the v1 crouch_top_contact
        // blocker). This is NOT a Cartesian hand task, so it does not destabilize
        // the descent (unlike a forward hand target).
        cmd.right_arm_posture_target_valid = true;
        cmd.right_arm_posture_target = t5FixedRightArmPostureQ();
        cmd.right_arm_posture_alpha =
            readEnvDouble("OPENLOONG_FSM_CROUCH_ARM_POSTURE_ALPHA", 1.0);
```
(c) **Remove the v1 experiments** that were empirically ruled out: delete the `OPENLOONG_FSM_CROUCH_RETRACT_ARM` branch (revert `step()` to the simple `in_hover_window ? pose6-hover : legs-only-descend`), delete the `side_contact_ticks_`/`top_contact_ticks_` sustained-counter members and the `OPENLOONG_FSM_CROUCH_CONTACT_ABORT_TICKS` gate — restore `checkAbort()` to abort immediately on `t5_basket_handle_side_contacts > 0` (reason `crouch_side_contact`) or `t5_basket_handle_top_non_under_contacts > 0` (reason `crouch_top_contact`). With G1 the forearm no longer grazes, so the simple immediate abort is correct again.

- [ ] **Step 3: Revert `crouchHoverTargetW`** in `demo/t5_fsm_helpers.h` to the simple pre-grasp hover (drop the v1 retracted high+back experiment): `hand_target = handleCenterW(target); hand_target.x() -= readEnvDouble("OPENLOONG_FSM_PRE_GRASP_X_OFFSET", 0.20); hand_target.z() += std::max(0.0, readEnvDouble("OPENLOONG_FSM_PRE_GRASP_ABOVE_Z", 0.04)); return hand_target;` and change the signature back to `(const PickPlaceTarget &target, const DataBus & /*robotState*/)`.

- [ ] **Step 4: Build + run** the crouch + config tests:
```bash
WT=/mnt/c/Users/Administrator/.config/superpowers/worktrees/OpenLoong-Dyn-Control/t5-m3-source
wsl.exe bash -lc "docker run --rm -v $WT:/work openloong-ubuntu22-build:local bash -lc 'cd /work && cmake --build build --target t5_crouch_phase_test t5_config_test t5_scheduler_test -j\$(nproc) 2>&1 | tail -3 && ./build/t5_crouch_phase_test && ./build/t5_config_test && ./build/t5_scheduler_test'"
```
Expected: all three `... passed`.

- [ ] **Step 5: Commit**
```bash
git -C "$WT" add demo/t5_phases_crouch.h demo/t5_fsm_helpers.h
git -C "$WT" add -f tests/t5_crouch_phase_test.cpp
git -C "$WT" commit -m "feat(t5-rewrite): M1b-v2 crouch holds fixed elbow-out posture (G1); drop v1 retract/tolerance"
```

---

### Task 3: `HandleGeometry` — axis_lift waypoint helpers

**Files:** Modify `demo/t5_handle_geometry.h`. Tested via the ApproachUnder/Grasp phase tests (Tasks 4-5).

- [ ] **Step 1: Add the waypoint helpers** to `struct HandleGeometry` (keep the existing `underPoint`/`axisUnderPoint`):

```cpp
    // --- axis_lift waypoints (old FSM graspAxis*UnderTargetW, fsm.cpp:433-488) ---
    // Base "under the bar" point: back in x, below the bar centerline.
    Eigen::Vector3d graspTargetUnder(double insert_x, double under_z) const
    {
        return frame.handle_center_W - insert_x * frame.x_H_W - under_z * frame.z_H_W;
    }
    // Under-contact point: graspTargetUnder shifted along the bar axis toward the
    // root side by contact_offset (final under-bar contact pocket).
    Eigen::Vector3d axisContactUnder(double insert_x, double under_z,
                                     double contact_offset) const
    {
        return graspTargetUnder(insert_x, under_z) - contact_offset * frame.y_H_W;
    }
    // Descend-start: the contact point backed off a further pre_offset along the
    // axis, so a vertical descent lands BEHIND the bar (not on top).
    Eigen::Vector3d axisStartUnder(double insert_x, double under_z,
                                   double contact_offset, double pre_offset) const
    {
        return axisContactUnder(insert_x, under_z, contact_offset) -
               pre_offset * frame.y_H_W;
    }
    // Hook-lift: contact point raised in z to seat the palm under the bar.
    Eigen::Vector3d axisLiftContact(double insert_x, double under_z,
                                    double contact_offset, double hook_lift_z) const
    {
        return axisContactUnder(insert_x, under_z, contact_offset) +
               hook_lift_z * frame.z_H_W;
    }
```

*(Verify-before-code: sign convention — the old FSM does `y += contact_offset` with `contact_offset = -0.075` (fsm.cpp:452,458) and `y -= pre_offset` with `pre_offset = 0.04` (:469,475). Here `axis_contact_offset`/`axis_pre_offset` are stored as POSITIVE magnitudes and subtracted along `+y_H_W`. Confirm `frame.y_H_W` points the same way as the old world `+y` axis toward the bar root; if the hand ends up on the wrong side of center in sim, flip the sign of the `* frame.y_H_W` terms.)*

- [ ] **Step 2: Commit** (compiles with the phase tests in Tasks 4-5; commit after Task 4 builds, or standalone-compile-check now by including the header in a scratch TU). Minimal check:
```bash
WT=/mnt/c/Users/Administrator/.config/superpowers/worktrees/OpenLoong-Dyn-Control/t5-m3-source
wsl.exe bash -lc "docker run --rm -v $WT:/work openloong-ubuntu22-build:local bash -lc 'cd /work && echo \"#include \\\"demo/t5_handle_geometry.h\\\"
int main(){return 0;}\" > /tmp/hg.cpp && g++-11 -std=c++17 -I. -Icommon -Ithird_party/eigen3 /tmp/hg.cpp demo/t5_pick_place_fsm.cpp -o /tmp/hg -lpthread 2>&1 | tail -5 && echo COMPILE_OK'"
```
Expected: `COMPILE_OK`.
```bash
git -C "$WT" add demo/t5_handle_geometry.h
git -C "$WT" commit -m "feat(t5-rewrite): HandleGeometry axis_lift under-bar waypoints"
```

---

### Task 4: `ApproachUnderPhase` — axis_lift 3-segment (hover → descend-behind → insert-under)

**Files:** Modify `demo/t5_phases_approach_under.h`; Test `tests/t5_approach_under_phase_test.cpp`.

- [ ] **Step 1: Overwrite the test** `tests/t5_approach_under_phase_test.cpp`:

```cpp
#include "../demo/t5_phases_approach_under.h"

#include <iostream>

using namespace T5M1;

static DataBus underRobot(int side, int top)
{
    DataBus r(37);
    r.base_pos = Eigen::Vector3d(0.9, -0.06, 0.90);
    r.hd_r_pos_W = Eigen::Vector3d(0.9, -0.40, 0.66);
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

    // (a) side/top contact aborts
    {
        DataBus r = underRobot(1, 0);
        Ctx ctx{r, st, tgt, cfg, 5.0};
        ApproachUnderPhase ph; ph.onEnter(ctx);
        auto ab = ph.checkAbort(ctx);
        if (!ab || ab->reason.find("side") == std::string::npos) {
            std::cerr << "side contact must abort approach\n"; return 1;
        }
    }
    // (b) pose6 palm-up, crouch held; DESCEND target is below bar + behind along axis
    {
        DataBus r = underRobot(0, 0);
        // put time in the descend segment (after hover_s)
        Ctx ctx{r, st, tgt, cfg, 5.0};
        ApproachUnderPhase ph; ph.onEnter(ctx);
        ctx.simTime = 5.0 + cfg.approach.hover_s + 0.01;
        StateCommand c = ph.step(ctx);
        if (c.hand_task_mode != 2 || !c.hand_des_valid || !c.use_leg_ik_crouch) {
            std::cerr << "approach must be pose6 + crouch\n"; return 1;
        }
        // descend target z must be below the handle centerline (0.68)
        if (c.hand_pos_des_W.z() >= 0.68) {
            std::cerr << "descend target must be below the bar\n"; return 1;
        }
    }
    std::cout << "t5_approach_under_phase_test passed\n";
    return 0;
}
```

- [ ] **Step 2: Overwrite `demo/t5_phases_approach_under.h` `step()`/`done()`** with the 3-segment axis_lift (keep the existing `onEnter`/`checkAbort` which already lock orientation + abort on side/top; ensure `checkAbort` uses `t5_basket_handle_side_contacts`/`_top_non_under_contacts`):

```cpp
    StateCommand step(Ctx &ctx) override {
        HandleGeometry g = HandleGeometry::fromRobot(ctx.robot);
        const auto &a = ctx.cfg.approach;
        const double t = ctx.state.entry_time_valid
                             ? ctx.simTime - ctx.state.entry_time : 0.0;
        // axis_lift: hover (reach from crouch) -> descend BEHIND the bar (under
        // depth, retracted along axis) -> insert UNDER along the axis. Never a
        // top-down drop.
        Eigen::Vector3d hover, descend, insert;
        if (g.valid) {
            descend = g.axisStartUnder(a.insert_x_offset, a.under_z,
                                       a.axis_contact_offset, a.axis_pre_offset);
            insert  = g.axisContactUnder(a.insert_x_offset, a.under_z,
                                         a.axis_contact_offset);
            hover   = descend + a.pre_grasp_x * g.frame.x_H_W +
                      (a.above_z + a.under_z) * g.frame.z_H_W;  // back + above depth
        } else {
            hover = descend = insert = ctx.robot.hd_r_pos_W;
        }
        Eigen::Vector3d target_W;
        const char *reason;
        if (t < a.hover_s) {
            target_W = hover; reason = "approach_hover";
        } else if (t < a.hover_s + a.descend_s) {
            const double s = clamp01((t - a.hover_s) / a.descend_s);
            target_W = hover + s * (descend - hover); reason = "approach_descend";
        } else {
            const double s = clamp01((t - a.hover_s - a.descend_s) / a.insert_s);
            target_W = descend + s * (insert - descend); reason = "approach_insert";
        }

        HandGoal goal;
        goal.mode = ConstraintMode::Pose6HandleFrame;
        goal.pos_W = target_W;
        goal.rot_W = ctx.state.captured_right_hand_rot_valid
                         ? ctx.state.captured_right_hand_rot_W
                         : palmUpRotationW();
        goal.reason = reason;
        goal.has_freeze_pos = false;
        StateCommand cmd = buildHandCommand(goal, 0.0, 0.0);
        const double fe_z = readEnvDouble("OPENLOONG_FSM_CROUCH_FE_Z",
                                          ctx.cfg.crouch.fe_z_des);
        configureLegIkCrouchCommand(cmd, fe_z, reason);
        return cmd;
    }

    bool done(const Ctx &ctx) const override {
        if (!ctx.state.entry_time_valid) return false;
        const auto &a = ctx.cfg.approach;
        const double t = ctx.simTime - ctx.state.entry_time;
        if (t < a.hover_s + a.descend_s + a.insert_s) return false;
        HandleGeometry g = HandleGeometry::fromRobot(ctx.robot);
        if (!g.valid) return false;
        const Eigen::Vector3d insert = g.axisContactUnder(
            a.insert_x_offset, a.under_z, a.axis_contact_offset);
        return (ctx.robot.hd_r_pos_W - insert).norm() <= ctx.cfg.grasp.pos_err_tol;
    }
```
Add a small file-local helper at the top of the namespace if not already present:
```cpp
inline double clamp01(double v) { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); }
```
Use `palmUpRotationW()` as the captured-rot fallback (the crouch already captured a palm-up orientation; ApproachUnder keeps it).

- [ ] **Step 3: Build + run** (ad hoc against fsm.cpp; Eigen at third_party/eigen3, DataBus in common/):
```bash
WT=/mnt/c/Users/Administrator/.config/superpowers/worktrees/OpenLoong-Dyn-Control/t5-m3-source
wsl.exe bash -lc "docker run --rm -v $WT:/work openloong-ubuntu22-build:local bash -lc 'cd /work && g++-11 -std=c++17 -I. -Icommon -Ithird_party/eigen3 tests/t5_approach_under_phase_test.cpp demo/t5_pick_place_fsm.cpp -o /tmp/aut -lpthread 2>&1 | tail -8 && /tmp/aut'"
```
Expected: `t5_approach_under_phase_test passed`.

- [ ] **Step 4: Commit**
```bash
git -C "$WT" add demo/t5_phases_approach_under.h
git -C "$WT" add -f tests/t5_approach_under_phase_test.cpp
git -C "$WT" commit -m "feat(t5-rewrite): ApproachUnderPhase axis_lift 3-segment (descend-behind -> insert-under)"
```

---

### Task 5: `GraspPhase` — hook-lift + caging-gated weld + verify-lift

**Files:** Modify `demo/t5_phases_grasp.h`; Test `tests/t5_grasp_phase_test.cpp`.

- [ ] **Step 1: Overwrite the test** `tests/t5_grasp_phase_test.cpp`:

```cpp
#include "../demo/t5_phases_grasp.h"

#include <iostream>

using namespace T5M1;

static DataBus graspRobot(bool caging_ready, double force)
{
    DataBus r(37);
    r.base_pos = Eigen::Vector3d(0.9, -0.06, 0.90);
    r.hd_r_pos_W = Eigen::Vector3d(1.0, -0.395, 0.66);
    r.hd_r_rot_W = Eigen::Matrix3d::Identity();
    r.t5_handle_canonical_valid = true;
    r.t5_handle_canonical_pos_W = Eigen::Vector3d(1.0, -0.32, 0.68);
    r.t5_handle_frame_y_W = Eigen::Vector3d(0, 1, 0);
    r.t5_handle_frame_z_W = Eigen::Vector3d(0, 0, 1);
    r.t5_basket_handle_force_norm_max = force;
    r.t5_caging_ready = caging_ready;
    r.t5_basket_handle_under_contacts = caging_ready ? 1 : 0;
    return r;
}

int main()
{
    T5Config cfg; PhaseState st; PickPlaceTarget tgt;

    // (a) mode 1 + strong wrist reg
    {
        DataBus r = graspRobot(false, 0.0);
        Ctx ctx{r, st, tgt, cfg, 5.0};
        GraspPhase ph; ph.onEnter(ctx);
        StateCommand c = ph.step(ctx);
        if (c.hand_task_mode != 1 || c.right_arm_posture_alpha < 0.69) {
            std::cerr << "grasp must be mode 1 + wrist reg\n"; return 1;
        }
    }
    // (b) caging NOT ready -> no weld even with contact
    {
        DataBus r = graspRobot(false, 10.0);
        Ctx ctx{r, st, tgt, cfg, 5.0};
        GraspPhase ph; ph.onEnter(ctx);
        for (int i = 0; i < cfg.grasp.under_contact_stable_n + 1; ++i) {
            ctx.simTime = 5.0 + 0.01 * i; ph.step(ctx);
        }
        if (ph.weldRequested()) { std::cerr << "no weld without caging_ready\n"; return 1; }
    }
    // (c) caging ready + in-budget force stable N -> weld requested
    {
        DataBus r = graspRobot(true, 10.0);
        Ctx ctx{r, st, tgt, cfg, 5.0};
        GraspPhase ph; ph.onEnter(ctx);
        for (int i = 0; i < cfg.grasp.under_contact_stable_n + 1; ++i) {
            ctx.simTime = 5.0 + 0.01 * i; ph.step(ctx);
        }
        if (!ph.weldRequested()) { std::cerr << "caging_ready+stable must weld\n"; return 1; }
    }
    // (d) force over hard abort -> abort
    {
        DataBus r = graspRobot(false, 90.0);
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

- [ ] **Step 2: Overwrite `demo/t5_phases_grasp.h`** to: hook-lift segment (contact→lift target), position3+wrist-reg, force-budget freeze, weld gate = `t5_caging_ready` (§4) + in-budget force stable N + verify-lift. Keep the `q.segment<7>(14)` posture capture and `weldRequested()` accessor from v1.

```cpp
#pragma once

#include "t5_fixed_arm_posture.h"
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
        stable_count_ = 0;
        weld_requested_ = false;
        lift_started_ = false;
        seek_valid_ = false;
    }

    std::optional<Abort> checkAbort(const Ctx &ctx) const override {
        if (ctx.robot.t5_basket_handle_force_norm_max >= ctx.cfg.grasp.hard_abort_n)
            return Abort{"grasp_force_abort"};
        if (ctx.robot.t5_basket_handle_top_non_under_contacts > 0)
            return Abort{"grasp_top_contact"};
        if (ctx.robot.t5_basket_handle_side_contacts > 0)
            return Abort{"grasp_side_contact"};
        if (weld_requested_ && ctx.robot.t5_basket_handle_under_contacts == 0)
            return Abort{"grasp_lost_contact"};
        return std::nullopt;
    }

    StateCommand step(Ctx &ctx) override {
        HandleGeometry g = HandleGeometry::fromRobot(ctx.robot);
        const auto &a = ctx.cfg.approach;
        const auto &gc = ctx.cfg.grasp;
        const double force = ctx.robot.t5_basket_handle_force_norm_max;
        const double t = ctx.state.entry_time_valid
                             ? ctx.simTime - ctx.state.entry_time : 0.0;

        // Hook-lift: from the under-contact point up by hook_lift_z; after weld,
        // verify-lift up by verify_lift_z. Position3 + strong wrist reg (avoid the
        // G1 finger-axis bend); force-budget freeze so we never over-press.
        Eigen::Vector3d contact = g.valid
            ? g.axisContactUnder(a.insert_x_offset, a.under_z, a.axis_contact_offset)
            : ctx.robot.hd_r_pos_W;
        Eigen::Vector3d target;
        if (weld_requested_) {
            if (!lift_started_) { lift_base_ = ctx.robot.hd_r_pos_W; lift_started_ = true; }
            target = lift_base_ + Eigen::Vector3d(0, 0, gc.verify_lift_z);
        } else {
            const double s = clamp01(t / gc.hook_s);
            target = contact + s * (g.valid ? gc.hook_lift_z * g.frame.z_H_W
                                            : Eigen::Vector3d(0,0,gc.hook_lift_z));
        }
        if (!seek_valid_) { seek_valid_ = true; }

        HandGoal goal;
        goal.mode = ConstraintMode::Position3WithWristReg;
        goal.pos_W = target;
        goal.rot_W = ctx.robot.hd_r_rot_W;
        goal.posture_target = t5FixedRightArmPostureQ();
        goal.posture_valid = true;
        goal.posture_alpha = gc.wrist_posture_alpha;
        goal.reason = weld_requested_ ? "grasp_verify_lift" : "grasp_hook";
        goal.freeze_pos_on_budget = ctx.robot.hd_r_pos_W;
        goal.has_freeze_pos = true;
        StateCommand cmd = buildHandCommand(goal, force, gc.force_budget_n);

        // Weld gate: caging_ready (forearm legally under the bar) AND force in
        // budget, stable for under_contact_stable_n ticks.
        const bool in_budget = force > 0.0 && force < gc.hard_abort_n;
        if (!weld_requested_ && ctx.robot.t5_caging_ready && in_budget) {
            if (++stable_count_ >= gc.under_contact_stable_n) weld_requested_ = true;
        } else if (!weld_requested_) {
            stable_count_ = 0;
        }

        const double fe_z = readEnvDouble("OPENLOONG_FSM_CROUCH_FE_Z",
                                          ctx.cfg.crouch.fe_z_des);
        configureLegIkCrouchCommand(cmd, fe_z, goal.reason);
        cmd.weld_request_valid = weld_requested_;
        cmd.weld_active = weld_requested_;
        return cmd;
    }

    bool done(const Ctx &ctx) const override {
        if (!weld_requested_ || !lift_started_) return false;
        return (ctx.robot.hd_r_pos_W.z() - lift_base_.z())
                   >= ctx.cfg.grasp.verify_lift_z - 1e-3;
    }

private:
    int stable_count_ = 0;
    bool weld_requested_ = false;
    bool lift_started_ = false;
    bool seek_valid_ = false;
    Eigen::Vector3d lift_base_ = Eigen::Vector3d::Zero();
};

}  // namespace T5M1
```
*(Note: `clamp01` is defined in `t5_phases_approach_under.h`; if Grasp is compiled without it, add the same inline helper here or move it to `t5_fsm_helpers.h`. Simplest: move `inline double clamp01(...)` into `t5_fsm_helpers.h` in Task 4 Step 2 and include it here.)*

- [ ] **Step 3: Build + run**:
```bash
WT=/mnt/c/Users/Administrator/.config/superpowers/worktrees/OpenLoong-Dyn-Control/t5-m3-source
wsl.exe bash -lc "docker run --rm -v $WT:/work openloong-ubuntu22-build:local bash -lc 'cd /work && g++-11 -std=c++17 -I. -Icommon -Ithird_party/eigen3 tests/t5_grasp_phase_test.cpp demo/t5_pick_place_fsm.cpp -o /tmp/gt -lpthread 2>&1 | tail -10 && /tmp/gt'"
```
Expected: `t5_grasp_phase_test passed`.

- [ ] **Step 4: Build the full demo + all unit tests** (integration check — all phases compile in the demo TU):
```bash
wsl.exe bash -lc "docker run --rm -v $WT:/work openloong-ubuntu22-build:local bash -lc 'cd /work && cmake --build build --target t5_config_test t5_approach_under_phase_test t5_grasp_phase_test t5_crouch_phase_test t5_scheduler_test t5_locomotion_phase_test walk_mpc_wbc_t5_pick_place -j\$(nproc) 2>&1 | tail -4 && cd build && ./t5_config_test && ./t5_approach_under_phase_test && ./t5_grasp_phase_test && ./t5_crouch_phase_test && ./t5_scheduler_test && ./t5_locomotion_phase_test'"
```
Expected: each `... passed`; demo links clean.

- [ ] **Step 5: Commit**
```bash
git -C "$WT" add demo/t5_phases_grasp.h demo/t5_fsm_helpers.h
git -C "$WT" add -f tests/t5_grasp_phase_test.cpp
git -C "$WT" commit -m "feat(t5-rewrite): GraspPhase hook-lift + caging-gated weld + verify-lift"
```

---

### Task 6: Sim gate — under-handle grasp on the new FSM

**Requires WSL2+Docker.** Config `T5_pick_place_newfsm_m1a.env` (NEW_FSM=1; reset to stable crouch baseline). The engine now forwards the extended `-e` allowlist. Runner: `run_t5_pick_place_visual_wslg.sh --config <env> --source-repo <worktree> --smoke-seconds 90`.

- [ ] **Step 1:** Confirm the config crouch knobs are at the stable baseline (FE_Z=-0.83, BASE_Z=0.90, RETRACT_ARM=0, CONTACT_ABORT_TICKS=1) and `OPENLOONG_T5_ELBOW_OUT_BIAS=0.25` is present (it is — needed for the IK-seed elbow-out during ApproachUnder/Grasp). Run the sim.

- [ ] **Step 2: Acceptance (M1b-v2):** from the runtime log:
  - **Crouch: no `crouch_top_contact`** — `t5_basket_handle_top_non_under_contacts == 0` throughout the crouch (G1 forearm clear). Phase reaches `approach_hover → approach_descend → approach_insert → grasp_hook`.
  - No `side_contact`/`top_contact` during ApproachUnder; the hand reaches the under-bar contact point.
  - **`t5_caging_ready` becomes true** (forearm legally under: `t5_caging_r07_below_handle` && `t5_caging_no_illegal_contact` && `t5_caging_axis_within_segment` && `t5_caging_lateral_aligned`).
  - Grasp peak force `< hard_abort_n` (80 N); no `grasp_force_abort`; basket not flicked (`t5_handle_canonical_valid` stays true).
  - `weld_active=1` after caging_ready; verify-lift ~`verify_lift_z` and the basket follows.
  - Base stays crouched-stable (base_z ≈ 0.90, |roll|,|pitch| small).

- [ ] **Step 3: Record** to `outputs/t5_m1b_v2_grasp_<date>/` (GATE_SUMMARY.md + runtime.log + caging/pose timeline), mirroring `outputs/t5_m1a_walkcrouch_20260713/`.

- [ ] **Step 4:** If the forearm still grazes the bar top during ApproachUnder (caging never `r07_below_handle`), escalate to **Route B** (r07 forearm surface-proxy relative targeting, spec §1 G3 / §2): command the pose so the forearm surface point lands on the under-bar target (`supportPoseToHandPose`, `t5_support_pose_planner.h:110-130`) instead of the hand origin. Do NOT tune blindly — compare against the guarded path in `t5_pick_place_fsm.cpp` (grep `guardedSupportCommand`).

---

## Self-Review

- **Spec coverage:** §1 G1 posture → Task 1 (fixed posture) + Task 2 (crouch reg). §1 G2 path → Task 3 (waypoints) + Task 4 (ApproachUnder 3-segment) + Task 5 (hook-lift). §4 caging weld gate → Task 5 (`t5_caging_ready`). §3.0 drop v1 experiments → Task 2 Steps 2-3. §5 config → Task 1. §7 tests → Tasks 1,2,4,5; sim gate → Task 6. §8 Route B fallback → Task 6 Step 4. **Deferred (correct):** M1c payload stand-up; Route B full port (only if Route A's forearm grazes).
- **Placeholder scan:** No TBD/TODO. All offsets are the old FSM's proven values (spec §1). Verify-before-code notes (axis sign in Task 3; `clamp01` location in Task 5) are checks with a concrete resolution, not placeholders.
- **Type consistency:** `t5FixedRightArmPostureQ()` (Task 1) used in Tasks 2,5. `ApproachCfg.{hover_s,descend_s,insert_s,pre_grasp_x,above_z,under_z,insert_x_offset,axis_contact_offset,axis_pre_offset}` and `GraspCfg.{hook_lift_z,hook_s,force_budget_n,hard_abort_n,wrist_posture_alpha,verify_lift_z,under_contact_stable_n,pos_err_tol}` (Task 1) used consistently in Tasks 4,5. `HandleGeometry.{graspTargetUnder,axisContactUnder,axisStartUnder,axisLiftContact}` (Task 3) used in Tasks 4,5. `t5_caging_ready` (DataBus) in Task 5. `weldRequested()` on GraspPhase used by its test. `clamp01` shared via `t5_fsm_helpers.h`. `T5M1` namespace throughout.
