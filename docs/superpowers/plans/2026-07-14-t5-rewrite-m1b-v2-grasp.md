# T5 Rewrite — Plan M1b-v2: Under-Handle Grasp (G2-only axis_lift trajectory)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> Design spec: `docs/superpowers/specs/2026-07-14-t5-pick-place-rewrite-m1b-v2-design.md` (G2-only). Supersedes the ApproachUnder/Grasp/Crouch parts of the v1 plan (`2026-07-13-t5-rewrite-m1b-grasp.md`). M1a is done+committed (`cb319c37`); M1b v1 phases exist and are wired (WIP `e6d5fc3c`) — this plan REWRITES those three phases (no reuse of v1 phase bodies).

**Goal:** Get the right hand UNDER the basket handle and grasp it — with a 3-segment axis_lift trajectory (hover → descend-behind → insert-under-along-axis, then hook-lift), never a top-down drop. The forearm is kept off the bar top by the main-loop elbow-out IK-seed bias (`OPENLOONG_T5_ELBOW_OUT_BIAS=0.25`), NOT by an in-phase fixed posture. Weld only when the caging gate confirms the forearm is legally under the bar; verify with a small lift.

**Architecture:** `StateCommand` stays the sole contract. G2-only Route A (analytic axis_lift). Three REWRITTEN `T5M1` phases behind `OPENLOONG_T5_NEW_FSM`, keeping the M1a kernel (Scheduler, Phase, HandCommandBuilder, HandleGeometry, SafeHold, ConstraintMode, main-loop WBC/MPC/IK/weld/DataBus): `CrouchPhase` (revert v1 retract/tolerance experiments → clean M1a form + high-and-back hover), `ApproachUnderPhase` (3-segment axis waypoints via `HandleGeometry`), `GraspPhase` (hook-lift + `t5_caging_ready`-gated weld + verify-lift). All offsets are the old FSM's proven axis_lift values. **No `t5FixedRightArmPostureQ` / G1 fixed posture.**

**Tech Stack:** C++17, Eigen, standalone `int main()` assert tests, WSL2+Docker (`openloong-ubuntu22-build:local`). Branch `codex/t5-rewrite-m1`, worktree `C:/Users/Administrator/.config/superpowers/worktrees/OpenLoong-Dyn-Control/t5-m3-source`. `tests/` gitignored → `git add -f`.

---

## Confirmed interfaces (verified in-tree, use these exact names)

- DataBus (`common/data_bus.h`): `t5_caging_ready` (:86), `t5_caging_no_illegal_contact` (:120), `t5_caging_r07_below_handle` (:122), `t5_caging_axis_within_segment` (:121), `t5_caging_lateral_aligned` (:123), `t5_caging_top_handle_contact` (:119), `t5_basket_handle_side_contacts`, `t5_basket_handle_top_non_under_contacts`, `t5_basket_handle_under_contacts`, `t5_basket_handle_force_norm_max`, `t5_handle_canonical_valid`, `t5_handle_canonical_pos_W`, `t5_handle_frame_y_W`, `t5_handle_frame_z_W`, `base_pos`, `hd_r_pos_W`, `hd_r_rot_W`, `q`.
- `StateCommand` (`demo/t5_pick_place_fsm.h`): `weld_request_valid`/`weld_active` (:128-129); `hand_task_mode`, `hand_des_valid`, `hand_pos_des_W`, `hand_rot_des_W`, `use_leg_ik_crouch`; `right_arm_posture_{target,alpha,target_valid}` (:142-146, used only by GraspPhase's wrist reg via position3, NOT by crouch).
- `HandGoal`/`buildHandCommand` (`demo/t5_hand_command_builder.h`) — **posture reg fields apply ONLY when `mode==Position3WithWristReg`** (builder :48). `HandleGeometry::fromRobot`/`frame.{handle_center_W,x_H_W,y_H_W,z_H_W}` (`demo/t5_handle_geometry.h`), `configureLegIkCrouchCommand`/`readEnvDouble`/`palmUpRotationW`/`crouchHoverTargetW` (`demo/t5_fsm_helpers.h`), `ConstraintMode::{Pose6HandleFrame,Position3WithWristReg}`.

---

### Task 1: config v2 fields (ApproachCfg axis_lift + GraspCfg hook) — G2-only, no posture helper

**Files:** Modify `demo/t5_config.h`; Test `tests/t5_config_test.cpp`.

- [ ] **Step 1: Add failing assertions** to `tests/t5_config_test.cpp` before the final `std::cout`:

```cpp
    // M1b-v2 (G2-only): axis_lift waypoint + hook invariants.
    if (!(fast.approach.hover_s > 0.0) || !(fast.approach.descend_s > 0.0) ||
        !(fast.approach.insert_s > 0.0) ||
        !(fast.approach.axis_contact_offset > 0.0) ||
        !(fast.approach.axis_pre_offset > 0.0) ||
        !(fast.grasp.hook_lift_z > 0.0) || !(fast.grasp.hook_s > 0.0)) {
        std::cerr << "m1b-v2 axis/hook cfg must be positive\n";
        return 1;
    }
```

- [ ] **Step 2: Run test to verify it fails**

```bash
WT=/mnt/c/Users/Administrator/.config/superpowers/worktrees/OpenLoong-Dyn-Control/t5-m3-source
wsl.exe bash -lc "docker run --rm -v $WT:/work openloong-ubuntu22-build:local bash -lc 'cd /work && g++-11 -std=c++17 -I. -Icommon -Ithird_party/eigen3 tests/t5_config_test.cpp -o /tmp/ct 2>&1 | tail -5'"
```
Expected: FAIL — compile error `no member named 'hover_s' in 'T5M1::ApproachCfg'` (and hook_s/hook_lift_z).

- [ ] **Step 3: Replace `ApproachCfg` and extend `GraspCfg`** in `demo/t5_config.h`. Replace the entire existing `struct ApproachCfg { ... };` (the v1 center_axis_s/lower_under_s/under_x_offset/under_z/insert_start_axis/insert_contact_axis block) with:

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
And add to `GraspCfg` (keep the existing v1 fields force_budget_n/hard_abort_n/wrist_posture_alpha/weld_solref/contact_seek_z_rate/under_contact_stable_n/pos_err_tol/verify_lift_z):
```cpp
    double hook_lift_z = 0.03;       // hook up under the bar (GRASP_HOOK_LIFT_Z)
    double hook_s = 0.50;            // hook-lift segment duration
```

- [ ] **Step 4: Run test to verify it passes**
```bash
wsl.exe bash -lc "docker run --rm -v $WT:/work openloong-ubuntu22-build:local bash -lc 'cd /work && cmake --build build --target t5_config_test -j\$(nproc) 2>&1 | tail -3 && ./build/t5_config_test'"
```
Expected: `t5_config_test passed`.

- [ ] **Step 5: Commit**
```bash
git -C "$WT" add demo/t5_config.h
git -C "$WT" add -f tests/t5_config_test.cpp
git -C "$WT" commit -m "feat(t5-rewrite): M1b-v2 axis_lift/hook config (G2-only)"
```

---

### Task 2: `CrouchPhase` — revert v1 experiments → clean M1a form + high-and-back hover (no G1)

**Files:** Modify `demo/t5_phases_crouch.h`, `demo/t5_fsm_helpers.h`; Test `tests/t5_crouch_phase_test.cpp`.

- [ ] **Step 1: Update the crouch test** `tests/t5_crouch_phase_test.cpp`. The current test has cases (a) sustained side contact, (a2) benign under contact, (b) pose6 hover. For G2-only, restore IMMEDIATE side/top abort and drop the sustained-tick expectation. Overwrite case (a) and (a2):

```cpp
    // (a) side contact during crouch -> immediate abort (G2: no forearm graze
    // expected once elbow-out seed is active, so the simple abort is correct).
    {
        DataBus r = crouchRobot(0.95, /*side=*/1, /*top=*/0);
        Ctx ctx{r, st, tgt, cfg, 3.2};
        CrouchPhase ph; ph.onEnter(ctx);
        ph.step(ctx);   // step first (any tick counters removed, but keep call order)
        auto ab = ph.checkAbort(ctx);
        if (!ab || ab->reason != std::string("crouch_side_contact")) {
            std::cerr << "side contact must abort crouch immediately\n"; return 1;
        }
    }
    // (a2) top-non-under contact -> immediate abort
    {
        DataBus r = crouchRobot(0.95, /*side=*/0, /*top=*/1);
        Ctx ctx{r, st, tgt, cfg, 3.2};
        CrouchPhase ph; ph.onEnter(ctx);
        ph.step(ctx);
        auto ab = ph.checkAbort(ctx);
        if (!ab || ab->reason != std::string("crouch_top_contact")) {
            std::cerr << "top contact must abort crouch immediately\n"; return 1;
        }
    }
```
Keep case (b) (pose6 hover mode 2 + crouch leg IK) as-is.

- [ ] **Step 2: Run test to verify it fails** (current code requires sustained ticks, so immediate-abort assertion fails):
```bash
WT=/mnt/c/Users/Administrator/.config/superpowers/worktrees/OpenLoong-Dyn-Control/t5-m3-source
wsl.exe bash -lc "docker run --rm -v $WT:/work openloong-ubuntu22-build:local bash -lc 'cd /work && g++-11 -std=c++17 -I. -Icommon -Ithird_party/eigen3 tests/t5_crouch_phase_test.cpp demo/t5_pick_place_fsm.cpp -o /tmp/cct -lpthread 2>&1 | tail -5 && /tmp/cct'"
```
Expected: FAIL (`side contact must abort crouch immediately` — the sustained-tick gate needs abort_ticks reached).

- [ ] **Step 3: Rewrite `demo/t5_phases_crouch.h`** to the clean M1a form (remove all v1 experiments). Replace the whole class body with:

```cpp
class CrouchPhase : public Phase {
public:
    const char *name() const override { return "Crouch"; }

    void onEnter(Ctx &ctx) override {
        ctx.state.entry_time = ctx.simTime;
        ctx.state.entry_time_valid = true;
        ctx.state.settle_started = false;
        if (ctx.robot.hd_r_rot_W.allFinite()) {
            ctx.state.captured_right_hand_rot_W = ctx.robot.hd_r_rot_W;
            ctx.state.captured_right_hand_rot_valid = true;
        }
    }

    // G2-only: abort immediately on an illegal side/top handle contact. With the
    // elbow-out IK seed the forearm no longer grazes the bar top, so the M1a
    // immediate-abort rule is correct (the v1 sustained-tick tolerance and the
    // retract experiment were empirically ruled out — see spec 2026-07-14 §3.0).
    std::optional<Abort> checkAbort(const Ctx &ctx) const override {
        if (ctx.robot.t5_basket_handle_side_contacts > 0)
            return Abort{"crouch_side_contact"};
        if (ctx.robot.t5_basket_handle_top_non_under_contacts > 0)
            return Abort{"crouch_top_contact"};
        return std::nullopt;
    }

    StateCommand step(Ctx &ctx) override {
        // Legs-first crouch: only command the pose6 hand hover once the base has
        // descended into the hover window; during the descent, legs-only (no hand
        // Cartesian task) — a forward hand task here destabilizes the descent
        // (M1a finding). The hand is held high-and-back (crouchHoverTargetW) so
        // the forearm stays clear above the handle.
        const double hover_margin =
            readPositiveEnv("OPENLOONG_FSM_CROUCH_HOVER_BASE_Z_MARGIN", 0.06);
        const bool in_hover_window =
            ctx.robot.base_pos.z() <= ctx.cfg.crouch.base_z + hover_margin;
        const char *reason = in_hover_window ? "crouch_hover" : "crouch_descend";

        StateCommand cmd;
        if (in_hover_window) {
            HandGoal g;
            g.mode = ConstraintMode::Pose6HandleFrame;
            g.pos_W = crouchHoverTargetW(ctx.target, ctx.robot);
            g.rot_W = ctx.state.captured_right_hand_rot_valid
                          ? ctx.state.captured_right_hand_rot_W
                          : palmUpRotationW();
            g.reason = reason;
            g.has_freeze_pos = false;
            cmd = buildHandCommand(g, 0.0, 0.0);
        } else {
            cmd.enabled = true;
            cmd.motion_state = DataBus::Stand;
            cmd.hand_des_valid = false;
            cmd.hand_task_mode = 0;
            cmd.reason = reason;
        }
        const double fe_z =
            readEnvDouble("OPENLOONG_FSM_CROUCH_FE_Z", ctx.cfg.crouch.fe_z_des);
        configureLegIkCrouchCommand(cmd, fe_z, reason);
        return cmd;
    }

    bool done(const Ctx &ctx) const override {
        const double target_z = ctx.cfg.crouch.base_z;
        if (ctx.robot.base_pos.z() > target_z + ctx.cfg.crouch.base_z_settle_tol ||
            !baseVelocityNearZero(ctx.robot)) {
            return false;
        }
        return ctx.state.entry_time_valid &&
               ctx.simTime - ctx.state.entry_time >= ctx.cfg.crouch.settle_time_s;
    }
};
```
(This removes the `side_contact_ticks_`/`top_contact_ticks_` members, the `OPENLOONG_FSM_CROUCH_RETRACT_ARM` branch, and the `OPENLOONG_FSM_CROUCH_CONTACT_ABORT_TICKS` gate.)

- [ ] **Step 4: Revert `crouchHoverTargetW`** in `demo/t5_fsm_helpers.h` to the simple pre-grasp hover if v1 changed it. Ensure the body is:
```cpp
inline Eigen::Vector3d crouchHoverTargetW(const PickPlaceTarget &target,
                                          const DataBus & /*robotState*/)
{
    Eigen::Vector3d hand_target = handleCenterW(target);
    const double pre_grasp_x_offset =
        readEnvDouble("OPENLOONG_FSM_PRE_GRASP_X_OFFSET", 0.20);
    const double pre_grasp_above_z =
        std::max(0.0, readEnvDouble("OPENLOONG_FSM_PRE_GRASP_ABOVE_Z", 0.04));
    hand_target.x() -= pre_grasp_x_offset;
    hand_target.z() += pre_grasp_above_z;
    return hand_target;
}
```
(If it is already this, no change — verify before editing.)

- [ ] **Step 5: Run test to verify it passes**
```bash
wsl.exe bash -lc "docker run --rm -v $WT:/work openloong-ubuntu22-build:local bash -lc 'cd /work && cmake --build build --target t5_crouch_phase_test t5_scheduler_test -j\$(nproc) 2>&1 | tail -3 && ./build/t5_crouch_phase_test && ./build/t5_scheduler_test'"
```
Expected: both `... passed`.

- [ ] **Step 6: Commit**
```bash
git -C "$WT" add demo/t5_phases_crouch.h demo/t5_fsm_helpers.h
git -C "$WT" add -f tests/t5_crouch_phase_test.cpp
git -C "$WT" commit -m "feat(t5-rewrite): M1b-v2 crouch clean M1a form (drop v1 retract/tolerance; G2-only)"
```

---

### Task 3: `HandleGeometry` — axis_lift waypoint helpers (G2 trajectory)

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

*(Verify-before-code: sign convention — the old FSM does `y += contact_offset` with `contact_offset = -0.075` (fsm.cpp:452,458) and `y -= pre_offset` with `pre_offset = 0.04` (:469,475). Here `axis_contact_offset`/`axis_pre_offset` are POSITIVE magnitudes subtracted along `+y_H_W`. Confirm `frame.y_H_W` points the same way as the old world `+y` toward the bar root; if the hand ends up on the wrong side of center in sim (Task 6), flip the sign of the `* frame.y_H_W` terms.)*

- [ ] **Step 2: Compile-check** (compiles with the phase tests in Tasks 4-5; standalone check now):
```bash
WT=/mnt/c/Users/Administrator/.config/superpowers/worktrees/OpenLoong-Dyn-Control/t5-m3-source
wsl.exe bash -lc "docker run --rm -v $WT:/work openloong-ubuntu22-build:local bash -lc 'cd /work && printf \"#include \\\"demo/t5_handle_geometry.h\\\"\nint main(){return 0;}\n\" > /tmp/hg.cpp && g++-11 -std=c++17 -I. -Icommon -Ithird_party/eigen3 /tmp/hg.cpp demo/t5_pick_place_fsm.cpp -o /tmp/hg -lpthread 2>&1 | tail -5 && echo COMPILE_OK'"
```
Expected: `COMPILE_OK`.

- [ ] **Step 3: Commit**
```bash
git -C "$WT" add demo/t5_handle_geometry.h
git -C "$WT" commit -m "feat(t5-rewrite): HandleGeometry axis_lift under-bar waypoints (G2)"
```

---

### Task 4: `ApproachUnderPhase` — REWRITE to axis_lift 3-segment (hover → descend-behind → insert-under)

**Files:** Modify `demo/t5_phases_approach_under.h`, `demo/t5_fsm_helpers.h` (add `clamp01`); Test `tests/t5_approach_under_phase_test.cpp`.

- [ ] **Step 1: Add `clamp01` to `demo/t5_fsm_helpers.h`** (shared by ApproachUnder + Grasp) inside `namespace T5M1`, near the other inline helpers:
```cpp
inline double clamp01(double v) { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); }
```

- [ ] **Step 2: Overwrite the test** `tests/t5_approach_under_phase_test.cpp`:

```cpp
#include "../demo/t5_phases_approach_under.h"

#include <iostream>
#include <string>

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

    // (a) side contact aborts
    {
        DataBus r = underRobot(1, 0);
        Ctx ctx{r, st, tgt, cfg, 5.0};
        ApproachUnderPhase ph; ph.onEnter(ctx);
        auto ab = ph.checkAbort(ctx);
        if (!ab || ab->reason.find("side") == std::string::npos) {
            std::cerr << "side contact must abort approach\n"; return 1;
        }
    }
    // (b) pose6, crouch held; DESCEND target is below the bar centerline (0.68)
    {
        DataBus r = underRobot(0, 0);
        Ctx ctx{r, st, tgt, cfg, 5.0};
        ApproachUnderPhase ph; ph.onEnter(ctx);
        ctx.simTime = 5.0 + cfg.approach.hover_s + 0.01;  // into descend segment
        StateCommand c = ph.step(ctx);
        if (c.hand_task_mode != 2 || !c.hand_des_valid || !c.use_leg_ik_crouch) {
            std::cerr << "approach must be pose6 (mode 2) + crouch\n"; return 1;
        }
        if (c.hand_pos_des_W.z() >= 0.68) {
            std::cerr << "descend target must be below the bar\n"; return 1;
        }
    }
    // (c) insert target is along the axis from descend (same z, differs in y)
    {
        DataBus r = underRobot(0, 0);
        Ctx ctx{r, st, tgt, cfg, 5.0};
        ApproachUnderPhase ph; ph.onEnter(ctx);
        ctx.simTime = 5.0 + cfg.approach.hover_s + cfg.approach.descend_s +
                      cfg.approach.insert_s + 0.01;  // end of insert
        StateCommand c = ph.step(ctx);
        // insert point sits below bar and near handle center in x
        if (c.hand_pos_des_W.z() >= 0.68) {
            std::cerr << "insert target must be below the bar\n"; return 1;
        }
    }
    std::cout << "t5_approach_under_phase_test passed\n";
    return 0;
}
```

- [ ] **Step 3: Run test to verify it fails**
```bash
WT=/mnt/c/Users/Administrator/.config/superpowers/worktrees/OpenLoong-Dyn-Control/t5-m3-source
wsl.exe bash -lc "docker run --rm -v $WT:/work openloong-ubuntu22-build:local bash -lc 'cd /work && g++-11 -std=c++17 -I. -Icommon -Ithird_party/eigen3 tests/t5_approach_under_phase_test.cpp demo/t5_pick_place_fsm.cpp -o /tmp/aut -lpthread 2>&1 | tail -8 && /tmp/aut'"
```
Expected: FAIL — v1 `step()` uses `axisUnderPoint`/`center_axis_s` (removed cfg fields) → compile error, or wrong z if it compiles.

- [ ] **Step 4: Overwrite `demo/t5_phases_approach_under.h` `step()`/`done()`** with the 3-segment axis_lift (keep `onEnter` + `checkAbort` — they already lock orientation and abort on side/top). Replace the v1 `step()` and `done()` bodies with:

```cpp
    StateCommand step(Ctx &ctx) override {
        HandleGeometry g = HandleGeometry::fromRobot(ctx.robot);
        const auto &a = ctx.cfg.approach;
        const double t = ctx.state.entry_time_valid
                             ? ctx.simTime - ctx.state.entry_time : 0.0;
        // axis_lift: hover (reach from crouch) -> descend BEHIND the bar (under
        // depth, retracted along the axis) -> insert UNDER along the axis. Never
        // a top-down drop. Forearm clearance comes from the elbow-out IK seed.
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

- [ ] **Step 5: Run test to verify it passes**
```bash
wsl.exe bash -lc "docker run --rm -v $WT:/work openloong-ubuntu22-build:local bash -lc 'cd /work && g++-11 -std=c++17 -I. -Icommon -Ithird_party/eigen3 tests/t5_approach_under_phase_test.cpp demo/t5_pick_place_fsm.cpp -o /tmp/aut -lpthread 2>&1 | tail -8 && /tmp/aut'"
```
Expected: `t5_approach_under_phase_test passed`.

- [ ] **Step 6: Commit**
```bash
git -C "$WT" add demo/t5_phases_approach_under.h demo/t5_fsm_helpers.h
git -C "$WT" add -f tests/t5_approach_under_phase_test.cpp
git -C "$WT" commit -m "feat(t5-rewrite): ApproachUnderPhase axis_lift 3-segment (G2, descend-behind -> insert-under)"
```

---

### Task 5: `GraspPhase` — REWRITE: hook-lift + caging-gated weld + verify-lift (no G1 posture)

**Files:** Modify `demo/t5_phases_grasp.h`; Test `tests/t5_grasp_phase_test.cpp`.

- [ ] **Step 1: Overwrite the test** `tests/t5_grasp_phase_test.cpp`:

```cpp
#include "../demo/t5_phases_grasp.h"

#include <iostream>
#include <string>

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
    // (b) caging NOT ready -> no weld even with contact force
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

- [ ] **Step 2: Run test to verify it fails**
```bash
WT=/mnt/c/Users/Administrator/.config/superpowers/worktrees/OpenLoong-Dyn-Control/t5-m3-source
wsl.exe bash -lc "docker run --rm -v $WT:/work openloong-ubuntu22-build:local bash -lc 'cd /work && g++-11 -std=c++17 -I. -Icommon -Ithird_party/eigen3 tests/t5_grasp_phase_test.cpp demo/t5_pick_place_fsm.cpp -o /tmp/gt -lpthread 2>&1 | tail -10 && /tmp/gt'"
```
Expected: FAIL — v1 weld gate uses `t5_basket_handle_under_contacts` + `near_handle`, not `t5_caging_ready`; case (b) would weld (caging ignored) → assertion fails.

- [ ] **Step 3: Overwrite `demo/t5_phases_grasp.h`** with the G2 hook-lift + caging-gated weld. Note the wrist reg holds the CAPTURED arm q (`q.segment<7>(14)`) — NOT the G1 fixed posture:

```cpp
#pragma once

// T5 rewrite (M1b-v2) — GraspPhase: hook-lift the hand up under the bar from the
// axis_lift under-contact point, position3 + strong wrist reg (hold the captured
// arm config, avoid finger-axis bend), force-budget freeze so the light basket is
// never over-pressed or flicked. Weld ONLY when the caging gate confirms the
// forearm is legally under the bar (t5_caging_ready) with in-budget force stable
// N ticks. After welding, a small verify-lift confirms the basket is caught.

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
        // Hold the good wrist/arm config via strong reg. Right-arm 7-DoF slice is
        // q.segment<7>(14) (same slice the old FSM uses, fsm.cpp:1099). This is
        // the CAPTURED config, not a G1 fixed posture.
        if (ctx.robot.q.size() >= 21) {
            captured_arm_q_ = ctx.robot.q.segment<7>(14);
            captured_arm_q_valid_ = captured_arm_q_.array().isFinite().all();
        } else {
            captured_arm_q_valid_ = false;
        }
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

        // Hook-lift from the under-contact point up by hook_lift_z; after weld,
        // verify-lift up by verify_lift_z from the latched hand position.
        Eigen::Vector3d contact = g.valid
            ? g.axisContactUnder(a.insert_x_offset, a.under_z, a.axis_contact_offset)
            : ctx.robot.hd_r_pos_W;
        Eigen::Vector3d target;
        if (weld_requested_) {
            if (!lift_started_) { lift_base_ = ctx.robot.hd_r_pos_W; lift_started_ = true; }
            target = lift_base_ + Eigen::Vector3d(0, 0, gc.verify_lift_z);
        } else {
            const double s = clamp01(t / gc.hook_s);
            const Eigen::Vector3d up = g.valid ? gc.hook_lift_z * g.frame.z_H_W
                                               : Eigen::Vector3d(0, 0, gc.hook_lift_z);
            target = contact + s * up;
        }

        HandGoal goal;
        goal.mode = ConstraintMode::Position3WithWristReg;
        goal.pos_W = target;
        goal.rot_W = ctx.robot.hd_r_rot_W;
        goal.posture_target = captured_arm_q_valid_
                                  ? captured_arm_q_
                                  : Eigen::Matrix<double, 7, 1>::Zero();
        goal.posture_valid = captured_arm_q_valid_;
        goal.posture_alpha = gc.wrist_posture_alpha;
        goal.reason = weld_requested_ ? "grasp_verify_lift" : "grasp_hook";
        goal.freeze_pos_on_budget = ctx.robot.hd_r_pos_W;
        goal.has_freeze_pos = true;
        StateCommand cmd = buildHandCommand(goal, force, gc.force_budget_n);

        // Weld gate: caging_ready (forearm legally under the bar, spec §4) AND
        // force in budget, stable for under_contact_stable_n ticks.
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
    Eigen::Vector3d lift_base_ = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 7, 1> captured_arm_q_ =
        Eigen::Matrix<double, 7, 1>::Zero();
    bool captured_arm_q_valid_ = false;
};

}  // namespace T5M1
```

- [ ] **Step 4: Run test to verify it passes**
```bash
wsl.exe bash -lc "docker run --rm -v $WT:/work openloong-ubuntu22-build:local bash -lc 'cd /work && g++-11 -std=c++17 -I. -Icommon -Ithird_party/eigen3 tests/t5_grasp_phase_test.cpp demo/t5_pick_place_fsm.cpp -o /tmp/gt -lpthread 2>&1 | tail -10 && /tmp/gt'"
```
Expected: `t5_grasp_phase_test passed`.

- [ ] **Step 5: Build the full demo + all unit tests** (integration — all phases compile in the demo TU + wiring intact):
```bash
wsl.exe bash -lc "docker run --rm -v $WT:/work openloong-ubuntu22-build:local bash -lc 'cd /work && cmake --build build --target t5_config_test t5_approach_under_phase_test t5_grasp_phase_test t5_crouch_phase_test t5_scheduler_test t5_locomotion_phase_test walk_mpc_wbc_t5_pick_place -j\$(nproc) 2>&1 | tail -4 && cd build && ./t5_config_test && ./t5_approach_under_phase_test && ./t5_grasp_phase_test && ./t5_crouch_phase_test && ./t5_scheduler_test && ./t5_locomotion_phase_test'"
```
Expected: each `... passed`; demo links clean.

- [ ] **Step 6: Commit**
```bash
git -C "$WT" add demo/t5_phases_grasp.h
git -C "$WT" add -f tests/t5_grasp_phase_test.cpp
git -C "$WT" commit -m "feat(t5-rewrite): GraspPhase hook-lift + caging-gated weld + verify-lift (G2-only)"
```

---

### Task 6: Sim gate — under-handle grasp on the new FSM

**Requires WSL2+Docker.** Config `T5_pick_place_newfsm_m1a.env` (NEW_FSM=1; stable crouch baseline). The engine now forwards the extended `-e` allowlist (fixed in a prior turn). Runner: `run_t5_pick_place_visual_wslg.sh --config <env> --source-repo <worktree> --smoke-seconds 90`.

- [ ] **Step 1: Verify-before-code — elbow-out seed active in NEW_FSM path.** This is the load-bearing assumption of G2-only. Confirm `OPENLOONG_T5_ELBOW_OUT_BIAS=0.25` is (a) in the config, (b) forwarded by the engine `-e` allowlist, AND (c) actually applied to the right-arm IK seed when `OPENLOONG_T5_NEW_FSM=1`. Grep the main loop for `applyRightElbowOutBias`/`ELBOW_OUT_BIAS` and confirm it is not gated behind the old FSM path. If it is gated off for NEW_FSM, wire it in before running (the forearm will otherwise press the bar top exactly as v1). Record the finding.

- [ ] **Step 2: Confirm the config crouch knobs** are at the stable baseline (FE_Z=-0.83, BASE_Z=0.90; RETRACT_ARM/CONTACT_ABORT_TICKS absent or 0/1 — they no longer exist in code). Run the sim (smoke-seconds 90).

- [ ] **Step 3: Acceptance (M1b-v2), from the runtime log:**
  - **Crouch: no `crouch_top_contact`** — `t5_basket_handle_top_non_under_contacts == 0` throughout the crouch. Phase progresses `crouch_* → approach_hover → approach_descend → approach_insert → grasp_hook`.
  - No `side_contact`/`top_contact` during ApproachUnder; the hand reaches the under-bar contact point (`approach_insert` completes, phase hands off to Grasp).
  - **`t5_caging_ready` becomes true** (`t5_caging_r07_below_handle` && `t5_caging_no_illegal_contact` && `t5_caging_axis_within_segment` && `t5_caging_lateral_aligned`).
  - Grasp peak force `< hard_abort_n` (80 N); no `grasp_force_abort`; basket not flicked (`t5_handle_canonical_valid` stays true).
  - `weld_active=1` after caging_ready; verify-lift ~`verify_lift_z` and the basket follows (handle canonical z rises with the hand).
  - Base stays crouched-stable (base_z ≈ 0.90, |roll|,|pitch| small); no collapse.

- [ ] **Step 4: Record** to `outputs/t5_m1b_v2_grasp_<date>/` (GATE_SUMMARY.md + runtime.log + caging/pose timeline), mirroring `outputs/t5_m1a_walkcrouch_20260713/`.

- [ ] **Step 5: Escalation.** If the forearm still grazes the bar top during ApproachUnder (`t5_caging_r07_below_handle` never true, `crouch`/`approach` top contact persists) even with the elbow-out seed confirmed active, the analytic G2 waypoints are insufficient and the decision is the user's: escalate to **Route B** (r07 forearm surface-proxy relative targeting — command the pose so the forearm surface point lands on the under-bar target, `supportPoseToHandPose` at `t5_support_pose_planner.h:110-130`; compare against the guarded path in `t5_pick_place_fsm.cpp`, grep `guardedSupport`). Do NOT tune blindly. Present findings and stop for a decision.

---

## Self-Review

- **Spec coverage (G2-only):** §3.0 CrouchPhase revert v1 experiments + clean M1a form → Task 2. §3.1 ApproachUnder 3-segment axis_lift → Task 3 (waypoints) + Task 4 (phase). §3.2 GraspPhase hook-lift + force budget → Task 5. §4 caging weld gate (`t5_caging_ready`) → Task 5. §5 config → Task 1. §7 unit tests → Tasks 1,2,4,5; sim gate → Task 6. §8 elbow-out seed verify-before-code → Task 6 Step 1; Route B fallback → Task 6 Step 5. **No G1 fixed posture anywhere** (spec §1 G1 marked 不选). **Deferred (correct):** M1c payload stand-up; Route B full port (only if Route A's forearm grazes).
- **Placeholder scan:** No TBD/TODO. All offsets are the old FSM's proven axis_lift values (spec §1 G2). Verify-before-code notes (axis sign in Task 3; elbow-out seed in Task 6 Step 1) are concrete checks with a defined resolution, not placeholders.
- **Type consistency:** `ApproachCfg.{hover_s,descend_s,insert_s,pre_grasp_x,above_z,under_z,insert_x_offset,axis_contact_offset,axis_pre_offset}` and `GraspCfg.{hook_lift_z,hook_s,force_budget_n,hard_abort_n,wrist_posture_alpha,verify_lift_z,under_contact_stable_n,pos_err_tol}` (Task 1) used consistently in Tasks 4,5. `HandleGeometry.{graspTargetUnder,axisContactUnder,axisStartUnder,axisLiftContact}` (Task 3) used in Tasks 4,5. `t5_caging_ready` (DataBus) in Task 5. `weldRequested()` accessor used by the Grasp test. `clamp01` added to `t5_fsm_helpers.h` (Task 4 Step 1), used by ApproachUnder + Grasp. `q.segment<7>(14)` captured posture (not G1). `T5M1` namespace throughout.
