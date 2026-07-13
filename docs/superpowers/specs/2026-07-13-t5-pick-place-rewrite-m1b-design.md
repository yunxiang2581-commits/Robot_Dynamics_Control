# T5 Pick-Place 重写 —— M1b 设计:下探 + 抓取/焊住 + 轻提验证

> 父 spec:`docs/superpowers/specs/2026-07-12-t5-pick-place-rewrite-m1-design.md`(整条 pick-place 状态机重写,范围决策 B)。
> 本文是 **M1 的下半**:M1a(走行→下蹲 parity)已完成并提交(`cb319c37`,分支 `codex/t5-rewrite-m1`,sim 闸门绿 + 目视确认)。
> **M1b 到"焊住 + 轻提验证"为止;完整负载前馈起身 = M1c(单独 spec)。**

---

## 1. 目标与范围

**M1b 目标:** 在新 Scheduler 架构下,机器人从稳定蹲姿(base_z≈0.90)**分级下探到篮筐把手下方 → 温柔接触 → 建立 dynamic-weld → 轻提 ~3cm 确认篮子被牵住**,全程不摔、不挑飞轻篮子(0.27 kg)、不冲破力阈。

**终点(M1b sim 闸门):** `Init→…→Crouch→ApproachUnder→Grasp`,Grasp 完成 weld + 轻提验证,机器人保持蹲姿稳定。

**非目标(留 M1c):** 负载前馈完整起身、站直平衡、搬运/放置(M2)。不改抓取机理(保留 rigid dynamic-weld + 现有 `scene_basket.xml`,仅软化 weld `solref`);不动 WBC/MPC 内核;不做真实夹爪。

## 2. 关键决策(brainstorm 2026-07-13)

1. **范围拆分:** M1b = ApproachUnder + Grasp/weld + 轻提验证;M1c = 负载前馈完整起身。理由:抓取翻车与起身翻车是两个独立物理难点,sim 调试昂贵(~5 轮/难点),风险隔离、调试聚焦。
2. **分阶段腕约束(`ConstraintMode`,一处可切):** ApproachUnder = **`Pose6HandleFrame`**(远离把手,锁定 Crouch 捕获的入场朝向 —— M1a 已证 pose6 在下蹲稳定、无腕漂);Grasp = **`Position3WithWristReg`**(手指轴贴杆时,遵 G1"pose6 会把手指弯 53.7°",用强腕正则堵住视频 `20260712_150543` 里 r06 −0.10→+1.49 的漂移)。这正是父 spec §4 的"远目6、近抓3",现由 M1a 的 pose6 正面证据背书。
3. **接触→焊住 = 慢接触 seek 下降:** Grasp 以小 z-rate 主动缓降,力预算冻结封顶,检到下表面 under-contact 且稳定后建立 weld。比"固定 under 点 + 持位"对几何误差更鲁棒、接触更温柔(记忆:7/6 刚性 weld + 硬顶 → 力 44→137N 冲破 80N → abort/挑飞)。

## 3. 架构(继承 M1a)

`StateCommand`(`demo/t5_pick_place_fsm.h:111`)仍是唯一契约;主循环 `walk_mpc_wbc_t5_pick_place.cpp:2438` 区消费它,不关心生产者。M1b 只在 M1a 已建的 `Scheduler` 相位链 `CrouchPhase` 之后**追加两个 `Phase`**,各相位:
- 只通过唯一的 `HandCommandBuilder` 发手命令(交 `HandGoal{pos,rot,mode,force_budget,posture}`);
- 只通过唯一的 `HandleGeometry`(封装 `CanonicalHandleFrame`)算 waypoint;
- 参数全在 typed `T5Config`;
- `checkAbort()` 非空 → Scheduler 路由到 `SafeHoldPhase`(M1a 已实现的分级 crouch 稳下肢)。

复用 M1a 已实现且已验证的:`SafeHold`(分级 crouch 腿指令连续保持)、`HandCommandBuilder` 力预算冻结(`buildHandCommand(goal, contact_force_n, force_budget_n)`)、`ConstraintMode` 枚举、`HandleGeometry`。

## 4. 相位设计

### 4.1 `ApproachUnderPhase`(约束 = pose6,锁定入场朝向)

- **入场:** 捕获 Crouch 末态的 hand 朝向(或复用 `PhaseState.captured_right_hand_rot_W`),pose6 全程锁定它。
- **分级 waypoint(相对 `HandleGeometry`,cfg 时长):**
  1. `CenterAxis` —— 手对中把手轴(`ApproachCfg.center_axis_s`);
  2. `LowerUnder` —— 降到把手下方目标高度(`lower_under_s`);
  3. `ApproachUnder` —— 沿把手轴推进到 under 接触**前置点**(`approach_under_s`,`under_x_offset`)。
- **`step()`:** `HandGoal{ mode=Pose6HandleFrame, pos=当前段 waypoint, rot=锁定朝向, has_freeze_pos=false }` → `buildHandCommand`;叠加 crouch 腿指令(`configureLegIkCrouchCommand`,保持 base_z≈0.90)。
- **`done()`:** 手到达 under 前置点(位误差入容差)且位姿有效且无侧/顶接触。
- **`checkAbort()`:** 侧/顶接触(`robot_handle_side_contacts>0` 或 `robot_handle_top_non_under_contacts>0`)、位姿无效 → SafeHold(reason `approach_under_side_contact` / `_invalid_pose`)。

### 4.2 `GraspPhase`(约束 = position3 + 强腕正则)—— 慢接触 seek + weld

- **约束切换:** `HandGoal.mode = Position3WithWristReg`,`posture_alpha = GraspCfg.wrist_posture_alpha`(强正则,~0.7),`posture_target` = 抓取姿态(锁住好的腕构型)。
- **慢接触 seek:** 每 tick 手位目标沿 −z(向把手下表面)推进 `GraspCfg.contact_seek_z_rate`(小速率);`buildHandCommand` 用力预算冻结:接触力 ≥ `force_budget_n`(τ_safe=18N)时冻结当前手位(`freeze_pos_on_budget`),不再下压 → 防冲力/挑飞。
- **weld 门控(三条同时,父 spec §5.1-C):**
  (a) **下表面** under-contact:`robot_handle_under_contacts>0`(**非**侧/顶);
  (b) 力在预算内稳定 `under_contact_stable_n` 帧;
  (c) 手位误差入 `pos_err_tol`。
  三条满足 → 建立 dynamic-weld;`weld_solref` 软化到 **0.05**(避免刚性冲击)。否则 abort **不焊**。
- **验证轻提:** weld 建立后,命令小幅 `verify_lift_z`(~0.03m)抬手并持 weld 位姿。
- **`done()`:** weld 已建立 且 轻提到位 且 篮子随手抬起(`hd_r` 与篮子把手相对位移锁定在容差内)。
- **`checkAbort()`:** 力冲破硬 abort `hard_abort_n`(80N)、weld 后失接触、顶/侧接触 → SafeHold(reason `grasp_force_abort` / `grasp_lost_contact` / `grasp_top_contact`)。

### 4.3 复用 `SafeHold`(M1a 已实现)

任何 abort → Scheduler 切 `SafeHoldPhase`:分级 crouch 腿指令稳下肢(定住脚、base 零命令、持手/weld),让机器人沉降稳住而非任其翻。M1a 已验证其 crouch→SafeHold 连续性。

## 5. Config 增量

`T5Config` 已有 `GraspCfg`/`LiftCfg`/`ApproachCfg` 骨架(M1a 建)。M1b 填充/新增:

```cpp
struct GraspCfg {
    double force_budget_n = 18.0;        // τ_safe:接触即冻结(M1a 已有)
    double hard_abort_n   = 80.0;        // 硬 abort(M1a 已有)
    double wrist_posture_alpha = 0.7;    // 强腕正则(M1a 已有)
    double weld_solref    = 0.05;        // 软化(旧 0.005);M1b 用
    double contact_seek_z_rate = 0.004;  // 慢接触 seek 速率(新)
    int    under_contact_stable_n = 20;  // weld 门:under 稳定帧数(新)
    double pos_err_tol    = 0.03;        // weld 门:手位误差容差(新)
    double verify_lift_z  = 0.03;        // 轻提验证高度(新)
};
```
ApproachUnder 分级时长复用 `ApproachCfg{ center_axis_s, lower_under_s, approach_under_s, under_x_offset }`(M1a 已有)。

> **具体数字为 M1b 起点,sim 里调优;单测只钉不变式**(budget>0、budget<hard_abort、solref>0、verify_lift_z>0)。

## 6. 测试

**单测(每 Phase 隔离测 step/done/abort,方案 2 红利):**
- `ApproachUnderPhase`:顶/侧接触 → abort=SafeHold;waypoint 分级几何(CenterAxis→LowerUnder→ApproachUnder 相对 HandleGeometry)正确。
- `GraspPhase`:力 ≥ τ_safe → 位置冻结(复用 M1a `HandCommandBuilder` 测的冻结);顶接触 → 不 weld;下表面 under 稳定 N 帧 + 力入预算 + 位误差入容差 → weld 请求;力冲破 hard_abort → abort。
- Config:GraspCfg 不变式。

**M1b sim 闸门(跑到 Grasp done,断言):**
1. crouch→分级下探到把手下方,**全程无侧/顶接触**;
2. Grasp 峰值接触力 **< hard_abort**(≤ τ_safe + 余量),无 `grasp_force_abort`,篮子不挑飞;
3. weld 建立于**下表面 under-contact**(非侧/顶);
4. 轻提 ~`verify_lift_z` 篮子随手抬起(相对位移锁定在容差内);
5. 全程机器人保持**蹲姿稳定**(base_z≈0.90,|roll/pitch| 小),不摔。

## 7. 文件落点

- 新增 `demo/t5_phases_approach_under.h` —— `ApproachUnderPhase`。
- 新增 `demo/t5_phases_grasp.h` —— `GraspPhase`(慢接触 seek + weld 门控 + 轻提)。
- 改 `demo/t5_config.h` —— 填充 `GraspCfg` 新字段。
- 改 `demo/t5_scheduler.h` 或接线处(`walk_mpc_wbc_t5_pick_place.cpp` 相位列表)—— 在 `CrouchPhase` 后 append `ApproachUnderPhase`、`GraspPhase`。
- 新增测试 `tests/t5_approach_under_phase_test.cpp`、`tests/t5_grasp_phase_test.cpp`;`CMakeLists.txt` 注册(`tests/` gitignore → `git add -f`)。
- weld 建立/门控复用下游既有 `StateCommand` 字段与主循环 weld 逻辑(M1a 契约不变);仅 `scene_basket.xml` 的 `solref` 由 cfg 软化值驱动(若走 env/cfg 而非改场景文件,优先 cfg)。

## 8. 已知风险(如实标注)

- **挑飞 / 力冲破:** 0.27 kg 轻篮子 + rigid weld,硬顶会冲破 80N 或把篮子弹开。缓解:慢接触 seek(小 z-rate)+ 力预算冻结(τ_safe=18N ≪ 80N)+ solref 软化。**这是 M1b 主风险,sim 里重点观测接触力曲线。**
- **weld 门控误判:** under vs 侧/顶接触分类依赖 `robot_handle_*_contacts`(父 spec 里 G2 `c33bef7e` 已把 grasp 门控接到 robot-handle-only 接触,非 floor/platform 污染)。M1b 复用该干净分类。
- **决定性负载杠杆(`OPENLOONG_T5_PAYLOAD_COMP` 前馈、weld `solref`)** 之前 10 次 sweep 从未动过 —— M1b 动 solref,payload 前馈留 M1c。

## 9. 交接

**实现:** 转 `superpowers:writing-plans` 起草 M1b 实现计划(逐任务:ApproachUnder → Grasp → CMake/接线 → sim 闸门)。
