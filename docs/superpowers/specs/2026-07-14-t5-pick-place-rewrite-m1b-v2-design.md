# T5 Pick-Place 重写 —— M1b-v2 设计:下探抓取(移植旧 FSM caging/axis-lift 几何)

> 父 spec:`docs/superpowers/specs/2026-07-13-t5-pick-place-rewrite-m1b-design.md`(M1b v1)。**本文取代 v1 的 ApproachUnder/Grasp/Crouch-posture 部分**;v1 的范围拆分(M1b 下探+焊住+轻提 / M1c 负载起身)、GraspCfg 骨架、单一 HandCommandBuilder/HandleGeometry 契约仍然有效。
> 起因:v1 sim 闸门卡死——机器人下蹲到 base_z=0.90 时**右前臂持续压在把手顶**(`crouch_top_contact`),ApproachUnder/Grasp 从未运行。v1 的简化 waypoint + 收臂 + 深蹲 + 容忍**全部实测失败或反效果**(收臂使前臂更早压顶;深蹲塌地;容忍无效因接触是持续的)。
> 根因(经只读探查 `t5_pick_place_fsm.cpp` / `walk_mpc_wbc_t5_pick_place.cpp` / `t5_caging_diagnostics.h` / `t5_support_pose_planner.h` 确认):v1 缺了旧 FSM **真正成功抓取**所依赖的两个承重几何。

---

## 1. 根因(两个缺失的承重几何)

旧 FSM 成功把手钩到横杆下方而前臂不压顶,靠两件事(单纯 waypoint 位置改不出来):

**(G1) 清开前臂的臂姿 —— elbow-out + palm-up。** 旧 FSM 全程把右臂保持在 `t5FixedRightArmPosture()`(`algorithm/hand_track_task.cpp:113-120`,7 关节 `[0.202922, -1.324359, -3.045075, 0.134528, -2.989420, -0.118958, 1.056396]`,注释"captured from the visual grasp pose"),再叠加 IK 种子 **elbow-out bias 0.25**(`walk_mpc_wbc_t5_pick_place.cpp:1981-1984`,`OPENLOONG_T5_ELBOW_OUT_BIAS`)+ 腕 **palm-in 0.20**(`palmUpRotationW`,`t5_pick_place_fsm.cpp:342-360`)。结果:掌心朝上、肘部外摆,**前臂旋到侧面而不是横跨杆顶**。v1 的下蹲让臂停在 legacy 前伸姿(前臂横在杆上)→ 压顶。

**(G2) 下探路径 —— 从轴向侧插,绝不从上往下砸。** 旧 FSM 的 `axis_lift` 路径(`GRASP_LOCK`,`t5_pick_place_fsm.cpp:2427-2533`)按时间分三段:①**降到杆下深度、但沿把手轴后撤 `PRE_OFFSET 0.04`**(`graspAxisStartUnderTargetW`:463-477)——竖直下降落在**杆的侧后方**而非杆上;②**沿轴前移 `CONTACT_OFFSET 0.075`** 滑进杆下口袋(`graspAxisContactUnderTargetW`:445-460)——这一步把手送到杆**下方**;③**hook-lift `HOOK_LIFT_Z 0.03`** 顶到杆下沿(`graspAxisLiftContactTargetW`:479-488)。v1 的 ApproachUnder 只做了近似的轴插但**没配 G1 臂姿**,且下蹲阶段前臂已经压顶,根本走不到这一步。

**(可选 G3,更强保证)`guarded` 路径**用 **r07 前臂表面代理**:把目标设成"让前臂网格上最近点落到杆下支撑点"(`supportPoseToHandPose`,`t5_support_pose_planner.h:110-130`:`position_W = support_target_W − rotation_W·r_support_R07`),即**命令的是前臂而非手心**,前臂按构造进到杆下。再用 **caging 判据**校验(见 §4)。比 axis_lift 重,但保证前臂在下。

---

## 2. 目标与范围(v2)

**目标:** 从稳定蹲姿,右臂在 **elbow-out palm-up** 姿下,沿把手轴**侧插到杆下**(descend-behind → translate-under → hook-lift),`caging_ready` 门控确认前臂在杆下且无非法接触后建立 weld,轻提 ~3cm 验证篮子随手。全程不摔、不压顶、不挑飞。

**范围:** ApproachUnder + Grasp/weld + 轻提验证(同 v1 终点)。**非目标:** M1c 负载完整起身;不改抓取机理(仍 rigid dynamic-weld);不动 WBC/MPC/IK 内核(只复用其 elbow-out 种子与 hand IK)。

**两条实现路线(§8 决策点):**
- **路线 A(axis_lift,推荐先做):** 移植三段解析 waypoint + G1 臂姿 + caging 门控。轻、可单测、贴合已建的 T5M1 相位架构。
- **路线 B(guarded 代理,A 若前臂仍擦顶再上):** 额外移植 r07 表面代理相对目标(前臂保证在下)。重,但最鲁棒。

---

## 3. 相位设计(路线 A)

复用 M1a/M1b-v1 已建:Scheduler、Phase、HandCommandBuilder、HandleGeometry、SafeHold、`ConstraintMode`。

### 3.0 `CrouchPhase` 改动(G1 臂姿,替代 v1 的失败收臂)
- **下蹲全程(descend + hover)对右臂施加 posture 正则,目标 = `t5FixedRightArmPosture`(elbow-out palm-up 固定抓取姿)**,alpha 强(~1.0),**关节空间、不发 Cartesian 手任务**(避免 M1a 观测到的下降期 Cartesian 任务塌陷)。这把前臂旋到侧面,下蹲沉降时不横压杆顶。
- 新增 T5M1 helper `t5FixedRightArmPostureQ()` 返回上面 7 关节值(与主循环一致)。
- CrouchPhase.checkAbort 保留 v1 的 side/top 中止(现在前臂清开,不该再有 sustained top);done() 不变(base_z≤0.93 + 静止 + settle)。
- **删除** v1 的 `crouchHoverTargetW` 收高+后撤改动与 `CROUCH_RETRACT_ARM`/`CONTACT_ABORT_TICKS` 实验(已证无效),恢复 hover 目标为杆上后撤的 pre-grasp 悬停(见 3.1 hover)。

### 3.1 `ApproachUnderPhase`(axis_lift 三段,pose6 锁 palm-up)
握手朝向全程 = `palmUpRotationW`(掌心朝上 + palm-in 0.20)。分级 waypoint(相对 `HandleGeometry.frame`,cfg 时长):
1. **Hover**(`approach_hover`):`x = handle.x − PRE_GRASP_X 0.20`,轴向对齐 axis-start,`z = 杆下深度 + ABOVE 0.06`(杆上 6cm 后撤,同旧 preGraspTargetW)。让臂从蹲姿平顺到位。
2. **DescendBehind**(`approach_descend`,dur `descend_s 0.40`):lerp hover→`axisStartUnder`(= 杆下深度、沿轴后撤 `AXIS_PRE_OFFSET 0.04`)。**竖直降到杆下深度,但落在杆侧后**。
3. **InsertUnder**(`approach_insert`,dur `insert_s 0.50`):lerp `axisStartUnder`→`axisContactUnder`(= 沿轴前移 `AXIS_CONTACT_OFFSET 0.075` 到杆下接触点)。**沿轴水平滑进杆下口袋**。
- `step()`:`HandGoal{ mode=Pose6HandleFrame, pos=当前段, rot=palmUpRotationW, has_freeze_pos=false }` → buildHandCommand;叠加 `configureLegIkCrouchCommand`(保持蹲姿);IK 种子的 elbow-out bias 由主循环 `OPENLOONG_T5_ELBOW_OUT_BIAS=0.25` 提供(已在 config)。
- `done()`:到达 `axisContactUnder`(位误差入 `pos_err_tol`)且无 side/top 接触。
- `checkAbort()`:side/top 接触(`t5_basket_handle_side_contacts`/`_top_non_under_contacts`)、位姿无效 → SafeHold。

### 3.2 `GraspPhase`(hook-lift + 力预算 + caging 门控 weld)
- **HookLift**(`grasp_hook`,dur `hook_s 0.50`):从 `axisContactUnder` lerp 到 `axisLiftContact`(= 接触点 +`HOOK_LIFT_Z 0.03` z)。顶到杆下沿。约束 = position3 + 强腕正则(避 G1 弯指),力预算冻结(τ_safe=18N)防冲力/挑飞。
- **weld 门控 = caging_ready(§4)** 且 力在预算内稳定 N 帧 且 手位误差入容差。满足 → dynamic-weld(`solref` 软化 0.05)。否则不焊。
- **验证轻提**:weld 后小幅 `verify_lift_z 0.03` 抬手持 weld 位姿;done() 断言篮子随手(handle canonical z 随手上升 / 相对位移锁定)。
- `checkAbort()`:力冲破硬 abort(80N)、weld 后失接触、非法 side/top 接触 → SafeHold。

---

## 4. caging 门控(weld 前置,移植 `t5_caging_diagnostics.h`)

weld 只在**前臂确实在杆下且无非法接触**时建立。读 DataBus `t5_caging_*`(主循环 `walk_mpc_wbc_t5_pick_place.cpp:933-967` 已发布)或等价字段,`caging_ready` = 全部满足:
- `top_handle_true_under_contact > 0`(掌/前臂在杆**下表面**接触,非顶);
- `no_illegal_contact`(无 side、无 top-non-under);
- `axis_within_segment`:`|axis_t| ≤ 0.085`(前臂投影在杆半长内,没滑出端);
- `r07_below_handle`:`under_gap ≥ 0.004`(前臂表面点沿 `z_H` 在杆下 ≥4mm);
- `lateral_aligned`:`|lateral_error| ≤ 0.015`(前臂横向对齐杆 ≤15mm);
- `palm_aligned`:腕朝向与 `R_H·rotY(palm_in 0.20)` 误差 <10°。

**验证前置(实现前必查):** 确认 DataBus 暴露了 `t5_caging_ready` 或分量字段(`t5_caging_under_gap_m`/`_axis_t_m`/`_lateral_error_m`/`_rotation_error_deg` + `t5_basket_handle_top_true_under_contacts`)。若只在 caging_diagnostics 内部而未上 DataBus,GraspPhase 改读分量 `t5_basket_handle_*` + 自己算 caging(阈值硬编码同上)。

---

## 5. Config 增量(T5Config)

`ApproachCfg` 增(取代 v1 的 under_z/insert_* 近似):
```cpp
struct ApproachCfg {
    double hover_s = 0.5, descend_s = 0.40, insert_s = 0.50;  // 段时长
    double pre_grasp_x = 0.20;      // hover 后撤 x
    double above_z = 0.06;          // hover 在杆下深度之上
    double under_z = 0.02;          // graspTargetW 杆下深度
    double insert_x_offset = 0.08;  // graspTargetW 后撤 x
    double axis_contact_offset = 0.075;  // 沿轴到接触点
    double axis_pre_offset = 0.04;       // 沿轴后撤到 descend-start
};
```
`GraspCfg` 增:`hook_s = 0.50`,`hook_lift_z = 0.03`;caging 阈值硬编码(§4)。臂姿常量 `t5FixedRightArmPostureQ()`。offsets 为旧 FSM 实测值,sim 可微调;单测只钉不变式。

---

## 6. 文件落点

- 新增 `demo/t5_fixed_arm_posture.h` —— `t5FixedRightArmPostureQ()`(7 关节)。
- 改 `demo/t5_phases_crouch.h` —— 下蹲期 reg 到固定臂姿;删 v1 收臂/容忍实验。
- 改 `demo/t5_handle_geometry.h` —— 加 `graspTargetUnder()` / `axisContactUnder()` / `axisStartUnder()` / `axisLiftContact()`(封装 §3.1/§3.2 offsets)。
- 改 `demo/t5_phases_approach_under.h` —— 三段 axis_lift。
- 改 `demo/t5_phases_grasp.h` —— hook-lift + caging 门控 weld + 轻提。
- 改 `demo/t5_config.h` —— ApproachCfg/GraspCfg 增量。
- 改测试 `tests/t5_approach_under_phase_test.cpp`、`t5_grasp_phase_test.cpp`、`t5_crouch_phase_test.cpp`。

---

## 7. 测试

**单测(每相位隔离):**
- Crouch:下蹲指令带固定臂姿 posture reg(right_arm_posture_target = fixed,alpha≥0.9);side/top→abort。
- ApproachUnder:三段 waypoint 几何(hover/descendBehind/insertUnder 相对 HandleGeometry)正确;pose6 palm-up;done 到 insert 点。
- Grasp:hook-lift 目标;力≥τ_safe→冻结;caging_ready 假→不焊,真+稳定→请求 weld;力>硬abort→abort。

**M1b-v2 sim 闸门(跑到 Grasp done):**
1. 下蹲期前臂**不压顶**(crouch 段 `t5_basket_handle_top_non_under_contacts==0`);
2. ApproachUnder 三段无 side/top 接触,手到杆下;
3. Grasp 峰值力 < 硬 abort、无挑飞;
4. `caging_ready` 达成后 weld 建于杆**下表面**;
5. 轻提 ~3cm 篮子随手;
6. 全程蹲姿稳(base_z≈0.90,|roll/pitch| 小)、不摔。

---

## 8. 决策点 & 风险

- **决策:路线 A(axis_lift)先做**;若 sim 显示前臂仍在 InsertUnder 段擦杆顶(手心到位但前臂几何越顶),升级路线 B(r07 表面代理相对目标,前臂保证在下)。
- **风险:** ①固定臂姿在蹲姿下是否可达/稳定(旧 FSM 用它成功过,但新相位序列不同)——下蹲期 posture reg 需 alpha 渐入避免突跳;②caging 字段是否上 DataBus(§4 验证前置);③weld solref 软化 + payload 前馈仍 OFF(payload=M1c)。
- **7 轮 sim 已证:** 简化 waypoint / 收臂 / 深蹲 / 容忍都不行;v2 的赌注是 **G1 臂姿(elbow-out palm-up)**——这是旧 FSM 前臂不压顶的真正原因,v1 完全没有。

---

## 9. 交接

转 `superpowers:writing-plans` 起草 M1b-v2 实现计划(逐任务:固定臂姿 helper → Crouch 臂姿 → HandleGeometry axis waypoints → ApproachUnder 三段 → Grasp hook+caging weld → sim 闸门)。
