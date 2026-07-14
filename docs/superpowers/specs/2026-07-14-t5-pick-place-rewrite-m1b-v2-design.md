# T5 Pick-Place 重写 —— M1b-v2 设计:下探抓取(移植旧 FSM caging/axis-lift 几何)

> 父 spec:`docs/superpowers/specs/2026-07-13-t5-pick-place-rewrite-m1b-design.md`(M1b v1)。**本文取代 v1 的 ApproachUnder/Grasp/Crouch-posture 部分**;v1 的范围拆分(M1b 下探+焊住+轻提 / M1c 负载起身)、GraspCfg 骨架、单一 HandCommandBuilder/HandleGeometry 契约仍然有效。
> 起因:v1 sim 闸门卡死——机器人下蹲到 base_z=0.90 时**右前臂持续压在把手顶**(`crouch_top_contact`),ApproachUnder/Grasp 从未运行。v1 的简化 waypoint + 收臂 + 深蹲 + 容忍**全部实测失败或反效果**(收臂使前臂更早压顶;深蹲塌地;容忍无效因接触是持续的)。
> 根因(经只读探查 `t5_pick_place_fsm.cpp` / `walk_mpc_wbc_t5_pick_place.cpp` / `t5_caging_diagnostics.h` / `t5_support_pose_planner.h` 确认):v1 缺了旧 FSM **真正成功抓取**所依赖的两个承重几何。

---

## 1. 根因 & v2 路径(选定 G2 轨迹)

v1 sim 卡死根因:机器人蹲到 base_z=0.90 时右前臂持续压杆顶(`crouch_top_contact`),抓取相位从未运行。经只读探查确认,旧 FSM 成功钩到杆下靠两类机制,**v2 选定 G2 轨迹**:

**(G2,选定)下探路径 —— 从轴向侧插,绝不从上往下砸。** 旧 FSM 的 `axis_lift` 路径(`GRASP_LOCK`,`t5_pick_place_fsm.cpp:2427-2533`)按时间分三段:①**降到杆下深度、但沿把手轴后撤 `PRE_OFFSET 0.04`**(`graspAxisStartUnderTargetW`:463-477)——竖直下降落在**杆的侧后方**而非杆上;②**沿轴前移 `CONTACT_OFFSET 0.075`** 滑进杆下口袋(`graspAxisContactUnderTargetW`:445-460)——这一步把手送到杆**下方**;③**hook-lift `HOOK_LIFT_Z 0.03`** 顶到杆下沿(`graspAxisLiftContactTargetW`:479-488)。轨迹全程手心在杆下高度,沿轴滑入,不从上砸。**前臂 elbow-out 清空靠主循环 IK 种子偏置 `OPENLOONG_T5_ELBOW_OUT_BIAS=0.25`(§3.0 verify-before-code),非相位内固定姿。**

**(G1,不选)固定臂姿 reg。** 旧 FSM 的 `t5FixedRightArmPosture()` 在 axis_lift 路径下 `GRASP_AXIS_POSTURE_ALPHA` 默认 **0.0**(不加权),仅非 axis 路径才用。v2 **不做 G1**——只靠 G2 轨迹 + 现有 elbow-out 种子,与旧 axis_lift 自洽。

**(G3,不选,留作升级)`guarded` r07 前臂表面代理。** 把目标设成"前臂网格最近点落到杆下支撑点"(`supportPoseToHandPose`,`t5_support_pose_planner.h:110-130`),命令前臂而非手心,前臂按构造进杆下。比 axis_lift 重;仅当 G2 sim 显示前臂仍擦顶时再升级(§8)。

---

## 2. 目标与范围(v2)

**目标:** 从稳定蹲姿,沿把手轴**侧插到杆下**(descend-behind → translate-under → hook-lift 三段 G2 轨迹),`caging_ready` 门控确认前臂在杆下且无非法接触后建立 weld,轻提 ~3cm 验证篮子随手。全程不摔、不压顶、不挑飞。前臂 elbow-out 靠主循环 IK 种子偏置(非相位内固定臂姿)。

**范围:** ApproachUnder + Grasp/weld + 轻提验证(同 v1 终点)。**非目标:** M1c 负载完整起身;不改抓取机理(仍 rigid dynamic-weld);不动 WBC/MPC/IK 内核(只复用其 elbow-out 种子与 hand IK)。

**两条实现路线(§8 决策点):**
- **路线 A(axis_lift,推荐先做):** 移植三段解析 waypoint(G2)+ caging 门控,前臂靠主循环 elbow-out 种子。轻、可单测、贴合已建的 T5M1 相位架构。
- **路线 B(guarded 代理,A 若前臂仍擦顶再上):** 额外移植 r07 表面代理相对目标(前臂保证在下)。重,但最鲁棒。

---

## 3. 相位设计(路线 A,**G2-only**)

**范围决策(2026-07-14 更新):只做 G2 轨迹几何,不做 G1 固定臂姿。** 依据:探查报告确认 axis_lift 路径下 `GRASP_AXIS_POSTURE_ALPHA` 默认 **0.0**,即固定臂姿本就不加权;前臂 elbow-out 清空靠的是 **IK 种子偏置** `OPENLOONG_T5_ELBOW_OUT_BIAS=0.25`(主循环层机制,非相位内固定姿)。所以"G2 轨迹 + 现有 elbow-out 种子"是旧 axis_lift 的自洽做法,G1 属另一套(非 axis 路径才用)。

保留 M1a 内核(Scheduler、Phase、HandCommandBuilder、HandleGeometry、SafeHold、`ConstraintMode`、主循环 WBC/MPC/IK/weld/DataBus);**M1b 相位代码(ApproachUnder/Grasp,以及 CrouchPhase 的 v1 实验改动)全部重写,不复用 v1 相位实现**。

### 3.0 `CrouchPhase` 改动(回退 v1 实验 + high-and-back hover)
- **删除** v1 的 `CROUCH_RETRACT_ARM`/`CONTACT_ABORT_TICKS`/收高-后撤实验(已实测无效/反效果);CrouchPhase 回到 M1a 干净形态:legs-only 下降(descend 段不发 Cartesian 手任务,避免 M1a 观测到的下降期塌陷)+ 进入 hover 窗口后 pose6 hover。
- **hover 目标 = high-and-back pre-grasp 悬停**(杆上后撤,见 3.1 Hover):下蹲期把手 park 在远离杆的高后位,配合 elbow-out 种子偏置,让前臂在沉降时不横压杆顶。**去掉 G1**后,不压顶靠这两条(hover 位置 + elbow-out 种子),不靠相位内固定姿。
- checkAbort:side/top 中止(与 ApproachUnder/Grasp 统一);done() 不变(base_z≤0.93 + 静止 + settle_time_s)。
- **实现前必查(verify-before-code):** 确认 `OPENLOONG_T5_ELBOW_OUT_BIAS=0.25` 在 `OPENLOONG_T5_NEW_FSM=1` 路径下**确实生效**(种子偏置施加到右臂 IK)。理由:v1 已证"只靠 hover 位置"不足以清开前臂;若 NEW_FSM 路径未接 elbow-out 种子,须先补接,否则 G2 轨迹仍会压顶。engine `-e` 白名单已含此变量(前轮修复)。

### 3.1 `ApproachUnderPhase`(axis_lift 三段轨迹,pose6 锁朝向)

握手朝向全程锁定(pose6):用 crouch 末态捕获朝向,或 `palmUpRotationW`(掌心朝上 + palm-in 0.20)。**不施加固定臂姿 reg**(G2-only);前臂几何靠 elbow-out 种子。分级 waypoint(相对 `HandleGeometry.frame`,cfg 时长):
1. **Hover**(`approach_hover`):`x = handle.x − PRE_GRASP_X 0.20`,轴向对齐 axis-start,`z = 杆下深度 + ABOVE 0.06`(杆上 6cm 后撤,同旧 preGraspTargetW)。让臂从蹲姿平顺到位。
2. **DescendBehind**(`approach_descend`,dur `descend_s 0.40`):lerp hover→`axisStartUnder`(= 杆下深度、沿轴后撤 `AXIS_PRE_OFFSET 0.04`)。**竖直降到杆下深度,但落在杆侧后**。
3. **InsertUnder**(`approach_insert`,dur `insert_s 0.50`):lerp `axisStartUnder`→`axisContactUnder`(= 沿轴前移 `AXIS_CONTACT_OFFSET 0.075` 到杆下接触点)。**沿轴水平滑进杆下口袋**。
- `step()`:`HandGoal{ mode=Pose6HandleFrame, pos=当前段插值, rot=锁定朝向, has_freeze_pos=false }` → buildHandCommand;叠加 `configureLegIkCrouchCommand`(保持蹲姿);IK 种子的 elbow-out bias 由主循环 `OPENLOONG_T5_ELBOW_OUT_BIAS=0.25` 提供(已在 config)。
- `done()`:到达 `axisContactUnder`(位误差入 `pos_err_tol`)且无 side/top 接触。
- `checkAbort()`:side/top 接触(`t5_basket_handle_side_contacts`/`_top_non_under_contacts`)、位姿无效 → SafeHold。

### 3.2 `GraspPhase`(hook-lift + 力预算 + caging 门控 weld)
- **HookLift**(`grasp_hook`,dur `hook_s 0.50`):从 `axisContactUnder` lerp 到 `axisLiftContact`(= 接触点 +`HOOK_LIFT_Z 0.03` z)。顶到杆下沿。约束 = position3 + 腕正则(锁腕朝向,避弯指),力预算冻结(τ_safe=18N)防冲力/挑飞。
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
`GraspCfg` 增:`hook_s = 0.50`,`hook_lift_z = 0.03`;caging 阈值硬编码(§4)。**无固定臂姿常量(G2-only)。** offsets 为旧 FSM 实测值,sim 可微调;单测只钉不变式。

---

## 6. 文件落点

- 改 `demo/t5_phases_crouch.h` —— **删 v1 收臂/容忍实验**,回退到 M1a 干净形态 + high-and-back hover(G2-only,无固定臂姿 reg)。
- 改 `demo/t5_handle_geometry.h` —— 加 `axisStartUnder()` / `axisContactUnder()` / `axisLiftContact()`(封装 §3.1/§3.2 offsets;**重写,替换 v1 的 `axisUnderPoint`**)。
- 改 `demo/t5_phases_approach_under.h` —— **重写**为三段 axis_lift 轨迹(hover → descendBehind → insertUnder)。
- 改 `demo/t5_phases_grasp.h` —— **重写**为 hook-lift + caging 门控 weld + 轻提。
- 改 `demo/t5_config.h` —— ApproachCfg/GraspCfg 增量(替换 v1 的 axis 近似字段)。
- 改测试 `tests/t5_approach_under_phase_test.cpp`、`t5_grasp_phase_test.cpp`、`t5_crouch_phase_test.cpp`(**重写以匹配新几何**)。

---

## 7. 测试

**单测(每相位隔离):**
- Crouch:descend 段 legs-only(hand_task_mode==0,无 Cartesian 手任务);hover 窗口 pose6 hover 于 high-and-back;side/top→abort;done 需 base_z≤0.93+静止+settle。**无固定臂姿 reg**(G2-only)。
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

- **决策:G2-only 路线 A(axis_lift 三段轨迹 + elbow-out 种子)**;若 sim 显示前臂仍在 InsertUnder 段擦杆顶(手心到位但前臂几何越顶),再评估路线 B(r07 表面代理相对目标,前臂保证在下)。**不做 G1 固定臂姿。**
- **v2 的赌注(去掉 G1 后):** **G2 轴向侧插轨迹**(descend-behind → insert-under-along-axis,绝不从上砸)+ **主循环 elbow-out IK 种子偏置 0.25**。这两条正是旧 axis_lift 路径前臂不压顶的机制(该路径 posture alpha=0.0,本就不靠固定姿)。
- **风险:** ①**elbow-out 种子是否在 NEW_FSM 路径生效**(§3.0 verify-before-code)——v1 已证只靠 hover 位置不够,若种子未接须先补;②caging 字段已上 DataBus(§4,已确认 `t5_caging_ready` + 分量存在);③weld solref 软化 + payload 前馈仍 OFF(payload=M1c)。
- **7 轮 sim 已证:** 简化 waypoint / 收臂 / 深蹲 / 容忍都不行;v1 缺的是 **G2 轴向侧插轨迹**——只做了近似轴插且下蹲阶段前臂已压顶,走不到插入。v2 用完整三段 axis_lift 轨迹重写。

---

## 9. 交接

转 `superpowers:writing-plans` 起草 M1b-v2 实现计划(逐任务:Crouch 回退+high-and-back hover → HandleGeometry axis waypoints → ApproachUnder 三段重写 → Grasp hook+caging weld 重写 → sim 闸门)。elbow-out 种子在 NEW_FSM 路径生效性作为 Crouch 任务的 verify-before-code。
