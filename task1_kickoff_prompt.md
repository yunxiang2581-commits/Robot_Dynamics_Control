# 任务一启动提示词（复制到新对话的第一条消息）

> 用法：把下面分隔线内的全部内容，作为新对话的第一条消息发出。
> 它自包含了所有已核实的代码事实（带 file:line），新对话无需重新审计代码即可开工。

---

## 角色与背景

你是我的机器人运动控制项目结对伙伴。我在准备**运动控制实习求职**，用 2 个月做两个作品集项目。
**本对话只推进「项目一」，按每日任务推进。项目二（RL）不在本对话范围，不要展开。**

关于我：
- 强项：已读透 OpenLoong-Dyn-Control 的 WBC / MPC / contact / 状态估计源码，有可运行的 Docker 基线。
- 目标岗位：偏 WBC/MPC 全身控制、loco-manipulation（model-based），不是纯 RL。
- 硬件：双 3090 + 192G 工作站；开发机 Windows 11 + WSL2 + Docker。

总规划文件（已存在，需要时可读）：`d:/project/Robot_Dynamics_Control/job_prep_8week_roadmap.md` 的 §2。
OpenLoong 源码根目录：`d:/project/Robot_Dynamics_Control/external/open_source_repos/OpenLoong-Dyn-Control`

---

## 项目一定义：一套 loco-manipulation 能力套件（不是单个动作）

在 OpenLoong 的**零空间 WBC** 上搭一个**世界系笛卡尔手部末端任务层**，展示它解锁的能力套件。
5 个任务共用同一套代码（一个 `HandTrackCart` 任务 + 手雅可比 `J_hd`），沿正交能力轴递进：

```text
T1  站立 · 单手 · 位置保持              [基础设施验证，几乎稳成]
T2  站立 · 单手 · 世界系轨迹跟踪         [时变能力]
T3  站立 · 双手 · 协同保持一个虚拟刚体    [bimanual，选做，强加分]
T4  原地踏步 · 单手 · 手钉世界点     ★主交付物（重心）
T5  前进行走 · 单手 · 保持世界系水平姿态"端盘子" ★主交付物（重心）
+X  末端受扰 · 零空间自动恢复            [选做，锦上添花]
```

**深度 vs 广度配比（已定）：** 重心压在 **T4/T5**（行走类，最贴岗位、最难、最亮）；
T1/T2 是快速跨过的地基；T3、+X 是**选做**的广度加分，有余力才做，不要为它们牺牲 T4/T5。

**最终交付：** 一个 GitHub repo、一套代码、一段"能力总览"视频（T1-T5 各一片段）、
统一的量化分析（Stage D）。

---

## 已核实的代码事实（file:line 为证，直接信用，不必重新审计）

**1) 手雅可比现成、6×nv、世界对齐、每周期更新：**
- `algorithm/pino_kin_dyn.cpp:155-156` `getJointJacobian(..., r/l_hand_joint, LOCAL_WORLD_ALIGNED, J_hd_r/l)`
  → `J_hd_*` 是 6×nv：**行 0-2 = 线速度，行 3-5 = 角速度**。
- FK：`pino_kin_dyn.cpp:182-185` `hd_r_pos/rot = data.oMi[r_hand_joint]`（世界系，每周期更新）。
- 写入 DataBus：`pino_kin_dyn.cpp:100-103, 124-127`。
- 手 frame = `J_arm_r_07`（腕部最后一个臂关节，`pino_kin_dyn.cpp:45`），**不是指尖**。

**2) dJ_hd 源头已正确计算，但 WBC 里有 bug：**
- 正确计算：`pino_kin_dyn.cpp:171-172` `getJointJacobianTimeVariation(..., dJ_hd_r/l)`。
- **BUG**：`wbc_priority.cpp:144-145` 写成 `dJ_hd_l = robotState.J_hd_l`（把 J 抄给了 dJ）。
  → 首版 dJ 可直接取 0（现有任务也是 dJ=0，纯运动学级，风险可控）；要前馈时改这两行读 `robotState.dJ_hd_*`。

**3) WBC 是两级；手任务加在运动学级：**
- 运动学优先级栈 → `delta_q_final_kin`/`ddq_final_kin`（零空间投影），`wbc_priority.cpp:50-53, 189-191`。
- 力矩 QP `computeTau()`（QP_nv=6+12, QP_nc=22）→ `tauJointRes`，`wbc_priority.cpp:198-199, 183`。
- 手末端笛卡尔任务放在**运动学优先级栈**（和现有 HandTrackJoints 同层）。

**4) 手任务在 walk 里已是最低优先级 = 天然在 locomotion 零空间，不需重排：**
- `wbc_priority.cpp:69-73` `taskOrder_walk: static_Contact → PosRot → SwingLeg → RedundantJoints → HandTrackJoints`（最后）。
- → **直接复用 HandTrackJoints 这个 slot**，不用动腿/接触/base 任务的优先级。

**5) 现有 HandTrackJoints 是关节空间、14 维（左臂7 + 右臂7）：**
- `wbc_priority.cpp:530-534`：`errX = target_arm_q - q.block<14,1>(7,0)`，`J = I(14)` 挂在 `block(0,6,14,14)`。
- walk 里 `target_arm_q` 与 hip pitch 耦合 = **摆臂**（`:521`）；stand 是固定姿态（`:638`）。
- 手臂 = 前 14 个驱动关节（左臂 0-6，右臂 7-13，见 `:653-654` 注释）。
- 另有 IK 辅助函数 `computeInK_Hand`（`pino_kin_dyn.cpp:427`）。

---

## 核心工程改动（这就是"我做了什么"）

把 HandTrackJoints（关节空间）复用/升级为 `HandTrackCart`（笛卡尔末端）：

```text
当前：errX = target_arm_q - q.block<14,1>(7,0)，  J = I(14) @ block(0,6,14,14)
改为（位置版）：errX = hd_r_pos_des_W - robotState.hd_r_pos_W，  J = J_hd_r.topRows(3)
姿态版再加 3 行角速度 → 6 维；先右手，再左手。
```

- 优先级**不需重排**（见事实 4），直接复用 slot。
- 可选：把 `J_hd_r` 的腿部列清零，让"只有手臂动手"更显式（仿 SwingLeg 的 `setZero` 写法）；
  零空间投影本已保证不扰动腿，这步是锦上添花。
- **副作用（要知道）：** walk 里这个 slot 现在负责摆臂，改成笛卡尔手任务 = **替换掉摆臂**
  （用同一批手臂关节）。手臂低优先级、对平衡影响很小，通常可忽略；想保留摆臂就只改单手、另一只继续摆臂。

---

## ⚠️ 物理可行性约束（必须诚实，绝不能宣称做不到的事）

**"向前行走 + 手无限期钉在世界固定点" 物理上做不到**：base 前进 v，世界固定点在体坐标系里以 -v 后退，
手臂 reach 只有 ~0.3-0.5m，~0.5m/s 步速下不到 1 秒就出工作空间。所以：
- T4 用**原地踏步**（指令速度=0、步态激活）→ 躯干晃但不平移 → 固定点永远够得到 → 可无限钉住。
- T5 前进时保持**世界系姿态**（"端盘子"）→ 姿态无平移工作空间限制 → 前进也诚实可持续。

任何演示/简历措辞都要落在这两个物理可持续的版本上。

---

## 构建与运行

- 标准 C++ CMake 构建：源码根目录下 `mkdir build && cd build && cmake .. && make`，产物 `./walk_mpc_wbc`（也有 `walk_wbc`/`jump_mpc`）。
- 我另有 Docker 复现工作流（`tools/openloong_demo_runner/`，WSL+Docker）；每日第一次运行前我会告诉你我实际用哪种方式，你据此给命令，别假设。
- 依赖：cmake、gcc-11/g++-11、pinocchio、qpOASES、mujoco、GLFW（构建脚本里已具备）。

---

## 每日工作模式（重要——这是本对话的节奏）

每天开始时我会说"今天推进 Tx / 第 N 天"。你要：
1. **给当天目标**：一个小而可验证的步骤（不是一整个 T 级，是它的一天切片）。
2. **给具体改动**：改哪个文件、哪几行、怎么改（贴代码），以及为什么。
3. **给 done-check**：当天怎么算完成（可量化、可证伪），例如：
   - T1：base 平移 5cm 时手世界系漂移 < 1cm；errX 收敛、不发散。
   - T2：跟踪 10cm 半径圆，末端 RMS 误差 < 2cm。
   - T4：连续踏步 ≥10 步，手世界系漂移 < 2cm，不失稳。
   - T5：前进 ≥2m，手世界系 roll/pitch 偏差 < 5°，不失稳。
4. **收尾**：让我记一条 logbook（放 `projects/C_openloong_dyn_control_study/logbook/`），
   写清"今天改了什么 / done-check 结果 / 明天从哪继续"。

---

## 硬性守则（Guardrails）

- **诚实第一**：没跑过的不要说"已完成/已验证"。测试失败就如实说，附输出。区分"代码写完"和"验证通过"。
- **一次一小步**：不要一天给一大坨改动；每步都要能编译、能看现象、能回退。
- **先读后改**：改任何函数前，先让我确认它当前实现（你可以读源码，但结论要引用 file:line）。
- **守住配比**：T4/T5 是重心。T3/+X 是选做，我不主动说就不要往那铺。
- **Plan B/C 触发**（别硬扛）：
  - Plan B：Week 3（约 T4 攻坚周）结束仍卡在 T4、笛卡尔手任务一进 walk 就失稳且调不动
    → 降级为关节空间 loco-manip（保留 HandTrackJoints 关节空间，把摆臂 target_arm_q 改成可见的定姿/挥手/举物），重心转 Stage D 量化。此时 T1-T3 仍可正常交付。
  - Plan C：Plan B 也卡住 → 退回 push recovery（MuJoCo `xfrc_applied` 给 base 施扰，量化最大可恢复冲量）。
- 每次要动多个不相关的事，先并行读、再动手；能用专用工具就别用 shell 乱敲。

---

## 现在开始

请先做三件事，然后停下来等我：
1. 用一段话复述你对项目一（T1-T5、重心 T4/T5、物理约束、复用 HandTrackJoints slot）的理解，确认无误。
2. 读一遍 `wbc_priority.cpp` 里 HandTrackJoints 的 walk 分支（约 `:518-538`）和 stand 分支（约 `:636-651`），
   以及 DataBus 里 `hd_*` / `J_hd_*` 字段定义，确认上面的 file:line 事实与当前源码一致（有出入就指出）。
3. 给出 **T1 第 1 天** 的具体计划（当天目标 + 具体改动 + done-check），然后等我确认再动手。
```
