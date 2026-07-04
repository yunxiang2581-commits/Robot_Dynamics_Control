# 运动控制实习求职 · 项目 Roadmap（阶段制）

本文件是面向"机器人运动控制实习"求职的执行计划。目标不是读更多源码，
而是产出**两个做深、能讲透、可放进简历和 GitHub 的完整 loco-manipulation 项目**。

> 起草日期：2026-07-02
> 重大修订：2026-07-03（① 项目二从 RL 全流程改为复现+扩展 wb-mpc-locoman；
> ② 两个项目都是 model-based；③ 去掉周历，改为大阶段+阶段目标制）
> 截止目标：约 2026-08-底

---

## 0. 两个项目的全局定位（先把格局说清）

```text
两个项目现在【都是 model-based loco-manipulation】，RL 路线已放弃。
这不是退步——它正好收敛到目标岗位（WBC/MPC 全身控制、loco-manip，非纯 RL）。

  项目一：OpenLoong 人形 · 实时 WBC 全身控制（C++ / MuJoCo 闭环物理仿真）
          → "边走边用手在世界系完成末端任务" 的能力套件
  项目二：复现+扩展 wb-mpc-locoman（Python / CasADi 全身逆动力学 MPC）
          → B2+Z1 四足+臂，前沿 RA-L 2025 论文的开源复现与任务扩展

两者形成强对照（面试主线叙事）：
  "同一个 loco-manipulation 问题，我走过两条 model-based 路线——
     · 人形上的【实时优先级 WBC】（C++，MuJoCo 闭环，控制层）
     · 四足+臂上的【全身逆动力学 MPC 轨迹优化】（CasADi，规划层，前沿论文）
   我清楚两者的分工：WBC 是瞬时优先级投影，MPC 是有限时域预测优化，
   以及各自适合什么、边界在哪。"
```

**放弃 RL 的代价与对冲（诚实）：**

```text
代价：丢掉"跟得上 learning 范式"这一信号。
对冲：① 目标岗位本就偏 model-based，不是硬伤；
      ② 两个 model-based 项目【更聚焦、都能讲到源码级】，比"一深一浅"更可信；
      ③ RL 仍作为面试谈资保留（能讲 PPO/reward/DR/sim2real 原理，只是不做项目）。
```

---

## 1. 定位与前提（先写事实）

```text
强项：
  - 已读透 OpenLoong 的 WBC / MPC / contact / 状态估计源码
  - 有可运行的 OpenLoong Docker 基线（大量 demo_runs 产物）
  - 硬件：双 3090 + 192G 内存工作站可用

弱项：
  - 目前没有"能演示的产出"，读源码本身在简历上只值一行
  - 项目二是新框架（CasADi 轨迹优化），需要一段上手期

目标岗位偏向：
  - WBC / MPC 全身控制、loco-manipulation（非纯 RL locomotion）

面试官想看两样东西：
  1. 能演示的产出（视频/曲线/指标，最好是你改的）  ← 现在几乎是空的，这两个月补
  2. 能讲清的理解（MPC 建模、WBC 优先级零空间、接触摩擦锥、状态估计）  ← 已经很强
```

---

## 2. 项目一：OpenLoong 人形 loco-manipulation 能力套件（实时 WBC / C++）

### 2.0 代码复核结论（2026-07-02，直接读 OpenLoong 源码，file:line 为证）

复核推翻/确认了几条关键假设，直接影响下面的设计。**多数是好消息，风险已大幅下降。**

```text
【好消息 1】手雅可比现成、6 维、世界对齐、每周期更新 —— 不是"闲置"，是"没接到手任务上"。
  pino_kin_dyn.cpp:155-156  getJointJacobian(..., r/l_hand_joint, LOCAL_WORLD_ALIGNED, J_hd_r/l)
  → J_hd_* 是 6×nv（前3行线速度、后3行角速度），世界对齐坐标系
  pino_kin_dyn.cpp:182-185  hd_r_pos/rot = data.oMi[r_hand_joint] → 真 FK，世界系，每周期更新
  写入 DataBus：pino_kin_dyn.cpp:100-103, 124-127

【好消息 2】dJ_hd 其实被正确计算了 —— 之前以为"没有 dJ"是错的。
  pino_kin_dyn.cpp:171-172  getJointJacobianTimeVariation(..., dJ_hd_r/l)  → 真的 dJ
  但 wbc_priority.cpp:144-145 有 BUG：dJ_hd_l = robotState.J_hd_l（把 J 抄给了 dJ）。
  → 源头有正确 dJ_hd，WBC 里被这行覆盖。要用 dJ 只需改这两行读 robotState.dJ_hd_*。

【关键事实 3】WBC 是两级：先运动学优先级，再力矩 QP。手任务加在运动学级。
  运动学级 priority_tasks → delta_q_final_kin / ddq_final_kin（零空间投影）:50-53,189-191
  力矩级 computeTau() → 带接触力的 QP（QP_nv=6+12, QP_nc=22）→ tauJointRes  :198-199,183
  → 手末端笛卡尔任务放在【运动学优先级栈】里（和现有 HandTrackJoints 同一层）。

【关键事实 4】手任务在 walk 里【已经是最低优先级】—— 天然就在 locomotion 的零空间。
  taskOrder_walk: static_Contact → PosRot → SwingLeg → RedundantJoints → HandTrackJoints(最后)
  → 不用重排优先级！手任务已在腿/接触/base 任务的零空间。这正是我们要的。

【确认 5】现有 HandTrackJoints 确是关节空间、14 维（左7+右7）。
  errX = target_arm_q - q.block<14,1>(7,0)；J = I(14) 挂在 block(0,6,14,14)  :530-534
  walk 里 target_arm_q 与 hip pitch 耦合做摆臂（:521）；stand 是固定姿态（:638）
  手 frame = 腕部最后一个臂关节（pino_kin_dyn.cpp:45），不是指尖。

【确认 6】staircase demo 与 walk_wbc 几乎同源 —— 上楼梯 demo 复用成本低。
  demo/walk_wbc_staircase.cpp vs demo/walk_wbc.cpp：include 仅差 StateEst.h，
  核心差异只有场景文件（scene_staircase.xml vs 默认）。手任务逻辑在共享的
  wbc_priority.cpp 里 → 手任务代码可直接复用到楼梯场景，主要工作是 demo 层门控搬移。
```

### 2.1 核心工程改动（这就是"我做了什么"）

```text
把 HandTrackJoints（关节空间）升级/新增为 HandTrackCart（笛卡尔末端）：

  当前：errX = target_arm_q - q.block<14,1>(7,0)，  J = I(14) @ block(0,6,14,14)
  改为（位置版）：errX = hd_r_pos_des_W - robotState.hd_r_pos_W，  J = J_hd_r.topRows(3)
  姿态版再加 3 行角速度 → 6 维；先右手，再左手。

  · dJ 首版取 0（现有关节任务也是 dJ=0，纯运动学级，风险可控）；要前馈时改 :144-145。
  · 优先级【不需重排】——HandTrackJoints 已是 walk 最低优先级，天然零空间（事实 4）。

复用同一 slot 的副作用：walk 里 HandTrackJoints 现在做摆臂（耦合 hip pitch）。
  改成笛卡尔手任务 = 用同一批手臂关节，等于【替换掉摆臂】。手臂低优先级，
  对平衡影响很小，通常可忽略；想保留摆臂可让另一只手继续摆臂、单手做笛卡尔。
```

### 2.2 物理可行性修正（必须诚实，写进 README/面试都要守）

```text
"向前行走 + 手无限期钉在世界固定点" 物理上做不到：
  base 前进速度 v，世界固定点在体坐标系里以 -v 后退，手臂 reach 只有 ~0.3-0.5m，
  ~0.5m/s 步速下不到 1 秒就到工作空间边界。→ 不能宣称"边走边无限钉住"。

因此两个 loco-manip 交付物都用【物理可持续、诚实】的形式：
  · 原地踏步 + 手钉世界固定点（速度=0 步态激活，世界点永远在工作空间内 → 可无限持续）
  · 缓慢前进 + 手保持世界系【姿态】水平（"端盘子"，姿态不受平移工作空间限制）
```

### 2.3 能力套件（同一套笛卡尔任务层解锁的能力，不是零散动作）

```text
定位：项目一不是"一个 loco-manip 动作"，而是【我在 OpenLoong 零空间 WBC 上
搭了一个世界系笛卡尔末端任务层，下面是它解锁的一整套能力】。
所有能力共用一套代码（同一个 HandTrackCart 任务 + J_hd），沿正交能力轴递进：
  任务空间(位置→姿态→位姿) × 手数(单→双) × 时变(静态→轨迹) × 下肢(站→踏步→行走→上楼梯)
  × 抗扰(无→末端受力/负载)
```

能力清单（★=必做核心，◇=已定加入，○=选做加分）：

```text
★ C1  站立 · 单手 · 位置保持           基础设施验证；base 扰动手自动补偿
★ C2  站立 · 单手 · 世界系轨迹跟踪      时变能力；画圆/直线
◇ C3  站立 · 双手 · 协同保持虚拟刚体    bimanual；两手相对位姿不变（像端箱子）
★ C4  原地踏步 · 单手 · 手钉世界点      loco-manip 核心；踏步中身晃手不动
★ C5  前进行走 · 单手 · 世界系姿态水平   "端盘子"行走
◇ C6  上楼梯 · 手同时端托盘/钉世界点    最强画面；复用 staircase demo（事实 6）
◇ C7  抗扰恢复 · 末端受脉冲力/持续负载   MuJoCo xfrc_applied；零空间自动拉回
```

> C3/C6/C7 是你明确要加的"补厚度"能力（bimanual + 上楼梯 + 抗扰），
> 让项目一从"一个动作"变成"一套能力矩阵"，且都建立在同一套代码上、不散。

### 2.4 大阶段划分（阶段目标 = 可验收的里程碑）

```text
━━ 阶段 1：笛卡尔任务层地基（C1 + C2）━━
  目标：在 stand 模式把 HandTrackCart 接对、跑通、可复用。
  验收：
    · C1 —— base 平移 5cm 时手世界系漂移 < 1cm，errX 收敛不发散
    · C2 —— 跟踪 10cm 半径圆，末端 RMS 误差 < 2cm
  产出：站立手钉点/画圆的自检视频 + 建 GitHub repo + README 骨架
  意义：证明"笛卡尔 errX + J_hd 方向对、复用 slot 生效"，是后续全部能力的地基。

━━ 阶段 2：loco-manipulation 核心（C4 + C5）★项目主体 ━━
  目标：把手任务叠加到 locomotion（踏步→前进），上下肢解耦、腿不失稳。
  验收：
    · C4 —— 连续踏步 ≥10 步，手世界系漂移 < 2cm，不失稳
    · C5 —— 前进 ≥2m，手世界系 roll/pitch 偏差 < 5°，不失稳
  产出：踏步手钉点 + 端盘子前进的正式演示视频。
  ★这是简历最亮、最贴岗位的部分。风险最高（攻坚），配 Plan B（见 2.5）。

━━ 阶段 3：能力扩展（C3 + C6 + C7，补厚度）━━
  目标：在同一套代码上叠加 bimanual、上楼梯、抗扰三个能力点。
  验收：
    · C3 —— 扰 base 时两手【相对】距离/朝向漂移 < 2cm/5°
    · C6 —— 上楼梯全程手端托盘保持水平（或钉世界点），不失稳、托盘不倾覆
    · C7 —— 末端施脉冲力/负载后偏离，零空间自动拉回，腿不失稳
  产出：三个能力片段视频。
  意义：把项目从"能走能端"扩成"一套能力矩阵"，bimanual/上楼梯/抗扰都是强加分词。
  弹性：阶段 2 若严重超期，本阶段按 C6 > C7 > C3 的优先级取舍（上楼梯画面最值）。

━━ 阶段 4：量化分析 + 交付（Stage D）★把 demo 升级为"项目"━━
  目标：用数据证明"手任务没破坏步态""零空间设计是必要的"。
  内容：
    · 末端跟踪误差 vs 踏步幅度 / 前进速度 / 上楼梯
    · 有/无手任务的稳定性对比：ZMP、双脚接触力 Fz 波动
    · 优先级消融：把手任务【提到腿之上】→ 预期破坏接触/失稳（反例证明零空间必要）
  产出：一个 repo、一套代码、一段"能力总览"视频（C1-C7 各片段）、
        Stage D 分析图表、技术文档（笛卡尔任务层设计 + 零空间/优先级排序理由）。
  ★ "完整项目" = 阶段 2 演示 + 阶段 3 扩展 + 阶段 4 分析。阶段 1 是地基，快速跨过。
```

### 2.5 风险与 Plan B / C（诚实）

```text
风险已大幅下降（复核后）：手雅可比/FK/dJ 全现成、优先级天然正确、是两级 WBC 的
  运动学级插入。主要工作量在调增益和验证，不是"从零接笛卡尔任务"。
  阶段划分自带降风险：即便阶段 2 卡住，阶段 1（站立笛卡尔）+ 阶段 3 的 C3（bimanual）
  已能构成一个"世界系笛卡尔任务层 + 双手协同"的可展示项目（只少 locomotion 亮点）。

Plan B（触发：阶段 2 攻坚数周仍卡在 C4，笛卡尔手任务一进 walk 就失稳且调不动）：
  降级为【关节空间 loco-manipulation】：保留 HandTrackJoints 关节空间，把摆臂
  target_arm_q 改成可见的非摆臂定姿/举物动作，重点转 Stage D 量化。
  简历弱一档（关节 vs 笛卡尔），但仍有"改 WBC 任务 + 量化行走稳定性"实证；
  且 C1-C3 站立笛卡尔照常交付，项目不空。

Plan C（Plan B 也卡住，最终兜底）：
  退回 push recovery：xfrc_applied 给 base 施扰，展示 MPC+WBC 稳住，量化最大可恢复冲量。
  工作量小、成功率高、视频好，但离目标岗位远，最后兜底。
```

### 2.6 简历一句话（阶段 2+3+4 达成后）

```text
"在 OpenLoong 人形两级 WBC（运动学优先级 + 力矩 QP）控制栈中，新增世界系笛卡尔
 手部末端任务层（复用零空间最低优先级层），构建从单手位置保持、世界系轨迹跟踪、
 双手协同、到踏步/行走/上楼梯中末端稳定跟踪与末端抗扰的 loco-manipulation 能力套件：
 上肢操作与下肢步态解耦、腿部不失稳；量化了末端跟踪精度、行走接触力波动，
 并用优先级消融验证零空间设计的必要性。"
```

---

## 3. 项目二：复现 + 扩展 wb-mpc-locoman（全身逆动力学 MPC / Python·CasADi）

### 3.0 这个仓库到底是什么（2026-07-03 读码确认，file 为证）

```text
论文：Whole-Body Inverse Dynamics MPC for Legged Loco-Manipulation, RA-L 2025
      Molnar 等, ETH Zurich。arXiv:2511.19709 / DOI:10.1109/LRA.2025.3636005
仓库：external/open_source_repos/wb-mpc-locoman（已 clone，代码+模型开源）

机器人：B2 + Z1（宇树 B2 四足 + Z1 机械臂），四足+臂，【非人形】。
        模型只有 robots/b2_description/urdf/b2.urdf 和 b2_z1_description/urdf/b2_z1.urdf。
技术栈：Python + CasADi + Pinocchio，全身逆动力学（RNEA）轨迹优化。
        求解器阶梯：fatrop（>10x 快过 ipopt）/ ipopt / osqp(SQP)；支持 codegen 出 C 库。
动力学模型：whole_body_{rnea,acc,aba} + centroidal_{acc,vel}（多个变体，见 dynamics/）。
```

**⚠️ 最关键的诚实点（简历/面试绝不能说错）：**

```text
main.py 的 "mpc_loop" 【不是闭环物理仿真】：
  它反复求解 OCP（默认 200 次），用 OCP 自己的动力学积分推进状态（state_integrate），
  最后在 meshcat 里做【运动学回放】+ 力箭头可视化（那两个 gif 就是这么来的）。
  没有 MuJoCo / 物理引擎在环，没有真实接触。

"拉10kg / 推箱子 / 擦白板 / 柔顺交互" 全部通过设定 OCP 的
  arm_force_des（末端期望力，全局系）/ arm_vel_des（末端期望速度）/ base_vel_des 实现——
  是【给定期望接触力/轨迹的全身优化】，不是和仿真物体真接触。

→ 面试话术必须是："全身 MPC 在给定末端力/轨迹目标下的全身轨迹优化"，
  绝不能说成"MuJoCo 闭环抗扰"或"真实接触物理"。说错会被当场戳穿。
```

### 3.1 项目二要交付什么（分清"复现"和"扩展"）

```text
复现（reproduce）= 把作者的 pipeline 跑起来，重现论文的两个官方结果：
  · b2_z1_tracking（末端速度轨迹跟踪）
  · b2_z1_pulling（末端施力/拉拽）
扩展（extend）= 在同一 OCP 框架上，通过改 targets/权重/gait 做【新的 loco-manip 任务】：
  · 走着拉重物（调大 arm_force_des，看全身如何配重/调姿维持平衡）
  · 推箱子（持续前向末端力 + base 前进）
  · 擦白板（末端在竖直面走轨迹 + 法向保持力）
  · 柔顺交互（末端力目标随"外部输入"变化，展示 force-level 全身响应）
深化（stretch，非必做）：见阶段 4。
```

### 3.2 大阶段划分（阶段目标 = 可验收的里程碑）

```text
━━ 阶段 1：环境搭建 + 官方复现 ━━
  目标：conda 环境跑通，重现论文两个官方 demo。
  内容：
    · conda env create -f environment.yaml；跑通 python main.py（默认 B2_Z1 + trot）
    · fatrop 求解器跑通；重现 b2_z1_tracking 和 b2_z1_pulling（meshcat 回放 + 力可视化）
    · 记录求解时间、约束违反（main.py 已打印 solve time / CV）
  验收：两个官方结果能复现出来、meshcat 里 B2+Z1 动起来、求解稳定收敛。
  产出：复现视频 + solve-time/CV 记录 + 建 GitHub repo（fork 或独立复现仓）。
  ⚠️ 首要风险在这一阶段（新框架、CasADi/fatrop 依赖、codegen 工具链），先趟通。

━━ 阶段 2：读透 OCP 建模 + 讲清（这是面试最值钱的部分）━━
  目标：能白板讲清这套全身逆动力学 MPC 的建模，不只是会跑。
  内容（对着 optimization/ 和 dynamics/ 读）：
    · 状态/输入定义、whole-body RNEA 动力学如何进 OCP（ocp_whole_body_rnea.py）
    · 决策变量、代价（Q_diag/R_diag）、约束（接触、摩擦锥、力矩限幅、swing 轨迹）
    · gait schedule 如何以 contact/swing schedule 进 OCP
    · 求解器阶梯：fatrop 为何靠 block-sparse 结构比 ipopt 快 >10x
    · centroidal vs whole-body 变体的差异（论文 benchmark 的意义）
  验收：产出一份"OCP 建模讲解"技术笔记，能回答"这套 MPC 和 OpenLoong 的
        WBC 差在哪、和 MuJoCo MPC 差在哪"。
  产出：建模讲解文档（面试白板素材）。

━━ 阶段 3：任务扩展（新 loco-manip 任务）★项目二的"我做了什么"━━
  目标：在 OCP 框架上做出作者没直接给的新任务，证明"改得动、理解到位"。
  内容：改 arm_force_des / arm_vel_des / base_vel_des / gait / 权重，实现：
    · 走着拉重物（force 目标加大，观察全身配重）
    · 推箱子 / 擦白板（末端沿面轨迹 + 法向力）中挑 1-2 个做深
    · 柔顺交互（force 目标时变）
  验收：每个新任务能收敛、meshcat 回放合理、末端力/轨迹达到设定目标。
  产出：新任务视频 + 一张"改了什么参数→得到什么行为"的对照表。
  弹性：任务多选精不选全——做深 2 个（如"拉重物 + 擦白板"）好过浅尝 4 个。

━━ 阶段 4：深化（stretch，有余力才做，明确标注为加分）━━
  目标：把"轨迹优化+运动学回放"往"真闭环"推一步，或做定量对比。
  三选一（按性价比）：
    a) 求解器 benchmark：fatrop vs ipopt vs osqp 的 solve-time/收敛对比（改 solver 即可，最省力）
    b) MuJoCo 闭环：把 MPC 解作为参考轨迹喂给 MuJoCo 里的跟踪控制器，做真闭环
       —— 工作量大、是真加分，但别为它赌上项目二主体
    c) 换机器人模型：把另一个四足+臂 URDF 接进去（工作量中，展示框架理解深度）
  验收：视所选而定；这一阶段【不做也不影响项目二成立】。
```

### 3.3 与项目一的关系（对照叙事 = 面试主线）

```text
两个项目讲"同一个 loco-manip 问题的两条 model-based 路线"：
  项目一 OpenLoong：人形 · 实时优先级 WBC · C++ · MuJoCo 闭环 · 控制层（瞬时投影）
  项目二 wb-mpc：   四足+臂 · 全身逆动力学 MPC · CasADi · 轨迹优化 · 规划层（有限时域预测）

能讲清的边界（面试金句）：
  · WBC = 瞬时的加权/零空间投影，无预测，靠优先级排任务；实时性天生好。
  · 全身 MPC = 有限时域预测优化，能提前考虑未来接触/力，但要解 NLP，靠 fatrop 才实时。
  · loco-manip 里"末端力/接触"怎么进公式：WBC 在力矩 QP 层，MPC 作为 OCP 的力目标/约束。
```

### 3.4 简历一句话（阶段 1-3 达成后）

```text
"复现并扩展 ETH Zurich 全身逆动力学 MPC 工作（RA-L 2025, wb-mpc-locoman）：在
 B2+Z1 四足机械臂上用 CasADi/Pinocchio 构建全身 MPC，复现末端跟踪与施力 loco-manip，
 并扩展出走行拉重物/擦白板等新任务；理解 whole-body 与 centroidal 动力学变体、
 摩擦锥与力矩约束，及 fatrop 相对 ipopt 的 block-sparse 加速。"
```

---

## 4. GitHub / 简历呈现要点

```text
- 两个独立 repo。README 顶部都放 GIF/视频 —— 招控制的人 3 秒内要看到机器人在动。
- 写清 "我做了什么 → 达到了什么指标"：
    项目一："将关节空间手任务升级为世界系笛卡尔任务层；踏步/行走/上楼梯中末端
             跟踪误差 < X cm，接触力波动 < Y%，含双手协同与末端抗扰。"
    项目二："复现+扩展 RA-L 2025 全身 MPC；新增走行拉重物/擦白板任务，末端力/轨迹
             达标，求解 fatrop 实时。" （诚实标注：轨迹优化+运动学回放，非物理闭环）
- 两个 repo README 都点一句共同叙事：同一 loco-manip 问题的两条 model-based 路线
  （实时 WBC vs 全身 MPC），让面试官看到这不是两个拼凑的小东西。
- logbook 阅读笔记是面试讲解素材，但简历呈现的是"项目"不是"读书笔记"。
```

---

## 5. 面试话术清单（需能白板推导）

```text
必须能手推/讲清：
  - MPC：单刚体模型（SRBM）vs 全身逆动力学（RNEA）、状态/输入、接触力、摩擦锥、预测时域
  - WBC：优先级任务、零空间投影 N、加权伪逆、task order 为什么这么排、两级（运动学+力矩QP）
  - 笛卡尔 vs 关节空间任务：手雅可比 J_hd 的作用、为什么笛卡尔任务能"钉世界点"
  - loco-manipulation：手任务如何进 WBC 零空间/如何作为 MPC 的末端力目标、为何不破坏行走
  - 接触：摩擦锥、支撑多边形、ZMP / 接触力约束
  - 状态估计：为什么行走要靠触地做零速修正
  - 求解器：fatrop 的 block-sparse 结构为何比 ipopt 快、SQP(osqp) 与 IP 法的区别
  - 范式边界：实时 WBC（瞬时投影）vs 全身 MPC（预测优化）各自适合什么、边界在哪
  - RL（仅谈资，讲清即可）：PPO 流程、reward 各项、domain randomization 作用、
    model-based vs learning 的边界
```

---

## 6. 关于项目 E（AugMPC / IBRIDO）的明确决定

```text
决定：不复现项目 E。
理由：eval 路线要 build .sif + clone ~21 个仓库 + 多进程共享内存 IPC + 适配作者
      HPC 路径，投入产出比极差；跑通也只是"复现别人的 eval 回放"，故事弱。
定位：作为"我理解 RL-augmented MPC 架构（non-gaited 接触调度）"的面试谈资即可。
```

---

## 7. 下一步（立即可开工）

```text
项目一（主线，占专注时间块）：
  1. 复核 2.0 的 file:line 与当前源码一致（防行号漂移）
  2. 在 stand 模式接出 HandTrackCart：errX = hd_r_pos_des_W - hd_r_pos_W，J = J_hd_r.topRows(3)
  3. 跑 C1（站立手钉点 + 扰 base 验证补偿），过 done-check
  4. 建项目一 GitHub repo
  （项目一有独立的每日推进对话 + task1_kickoff_prompt.md 注入提示词）

项目二（可在项目一的编译/仿真等待时间穿插上手）：
  1. conda env create -f environment.yaml，趟通 CasADi/fatrop 依赖（首要风险，先探）
  2. python main.py 复现官方 B2_Z1 trot + tracking/pulling
  3. 对着 optimization/ocp_whole_body_rnea.py 起步阶段 2 的建模阅读
```
