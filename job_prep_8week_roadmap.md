# 运动控制实习求职 · 8 周项目 Roadmap

本文件是面向"机器人运动控制实习"求职的两个月执行计划。目标不是读更多源码，
而是产出一个**做深、能讲透、可放进简历和 GitHub 的完整项目**，外加一个证明
"跟得上 learning 范式"的轻量第二项目。

> 起草日期：2026-07-02
> 修订日期：2026-07-02（基于对 wbc_priority.cpp 的代码复核，纠正了对 HandTrack 任务的关键误解）
> 截止目标：约 2026-08-底（8 周）

---

## 1. 定位与前提（先写事实）

已确认的现状：

```text
强项：
  - 已读透 OpenLoong 的 WBC / MPC / contact / 状态估计源码
  - 有可运行的 OpenLoong Docker 基线（大量 openloong_demo_runs 产物）
  - 硬件：双 3090 + 192G 内存工作站可用

弱项：
  - RL 基础较弱（PPO/SAC/Isaac Lab/legged_gym 未成体系）
  - 目前没有"能演示的产出"，读源码本身在简历上只值一行

目标岗位偏向：
  - WBC / MPC 全身控制、loco-manipulation（非纯 RL locomotion）
```

关键认知：

```text
面试官招运动控制实习生，想看到两样东西：
  1. 能演示的产出（视频/曲线/指标，最好是你改的）  ← 你现在几乎是空的
  2. 能讲清的理解（MPC 建模、WBC 优先级零空间、接触摩擦锥、状态估计）  ← 你已经很强
两个月的核心任务是补第 1 项，且要补成"一个完整的东西"，不是几个零散 demo。
```

### 1.1 代码复核结论（决定项目主线，2026-07-02 核对 external 原始仓库）

复核 `algorithm/wbc_priority.cpp` 后，**纠正了老计划里一个会导致项目跑偏的误解**：

```text
误解（老计划）：HandTrackJoints 是"手臂末端笛卡尔跟踪"任务，激活即可。
事实（代码）：
  - HandTrackJoints 是【关节空间】任务，不是笛卡尔末端任务。
    · errX = target_arm_q(14) - q(手臂14关节)，雅可比 J = 手臂关节上的单位阵
      （wbc_priority.cpp:516-534 walk / 636-651 stand）
  - walk 模式下手臂【已经在动】：target_arm_q 与髋 pitch 耦合，做行走摆臂
      target_arm_q << 0.475 - 0.75*r_hip_pitch, ... （:521）
  - stand 模式是【固定关节姿态】（:638）
  - 真正的笛卡尔量【存在但被闲置】：
      DataBus 有世界系手位置 hd_l/r_pos_W、手姿态 hd_l/r_rot_W、手雅可比 J_hd_l / J_hd_r
      这些在 dataBusRead 里被读进 WBC（:114-117, :142-143），但 HandTrackJoints 没用它们。
```

这个事实**同时是坏消息和好消息**：

```text
坏消息：想做成简历上讲得响的"末端任务"，不能靠激活现成任务——
        关节空间摆臂谁都能调参，故事弱。
好消息：把关节空间任务【升级为笛卡尔末端任务】（用那两个闲置的手雅可比 J_hd_*），
        才是真正"把读懂源码变成改得动系统"的证明——这是更强的简历故事。
        且地基是稳的：优先级栈是标准零空间投影，任务框架真实存在、能激活。
```

已定的两个主线决策（2026-07-02）：

```text
① 主线难度 = 笛卡尔末端任务（把 HandTrackJoints 升级为用 J_hd_* 的笛卡尔任务）
② 8 周结构 = 主项目做深 + 保留 RL 四足作为轻量第二项目
```

---

## 2. 主项目：一套 loco-manipulation 能力套件（不是一个动作，也不是零散拼凑）

### 2.0 代码复核结论（2026-07-02，直接读 OpenLoong 源码，file:line 为证）

复核推翻/确认了几条关键假设，直接影响下面的设计。**多数是好消息。**

```text
【好消息 1】手雅可比是现成的、6 维、世界对齐、每周期更新 —— 不是"闲置"，是"没接到手任务上"。
  pino_kin_dyn.cpp:155-156  getJointJacobian(..., r/l_hand_joint, LOCAL_WORLD_ALIGNED, J_hd_r/l)
  → J_hd_* 是 6×nv（前3行线速度、后3行角速度），世界对齐坐标系
  pino_kin_dyn.cpp:182-185  hd_r_pos/rot = data.oMi[r_hand_joint] → 真 FK，世界系，每周期更新
  写入 DataBus：pino_kin_dyn.cpp:100-103, 124-127

【好消息 2】dJ_hd 其实被正确计算了 —— 之前以为"没有 dJ"是错的。
  pino_kin_dyn.cpp:171-172  getJointJacobianTimeVariation(..., dJ_hd_r/l)  → 真的 dJ
  写入 DataBus：pino_kin_dyn.cpp:102-103
  但是——wbc_priority.cpp:144-145 有 BUG：dJ_hd_l = robotState.J_hd_l（把 J 抄给了 dJ）。
  → 源头有正确 dJ_hd，WBC 里被这行覆盖。要用 dJ 只需改这两行读 robotState.dJ_hd_*。

【关键事实 3】WBC 是两级：先运动学优先级，再力矩 QP。手任务加在运动学级。
  运动学级 priority_tasks → delta_q_final_kin / ddq_final_kin（零空间投影）wbc_priority.cpp:50-53,189-191
  力矩级 computeTau() → 带接触力的 QP（QP_nv=6+12, QP_nc=22）→ tauJointRes  :198-199,183
  → 手末端笛卡尔任务放在【运动学优先级栈】里（和现有 HandTrackJoints 同一层）。

【关键事实 4】手任务在 walk 里【已经是最低优先级】—— 天然就在 locomotion 的零空间。
  taskOrder_walk: static_Contact → PosRot → SwingLeg → RedundantJoints → HandTrackJoints(最后)
  wbc_priority.cpp:69-73
  → 不用重排优先级！手任务已在腿/接触/base 任务的零空间，改不了它们。这正是我们要的。

【确认 5】现有 HandTrackJoints 确是关节空间、14 维（左7+右7）。
  errX = target_arm_q - q.block<14,1>(7,0)；J = I(14) 挂在 block(0,6,14,14)  wbc_priority.cpp:530-534
  walk 里 target_arm_q 与 hip pitch 耦合做摆臂（:521）；stand 是固定姿态（:638）
  手臂 = 前 14 个驱动关节（左臂0-6，右臂7-13，见 :653-654 注释）
  手 frame = J_arm_r_07（腕部最后一个臂关节，pino_kin_dyn.cpp:45），不是指尖。
```

### 2.1 一句话定义这个"完整任务"（已按物理可行性修正）

```text
让 OpenLoong 人形【下肢做 locomotion 的同时，上肢在世界系里独立完成一个末端任务】：
手末端作为【笛卡尔任务】进入 WBC 运动学优先级栈的最低层（现成位置），
腿部 MPC + 接触维持步态；步态引起的躯干晃动/俯仰由零空间自动补偿，
使手末端在世界系里稳定完成任务，且腿不因手的动作失稳。
```

**⚠️ 物理可行性修正（必须诚实，不能含糊）：**

```text
"向前行走 + 手无限期钉在世界固定点" 物理上做不到：
  base 前进速度 v，世界固定点在体坐标系里以 -v 后退，手臂 reach 只有 ~0.3-0.5m，
  ~0.5m/s 步速下不到 1 秒就到工作空间边界。→ 不能宣称"边走边无限钉住"。

因此交付物改为下面两个【都物理可持续、都诚实】的版本：
```

**主交付物（可无限持续、视频最干净、推荐）：**

```text
原地踏步 + 手在世界系钉住一个固定点。
  指令速度=0 但步态激活 → 躯干上下起伏、左右摇摆、微俯仰，但不整体平移，
  所以世界固定点永远在工作空间内 → 手可以无限钉住 → 循环视频干净。
  画面：机器人在踏步、上身在晃，手却"焊"在空间一点不动。
  → 无歧义地展示"世界系笛卡尔控制 + 全身零空间补偿"。
```

**扩展交付物（加"真的在走"的观感、仍诚实）：**

```text
缓慢前进行走 + 手保持世界系【姿态】水平（"端盘子"）。
  姿态（orientation）在世界系可无限保持，不受前进平移的工作空间限制；
  位置可用"体坐标系前方固定偏移"让手始终在身前。
  画面：像服务员端着托盘往前走，躯干在颠、盘子始终水平。
  → 前进行走 + 世界系任务，honest 且直观。
```

### 2.2 核心工程改动（这就是"我做了什么"）

```text
把 HandTrackJoints（关节空间）升级/新增为 HandTrackCart（笛卡尔末端）：

  当前：errX = target_arm_q - q.block<14,1>(7,0)，  J = I(14) @ block(0,6,14,14)
  改为（位置版）：errX = hd_r_pos_des_W - robotState.hd_r_pos_W，  J = J_hd_r.topRows(3)
       （J_hd_r 是 6×nv 世界对齐，前 3 行即线速度 → 位置任务取 topRows(3)）
  姿态版再加 3 行角速度 → 6 维；先右手，再左手。

  · dJ 首版取 0 即可（现有关节任务也是 dJ=0，纯运动学级，风险可控）；
    要前馈时，改 wbc_priority.cpp:144-145 读 robotState.dJ_hd_*（源头已有正确 dJ）。
  · 优先级：【不需重排】——HandTrackJoints 已是 walk 最低优先级，天然在 locomotion 零空间。
    直接复用这个 slot 即可（见事实 4）。
  · 可选：把 J_hd_r 的腿部列清零，让"只有手臂动手"更显式（仿 SwingLeg 的 setZero 写法）；
    严格说零空间投影已保证不扰动腿，这一步是锦上添花。
```

**复用同一 task slot 的副作用（要知道）：**

```text
walk 里 HandTrackJoints 现在负责【摆臂】（target_arm_q 耦合 hip pitch）。
改成笛卡尔手任务 = 用同一批手臂关节，等于【替换掉摆臂】，不是"和摆臂打架"。
  → 会失去自然摆臂（手臂低优先级，对平衡影响很小，通常可忽略）。
  → 若想保留部分摆臂，可让另一只手继续做摆臂、只改单手做笛卡尔任务。
```

### 2.3 能力阶梯（T1-T5：同一套笛卡尔任务层解锁的能力套件，重心在行走）

```text
定位转变：项目一不是"一个 loco-manip 动作"，而是【我在 OpenLoong 零空间 WBC 上
搭了一个世界系笛卡尔末端任务层，下面是它解锁的一整套能力】。
5 个任务共用一套代码（同一个 HandTrackCart 任务 + J_hd），沿正交能力轴递进：
  任务空间(位置→姿态→位姿) × 手数(单→双) × 时变(静态→轨迹) × 下肢(站→踏步→行走)

配比：重心压在 T4/T5（行走类，最贴岗位、最难、最亮）。
     T1-T3（站立类，几乎稳成）是快速台阶 + 红线周失败时的降风险备份。
```

```text
T1  站立 · 单手 · 位置保持              [基础设施验证，原 Stage A]
  - 手末端保持世界系一个固定点；手动扰 base，看零空间自动补偿
  - done-check：base 平移 5cm 时手世界系漂移 < 1cm；errX 收敛、不发散
  - 能力点：笛卡尔 errX + J_hd.topRows(3) 接对、复用 slot 生效

T2  站立 · 单手 · 世界系轨迹跟踪        [时变能力，原 Stage B]
  - 手末端在世界系画圆 / 走直线
  - done-check：跟踪 10cm 半径圆，末端 RMS 误差 < 2cm
  - 能力点：时变笛卡尔跟踪、增益合适

T3  站立 · 双手 · 协同保持一个虚拟刚体   [bimanual，选做但强烈建议]
  - 左右手各一个笛卡尔任务，维持两手相对位姿不变（像双手端一个箱子）
  - done-check：扰 base 时两手【相对】距离/朝向漂移 < 2cm/5°
  - 能力点：双臂在零空间协同、多末端任务叠加——bimanual 是很强的加分词

T4  原地踏步 · 单手 · 手钉世界点     ★主交付物 [loco-manip 核心，原 Stage C]
  - walk_mpc_wbc，指令速度=0、步态激活；叠加笛卡尔手任务
  - done-check：连续踏步 ≥10 步，手世界系漂移 < 2cm，不失稳
  - 产出：踏步中身体晃、手不动的循环视频

T5  前进行走 · 单手 · 保持世界系水平姿态 ★主交付物 ["端盘子"，原 Stage C2]
  - 指令速度 0.1-0.3 m/s；手任务用世界系姿态(+体前偏移位置)
  - done-check：前进 ≥2m，手世界系 roll/pitch 偏差 < 5°，不失稳
  - 产出："端盘子行走"视频

+X  抗扰增强（任选一级叠加，锦上添花，非必做）
  - 在 T1/T3/T4 任一级，用 MuJoCo xfrc_applied 给手末端施一个脉冲力/持续负载
  - done-check：施力后末端偏离 → 零空间自动拉回，腿不失稳
  - 能力点：末端受扰的全身恢复——把"零空间补偿"讲成动态的、有说服力
```

**深度 vs 广度的配比（重心在行走）：**

```text
必做且做深：T4、T5（+ 各自的 Stage D 量化）——这是简历最亮、最贴岗位的部分
必做但快速跨过：T1、T2（基础设施，几乎稳成，是 T4/T5 的地基）
选做（有余力才做）：T3 双手、+X 抗扰——广度加分项，做了叙事更厚，不做不伤主线
展示方式：一个 repo、一套代码、一段"能力总览"视频（T1-T5 各一个片段）、统一 Stage D 分析
```

Stage D  量化分析（把能力套件升级为"项目"）  ★ 和 T4/T5 一起构成完整项目
  - 末端跟踪误差 vs 踏步幅度 / 前进速度
  - 行走稳定性对比：有/无手任务时的 ZMP、双脚接触力 Fz 波动（证明没破坏步态）
  - 优先级消融：把手任务优先级【提到腿之上】会怎样（预期：破坏接触/失稳）
    → 用反例证明"为什么手任务必须放零空间"
```

**"完整项目" = T4/T5 演示（+可选 T3/抗扰）+ Stage D 分析。T1/T2 是地基，快速跨过。**

### 2.4 风险与 Plan B（诚实）

```text
风险已大幅下降（复核后）：手雅可比/FK/dJ 全现成，优先级天然正确，是两级 WBC 的
  运动学级插入——比原以为的"从零接笛卡尔任务"轻。主要工作量在调增益和验证。
  能力阶梯还自带降风险：即便 T4/T5 卡住，T1-T3 站立类几乎稳成，已能构成一个
  "世界系笛卡尔任务层 + bimanual"的可展示项目（只是少了 locomotion 那一亮点）。

Plan B（触发条件：Week 3 结束仍卡在 T4，笛卡尔手任务一进 walk 就失稳且调不动）：
  降级为【关节空间 loco-manipulation】：保留 HandTrackJoints 关节空间，但把摆臂
  target_arm_q 改成一个可见的、非摆臂的定姿/挥手/举物动作，重点转到 Stage D 量化。
  - 简历故事弱一档（关节空间 vs 笛卡尔），但仍有"改动 WBC 任务 + 量化行走稳定性"实证。
  - 注意：即便走 Plan B，T1-T3（站立笛卡尔 + bimanual）仍可正常交付，项目不空。

Plan C（Plan B 也卡住，Week 4 兜底）：
  退回 push recovery：MuJoCo xfrc_applied 给 base 施扰，展示 MPC+WBC 稳住，
  量化最大可恢复冲量。工作量小、成功率高、视频效果好，但离目标岗位更远，是最后兜底。
```

### 2.5 简历一句话（T4/T5 + Stage D 达成后）

```text
"在 OpenLoong 人形两级 WBC（运动学优先级 + 力矩 QP）控制栈中，新增世界系笛卡尔
 手部末端任务层（复用零空间最低优先级层），构建了从单手位置保持、世界系轨迹跟踪、
 双手协同到 locomotion 中末端稳定跟踪的能力套件（loco-manipulation）：踏步/行走中
 手末端在世界系稳定跟踪、上肢操作与下肢步态解耦、腿部不失稳；量化了末端跟踪精度、
 行走接触力波动，并用优先级消融验证零空间设计的必要性。"
```

---

## 3. 第二项目：以 unitree_rl_gym 为基座，端到端走完 RL 运动控制全流程（Week 6-8）

第二项目的价值【不是"训出一个会走的策略"，而是"我完整走过一遍 RL 运动控制全流程"】。
招 RL 岗真正想看的，是你理解并动手做过每一环——环境/任务、奖励、域随机化、训练、
评估、迁移(sim2sim)、部署式推理——而不是跑通一个官方 demo 就交差。

基座已定：**unitree_rl_gym（Unitree 官方，github.com/unitreerobotics/unitree_rl_gym）**，
用其自带人形（G1 / H1 / H1_2）locomotion 任务做改动，人形收敛不了可在【同仓】退四足（Go2）。
选它而非 Isaac Lab 的理由：结构轻、代码读得透（legged_gym + rsl_rl），
且【七环逐环都有现成落点】，尤其 sim2sim 是现成的 130 行脚本，不用自写——见 3.2。
代价：它绑 Isaac Gym（Preview 版，挑 Ubuntu+CUDA 版本），装环境是一次性门槛。

> 基座核查（2026-07-02，本机浅克隆 depth=1 实读，非联网臆测）：
> envs/{g1,h1,h1_2,go2} 人形四足同仓；legged_robot.py 有 ~25 个 `_reward_*` 函数；
> config 有 randomize_friction / randomize_base_mass / push_robots 三个 DR 开关；
> scripts/{train,play}.py；deploy/deploy_mujoco/deploy_mujoco.py（130 行 sim2sim）
> + deploy/deploy_real（含 G1 的 C++）+ deploy/pre_train 预训练权重。

### 3.1 判据的转变（这决定第二项目怎么算"做成了"）

```text
旧判据（已废弃）：与主项目"同构对照"像不像、是否字面同一个机器人。
新判据（现行）：RL 运动控制全流程是否每一环都走到、动过手、讲得清。

  → 换 unitree_rl_gym 后，sim2sim 从"弹性可砍"升为【必做且能逐行讲】：
    因为它是仓库现成的 130 行 deploy_mujoco.py，成本极低、收益极高
    （最能体现"训练→部署解耦"，也是多数人会跳过的一段）。
    只有在 Week 7 严重超期时，才降级为"读懂并讲清"而非"自己跑通"。
```

### 3.2 RL 运动控制全流程（第二项目的骨架 = 逐环覆盖，附 unitree_rl_gym 落点）

```text
这七环就是第二项目的主线，每一环都要"动过手 + 能讲"，而不是用默认值跑过去。
右列是本机核查确认的现成落点，起步即有抓手：

  ① 环境 / 任务    envs/{g1,h1,h1_2,go2}；看懂 obs/action/episode，改一处(如指令速度范围)
  ② 奖励设计       legged_robot.py 的 ~25 个 _reward_*（tracking_lin_vel / feet_air_time 等），
                   动手改一项并观察后果（这是"设计过奖励"的证据）
  ③ 域随机化 DR    config 里 randomize_friction / randomize_base_mass / push_robots 三开关，
                   开关做对照，为⑤的消融埋点
  ④ 训练 PPO       scripts/train.py（rsl_rl 的 PPO），双 3090 起训，读懂关键超参，采训练曲线
  ⑤ 评估           scripts/play.py 回放；指令速度跟踪误差 / 成功率 / 抗扰；出曲线 + 视频
  ⑥ 迁移 sim2sim   deploy/deploy_mujoco/deploy_mujoco.py（现成 130 行）：
                   Isaac Gym 训的策略权重 → MuJoCo 里跑，体现"训练与部署解耦"★必做
  ⑦ 部署式推理     deploy_mujoco.py + deploy_real：obs 装配 / action 缩放 / 控制频率对齐，
                   把"策略怎么真正被调用"讲清楚（有真机 C++ 接口可指着讲）

交付物 = 这条链的证据链：改过的 env/reward 片段 + 训练曲线 + 评估曲线 + 行走视频
        + sim2sim(MuJoCo)回放视频 + 一份"全流程七环我各做了什么"的 README。
```

### 3.2b 基座选择（2026-07-02 核实：本地浅克隆 6 个前沿仓库逐一读码）

```text
筛选条件（用户拍板）：代码 + 数据都必须开源、可获取。
这个条件是决定性的——它直接淘汰了"前沿 motion-tracking"那一类：

  ASAP(RSS2025) / HOVER / ProtoMotions-mimic：代码开源，但训练数据【被 gate】——
    依赖 AMASS 数据集（需学术注册、license 禁止再分发）+ SMPL 人体模型（需注册）
    + SMPL 拟合/重定向管线。→ 数据不满足"开源可获取"，淘汰为基座。
    （ASAP 自带一小份 TairanTestbed 重定向 G1 动作，但其通用管线与前沿叙事仍靠 AMASS。）

  locomotion RL（HumanoidVerse-locomotion / unitree_rl_gym）：【完全无动作数据集】——
    靠 reward 函数 + 随机速度指令 + 程序化地形训练，唯一"数据"是仓库自带的
    机器人 URDF/MJCF（permissive license）。→ 代码+数据全开源，通过筛选。

决策：基座 = HumanoidVerse（locomotion 任务）。三个约束（开源数据 / 弱 RL / 前沿）在此收敛：
  ① 代码+数据全开源：reward 驱动，无 AMASS/SMPL，机器人资产随仓库自带。
  ② 前沿可信：它就是 CMU LeCAR-Lab 的框架，RSS2025 的 ASAP 正是【构建在其上】
     （同一个 humanoidverse/ 包）。简历写"基于 ASAP/HumanoidVerse 框架"属实。
  ③ 全流程 + 弱RL友好：envs/locomotion + agents/ppo + DR(承自 legged_gym) +
     train_agent.py/eval_agent.py + 【多仿真器 = 天然 sim2sim】(改一个 +simulator= 标志
     即可 IsaacGym→Genesis→IsaacSim)，且无 teacher-student/蒸馏，读得透。
  回退基座 = unitree_rl_gym（同样全开源、自带 sim2sim→MuJoCo、同为 G1/H1/Go2 人形）。
```

### 3.3 与主项目的关系（对照叙事保留，但降为"加分项"不是"主卖点")

```text
两个项目仍能讲同一个故事的两面，面试时作为加分叙事：
  - 主项目（model-based / WBC）：manipulation 需要显式任务/优先级，这是 model-based 主场
  - 第二项目（learning / RL）：locomotion 的抗扰、地形泛化，是 learning 主场
  → "同一类人形运动控制，我既走过 model-based 全身控制，也走过 RL 全流程，清楚各自边界。"
  额外加分：HumanoidVerse 支持 G1/H1 人形，与主项目 OpenLoong 同为人形 locomotion 问题域。

但第二项目【首要卖点是"RL 全流程覆盖 + 前沿框架"】，对照只是顺带成立的加分，不增工程负担。
```

### 3.4 技术路径与产出

```text
- 工具：HumanoidVerse（legged_gym 衍生 + PPO + 多仿真器）。主训用 Isaac Gym Preview——
        Week 6 第一件事就是【把环境装起来 + 跑通官方 locomotion 最小示例
        (+exp=locomotion +rewards=loco/reward_h1_locomotion)】，先确认全链路能通，再动手改。
        装不起来的兜底见 3.5。
- 双 3090 可用；【人形比四足慢，预留更多训练时间，不是几小时级】，别按四足预期排期。
- 每一环都留下"我动过手"的痕迹：改过的配置/代码片段、开关 DR 的对照、超参说明。
- 产出：改动片段 + 训练曲线（reward / tracking error vs steps）+ 评估曲线 + 行走视频
        + 【sim2sim 跨仿真器回放：同一策略 IsaacGym 训→Genesis/IsaacSim 评】+ 全流程 README。
- 重点是"讲清全流程"：observation/action 怎么定义、reward 各项为何这么设、
  DR 为何对鲁棒性/sim2real 重要、PPO 关键超参、训练与部署如何解耦。
- 【前沿钩子（不需 gated 数据）】：locomotion 跑通后，做一个"跨仿真器 sim2sim 鲁棒性"研究
  （IsaacGym→Genesis 策略迁移掉多少），这正是 ASAP"对齐仿真与现实物理"主题的开源可做版；
  并可【读 + 讲清 ASAP 的 delta-action-model 思想】作为前沿延伸（读代码即可，不必复现其数据管线）。
```

### 3.5 风险与回退（判据是"全流程闭环"，所以回退不砍环、只降难度）

```text
风险：① 人形 RL 比四足难、你 RL 基础弱、3 周窗口紧；② Isaac Gym Preview 环境挑版本、可能装不起来。

时间盒 + 分级回退（触发点明确，不拖到 Week 8）：
  【Week 6 前半】Isaac Gym 环境装不起来（版本/驱动冲突不可解）
    → 回退⓪-a：HumanoidVerse 本身支持 Genesis/IsaacSim，换一个后端跑 locomotion（改 +simulator=）；
      → 回退⓪-b：整体换基座到 unitree_rl_gym（自带 sim2sim→MuJoCo，全开源），全流程同样能走。
  【Week 6 结束】人形策略仍无法收敛到"能走"
    → 回退①：同框架换四足（Go2）任务，七环流程照走全（判据是流程，不是人形）。
  【Week 7 中】若跨仿真器 sim2sim 真跑不通（多是 obs 对齐/坐标系问题）
    → 回退②：⑥环降为"读懂并逐行讲清多仿真器切换 + 部署推理接口"，用⑦顶替，全流程叙事仍闭环。
  三级回退都【保住"全流程走全"这个核心判据】，只降机器人/环境/迁移的难度。
```

### 3.6 简历一句话

```text
达成（人形，全流程）：
  "基于 unitree_rl_gym（Unitree G1/H1）端到端实践 RL 运动控制全流程：环境/奖励改造、
   域随机化、PPO 训练、评估、sim2sim（Isaac Gym→MuJoCo）迁移与部署式推理，
   理解 reward shaping 与 domain randomization 对策略鲁棒性的影响。"

回退（四足，全流程）：
  "基于 unitree_rl_gym（Unitree Go2）完整走通 RL 运动控制流程（环境/奖励/DR/PPO 训练/
   评估/sim2sim/部署推理），并与人形 model-based WBC 项目形成范式对照。"
```

---

## 4. 8 周周历（错峰并行，deliverable 拆解）

### 4.0 并行原则（先定规则，再看排期）

```text
目的：不压缩 8 周总时长，用并行换【从容 + 风险前置】。里程碑不变——
     主项目仍 Week 1-5 交付，RL 出成果仍 Week 6-7。

错峰并行的三条规则（守住它们，并行才是收益不是负担）：
  1. 主项目 = 主线，占据你【坐下来连续写代码】的专注时间块。
  2. 第二项目在 Week 1-5 只做【低专注 / 挂机型 / 阅读型】任务，
     填充主项目的等待时间（Docker build、编译、GPU 挂机训练）。
     真正动脑的七环改造（改 reward、DR 消融、sim2sim 讲透）仍留在 Week 6-8。
  3. 【Week 3 红线】主项目 Stage C 攻坚周，副线只允许挂机训练，
     不启动任何需要专注的新东西，避免两头打架。

为什么错峰并行比串行更好（这是选它的理由）：
  - 风险前置：Isaac Gym 能否装起来是第二项目最大未知。串行拖到 Week 6 才暴露，
    装不上就三周全废；并行让它 Week 1 就开始趟（且装环境用碎片时间即可）。
  - GPU 不空转：串行时 Week 1-5 双 3090 全闲；并行让它 Week 2 就跑起来。
  - 天然错峰：RL 挂机 ↔ 主项目编码，对"人的专注"需求刚好互补。
```

### 4.1 逐周排期（主线 ‖ 副线）

```text
Week 1  主线：T1（+ 顺带 T2 起步）
        - 读透 HandTrackJoints 调用链：q 手臂索引 block(7)、J_hd_* 来自 pino_kin_dyn、
          它已是 walk 最低优先级（天然零空间）
        - 实现笛卡尔任务（右手仅位置，J = J_hd_r.topRows(3)），站立手钉世界固定点，
          base 扰动自动补偿；done-check：base 平移5cm 时手漂移<1cm
        ‖ 副线（碎片时间，风险前置）：装 Isaac Gym + clone unitree_rl_gym
          —— 这是第二项目最大未知，最先趟；装不上早知道早想办法
        deliverable：T1 站立手钉固定点、扰 base 手不动的自检视频
                   ‖（副）Isaac Gym 环境跑起来、import 不报错

Week 2  主线：T2 + 建仓（有余力起步 T3 双手）
        - 手在世界系画圆/直线，时变跟踪跑稳；建主项目 GitHub repo + README 骨架
        ‖ 副线（挂机型）：跑通 g1/h1 官方 train.py，起一次训练看能否收敛
          —— 验证整条工具链通，GPU 开始运转（挂机，不占专注）
        deliverable：站立时变笛卡尔跟踪的误差曲线
                   ‖（副）官方范例能训、loss 下降、能 play.py 回放

Week 3  主线：T4（核心，攻坚周）★红线周
        - walk_mpc_wbc 指令速度=0 起步（原地踏步）叠加笛卡尔手任务，复用 HandTrackJoints slot
        - done-check：连续踏步≥10步，手世界系漂移<2cm，不失稳；顺利则试 T5 前进+姿态
        - 周末仍卡住 → 评估切 Plan B（关节空间定姿/挥手）；T1-T3 已可保底成项目
        ‖ 副线：【只挂机，不启新事】让上周的训练继续跑/复现，专注全给主线
        deliverable：T4 原地踏步 + 手钉世界固定点的初步视频（哪怕不完美）

Week 4  主线：T4 打磨 + T5 前进"端盘子" + Stage D 起步
        - 稳住 T4；做 T5（前进+世界系姿态）；开始采稳定性证据（ZMP/接触力）；录正式演示
        - 有余力：T3 双手协同 / +X 末端抗扰（选做，广度加分）
        ‖ 副线（阅读型，利用训练挂机时间）：环①读懂 envs/g1 的 obs/action/episode
        deliverable：T4+T5 演示视频 + 首批稳定性曲线
                   ‖（副）环①笔记：obs/action/episode 结构清楚

Week 5  主线：Stage D 收尾 + "能力总览"视频 + 交付
        - 跟踪误差 vs 速度、有/无手任务稳定性对比、优先级消融
        - 剪一段 T1-T5 能力总览视频；README 精修（GIF 置顶、指标写清）；
          技术文档（笛卡尔任务层设计、零空间/优先级排序理由）
        ‖ 副线（阅读型）：环②读懂 legged_robot.py 里 ~25 个 _reward_* 各项含义
        deliverable：主项目 GitHub 完整交付（能力套件 demo + 分析 + 文档）★主项目收官
                   ‖（副）环②笔记：reward 各项含义 + 打算改哪一项

Week 6  全力副线：环①②③改造 + 环④起训（专注转移到第二项目）
        - 环①：改一处（如指令速度范围）  环②：动手改一项 reward 看后果
        - 环③：开关 randomize_friction / base_mass / push_robots，为消融埋点
        - 环④：起训 PPO（rsl_rl），采训练曲线（此前已装好环境、跑通范例，直接进改造）
        ★ 时间盒关卡：周末人形仍无法收敛 → 同仓换 go2 四足，七环照走全
        deliverable：改过的 env/reward 片段 + 训练曲线 + 能回放策略

Week 7  副线出成果：环④收敛 + 环⑤评估 + 环⑥ sim2sim
        - 训到稳定行走策略（人形或回退四足），play.py 录行走视频
        - 环⑤评估：速度跟踪误差 / 成功率 / 抗扰；出评估曲线
        - DR 开/关 鲁棒性消融（呼应"DR 为何重要"）
        - 环⑥ sim2sim：跑 deploy/deploy_mujoco/deploy_mujoco.py（现成 130 行），
          Isaac Gym 训 → MuJoCo 部署，读透 obs 装配/action 缩放/控制频率
        deliverable：训练曲线 + 评估曲线 + DR 消融 + 行走视频 + sim2sim 回放

Week 8  副线收尾（环⑦ + 全流程 README）+ 全局面试准备
        - 环⑦：读 deploy/deploy_real 接口，讲清"策略如何被真机调用"（与 sim2sim 对照）
        - 写"全流程七环我各做了什么"的 README（证据链齐全）
        - 打磨双项目共同叙事（加分项）：locomotion 用 RL、manipulation 用 model-based
        - logbook 阅读笔记整理成"能白板推导"的清单
        deliverable：两个项目 GitHub 齐全 + 全流程 README + 面试话术清单
```

> 因为环境（Week 1）、工具链（Week 2）、环①②阅读（Week 4-5）都已在主项目期间用
> 碎片/挂机时间铺完，Week 6 一开始就能直接进"改造 + 起训"，不再花时间在装环境和
> 跑通范例上——这就是风险前置换来的从容：Week 6-8 三周纯用于七环的动手与讲透。

---

## 5. GitHub / 简历呈现要点

```text
- 主项目一个独立 repo；第二项目一个独立 repo
- README 顶部放 GIF/视频 —— 招控制的人 3 秒内要看到机器人在动
- 写清 "我做了什么 → 达到了什么指标"：
    主项目："将关节空间手任务升级为世界系笛卡尔任务；行走中末端跟踪误差 < X cm，
             接触力波动 < Y%"
    第二项目（RL 全流程）："基于 unitree_rl_gym（Unitree G1/H1）locomotion 任务，
             端到端走通 RL 运动控制全流程（环境/奖励改造 → 域随机化 → PPO 训练 →
             评估 → sim2sim（Isaac Gym→MuJoCo）→ 部署接口），每一环均动手改过并能讲清"
             （回退 Go2 四足则相应改措辞）
- 第二项目 README 的骨架 =【全流程七环，每环写清"我做了什么 + 一张图/一段证据"】，
  这比"我训了个会走的策略"更能证明你真的走过整条 RL 运动控制流水线
- logbook 的 R0-R4 阅读笔记是面试讲解素材，但简历呈现的是"项目"不是"读书笔记"
```

---

## 6. 面试话术清单（Week 8 整理，需能白板推导）

```text
必须能手推/讲清：
  - MPC：单刚体模型（SRBM）、状态 nx=12、接触力为输入、摩擦锥约束、预测时域
  - WBC：优先级任务、零空间投影 N、加权伪逆、task order 为什么这么排
  - 笛卡尔 vs 关节空间任务：手雅可比 J_hd 的作用、为什么笛卡尔任务能"钉世界点"
  - loco-manipulation：手任务如何进 WBC 栈的零空间、为什么不破坏行走
  - 接触：摩擦锥、支撑多边形、ZMP / 接触力约束
  - 状态估计：为什么行走要靠触地做零速修正
  - RL（讲清即可）：PPO 基本流程、reward 各项、domain randomization 作用
  - 范式边界：model-based vs learning-based 各自适合什么、边界在哪
```

---

## 7. 关于项目 E（AugMPC / IBRIDO）的明确决定

```text
决定：两个月内不复现项目 E。
理由：eval 路线要 build .sif + clone ~21 个仓库 + 多进程共享内存 IPC + 适配作者
      HPC 路径，投入产出比极差；跑通也只是"复现别人的 eval 回放"，故事弱。
定位：作为"我理解 RL-augmented MPC 架构（non-gaited 接触调度）"的面试谈资即可。
```

---

## 8. 下一步（立即开始 Week 1 Stage A）

```text
1. 定位 J_hd_l / J_hd_r 在 DataBus 里如何被填充（哪个模块算的、维度、坐标系），
   以及 q 里手臂关节的确切索引区间
2. 在 stand 模式新增/改造出笛卡尔手任务：errX = hd_r_pos_des_W - hd_r_pos_W，J = J_hd_r
3. 站立下让手钉住世界固定点，手动扰动 base 目标，验证手自动补偿
4. 建主项目 GitHub repo
```
