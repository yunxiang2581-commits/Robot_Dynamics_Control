# 13. 项目一编码约束（贴合 OpenLoong 原仓库风格）

> 适用范围：项目一（在 OpenLoong-Dyn-Control 上新增/修改的所有 C++ 代码）。
> 目的：让后续每一处新代码都与上游仓库风格一致，且默认行为对原仓库零污染。
> 依据：本文件规则**全部从上游真实源码归纳**（`wbc_priority.h` / `priority_tasks.h` /
> `hand_track_task.cpp` / `data_bus.h` 等），不是凭空约定。
> 状态：**硬约束**——除非明确记录例外，新代码必须遵守。

---

## 0. 一句话总纲

**新代码读起来要像上游作者写的，且不开环境变量时原仓库逐字节不变。**

---

## 1. 硬约束（MUST / MUST NOT）

### 1.1 文件与版权头

- **MUST**：每个新增 `.h` / `.cpp` 顶部放上游那段**固定 6 行版权头**（逐字节复制现有文件的头部），
  紧接 `#pragma once`（头文件）。
- **MUST**：算法模块成对出现 `xxx.h` + `xxx.cpp`，放 `algorithm/`；数学/通用工具放 `math/` 或 `common/`。
- **MUST NOT**：不在 `external/open_source_repos/OpenLoong-Dyn-Control` 之外复制源码副本；改动就地改上游树。

### 1.2 命名（跟随现有惯例，不得自创风格）

| 类别 | 惯例 | 例（源码实际存在） |
|---|---|---|
| 变量 | `snake_case` | `hd_r_pos_cur_W`、`base_pos_des`、`legStateCur` |
| 类型 / 类 | `PascalCase` 或上游既有拼写 | `DataBus`、`WBC_priority`、`Pin_KinDyn`、`PriorityTasks` |
| 函数 / 方法 | `camelCase` | `dataBusRead`、`computeTau`、`fillWalkHandTrackTask` |
| 命名空间 | `PascalCase` | `HandTrackTask` |

- **MUST**：坐标系后缀是**硬规矩**——`_W`=世界系、`_L`=体坐标系。凡是有坐标系含义的量必须带后缀
  （对照源码 `hd_r_pos_W` vs `hd_r_pos_L`）。
- **MUST**：期望/当前用 `des` / `cur`；左右用 `_l` / `_r`；时间导数用 `d` 前缀（`J` → `dJ`）。
- **MUST NOT**：不引入匈牙利命名、不引入 `m_` 成员前缀（上游没有）。

### 1.3 WBC 任务的构造方式

- **MUST**：新的运动学任务通过 `fillXxxTask(Task &task, ...)` 填充 `task` 的
  `errX / derrX / dxDes / ddxDes / J / dJ / kp / kd / W`，**复用** `priority_tasks.h` 的 `Task` 结构，
  不自建求解流程、不绕过零空间投影（对照 `hand_track_task.cpp`）。
- **MUST**：任务维度 = `errX / J.rows()` 必须一致；`J` 列数 = `model_nv`。
- **MUST**：运动学级任务 `dJ` 若无可靠时间导数就置 0，并注释说明
  （对照现状：绕开 `wbc_priority.cpp:144-145` 的 `dJ_hd_*` 误赋值 bug）。
- **MUST NOT**：不改动 `taskOrder_walk` / `taskOrder_stand` 的既有优先级顺序，除非该改动本身就是实验目的
  （手任务已在 walk 最低优先级，天然在 locomotion 零空间，复用即可）。

### 1.4 对原仓库零污染（env 门控）

- **MUST**：任何行为改动都用**环境变量门控**，变量名前缀 `OPENLOONG_`，语义化大写
  （既有：`OPENLOONG_T1_CART_HAND`、`OPENLOONG_T4_IN_PLACE`）。
- **MUST**：**未设置环境变量时，走原始分支，且输出与上游逐字节一致**（对照 `hand_track_task.cpp`
  里 `else { ...original... }` 分支）。这是可回退、可对照实验的地基，不可破坏。
- **MUST**：门控读取用 `static const bool xxx = (std::getenv("...") != nullptr);`（进程内读一次）。

### 1.5 可调参数不得裸埋（收编现有不一致）

- **MUST**：增益、权重、锁定计数、阈值等**可调量**，要么走 env helper
  （`readPositiveEnv` / `readNonNegativeEnv`，已存在于 `hand_track_task.cpp` 匿名 namespace），
  要么提为文件顶部**具名 `const`**，并在旁注默认值含义。
- **MUST NOT**：不在函数体中间散落无名魔法数字。
  - 正例（T1 & T4 均已做）：`readPositiveEnv("OPENLOONG_T1_HAND_TRAJ_AMP", 0.015)`、
    `readPositiveEnv("OPENLOONG_T4_HAND_WEIGHT", 5.0)`。
  - 反例（早期 T4，已于 2026-07-03 整改）：`t4_hand_weight = 5.0`、`kp*60`、`lock_count=3500` 曾直接硬写；
    现已全部提为 env 化具名 `const`（见 §5 旋钮清单）。

### 1.6 诊断日志

- **MUST**：诊断输出带**方括号标签**，标签体现任务与阶段（既有：`[T1-D1]`、`[T1-LOCK]`、`[T4-HOLD]`）。
- **MUST**：高频循环里的日志必须**稀疏化**（既有：`if (cnt % 500 == 0)`），避免刷屏。
- **MUST**：done-check 量以可解析格式打印（既有：`drift_mm=<val>`），便于离线脚本 grep。
- **SHOULD**：正式录制/交付版本移除或关闭临时诊断（用同一 env 或独立开关）。

### 1.7 注释

- **MUST**：涉及索引 / 坐标系 / 依赖行的地方，注释标**来源 file:line**
  （既有：`// world frame, from dataBusRead :115`）。
- **MUST**：`else` 原始分支旁标注 `// ---- original ... (unchanged) ----`，明示这是上游行为。
- **SHOULD**：注释用英文（与上游一致）；中文说明放 logbook，不放源码。

---

## 1.8 环境变量登记表（新增门控/旋钮必须登记在此）

> 每加一个 `OPENLOONG_*` 变量，都要在这里补一行。默认值必须等于"已验证运行"的值，
> 保证不设变量时行为与通过 done-check 的那次逐字节一致。

**门控开关（存在即启用）：**

| 变量 | 作用 | 归属 |
| --- | --- | --- |
| `OPENLOONG_T1_STAND_ONLY` | 禁止 walk_wbc 3s 后进入步态，验证 stand 分支 | `demo/walk_wbc.cpp` |
| `OPENLOONG_T1_CART_HAND` | stand 分支启用右手世界系笛卡尔位置保持 | `hand_track_task.cpp` |
| `OPENLOONG_T4_IN_PLACE` | 保留步态但令 `xv_des=0`（原地踏步），并锁 base 锚点 | `demo/walk_wbc.cpp` |
| `OPENLOONG_T4_CART_HAND` | walk 分支启用右手世界系笛卡尔位置保持 | `hand_track_task.cpp` |

**T1 可调旋钮（`readPositiveEnv` / `readNonNegativeEnv`，默认=已验证值）：**

| 变量 | 默认 | 含义 |
| --- | --- | --- |
| `OPENLOONG_T1_HAND_TRAJ` | NONE | 轨迹模式 LINE/CIRCLE/SINE |
| `OPENLOONG_T1_HAND_TRAJ_AMP` | 0.015 | 轨迹幅值 (m) |
| `OPENLOONG_T1_HAND_TRAJ_PERIOD` | 4.0 | 轨迹周期 (s) |
| `OPENLOONG_T1_HAND_TRAJ_SMOOTH_TIME` | 1.5 | 平滑启动时长 (s) |

**T4 可调旋钮（2026-07-03 整改新增，默认=已验证的 T4-D1 值）：**

| 变量 | 默认 | 含义 |
| --- | --- | --- |
| `OPENLOONG_T4_LOCK_COUNT` | 3500 | 目标锁定前的 WBC 周期数 |
| `OPENLOONG_T4_HAND_WEIGHT` | 5.0 | 手位置行权重 |
| `OPENLOONG_T4_RIGHT_ARM_WEIGHT` | 0.1 | 右臂正则行权重 |
| `OPENLOONG_T4_LEFT_ARM_WEIGHT` | 1.0 | 左臂保持行权重 |
| `OPENLOONG_T4_HAND_KP` / `_KD` | 60 / 6 | 手位置 PD 增益 |
| `OPENLOONG_T4_RIGHT_ARM_KP` / `_KD` | 10 / 1 | 右臂 PD 增益 |
| `OPENLOONG_T4_LEFT_ARM_KP` / `_KD` | 120 / 6 | 左臂 PD 增益 |

> ⚠️ 这些旋钮用 `readPositiveEnv`，**值必须 > 0**，不能设 0。若阶段 4 消融需要把某权重
> 归零（如关掉右臂正则），改用 `readNonNegativeEnv` 重读该变量。

---

## 2. 已知例外 / 技术债（记录在案，不算违规但要清楚）

- **静态局部状态**：`static bool ..._locked` / `static Eigen::Vector3d ..._des_W` 等用于跨周期保持，
  **仅在单实例 demo 下安全**。若将来一个进程跑多个控制器实例，必须改成 `WBC_priority` 成员变量。
- **T4 完备度仍落后于 T1（部分整改）**：
  - 已整改（2026-07-03）：env 化参数——权重/增益/锁定计数全部提为 `readPositiveEnv` 具名 const（见 §5）。
  - 仍缺：五次多项式平滑启动包络（`applySmoothStartEnvelope`）、轨迹模式。做 T5/上楼梯类任务时向 T1 看齐补上。
- **手 frame = 腕部关节 `J_arm_r_07`，非指尖**（`pino_kin_dyn.cpp:45`）。所有"手位置"实为腕位置，
  交付措辞需诚实。

---

## 3. 新增一个 WBC 末端任务的标准步骤清单

> 照这个顺序做，产出就会自动符合上面的硬约束。

1. **确认数据源**：目标量在 `DataBus` 里是否已有（`hd_*` / `J_hd_*` / `dJ_hd_*`）；
   没有就先在 `pino_kin_dyn.cpp` 里算好并 `dataBusWrite`，再在 `wbc_priority` 的 `dataBusRead` 读入。
   注释标出对应 file:line。
2. **决定模块归属**：新任务逻辑写进对应 `algorithm/xxx_task.cpp`（仿 `hand_track_task.cpp`），
   `namespace` 包裹，内部 helper 放匿名 `namespace {}`。不要把大段逻辑塞回 `wbc_priority.cpp`。
3. **加 env 门控**：`OPENLOONG_<TASK>_<VARIANT>`；写 `else` 原始分支，保证默认逐字节不变。
4. **填 `Task`**：设 `errX / J`（维度一致、`J` 列数=`model_nv`）、`kp / kd`（分块设增益）、
   `dJ`（无则置 0 并注释）、`W`。可调参数走 env helper 或具名 const。
5. **清浮动基列（按需）**：末端任务若不应驱动 base，`J.block(0,0,rows,6).setZero()`（仿现有手任务）。
6. **加诊断**：`[<TASK>-<STAGE>]` 标签 + `% N` 稀疏打印 + 可 grep 的 done-check 量。
7. **验证顺序**：先编译（维度不匹配会当场报错）→ 短 smoke → 可视长跑（暴露后期失稳，
   短 smoke 不够，这是 T1 的教训）→ 记 done-check 数值。
8. **写 logbook**：`logbook/daily/project1_loco_manip/<date>.md`，记"改了什么 / done-check 结果 /
   下一步"，源码里不留中文长注释。

---

## 4. 快速自检（提交前过一遍）

- [ ] 新文件有 6 行版权头 + `#pragma once`？
- [ ] 命名 snake/camel/Pascal 与上游一致？坐标系后缀 `_W`/`_L` 齐全？
- [ ] 不开 env 时，原 demo 行为逐字节不变？
- [ ] 可调参数没有裸魔法数字（走 env helper 或具名 const）？
- [ ] 日志带标签、稀疏化、done-check 量可 grep？
- [ ] 索引/坐标系/依赖处注释标了 file:line？
- [ ] 已编译 + 可视长跑验证，不是只写完？
- [ ] logbook 记录了 done-check 数值？
