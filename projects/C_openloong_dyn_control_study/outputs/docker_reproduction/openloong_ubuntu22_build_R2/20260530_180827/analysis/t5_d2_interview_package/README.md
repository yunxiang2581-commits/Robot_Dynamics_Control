# T5-D2 面试成果包：原地踏步 + 右手 6-DoF 位姿保持

## 一句话结果

在 OpenLoong `walk_wbc` 中，把右手任务从“关节空间摆臂”扩展为“世界系 6-DoF 位姿保持”。在原地踏步稳态阶段，右手位置误差最大 `14.5 mm`，姿态误差最大 `2.7 deg`，同时机身保持稳定。

## 问题背景

原始 walk 分支里的 `HandTrackJoints` 是关节空间任务，主要用于正常行走时的手臂摆动姿态。它可以让手臂看起来协调，但不能表达“端盘子”这类要求：右手不仅要保持空间位置，还要保持掌心姿态。

真正的难点不只是加一个 6-DoF 任务。前平举掌心向上的初始化单独打开时，机器人也会在进入踏步后失稳。原因是 walk 分支仍会把手臂拉回旧的摆臂目标，前平举姿态在切步时被突然改写。

## 实现位置

核心源码：

- `external/open_source_repos/OpenLoong-Dyn-Control/algorithm/hand_track_task.h`
- `external/open_source_repos/OpenLoong-Dyn-Control/algorithm/hand_track_task.cpp`
- `external/open_source_repos/OpenLoong-Dyn-Control/algorithm/wbc_priority.cpp`
- `external/open_source_repos/OpenLoong-Dyn-Control/demo/walk_wbc.cpp`

主要环境变量：

- `OPENLOONG_T4_IN_PLACE=1`：原地踏步，沿用 T4 的 base anchor 工况。
- `OPENLOONG_T5_FORWARD_PALM_UP=1`：右手初始化为前平举、掌心向上。
- `OPENLOONG_T5_POSTURE=1`：启用右手 6-DoF 位姿任务。
- `OPENLOONG_T5_WALK_LOCK_COUNT=1000`：T5-D2 默认右手目标锁定拍数。

## 任务结构

`HandTrackJoints` 在 T5-D2 下变成 20 维任务：

- 行 `0..2`：右手世界系位置误差。
- 行 `3..5`：右手姿态误差，使用 SO(3) log map。
- 行 `6..12`：右臂弱关节正则。
- 行 `13..19`：左臂姿态保持。

右手位姿行会清零 floating-base 列，使任务主要通过手臂自由度完成，而不是让 base 为了手部任务移动。

## 关键修复

最初摔倒不是 SO(3) 姿态误差公式导致的，而是 walk 分支仍在驱动旧摆臂目标。修复方式是在 `OPENLOONG_T5_FORWARD_PALM_UP=1` 时锁定当前 14-DoF 手臂姿态，日志标记为：

```text
[T5-WALK-ARM-LOCK]
```

这样进入 walk 分支后，手臂不会被拉回旧摆臂姿态，右手 6-DoF 位姿保持任务才有稳定工作的基础。

## 验证命令

```bash
OPENLOONG_T4_IN_PLACE=1 \
OPENLOONG_T5_POSTURE=1 \
OPENLOONG_T5_FORWARD_PALM_UP=1 \
timeout 70 xvfb-run -a -s "-screen 0 1280x720x24" ./walk_wbc
```

主日志：

```text
logs/20260704_t5_d2_walk6d_inplace_wrist112_smoke70.log
```

分析产物：

```text
analysis/t5_d2_walk6d_inplace_wrist112/t5_d2_walk_posture_summary.txt
analysis/t5_d2_walk6d_inplace_wrist112/t5_d2_walk_posture_samples.csv
analysis/t5_d2_walk6d_inplace_wrist112/figures/t5_d2_walk_posture_errors.png
analysis/t5_d2_walk6d_inplace_wrist112/figures/t5_d2_walk_posture_errors.pdf
```

## 关键指标

```text
locked_count=19
all_pos_max_mm=34.758
all_rot_max_deg=3.360
steady_count=12
steady_pos_avg_mm=10.496
steady_pos_max_mm=14.523
steady_pos_tail_mm=13.177
steady_rot_avg_deg=1.708
steady_rot_max_deg=2.744
steady_rot_tail_deg=1.578
done_check_result=PASS_STEADY_STATE
```

解释：

- `3.5s` 附近的 `34.758 mm` 是 gait handoff 瞬态峰值。
- `4.5s` 后进入稳态窗口，位置最大 `14.523 mm < 20 mm`，姿态最大 `2.744 deg < 5 deg`。
- WSLg 前台可视仿真也跑到 30s，末尾手部误差仍在阈值内，base 高度稳定。

## 1 分钟面试讲法

我把原来的手臂关节空间任务扩展成了一个 gated 的右手 6-DoF 笛卡尔位姿任务。位置部分使用右手线速度雅可比，姿态部分使用同一个 Pinocchio `LOCAL_WORLD_ALIGNED` 雅可比的角速度行。姿态误差用 SO(3) log map 表示，所以误差向量和 world-aligned 角雅可比处在同一个切空间里。

调试里最关键的一步，是发现机器人即使不开 6-DoF 姿态任务，只要开启前平举掌心向上初始化，也会在 walk 分支失稳。这说明根因不是姿态控制器本身，而是 walk 分支旧的摆臂目标在切步时覆盖了前平举手型。我先加了一个只在 forward-palm-up 模式下启用的手臂姿态锁定，再启用 6-DoF 位姿保持。最终结果是在原地踏步稳态阶段通过 `20 mm / 5 deg` 判据。
