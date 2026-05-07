# A Viewer Interface

## 1. 三个核心概念

viewer target:
在 viewer 中可视化或交互移动的任务目标。

mocap-style target:
用 MuJoCo mocap body 表示可移动目标，其 pose 来自 `data.mocap_pos` / `data.mocap_quat`。

target pose feeds IK:
target pose 是 IK 的任务输入，不是 actuator control 输出。

## 2. 为什么需要 Viewer interface

fixed target 适合可复现实验；viewer target 适合演示目标如何变化。mink 示例中 target 可以通过 viewer / mocap 交互改变。A 项目学习这个数据流，但不直接调用 mink。

## 3. mocap body 条件

未来派生 MJCF 中的 mocap body 应满足：

- 是 world child。
- 没有 joints。
- `mocap="true"`。
- 可用 sphere geom/site 可视化。
- 不参与碰撞。

当前 Step R-C 不创建派生 MJCF。

## 4. model.nmocap 与 data shape

```text
data.mocap_pos.shape = (model.nmocap, 3)
data.mocap_quat.shape = (model.nmocap, 4)
```

如果 `model.nmocap == 0`，不能访问 `data.mocap_pos[0]`。

## 5. 鼠标与键盘规划

鼠标拖动：

- 依赖 MuJoCo viewer 对 mocap body 的 perturbation。
- 每帧 `viewer.sync()` 后读取 target pose。
- 不自己实现 ray casting。

键盘移动：

- `key_callback` 记录按键。
- 主循环消费 pending delta。
- W/S 控制 X，A/D 控制 Y，Q/E 控制 Z。
- 修改 data 时使用 `viewer.lock()`。

## 6. 数学关系

```text
e_pos = p_target - p_current
dq = IK(e_pos, J_task, limits)
q_next = integrate(q, dq, dt)
```

这是 kinematic IK follow，不是 actuator tracking。A06 不写 `data.ctrl`，A07 才写。

## 7. 常见错误

- `model.nmocap == 0` 时强行访问 mocap arrays。
- 把 mocap target 当成 `data.ctrl`。
- 忘记 `viewer.sync()`。
- 不加 `viewer.lock()` 就修改 data。
- quaternion 顺序混淆。
- 在 A06 中实现 actuator tracking。
- 修改原始 scene.xml。
- 调用 mink 替代自己的实现。
