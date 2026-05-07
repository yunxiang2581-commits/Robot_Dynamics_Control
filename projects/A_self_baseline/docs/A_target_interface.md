# A Target Interface

## 1. 定位

Target interface 回答一个问题：机器人应该去哪里。

它不求 IK，不控制 actuator，不启动 viewer。它只把 fixed target、pose sequence、mocap placeholder 或 future viewer target 统一成 `TargetDefinition`。

## 2. TargetDefinition

| 字段 | 含义 | 维度 | 单位 |
|---|---|---|---|
| target_id | 目标编号 | 标量 | - |
| source | 目标来源 | 标量 | - |
| target_site | 末端 site 名称 | 标量 | - |
| target_body | 可选 body 名称 | 标量 | - |
| coordinate_frame | 坐标系 | 标量 | - |
| position | 目标位置 | (3,) | m |
| rotation_matrix | 目标姿态 | (3,3) | - |
| quat_wxyz | 目标四元数 | (4,) | - |
| position_offset | 相对当前位置偏移 | (3,) | m |
| orientation_mode | 姿态模式 | 标量 | - |

## 3. 数学关系

固定目标第一版：

```text
p_target = p_current + offset
R_target = R_current
```

姿态误差后续统一写成：

```text
R_err = R_target R_current^T
e_rot = log(R_err)
```

RPY 只作为输入接口，不作为误差计算主表示。

## 4. 与 mink 的关系

mink 示例中 target 常来自 viewer 或 mocap body。A 项目不调用 mink，而是把这些 target 来源转成自己的 `TargetDefinition`。

## 5. 验证标准

- `position.shape == (3,)`。
- `quat_wxyz.shape == (4,)`。
- `coordinate_frame` 明确。
- fixed target 的 `target-current` 与 offset 一致。
- A06 生成 JSON，A05 后续能读取。

## 6. 常见错误

- 坐标系混用。
- quaternion `wxyz` / `xyzw` 顺序混淆。
- 直接用 RPY 做误差。
- 忘记记录 target 来源。
- 在 Target interface 中求 IK 或写 `data.ctrl`。
