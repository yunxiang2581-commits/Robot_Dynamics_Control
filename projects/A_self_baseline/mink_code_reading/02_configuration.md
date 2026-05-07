# 02 - Configuration 源码拆解

## 本节目标

阅读 `external/mink_upstream/src/mink/configuration.py`。

`Configuration` 是 mink 的状态中心。它把 MuJoCo 的 `model`、`data` 和当前广义坐标 `q` 包装起来，让 task 和 solver 可以统一查询当前 pose、Jacobian，并更新状态。

## 对应源码

- `external/mink_upstream/src/mink/configuration.py`
- 可辅助阅读: `external/mink_upstream/tests/test_configuration.py`

## 为什么需要 Configuration

在机器人 IK 中，每一轮都需要：

1. 当前关节状态 `q`。
2. 根据 `q` 更新 forward kinematics。
3. 查询末端 frame/site 当前位姿。
4. 查询对应 Jacobian。
5. 根据 `dq` 积分得到新的 `q`。

如果每个 task 都自己管理这些步骤，代码会重复且容易不一致。`Configuration` 的价值就是把这些状态更新和查询统一起来。

## 建议阅读函数 / 方法

打开 `configuration.py` 后，优先找这些内容：

- 构造函数：model、data、q 如何初始化。
- `update` 或类似方法：如何把 q 写进 MuJoCo data 并刷新 kinematics。
- frame/site pose 查询方法：如何返回当前位姿。
- Jacobian 查询方法：如何从 MuJoCo 得到任务 Jacobian。
- `integrate` / `integrate_inplace`：如何根据 velocity 更新 q。
- 限制检查或 keyframe 初始化相关 helper。

## 输入和输出

输入通常包括：

- MuJoCo `MjModel`
- 当前 `q`
- frame/site/body 名称
- 关节速度 `v` 或 `dq`
- 时间步长 `dt`

输出通常包括：

- 当前 configuration 的 q。
- 某个 frame/site 的 pose。
- 某个 frame/site 的 Jacobian。
- 积分后的新 q。

## 关键阅读问题

1. `Configuration` 是否自己创建 MuJoCo `data`？
2. q 的 shape 是多少？它和 `model.nq` 有什么关系？
3. velocity 的 shape 是多少？它和 `model.nv` 有什么关系？
4. 查询 frame pose 前，为什么必须确保 data 是最新的？
5. Jacobian 的 shape 是什么？前 3 行和后 3 行分别代表什么？
6. `integrate_inplace` 和直接 `q += dq * dt` 有什么区别？

## 和 A 项目的关系

A 项目里目前把 Configuration 的功能拆散在几个学习脚本里：

- `02_configuration_site_pose.py`: 学习从 q 更新 MuJoCo data 并查询 site pose。
- `03_site_jacobian_check.py`: 学习 site Jacobian 和有限差分验证。
- `04_ik_dls_wrapper.py`: 学习用 Jacobian 生成 `dq` 并更新状态。

mink 把这些能力收敛到一个类里。A 项目先拆开学，是为了看清每一步的数学意义。

## TODO 学习笔记

- TODO 1: 写出 `Configuration` 里保存了哪些核心成员变量。
- TODO 2: 写出一次 `update(q)` 后 MuJoCo data 中哪些量会变。
- TODO 3: 记录 pose 查询和 Jacobian 查询各调用了哪些 MuJoCo API。
- TODO 4: 解释为什么 floating base 或 quaternion 机器人不能简单做 `q += v * dt`。

## 详细学习任务与路径

### 任务 2.1: 理解初始化

- 阅读源码:
  - `source_annotated/src/mink/configuration.py`
- 重点函数:
  - `Configuration.__init__`
  - `Configuration.update`
- 学习目标:
  - 知道 `model`、`data`、`q` 如何建立关系。
- 操作:
  - 记录 `self.model` 和 `self.data` 从哪里来。
  - 记录如果没有传入 q，初始 q 是什么。
- 验收:
  - 能回答 `Configuration(model)` 创建后，当前状态存在哪里。

### 任务 2.2: 理解状态更新

- 阅读源码:
  - `source_annotated/src/mink/configuration.py`
- 重点函数:
  - `update`
  - `update_from_keyframe`
- 学习目标:
  - 理解为什么每次改 q 后都要 forward kinematics。
- 操作:
  - 找出 `mujoco.mj_kinematics`、`mujoco.mj_comPos` 或类似更新调用。
  - 写出 “q 改变 -> data 更新 -> pose/Jacobian 可查询” 的链路。
- 验收:
  - 能解释为什么 task 不应该直接用旧 data。

### 任务 2.3: 理解 frame pose 查询

- 阅读源码:
  - `source_annotated/src/mink/configuration.py`
- 重点函数:
  - frame/body/geom/site transform 查询相关方法。
- 学习目标:
  - 知道 mink 如何统一 body、geom、site 的 pose 查询。
- 操作:
  - 找出 `frame_name` 和 `frame_type` 如何决定查询哪个 MuJoCo 对象。
  - 记录返回值是 translation + rotation，还是 SE3。
- 验收:
  - 能和 A02 的 site pose 查询建立对应。

### 任务 2.4: 理解 Jacobian 查询和积分

- 阅读源码:
  - `source_annotated/src/mink/configuration.py`
- 重点函数:
  - `get_frame_jacobian`
  - `integrate`
  - `integrate_inplace`
- 学习目标:
  - 理解 Jacobian 的 shape 和 velocity integration。
- 操作:
  - 记录 Jacobian 是 `6 x nv` 还是 `3 x nv`。
  - 找出 `mujoco.mj_integratePos` 的调用位置。
- 验收:
  - 能说明 `nq` 和 `nv` 为什么可能不同。

