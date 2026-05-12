# Project B MuJoCo XML Models

本目录用于放置 Project B 自建的 MuJoCo XML 模型文件。

当前规划：

```text
simulator/models/B01_single_joint.xml
simulator/models/B02_two_link.xml
```

## B01_single_joint.xml

用途：

- B01 单关节 hinge joint MPC demo 的最小 MuJoCo 模型。
- 后续由 `SingleJointEnv` 默认加载。

当前状态：

- 已创建最小单关节 MJCF 模型。
- 不运行 MuJoCo。
- 不生成仿真输出。

模型当前包含：

- 一个固定 base。
- 一个 hinge joint：`single_hinge`。
- 一个 capsule link：`single_link_geom`。
- 一个 torque actuator：`single_joint_motor`。
- 一个 tip site：`tip_site`。
- joint position / velocity / actuator force sensors。
- camera 和 light，方便后续视频导出。

## B02_two_link.xml

用途：

- B02 二连杆末端轨迹 tracking demo 的最小 MuJoCo 模型。
- 后续由 `TwoLinkEnv` 默认加载。

当前状态：

- 已创建最小二连杆 MJCF 模型。
- 包含 shoulder / elbow 两个 hinge joint。
- 包含 shoulder_motor / elbow_motor 两个 torque actuator。
- 包含末端 site：`ee_site`。
- 当前只作为 B02 skeleton 的模型入口，完整闭环控制仍待实现。
