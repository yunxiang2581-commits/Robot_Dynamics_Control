# A02 Site Pose Report

## 目标
确认在给定 configuration q 下，MuJoCo 中 target site 和 body 的世界坐标系位姿。

## 输入模型
- MJCF: /home/ubuntu/Robot_Dynamics_Control/shared/robot_assets/models/mink_universal_robots_ur5e/scene.xml
- model nq/nv/nu: 6/6/6

## q 来源
- keyframe:home

## 查询结果
- target site: attachment_site
  - position: [0.4919993  0.13399783 0.48800037]
  - rotation matrix:
```
[[ 1.00000000e+00 -3.67321860e-06  3.67319161e-06]
 [-3.67320510e-06 -1.00000000e+00 -3.67321860e-06]
 [ 3.67320510e-06  3.67320510e-06 -1.00000000e+00]]
```
- target body: wrist_3_link
  - position: [0.49199893 0.13399819 0.58800037]
  - rotation matrix:
```
[[-3.67321860e-06  3.67319161e-06  1.00000000e+00]
 [-1.00000000e+00 -3.67321860e-06 -3.67320510e-06]
 [ 3.67320510e-06 -1.00000000e+00  3.67320510e-06]]
```

## 与 A03 Jacobian 的关系
- A03 将基于同样的 q 和 target site 验证 Jacobian 的正确性。

## 当前不做什么
- 不验证 Jacobian，不做 IK，不做控制，不录视频。