# E01 Public Bundle And Rosbag Audit

## Public Metadata Only

Hugging Face 公开页显示 `AndrePatri/AugMPCModels` 是 demo bundle 集合，并推荐通过 `ibrido-containers` 使用。

## What Appears Publicly Reachable From Documentation

- Public demo bundles exist on Hugging Face metadata page.
- Bundle root convention: `/root/training_data/AugMPCModels/bundles/<robot_name>/<bundle_name>`.
- Bundle contents may include: checkpoint, configs, URDF, SRDF, helper scripts, bundle.yaml.
- `ibrido-containers` README states some public visualization routes can use bundle-contained ROS bags without launching simulation.
- Publicly mentioned robots for visualization availability include `Centauro` and `B2W`.

## What Remains Restricted Or Unverified In E01

- No bundle archive or model asset was downloaded.
- No ROS bag was downloaded.
- No replay command was executed.
- Kyon-related visualization / evaluation is documented as depending on private robot-description repositories.

## Audit Outcome

从“公开可见资料”角度，最像低风险第一落点的是“公共 bundle + 公共 rosbag 的非仿真可视化链路”；但这在 E01 仍然只是静态判断，不是运行验证。
