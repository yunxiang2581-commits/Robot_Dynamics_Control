# A Project Outputs

本目录保存 A 项目 pipeline 的运行产物。项目级输出应放在这里，根目录 `outputs/` 只作为总项目输出索引。

## 子目录规则

- `cache/`：保存可复用的轻量中间结果，例如模型摘要、frame 选择、QP 状态 JSON。不要保存大模型或大数组。
- `reports/`：保存每一步的人类可读 Markdown 或文本报告，例如 A01 模型检查、A03 Jacobian 验证、A06 PD 跟踪报告。
- `figures/`：保存小型结果图，例如误差曲线、轨迹对比图、Jacobian 验证图。
- `trajectories/`：保存轻量轨迹文件，例如 A04/A05 生成的 `q` 轨迹 CSV。大型 `.npy` 或长时间运行结果应先人工确认。
- `logs/`：保存运行日志和仿真 CSV。默认不进入 Git，只保留必要摘要。
- `videos/`：保存 MuJoCo 或可视化视频。默认不进入 Git。
- `legacy_samples/`：保存从旧项目迁入的小型示例结果，仅用于参考，不作为标准 pipeline 输出。

## 命名建议

标准输出文件建议使用步骤编号前缀：

```text
A01_inspect_urdf.md
A02_fk_frame_pose.md
A03_jacobian_fd_check.md
A04_dls_ik_q_traj.csv
A05_qp_ik_q_traj.csv
A06_pd_tracking.csv
A07_mini_wbc_qp.md
```

后续实现时优先通过 `src/robot_baseline/pipeline_io.py` 统一创建输出路径。
