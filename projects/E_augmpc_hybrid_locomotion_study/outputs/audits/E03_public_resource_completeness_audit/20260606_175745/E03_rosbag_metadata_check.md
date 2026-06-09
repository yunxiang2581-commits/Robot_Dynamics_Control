# E03 Rosbag Metadata Check

本文件只确认公开 rosbag metadata 证据，不下载任何 bag。

| Rosbag Candidate | Robot | Public Evidence | Requires Download? | Requires Container? | Requires GUI? | Suitable For First Visualization Smoke? | Notes |
|---|---|---|---|---|---|---|---|
| rosbag_centauro_big_wheels_ub_2026_02_21_13_59_20_ID_2026-02-22_10-51-13_5 | Centauro | README example path + HF bundle tree entry | yes, to actually replay | yes, future visualization path is container-oriented in README | yes, future visualization implies MPCViz / bag replay GUI stack | yes | strongest public rosbag evidence in current docs |
| rosbag_unitree_b2w_2026_03_28_11_09_54_ID_2026-03-29_03-11-09_6 | B2W | HF bundle tree entry + README says public B2W bag visualization remains available | yes, to actually replay | yes, README frames bag replay inside interactive container session | yes, future visualization implies MPCViz / bag replay GUI stack | yes | public evidence exists, but README example path is less explicit than Centauro |
| <path_to_rosbag> | Kyon | README shows placeholder viz command only | cannot confirm without public bundle visibility | yes | yes | no | blocked by private robot-description dependency |

## Summary

- 存在 public rosbag metadata 证据。
- 当前最强的 Centauro 证据来自 `ibrido-containers` README 的完整示例路径。
- B2W 在 HF bundle 树里能看到 rosbag 目录名，且 README 明确说 public B2W bag visualization remains available。
- 这些证据足以支撑 E04 去列执行前提，但还不足以当作“本地已可直接可视化”。
