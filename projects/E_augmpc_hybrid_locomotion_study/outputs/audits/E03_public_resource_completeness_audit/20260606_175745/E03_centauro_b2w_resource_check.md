# E03 Centauro B2W Resource Check

| Check | Centauro | B2W | Notes |
|---|---|---|---|
| public bundle evidence | found in public metadata | found in public metadata | both robot directories are visible in HF public tree |
| rosbag / visualization evidence | found in public metadata | found in public metadata | centauro has explicit README example path; B2W has HF rosbag entry and README availability statement |
| eval evidence | mentioned but not verified | mentioned but not verified | documentation shows generic eval contract; no eval run or download performed |
| config evidence | found in public metadata | found in public metadata | visible yaml files in bundle tree; matching training cfg directories documented locally |
| model/checkpoint evidence | found in public metadata | found in public metadata | visible `<bundle_name>_model` file with xet marker |
| URDF/SRDF evidence | found in public metadata | found in public metadata | visible `*.urdf` and `*.srdf` files |
| private resource risk | not found | not found | public routes look stronger than Kyon; still subject to later container/runtime requirements |
| expected future stage | E04 or E05 | E04 or E05 | both are suitable handoff targets for prerequisite and command planning |

## Assessment

- `Centauro` 仍然是最强的第一公开目标，因为同时有：
  - public bundle tree evidence
  - explicit README rosbag example path
  - explicit model card `MPATH` / `MNAME` eval example
- `B2W` 仍然适合作为第一公开目标候选，但证据强度略低于 Centauro：
  - HF tree contents are strong
  - README confirms public B2W bag visualization remains available
  - README does not provide a fully expanded B2W rosbag path example like Centauro
- 两者都足以支撑进入 E04；两者都还不能被描述成“本地已验证可运行”。
