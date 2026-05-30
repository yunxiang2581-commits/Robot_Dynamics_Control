# C04 FIX Runtime R2 wbc_speed_test Report

## 1. 本轮目标

C04-FIX-RUNTIME-R1 已经解决动态库 RUNPATH 不匹配问题，但 `wbc_speed_test` 因 R2 worktree 只读挂载，无法写入：

```text
../record/datalog.log
```

本轮目标：

- 不改官方源码。
- 不改 R2 worktree 源码。
- R2 worktree 仍以 `/run_root:ro` 只读挂载。
- 只给容器内 `/run_root/worktree/OpenLoong-Dyn-Control/record` 提供可写 overlay。
- 只运行 `wbc_speed_test`。
- 使用 `timeout 20s` 防止阻塞。

## 2. 上一轮失败摘要

C04-FIX-RUNTIME-R1 输出目录：

```text
projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_wbc_speed_test_runtime_fix_R1/20260530_184108/
```

上一轮状态：

```text
dynamic library lookup: PASS
wbc_speed_test execution: FAIL
exit code: 134
```

第一条真实错误：

```text
terminate called after throwing an instance of 'quill::QuillError'
  what():  fopen failed with error message errno: "30"
```

原因：`DataLogger("../record/datalog.log")` 需要写 `record/`，但 `/run_root` 是只读挂载。

## 3. 输入与挂载方式

R2 输出路径：

```text
projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827/
```

目标可执行文件：

```text
worktree/OpenLoong-Dyn-Control/build/wbc_speed_test
```

Docker image：

```text
openloong-ubuntu22-build:local
```

本轮 Docker 挂载：

```text
R2_ROOT:/run_root:ro
RUN_ROOT:/c04_fix_root
RUN_ROOT/runtime_record:/run_root/worktree/OpenLoong-Dyn-Control/record
```

解释：

- `/run_root` 仍然只读，匹配 build-time RUNPATH。
- `record/` 单独覆盖为可写目录，接收 `datalog.log`。
- datalog 写入 C04-FIX-R2 输出目录，不写回 R2 worktree。

## 4. 预检查结果

record overlay 写入探针通过：

```text
record overlay write probe: OK
```

`ldd` 能找到 qpOASES：

```text
libqpOASES.so.3.2 => /run_root/worktree/OpenLoong-Dyn-Control/third_party/qpOASES/lin_x64/libqpOASES.so.3.2
```

RUNPATH 仍为 `/run_root/...`：

```text
RUNPATH: /run_root/worktree/OpenLoong-Dyn-Control/third_party/.../lin_x64
```

预检查日志：

```text
logs/01_wbc_speed_test_precheck.log
```

## 5. 运行命令

宿主机 Docker 命令：

```bash
docker run --rm \
  -v "$R2_ROOT:/run_root:ro" \
  -v "$RUN_ROOT:/c04_fix_root" \
  -v "$RUN_ROOT/runtime_record:/run_root/worktree/OpenLoong-Dyn-Control/record" \
  openloong-ubuntu22-build:local \
  bash /c04_fix_root/commands/container_run_wbc_speed_test_record_overlay.sh
```

容器内核心命令：

```bash
cd /run_root/worktree/OpenLoong-Dyn-Control/build
timeout 20s ./wbc_speed_test \
  > /c04_fix_root/logs/02_wbc_speed_test_stdout.log \
  2> /c04_fix_root/logs/03_wbc_speed_test_stderr.log
```

命令记录：

```text
commands/C04_fix_runtime_R2_commands.sh
commands/container_run_wbc_speed_test_record_overlay.sh
```

## 6. 运行结果

结果：PASS。

退出码：

```text
0
```

stdout：

```text
10001 lines
```

stderr：

```text
0 lines
```

stdout 包含 WBC benchmark 循环耗时输出，例如：

```text
Execution time: 0.003083 sec.
Execution time: 0.002318 sec.
...
loop time recorded to the last column of record/datalog.log
```

生成 datalog：

```text
runtime_record/datalog.log
artifacts/datalog.log
```

datalog 行数：

```text
10000
```

证据日志：

```text
logs/02_wbc_speed_test_stdout.log
logs/03_wbc_speed_test_stderr.log
logs/04_wbc_speed_test_summary.log
logs/05_evidence_summary.log
artifacts/wbc_speed_test_exit_code.txt
artifacts/datalog.log
```

## 7. 当前结论

`wbc_speed_test` runtime smoke 已通过。

当前最小运行链路成立：

```text
Docker Ubuntu 22.04 + g++11
R2 build artifacts
RUNPATH /run_root matched
record/ writable overlay
wbc_speed_test exits 0
datalog.log generated with 10000 lines
```

这说明 C03 R2 build 产物具备最小 non-viewer 运行能力。

## 8. 下一步建议

建议进入两条路线之一：

1. `C05A-WBC-SPEED-TEST-SOURCE-TRACE`：追踪 `wbc_speed_test` 主循环、WBC 输入输出、datalog 字段含义。
2. `C06-WALK-WBC-RUNTIME-PLANNING`：规划 `walk_wbc` 的 GUI / X11 / OpenGL / datalog / 截图或录屏方案。

更稳的顺序是先做 C05A，把 non-viewer 的 WBC 计算链条读清楚，再进入 `walk_wbc` GUI。

## 9. 边界确认

本轮确认：

- 未修改 `external/open_source_repos/OpenLoong-Dyn-Control/` 官方源码。
- 未修改 R2 worktree 源码。
- R2 worktree 主体使用只读挂载。
- `record/` 使用 C04-FIX-R2 输出目录中的可写 overlay。
- 未运行 `walk_wbc`。
- 未运行 `walk_mpc_wbc`。
- 未运行长时间 MuJoCo。
- 未打开 GUI viewer。
- 未生成 MP4。
- 未执行 `git add`。
- 未执行 `git commit`。
- 未执行 `git push`。

补充验证：R2 worktree 原始 `record/` 目录未新增 `datalog.log`；datalog 写在本轮输出目录的 `runtime_record/` 和 `artifacts/` 中。
