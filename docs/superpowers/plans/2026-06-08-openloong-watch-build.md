# OpenLoong Watch Sync Build Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为任意 OpenLoong demo 提供一个宿主机 watcher，在源码保存后自动同步到 build-root worktree 并触发 Docker 内增量编译。

**Architecture:** 抽出一个共享 shell 配置文件，集中维护 demo 名、可执行文件名、build-root 路径和公共校验逻辑。现有 runner 改为 source 这个共享文件。新增 watcher 复用同一套解析逻辑，并提供 `--dry-run` 和轮询/`inotifywait` 双模式，保证当前环境可用。

**Tech Stack:** Bash, rsync, Docker, CMake, shell test scripts

---

### Task 1: 抽取共享 demo 配置

**Files:**
- Create: `projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/commands/openloong_demo_common.sh`
- Modify: `projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/commands/run_openloong_demo_host.sh`
- Test: `projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/tests/test_openloong_demo_runner.sh`

- [ ] 将 demo 映射、默认 build-root、路径校验函数迁到共享脚本
- [ ] 让 `run_openloong_demo_host.sh` 只保留运行入口逻辑
- [ ] 运行现有 runner 测试，确认行为不回归

### Task 2: 先写 watcher 的失败测试

**Files:**
- Create: `projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/tests/test_openloong_watch_sync_build.sh`
- Test: `projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/commands/watch_openloong_sync_and_build.sh`

- [ ] 为 `--dry-run walk_wbc_staircase` 写断言，要求输出 demo、target、source root、build root、docker image
- [ ] 为非法 demo 写失败断言
- [ ] 为 `--poll-interval` 参数写解析断言

### Task 3: 实现 watcher

**Files:**
- Create: `projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/commands/watch_openloong_sync_and_build.sh`
- Modify: `projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/commands/generate_demo_wrapper_scripts.sh`（如需生成 watcher 包装脚本）

- [ ] 实现参数解析：`demo_name`、`--dry-run`、`--debounce`、`--poll-interval`
- [ ] 实现单次 `sync_and_build_once()`，内部顺序固定为 `rsync -> docker build`
- [ ] 优先使用 `inotifywait`，缺失时退化到轮询模式
- [ ] 输出清晰日志，包含时间戳、demo、target、同步来源和编译结果

### Task 4: 验证

**Files:**
- Test: `projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/tests/test_openloong_demo_runner.sh`
- Test: `projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/tests/test_openloong_watch_sync_build.sh`

- [ ] 跑 runner 旧测试
- [ ] 跑 watcher 新测试
- [ ] 跑一次 watcher `--dry-run walk_wbc_staircase`
- [ ] 记录 `git diff --stat`
