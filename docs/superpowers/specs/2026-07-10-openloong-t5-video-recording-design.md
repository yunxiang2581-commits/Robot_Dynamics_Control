# OpenLoong T5 MuJoCo 仿真视频保存设计

## 目标

为 Project C 的 T5 原生 WSLg 仿真增加可选的 MP4 视频保存功能。目标仿真配置为
`pipeline_fast`、平台高度 `0.56`、`axis_lift`，以及默认 T-Yaw
`R07=0.956396`。

输入仍然是现有 MuJoCo 仿真状态和渲染帧缓冲区，输出是随本次运行产物保存的、
可以直接播放的 H.264 MP4。录制功能不得改变 IK、WBC、MPC、FSM 状态转换、
控制器增益、场景几何或仿真目标。

## 范围

实现涉及两个现有仓库：

1. `Robot_Dynamics_Control` 负责 T5 WSLg 启动脚本、支持录制的 Docker 镜像层、
   运行元数据和行为测试。
2. `OpenLoong-Dyn-Control` 的 T5 工作树负责把 MuJoCo 帧缓冲区以最小改动送入
   FFmpeg，并正确释放录制资源。

此功能只面向原生 WSLg，不得启动或依赖浏览器、VNC 或 noVNC 容器。

主项目负责的准确文件如下：

```text
projects/C_openloong_dyn_control_study/
├── outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827/
│   └── t5_basket_visual_wslg.sh
└── tools/openloong_demo_runner/
    ├── recording/Dockerfile
    ├── commands/build_openloong_recording_image.sh
    └── tests/test_t5_video_recording_contract.ps1
```

当前实际使用的 `t5_basket_visual_wslg.sh` 将在本次修改中成为受 Git 管理的项目
产物。录制镜像名称固定为 `openloong-ubuntu22-recording:local`。

## 选定方案

在仿真容器内直接编码 MP4，不再先生成 `rgbRec.out`。

现有渲染器已经在每个可视帧调用 `mjr_readPixels`。开启录制后，这些 RGB24 帧
将直接写入 FFmpeg 子进程。FFmpeg 对 MuJoCo 帧缓冲区执行垂直翻转，并编码为
H.264，像素格式使用 `yuv420p`。录制镜像从现有 Project C 构建镜像派生，只增加
FFmpeg，不加入浏览器相关组件。

该方案避免旧的裸帧文件路径。按 1200x800、60 FPS 计算，60 秒裸帧文件约占
10 GB。

## 使用接口

默认不录制视频。

通过现有 T5 可视启动脚本显式开启：

```bash
OPENLOONG_T5_SAVE_VIDEO=1 ./t5_basket_visual_wslg.sh 60
```

启动脚本只接受明确的 `OPENLOONG_T5_SAVE_VIDEO` 布尔值。录制模式要求使用原生
`wslg` 后端，并要求本地已经构建录制镜像。镜像不存在时，脚本应打印清晰的构建
命令，并在启动仿真前停止。

第一版录制帧率固定为 60 FPS，因为 T5 外层可视循环本来就是 60 Hz。自定义
编码器、CRF、输出文件名、音频、GIF 导出和相机脚本不在本次范围内。

## 输出契约

每次运行继续使用现有带时间戳的 `RUN_ROOT`。录制产物结构为：

```text
RUN_ROOT/
├── run_info.txt
├── logs/
└── runtime_record/
    └── t5_simulation.mp4
```

`run_info.txt` 记录是否启用视频、预期编码格式、帧率和输出路径。成功录制的视频
必须非空，并且能够被 `ffprobe` 读取。最终文件必须报告 H.264 视频、`yuv420p`
像素格式、正数时长，以及渲染器实际使用的帧缓冲区尺寸。

录制过程中，FFmpeg 写入临时 MP4 文件名。只有 FFmpeg 正常退出后，渲染器才
发布最终的 `t5_simulation.mp4`，防止异常中断的文件被误认为完整视频。

## 渲染器修改

T5 demo 读取 `OPENLOONG_T5_SAVE_VIDEO`，并把结果传给现有
`UIctr::createWindow` 的录制开关。其他 demo 保持当前默认行为。

开启录制时，`UIctr` 执行以下步骤：

1. GLFW 创建窗口后读取实际帧缓冲区尺寸。
2. 按实际尺寸分配 RGB 缓冲区。
3. 在仿真开始前确认 `ffmpeg` 可用。
4. 启动一个以 RGB24、60 FPS 为输入的 FFmpeg 子进程。
5. 每次 `updateScene()` 读取并写入一帧。
6. 检测帧写入不足和编码器失败。
7. 在销毁 MuJoCo 和 GLFW 资源前刷新并关闭 FFmpeg。
8. 释放帧缓冲区，并且仅在编码器成功退出后发布最终 MP4。

录制期间如果帧缓冲区尺寸变化，渲染器应给出明确错误并停止录制，不能继续写入
损坏的视频流。T5 录制窗口采用固定采集尺寸，因此这一限制可以接受。

## 容器与启动脚本修改

在现有 Project C demo runner 中增加一个小型录制镜像定义。该镜像从
`openloong-ubuntu22-build:local` 派生，只安装 FFmpeg，并继续使用相同的原生
WSLg 显示挂载。

`commands/build_openloong_recording_image.sh` 是
`openloong-ubuntu22-recording:local` 唯一支持的构建入口。镜像不存在时，启动
脚本打印这条准确命令。

T5 启动脚本应当：

1. 录制关闭时保持当前镜像和行为不变。
2. 仅在 `OPENLOONG_T5_SAVE_VIDEO=1` 时选择录制镜像。
3. 把录制开关传入容器。
4. 继续把 `runtime_record` 挂载到 OpenLoong 的 `record` 目录。
5. 为录制模式增加超时余量，使仿真正常退出时可以完成 FFmpeg 收尾。
6. 超时或编码器失败时，即使仿真日志仍在推进，也必须把录制判定为失败。
7. 使用录制镜像中的 `ffprobe` 验证最终 MP4，并打印宿主机文件路径。

## 失败处理

- 录制开关值无效：Docker 启动前停止。
- 录制时请求 noVNC 或浏览器后端：停止，并要求使用原生 WSLg。
- 录制镜像不存在：打印确定的镜像构建命令。
- 所选镜像中没有 FFmpeg：仿真循环开始前停止。
- 帧缓冲区分配或编码器启动失败：给出明确错误并停止。
- 帧缓冲区尺寸变化：停止录制，不能生成损坏输出。
- 帧写入不足或 FFmpeg 非零退出：保留诊断日志，不发布最终 MP4，并返回失败。
- 仿真超时：报告运行未完整结束，不得声称视频保存成功。
- 录制关闭：完全保留现有启动行为和控制行为。

## 测试

采用 TDD，并把无破坏性的契约测试与长时间运行测试分开。

1. 实现前先增加失败的启动器契约测试，验证默认关闭录制、录制镜像选择、原生
   WSLg 限制、环境变量传递、运行元数据、超时余量和输出路径。
2. 增加失败的源码契约测试或聚焦的渲染器测试，覆盖编码器启动、帧写入、资源
   关闭和临时文件发布。
3. 构建 T5 目标程序和录制镜像。
4. 分别在关闭和开启录制时运行启动脚本的 dry-run。
5. 运行 3 秒原生 WSLg 录制烟测。
6. 使用 `ffprobe` 验证 MP4：H.264、`yuv420p`、正数时长、非零帧数，以及预期
   帧缓冲区尺寸。
7. 确认没有生成 `rgbRec.out`。
8. 运行现有 T5 启动器和 WSLg 测试，检查回归。

短烟测用于证明录制链路。短烟测通过后，再用
`pipeline_fast / 0.56 / axis_lift / R07=0.956396` 完整运行 60 秒，作为最终
验收测试。

## 风险

- 编码会增加 CPU 使用率和实际运行时间。FFmpeg 应使用适合实时采集的 preset，
  同时为启动脚本保留超时余量。
- 如果编码器处理不及，帧同步写入可能反向阻塞渲染。必须先通过 3 秒烟测测量，
  再运行完整仿真。
- WSLg COPY MODE 属于独立的显示会话故障。如果再次出现，应先修复 WSLg 会话，
  不能退回浏览器录屏。
- OpenLoong T5 工作树中已有的无关脏文件必须保持不变。

## 验收条件

只有满足以下条件，才算实现完成：

- 默认不录制视频。
- 录制模式使用原生 WSLg，不启动浏览器或 noVNC 组件。
- 3 秒运行能够生成有效的 H.264/yuv420p MP4，且没有 `rgbRec.out`。
- 编码器正确关闭，部分输出不会以最终文件名发布。
- 完整 T5 配置运行后，MP4 位于带时间戳的运行目录中。
- 运行日志和最终 FSM 状态仍然保留，可用于对比。
- 不修改 IK、WBC、MPC、FSM、场景或控制器参数逻辑。
- 聚焦测试、现有启动器测试、`git diff --check` 和 `git diff --stat` 均通过。
- 不自动执行 `git push`。
