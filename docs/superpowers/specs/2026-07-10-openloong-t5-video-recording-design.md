# OpenLoong T5 MuJoCo Video Recording Design

## Goal

Add optional MP4 recording to the Project C T5 native WSLg simulation used for the
`pipeline_fast`, platform `0.56`, `axis_lift`, and default T-Yaw `R07=0.956396`
configuration.

The input remains the existing MuJoCo simulation state and rendered framebuffer.
The output is a playable H.264 MP4 stored with the current run artifacts. Recording
must not change IK, WBC, MPC, FSM transitions, controller gains, scene geometry, or
simulation targets.

## Scope

The implementation spans two existing repositories:

1. `Robot_Dynamics_Control` owns the T5 WSLg launcher, the recording-capable Docker
   image layer, run metadata, and behavior tests.
2. The `OpenLoong-Dyn-Control` T5 worktree owns the minimal MuJoCo framebuffer to
   FFmpeg stream and its cleanup path.

The feature is for native WSLg. It must not start or depend on a browser, VNC, or
noVNC container.

The exact project-owned files are:

```text
projects/C_openloong_dyn_control_study/
├── outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827/
│   └── t5_basket_visual_wslg.sh
└── tools/openloong_demo_runner/
    ├── recording/Dockerfile
    ├── commands/build_openloong_recording_image.sh
    └── tests/test_t5_video_recording_contract.ps1
```

The currently operational `t5_basket_visual_wslg.sh` will become a tracked
project artifact as part of this change. The recording image name is fixed to
`openloong-ubuntu22-recording:local`.

## Selected Approach

Encode frames directly inside the simulation container instead of first writing
`rgbRec.out`.

The existing renderer already calls `mjr_readPixels` once per visual frame. When
recording is enabled, those RGB24 frames will be written to an FFmpeg subprocess.
FFmpeg will vertically flip the MuJoCo framebuffer and encode H.264 with a
`yuv420p` pixel format. The recording image will derive from the existing Project C
build image and add FFmpeg only.

This avoids the legacy raw-file path, which would consume roughly 10 GB for a
60-second 1200x800 recording at 60 FPS.

## User Interface

Recording remains disabled by default.

Enable it through the existing T5 visual launcher:

```bash
OPENLOONG_T5_SAVE_VIDEO=1 ./t5_basket_visual_wslg.sh 60
```

The launcher will accept only explicit boolean values for
`OPENLOONG_T5_SAVE_VIDEO`. Recording mode requires the native `wslg` backend and a
locally built recording image. A missing image produces a clear build command and
stops before starting the simulation.

The first implementation fixes the capture rate at 60 FPS because the T5 outer
visual loop already renders at 60 Hz. Codec, CRF, arbitrary output names, audio,
GIF export, and camera scripting are outside this scope.

## Output Contract

Each run keeps its existing timestamped `RUN_ROOT`. Recording artifacts are:

```text
RUN_ROOT/
├── run_info.txt
├── logs/
└── runtime_record/
    └── t5_simulation.mp4
```

`run_info.txt` records whether video was enabled, the expected codec, frame rate,
and output path. A successful recording must be non-empty and readable by
`ffprobe`. The final file must report H.264 video, `yuv420p`, a positive duration,
and the framebuffer dimensions used by the renderer.

FFmpeg writes a temporary MP4 name during capture. The renderer publishes
`t5_simulation.mp4` only after FFmpeg exits successfully, preventing an interrupted
run from presenting a partial file as complete.

## Renderer Changes

The T5 demo reads `OPENLOONG_T5_SAVE_VIDEO` and passes the result to the existing
`UIctr::createWindow` recording flag. Other demos retain their current default.

When recording is enabled, `UIctr` will:

1. Read the actual framebuffer size after GLFW creates the window.
2. Allocate an RGB buffer sized for that framebuffer.
3. Verify that `ffmpeg` is available before simulation begins.
4. Start one FFmpeg subprocess configured for RGB24 input at 60 FPS.
5. Read and write one rendered frame per `updateScene()` call.
6. Detect short writes and encoder failures.
7. Flush and close FFmpeg before destroying MuJoCo and GLFW resources.
8. Free frame buffers and publish the final MP4 only after a successful encoder
   exit.

The renderer will reject framebuffer size changes during recording with a clear
error instead of writing a corrupt stream. This is acceptable because the T5
recording window uses a fixed capture size.

## Container And Launcher Changes

Add a small recording image definition under the existing Project C demo runner.
It derives from `openloong-ubuntu22-build:local`, installs FFmpeg without browser
packages, and keeps the same native WSLg display mounts.

`commands/build_openloong_recording_image.sh` is the single supported build entry
point for `openloong-ubuntu22-recording:local`; the launcher reports that exact
command when the image is absent.

The T5 launcher will:

1. Preserve the current image and behavior when recording is disabled.
2. Select the recording image only when `OPENLOONG_T5_SAVE_VIDEO=1`.
3. Pass the recording flag into the container.
4. Keep `runtime_record` mounted at the OpenLoong `record` directory.
5. Add a recording timeout margin so normal simulation exit can close FFmpeg.
6. Treat a timeout or encoder failure as a failed recording, even if simulation
   logs advanced.
7. Validate the final MP4 with `ffprobe` from the recording image and print its
   host path.

## Failure Handling

- Invalid recording flag: stop before Docker starts.
- noVNC or browser backend requested with recording: stop and require native WSLg.
- Recording image missing: print the deterministic image build command.
- FFmpeg missing in the selected image: stop before the simulation loop.
- Framebuffer allocation or encoder startup failure: stop with a clear error.
- Framebuffer dimensions change: stop recording rather than corrupting output.
- Short frame write or FFmpeg nonzero exit: keep diagnostic logs, do not publish
  the final MP4, and return failure.
- Simulation timeout: report the run as incomplete and do not claim video success.
- Recording disabled: preserve existing launch and control behavior exactly.

## Testing

Follow TDD and keep destructive or long tests separate from contract tests.

1. Add a failing launcher contract test before implementation. It verifies default
   recording-off behavior, recording image selection, native WSLg enforcement,
   environment propagation, run metadata, timeout margin, and output path.
2. Add a failing source contract or focused renderer test for encoder startup,
   frame writes, cleanup, and temporary-file publication.
3. Build the T5 target and recording image.
4. Run the launcher in dry-run mode with recording disabled and enabled.
5. Run a three-second native WSLg recording smoke test.
6. Validate the MP4 with `ffprobe`: H.264, `yuv420p`, positive duration, nonzero
   frame count, and expected framebuffer dimensions.
7. Confirm no `rgbRec.out` was created.
8. Run the existing T5 launcher and WSLg tests to detect regressions.

The short smoke test proves the recording pipeline. A full 60-second
`pipeline_fast / 0.56 / axis_lift / R07=0.956396` run is the final acceptance test
after the short test succeeds.

## Risks

- Encoding adds CPU usage and may increase wall-clock runtime. Use an FFmpeg preset
  intended for real-time capture and retain a timeout margin.
- Synchronous frame writes can back-pressure rendering if the encoder cannot keep
  up. The three-second smoke test must measure this before the full run.
- WSLg COPY MODE is a separate display-session failure. If it reappears, repair the
  WSLg session first; do not replace native recording with browser capture.
- Existing unrelated dirty files in the OpenLoong T5 worktree must remain
  untouched.

## Acceptance Criteria

Implementation is complete only when:

- Recording is off by default.
- Recording mode uses native WSLg and does not start browser/noVNC components.
- A three-second run produces a valid H.264/yuv420p MP4 without `rgbRec.out`.
- The encoder closes cleanly and partial output is not published as final.
- A full configured T5 run produces its MP4 under the timestamped run directory.
- Runtime logs and final FSM state remain available for comparison.
- No IK, WBC, MPC, FSM, scene, or controller parameter logic changes.
- Focused tests, existing launcher tests, `git diff --check`, and `git diff --stat`
  pass.
- No automatic `git push` is performed.
