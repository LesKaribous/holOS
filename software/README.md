# `software/` — directory map

Quick index of what lives where. Updated 2026-05.

## Production

| Path | Role |
|---|---|
| [`run.py`](run.py) | holOS server entry point. Flask + SocketIO on :5000. Starts pipeline registry, transport, brain, simulator. |
| [`brain.py`](brain.py) | Strategy orchestrator. Reads `strategy/match.py`, dispatches via the active transport. |
| [`vision_pipelines_def.py`](vision_pipelines_def.py) | **Main vision config.** Pipeline graph + camera/parallax/world params. Edit me. |
| [`requirements.txt`](requirements.txt) | Python deps (Flask, OpenCV, etc.). |

## Subsystems

| Path | Role |
|---|---|
| [`services/`](services/) | Backend services consumed by `run.py`. |
| [`services/vision_pipelines/`](services/vision_pipelines/) | Pipeline framework + node implementations (rectify, aruco, localization, parallax, …). The bricks `vision_pipelines_def.py` assembles. |
| [`services/vision_backend.py`](services/vision_backend.py) | Legacy match-time vision backend (`VisionBackend`). Predates the pipeline system. Still wired — provides the `/api/vision/state` family + the heading-sync legacy path. Will retire once the pipeline absorbs everything. |
| [`vision/`](vision/) | Core CV building blocks (ArUco detection, BEV rectifier, robot tracker, charuco intrinsics). Imported by every pipeline node via a `sys.path.insert(…/vision/src)` shim. The `calibrations/` folder holds `camera_intrinsics.json`. |
| [`transport/`](transport/) | Serial transport abstraction (XBee, USB-CDC, Virtual). Same protocol everywhere. |
| [`shared/`](shared/) | Constants and protocol encoding shared with the firmware. |
| [`strategy/`](strategy/) | `match.py` (hot-reloadable robot strategy), `missions.json`, `macros.json`. |
| [`sim/`](sim/) | Simulator (`bridge.py`, `physics.py`, `world.py`) + the holOS web UI under `static/`. |
| [`tests/`](tests/) | Hardware test harness. |

## Standalone tools

These run separately from holOS; not loaded by `run.py`.

| Path | Role | Default port |
|---|---|---|
| [`vision_runner/`](vision_runner/) | Standalone debug pipeline runner — reads a video/camera directly, runs the full pipeline (rectify → aruco → localization → parallax) and surfaces every intermediate stage in a web UI. Useful for offline tuning without booting holOS. | `:5175` |
| [`tools/vision_runner.py`](tools/vision_runner.py) | CLI launcher for `vision_runner/`. Interactive source picker, scans `vision/data/` and `vision/homography/data/`. |

## Other

| Path | Role |
|---|---|
| [`data/`](data/) | Runtime configs and persisted state. `parallax_calibration.json` (auto-tune output), `vision_config.json`, `settings.json`, etc. |
| [`scripts/`](scripts/) | Bash/cmd wrappers around tools + the `holos_cli.py` and hardware-test runner. |
| [`actuator_data/`](actuator_data/) | Servo / actuator calibration data. |
| [`_archive/`](_archive/) | Retired code kept for reference. Not imported anywhere. |
| [`_archive/vision_twinvision/`](_archive/vision_twinvision/) | Standalone TwinVision desktop GUI (PyQt) and its install/run scripts. Was the original lab-bench tool; superseded by holOS's pipeline system. |
| [`_archive/vision_pipelines_json/`](_archive/vision_pipelines_json/) | Old pipeline-graph JSON storage. Pipelines are now defined in `vision_pipelines_def.py` (Python). |
| [`_archive/vision_editor/`](_archive/vision_editor/) | Old in-browser pipeline editor. Superseded by the Python def file. |

## Two vision tools

```
   ┌────────────────────────────────────────────────────────────┐
   │ holOS run.py (port 5000)                                   │
   │   vision_source.FrameSource — one reader thread, 1-slot    │
   │     latest-BGR buffer (GStreamer + nvv4l2decoder on Jetson)│
   │       │                                                     │
   │       ├──▶ pipeline source nodes (8 FPS)                   │
   │       └──▶ vision_recorder.Recorder (16 FPS, writes AVI)    │
   └────────────────────────────────────────────────────────────┘

   ┌────────────────────────────────────────────────────────────┐
   │ vision_runner (port 5175) — standalone debug tool          │
   │   Opens its own source (file / V4L2). Runs the full         │
   │   pipeline + intermediate-view UI. Independent of holOS.    │
   └────────────────────────────────────────────────────────────┘
```

`vision_runner` is for offline pipeline tuning without booting the whole robot stack. `holOS` is the production app that talks to the firmware; it owns the camera and the recorder in-process so there is no queue between hardware capture and the pipeline tick.

## Where parameters live

| What | Where |
|---|---|
| Camera position, anchors, robot tag z, FPS, team mirroring | [`vision_pipelines_def.py`](vision_pipelines_def.py) — top of file |
| Auto-tuned parallax (per-team, persisted) | `data/parallax_calibration.json` |
| FrameSource defaults (source kind/path, GStreamer geom, FPS knobs) | `data/vision_config.json` |
| holOS UI prefs | `data/settings.json` |
| Camera intrinsics (charuco-calibrated, K + dist) | [`vision/calibrations/camera_intrinsics.json`](vision/calibrations/camera_intrinsics.json) |
| Firmware ↔ Python protocol constants | [`shared/config.py`](shared/config.py) |
