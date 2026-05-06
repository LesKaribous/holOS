"""
VisionBackend — Match-time vision pipeline for holOS.

Built on the TwinVision core (software/vision/src/core/) — we only consume
the runtime building blocks here, never the calibration logic. Calibration
is done in TwinVision and the resulting intrinsics + anchor config are
loaded from disk.

Responsibilities
----------------
1. Open a video source (camera index, video file, RTSP URL).
2. ArUco detection + per-frame solvePnP rectification with anchor caching.
   The 4 static anchor tags (corners of the table) are cached so the pose
   survives single-anchor occlusions during a match.
3. Track the OWN robot tag + opponent robot tag, with parallax correction
   and a 2D Kalman smoother.
4. Long-term OTOS drift correction:
       — captures (vision_theta - otos_theta) heading offset on demand
         (the tag is mounted in unknown orientation at match start)
       — exposes pop_correction() that returns (x, y, theta_rad) ready
         to be sent to the firmware via setAbsPosition(...) when:
           a) robot tag is currently visible,
           b) robot has moved more than MIN_CORRECTION_DIST_MM since the
              last correction,
           c) at least MIN_CORRECTION_PERIOD_S have elapsed.
       — never auto-fires; run.py polls and applies.
5. JPEG-encode raw + rectified frames for the visio webapp tab. Encoding
   is skipped when no client is subscribed (perf).

The visio tab is a LIVE view of this running pipeline — it has no
playback, no test pipeline (binarize/canny/...), no object registry.

Tag-ID conventions (Coupe de France de Robotique)
    blue team   → ROBOT IDs 1..5    | OPPONENT IDs 6..10
    yellow team → ROBOT IDs 6..10   | OPPONENT IDs 1..5
"""

from __future__ import annotations

import base64
import json
import os
import sys
import threading
import time
import math
from typing import Any, Optional

# ── Vision core path ──────────────────────────────────────────────────────────
_VISION_SRC = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'vision', 'src'
)
if _VISION_SRC not in sys.path:
    sys.path.insert(0, os.path.abspath(_VISION_SRC))

_VISION_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'vision'
)
_INTRINSICS_PATH = os.path.abspath(os.path.join(
    _VISION_DIR, 'calibrations', 'camera_intrinsics.json'
))
_TWINVISION_SETTINGS_PATH = os.path.abspath(os.path.join(
    _VISION_DIR, 'twinvision_settings.json'
))

CV2_AVAILABLE = False
try:
    import cv2
    import numpy as np
    from core.aruco_detector  import ArucoDetector, ARUCO_DICTS, DetectionResult
    from core.table_rectifier import TableRectifier, CornerConfig, TagAnchor
    from core.video_source    import VideoSource
    from core.robot_tracker   import RobotTracker, RobotConfig, RobotState
    from core.charuco_calib   import CameraIntrinsics
    CV2_AVAILABLE = True
except ImportError as _e:
    print(f"[VisionBackend] OpenCV / vision core not available: {_e}")
    ArucoDetector = TableRectifier = VideoSource = None
    CameraIntrinsics = RobotTracker = None
    ARUCO_DICTS = {}


# ── Tag-ID conventions ───────────────────────────────────────────────────────
# Per Coupe de France rules: a team gets a contiguous ID range.
TEAM_TAG_RANGES = {
    'blue':   {'own': range(1, 6),  'opp': range(6, 11)},   # blue   → own=1..5, opp=6..10
    'yellow': {'own': range(6, 11), 'opp': range(1, 6)},    # yellow → own=6..10, opp=1..5
}


# ── Default config ───────────────────────────────────────────────────────────
DEFAULT_CONFIG = {
    "dict":               "4x4_50",       # ArUco dictionary
    "fps_limit":          25,             # max processing fps
    "jpeg_quality":       70,             # 30..95
    "show_aruco":         True,           # draw detected markers on raw
    "show_ids":           True,           # draw IDs on raw
    "show_grid":          True,           # draw mm grid on rectified
    "show_table_overlay": True,           # draw table contour on raw
    "robot_z_mm":         490.0,          # tag height above floor (parallax)
    "anchors": {
        "top_left":     {"tag_id": 23, "x_mm": 600,  "y_mm": 600},
        "top_right":    {"tag_id": 22, "x_mm": 2400, "y_mm": 600},
        "bottom_right": {"tag_id": 20, "x_mm": 2400, "y_mm": 1400},
        "bottom_left":  {"tag_id": 21, "x_mm": 600,  "y_mm": 1400},
    },
    # Match-time correction throttling
    "correction_min_dist_mm":  50.0,   # don't correct if robot still close to last
    "correction_period_s":     1.5,    # min time between corrections
    "correction_max_jump_mm":  500.0,  # ignore vision pose if it jumps wildly
}


# ── VisionBackend ────────────────────────────────────────────────────────────

class VisionBackend:
    """
    Match-time vision pipeline. Thread-safe via internal RLock.

        be = VisionBackend()
        be.set_team('blue')
        be.set_source('0')          # USB camera index 0
        be.enable()
        # ... during match ...
        corr = be.pop_correction()  # returns (x_mm, y_mm, theta_rad) or None
        if corr: transport.fire(f'setAbsPosition({corr[0]},{corr[1]},{deg(corr[2])})')
    """

    def __init__(self, objects_file: Optional[str] = None):
        # objects_file kept for API compat with run.py — no longer used.
        self._objects_file = objects_file
        self._lock = threading.RLock()

        # ── Config ───────────────────────────────────────────────────────
        self._enabled       = False
        self._source_str: Optional[str] = None
        self._config        = dict(DEFAULT_CONFIG)
        self._team          = 'blue'         # 'blue' or 'yellow'

        # ── Match-time state ─────────────────────────────────────────────
        # Heading offset captured at recalage time:
        #     theta_otos = theta_tag_vision + heading_offset_rad
        # NaN until sync_heading() is called.
        self._heading_offset_rad: float = float('nan')
        self._heading_offset_t:    float = 0.0

        # Last applied correction (for distance throttling)
        self._last_correction_xy_mm: Optional[tuple[float, float]] = None
        self._last_correction_t:     float = 0.0

        # Pending correction queued for run.py to consume
        self._pending_correction: Optional[tuple[float, float, float]] = None
        self._pending_correction_t: float = 0.0
        # Latest vision pose (raw, before drift gating). Useful for the UI.
        self._latest_robot_pose:    Optional[dict] = None
        self._latest_opponent_pose: Optional[dict] = None
        # Latest OTOS pose snapshot (set by run.py via update_otos_pose)
        self._otos_xy_mm: Optional[tuple[float, float]] = None
        self._otos_theta_rad: Optional[float] = None

        # ── Live frame buffers (JPEG, base64-able) ──────────────────────
        self._latest_raw:  Optional[bytes] = None
        self._latest_rect: Optional[bytes] = None
        self._frame_info: dict = {
            "frame_idx": 0, "frame_count": -1, "fps": 0.0,
            "is_live": True, "source": "none", "width": 0, "height": 0,
        }
        self._detections: list = []      # raw aruco dets [{id, px, py, x_mm?, y_mm?}]
        self._anchor_status: dict = {    # which anchors are live vs cached
            "live": [], "cached": [], "missing": [],
        }

        # Encoding-on-demand: the JPEG step is skipped when no client is
        # listening (cuts ~30% CPU when nobody is watching the tab).
        self._stream_clients: int = 0

        # ── Diagnostics (visible via get_state) ─────────────────────────
        self._frames_processed: int = 0
        self._frames_encoded:   int = 0
        self._last_frame_t:     float = 0.0
        self._worker_status:    str = "starting"
        self._last_error:       Optional[str] = None
        self._init_done:        bool = False

        # ── Thread-private (touched only by _run_loop) ──────────────────
        self._source:     Optional[Any] = None
        self._detector:   Optional[Any] = None
        self._rectifier:  Optional[Any] = None
        self._tracker:    Optional[Any] = None
        self._intrinsics: Optional[Any] = None

        # ── Background thread ────────────────────────────────────────────
        self._stop_evt = threading.Event()
        self._thread = threading.Thread(
            target=self._run_loop, daemon=True, name="vision-backend",
        )
        self._thread.start()

    # ── Enable / Disable ─────────────────────────────────────────────────

    @property
    def enabled(self) -> bool:
        return self._enabled

    def enable(self) -> bool:
        if not CV2_AVAILABLE:
            return False
        self._enabled = True
        if self._source_str is not None and (
            self._source is None or not self._source.is_open
        ):
            is_image = self._source_str.lower().endswith(
                ('.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff', '.webp')
            )
            self._open_source(self._source_str, is_image=is_image)
        return True

    def disable(self):
        self._enabled = False

    # ── Streaming subscription (perf) ────────────────────────────────────

    def add_stream_client(self):
        with self._lock:
            self._stream_clients += 1

    def remove_stream_client(self):
        with self._lock:
            self._stream_clients = max(0, self._stream_clients - 1)

    @property
    def has_stream_clients(self) -> bool:
        return self._stream_clients > 0

    # ── Video source ─────────────────────────────────────────────────────

    def set_source(self, source: str) -> dict:
        """Set the video/camera/image source. Auto-enables vision so the
        worker thread starts processing.

        Returns: { ok, source, error?, is_camera, is_file, is_image } so the
        caller can surface a meaningful message in the UI.
        """
        if not CV2_AVAILABLE:
            return {'ok': False, 'error': 'OpenCV not available', 'source': source}

        # Validate source: camera index OR existing file path
        is_camera = False
        try:
            int(source)
            is_camera = True
        except (ValueError, TypeError):
            pass

        is_file = (not is_camera) and not source.lower().startswith(
            ('rtsp://', 'http://', 'https://')
        )

        # Resolve relative paths against software/vision/ for convenience —
        # the browser file-picker only gives a filename, and users commonly
        # store videos/images alongside their TwinVision settings.
        if is_file and not os.path.isabs(source):
            candidates = [
                os.path.join(_VISION_DIR, source),
                os.path.join(_VISION_DIR, 'data', source),
                os.path.abspath(source),
            ]
            for c in candidates:
                if os.path.exists(c):
                    source = os.path.abspath(c)
                    break

        is_image = is_file and source.lower().endswith(
            ('.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff', '.webp')
        )

        if is_file and not os.path.exists(source):
            return {
                'ok': False,
                'error': f'File not found: {source}',
                'source': source,
                'is_camera': is_camera,
                'is_file': is_file,
                'is_image': is_image,
            }

        self._source_str = source

        # Auto-enable vision so the worker actually starts processing.
        # Without this, set_source on a disabled backend silently stores the
        # path and the user sees no frames at all.
        was_disabled = not self._enabled
        self._enabled = True

        if self._open_source(source, is_image=is_image):
            print(f"[VisionBackend] Source opened: {source!r} "
                  f"({'image' if is_image else 'video/camera'})"
                  + (' — auto-enabled vision' if was_disabled else ''))
            return {
                'ok': True,
                'source': source,
                'is_camera': is_camera,
                'is_file': is_file,
                'is_image': is_image,
            }
        return {
            'ok': False,
            'error': f'OpenCV could not open source: {source}',
            'source': source,
            'is_camera': is_camera,
            'is_file': is_file,
            'is_image': is_image,
        }

    def _open_source(self, source: str, is_image: bool = False) -> bool:
        # Static images: cv2.VideoCapture is unreliable across builds. Read
        # the file once with cv2.imread and feed it as a synthetic 1-frame
        # "video" via _StaticImageSource.
        if is_image:
            img = cv2.imread(source)
            if img is None:
                return False
            vs = _StaticImageSource(source, img)
        else:
            try:
                src_arg = int(source)
            except (ValueError, TypeError):
                src_arg = source
            vs = VideoSource(src_arg)
            if not vs.open():
                return False
        with self._lock:
            old = self._source
            # Assign FIRST, then release the old one — so even if
            # release() raises, we don't leak a reference to a half-dead
            # capture that the worker thread could still read from.
            self._source = vs
            if old is not None:
                try:
                    old.release()
                except Exception as e:
                    print(f'[VisionBackend] old source release: {e}')
            info = vs.info
            self._frame_info.update({
                "frame_idx":   0,
                "frame_count": info.frame_count,
                "fps":         info.fps,
                "is_live":     info.is_live,
                "source":      info.source_name,
                "width":       info.width,
                "height":      info.height,
            })
            # Drop stale JPEGs from any previous source so the UI doesn't
            # flash old content while the new source warms up.
            self._latest_raw = None
            self._latest_rect = None
            self._frames_processed = 0
            self._frames_encoded = 0
            self._last_frame_t = 0.0
            self._last_error = None
        return True

    # ── Configuration ────────────────────────────────────────────────────

    def set_config(self, config: dict):
        with self._lock:
            self._config.update(config)

    def get_config(self) -> dict:
        with self._lock:
            return dict(self._config)

    # ── Team / robot tag config ──────────────────────────────────────────

    def set_team(self, team: str):
        team = (team or 'blue').lower()
        if team not in TEAM_TAG_RANGES:
            return
        with self._lock:
            if self._team == team:
                return
            self._team = team
        # Rebuild the tracker robot list so we own + opponent IDs match.
        self._reconfigure_tracker_for_team()

    @property
    def team(self) -> str:
        return self._team

    def _reconfigure_tracker_for_team(self):
        """Build a RobotConfig list for the current team. Called from the
        worker thread on team change AND once at startup."""
        if not CV2_AVAILABLE or self._tracker is None:
            return
        ranges = TEAM_TAG_RANGES[self._team]
        z_mm = float(self._config.get('robot_z_mm', 490.0))
        configs: list[RobotConfig] = []
        for tid in ranges['own']:
            configs.append(RobotConfig(
                tag_id=tid, label=f"OWN-{tid}",
                color_bgr=(0, 220, 0), z_mm=z_mm,
            ))
        for tid in ranges['opp']:
            configs.append(RobotConfig(
                tag_id=tid, label=f"OPP-{tid}",
                color_bgr=(0, 80, 255), z_mm=z_mm,
            ))
        self._tracker.set_robots(configs)

    # ── Match-time API ───────────────────────────────────────────────────

    def update_otos_pose(self, x_mm: float, y_mm: float, theta_rad: float):
        """run.py calls this whenever a fresh OTOS pose telemetry arrives.
        Used to (a) compute dist-since-last-correction and (b) snapshot a
        reference for sync_heading()."""
        with self._lock:
            self._otos_xy_mm = (float(x_mm), float(y_mm))
            self._otos_theta_rad = float(theta_rad)

    def sync_heading(self) -> dict:
        """Capture the heading offset between the OTOS and the robot tag.

        Called once after the robot's recalage routine completes. From this
        moment on, a vision-detected tag at orientation theta_tag corresponds
        to a robot heading theta_otos = theta_tag + offset.

        Returns a status dict so the caller (UI / strategy) knows whether the
        capture succeeded.
        """
        with self._lock:
            if self._latest_robot_pose is None:
                return {'ok': False, 'reason': 'robot tag not visible'}
            tag_theta = self._latest_robot_pose.get('theta_rad')
            if tag_theta is None or math.isnan(tag_theta):
                return {'ok': False, 'reason': 'robot tag heading invalid'}
            if self._otos_theta_rad is None:
                return {'ok': False, 'reason': 'no OTOS pose received yet'}
            offset = _wrap_pi(self._otos_theta_rad - tag_theta)
            self._heading_offset_rad = offset
            self._heading_offset_t = time.monotonic()
            # Reset throttling so we don't spam an immediate correction.
            self._last_correction_xy_mm = (
                self._otos_xy_mm if self._otos_xy_mm
                else (
                    self._latest_robot_pose['x_mm'],
                    self._latest_robot_pose['y_mm'],
                )
            )
            self._last_correction_t = self._heading_offset_t
            self._pending_correction = None
            return {
                'ok':           True,
                'offset_rad':   offset,
                'offset_deg':   math.degrees(offset),
                'tag_theta_rad': tag_theta,
                'otos_theta_rad': self._otos_theta_rad,
            }

    def reset_heading_offset(self):
        with self._lock:
            self._heading_offset_rad = float('nan')
            self._last_correction_xy_mm = None
            self._last_correction_t = 0.0
            self._pending_correction = None

    @property
    def heading_offset_rad(self) -> float:
        return self._heading_offset_rad

    def pop_correction(self) -> Optional[tuple[float, float, float]]:
        """Return a pending (x_mm, y_mm, theta_rad) correction, or None.
        Once popped, the pending slot is cleared; the throttling state is
        committed (last_correction_xy_mm/last_correction_t)."""
        with self._lock:
            corr = self._pending_correction
            if corr is None:
                return None
            self._pending_correction = None
            self._last_correction_xy_mm = (corr[0], corr[1])
            self._last_correction_t = self._pending_correction_t
            return corr

    # ── State / frame access ─────────────────────────────────────────────

    def get_latest_frames(self) -> tuple:
        with self._lock:
            return self._latest_raw, self._latest_rect

    def get_latest_frames_b64(self) -> tuple:
        raw, rect = self.get_latest_frames()
        return (
            base64.b64encode(raw).decode()  if raw  else None,
            base64.b64encode(rect).decode() if rect else None,
        )

    def get_frame_info(self) -> dict:
        with self._lock:
            return dict(self._frame_info)

    def get_state(self) -> dict:
        with self._lock:
            now = time.monotonic()
            last_frame_age = (now - self._last_frame_t) if self._last_frame_t else None
            return {
                "enabled":       self._enabled,
                "cv2_available": CV2_AVAILABLE,
                "intrinsics_loaded": self._intrinsics is not None,
                "intrinsics_path":   _INTRINSICS_PATH,
                "aruco_dicts":   list(ARUCO_DICTS.keys()),
                "source":        self._source_str,
                "source_open":   bool(self._source and self._source.is_open),
                "stream_clients": self._stream_clients,
                "frame_info":    dict(self._frame_info),
                "config":        dict(self._config),
                "team":          self._team,
                "detections":    list(self._detections),
                "anchor_status": dict(self._anchor_status),
                # Sanitize NaN → None: strict JSON.parse rejects NaN, and
                # Flask/json.dumps emits literal `NaN` by default.
                "robot_pose":    _json_safe(self._latest_robot_pose),
                "opponent_pose": _json_safe(self._latest_opponent_pose),
                "heading_offset_rad": (None
                                        if math.isnan(self._heading_offset_rad)
                                        else self._heading_offset_rad),
                "heading_offset_deg": (math.degrees(self._heading_offset_rad)
                                       if not math.isnan(self._heading_offset_rad)
                                       else None),
                "last_correction": (
                    {'x_mm': self._last_correction_xy_mm[0],
                     'y_mm': self._last_correction_xy_mm[1],
                     'age_s': now - self._last_correction_t}
                    if self._last_correction_xy_mm else None
                ),
                "has_homography": (
                    self._rectifier.has_homography if self._rectifier else False
                ),
                "homography_fresh": (
                    self._rectifier.homography_is_fresh if self._rectifier else False
                ),
                # Diagnostics
                "diag": {
                    "worker_status":     self._worker_status,
                    "init_done":         self._init_done,
                    "frames_processed":  self._frames_processed,
                    "frames_encoded":    self._frames_encoded,
                    "last_frame_age_s":  last_frame_age,
                    "last_error":        self._last_error,
                },
            }

    # ── Background processing thread ─────────────────────────────────────

    def _load_intrinsics(self) -> Optional[Any]:
        """Load CameraIntrinsics from disk. Returns None if missing/invalid."""
        if not os.path.exists(_INTRINSICS_PATH):
            print(f"[VisionBackend] Intrinsics not found at {_INTRINSICS_PATH} — "
                  f"falling back to homography-only rectification.")
            return None
        try:
            intr = CameraIntrinsics.load(_INTRINSICS_PATH)
            print(f"[VisionBackend] Intrinsics loaded ({intr.image_size}, "
                  f"rms={intr.rms_px:.3f}px)")
            return intr
        except Exception as e:
            print(f"[VisionBackend] Intrinsics load error: {e}")
            return None

    def _run_loop(self):
        if not CV2_AVAILABLE:
            self._worker_status = "cv2 unavailable"
            self._last_error = "OpenCV / aruco not importable"
            return

        # Initialise components in the worker thread (cv2 objects are not
        # safely shareable across threads). Wrap everything so the thread
        # never dies silently — errors stay visible via get_state().
        try:
            self._worker_status = "init: detector"
            self._detector = ArucoDetector(self._config.get("dict", "4x4_50"))
            self._worker_status = "init: rectifier"
            self._rectifier = TableRectifier(
                CornerConfig.from_dict(self._config.get("anchors", DEFAULT_CONFIG["anchors"]))
            )
            self._worker_status = "init: intrinsics"
            self._intrinsics = self._load_intrinsics()
            if self._intrinsics is not None:
                self._rectifier.set_intrinsics(self._intrinsics)
            self._worker_status = "init: tracker"
            self._tracker = RobotTracker(
                cam_mode='auto',         # use solvePnP via intrinsics
                smooth_calib=True,
                trail_len=40,
            )
            if self._intrinsics is not None:
                self._tracker.set_camera_intrinsics(
                    self._intrinsics.K, self._intrinsics.dist,
                )
            self._reconfigure_tracker_for_team()
            self._init_done = True
            self._worker_status = "idle (no source)"
            print("[VisionBackend] worker thread initialized — waiting for source")
        except Exception as e:
            self._worker_status = "init failed"
            self._last_error = f"init: {type(e).__name__}: {e}"
            print(f"[VisionBackend] FATAL init error: {e}")
            import traceback; traceback.print_exc()
            return

        last_step_t = 0.0

        while not self._stop_evt.is_set():
            try:
                if not self._enabled:
                    if self._worker_status not in ("disabled", "init failed"):
                        self._worker_status = "disabled"
                    time.sleep(0.05)
                    continue
                if self._source is None or not self._source.is_open:
                    self._worker_status = "no source"
                    time.sleep(0.05)
                    continue

                cfg     = self._config
                fps_lim = max(1, int(cfg.get("fps_limit", 25)))
                now     = time.monotonic()

                if (now - last_step_t) < (1.0 / fps_lim):
                    time.sleep(0.004)
                    continue
                last_step_t = now

                # Update detector dictionary if config changed
                dict_name = cfg.get("dict", "4x4_50")
                if self._detector.dictionary_name != dict_name:
                    self._detector.set_dictionary(dict_name)

                # Apply anchor config changes
                try:
                    new_cfg = CornerConfig.from_dict(
                        cfg.get("anchors", DEFAULT_CONFIG["anchors"])
                    )
                    if new_cfg.to_dict() != self._rectifier.config.to_dict():
                        self._rectifier.config = new_cfg
                        self._rectifier.clear_anchor_cache()
                except Exception:
                    pass

                # Read frame
                frame = self._source.read()
                if frame is None:
                    fi = self._frame_info
                    if fi.get("frame_count", -1) > 0:
                        self._source.seek(0)
                    else:
                        # Static images / cameras with hiccups: keep status
                        # truthful so the UI knows we're idle waiting.
                        self._worker_status = "no frame"
                        time.sleep(0.05)
                    continue

                self._process_frame(frame, cfg)
                self._frames_processed += 1
                self._last_frame_t = time.monotonic()
                if self._frames_processed == 1:
                    print(f"[VisionBackend] First frame processed "
                          f"(shape={frame.shape}, source={self._source_str!r})")
                self._worker_status = "running"
            except Exception as e:
                # Never let one bad frame kill the loop. Log + keep going.
                self._last_error = f"{type(e).__name__}: {e}"
                self._worker_status = "frame error (recovering)"
                print(f"[VisionBackend] frame error: {e}")
                import traceback; traceback.print_exc()
                time.sleep(0.1)

    def _process_frame(self, frame, cfg: dict):
        """Detect → rectify → track → correct → encode."""
        h, w = frame.shape[:2]

        # 1. ArUco detection
        result = self._detector.detect(frame)

        # 2. Rectifier update — pose-based when intrinsics available, else
        #    fall back to 4-anchor homography. Anchor caching is built-in
        #    so we survive single-anchor occlusions.
        if self._intrinsics is not None:
            self._rectifier.update_pose(result)
        else:
            self._rectifier.update(result)

        # 3. Robot tracking (own + opponent)
        states: dict[int, RobotState] = {}
        if self._rectifier.has_homography:
            try:
                states = self._tracker.process(result, self._rectifier, (w, h))
            except Exception as e:
                # Pose-degenerate frames can break solvePnP; non-fatal.
                states = {}

        # 4. Build anchor status snapshot (UI feedback)
        anchor_live, anchor_cached, anchor_missing = [], [], []
        for a in self._rectifier.config.anchors():
            if result.get_center_for_id(a.tag_id) is not None:
                anchor_live.append(a.tag_id)
            elif a.tag_id in self._rectifier._anchor_pixels_cache:
                anchor_cached.append(a.tag_id)
            else:
                anchor_missing.append(a.tag_id)

        # 5. Build raw detection list (for UI debug)
        new_dets = []
        for aruco_id, corners in zip(result.ids, result.corners):
            pts = corners.reshape(4, 2)
            cx = float(pts[:, 0].mean())
            cy = float(pts[:, 1].mean())
            det = {"id": int(aruco_id),
                   "px": round(cx, 1), "py": round(cy, 1)}
            if self._rectifier.has_homography:
                tp = self._rectifier.raw_pixel_to_table_mm(int(cx), int(cy))
                if tp:
                    det["x_mm"] = round(tp[0], 1)
                    det["y_mm"] = round(tp[1], 1)
            new_dets.append(det)

        # 6. Match-time correction logic — pick the OWN robot tag if visible
        own_pose, opp_pose = self._extract_team_poses(states)

        if own_pose is not None:
            self._maybe_queue_correction(own_pose)

        # 7. Frame encoding — when a client is subscribed, OR when we have
        #    no buffered frame yet (so a late-joining client doesn't see a
        #    blank pane while the encoder waits for the next subscription
        #    event to arrive). When no client is subscribed and a buffered
        #    frame already exists, skip encoding to save CPU.
        encoded_this_frame = False
        need_first_frame = self._latest_raw is None
        if self.has_stream_clients or need_first_frame:
            raw_bytes, rect_bytes = self._encode_frames(
                frame, result, states, cfg,
            )
            if raw_bytes is not None:
                self._frames_encoded += 1
                encoded_this_frame = True

        # 8. Commit shared state
        with self._lock:
            if encoded_this_frame:
                self._latest_raw = raw_bytes
                self._latest_rect = rect_bytes
            self._detections = new_dets
            self._anchor_status = {
                "live": anchor_live,
                "cached": anchor_cached,
                "missing": anchor_missing,
            }
            self._latest_robot_pose = own_pose
            self._latest_opponent_pose = opp_pose
            info = self._source.info if self._source else None
            if info:
                self._frame_info["frame_idx"]   = self._source.current_frame_idx
                self._frame_info["frame_count"] = info.frame_count
                self._frame_info["fps"]         = info.fps
                self._frame_info["is_live"]     = info.is_live

    # -- match-time helpers ----------------------------------------------

    def _extract_team_poses(
        self, states: dict[int, "RobotState"],
    ) -> tuple[Optional[dict], Optional[dict]]:
        """From the tracker output, pick a single OWN pose + a single OPP
        pose. We take the first detected tag in each ID range. The naive
        `pos_mm` already includes parallax + Kalman smoothing."""
        ranges = TEAM_TAG_RANGES.get(self._team, TEAM_TAG_RANGES['blue'])
        own = opp = None
        for tid, st in states.items():
            # Skip predict-only states (no detection this frame)
            if math.isnan(st.theta_rad):
                continue
            entry = {
                'tag_id':    tid,
                'x_mm':      float(st.pos_mm[0]),
                'y_mm':      float(st.pos_mm[1]),
                'theta_rad': float(st.theta_rad),
            }
            if tid in ranges['own'] and own is None:
                own = entry
            elif tid in ranges['opp'] and opp is None:
                opp = entry
        return own, opp

    def _maybe_queue_correction(self, pose: dict):
        """Apply the throttling rules. If everything passes, write the
        corrected (x, y, theta) to self._pending_correction, where pop_correction()
        can pick it up."""
        cfg = self._config
        min_dist  = float(cfg.get('correction_min_dist_mm', 50.0))
        period_s  = float(cfg.get('correction_period_s',     1.5))
        max_jump  = float(cfg.get('correction_max_jump_mm',  500.0))

        with self._lock:
            offset = self._heading_offset_rad
            if math.isnan(offset):
                # Heading not yet calibrated — never auto-correct
                return
            now = time.monotonic()
            if now - self._last_correction_t < period_s:
                return
            # Distance gate vs last correction position
            if self._last_correction_xy_mm is not None:
                dx = pose['x_mm'] - self._last_correction_xy_mm[0]
                dy = pose['y_mm'] - self._last_correction_xy_mm[1]
                d = math.hypot(dx, dy)
                if d < min_dist:
                    return
            # Sanity-check vs OTOS pose: if vision and OTOS disagree by more
            # than max_jump, ignore this frame (vision likely confused).
            if self._otos_xy_mm is not None:
                dx = pose['x_mm'] - self._otos_xy_mm[0]
                dy = pose['y_mm'] - self._otos_xy_mm[1]
                if math.hypot(dx, dy) > max_jump:
                    return
            # All good — queue.
            theta_rad = _wrap_pi(pose['theta_rad'] + offset)
            self._pending_correction = (
                pose['x_mm'], pose['y_mm'], theta_rad,
            )
            self._pending_correction_t = now

    # -- encoding helpers ------------------------------------------------

    def _encode_frames(self, frame, result, states, cfg):
        """Build raw + rectified annotated JPEGs. Returns (raw_bytes, rect_bytes)."""
        quality = [cv2.IMWRITE_JPEG_QUALITY, int(cfg.get("jpeg_quality", 70))]

        raw_out = frame
        if cfg.get('show_aruco', True):
            raw_out = self._detector.draw(
                frame, result,
                draw_ids=cfg.get('show_ids', True),
                draw_rejected=False,
            )
        else:
            raw_out = frame.copy()

        if cfg.get('show_table_overlay', True) and self._rectifier.has_homography:
            raw_out = self._rectifier.draw_overlay_on_raw(raw_out, result)

        # Status badge
        badge = ('LIVE' if self._rectifier.homography_is_fresh
                 else ('CACHE' if self._rectifier.homography_is_cached
                       else 'NO H'))
        badge_color = ((0, 200, 80)  if badge == 'LIVE'
                       else ((0, 160, 255) if badge == 'CACHE'
                             else (0, 0, 200)))
        cv2.putText(raw_out, badge, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, badge_color, 2, cv2.LINE_AA)

        # Heading offset badge
        if math.isnan(self._heading_offset_rad):
            cv2.putText(raw_out, 'HEAD: SYNC?', (8, 44),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 220), 1, cv2.LINE_AA)
        else:
            cv2.putText(
                raw_out,
                f'HEAD off={math.degrees(self._heading_offset_rad):+.1f}deg',
                (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 200, 80), 1, cv2.LINE_AA,
            )

        _, raw_j = cv2.imencode('.jpg', raw_out, quality)
        raw_bytes = bytes(raw_j)

        # Rectified (top-down) view
        rect_bytes = None
        if self._rectifier.has_homography:
            rect_out = self._rectifier.rectify(frame)
            if rect_out is not None:
                if cfg.get('show_grid', True):
                    rect_out = self._rectifier.draw_grid(rect_out)
                rect_out = self._draw_robots_on_rect(rect_out, states)
                _, rect_j = cv2.imencode('.jpg', rect_out, quality)
                rect_bytes = bytes(rect_j)

        return raw_bytes, rect_bytes

    def _draw_robots_on_rect(self, rect_img, states: dict):
        """Draw OWN + OPP robots on the rectified BEV with their heading."""
        if rect_img is None:
            return rect_img
        ranges = TEAM_TAG_RANGES.get(self._team, TEAM_TAG_RANGES['blue'])
        for tid, st in states.items():
            x, y = self._rectifier.table_mm_to_rectified(
                float(st.pos_mm[0]), float(st.pos_mm[1]),
            )
            in_own = tid in ranges['own']
            color = (0, 220, 0) if in_own else (0, 80, 255)
            cv2.circle(rect_img, (x, y), 12, color, 2)
            cv2.circle(rect_img, (x, y), 3, color, -1)
            # Heading arrow
            if not math.isnan(st.theta_rad):
                arrow_len_px = 28
                hx = int(x + arrow_len_px * math.cos(st.theta_rad))
                hy = int(y + arrow_len_px * math.sin(st.theta_rad))
                cv2.arrowedLine(rect_img, (x, y), (hx, hy), color, 2,
                                tipLength=0.3)
            label = f"{'OWN' if in_own else 'OPP'} #{tid}"
            cv2.putText(rect_img, label, (x + 14, y + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
        return rect_img


class _StaticImageSource:
    """Mimics core.video_source.VideoSource for a single static image. The
    same image is returned indefinitely (so the ArUco/tracker pipeline can
    keep refreshing the JPEG stream and the user sees a stable frame)."""

    def __init__(self, path: str, img):
        from core.video_source import VideoInfo
        h, w = img.shape[:2]
        self._img = img
        self._info = VideoInfo(
            width=w, height=h, fps=10.0,
            frame_count=-1, source_name=path,
        )
        self._opened = True
        self._idx = 0

    @property
    def is_open(self) -> bool:
        return self._opened

    @property
    def info(self):
        return self._info

    @property
    def current_frame_idx(self) -> int:
        return self._idx

    def read(self):
        if not self._opened:
            return None
        self._idx += 1
        # Return a copy so downstream mutations don't poison the cache.
        return self._img.copy()

    def seek(self, frame_idx: int) -> bool:
        return False

    def grab_current_frame(self):
        return self._img.copy() if self._opened else None

    def release(self):
        self._opened = False


def _json_safe(v):
    """Recursively replace NaN/Inf floats with None so the result survives
    `JSON.parse` in the browser. Strict JSON disallows NaN/Inf and Flask's
    `jsonify` emits them as the literal `NaN` / `Infinity` strings, which
    `JSON.parse` then rejects."""
    if v is None:
        return None
    if isinstance(v, float):
        return None if (math.isnan(v) or math.isinf(v)) else v
    if isinstance(v, dict):
        return {k: _json_safe(val) for k, val in v.items()}
    if isinstance(v, (list, tuple)):
        return [_json_safe(x) for x in v]
    return v


def _wrap_pi(a: float) -> float:
    """Wrap an angle into (-pi, pi]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a
