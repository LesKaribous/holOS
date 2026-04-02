"""
VisionBackend — Vision processing backend for holOS.

Runs in a dedicated daemon thread. Thread-safe via internal RLock.

Features:
  - Video source: live camera (index) or video file (mp4/mkv)
  - ArUco marker detection (configurable dictionary)
  - Camera undistortion (standard or fisheye calibration model)
  - Table rectification via homography (4 anchor ArUco tags)
  - Object registry: track positions + colors of known game objects
  - JPEG frame encoding for SocketIO streaming to the web UI

Object Registry:
  Each object has: id, name, type, aruco_id, color,
                   initial_pos, current_pos, on_table, last_seen_ms
  Objects tagged with aruco_id are auto-positioned from detection.
  Color is set manually via the holOS UI or via color detection helper.
"""

from __future__ import annotations

import base64
import json
import os
import sys
import threading
import time
from typing import Any, Optional

# ── Vision core path ──────────────────────────────────────────────────────────
_VISION_SRC = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'vision', 'src'
)
if _VISION_SRC not in sys.path:
    sys.path.insert(0, os.path.abspath(_VISION_SRC))

CV2_AVAILABLE = False
try:
    import cv2
    import numpy as np
    from core.aruco_detector import ArucoDetector, ARUCO_DICTS, DetectionResult
    from core.table_rectifier import TableRectifier, CornerConfig, TagAnchor
    from core.video_source import VideoSource
    from core.calibration import CalibrationData
    CV2_AVAILABLE = True
except ImportError as _e:
    print(f"[VisionBackend] OpenCV / vision core not available: {_e}")
    ArucoDetector = TableRectifier = VideoSource = CalibrationData = None
    ARUCO_DICTS = {}


# ── Constants ─────────────────────────────────────────────────────────────────

OBJECT_COLORS = ["unknown", "red", "green", "blue", "yellow", "white", "black", "brown"]

# HSV thresholds for basic color classification (used when auto-detect is enabled)
_COLOR_RANGES_HSV = {
    "red":    [([0, 80, 60],    [10, 255, 255]),
               ([160, 80, 60],  [180, 255, 255])],
    "green":  [([40, 60, 50],   [85, 255, 255])],
    "blue":   [([95, 60, 50],   [135, 255, 255])],
    "yellow": [([20, 80, 80],   [38, 255, 255])],
    "white":  [([0, 0, 190],    [180, 30, 255])],
    "black":  [([0, 0, 0],      [180, 255, 50])],
    "brown":  [([8, 50, 30],    [22, 200, 160])],
}

DEFAULT_CONFIG = {
    "dict":                "4x4_50",
    "undistort":           False,
    "show_aruco":          True,
    "show_ids":            True,
    "show_rejected":       False,
    "show_grid":           True,
    "show_table_overlay":  True,
    "auto_color":          False,   # auto-detect color from pixel region
    "fps_limit":           25,
    "jpeg_quality":        70,
    "anchors": {
        "top_left":     {"tag_id": 23, "x_mm": 0,    "y_mm": 0},
        "top_right":    {"tag_id": 22, "x_mm": 3000, "y_mm": 0},
        "bottom_right": {"tag_id": 20, "x_mm": 3000, "y_mm": 2000},
        "bottom_left":  {"tag_id": 21, "x_mm": 0,    "y_mm": 2000},
    },
    # ── Processing pipeline ───────────────────────────────────────────────────
    # proc_mode: "none" | "binarize" | "color_mask" | "canny" | "blur"
    "proc_mode":           "none",
    "binary_method":       "global",   # "global" | "otsu" | "adaptive_mean" | "adaptive_gaussian"
    "binary_threshold":    128,        # 0-255 (ignored for otsu)
    "binary_block_size":   11,         # odd int, for adaptive methods
    "binary_invert":       False,
    "color_target":        "red",      # one of OBJECT_COLORS
    "color_lo_h":          0,          # HSV low H (0-179)
    "color_lo_s":          80,         # HSV low S (0-255)
    "color_lo_v":          60,         # HSV low V (0-255)
    "color_hi_h":          10,         # HSV high H (0-179)
    "color_hi_s":          255,        # HSV high S (0-255)
    "color_hi_v":          255,        # HSV high V (0-255)
    "color_hi_h2":         -1,         # second hue range high (-1 = disabled, for red wrap)
    "color_lo_h2":         160,        # second hue range low
    "color_show_masked":   True,       # True = masked image, False = binary mask only
    "blur_kernel":         5,          # Gaussian blur kernel size (odd int, ≥ 1)
    "canny_low":           50,         # Canny lower threshold
    "canny_high":          150,        # Canny upper threshold
    "morph_op":            "none",     # "none" | "erode" | "dilate" | "open" | "close"
    "morph_kernel":        3,          # morphological kernel size
}

# BGR colors for drawing objects on the rectified view
_DRAW_COLORS = {
    "red":     (0,   0,   220),
    "green":   (0,   180, 0),
    "blue":    (220, 0,   0),
    "yellow":  (0,   200, 255),
    "white":   (240, 240, 240),
    "black":   (60,  60,  60),
    "brown":   (30,  80,  130),
    "unknown": (160, 160, 160),
}


# ── VisionBackend ─────────────────────────────────────────────────────────────

class VisionBackend:
    """
    Vision processing backend.  Start with enable(), control via methods.
    All public methods are thread-safe.
    """

    def __init__(self, objects_file: str):
        self._objects_file = objects_file
        self._lock = threading.RLock()

        # ── State (protected by _lock) ──────────────────────────────────────
        self._enabled       = False
        self._source_str: Optional[str] = None
        self._config        = dict(DEFAULT_CONFIG)
        self._latest_raw: Optional[bytes]  = None
        self._latest_rect: Optional[bytes] = None
        self._latest_proc: Optional[bytes] = None   # processed pipeline output
        self._frame_info: dict = {
            "frame_idx": 0, "frame_count": -1, "fps": 0.0,
            "is_live": True, "source": "none", "width": 0, "height": 0,
        }
        self._detections: list = []      # list of {id, px, py, x_mm?, y_mm?}
        self._objects: list    = []
        self._calibration_dict: Optional[dict] = None

        # ── Thread-private state (touched only by _run_loop) ────────────────
        self._source: Optional[Any]      = None
        self._detector: Optional[Any]    = None
        self._rectifier: Optional[Any]   = None
        self._calibration: Optional[Any] = None

        self._paused              = False
        self._seek_target: Optional[int] = None
        self._step_delta: Optional[int]  = None
        self._speed: float        = 1.0

        self._load_objects()

        self._thread = threading.Thread(
            target=self._run_loop, daemon=True, name="vision-backend"
        )
        self._thread.start()

    # ── Enable / Disable ──────────────────────────────────────────────────────

    @property
    def enabled(self) -> bool:
        return self._enabled

    def enable(self) -> bool:
        if not CV2_AVAILABLE:
            return False
        self._enabled = True
        if self._source_str is not None:
            self._open_source(self._source_str)
        return True

    def disable(self):
        self._enabled = False

    # ── Video source ──────────────────────────────────────────────────────────

    def set_source(self, source: str) -> bool:
        """source = "0", "1", "/path/to/file.mp4", or RTSP URL."""
        if not CV2_AVAILABLE:
            return False
        self._source_str = source
        if self._enabled:
            return self._open_source(source)
        return True

    def _open_source(self, source: str) -> bool:
        try:
            src_arg = int(source)
        except (ValueError, TypeError):
            src_arg = source

        vs = VideoSource(src_arg)
        if not vs.open():
            return False

        with self._lock:
            if self._source is not None:
                self._source.release()
            self._source = vs
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
            self._paused = False
        return True

    # ── Playback controls ─────────────────────────────────────────────────────

    def play_pause(self):
        self._paused = not self._paused

    def seek(self, frame_idx: int):
        self._seek_target = int(frame_idx)

    def step(self, delta: int):
        self._step_delta = int(delta)

    def set_speed(self, speed: float):
        self._speed = max(0.1, min(8.0, float(speed)))

    # ── Configuration ─────────────────────────────────────────────────────────

    def set_config(self, config: dict):
        with self._lock:
            self._config.update(config)

    def get_config(self) -> dict:
        with self._lock:
            return dict(self._config)

    def set_calibration_dict(self, calib: Optional[dict]):
        """Load calibration from a serialised dict (as produced by CalibrationData.to_dict())."""
        with self._lock:
            self._calibration_dict = calib
        # Signal thread to reload calibration
        self._calibration = None

    # ── Object registry ───────────────────────────────────────────────────────

    def _load_objects(self):
        if os.path.exists(self._objects_file):
            try:
                with open(self._objects_file, encoding="utf-8") as f:
                    self._objects = json.load(f)
                return
            except Exception:
                pass
        self._objects = []

    def save_objects(self):
        try:
            with open(self._objects_file, "w", encoding="utf-8") as f:
                json.dump(self._objects, f, indent=2, ensure_ascii=False)
        except Exception as e:
            print(f"[VisionBackend] Could not save objects: {e}")

    def get_objects(self) -> list:
        with self._lock:
            return list(self._objects)

    def set_objects(self, objects: list):
        with self._lock:
            self._objects = list(objects)
        self.save_objects()

    def update_object(self, obj_id: str, patch: dict):
        """Partial update of an object by id."""
        with self._lock:
            for obj in self._objects:
                if obj.get("id") == obj_id:
                    obj.update(patch)
                    break
        self.save_objects()

    def delete_object(self, obj_id: str):
        with self._lock:
            self._objects = [o for o in self._objects if o.get("id") != obj_id]
        self.save_objects()

    def get_object_color(self, identifier) -> str:
        """Query color by object id, aruco_id (int), or name."""
        with self._lock:
            for obj in self._objects:
                if (obj.get("id") == identifier
                        or obj.get("aruco_id") == identifier
                        or obj.get("name") == identifier):
                    return obj.get("color", "unknown")
        return "unknown"

    def _update_object_from_detection(self, aruco_id: int, x_mm: float, y_mm: float):
        now_ms = int(time.monotonic() * 1000)
        with self._lock:
            for obj in self._objects:
                if obj.get("aruco_id") == aruco_id:
                    obj["current_pos"]  = {"x": round(x_mm, 1), "y": round(y_mm, 1)}
                    obj["last_seen_ms"] = now_ms
                    obj["on_table"]     = True
                    break

    # ── Frame access ──────────────────────────────────────────────────────────

    def get_latest_frames(self) -> tuple:
        """Returns (raw_jpg_bytes, rect_jpg_bytes_or_None, proc_jpg_bytes_or_None)."""
        with self._lock:
            return self._latest_raw, self._latest_rect, self._latest_proc

    def get_latest_frames_b64(self) -> tuple:
        """Returns (raw_b64_str_or_None, rect_b64_str_or_None, proc_b64_str_or_None)."""
        raw, rect, proc = self.get_latest_frames()
        return (
            base64.b64encode(raw).decode()  if raw  else None,
            base64.b64encode(rect).decode() if rect else None,
            base64.b64encode(proc).decode() if proc else None,
        )

    def get_frame_info(self) -> dict:
        with self._lock:
            return dict(self._frame_info)

    def get_state(self) -> dict:
        with self._lock:
            return {
                "enabled":       self._enabled,
                "cv2_available": CV2_AVAILABLE,
                "aruco_dicts":   list(ARUCO_DICTS.keys()),
                "source":        self._source_str,
                "frame_info":    dict(self._frame_info),
                "paused":        self._paused,
                "speed":         self._speed,
                "config":        dict(self._config),
                "detections":    list(self._detections),
                "has_homography": (self._rectifier.has_homography
                                   if self._rectifier else False),
                "homography_fresh": (self._rectifier.homography_is_fresh
                                     if self._rectifier else False),
            }

    # ── Color detection helper ────────────────────────────────────────────────

    @staticmethod
    def _classify_color_hsv(frame_bgr, cx: int, cy: int, radius: int = 20) -> str:
        """Sample a circular region around (cx, cy) and classify its dominant color."""
        if not CV2_AVAILABLE:
            return "unknown"
        h, w = frame_bgr.shape[:2]
        x1, y1 = max(0, cx - radius), max(0, cy - radius)
        x2, y2 = min(w, cx + radius), min(h, cy + radius)
        roi = frame_bgr[y1:y2, x1:x2]
        if roi.size == 0:
            return "unknown"
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        best_color, best_score = "unknown", 0
        for color_name, ranges in _COLOR_RANGES_HSV.items():
            score = 0
            for lo, hi in ranges:
                mask = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
                score += int(mask.sum() / 255)
            if score > best_score:
                best_score = score
                best_color = color_name
        return best_color if best_score > 30 else "unknown"

    # ── Processing pipeline ───────────────────────────────────────────────────

    def _apply_processing(self, frame_bgr, cfg: dict) -> Optional[bytes]:
        """
        Apply the selected processing pipeline to frame_bgr.
        Returns JPEG bytes of the result, or None if proc_mode is "none".
        """
        if not CV2_AVAILABLE:
            return None

        mode = cfg.get("proc_mode", "none")
        if mode == "none":
            return None

        quality = [cv2.IMWRITE_JPEG_QUALITY, cfg.get("jpeg_quality", 70)]

        # ── 1. Optional blur pre-step (applied before everything when mode == blur) ──
        if mode == "blur":
            k = int(cfg.get("blur_kernel", 5))
            k = max(1, k if k % 2 == 1 else k + 1)
            out = cv2.GaussianBlur(frame_bgr, (k, k), 0)
            _, enc = cv2.imencode(".jpg", out, quality)
            return bytes(enc)

        # ── 2. Binarize ────────────────────────────────────────────────────────
        if mode == "binarize":
            gray   = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
            method = cfg.get("binary_method", "global")
            invert = cfg.get("binary_invert", False)
            thresh = int(cfg.get("binary_threshold", 128))
            bsize  = int(cfg.get("binary_block_size", 11))
            bsize  = max(3, bsize if bsize % 2 == 1 else bsize + 1)
            tflag  = cv2.THRESH_BINARY_INV if invert else cv2.THRESH_BINARY

            if method == "otsu":
                _, bin_img = cv2.threshold(
                    gray, 0, 255, tflag | cv2.THRESH_OTSU)
            elif method in ("adaptive_mean", "adaptive_gaussian"):
                adaptive_m = (cv2.ADAPTIVE_THRESH_MEAN_C
                              if method == "adaptive_mean"
                              else cv2.ADAPTIVE_THRESH_GAUSSIAN_C)
                bin_img = cv2.adaptiveThreshold(
                    gray, 255, adaptive_m, tflag, bsize, 2)
            else:  # global
                _, bin_img = cv2.threshold(gray, thresh, 255, tflag)

            # Apply morphological op if configured
            bin_img = self._apply_morph(bin_img, cfg)
            out = cv2.cvtColor(bin_img, cv2.COLOR_GRAY2BGR)
            _, enc = cv2.imencode(".jpg", out, quality)
            return bytes(enc)

        # ── 3. Color mask ──────────────────────────────────────────────────────
        if mode == "color_mask":
            target  = cfg.get("color_target", "red")
            lo  = np.array([int(cfg.get("color_lo_h", 0)),
                             int(cfg.get("color_lo_s", 80)),
                             int(cfg.get("color_lo_v", 60))], np.uint8)
            hi  = np.array([int(cfg.get("color_hi_h", 10)),
                             int(cfg.get("color_hi_s", 255)),
                             int(cfg.get("color_hi_v", 255))], np.uint8)
            lo2_h = int(cfg.get("color_lo_h2", 160))
            hi2_h = int(cfg.get("color_hi_h2", -1))

            hsv  = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lo, hi)

            # Second hue range (for red which wraps around 0/180)
            if hi2_h > 0:
                lo2 = np.array([lo2_h, lo[1], lo[2]], np.uint8)
                hi2 = np.array([hi2_h, hi[1], hi[2]], np.uint8)
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo2, hi2))

            mask = self._apply_morph(mask, cfg)

            if cfg.get("color_show_masked", True):
                out = cv2.bitwise_and(frame_bgr, frame_bgr, mask=mask)
            else:
                out = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

            _, enc = cv2.imencode(".jpg", out, quality)
            return bytes(enc)

        # ── 4. Canny edge detect ───────────────────────────────────────────────
        if mode == "canny":
            gray  = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
            low   = int(cfg.get("canny_low", 50))
            high  = int(cfg.get("canny_high", 150))
            edges = cv2.Canny(gray, low, high)
            edges = self._apply_morph(edges, cfg)
            out   = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            _, enc = cv2.imencode(".jpg", out, quality)
            return bytes(enc)

        return None

    @staticmethod
    def _apply_morph(img, cfg: dict):
        """Apply morphological operation (erode/dilate/open/close) if configured."""
        op_name = cfg.get("morph_op", "none")
        if op_name == "none" or not CV2_AVAILABLE:
            return img
        k = max(1, int(cfg.get("morph_kernel", 3)))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        ops = {
            "erode":  cv2.MORPH_ERODE,
            "dilate": cv2.MORPH_DILATE,
            "open":   cv2.MORPH_OPEN,
            "close":  cv2.MORPH_CLOSE,
        }
        if op_name in ops:
            return cv2.morphologyEx(img, ops[op_name], kernel)
        return img

    # ── Background processing thread ──────────────────────────────────────────

    def _run_loop(self):
        if not CV2_AVAILABLE:
            return

        # Initialise components (in-thread, avoids sharing cv2 objects across threads)
        self._detector  = ArucoDetector(self._config.get("dict", "4x4_50"))
        self._rectifier = TableRectifier(
            CornerConfig.from_dict(self._config.get("anchors", DEFAULT_CONFIG["anchors"]))
        )

        last_frame_t = 0.0

        while True:
            if not self._enabled or self._source is None or not self._source.is_open:
                time.sleep(0.05)
                continue

            cfg     = self._config      # snapshot (dict reads are GIL-safe)
            fps_lim = max(1, cfg.get("fps_limit", 25))
            now     = time.monotonic()

            # ── Rate limiting ─────────────────────────────────────────────
            if (now - last_frame_t) < (1.0 / fps_lim):
                time.sleep(0.004)
                continue
            last_frame_t = now

            # ── Reload calibration if dict was updated ────────────────────
            if self._calibration is None and self._calibration_dict is not None:
                try:
                    cd = self._calibration_dict
                    self._calibration = CalibrationData(
                        camera_matrix=np.array(cd["camera_matrix"]),
                        dist_coeffs=np.array(cd["dist_coeffs"]),
                        rms_error=cd.get("rms_error", 0.0),
                        image_size=tuple(cd.get("image_size", [0, 0])),
                        model=cd.get("model", "standard"),
                    )
                except Exception as e:
                    print(f"[VisionBackend] Calibration load error: {e}")
                    self._calibration_dict = None

            # ── Apply anchor config changes ───────────────────────────────
            try:
                new_cfg = CornerConfig.from_dict(
                    cfg.get("anchors", DEFAULT_CONFIG["anchors"])
                )
                if new_cfg.to_dict() != self._rectifier.config.to_dict():
                    self._rectifier.config = new_cfg
            except Exception:
                pass

            # ── Handle seek / step requests ───────────────────────────────
            if self._seek_target is not None:
                self._source.seek(self._seek_target)
                self._seek_target = None

            if self._step_delta is not None:
                cur = self._source.current_frame_idx
                self._source.seek(cur + self._step_delta)
                self._step_delta = None
                frame = self._source.read()
                self._paused = True
                if frame is not None:
                    self._process_frame(frame, cfg)
                continue

            # ── Pause: re-render current frame ────────────────────────────
            fi = self._frame_info
            if self._paused and fi.get("frame_count", -1) > 0:
                frame = self._source.grab_current_frame()
                if frame is not None:
                    self._process_frame(frame, cfg)
                time.sleep(0.04)
                continue

            # ── Read next frame ───────────────────────────────────────────
            frame = self._source.read()
            if frame is None:
                # End of file → loop
                if fi.get("frame_count", -1) > 0:
                    self._source.seek(0)
                else:
                    time.sleep(0.1)
                continue

            self._process_frame(frame, cfg)

    def _process_frame(self, frame, cfg: dict):
        """Full processing pipeline for a single frame."""
        # 1. Undistort
        if cfg.get("undistort") and self._calibration is not None:
            frame = self._calibration.undistort(frame)

        # 2. ArUco detection
        result = None
        if cfg.get("show_aruco", True):
            dict_name = cfg.get("dict", "4x4_50")
            self._detector.set_dictionary(dict_name)
            result = self._detector.detect(frame)

        # 3. Update homography
        if result is not None:
            self._rectifier.update(result)

        # 4. Update object positions + auto-color
        new_detections = []
        if result is not None:
            for aruco_id, corners in zip(result.ids, result.corners):
                pts = corners.reshape(4, 2)
                cx  = float(pts[:, 0].mean())
                cy  = float(pts[:, 1].mean())
                det = {"id": aruco_id, "px": round(cx, 1), "py": round(cy, 1)}

                if self._rectifier.has_homography:
                    tp = self._rectifier.raw_pixel_to_table_mm(int(cx), int(cy))
                    if tp:
                        det["x_mm"] = round(tp[0], 1)
                        det["y_mm"] = round(tp[1], 1)
                        self._update_object_from_detection(aruco_id, tp[0], tp[1])

                # Auto-color from pixel region around tag
                if cfg.get("auto_color", False):
                    color = self._classify_color_hsv(frame, int(cx), int(cy))
                    if color != "unknown":
                        self._update_object_color_if_unknown(aruco_id, color)

                new_detections.append(det)

        # 5. Draw on raw frame
        raw_out = frame.copy()
        if result is not None:
            raw_out = self._detector.draw(
                raw_out, result,
                draw_ids=cfg.get("show_ids", True),
                draw_rejected=cfg.get("show_rejected", False),
            )
        if cfg.get("show_table_overlay", True) and self._rectifier.has_homography:
            raw_out = self._rectifier.draw_overlay_on_raw(raw_out, result or DetectionResult([], [], []))

        # Status badge (homography)
        badge = ("LIVE" if self._rectifier.homography_is_fresh
                 else ("CACHE" if self._rectifier.homography_is_cached
                       else "NO H"))
        badge_color = ((0, 200, 80) if badge == "LIVE"
                       else ((0, 160, 255) if badge == "CACHE" else (0, 0, 200)))
        cv2.putText(raw_out, badge, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, badge_color, 2, cv2.LINE_AA)

        # 6. Rectify
        rect_out = None
        if self._rectifier.has_homography:
            rect_out = self._rectifier.rectify(raw_out)
            if rect_out is not None:
                if cfg.get("show_grid", True):
                    rect_out = self._rectifier.draw_grid(rect_out)
                rect_out = self._draw_objects_on_rect(rect_out)

        # 7. Encode JPEG
        quality   = [cv2.IMWRITE_JPEG_QUALITY, cfg.get("jpeg_quality", 70)]
        _, raw_j  = cv2.imencode(".jpg", raw_out,  quality)
        raw_bytes = bytes(raw_j)

        rect_bytes = None
        if rect_out is not None:
            _, rect_j  = cv2.imencode(".jpg", rect_out, quality)
            rect_bytes = bytes(rect_j)

        # 8. Processing pipeline (on raw undistorted frame, before ArUco overlay)
        proc_bytes = self._apply_processing(frame, cfg)

        # 9. Commit to shared buffer
        with self._lock:
            self._latest_raw   = raw_bytes
            self._latest_rect  = rect_bytes
            self._latest_proc  = proc_bytes
            self._detections   = new_detections
            info = self._source.info if self._source else None
            if info:
                self._frame_info["frame_idx"]   = self._source.current_frame_idx
                self._frame_info["frame_count"] = info.frame_count
                self._frame_info["fps"]         = info.fps
                self._frame_info["is_live"]     = info.is_live

    def _update_object_color_if_unknown(self, aruco_id: int, color: str):
        with self._lock:
            for obj in self._objects:
                if obj.get("aruco_id") == aruco_id and obj.get("color", "unknown") == "unknown":
                    obj["color"] = color
                    break

    def _draw_objects_on_rect(self, frame):
        """Draw all registered objects on the rectified (top-down) view."""
        with self._lock:
            objects = list(self._objects)

        for obj in objects:
            pos = obj.get("current_pos") or obj.get("initial_pos")
            if not pos:
                continue
            x_mm = pos.get("x", 0)
            y_mm = pos.get("y", 0)
            px, py = self._rectifier.table_mm_to_rectified(x_mm, y_mm)

            dot_color = _DRAW_COLORS.get(obj.get("color", "unknown"), _DRAW_COLORS["unknown"])
            on_table  = obj.get("on_table", True)

            if on_table:
                cv2.circle(frame, (px, py), 9, dot_color, -1)
                cv2.circle(frame, (px, py), 9, (255, 255, 255), 1)
            else:
                # Ghost ring for objects removed from table
                cv2.circle(frame, (px, py), 9, dot_color, 1)

            label = str(obj.get("name", obj.get("id", "?")))[:10]
            cv2.putText(frame, label, (px + 11, py + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, (255, 255, 255), 1, cv2.LINE_AA)

        return frame
