"""
embed_cam.py — ESP32-CAM embedded vision: fetch a single JPEG from the
robot-mounted camera, run ArUco detection, return the lateral offset
needed to align the gripper on the 4 stock objects.

Each stock object carries a 4x4_50 ArUco tag at its centre:
    id = 36  → blue object
    id = 47  → yellow object
The strategy expects 4 tags spread over EXPECTED_SPREAD_MM (=150 mm).

Self-calibrating: when all 4 tags are detected the leftmost↔rightmost
pixel spread gives us a px→mm scale on the fly. With <4 tags we fall
back to the most recent scale (or a default calibrated value) and
flag the result as partial.

Wire format expected by the firmware (see jetson_bridge.cpp):
    embed_detect_reply(n=N,offset=Δmm,bias=-1|0|+1,valid=0|1)
  n        : tags detected (0..4)
  offset   : lateral offset of detection centroid in mm (image-frame,
             +X = right side of frame). Strategy converts to robot frame.
  bias     : direction hint when n < EXPECTED_COUNT:
                -1  detections clustered right → move LEFT to find more
                +1  detections clustered left  → move RIGHT to find more
                 0  no info / centred
  valid    : 1 if scale calibrated AND n >= 1.

The UI Detection sub-tab listens to:
    vision_feed{feed_id='detect_preview', kind='frame', jpeg=...}
    vision_feed{feed_id='detect_results', kind='json', data=[...]}
"""
from __future__ import annotations

import io
import time
import base64
import threading
import urllib.request
from typing import Optional

try:
    import cv2
    import numpy as np
    _CV2_OK = True
    _HAS_NEW_ARUCO_API = hasattr(cv2.aruco, 'ArucoDetector')
except Exception as e:                                     # pragma: no cover
    _CV2_OK = False
    _CV2_ERR = str(e)
    _HAS_NEW_ARUCO_API = False


# ── Configuration (mutable at runtime via set_config) ─────────────────
# Tag id convention (stock objects, 4x4_50 dictionary):
#   id = 36 → BLUE object
#   id = 47 → YELLOW object
TAG_BLUE   = 36
TAG_YELLOW = 47
_STOCK_IDS = {TAG_BLUE, TAG_YELLOW}

_cfg: dict = {
    # ESPCAM HTTP endpoint that returns a single JPEG when fetched.
    # One-picture-at-a-time is the only mode we support — the firmware
    # is single-client and any persistent connection blocks the camera.
    'url':                'http://192.168.0.102/capture',
    # Liveness prober. Runs in a background thread and opens a single
    # TCP connection to the camera host:port — no HTTP request sent,
    # the socket is closed immediately on accept. Costs the ESP one
    # accept()+close() pair; cannot fire the camera. The prober skips
    # entirely when a /capture has happened recently (see
    # `probe_skip_if_capture_within_s`) so during active use we add
    # zero ESP traffic.
    'probe_enabled':                True,
    'probe_interval_s':             15.0,
    'probe_timeout_s':              1.0,
    'probe_skip_if_capture_within_s': 30.0,
    'fetch_timeout_s':    5.0,
    # ArUco detection — DICT_4X4_50, IDs 36 (blue) / 47 (yellow). The
    # `team` knob lets the operator restrict detection to one colour
    # in the Detection sub-tab: 'auto' = both, 'blue' = id 36 only,
    # 'yellow' = id 47 only. Strategy queries leave this on 'auto'.
    'aruco_dict':         '4x4_50',
    'refine':             'subpix',      # 'none' | 'subpix' | 'contour'
    'team':               'auto',        # 'auto' | 'blue' | 'yellow'
    # Geometry priors used to project pixel offset to mm.
    'expected_count':     4,
    'expected_spread_mm': 150.0,         # mm between leftmost & rightmost
    # Fallback px→mm scale used when n < 2 (so spread is unknown).
    # Calibrate empirically once; the auto-scale takes over the moment
    # we see all 4 tags.
    'fallback_scale_mm_per_px': 0.5,
    # When n < expected_count, classify the centroid offset against
    # this fraction of the image width to decide left/right bias.
    'bias_dead_zone_frac': 0.10,
    # Single-tag case: assume the visible tag is the outermost of the
    # row. Distance (in mm) from that outermost tag CENTRE to the row
    # middle. Operator-specified: 30 (module width) + 15 (half module)
    # = 45 mm. Adjust if the physical stock geometry changes.
    'single_tag_offset_mm': 45.0,
}

# Most recent successful (n>=2) auto-calibrated scale. Used as the
# default for partial detections so we still report a meaningful
# offset even when we can't self-calibrate.
_last_scale_mm_per_px: Optional[float] = _cfg['fallback_scale_mm_per_px']

_lock = threading.Lock()


def get_config() -> dict:
    return dict(_cfg)


def set_config(patch: dict) -> dict:
    """Merge runtime overrides (Detection-tab inputs, runtime config)."""
    if not isinstance(patch, dict):
        return get_config()
    for k, v in patch.items():
        if k in _cfg and v is not None:
            try:
                _cfg[k] = type(_cfg[k])(v) if not isinstance(_cfg[k], str) else str(v)
            except (TypeError, ValueError):
                _cfg[k] = v
    return get_config()


# ── Liveness prober ───────────────────────────────────────────────────
# Detects "ESP online/offline" without ever touching the camera. Opens
# a TCP socket to the ESP host:port, closes it immediately — no HTTP
# request is sent, no handler is invoked on the ESP, the camera sensor
# is never triggered. Cost to the ESP: one accept()+close() pair.
#
# Smart polling: a successful /capture is itself proof the ESP is alive,
# so the probe loop skips entirely when fetch_frame_timed has succeeded
# within `probe_skip_if_capture_within_s`. During an active match
# (continuous embed_detect calls) this means zero extra ESP traffic;
# during idle, one TCP-connect every `probe_interval_s` (default 15 s).

import socket as _socket
from urllib.parse import urlparse as _urlparse


class EspProber:
    _HISTORY_MAX = 12

    def __init__(self):
        self._thread: 'Optional[threading.Thread]' = None
        self._running = False
        self._lock = threading.Lock()
        self._last: dict = {
            't_ms':        0,
            'ok':          False,
            'duration_ms': -1,
            'error':       '',
            'host':        '',
            'port':        0,
            'kind':        '',
        }
        self._history: 'list[dict]' = []
        self._last_capture_t = 0.0    # monotonic seconds
        self._last_capture_ok = False

    # ── Lifecycle ─────────────────────────────────────────────────────
    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop,
                                        name='embedcam-probe',
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        self._thread = None

    # ── Consumer API ──────────────────────────────────────────────────
    def status(self) -> dict:
        with self._lock:
            now = time.monotonic()
            cap_age_s = (now - self._last_capture_t) if self._last_capture_t else -1.0
            return {
                'enabled':            bool(self._running),
                'last':               dict(self._last),
                'history':            list(self._history),
                'last_capture_age_s': round(cap_age_s, 2) if cap_age_s >= 0 else -1.0,
                'last_capture_ok':    bool(self._last_capture_ok),
                'online':             self._derive_online_locked(now),
            }

    def probe_now(self) -> dict:
        """Synchronous one-shot probe — used by the panel's manual button."""
        return self._do_probe_and_record('probe-manual')

    def record_capture(self, ok: bool) -> None:
        """Called by fetch_frame_timed on every /capture attempt. A
        recent successful capture is a stronger health signal than the
        TCP probe — _derive_online prefers it."""
        with self._lock:
            self._last_capture_t = time.monotonic()
            self._last_capture_ok = bool(ok)

    # ── Internal ──────────────────────────────────────────────────────
    def _derive_online_locked(self, now: float) -> bool:
        skip_s = float(_cfg.get('probe_skip_if_capture_within_s', 30.0))
        capture_recent = (self._last_capture_t > 0
                          and (now - self._last_capture_t) <= skip_s)
        if capture_recent:
            return self._last_capture_ok
        return bool(self._last.get('ok', False))

    def _loop(self) -> None:
        while self._running:
            try:
                self._maybe_probe()
            except Exception:
                pass
            interval = float(_cfg.get('probe_interval_s', 15.0))
            slept = 0.0
            while self._running and slept < interval:
                time.sleep(0.25)
                slept += 0.25

    def _maybe_probe(self) -> None:
        """Skip the TCP probe if a /capture succeeded recently."""
        skip_s = float(_cfg.get('probe_skip_if_capture_within_s', 30.0))
        with self._lock:
            cap_age = (time.monotonic() - self._last_capture_t
                       if self._last_capture_t else 1e9)
            cap_ok = self._last_capture_ok
        if cap_age <= skip_s and cap_ok:
            return
        self._do_probe_and_record('probe')

    def _do_probe_and_record(self, kind: str) -> dict:
        host, port = self._resolve_host_port()
        if not host:
            result = {
                't_ms':        int(time.time() * 1000),
                'ok':          False,
                'duration_ms': -1,
                'error':       'no host configured',
                'host':        '', 'port': 0,
                'kind':        kind,
            }
        else:
            result = self._raw_probe(host, port)
            result['kind'] = kind
        with self._lock:
            self._last = result
            self._history.append(result)
            if len(self._history) > self._HISTORY_MAX:
                del self._history[:-self._HISTORY_MAX]
        return result

    @staticmethod
    def _resolve_host_port() -> 'tuple[str, int]':
        cap = str(_cfg.get('url') or '').strip()
        if not cap:
            return '', 0
        try:
            p = _urlparse(cap)
            host = p.hostname or ''
            port = p.port if p.port else (443 if p.scheme == 'https' else 80)
            return host, port
        except Exception:
            return '', 0

    @staticmethod
    def _raw_probe(host: str, port: int) -> dict:
        """TCP connect + immediate close. No HTTP request sent."""
        timeout_s = float(_cfg.get('probe_timeout_s', 1.0))
        info = {
            't_ms':        int(time.time() * 1000),
            'ok':          False,
            'duration_ms': 0,
            'error':       '',
            'host':        host, 'port': port,
        }
        t0 = time.monotonic()
        s = None
        try:
            s = _socket.create_connection((host, port), timeout=timeout_s)
            info['ok'] = True
            info['duration_ms'] = int((time.monotonic() - t0) * 1000)
        except _socket.timeout:
            info['duration_ms'] = int((time.monotonic() - t0) * 1000)
            info['error'] = 'timeout'
        except OSError as e:
            info['duration_ms'] = int((time.monotonic() - t0) * 1000)
            info['error'] = f'{type(e).__name__}: {e}'
        except Exception as e:
            info['duration_ms'] = int((time.monotonic() - t0) * 1000)
            info['error'] = f'{type(e).__name__}: {e}'
        finally:
            if s is not None:
                try: s.close()
                except Exception: pass
        return info


_prober = EspProber()


def start_prober() -> None:
    if _cfg.get('probe_enabled', True):
        _prober.start()


def stop_prober() -> None:
    _prober.stop()


def prober_status() -> dict:
    return _prober.status()


def probe_now() -> dict:
    return _prober.probe_now()


# ── Image fetch ───────────────────────────────────────────────────────

def _fetch_jpeg(url: str, timeout_s: float) -> Optional[bytes]:
    """GET `url`, return the response body. Raw HTTP/1.0 over a raw
    socket — same architecture as the prober. Avoids urllib's HTTP/1.1
    keep-alive negotiation + per-recv timeout quirks that were tripping
    on the ESP's 1-2 s capture latency."""
    try:
        p = _urlparse(url)
    except Exception:
        return None
    host = p.hostname or ''
    if not host:
        return None
    port = p.port if p.port else (443 if p.scheme == 'https' else 80)
    path = p.path or '/'
    if p.query:
        path += '?' + p.query
    req = (f"GET {path} HTTP/1.0\r\n"
           f"Host: {host}\r\n"
           f"User-Agent: holOS\r\n"
           f"Accept: image/jpeg, */*\r\n"
           f"Connection: close\r\n\r\n").encode()
    s = None
    try:
        s = _socket.create_connection((host, port), timeout=timeout_s)
        s.settimeout(timeout_s)
        s.setsockopt(_socket.IPPROTO_TCP, _socket.TCP_NODELAY, 1)
        s.sendall(req)
        # Read until peer closes (HTTP/1.0 + Connection: close).
        buf = bytearray()
        while True:
            chunk = s.recv(16384)
            if not chunk:
                break
            buf.extend(chunk)
        # Split headers / body at the blank line.
        sep = buf.find(b'\r\n\r\n')
        if sep < 0:
            return None
        return bytes(buf[sep + 4:])
    except Exception:
        return None
    finally:
        if s is not None:
            try: s.close()
            except Exception: pass


def fetch_frame() -> Optional['np.ndarray']:
    """Backwards-compat: returns just the decoded BGR frame, no timings."""
    raw, frame, _ = fetch_frame_timed()
    return frame


def fetch_frame_timed() -> 'tuple[Optional[bytes], Optional[np.ndarray], dict]':
    """Fetch one frame from the ESP32-CAM via /capture. Returns
    (raw_jpeg, bgr, timings). One picture at a time — no streaming, no
    MJPEG slicing, no caching. The outcome is stamped into the prober
    so a successful capture short-circuits the next periodic probe."""
    timings = {'fetch_ms': 0, 'decode_ms': 0, 'source_url': '', 'source_kind': ''}
    if not _CV2_OK:
        _prober.record_capture(False)
        return None, None, timings
    url     = str(_cfg.get('url', 'http://192.168.1.81/capture'))
    timeout = float(_cfg.get('fetch_timeout_s', 1.5))
    t0 = time.monotonic()
    blob = _fetch_jpeg(url, timeout)
    timings['fetch_ms']    = int((time.monotonic() - t0) * 1000)
    timings['source_url']  = url
    timings['source_kind'] = 'capture'
    if blob and blob[:2] == b'\xff\xd8':
        t1 = time.monotonic()
        arr = np.frombuffer(blob, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        timings['decode_ms'] = int((time.monotonic() - t1) * 1000)
        if frame is not None:
            _prober.record_capture(True)
            return bytes(blob), frame, timings
    _prober.record_capture(False)
    return None, None, timings


# ── ArUco detection ────────────────────────────────────────────────────

_ARUCO_DICTS: dict = {}
if _CV2_OK:
    _ARUCO_DICTS = {
        '4x4_50':    cv2.aruco.DICT_4X4_50,
        '4x4_100':   cv2.aruco.DICT_4X4_100,
        '4x4_250':   cv2.aruco.DICT_4X4_250,
        '5x5_50':    cv2.aruco.DICT_5X5_50,
        '6x6_50':    cv2.aruco.DICT_6X6_50,
    }

# Cached detector: rebuilt only when the dict/refine knobs change.
_det_cache: dict = {'key': None, 'detector': None, 'dict': None, 'params': None}


def _build_detector(dict_name: str, refine: str):
    """Return (detector, dict, params) for OpenCV's ArUco API. Handles
    both the legacy (≤4.6) and the modern (4.7+) call shapes."""
    cv_dict_id = _ARUCO_DICTS.get(dict_name, cv2.aruco.DICT_4X4_50)
    cv_dict = cv2.aruco.getPredefinedDictionary(cv_dict_id)
    if _HAS_NEW_ARUCO_API:
        params = cv2.aruco.DetectorParameters()
    else:
        params = cv2.aruco.DetectorParameters_create()
    # Corner refinement — same enum values across versions.
    refine_map = {
        'none':    cv2.aruco.CORNER_REFINE_NONE,
        'subpix':  cv2.aruco.CORNER_REFINE_SUBPIX,
        'contour': cv2.aruco.CORNER_REFINE_CONTOUR,
    }
    params.cornerRefinementMethod = refine_map.get(
        refine, cv2.aruco.CORNER_REFINE_SUBPIX)
    if _HAS_NEW_ARUCO_API:
        det = cv2.aruco.ArucoDetector(cv_dict, params)
    else:
        det = None
    return det, cv_dict, params


def _get_detector():
    key = (str(_cfg.get('aruco_dict', '4x4_50')),
           str(_cfg.get('refine', 'subpix')))
    if _det_cache['key'] != key:
        det, d, p = _build_detector(*key)
        _det_cache['key']      = key
        _det_cache['detector'] = det
        _det_cache['dict']     = d
        _det_cache['params']   = p
    return _det_cache


def _team_filter(override: 'Optional[str]' = None) -> set:
    """Return the set of tag IDs we accept this run, based on cfg.team
    or an explicit override ('blue' / 'yellow' / 'auto' / None)."""
    team = str(override if override is not None else _cfg.get('team', 'auto')).lower()
    if team == 'blue':   return {TAG_BLUE}
    if team == 'yellow': return {TAG_YELLOW}
    return set(_STOCK_IDS)


def _detect_tags(frame: 'np.ndarray',
                 team_override: 'Optional[str]' = None) -> list[dict]:
    """Run ArUco detection, filter to stock-object IDs (36 blue / 47
    yellow), return a list of {tag_id, team, cx, cy, corners} sorted
    by cx. `team_override` ('blue'/'yellow'/'auto') wins over the
    cfg-level default — the firmware passes its own team here so the
    opposite-color tag IDs never enter the result."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cache = _get_detector()
    if _HAS_NEW_ARUCO_API:
        corners, ids, _ = cache['detector'].detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, cache['dict'], parameters=cache['params'])
    if ids is None or len(ids) == 0:
        return []
    accepted = _team_filter(team_override)
    out: list[dict] = []
    for c, tag_id in zip(corners, ids.flatten().tolist()):
        if int(tag_id) not in accepted:
            continue
        pts = c.reshape(4, 2)
        cx = float(pts[:, 0].mean())
        cy = float(pts[:, 1].mean())
        team = 'blue' if int(tag_id) == TAG_BLUE else 'yellow'
        out.append({
            'tag_id':  int(tag_id),
            'team':    team,
            'cx':      cx,
            'cy':      cy,
            'corners': pts.tolist(),
        })
    out.sort(key=lambda d: d['cx'])
    # If the camera somehow picks up more than `expected_count` tags
    # (stray ArUco in the background), keep the cluster closest to
    # the image centre — the stock row is centred under the gripper
    # by definition.
    cap = int(_cfg.get('expected_count', 4))
    if cap > 0 and len(out) > cap:
        h, w = frame.shape[:2]
        cx_img = w / 2.0
        out = sorted(out, key=lambda d: abs(d['cx'] - cx_img))[:cap]
        out.sort(key=lambda d: d['cx'])
    return out


def _annotate(frame: 'np.ndarray', tags: list[dict],
              spread_px: float, scale_mm_per_px: float,
              offset_mm: float, dominant_team: str) -> 'np.ndarray':
    """Overlay ArUco corners + ID + summary line on a copy of `frame`."""
    img = frame.copy()
    h, w = img.shape[:2]
    cx_img = w / 2.0
    # Vertical centre line for the operator to eyeball alignment.
    cv2.line(img, (int(cx_img), 0), (int(cx_img), h), (60, 60, 60), 1)
    for i, t in enumerate(tags):
        # Team-coloured box (blue for id 36, yellow for id 47).
        col = (255, 180, 60) if t['team'] == 'blue' else (40, 220, 255)
        pts = np.array(t['corners'], dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(img, [pts], True, col, 2)
        p = (int(t['cx']), int(t['cy']))
        cv2.circle(img, p, 4, col, -1)
        cv2.putText(img, f"#{i} id={t['tag_id']}",
                    (p[0] + 8, p[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
    if tags:
        cx_tags = sum(t['cx'] for t in tags) / len(tags)
        cv2.line(img, (int(cx_tags), 0), (int(cx_tags), h),
                 (40, 120, 255), 2)
        cv2.putText(img,
                    f"n={len(tags)} team={dominant_team} "
                    f"offset={offset_mm:+.0f}mm "
                    f"scale={scale_mm_per_px:.3f}mm/px "
                    f"spread_px={spread_px:.0f}",
                    (8, h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                    cv2.LINE_AA)
    else:
        cv2.putText(img, "no ArUco tags",
                    (8, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 255), 1, cv2.LINE_AA)
    return img


def analyze_frame(frame: 'np.ndarray',
                  team_override: 'Optional[str]' = None) -> dict:
    """Run ArUco detection on `frame` and compute the offset/scale
    geometry. Returns a dict with every field the firmware + UI need.
    `team_override` lets a caller (the firmware) restrict the accepted
    tag IDs without touching the persistent cfg."""
    global _last_scale_mm_per_px
    tags = _detect_tags(frame, team_override=team_override)
    h, w = frame.shape[:2]
    cx_img = w / 2.0
    expected = int(_cfg.get('expected_count', 4))
    spread_mm = float(_cfg.get('expected_spread_mm', 150.0))
    n = len(tags)
    # Dominant team: majority of detected tag IDs.  Mixed rows
    # (shouldn't happen on a sane stock, but useful diagnostic) fall
    # back to 'mixed'.
    if n == 0:
        dominant_team = 'unknown'
    else:
        n_blue   = sum(1 for t in tags if t['team'] == 'blue')
        n_yellow = n - n_blue
        if n_blue == n:   dominant_team = 'blue'
        elif n_yellow == n: dominant_team = 'yellow'
        else:               dominant_team = 'mixed'
    # ── Auto-scale ────────────────────────────────────────────────
    # With all 4 tags we trust the EXPECTED_SPREAD_MM prior 100% and
    # update the rolling scale. With fewer detections we hold the
    # most recent good scale.
    spread_px = 0.0
    scale = _last_scale_mm_per_px or float(_cfg.get('fallback_scale_mm_per_px', 0.5))
    if n >= 2:
        spread_px = tags[-1]['cx'] - tags[0]['cx']
        if n >= expected and spread_px > 1.0:
            scale = spread_mm / spread_px
            with _lock:
                _last_scale_mm_per_px = scale
    # ── Row-middle estimation (partial-detection aware) ───────────
    # 4 stocks in a row; the true row centre is the midpoint of the
    # inner two tags (T2 and T3). With <4 visible, we pick the
    # interpretation where the resulting row-middle estimate is
    # closest to an image edge — that's the side where the missing
    # tag sits off-screen.
    #
    #   n=4 → midpoint of t[1] and t[2] (inner two).
    #   n=3 → choose midpoint(t[0],t[1]) (LEFT-of-cluster) or
    #         midpoint(t[1],t[2]) (RIGHT-of-cluster); the one with the
    #         smaller distance to its nearest image edge wins.
    #   n=2 → measure pixel gap between the two visible tags, then
    #         project one half-gap OUTSIDE the cluster on the side
    #         closest to the image edge (assumes the visible pair are
    #         T1+T2 or T3+T4, not the middle T2+T3).
    #   n=1 → assume the tag is the outermost; row centre sits
    #         `single_tag_offset_mm` toward image centre.
    #   n=0 → no info; report cx_img (offset 0). Strategy treats this
    #         as "no stock" and skips the grab.
    #
    # The bias is rewritten to mean "side where the missing tag is"
    # (-1 = LEFT, +1 = RIGHT) so the strategy can step the robot in
    # that direction to bring missing tags back into frame.
    single_off_mm = float(_cfg.get('single_tag_offset_mm', 45.0))
    px_per_mm = (1.0 / scale) if scale and scale > 0 else 0.0
    row_center_px = cx_img
    bias = 0
    if n == 1:
        if tags[0]['cx'] >= cx_img:
            row_center_px = tags[0]['cx'] - single_off_mm * px_per_mm
            bias = -1
        else:
            row_center_px = tags[0]['cx'] + single_off_mm * px_per_mm
            bias = +1
    elif n == 2:
        gap_px = tags[1]['cx'] - tags[0]['cx']
        opt_left  = tags[0]['cx'] - gap_px / 2.0   # visible = (T3,T4)
        opt_right = tags[1]['cx'] + gap_px / 2.0   # visible = (T1,T2)
        dist_left  = min(opt_left,  w - opt_left)
        dist_right = min(opt_right, w - opt_right)
        if dist_left <= dist_right:
            row_center_px = opt_left
            bias = -1
        else:
            row_center_px = opt_right
            bias = +1
    elif n == 3:
        opt_left  = (tags[0]['cx'] + tags[1]['cx']) / 2.0  # visible = (T2,T3,T4)
        opt_right = (tags[1]['cx'] + tags[2]['cx']) / 2.0  # visible = (T1,T2,T3)
        dist_left  = min(opt_left,  w - opt_left)
        dist_right = min(opt_right, w - opt_right)
        if dist_left <= dist_right:
            row_center_px = opt_left
            bias = -1
        else:
            row_center_px = opt_right
            bias = +1
    elif n >= 4:
        # All visible — true row middle is the midpoint of the inner two.
        row_center_px = (tags[1]['cx'] + tags[2]['cx']) / 2.0
        bias = 0
    # n == 0 → leave row_center_px=cx_img, bias=0 (offset_mm = 0).
    offset_mm = (row_center_px - cx_img) * scale
    valid = (n >= 1) and (scale is not None) and (scale > 0)
    preview = _annotate(frame, tags, spread_px, scale, offset_mm, dominant_team)
    return {
        'n':           n,
        'expected':    expected,
        'team':        dominant_team,
        'offset_mm':   offset_mm,
        'scale_mm_per_px': scale,
        'spread_px':   spread_px,
        'bias':        bias,
        'valid':       bool(valid),
        'tags':        tags,
        'tag_ids':     [t['tag_id'] for t in tags],
        'image_w':     w,
        'image_h':     h,
        'preview':     preview,        # BGR ndarray (not JSON-serialisable)
    }


def detect_once(team_override: 'Optional[str]' = None) -> dict:
    """Fetch + analyse + return result dict.  On failure, the dict
    still has all keys but `valid=False`, `n=0` and `preview=None`.
    `team_override` ('blue' | 'yellow' | 'auto') trumps cfg.team.
    Timings are split into fetch_ms / decode_ms / analyze_ms so the
    Detection tab can show where the round-trip went."""
    if not _CV2_OK:
        return _empty('opencv-unavailable')
    raw_jpeg, frame, t = fetch_frame_timed()
    if frame is None:
        r = _empty('fetch-failed')
        r['fetch_ms']   = t['fetch_ms']
        r['decode_ms']  = t['decode_ms']
        r['source_url']  = t['source_url']
        r['source_kind'] = t['source_kind']
        r['raw_jpeg']   = raw_jpeg
        return r
    t0 = time.monotonic()
    result = analyze_frame(frame, team_override=team_override)
    result['analyze_ms'] = int((time.monotonic() - t0) * 1000)
    result['fetch_ms']   = t['fetch_ms']
    result['decode_ms']  = t['decode_ms']
    result['source_url']  = t['source_url']
    result['source_kind'] = t['source_kind']
    result['raw_jpeg']   = raw_jpeg
    return result


def _empty(reason: str) -> dict:
    return {
        'n':           0,
        'expected':    int(_cfg.get('expected_count', 4)),
        'team':        'unknown',
        'offset_mm':   0.0,
        'scale_mm_per_px': _last_scale_mm_per_px or 0.0,
        'spread_px':   0.0,
        'bias':        0,
        'valid':       False,
        'tags':        [],
        'tag_ids':     [],
        'image_w':     0,
        'image_h':     0,
        'preview':     None,
        'raw_jpeg':    None,
        'fetch_ms':    0,
        'decode_ms':   0,
        'analyze_ms':  0,
        'source_url':  '',
        'source_kind': '',
        'error':       reason,
    }


# ── Serialisation helpers ─────────────────────────────────────────────

def preview_to_jpeg(result: dict, jpeg_quality: int = 75) -> Optional[bytes]:
    """Encode the BGR preview ndarray to JPEG bytes for the UI. Returns
    None if there's no preview (camera offline, etc.)."""
    img = result.get('preview')
    if img is None or not _CV2_OK:
        return None
    ok, buf = cv2.imencode('.jpg', img,
                           [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)])
    if not ok:
        return None
    return bytes(buf)


def result_to_json(result: dict) -> dict:
    """Strip the (non-JSON) preview ndarray + raw JPEG bytes so the
    dict can ride a SocketIO frame as JSON."""
    out = {k: v for k, v in result.items() if k not in ('preview', 'raw_jpeg')}
    return out


def result_to_checks(result: dict) -> list[dict]:
    """Render the result as a list of {name, status, value} rows for
    the Detection sub-tab's `detect_results` feed. Mirrors the format
    used by the existing pipeline detection nodes."""
    n        = int(result.get('n', 0))
    expected = int(result.get('expected', 4))
    full     = (n >= expected)
    valid    = bool(result.get('valid', False))
    bias     = int(result.get('bias', 0))
    bias_str = {-1: '← LEFT', 0: 'centred', +1: 'RIGHT →'}.get(bias, '?')
    team     = result.get('team', 'unknown')
    tag_ids  = result.get('tag_ids', [])
    rows = [
        {'name': 'tags found', 'status':
            'pass' if full else ('fail' if n == 0 else 'unknown'),
         'value': f'{n} / {expected}'},
        {'name': 'tag IDs',
         'status': 'pass' if n > 0 else 'unknown',
         'value': ', '.join(str(i) for i in tag_ids) or '—'},
        {'name': 'team',
         'status': 'pass' if team in ('blue', 'yellow') else
                   ('fail' if team == 'mixed' else 'unknown'),
         'value': str(team)},
        {'name': 'lateral offset',
         'status': 'pass' if valid else 'unknown',
         'value': f"{result.get('offset_mm', 0.0):+.1f} mm"},
        {'name': 'scale',
         'status': 'pass' if (result.get('scale_mm_per_px') or 0) > 0 else 'unknown',
         'value': f"{result.get('scale_mm_per_px', 0.0):.3f} mm/px"},
        {'name': 'spread (px)',
         'status': 'pass' if result.get('spread_px', 0) > 0 else 'unknown',
         'value': f"{result.get('spread_px', 0.0):.0f}"},
        {'name': 'bias hint',
         'status': 'pass' if (full or bias == 0) else 'unknown',
         'value': bias_str},
    ]
    # Timing breakdown — primary diagnostic when fetches stretch into
    # multiple rounds. Pass if everything happened under ~500 ms, fail
    # if fetch itself blew past 1 s (the firmware typically gives up
    # well before that anyway).
    fetch_ms   = int(result.get('fetch_ms', 0))
    decode_ms  = int(result.get('decode_ms', 0))
    analyze_ms = int(result.get('analyze_ms', 0))
    total_ms   = fetch_ms + decode_ms + analyze_ms
    rows.extend([
        {'name': 'fetch',
         'status': 'fail' if fetch_ms >= 1000 else ('unknown' if fetch_ms >= 500 else 'pass'),
         'value': f'{fetch_ms} ms'},
        {'name': 'decode',
         'status': 'pass' if decode_ms < 100 else 'unknown',
         'value': f'{decode_ms} ms'},
        {'name': 'analyze',
         'status': 'pass' if analyze_ms < 100 else 'unknown',
         'value': f'{analyze_ms} ms'},
        {'name': 'total',
         'status': 'pass' if total_ms < 500 else ('fail' if total_ms >= 1500 else 'unknown'),
         'value': f'{total_ms} ms'},
    ])
    src_kind = result.get('source_kind')
    if src_kind:
        rows.append({'name': 'source', 'status': 'pass',
                     'value': str(src_kind)})
    if result.get('error'):
        rows.insert(0, {'name': 'error', 'status': 'fail',
                        'value': str(result['error'])})
    return rows
