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
    'url':                'http://192.168.1.81/capture',
    # Fallback URL: a `/` endpoint on the bare AI-Thinker firmware
    # only serves MJPEG; fetching the multipart stream and slicing
    # out the first JPEG works too. Try `url` first, fall back here
    # if the first attempt times out / returns a multipart payload.
    'mjpeg_url':          'http://192.168.1.81/',
    'fetch_timeout_s':    1.5,
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


# ── Image fetch ───────────────────────────────────────────────────────

def _fetch_jpeg(url: str, timeout_s: float) -> Optional[bytes]:
    """GET `url`, return the raw response body. Returns None on error."""
    try:
        req = urllib.request.Request(url, headers={'User-Agent': 'holOS'})
        with urllib.request.urlopen(req, timeout=timeout_s) as resp:
            return resp.read()
    except Exception:
        return None


def _slice_first_jpeg_from_mjpeg(blob: bytes) -> Optional[bytes]:
    """ESP32-CAM AI-Thinker firmware serves only MJPEG on `/`. Pull the
    first complete JPEG frame out of a multipart blob by hunting for
    the SOI (0xFFD8) and EOI (0xFFD9) markers."""
    soi = blob.find(b'\xff\xd8')
    if soi < 0:
        return None
    eoi = blob.find(b'\xff\xd9', soi + 2)
    if eoi < 0:
        return None
    return blob[soi:eoi + 2]


def fetch_frame() -> Optional['np.ndarray']:
    """Fetch a single BGR frame from the embedded camera. Returns None
    on any error (camera offline, timeout, decode failure)."""
    if not _CV2_OK:
        return None
    url       = str(_cfg.get('url', 'http://192.168.1.81/capture'))
    mjpeg_url = str(_cfg.get('mjpeg_url', 'http://192.168.1.81/'))
    timeout   = float(_cfg.get('fetch_timeout_s', 1.5))
    # 1) Try the dedicated single-shot endpoint first.
    blob = _fetch_jpeg(url, timeout)
    if blob and blob[:2] == b'\xff\xd8':
        # Standard JPEG SOI — decode straight.
        arr = np.frombuffer(blob, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is not None:
            return frame
    # 2) Fallback: pull from the MJPEG stream, slice the first frame.
    blob = _fetch_jpeg(mjpeg_url, timeout)
    if blob:
        jpeg = _slice_first_jpeg_from_mjpeg(blob)
        if jpeg:
            arr = np.frombuffer(jpeg, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is not None:
                return frame
    return None


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


def _team_filter() -> set:
    """Return the set of tag IDs we accept this run, based on cfg.team."""
    team = str(_cfg.get('team', 'auto')).lower()
    if team == 'blue':   return {TAG_BLUE}
    if team == 'yellow': return {TAG_YELLOW}
    return set(_STOCK_IDS)


def _detect_tags(frame: 'np.ndarray') -> list[dict]:
    """Run ArUco detection, filter to stock-object IDs (36 blue / 47
    yellow), return a list of {tag_id, team, cx, cy, corners} sorted
    by cx."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cache = _get_detector()
    if _HAS_NEW_ARUCO_API:
        corners, ids, _ = cache['detector'].detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, cache['dict'], parameters=cache['params'])
    if ids is None or len(ids) == 0:
        return []
    accepted = _team_filter()
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


def analyze_frame(frame: 'np.ndarray') -> dict:
    """Run ArUco detection on `frame` and compute the offset/scale
    geometry. Returns a dict with every field the firmware + UI need."""
    global _last_scale_mm_per_px
    tags = _detect_tags(frame)
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
    # ── Lateral offset ────────────────────────────────────────────
    # Centre of the detected tags minus the image centre, scaled.
    offset_mm = 0.0
    if n > 0:
        cx_tags = sum(t['cx'] for t in tags) / n
        offset_mm = (cx_tags - cx_img) * scale
    # ── Bias hint when n < expected ───────────────────────────────
    # Tell the strategy which way to nudge to bring the missing tags
    # into the frame. If the surviving detections cluster left of
    # centre, the missing ones probably sit to the right → bias=+1
    # (and vice versa). This is the OPPOSITE side of the cluster,
    # since the missing tags are on the side the camera doesn't see.
    bias = 0
    if 0 < n < expected:
        cx_tags = sum(t['cx'] for t in tags) / n
        dz = float(_cfg.get('bias_dead_zone_frac', 0.10)) * w
        if (cx_tags - cx_img) > dz:
            # Detections lean right → the missing ones are out to the LEFT.
            bias = -1
        elif (cx_tags - cx_img) < -dz:
            bias = +1
        else:
            bias = 0
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


def detect_once() -> dict:
    """Fetch + analyse + return result dict.  On failure, the dict
    still has all keys but `valid=False`, `n=0` and `preview=None`."""
    if not _CV2_OK:
        return _empty('opencv-unavailable')
    frame = fetch_frame()
    if frame is None:
        return _empty('fetch-failed')
    return analyze_frame(frame)


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
    """Strip the (non-JSON) preview ndarray so the dict can ride a
    SocketIO frame as JSON."""
    out = {k: v for k, v in result.items() if k != 'preview'}
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
    if result.get('error'):
        rows.insert(0, {'name': 'error', 'status': 'fail',
                        'value': str(result['error'])})
    return rows
