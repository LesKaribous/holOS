"""
holOS vision pipelines — defined in pure Python, no JSON.

Edit this file to change which pipelines holOS runs at startup. Each
function in `PIPELINES` returns a configured `Pipeline` instance; they're
all registered + enabled when the server boots and run concurrently in
their own worker threads.

Layout:
  1. WORLD CONFIG    — coordinate frame + table dimensions (top of file)
  2. ANCHORS         — physical mm positions of the 4 corner ArUcos
  3. NODE TUNING     — preprocess / aruco / parallax / output knobs
  4. build_xxx       — one builder per pipeline
  5. PIPELINES       — registry mapping name → builder

Source kind + path live in data/vision_config.json and are loaded by the
in-process FrameSource (software/vision_source.py). The pipeline source
nodes here are thin wrappers that snapshot FrameSource's latest-BGR
slot — no per-node cv2 capture, no buffer.
"""

from __future__ import annotations

import json
import os

import services.vision_pipelines.nodes  # noqa: F401  — registers all node kinds
from services.vision_pipelines.pipeline import Pipeline
from services.vision_pipelines.nodes.base import NODE_KINDS


# ═══════════════════════════════════════════════════════════════════════════
# 1. LOAD VISION CONFIG (single source of truth — data/vision_config.json)
# ═══════════════════════════════════════════════════════════════════════════
# Field-tunables (FPS, anchors, table size, camera position, robot Z) live
# in JSON so they can change without editing this file. Static structural
# choices (ArUco dict, homography mode, detector tag IDs) stay below as
# Python constants — they're code-shape, not configuration.
_CFG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         'data', 'vision_config.json')
_CFG_DEFAULTS = {
    'source_fps':          16,
    'pipeline_fps':        4,
    'world_origin_corner': 'bottom_right',
    'world_flip_theta':    True,
    'table_width_mm':      3000.0,
    'table_height_mm':     2000.0,
    'anchors': {
        'top_left':     {'tag_id': 23, 'x_mm': 2400, 'y_mm': 1400, 'yaw_deg': 0},
        'top_right':    {'tag_id': 22, 'x_mm':  600, 'y_mm': 1400, 'yaw_deg': 0},
        'bottom_right': {'tag_id': 20, 'x_mm':  600, 'y_mm':  600, 'yaw_deg': 0},
        'bottom_left':  {'tag_id': 21, 'x_mm': 2400, 'y_mm':  600, 'yaw_deg': 0},
    },
    'cam_yellow_xyz_mm':   [1275.0, -100.0, 1100.0],
    'cam_blue_xyz_mm':     [1725.0, -100.0, 1100.0],
    'robot_z_mm':          550.0,
    'opp_object_z_mm':     550.0,
}


def _load_cfg() -> dict:
    cfg = dict(_CFG_DEFAULTS)
    try:
        with open(_CFG_PATH) as f:
            cfg.update(json.load(f))
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f'[vision_pipelines_def] config load failed ({e}) — using defaults')
    return cfg


_CFG = _load_cfg()

# Convenience aliases — read once at module import (pipelines are built
# at server boot; live config edits need a holOS restart).
PIPELINE_FPS         = int(_CFG['pipeline_fps'])
WORLD_ORIGIN_CORNER  = str(_CFG['world_origin_corner'])
WORLD_FLIP_THETA     = bool(_CFG['world_flip_theta'])
WORLD_TABLE_W_MM     = float(_CFG['table_width_mm'])
WORLD_TABLE_H_MM     = float(_CFG['table_height_mm'])
ANCHORS              = dict(_CFG['anchors'])
ROBOT_Z_MM           = float(_CFG['robot_z_mm'])


# ═══════════════════════════════════════════════════════════════════════════
# 3. NODE TUNING
# ═══════════════════════════════════════════════════════════════════════════

# Source — pipeline source nodes are now thin wrappers around the
# in-process FrameSource singleton (software/vision_source.py). The
# actual source kind + path live in vision_config.json and the
# FrameSource owns the device/file; SOURCE_KIND here only picks which
# pipeline-node class to instantiate (their params are informational —
# the source.* nodes ignore them at runtime).
SOURCE_KIND = str(_CFG.get('source_kind', 'camera'))
SOURCE_PATH = str(_CFG.get('source_path', ''))

# ArUco detection
ARUCO_DICT   = '4x4_50'
ARUCO_REFINE = 'none'   # 'subpix' is ~2× slower on 1080p; with our tag
                         # size (100 mm) + 1080p capture, raw corner accuracy
                         # is already <1 px / ~1 mm in BEV. Bump to 'subpix'
                         # only if you see tag positions visibly jittering
                         # AND the Jetson has CPU headroom.

# Rectifier intrinsics — set to '' to disable solvePnP and fall back to
# pure 4-anchor findHomography (lens curvature stays in the warp). Default
# loads software/vision/calibrations/camera_intrinsics.json, which MUST be
# calibrated at the same resolution your camera is serving (currently
# gst_width × gst_height in data/vision_config.json). None = node default.
INTRINSICS_PATH = None

# Homography mode — 'auto' (default) tries 4-anchor findHomography and
# falls back to the corner-based fit (SIM_TAG_IDS) when fewer than 4
# anchors are detected this tick. 'h4' / 'sim2' force one mode.
# All ids in SIM_TAG_IDS must be present in ANCHORS (corners or extras).
# Use ≥ 3 non-collinear ids — 2 colinear tags give a degenerate H.
# Standard 4-anchor mode. With intrinsics loaded the rectifier runs
# update_pose() per tick (solvePnP + lens undistortion → priority-1 path
# in TableRectifier.rectify), which gives the cleanest BEV. 'auto' /
# 'sim2' fallbacks are still implemented but unused with this layout.
HOMOGRAPHY_MODE = 'h4'
SIM_TAG_IDS     = []     # unused in h4 mode; kept for back-compat
SIM_TAG_A_ID    = 20
SIM_TAG_B_ID    = 22
TAG_SIZE_MM     = 100.0  # only consulted by sim2 corner-based path

# Initial camera-position values used at pipeline build — overwritten by
# `_apply_team` in run.py at every team flip (and at boot via force=True),
# which reads `cam_<team>_xyz_mm` from vision_config.json. So these just
# need to be a valid placeholder; the runtime path is authoritative.
_CAM_BLUE          = _CFG['cam_blue_xyz_mm']
CAMERA_X_MM        = float(_CAM_BLUE[0])
CAMERA_Y_MM        = float(_CAM_BLUE[1])
CAMERA_Z_MM        = float(_CAM_BLUE[2])

TEAM      = 'blue'        # 'blue' or 'yellow'  (overridden live by topbar)
TRACK_IDS = list(range(1, 11))


# ═══════════════════════════════════════════════════════════════════════════
# 4. PIPELINE BUILDERS
# ═══════════════════════════════════════════════════════════════════════════

def _world_to_bev_anchors(anchors_world: dict, origin_corner: str,
                          table_w: float, table_h: float) -> dict:
    """Convert ANCHORS values from world frame (user-facing) to BEV-native
    frame (what TableRectifier expects). Mirrors the (flip_x, flip_y) the
    rectify node uses internally on output data, so the user only ever
    writes / reads world-frame numbers."""
    flip_x = origin_corner in ('top_right', 'bottom_right')
    flip_y = origin_corner in ('bottom_left', 'bottom_right')

    def _convert(v: dict) -> dict:
        x = (table_w - v['x_mm']) if flip_x else v['x_mm']
        y = (table_h - v['y_mm']) if flip_y else v['y_mm']
        # Reflections compose as: a flip on X mirrors yaw about Y axis
        # (yaw → 180-yaw); flip on Y mirrors about X (yaw → -yaw); both
        # together is a 180° rotation (yaw → yaw + 180).
        yaw = float(v.get('yaw_deg', 0.0))
        if flip_x and flip_y:
            yaw_bev = yaw + 180.0
        elif flip_x:
            yaw_bev = 180.0 - yaw
        elif flip_y:
            yaw_bev = -yaw
        else:
            yaw_bev = yaw
        return {'tag_id':  int(v['tag_id']),
                'x_mm':    float(x),
                'y_mm':    float(y),
                'yaw_deg': yaw_bev}

    out = {}
    for slot, v in anchors_world.items():
        if slot == 'extras':
            out['extras'] = [_convert(x) for x in (v or [])]
        else:
            out[slot] = _convert(v)
    return out


def _add(p: Pipeline, kind: str, node_id: str, params: dict | None = None):
    """Instantiate a registered node kind and attach it. Returns the id."""
    cls = NODE_KINDS.get(kind)
    if cls is None:
        raise KeyError(f'unknown node kind {kind!r} — registered: '
                       f'{sorted(NODE_KINDS)}')
    p.add_node(node_id, kind, cls(params or {}))
    return node_id


def _add_source(p: Pipeline) -> str:
    """Common source.* node setup. Source kind + path live up top so all
    pipelines share the same input."""
    if SOURCE_KIND == 'video':
        return _add(p, 'source.video', 'src',
                    {'path': SOURCE_PATH, 'loop': True,
                     'playback': 'live', 'speed': 1.0})
    if SOURCE_KIND == 'camera':
        return _add(p, 'source.camera', 'src',
                    {'device': int(SOURCE_PATH) if SOURCE_PATH.isdigit() else 0,
                     'playback': 'live'})
    if SOURCE_KIND == 'image':
        return _add(p, 'source.image', 'src',
                    {'path': SOURCE_PATH, 'playback': 'every_tick'})
    raise ValueError(f'unknown SOURCE_KIND {SOURCE_KIND!r}')


# ── Localization pipeline — feeds the Vision · Localization view ─────────

def build_localization() -> Pipeline:
    """ArUco → rectify → localization → parallax. Publishes:
        aruco          → raw frame with markers drawn
        loc_naive      → BEV with naive (pre-parallax) tag positions
        loc_corrected  → BEV with parallax-corrected positions
        poses          → team-filtered pose_list
    """
    p = Pipeline(name='localization', fps_limit=PIPELINE_FPS)
    src = _add_source(p)

    pre = _add(p, 'preprocess', 'pre', {'mode': 'passthrough'})
    p.connect(src, 'frame', pre, 'frame')

    aru = _add(p, 'aruco', 'aruco',
               {'dict': ARUCO_DICT, 'refine': ARUCO_REFINE,
                'draw_markers': True, 'draw_ids': True})
    p.connect(pre, 'frame', aru, 'frame')

    # Translate the user-facing world-frame anchors into the BEV-native
    # frame TableRectifier consumes. Done once at build time so the rest
    # of the pipeline never needs to think about which frame ANCHORS is in.
    anchors_bev = _world_to_bev_anchors(
        ANCHORS, WORLD_ORIGIN_CORNER, WORLD_TABLE_W_MM, WORLD_TABLE_H_MM)

    rec_params = {'anchors':              anchors_bev,
                  'world_origin_corner':  WORLD_ORIGIN_CORNER,
                  'world_table_w_mm':     WORLD_TABLE_W_MM,
                  'world_table_h_mm':     WORLD_TABLE_H_MM,
                  'world_flip_theta':     WORLD_FLIP_THETA,
                  'homography_mode':      HOMOGRAPHY_MODE,
                  'sim_tag_ids':          list(SIM_TAG_IDS),
                  'sim_tag_a_id':         SIM_TAG_A_ID,
                  'sim_tag_b_id':         SIM_TAG_B_ID,
                  'tag_size_mm':          TAG_SIZE_MM,
                  'draw_grid':            False}
    if INTRINSICS_PATH is not None:
        rec_params['intrinsics_path'] = INTRINSICS_PATH
    rec = _add(p, 'rectify', 'rectify', rec_params)
    p.connect(pre, 'frame',     rec, 'frame')
    p.connect(aru, 'detection', rec, 'detection')

    loc_naive = _add(p, 'localization', 'loc_naive',
                     {'anchors': anchors_bev, 'cam_mode': 'auto',
                      'team': TEAM, 'track_ids': TRACK_IDS,
                      'robot_z_mm': ROBOT_Z_MM, 'preview_use': 'naive'})
    p.connect(aru, 'detection',        loc_naive, 'detection')
    p.connect(rec, 'frame',            loc_naive, 'frame')
    p.connect(rec, 'homography_state', loc_naive, 'rectifier_state')
    p.connect(rec, 'rectifier',        loc_naive, 'rectifier')
    p.connect(rec, 'coord_state',      loc_naive, 'coord_state')

    cam = _add(p, 'camera.manual', 'cam',
               {'cam_x_mm': CAMERA_X_MM, 'cam_y_mm': CAMERA_Y_MM,
                'cam_z_mm': CAMERA_Z_MM, 'coord_frame': 'world'})

    par = _add(p, 'parallax', 'par',
               {'robot_z_mm':          ROBOT_Z_MM,
                'use_naive_xy':        True,
                'preview_draw_camera': True,
                'preview_draw_grid':   False,
                'preview_gate_feed':   'loc_corrected'})
    p.connect(loc_naive, 'pose_list', par, 'pose_list')
    p.connect(cam, 'cam_xyz',         par, 'cam_xyz')
    p.connect(rec, 'frame',           par, 'frame')
    p.connect(rec, 'coord_state',     par, 'coord_state')

    grid = _add(p, 'overlay.grid', 'grid',
                {'step_mm': 200, 'draw_labels': True,
                 'table_w_mm': int(WORLD_TABLE_W_MM),
                 'table_h_mm': int(WORLD_TABLE_H_MM)})
    p.connect(par, 'preview',     grid, 'frame')
    p.connect(rec, 'coord_state', grid, 'coord_state')

    # ── Data outputs (consumed by holOS minimalist view, text-only) ─
    # Raw aruco list (no team filter, no rectify): tag_id + pixel center.
    o_arulist = _add(p, 'output.aruco_list', 'out_arulist',
                     {'feed_id': 'aruco_list',
                      'label':   'Detected ArUco tags',
                      'fps_limit': 5})
    p.connect(aru, 'detection', o_arulist, 'detection')

    # Pose list (already team-filtered + has naive + corrected fields).
    # Naive parallax-corrected pose_list before parallax node.
    o_naive_pl = _add(p, 'output.pose_list', 'out_naive_pl',
                      {'feed_id': 'poses_naive',
                       'label':   'Poses — pre-parallax',
                       'fps_limit': 5})
    p.connect(loc_naive, 'pose_list', o_naive_pl, 'pose_list')

    # Pose list after the standalone parallax pass (extra correction).
    o_corr_pl = _add(p, 'output.pose_list', 'out_corr_pl',
                     {'feed_id': 'poses_corrected',
                      'label':   'Poses — post-parallax',
                      'fps_limit': 5})
    p.connect(par, 'pose_list', o_corr_pl, 'pose_list')

    # ── Frame outputs (consumed by the debug page only — vision.bat) ─
    # holOS itself doesn't render these; they're served via /vision_debug.
    # encode_grayscale=True drops the JPEG to single-channel just before
    # imencode — overlays drawn upstream in BGR fold into luminance, the
    # encode is ~30% cheaper and the payload ~half size. Acceptable for
    # debug feeds.
    # fps_limit matches PIPELINE_FPS — pipeline ticks N×/s, frame outputs
    # emit N×/s. Decoupling them only makes sense when you want the UI
    # slower than detection, and right now the user wants them in sync.
    _OUT_PARAMS = {'jpeg_quality': 60, 'fps_limit': PIPELINE_FPS,
                   'encode_grayscale': True}
    o_aruco_frame = _add(p, 'output', 'out_aruco_frame',
                         {'feed_id': 'aruco', 'label': 'ArUco detection',
                          **_OUT_PARAMS})
    p.connect(aru, 'preview', o_aruco_frame, 'preview')

    o_naive = _add(p, 'output', 'out_naive_frame',
                   {'feed_id': 'loc_naive', 'label': 'Naive (pre-parallax)',
                    **_OUT_PARAMS})
    p.connect(loc_naive, 'preview', o_naive, 'preview')

    o_corr = _add(p, 'output', 'out_corrected_frame',
                  {'feed_id': 'loc_corrected', 'label': 'Parallax-corrected',
                   **_OUT_PARAMS})
    p.connect(grid, 'preview', o_corr, 'preview')

    return p


# ── Detection pipeline — feeds the Vision · Detection view ────────────────

def build_detection() -> Pipeline:
    """Stub for game-element detection (color checks, action verification).
    Currently just publishes the source frame as `detect_preview` and an
    empty result list. Replace with real detection logic when the rules
    are nailed down."""
    p = Pipeline(name='detection', fps_limit=PIPELINE_FPS)
    src = _add_source(p)

    o_prev = _add(p, 'output', 'out_detpreview',
                  {'feed_id': 'detect_preview',
                   'label': 'Detection preview (placeholder)'})
    p.connect(src, 'frame', o_prev, 'frame')

    # Empty checks list — placeholder until real check nodes exist.
    # The detection view will show "no checks reported this frame" until
    # a node publishes [{name, status, value, color}, …] under feed_id
    # 'detect_results'.
    return p


# ═══════════════════════════════════════════════════════════════════════════
# 5. PIPELINE REGISTRY
# ═══════════════════════════════════════════════════════════════════════════
# Maps pipeline name → builder function. Both entries below are loaded
# at server startup and run concurrently. The Vision · Localization view
# is bound to 'localization' by default; Vision · Detection to 'detection'.

PIPELINES = {
    'localization': build_localization,
    # 'detection': build_detection,   # disabled — placeholder pipeline,
                                       # was eating CPU for nothing.
}

# Pipeline auto-enabled at startup. None = none of them.
DEFAULT_PIPELINE = 'localization'
