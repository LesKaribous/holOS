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

To run a video file as a virtual camera, launch the runner alongside
holOS — the pipelines below default the source.video URL to the runner's
MJPEG endpoint:

    cd software
    vision.bat path\to\clip.mp4
    # then start holOS in another terminal — its localization pipeline
    # will read frames from http://127.0.0.1:5174/stream.mjpg
"""

from __future__ import annotations

import services.vision_pipelines.nodes  # noqa: F401  — registers all node kinds
from services.vision_pipelines.pipeline import Pipeline
from services.vision_pipelines.nodes.base import NODE_KINDS


# ═══════════════════════════════════════════════════════════════════════════
# 1. WORLD COORDINATE FRAME
# ═══════════════════════════════════════════════════════════════════════════
# Where is (0, 0) on the BEV image? Mapping origin → axis directions:
#     'top_left'      → X+ right, Y+ down  (TwinVision native)
#     'top_right'     → X+ left,  Y+ down
#     'bottom_left'   → X+ right, Y+ up
#     'bottom_right'  → X+ left,  Y+ up    (this year's match convention)
WORLD_ORIGIN_CORNER = 'bottom_right'
WORLD_TABLE_W_MM    = 3000.0
WORLD_TABLE_H_MM    = 2000.0
WORLD_FLIP_THETA    = True


# ═══════════════════════════════════════════════════════════════════════════
# 2. CORNER ANCHORS (TwinVision native frame)
# ═══════════════════════════════════════════════════════════════════════════
# Names refer to the ArUco's POSITION in the rectified BEV (used by the
# rectifier's homography). Don't rename. Set WORLD_ORIGIN_CORNER above to
# pick which corner is your world (0, 0).
ANCHORS = {
    'top_left':     {'tag_id': 23, 'x_mm':  600, 'y_mm':  600},
    'top_right':    {'tag_id': 22, 'x_mm': 2400, 'y_mm':  600},
    'bottom_right': {'tag_id': 20, 'x_mm': 2400, 'y_mm': 1400},
    'bottom_left':  {'tag_id': 21, 'x_mm':  600, 'y_mm': 1400},
}


# ═══════════════════════════════════════════════════════════════════════════
# 3. NODE TUNING
# ═══════════════════════════════════════════════════════════════════════════

# Source — by default holOS reads frames from the standalone runner's
# MJPEG stream. Replace with a USB camera index ('0') or a file path to
# go direct.
SOURCE_KIND = 'video'
SOURCE_PATH = 'http://127.0.0.1:5174/stream.mjpg'

# ArUco detection
ARUCO_DICT   = '4x4_50'
ARUCO_REFINE = 'subpix'

# Rectifier intrinsics — set to '' to disable solvePnP and fall back to
# pure 4-anchor findHomography (lens curvature stays in the warp). Default
# loads software/vision/calibrations/camera_intrinsics.json, which MUST be
# calibrated at the same resolution your camera is serving (currently
# 1920×1080 per data/vision_camera_config.json). None = use node default.
INTRINSICS_PATH = None

# Homography mode — 'auto' (default) tries 4-anchor findHomography and
# falls back to 2-anchor similarity (SIM_TAG_A/B_ID) when fewer than 4
# anchors are detected this tick. 'h4' / 'sim2' force one mode. SIM ids
# must be present in ANCHORS.
HOMOGRAPHY_MODE = 'auto'
SIM_TAG_A_ID    = 20
SIM_TAG_B_ID    = 22

# Camera position (manual override) — entered in WORLD frame.
CAMERA_X_MM = 1275.0
CAMERA_Y_MM = -200.0
CAMERA_Z_MM = 1600.0

# Parallax
ROBOT_Z_MM = 490.0

# Common
FPS_LIMIT = 1   # we don't need real-time vision — 1 Hz is plenty for
                # recalage / pose queries and saves a lot of Jetson CPU.
TEAM      = 'blue'        # 'blue' or 'yellow'  (overridden live by topbar)
TRACK_IDS = list(range(1, 11))


# ═══════════════════════════════════════════════════════════════════════════
# 4. PIPELINE BUILDERS
# ═══════════════════════════════════════════════════════════════════════════

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
    p = Pipeline(name='localization', fps_limit=FPS_LIMIT)
    src = _add_source(p)

    pre = _add(p, 'preprocess', 'pre', {'mode': 'passthrough'})
    p.connect(src, 'frame', pre, 'frame')

    aru = _add(p, 'aruco', 'aruco',
               {'dict': ARUCO_DICT, 'refine': ARUCO_REFINE,
                'draw_markers': True, 'draw_ids': True})
    p.connect(pre, 'frame', aru, 'frame')

    rec_params = {'anchors':              ANCHORS,
                  'world_origin_corner':  WORLD_ORIGIN_CORNER,
                  'world_table_w_mm':     WORLD_TABLE_W_MM,
                  'world_table_h_mm':     WORLD_TABLE_H_MM,
                  'world_flip_theta':     WORLD_FLIP_THETA,
                  'homography_mode':      HOMOGRAPHY_MODE,
                  'sim_tag_a_id':         SIM_TAG_A_ID,
                  'sim_tag_b_id':         SIM_TAG_B_ID,
                  'draw_grid':            False}
    if INTRINSICS_PATH is not None:
        rec_params['intrinsics_path'] = INTRINSICS_PATH
    rec = _add(p, 'rectify', 'rectify', rec_params)
    p.connect(pre, 'frame',     rec, 'frame')
    p.connect(aru, 'detection', rec, 'detection')

    loc_naive = _add(p, 'localization', 'loc_naive',
                     {'anchors': ANCHORS, 'cam_mode': 'auto',
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
               {'object_z_mm':         ROBOT_Z_MM,
                'use_naive_xy':        True,
                'preview_draw_camera': True,
                'preview_draw_grid':   False})
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
    o_aruco_frame = _add(p, 'output', 'out_aruco_frame',
                         {'feed_id': 'aruco', 'label': 'ArUco detection'})
    p.connect(aru, 'preview', o_aruco_frame, 'preview')

    o_naive = _add(p, 'output', 'out_naive_frame',
                   {'feed_id': 'loc_naive', 'label': 'Naive (pre-parallax)'})
    p.connect(loc_naive, 'preview', o_naive, 'preview')

    o_corr = _add(p, 'output', 'out_corrected_frame',
                  {'feed_id': 'loc_corrected', 'label': 'Parallax-corrected'})
    p.connect(grid, 'preview', o_corr, 'preview')

    return p


# ── Detection pipeline — feeds the Vision · Detection view ────────────────

def build_detection() -> Pipeline:
    """Stub for game-element detection (color checks, action verification).
    Currently just publishes the source frame as `detect_preview` and an
    empty result list. Replace with real detection logic when the rules
    are nailed down."""
    p = Pipeline(name='detection', fps_limit=FPS_LIMIT)
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
