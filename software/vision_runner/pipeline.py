"""
Edit this file to define your vision pipeline.

The runner watches this file's mtime and rebuilds on save — no restart.
`build_pipeline(source_kind, source_path)` is the only required entry point.

Layout:
  1. WORLD CONFIG    — coordinate frame + table dimensions (top of file)
  2. ANCHORS         — physical mm positions of the 4 corner ArUcos
  3. NODE TUNING     — preprocess / aruco / parallax / output knobs
  4. build_pipeline  — wires nodes together
"""

from __future__ import annotations

import services.vision_pipelines.nodes  # noqa: F401  — registers all node kinds
from services.vision_pipelines.pipeline import Pipeline
from services.vision_pipelines.nodes.base import NODE_KINDS


# ═══════════════════════════════════════════════════════════════════════════
# 1. WORLD COORDINATE FRAME
# ═══════════════════════════════════════════════════════════════════════════
#
# The BEV image is always rectified into TwinVision's NATIVE frame:
#     origin (0, 0) at TOP-LEFT, X grows RIGHT, Y grows DOWN.
#
# Real robots usually want a different convention. Pick which BEV corner
# holds your world origin (0, 0). Everything downstream — pose_list,
# cam_xyz, grid labels, the dashboard — is mirrored to match.
#
# THE BEV IMAGE ITSELF IS NEVER ROTATED OR FLIPPED.  Only the data-side
# convention changes. The camera view of the table stays where the camera
# sees it.
#
# Mapping origin → axis directions on the BEV image:
#
#     origin            X+ direction         Y+ direction
#     ──────────────    ──────────────       ──────────────
#     'top_left'        right (BEV native)   down  (BEV native)
#     'top_right'       LEFT                 down
#     'bottom_left'     right                UP
#     'bottom_right'    LEFT                 UP                ← typical robotics
#
# 2026 match table: origin near image bottom-right, X+ toward bottom-left,
# Y+ toward top-right.  → 'bottom_right'.
WORLD_ORIGIN_CORNER = 'bottom_right'

# Physical playable area in mm.  Used to mirror coords across the midline
# when flipping.  Match the actual table; default = TwinVision native.
WORLD_TABLE_W_MM = 3000.0
WORLD_TABLE_H_MM = 2000.0

# Headings (theta_rad) get mirrored too when ON, so an arrow pointing
# "physically toward the wall" stays pointing toward that wall after the
# coord flip.  Leave on unless you have a reason.
WORLD_FLIP_THETA = True


# ═══════════════════════════════════════════════════════════════════════════
# 2. CORNER ANCHORS (TwinVision native frame — used to compute homography)
# ═══════════════════════════════════════════════════════════════════════════
#
# These are the ArUco tags glued to the table corners.  The mm coordinates
# below are in TwinVision's NATIVE frame (origin top-left of table) — they
# define WHERE on the table each tag sits, and the rectifier uses them to
# warp the camera image into a flat top-down view.
#
# The names ('top_left', 'top_right'…) refer to where the tags appear on
# the rectified BEV image, NOT to your world frame.  Don't change those
# names — they're how TwinVision identifies which tag goes where in the
# warp.  If your world has origin at image bottom-right, just set
# WORLD_ORIGIN_CORNER = 'bottom_right' above; this dict stays as-is.
#
# 600mm offset = each anchor sits 600mm from the table edge.
#
# ON OFFICIAL MATCH TABLES all 4 anchors are visible → 4-point homography,
# best accuracy.  ON YOUR TRAINING TABLE only 2 are visible (BR+TR) →
# TwinVision falls back to a cached homography from when 4 were seen, OR
# a 2-anchor + heading similarity if you've extended it.  Either way,
# this dict is the source of truth for the physical positions.
ANCHORS = {
    'top_left':     {'tag_id': 20, 'x_mm':  600, 'y_mm':  600},
    'top_right':    {'tag_id': 21, 'x_mm': 2400, 'y_mm':  600},
    'bottom_right': {'tag_id': 23, 'x_mm': 2400, 'y_mm': 2400},
    'bottom_left':  {'tag_id': 22, 'x_mm':  600, 'y_mm': 2400},
}


# ═══════════════════════════════════════════════════════════════════════════
# 3. NODE TUNING
# ═══════════════════════════════════════════════════════════════════════════

# ArUco detection
ARUCO_DICT   = '4x4_50'
ARUCO_REFINE = 'subpix'   # 'none' | 'subpix' | 'contour' | 'apriltag'

# Camera position (manual override).  Entered in YOUR WORLD FRAME — the same
# coordinates you'd read off the table with a tape measure relative to the
# world origin chosen above.  Downstream nodes (parallax / localization)
# convert internally to TwinVision native when they need to.
CAMERA_X_MM = 1275.0   # X in world frame (mm)
CAMERA_Y_MM = -200.0   # Y in world frame (mm) — negative if camera sits
                       #     behind the world Y=0 edge of the table
CAMERA_Z_MM = 1600.0   # Z = height above the floor (frame-independent)

# Parallax / robot tag height
ROBOT_Z_MM = 490.0

# Pipeline
FPS_LIMIT = 25
TEAM      = 'blue'        # 'blue' or 'yellow'
TRACK_IDS = list(range(1, 11))


# ═══════════════════════════════════════════════════════════════════════════
# 4. PIPELINE WIRING
# ═══════════════════════════════════════════════════════════════════════════

def _add(p: Pipeline, kind: str, node_id: str, params: dict | None = None):
    """Instantiate a registered node kind and attach it. Returns the id."""
    cls = NODE_KINDS.get(kind)
    if cls is None:
        raise KeyError(f'unknown node kind {kind!r} — registered: '
                       f'{sorted(NODE_KINDS)}')
    p.add_node(node_id, kind, cls(params or {}))
    return node_id


def build_pipeline(source_kind: str, source_path: str) -> Pipeline:
    p = Pipeline(name='manual', fps_limit=FPS_LIMIT)

    # ── Source ──────────────────────────────────────────────────────
    # Typically the runner reads from the vision_camera virtual camera
    # (http://127.0.0.1:5174/stream.mjpg). Pass --use-camera-server on
    # the CLI to override the source_path with that URL.
    if source_kind == 'video':
        src = _add(p, 'source.video', 'src',
                   {'path': source_path, 'loop': True,
                    'playback': 'live', 'speed': 1.0})
    elif source_kind == 'camera':
        src = _add(p, 'source.camera', 'src',
                   {'device': int(source_path) if source_path.isdigit() else 0,
                    'playback': 'live'})
    elif source_kind == 'image':
        src = _add(p, 'source.image', 'src',
                   {'path': source_path, 'playback': 'once'})
    else:
        raise ValueError(f'unknown source_kind {source_kind!r}')

    # ── Preprocess + ArUco ─────────────────────────────────────────
    pre = _add(p, 'preprocess', 'pre', {'mode': 'passthrough'})
    p.connect(src, 'frame', pre, 'frame')

    aru = _add(p, 'aruco', 'aruco',
               {'dict': ARUCO_DICT, 'refine': ARUCO_REFINE,
                'draw_markers': True, 'draw_ids': True})
    p.connect(pre, 'frame', aru, 'frame')

    # ── Rectify (defines the WORLD FRAME for everyone downstream) ──
    rec = _add(p, 'rectify', 'rectify',
               {'anchors':              ANCHORS,
                'world_origin_corner':  WORLD_ORIGIN_CORNER,
                'world_table_w_mm':     WORLD_TABLE_W_MM,
                'world_table_h_mm':     WORLD_TABLE_H_MM,
                'world_flip_theta':     WORLD_FLIP_THETA,
                'draw_grid':            False})
    p.connect(pre, 'frame',     rec, 'frame')
    p.connect(aru, 'detection', rec, 'detection')

    # ── Localization (parallax-corrected, world-frame outputs) ─────
    loc = _add(p, 'localization', 'loc',
               {'anchors':     ANCHORS,
                'cam_mode':    'auto',
                'team':        TEAM,
                'track_ids':   TRACK_IDS,
                'robot_z_mm':  ROBOT_Z_MM,
                'preview_use': 'naive'})
    p.connect(aru, 'detection',        loc, 'detection')
    p.connect(rec, 'frame',            loc, 'frame')
    p.connect(rec, 'homography_state', loc, 'rectifier_state')
    p.connect(rec, 'coord_state',      loc, 'coord_state')   # ← world frame

    # ── Manual camera + parallax (debug-friendly second pass) ──────
    cam = _add(p, 'camera.manual', 'cam',
               {'cam_x_mm':    CAMERA_X_MM,
                'cam_y_mm':    CAMERA_Y_MM,
                'cam_z_mm':    CAMERA_Z_MM,
                'coord_frame': 'world'})        # values entered in world frame

    par = _add(p, 'parallax', 'par',
               {'robot_z_mm':          ROBOT_Z_MM,
                'use_naive_xy':        True,
                'preview_draw_camera': True,
                'preview_draw_grid':   False})
    p.connect(loc, 'pose_list',   par, 'pose_list')
    p.connect(cam, 'cam_xyz',     par, 'cam_xyz')
    p.connect(rec, 'frame',       par, 'frame')
    p.connect(rec, 'coord_state', par, 'coord_state')        # ← world frame

    # ── Grid overlay (labels in world frame) ────────────────────────
    grid = _add(p, 'overlay.grid', 'grid',
                {'step_mm':          200,
                 'table_w_mm':       int(WORLD_TABLE_W_MM),
                 'table_h_mm':       int(WORLD_TABLE_H_MM),
                 'draw_labels':      True,
                 'draw_table_edge':  True})
    p.connect(par, 'preview',     grid, 'frame')
    p.connect(rec, 'coord_state', grid, 'coord_state')       # ← world frame

    # ── Dashboard outputs ──────────────────────────────────────────
    # ArUco debug — raw camera frame with detected markers + IDs drawn.
    # Useful to confirm detection is working (and where) before any
    # rectification / parallax math kicks in.
    o_aruco = _add(p, 'output', 'out_aruco',
                   {'feed_id': 'aruco', 'label': 'ArUco detection (raw frame)'})
    p.connect(aru, 'preview', o_aruco, 'preview')

    o1 = _add(p, 'output', 'out_grid',
              {'feed_id': 'main', 'label': 'BEV + grid + parallax'})
    p.connect(grid, 'preview', o1, 'preview')

    o2 = _add(p, 'output.pose_list', 'out_poses',
              {'feed_id': 'poses', 'label': 'Poses (world frame)'})
    p.connect(loc, 'pose_list', o2, 'pose_list')

    o3 = _add(p, 'output.json', 'out_coord',
              {'feed_id': 'coord', 'label': 'Coord state'})
    p.connect(rec, 'coord_state', o3, 'json')

    return p
