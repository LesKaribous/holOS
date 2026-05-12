"""
localization node — wraps the TwinVision RobotTracker so the pipeline can
emit world-space poses (with parallax correction + Kalman smoothing) for
configured robot tag IDs.

Inputs
------
    detection (DetectionResult) : output of an aruco node
    rectifier_state (json)      : optional — { has_h, fresh } from a rectify
                                  node (used only to decide whether to skip
                                  the per-frame solve).
    frame (frame, optional)     : carries image_size — needed for solvePnP
                                  cam-pose estimation. Wire from the same
                                  frame the aruco detector saw.

Outputs
-------
    pose_list (pose_list) : [{tag_id, label, x_mm, y_mm, theta_rad,
                              naive_x_mm, naive_y_mm, classification}]
    cam_xyz (json)        : estimated camera position (mm) — useful for the
                            UI, optional for downstream nodes.

Params
------
    intrinsics_path : path to camera_intrinsics.json (defaults to TwinVision)
    anchors         : 4-anchor dict (same shape as rectify node)
    robot_z_mm      : tag height above the floor (parallax)
    cam_mode        : 'auto' (PnP via intrinsics) | 'manual'
    cam_override    : [x, y, z] mm — used when cam_mode == 'manual'
    track_ids       : list of tag IDs to follow (e.g. [1,2,3,4,5,6,7,8,9,10]).
                      Each becomes an OWN/OPP based on `team`.
    team            : 'blue' (own=1..5) | 'yellow' (own=6..10)
"""

from __future__ import annotations

import os
import sys
from typing import Optional

_HERE = os.path.dirname(os.path.abspath(__file__))
_VISION_SRC = os.path.abspath(os.path.join(_HERE, '..', '..', '..', 'vision', 'src'))
if _VISION_SRC not in sys.path:
    sys.path.insert(0, _VISION_SRC)

try:
    import cv2
    import numpy as np
    from core.aruco_detector import DetectionResult
    from core.table_rectifier import TableRectifier, CornerConfig
    from core.robot_tracker import RobotTracker, RobotConfig
    from core.charuco_calib import CameraIntrinsics
    _CV2_OK = True
except Exception:
    _CV2_OK = False

from ..pipeline import NodeIO, Port, PortKind
from .base import Node, register_node
from ._coords_helpers import (
    coord_state_or_none, flip_pose_list, flip_cam_xyz,
    world_mm_to_bev_pixel,
)

_DEFAULT_INTRINSICS = os.path.abspath(
    os.path.join(_HERE, '..', '..', '..', 'vision', 'calibrations', 'camera_intrinsics.json')
)
_DEFAULT_ANCHORS = {
    'top_left':     {'tag_id': 23, 'x_mm': 600,  'y_mm': 600},
    'top_right':    {'tag_id': 22, 'x_mm': 2400, 'y_mm': 600},
    'bottom_right': {'tag_id': 20, 'x_mm': 2400, 'y_mm': 1400},
    'bottom_left':  {'tag_id': 21, 'x_mm': 600,  'y_mm': 1400},
}
_DEFAULT_TRACK_IDS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

_TEAM_RANGES = {
    'blue':   {'own': set(range(1, 6)),  'opp': set(range(6, 11))},
    'yellow': {'own': set(range(6, 11)), 'opp': set(range(1, 6))},
}


@register_node('localization')
class LocalizationNode(Node):
    IO = NodeIO(
        inputs=[
            Port('detection',       PortKind.DETECTION),
            Port('rectifier_state', PortKind.JSON, optional=True),
            Port('rectifier',       PortKind.JSON, optional=True,
                 description='wire from rectify.rectifier to share the same '
                             'TableRectifier instance (so the homography is '
                             'computed once, in the rectify node, instead of '
                             'duplicated here — keeps sim2 / h4 / auto modes '
                             'consistent between the two nodes).'),
            Port('frame',           PortKind.FRAME, optional=True),
            Port('cam_xyz',         PortKind.JSON, optional=True,
                 description='override the internal solvePnP camera estimate'),
            Port('coord_state',     PortKind.JSON, optional=True,
                 description='wire from rectify.coord_state to flip outputs '
                             '(pose_list + cam_xyz) into your world frame'),
        ],
        outputs=[
            Port('pose_list', PortKind.POSE_LIST,
                 'list of {tag_id, x_mm, y_mm, theta_rad, classification}'),
            Port('cam_xyz', PortKind.JSON,
                 'camera position used this frame (input override or internal estimate)'),
            Port('preview', PortKind.PREVIEW,
                 'frame with the localized poses drawn (only when frame wired)'),
        ],
    )
    params_schema = {
        'intrinsics_path': {'type': 'str',  'default': _DEFAULT_INTRINSICS},
        'anchors':         {'type': 'dict', 'default': _DEFAULT_ANCHORS},
        'robot_z_mm':      {'type': 'float','default': 490.0,
                            'help': 'tag height above floor (mm)'},
        'cam_mode':        {'type': 'str',  'default': 'auto',
                            'enum': ['auto', 'manual']},
        'cam_override_x':  {'type': 'float','default': 1500.0},
        'cam_override_y':  {'type': 'float','default': -200.0},
        'cam_override_z':  {'type': 'float','default': 1600.0},
        'track_ids':       {'type': 'json', 'default': _DEFAULT_TRACK_IDS},
        'team':            {'type': 'str',  'default': 'blue',
                            'enum': ['blue', 'yellow']},
        'preview_view_kind':     {'type': 'str',   'default': 'bev',
                                  'enum': ['raw', 'bev'],
                                  'label': 'preview projection',
                                  'description': 'bev = mm * scale on a '
                                                 'rectified frame; raw = '
                                                 'pixel space from the '
                                                 'original frame'},
        'preview_use':           {'type': 'str',   'default': 'naive',
                                  'enum': ['naive', 'corrected'],
                                  'label': 'preview source xy',
                                  'description': 'naive = projection of '
                                                 'the tag pixel through '
                                                 'homography (matches what '
                                                 'you see on the BEV image '
                                                 '— use this to verify '
                                                 'rectification). corrected '
                                                 '= parallax-corrected '
                                                 'ground position (the '
                                                 'truth — will be offset '
                                                 'from the BEV marker '
                                                 'because the marker is '
                                                 'elevated)'},
        'preview_bev_scale':     {'type': 'float', 'default': 0.25,
                                  'min': 0.05, 'max': 1.0, 'step': 0.05,
                                  'label': 'preview BEV scale', 'unit': 'px/mm'},
        'preview_bev_margin_mm': {'type': 'float', 'default': 100.0,
                                  'min': 0, 'max': 500, 'step': 10,
                                  'label': 'preview BEV margin', 'unit': 'mm'},
        'preview_dot_radius_px': {'type': 'int',   'default': 8,
                                  'min': 2, 'max': 30,
                                  'label': 'preview dot radius', 'unit': 'px'},
        'preview_draw_camera':   {'type': 'bool',  'default': False,
                                  'label': 'preview: draw camera',
                                  'description': 'plot the camera ground '
                                                 'projection (Xc, Yc) as a '
                                                 'magenta cross'},
        'preview_draw_grid':     {'type': 'bool',  'default': False,
                                  'label': 'preview: draw mm grid',
                                  'description': 'overlay a 200 mm grid on '
                                                 'the BEV preview'},
    }

    # ── Lifecycle ─────────────────────────────────────────────────────

    def __init__(self, params=None):
        super().__init__(params)
        # Pre-init the attributes start() will populate, so get_state()
        # called before the worker reaches start() doesn't AttributeError.
        self._rect    = None
        self._intr    = None
        self._tracker = None

    def start(self):
        if not _CV2_OK:
            self._last_error = 'cv2 unavailable'
            return
        # Build a TableRectifier just to host anchor positions + reuse its
        # raw_pixel_to_table_mm. We don't render through it — the rectify
        # node does that. Sharing a single rectifier across pipeline nodes
        # would be cleaner, but per-node ownership keeps the graph simple.
        anchors_cfg = self._params.get('anchors', _DEFAULT_ANCHORS)
        self._rect = TableRectifier(CornerConfig.from_dict(anchors_cfg))
        intr_path = self._params.get('intrinsics_path', _DEFAULT_INTRINSICS)
        self._intr = None
        if os.path.exists(intr_path):
            try:
                self._intr = CameraIntrinsics.load(intr_path)
                self._rect.set_intrinsics(self._intr)
            except Exception as e:
                self._last_error = f'intrinsics load: {e}'
        # Build the tracker
        cam_mode = self._params.get('cam_mode', 'auto')
        cam_override = (
            float(self._params.get('cam_override_x', 1500.0)),
            float(self._params.get('cam_override_y', -200.0)),
            float(self._params.get('cam_override_z', 1600.0)),
        )
        self._tracker = RobotTracker(
            cam_mode=cam_mode,
            cam_override=cam_override,
            smooth_calib=True,
            trail_len=40,
        )
        if self._intr is not None:
            self._tracker.set_camera_intrinsics(self._intr.K, self._intr.dist)
        # Disable the Kalman smoother. Localization is pull-based now —
        # the strategy queries the latest pose only when it wants one,
        # so temporal smoothing actively HURTS by:
        #   (1) lagging behind real motion (low-pass effect),
        #   (2) producing predict-only "ghost" poses when the tag is
        #       briefly lost (we already filter those, but they cost CPU).
        # Emitting the raw parallax-corrected position is what the user
        # asked for: "current localization when available, nothing else".
        self._tracker.set_use_kalman(False)
        self._sync_robot_configs()

    def _sync_robot_configs(self):
        """(Re)build the tracker's robot list to match track_ids + team."""
        track_ids = self._params.get('track_ids', _DEFAULT_TRACK_IDS) or []
        if isinstance(track_ids, str):
            # Permit "1,2,3" as a fallback
            try:
                track_ids = [int(x.strip()) for x in track_ids.split(',') if x.strip()]
            except ValueError:
                track_ids = _DEFAULT_TRACK_IDS
        z_mm = float(self._params.get('robot_z_mm', 490.0))
        ranges = _TEAM_RANGES.get(self._params.get('team', 'blue'),
                                  _TEAM_RANGES['blue'])
        configs = []
        for tid in sorted(set(int(x) for x in track_ids)):
            if tid in ranges['own']:
                color = (0, 220, 0); label = f'OWN-{tid}'
            elif tid in ranges['opp']:
                color = (0, 80, 255); label = f'OPP-{tid}'
            else:
                color = (180, 180, 180); label = f'OBJ-{tid}'
            configs.append(RobotConfig(
                tag_id=tid, label=label, color_bgr=color, z_mm=z_mm,
            ))
        self._tracker.set_robots(configs)

    # ── Per-tick processing ───────────────────────────────────────────

    def process(self, inputs):
        det = inputs.get('detection')
        frame = inputs.get('frame')
        if det is None or getattr(self, '_tracker', None) is None:
            return {}
        # Prefer the rectifier wired in from the rectify node (single source
        # of truth — already updated this tick with the right homography
        # mode: h4, sim2 or auto). Fall back to our own when not wired so
        # legacy graphs without the wire keep working.
        wired_rect = inputs.get('rectifier')
        if wired_rect is not None:
            rect = wired_rect
        else:
            rect = self._rect
            # No upstream rectifier — refresh anchor pose ourselves. Same
            # logic as the legacy match-time backend.
            if rect.intrinsics is not None:
                rect.update_pose(det)
            else:
                rect.update(det)

        if not rect.has_homography:
            # No homography → cannot project tag pixel coords into the
            # world frame. We still emit a pose_list with the detected
            # own/opp tag IDs + null positions so the dashboard cards
            # populate (tag_id + classification + "—" for x/y/theta).
            # Without this the operator can't tell "no detection" from
            # "detection works but homography is broken".
            ranges = _TEAM_RANGES.get(self._params.get('team', 'blue'),
                                      _TEAM_RANGES['blue'])
            pose_list = []
            for tid in (det.ids or []):
                tid = int(tid)
                if tid in ranges['own']:
                    cls = 'own'
                elif tid in ranges['opp']:
                    cls = 'opponent'
                else:
                    continue
                pose_list.append({
                    'tag_id':         tid,
                    'label':          f'tag {tid}',
                    'x_mm':           None,
                    'y_mm':           None,
                    'naive_x_mm':     None,
                    'naive_y_mm':     None,
                    'theta_rad':      None,
                    'classification': cls,
                })
            return {'pose_list': pose_list}

        # If a cam_xyz override is wired in, push it into the tracker as
        # a manual override BEFORE process() so the parallax math uses
        # the user's truth instead of the internal solvePnP guess.
        cam_in = inputs.get('cam_xyz') if isinstance(inputs.get('cam_xyz'), dict) else None
        if cam_in is not None:
            try:
                # RobotTracker works in TwinVision-native frame internally
                # (uses anchor mm coords for projection). If the wired cam
                # is tagged 'world' and we have a coord_state to flip
                # back, mirror it to TwinVision before handing it over.
                cam_for_tracker = cam_in
                cs_for_cam = coord_state_or_none(inputs.get('coord_state'))
                if (cs_for_cam is not None
                        and str(cam_in.get('frame', 'twin_vision')) == 'world'):
                    cam_for_tracker = flip_cam_xyz(cam_in, cs_for_cam)
                self._tracker.set_cam_mode('manual')
                self._tracker.set_cam_override((
                    float(cam_for_tracker.get('x_mm', 1500.0)),
                    float(cam_for_tracker.get('y_mm', -200.0)),
                    float(cam_for_tracker.get('z_mm', 1600.0)),
                ))
            except Exception:
                pass

        h, w = (frame.shape[:2] if frame is not None else (1080, 1920))
        try:
            states = self._tracker.process(det, rect, (w, h))
        except Exception as e:
            self._last_error = f'tracker: {e}'
            return {}

        ranges = _TEAM_RANGES.get(self._params.get('team', 'blue'),
                                  _TEAM_RANGES['blue'])
        pose_list = []
        import math as _m
        def _safe(v):
            f = float(v)
            return None if (_m.isnan(f) or _m.isinf(f)) else f
        for tid, st in states.items():
            # Skip predict-only states. RobotTracker emits a Kalman-
            # predicted pose for tags that weren't detected this frame —
            # which makes ghost rows appear in the Objects tab and pose
            # lists (with coords drifting because the Kalman keeps
            # projecting forward). Strict mode: only emit poses backed by
            # an actual ArUco detection on the current tick. The tracker
            # signals "no fresh detection" by setting pixel_center to NaN.
            px, py = st.pixel_center
            if (px is None or py is None
                    or _m.isnan(float(px)) or _m.isnan(float(py))):
                continue
            if tid in ranges['own']:
                cls = 'own'
            elif tid in ranges['opp']:
                cls = 'opponent'
            else:
                cls = 'object'
            pose_list.append({
                'tag_id':         tid,
                'label':          st.label,
                'x_mm':           _safe(st.pos_mm[0]),
                'y_mm':           _safe(st.pos_mm[1]),
                'naive_x_mm':     _safe(st.naive_mm[0]),
                'naive_y_mm':     _safe(st.naive_mm[1]),
                'theta_rad':      _safe(st.theta_rad),
                'classification': cls,
            })

        cam_xyz = None
        if self._tracker.cam_pos is not None:
            cam_xyz = {
                'x_mm':   float(self._tracker.cam_pos[0]),
                'y_mm':   float(self._tracker.cam_pos[1]),
                'z_mm':   float(self._tracker.cam_pos[2]),
                'source': self._tracker.cam_source,
                'frame':  'twin_vision',
            }

        # ── Apply coord_state flip if wired from rectify ────────────
        # Internal computations live in TwinVision native frame (origin
        # top-left, X→right, Y→bottom). When rectify pins a world frame,
        # we flip the outputs to match. Preview drawing happens AFTER the
        # flip so dots project correctly onto the (already flipped) BEV.
        cs = coord_state_or_none(inputs.get('coord_state'))
        if cs is not None and (cs.get('flip_x') or cs.get('flip_y')):
            pose_list = flip_pose_list(pose_list, cs)
            if cam_xyz is not None:
                cam_xyz = flip_cam_xyz(cam_xyz, cs)
                cam_xyz['frame'] = 'world'

        result = {'pose_list': pose_list, 'cam_xyz': cam_xyz}

        # Optional preview — only the localized (= filtered to track_ids,
        # team-classified) poses get drawn on the input frame. Use a BEV
        # backdrop (wired from rectify) for view_kind='bev'; raw is a no-op.
        if frame is not None and _CV2_OK:
            try:
                vis = frame.copy()
                view_kind = str(self._params.get('preview_view_kind', 'bev'))
                if view_kind == 'bev':
                    scale  = float(self._params.get('preview_bev_scale', 0.25))
                    margin = float(self._params.get('preview_bev_margin_mm', 100.0))
                    radius = int(self._params.get('preview_dot_radius_px', 8))
                    use_naive = (self._params.get('preview_use', 'naive')
                                 == 'naive')
                    # _project_bev — input is BEV-native mm, output BEV pixel.
                    #   Use for grid lines that span the full BEV.
                    # _project_world — input is world-frame mm (the frame
                    #   pose_list / cam_xyz are emitted in when cs is wired),
                    #   output BEV pixel. The helper inverts the flip so
                    #   dots land at the right visual position on the
                    #   (unflipped) BEV.
                    def _project_bev(x_mm, y_mm):
                        try:
                            return (int(round((float(x_mm) + margin) * scale)),
                                    int(round((float(y_mm) + margin) * scale)))
                        except (TypeError, ValueError):
                            return None
                    def _project_world(x_mm, y_mm):
                        return world_mm_to_bev_pixel(x_mm, y_mm, cs,
                                                     margin, scale)

                    # Optional grid — drawn in BEV-native frame.  Lines
                    # span the BEV directly; values along the lines reflect
                    # BEV-native mm so this is just a visual reference.
                    if self._params.get('preview_draw_grid', False):
                        gc = (0, 200, 80)
                        h_px, w_px = vis.shape[:2]
                        x_min_mm = -margin; y_min_mm = -margin
                        x_max_mm = w_px / scale - margin
                        y_max_mm = h_px / scale - margin
                        step = 200
                        for xm in range(int(x_min_mm // step) * step,
                                        int(x_max_mm) + 1, step):
                            p0 = _project_bev(xm, y_min_mm)
                            p1 = _project_bev(xm, y_max_mm)
                            if p0 and p1:
                                cv2.line(vis, p0, p1, gc, 1, cv2.LINE_AA)
                        for ym in range(int(y_min_mm // step) * step,
                                        int(y_max_mm) + 1, step):
                            p0 = _project_bev(x_min_mm, ym)
                            p1 = _project_bev(x_max_mm, ym)
                            if p0 and p1:
                                cv2.line(vis, p0, p1, gc, 1, cv2.LINE_AA)

                    # Optional camera ground projection — cam_xyz is in
                    # WORLD frame (post-flip) so use the world projector.
                    if (self._params.get('preview_draw_camera', False)
                            and cam_xyz is not None):
                        cam_pt = _project_world(cam_xyz['x_mm'],
                                                cam_xyz['y_mm'])
                        if cam_pt is not None:
                            cv2.drawMarker(vis, cam_pt, (255, 0, 255),
                                           cv2.MARKER_CROSS, radius * 3, 2)
                            cv2.putText(vis,
                                f"cam ({int(cam_xyz['x_mm'])},"
                                f"{int(cam_xyz['y_mm'])},"
                                f"{int(cam_xyz['z_mm'])})",
                                (cam_pt[0] + 8, cam_pt[1] - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                                (255, 0, 255), 1, cv2.LINE_AA)
                    color_by_cls = {
                        'own':      (0, 220, 0),
                        'opponent': (0, 80, 255),
                        'object':   (200, 200, 200),
                    }
                    for q in pose_list:
                        # naive_xy matches the BEV image (tag pixel-center
                        # projected through homography — same op the warp
                        # used). corrected_xy is the true ground position
                        # which is offset due to parallax.
                        if use_naive:
                            x = q.get('naive_x_mm', q.get('x_mm'))
                            y = q.get('naive_y_mm', q.get('y_mm'))
                        else:
                            x, y = q.get('x_mm'), q.get('y_mm')
                        if x is None or y is None:
                            continue
                        # Pose values are in WORLD frame after the flip
                        # at the top of process(); project back through
                        # the inverse onto BEV-native pixels.
                        pt = _project_world(x, y)
                        if pt is None:
                            continue
                        px, py = pt
                        col = color_by_cls.get(q.get('classification'),
                                               (200, 200, 200))
                        cv2.circle(vis, (px, py), radius, col, -1)
                        cv2.circle(vis, (px, py), radius, (255, 255, 255), 1)
                        # Heading arrow — theta is also in world frame.
                        # On the BEV-native image, mirror the angle to
                        # match the unflipped image's orientation.
                        th = q.get('theta_rad')
                        if th is not None and not _m.isnan(float(th)):
                            th_draw = float(th)
                            if cs is not None:
                                if cs.get('flip_x'):
                                    th_draw = _m.pi - th_draw
                                if cs.get('flip_y'):
                                    th_draw = -th_draw
                            arrow = radius * 3
                            hx = int(px + arrow * _m.cos(th_draw))
                            hy = int(py + arrow * _m.sin(th_draw))
                            cv2.arrowedLine(vis, (px, py), (hx, hy), col,
                                            2, tipLength=0.3)
                        tid = q.get('tag_id')
                        if tid is not None:
                            cv2.putText(vis, f'#{tid}',
                                        (px + radius + 2, py + 4),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                                        col, 1, cv2.LINE_AA)
                # raw view — leave a no-op for now (would need 3D->image proj)
                result['preview'] = vis
            except Exception as e:
                self._last_error = f'preview: {e}'

        return result

    # ── Param updates pick up live ─────────────────────────────────────

    def set_params(self, params):
        super().set_params(params)
        # Reconfigure the tracker if the team / track_ids / z_mm changed.
        try:
            self._sync_robot_configs()
        except Exception:
            pass

    def get_state(self):
        # All instance attributes set in start() may not exist if start()
        # exited early (e.g. _CV2_OK was False, or the registry was hit
        # before the worker thread got to its init). Use getattr() for
        # every field so /api/vision/pipelines never raises AttributeError.
        tr   = getattr(self, '_tracker', None)
        rect = getattr(self, '_rect',    None)
        intr = getattr(self, '_intr',    None)
        return {
            'has_homography': bool(rect and rect.has_homography),
            'has_intrinsics': intr is not None,
            'cam_source':     tr.cam_source if tr else 'none',
            'last_error':     self._last_error,
        }
