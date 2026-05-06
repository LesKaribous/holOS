"""
overlay node — composites debug overlays on top of an input frame.

Takes a frame (required) and any combination of optional debug streams:

    detection (DetectionResult, optional)
        Draws ArUco markers + IDs in pixel space. Same renderer as the
        ArucoDetector.draw() — useful when you want to see detections on
        top of, say, an undistorted frame instead of the raw one.

    pose_list (POSE_LIST, optional)
        Draws coloured dots at each pose's (x_mm, y_mm). Reads
        `pose_view_kind`:
            'bev'   — interprets x_mm/y_mm directly as the rectified BEV
                      pixel space ((mm + bev_margin_mm) * bev_scale).
                      Use when wired downstream of `rectify`.
            'raw'   — currently a no-op (drawing poses on a non-rectified
                      frame requires the inverse projection; not done
                      here to keep the node lightweight). Wire to a
                      rectified frame instead.

The output is a frame ready to feed into an `output` node.

This is a pure debug aid — drawing happens on a copy so the input is
never mutated.
"""

from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_VISION_SRC = os.path.abspath(os.path.join(_HERE, '..', '..', '..', 'vision', 'src'))
if _VISION_SRC not in sys.path:
    sys.path.insert(0, _VISION_SRC)

try:
    import cv2
    from core.aruco_detector import ArucoDetector, DetectionResult
    _CV2_OK = True
except Exception:
    _CV2_OK = False

from ..pipeline import NodeIO, Port, PortKind
from .base import Node, register_node
from ._coords_helpers import coord_state_or_none, world_mm_to_bev_pixel


@register_node('overlay')
class OverlayNode(Node):
    IO = NodeIO(
        inputs=[
            # Accept either a clean FRAME or an already-annotated PREVIEW —
            # editor validation allows FRAME→PREVIEW (a clean frame is a
            # valid backdrop to overlay onto) but blocks PREVIEW→FRAME.
            Port('frame',     PortKind.PREVIEW,
                 description='backdrop image (FRAME or PREVIEW both OK)'),
            Port('detection', PortKind.DETECTION, optional=True,
                 description='draws ArUco markers + IDs in pixel space'),
            Port('pose_list', PortKind.POSE_LIST, optional=True,
                 description='draws dots at each pose\'s (x_mm, y_mm); '
                             'requires pose_view_kind = "bev"'),
            Port('coord_state', PortKind.JSON, optional=True,
                 description='wire from rectify.coord_state — pose_list '
                             'coords are then interpreted as world-frame '
                             'mm and projected back through the inverse '
                             'flip onto the (BEV-native) image.'),
        ],
        outputs=[Port('preview', PortKind.PREVIEW, 'frame with overlays')],
    )
    params_schema = {
        'pose_view_kind': {
            'type': 'str', 'default': 'bev', 'enum': ['raw', 'bev'],
            'label': 'pose interpretation',
            'description': 'how to project pose_list xy onto the frame: '
                           'bev = rectified pixels (mm * scale), '
                           'raw = no-op (skipped)',
        },
        'bev_scale': {
            'type': 'float', 'default': 0.25, 'min': 0.05, 'max': 1.0, 'step': 0.05,
            'label': 'BEV pixel/mm scale', 'unit': 'px/mm',
            'description': 'must match the rectify node (default 0.25)',
        },
        'bev_margin_mm': {
            'type': 'float', 'default': 100.0, 'min': 0, 'max': 500, 'step': 10,
            'label': 'BEV margin', 'unit': 'mm',
            'description': 'must match the rectify node (default 100)',
        },
        'dot_radius_px': {
            'type': 'int', 'default': 8, 'min': 2, 'max': 30,
            'label': 'pose dot radius', 'unit': 'px',
        },
        'draw_aruco_ids': {
            'type': 'bool', 'default': True,
            'label': 'draw ArUco IDs',
        },
        'restrict_aruco_to_pose_list': {
            'type': 'bool', 'default': True,
            'label': 'restrict ArUco to pose_list',
            'description': 'when both detection AND pose_list are wired, '
                           'only draw aruco markers whose ids appear in '
                           'pose_list. Lets upstream filters propagate to '
                           'the overlay. Off = draw every detected marker.',
        },
    }

    # Distinct colors per classification — matches the dashboard scheme.
    _CLASS_COLOR_BGR = {
        'own':       (0, 220, 0),     # green
        'opponent':  (0, 80, 255),    # red-ish
        'anchor':    (0, 165, 255),   # orange
        'object':    (200, 200, 200), # grey
    }

    def __init__(self, params=None):
        super().__init__(params)
        self._aruco_dummy = None  # reused ArucoDetector for its draw() helper

    def start(self):
        if not _CV2_OK:
            self._last_error = 'cv2 unavailable'
            return
        # We only use ArucoDetector for its draw() helper — no detection.
        try:
            self._aruco_dummy = ArucoDetector('4x4_50')
        except Exception:
            self._aruco_dummy = None

    def process(self, inputs):
        frame = inputs.get('frame')
        if frame is None or not _CV2_OK:
            return {}
        out = frame.copy()
        det   = inputs.get('detection')
        poses = inputs.get('pose_list')

        # ── ArUco marker overlay ───────────────────────────────────────
        # When both detection and pose_list are wired, the user's filter
        # chain already says "only these tags matter". Subset the
        # detection so visually we only highlight the surviving tags
        # (otherwise upstream filters get bypassed by the overlay and
        # you see every raw-detected marker including anchors etc).
        if det is not None and self._aruco_dummy is not None:
            try:
                draw_det = det
                if (poses is not None
                        and self._params.get('restrict_aruco_to_pose_list', True)):
                    keep_ids = {q.get('tag_id') for q in poses
                                if isinstance(q, dict)}
                    keep_corners, keep_ids_list = [], []
                    for c, tid in zip(det.corners, det.ids):
                        if int(tid) in keep_ids:
                            keep_corners.append(c)
                            keep_ids_list.append(int(tid))
                    draw_det = DetectionResult(
                        corners=keep_corners, ids=keep_ids_list, rejected=[],
                    )
                out = self._aruco_dummy.draw(
                    out, draw_det,
                    draw_ids=bool(self._params.get('draw_aruco_ids', True)),
                )
            except Exception as e:
                self._last_error = f'aruco draw: {e}'

        # ── Pose dot overlay ──────────────────────────────────────────
        if poses and self._params.get('pose_view_kind', 'bev') == 'bev':
            scale  = float(self._params.get('bev_scale', 0.25))
            margin = float(self._params.get('bev_margin_mm', 100.0))
            radius = int(self._params.get('dot_radius_px', 8))
            cs = coord_state_or_none(inputs.get('coord_state'))
            for q in poses:
                if not isinstance(q, dict):
                    continue
                x_mm, y_mm = q.get('x_mm'), q.get('y_mm')
                if x_mm is None or y_mm is None:
                    continue
                # Project from world frame back to BEV pixels (helper
                # is a no-op when cs is None).
                pt = world_mm_to_bev_pixel(x_mm, y_mm, cs, margin, scale)
                if pt is None:
                    continue
                px, py = pt
                color = self._CLASS_COLOR_BGR.get(
                    str(q.get('classification', 'object')),
                    self._CLASS_COLOR_BGR['object'],
                )
                try:
                    cv2.circle(out, (px, py), radius, color, -1)
                    cv2.circle(out, (px, py), radius, (255, 255, 255), 1)
                    # Label with tag id
                    tid = q.get('tag_id')
                    if tid is not None:
                        cv2.putText(out, f'#{tid}',
                                    (px + radius + 2, py + 4),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                                    color, 1, cv2.LINE_AA)
                    # Heading arrow when theta is known. theta is in
                    # world frame (post-flip) — undo the flip for drawing
                    # on the BEV-native image.
                    th = q.get('theta_rad')
                    if th is not None:
                        import math as _m
                        thf = float(th)
                        if not _m.isnan(thf):
                            if cs is not None:
                                if cs.get('flip_x'):
                                    thf = _m.pi - thf
                                if cs.get('flip_y'):
                                    thf = -thf
                            arrow = radius * 3
                            hx = int(px + arrow * _m.cos(thf))
                            hy = int(py + arrow * _m.sin(thf))
                            cv2.arrowedLine(out, (px, py), (hx, hy), color,
                                            2, tipLength=0.3)
                except Exception as e:
                    self._last_error = f'pose draw: {e}'

        return {'preview': out}

    def get_state(self):
        return {'last_error': self._last_error}


# ── BEV grid overlay (standalone) ────────────────────────────────────────


@register_node('overlay.grid')
class GridOverlayNode(Node):
    """Draws a millimetre grid + table edge on a rectified BEV frame.

    Lifted out of the rectify node so the user can compose it with
    other overlays. Wire it AFTER rectify (or any frame node), and BEFORE
    `output` if you want the grid baked into the dashboard tile.

    Assumes the frame is in BEV pixel space:
        pixel_x = (mm_x + margin_mm) * scale
    Match `scale` and `margin_mm` to your rectify node's settings (defaults
    0.25 px/mm and 100 mm — same defaults as rectify so they line up
    out of the box).
    """
    IO = NodeIO(
        inputs=[
            Port('frame', PortKind.PREVIEW,
                 description='backdrop image (FRAME or PREVIEW both OK)'),
            Port('coord_state', PortKind.JSON, optional=True,
                 description='wire from rectify.coord_state — when wired, '
                             'grid labels are shown in the WORLD frame '
                             '(label values are user mm) but lines stay on '
                             'the BEV image where they correspond.'),
        ],
        outputs=[Port('preview', PortKind.PREVIEW, 'frame with grid + edges')],
    )
    params_schema = {
        'step_mm': {
            'type': 'int', 'default': 200, 'min': 50, 'max': 1000, 'step': 50,
            'label': 'grid step', 'unit': 'mm',
        },
        'scale': {
            'type': 'float', 'default': 0.25, 'min': 0.05, 'max': 1.0, 'step': 0.05,
            'label': 'BEV scale', 'unit': 'px/mm',
            'description': 'must match the rectify node',
        },
        'margin_mm': {
            'type': 'float', 'default': 100.0, 'min': 0, 'max': 500, 'step': 10,
            'label': 'BEV margin', 'unit': 'mm',
            'description': 'must match the rectify node',
        },
        'table_w_mm': {
            'type': 'int', 'default': 3000, 'min': 500, 'max': 10000, 'step': 100,
            'label': 'table width', 'unit': 'mm',
        },
        'table_h_mm': {
            'type': 'int', 'default': 2000, 'min': 500, 'max': 10000, 'step': 100,
            'label': 'table height', 'unit': 'mm',
        },
        'draw_labels': {
            'type': 'bool', 'default': True,
            'label': 'draw mm labels',
        },
        'draw_table_edge': {
            'type': 'bool', 'default': True,
            'label': 'draw table border',
        },
        'color_r': {'type': 'int', 'default':   0, 'min': 0, 'max': 255, 'label': 'grid R'},
        'color_g': {'type': 'int', 'default': 200, 'min': 0, 'max': 255, 'label': 'grid G'},
        'color_b': {'type': 'int', 'default':  80, 'min': 0, 'max': 255, 'label': 'grid B'},
    }

    def process(self, inputs):
        frame = inputs.get('frame')
        if frame is None or not _CV2_OK:
            return {}
        out = frame.copy()
        step  = max(1, int(self._params.get('step_mm', 200)))
        scale = float(self._params.get('scale', 0.25))
        m     = float(self._params.get('margin_mm', 100.0))
        tw    = int(self._params.get('table_w_mm', 3000))
        th    = int(self._params.get('table_h_mm', 2000))
        color = (
            int(self._params.get('color_b', 80)),
            int(self._params.get('color_g', 200)),
            int(self._params.get('color_r',   0)),
        )

        def mm_to_px(x_mm, y_mm):
            return (int(round((x_mm + m) * scale)),
                    int(round((y_mm + m) * scale)))

        # If coord_state is wired, grid LINES still draw in BEV-native
        # frame (the image hasn't moved), but LABEL VALUES reflect the
        # user-world frame. The line at BEV x_mm=2400 is at world x=600
        # when flip_x is on, etc.
        cs = coord_state_or_none(inputs.get('coord_state'))
        fx = bool(cs.get('flip_x')) if cs else False
        fy = bool(cs.get('flip_y')) if cs else False
        wtw = float(cs.get('table_w_mm', tw)) if cs else float(tw)
        wth = float(cs.get('table_h_mm', th)) if cs else float(th)

        def x_label_at(x_mm_bev):
            return int(round(wtw - x_mm_bev)) if fx else int(x_mm_bev)
        def y_label_at(y_mm_bev):
            return int(round(wth - y_mm_bev)) if fy else int(y_mm_bev)

        # Grid lines
        try:
            for x_mm in range(0, tw + 1, step):
                p0 = mm_to_px(x_mm, 0); p1 = mm_to_px(x_mm, th)
                cv2.line(out, p0, p1, color, 1, cv2.LINE_AA)
                if self._params.get('draw_labels', True) and 0 < x_mm < tw:
                    cv2.putText(out, str(x_label_at(x_mm)),
                                (p0[0] + 2, p0[1] + 14),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, color,
                                1, cv2.LINE_AA)
            for y_mm in range(0, th + 1, step):
                p0 = mm_to_px(0, y_mm); p1 = mm_to_px(tw, y_mm)
                cv2.line(out, p0, p1, color, 1, cv2.LINE_AA)
                if self._params.get('draw_labels', True) and 0 < y_mm < th:
                    cv2.putText(out, str(y_label_at(y_mm)),
                                (p0[0] + 2, p0[1] - 2),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, color,
                                1, cv2.LINE_AA)
            # Table border (a touch thicker, red, regardless of color params)
            if self._params.get('draw_table_edge', True):
                cv2.rectangle(out, mm_to_px(0, 0), mm_to_px(tw, th),
                              (0, 0, 220), 2)
        except Exception as e:
            self._last_error = f'draw: {e}'
        return {'preview': out}

    def get_state(self):
        return {'last_error': self._last_error}
