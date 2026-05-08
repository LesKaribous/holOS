"""
rectify (BEV) node — pose-based rectification using 4 anchor ArUco tags.
Re-uses the TwinVision TableRectifier (with anchor caching).

Also defines the WORLD COORDINATE FRAME for the whole pipeline.

The user picks **which BEV corner is their world origin (0, 0)** via the
`world_origin_corner` param. From that, the node derives a discrete
flip_x / flip_y and emits it as a `coord_state` JSON output that
downstream nodes (localization, parallax, overlay, overlay.grid) consume.

IMPORTANT — the BEV IMAGE is NEVER flipped. Only the data-side
convention changes. pose lists are emitted in user-world frame, grid
labels show user-world values, but the image keeps its native camera-
view orientation — so the user's spatial intuition about the table
doesn't get scrambled when they pick a different origin corner.

Mapping origin choice → flips:
    top_left      → no flip (TwinVision native: X→right, Y→down, origin TL)
    top_right     → flip_x only       (X→left, Y→down)
    bottom_left   → flip_y only       (X→right, Y→up)
    bottom_right  → flip_x + flip_y   (X→left, Y→up — common 'robotics' frame)

Inputs
    frame      — the source frame (will be undistorted internally if intr loaded)
    detection  — ArUco DetectionResult (anchor pixel positions come from here)
Outputs
    frame             — top-down BEV (BEV-native orientation, never flipped)
    preview           — same, with optional embedded grid (legacy)
    homography_state  — {has_h, fresh, cached}
    coord_state       — {flip_x, flip_y, flip_theta, table_w_mm, table_h_mm,
                         origin_corner} — wire this to localization,
                        parallax, overlay, overlay.grid.
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
    from core.table_rectifier import TableRectifier, CornerConfig
    from core.charuco_calib import CameraIntrinsics
    _CV2_OK = True
except Exception:
    _CV2_OK = False

from ..pipeline import NodeIO, Port, PortKind
from .base import Node, register_node

_DEFAULT_ANCHORS = {
    'top_left':     {'tag_id': 23, 'x_mm': 600,  'y_mm': 600},
    'top_right':    {'tag_id': 22, 'x_mm': 2400, 'y_mm': 600},
    'bottom_right': {'tag_id': 20, 'x_mm': 2400, 'y_mm': 1400},
    'bottom_left':  {'tag_id': 21, 'x_mm': 600,  'y_mm': 1400},
}
_DEFAULT_INTRINSICS = os.path.abspath(
    os.path.join(_HERE, '..', '..', '..', 'vision', 'calibrations', 'camera_intrinsics.json')
)


@register_node('rectify')
class RectifyNode(Node):
    IO = NodeIO(
        inputs=[
            Port('frame',     PortKind.FRAME),
            Port('detection', PortKind.DETECTION),
        ],
        outputs=[
            Port('frame',            PortKind.FRAME,   'rectified BEV (clean, in world frame)'),
            Port('preview',          PortKind.PREVIEW, 'rectified BEV (with optional grid)'),
            Port('homography_state', PortKind.JSON),
            Port('coord_state',      PortKind.JSON,
                 'world frame definition — wire to localization, '
                 'parallax, overlay so downstream coords are flipped '
                 'into the user frame'),
        ],
    )
    params_schema = {
        'anchors': {
            'type': 'dict', 'default': _DEFAULT_ANCHORS,
            'label': 'anchor tag IDs + table positions',
        },
        'intrinsics_path': {
            'type': 'str',  'default': _DEFAULT_INTRINSICS,
            'label': 'intrinsics file',
            'description': 'TwinVision camera_intrinsics.json (improves '
                           'rectification accuracy via solvePnP)',
        },
        'draw_grid': {
            'type': 'bool', 'default': False,
            'label': 'embed grid (legacy)',
            'description': 'OFF by default — use the standalone '
                           'overlay.grid node instead. Kept for back-compat.',
        },
        # ─── World frame definition ─────────────────────────────────
        'world_origin_corner': {
            'type': 'str', 'default': 'top_left',
            'enum': ['top_left', 'top_right', 'bottom_left', 'bottom_right'],
            'label': 'world origin (0,0) corner',
            'description': 'which corner of the BEV image is your world '
                           'origin (0, 0). top_left = TwinVision native '
                           '(no transform). bottom_right = X axis points '
                           'LEFT, Y axis points UP (common robotics frame).',
        },
        'world_table_w_mm': {
            'type': 'float', 'default': 3000.0,
            'label': 'table width (world)', 'unit': 'mm',
            'description': 'used to mirror x_mm. TwinVision native = 3000.',
        },
        'world_table_h_mm': {
            'type': 'float', 'default': 2000.0,
            'label': 'table height (world)', 'unit': 'mm',
        },
        'world_flip_theta': {
            'type': 'bool', 'default': True,
            'label': 'flip headings downstream',
            'description': 'when ON, downstream nodes mirror theta_rad '
                           'so headings stay consistent with the flipped '
                           'frame.',
        },
        # ─── Reduced-anchor mode ────────────────────────────────────
        'homography_mode': {
            'type': 'str', 'default': 'auto',
            'enum': ['auto', 'h4', 'sim2'],
            'label': 'homography mode',
            'description': 'auto = try 4-anchor findHomography, fall back to '
                           '2-anchor similarity if <4 anchors are detected '
                           '(default). h4 = force 4-anchor (perspective, '
                           '8 DOF). sim2 = force 2-anchor similarity '
                           '(rotation+scale+translation, 4 DOF, no '
                           'perspective correction).',
        },
        'sim_tag_a_id': {
            'type': 'int', 'default': 20,
            'label': 'sim2 anchor A',
            'description': 'first of the 2 anchor tags used by sim2 mode '
                           '(or by auto when forced into sim2 fallback). '
                           'Must appear in the anchors dict.',
        },
        'sim_tag_b_id': {
            'type': 'int', 'default': 22,
            'label': 'sim2 anchor B',
            'description': 'second of the 2 anchor tags used by sim2 mode.',
        },
    }

    def __init__(self, params=None):
        super().__init__(params)
        self._rect = None

    def start(self):
        if not _CV2_OK:
            self._last_error = 'cv2 unavailable'
            return
        cfg = self._params.get('anchors', _DEFAULT_ANCHORS)
        try:
            self._rect = TableRectifier(CornerConfig.from_dict(cfg))
        except Exception as e:
            self._last_error = f'rectifier init: {e}'
            return
        intr_path = self._params.get('intrinsics_path', _DEFAULT_INTRINSICS)
        if os.path.exists(intr_path):
            try:
                intr = CameraIntrinsics.load(intr_path)
                self._rect.set_intrinsics(intr)
            except Exception as e:
                self._last_error = f'intrinsics load: {e}'

    # --- world-frame helpers -----------------------------------------

    @staticmethod
    def _flips_for_origin(origin_corner: str):
        """Given which BEV corner the user wants as world (0,0), return
        the (flip_x, flip_y) that downstream nodes should apply to map
        TwinVision-frame mm into world-frame mm.

            top_left      → (False, False) — TwinVision native
            top_right     → (True,  False)
            bottom_left   → (False, True )
            bottom_right  → (True,  True ) — origin at TwinVision (W, H)
        """
        fx = origin_corner in ('top_right', 'bottom_right')
        fy = origin_corner in ('bottom_left', 'bottom_right')
        return fx, fy

    # --- per-tick ----------------------------------------------------

    def process(self, inputs):
        frame = inputs.get('frame')
        det = inputs.get('detection')
        if frame is None or det is None or getattr(self, '_rect', None) is None:
            return {}
        mode = str(self._params.get('homography_mode', 'auto'))
        sim_a = int(self._params.get('sim_tag_a_id', 20))
        sim_b = int(self._params.get('sim_tag_b_id', 22))

        def _do_sim2():
            self._rect.update_2pt_similarity(det, sim_a, sim_b)

        def _do_h4():
            if self._rect.intrinsics is not None:
                self._rect.update_pose(det)
            else:
                self._rect.update(det)

        if mode == 'sim2':
            _do_sim2()
        elif mode == 'h4':
            _do_h4()
        else:
            # 'auto' — count live anchors detected this tick. >= 4 → 4-anchor
            # (perspective, more accurate when all are visible). 2-3 → fall
            # back to similarity from sim_tag_a / sim_tag_b. 0-1 → try h4
            # anyway so the rectifier can lean on its cache.
            try:
                anchor_ids = [a.tag_id for a in self._rect.config.anchors()]
            except Exception:
                anchor_ids = []
            live = sum(1 for tid in anchor_ids
                       if det.get_center_for_id(tid) is not None)
            if live >= 4:
                _do_h4()
            elif live >= 2 and (det.get_center_for_id(sim_a) is not None
                                or det.get_center_for_id(sim_b) is not None):
                _do_sim2()
            else:
                _do_h4()   # cache-driven fallback inside update()

        out = {}
        out['homography_state'] = {
            'has_h':  self._rect.has_homography,
            'fresh':  self._rect.homography_is_fresh,
            'cached': self._rect.homography_is_cached,
        }

        # ── BEV rectify (image stays in BEV-native orientation) ─────
        if self._rect.has_homography:
            bev = self._rect.rectify(frame)
            if bev is not None:
                out['frame'] = bev
                if self._params.get('draw_grid', False):
                    bev_with_grid = self._rect.draw_grid(bev.copy())
                    out['preview'] = bev_with_grid
                else:
                    out['preview'] = bev

        # ── Coord state output ──────────────────────────────────────
        origin = str(self._params.get('world_origin_corner', 'top_left'))
        flip_x, flip_y = self._flips_for_origin(origin)
        out['coord_state'] = {
            'flip_x':        bool(flip_x),
            'flip_y':        bool(flip_y),
            'flip_theta':    bool(self._params.get('world_flip_theta', True)),
            'table_w_mm':    float(self._params.get('world_table_w_mm', 3000.0)),
            'table_h_mm':    float(self._params.get('world_table_h_mm', 2000.0)),
            'origin_corner': origin,
        }
        return out

    def get_state(self):
        r = getattr(self, '_rect', None)
        origin = str(self._params.get('world_origin_corner', 'top_left'))
        fx, fy = self._flips_for_origin(origin)
        return {
            'has_homography': bool(r and r.has_homography),
            'has_intrinsics': bool(r and r.has_intrinsics),
            'origin_corner':  origin,
            'flip_x':         fx,
            'flip_y':         fy,
            'last_error':     self._last_error,
        }
