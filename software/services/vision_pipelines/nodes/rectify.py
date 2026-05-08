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
import time
from typing import Optional

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
            Port('rectifier',        PortKind.JSON,
                 'shared TableRectifier instance — wire to localization '
                 'so it reuses the same H without re-running the homography '
                 '(keeps the two nodes in sync, especially in sim2 mode)'),
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
            'label': 'sim2 anchor A (legacy)',
            'description': 'used only when sim_tag_ids is empty. Prefer '
                           'sim_tag_ids for new configs.',
        },
        'sim_tag_b_id': {
            'type': 'int', 'default': 22,
            'label': 'sim2 anchor B (legacy)',
            'description': 'same back-compat note as sim_tag_a_id.',
        },
        'sim_tag_ids': {
            'type': 'json', 'default': [],
            'label': 'sim2 anchor ids',
            'description': 'list of anchor tag ids whose 4 corners feed '
                           'the corner-based homography. ≥ 2 visible tags '
                           'are required per tick. Use 3+ non-collinear '
                           'tags (e.g. add a 5th tag at the table center) '
                           'to avoid the 2-tag colinearity trap. Empty = '
                           'fall back to [sim_tag_a_id, sim_tag_b_id].',
        },
        'tag_size_mm': {
            'type': 'float', 'default': 100.0,
            'label': 'ArUco tag edge length',
            'unit': 'mm',
            'description': 'when > 0 (and 2-tag mode is active), use the 4 '
                           'corners of each tag (8 points total) to fit a '
                           'full perspective homography instead of a 4-DOF '
                           'similarity. Recovers yaw + perspective. Set to 0 '
                           'to fall back to the centers-only similarity. '
                           'Tags MUST be placed aligned with the table axes.',
        },
    }

    def __init__(self, params=None):
        super().__init__(params)
        self._rect = None
        self._mode_active = None             # which path actually fired this tick
        self._live_anchor_count = 0          # how many anchors detected total
        self._detected_anchor_ids = []       # ids actually seen this tick
        self._configured_sim_ids = []        # sim_tag_ids resolved
        self._detected_sim_ids = []          # subset of sim_tag_ids seen this tick

        # ── Capture state ─────────────────────────────────────────────
        # The node sits IDLE at boot — H is not computed and the BEV
        # frame is not emitted. The recalage routine sends a one-shot
        # `request_capture()` (via T:vis homography_capture); the next
        # tick fits H and on success freezes both H and the BEV image.
        # While LOCKED, every tick re-emits the snapshot — no work
        # other than reading a cached buffer.
        self._capture_pending: bool = False
        self._capture_result: Optional[dict] = None
        self._locked_bev: Optional['np.ndarray'] = None
        self._locked_raw: Optional['np.ndarray'] = None
        self._locked_at_t: Optional[float] = None

    # ── Capture state machine API ─────────────────────────────────────
    # Called from run.py when a T:vis homography_capture / homography_release
    # arrives from the firmware. The actual fit happens on the worker
    # thread inside process(); these methods only flip the request flag
    # so the worker picks it up without us reaching into cv2 from another
    # thread.

    def request_capture(self):
        """Arm a one-shot homography capture. The next tick will fit H
        and (on success) lock it + snapshot the BEV image. Idempotent —
        a second request after lock re-captures from a fresh frame."""
        if self._rect is not None:
            self._rect.unlock()
            self._rect.arm()
        self._locked_bev = None
        self._locked_raw = None
        self._locked_at_t = None
        self._capture_pending = True
        self._capture_result = None

    def release_capture(self):
        """Return to IDLE. H + BEV snapshot dropped. Update_* stops."""
        if self._rect is not None:
            self._rect.unlock()
            self._rect.disarm()
        self._locked_bev = None
        self._locked_raw = None
        self._locked_at_t = None
        self._capture_pending = False
        self._capture_result = None

    def get_capture_result(self) -> 'Optional[dict]':
        """Latest result for an in-flight request_capture(). None while
        the worker hasn't ticked yet, then a dict {'ok': bool, ...}.
        Caller polls until non-None or timeout."""
        return self._capture_result

    def get_locked_raw(self):
        """The undistorted source frame captured at lock time. Returned
        by the API endpoint that powers the homography-debug tile."""
        return self._locked_raw

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

    def _build_coord_state(self) -> dict:
        origin = str(self._params.get('world_origin_corner', 'top_left'))
        flip_x, flip_y = self._flips_for_origin(origin)
        return {
            'flip_x':        bool(flip_x),
            'flip_y':        bool(flip_y),
            'flip_theta':    bool(self._params.get('world_flip_theta', True)),
            'table_w_mm':    float(self._params.get('world_table_w_mm', 3000.0)),
            'table_h_mm':    float(self._params.get('world_table_h_mm', 2000.0)),
            'origin_corner': origin,
        }

    def process(self, inputs):
        frame = inputs.get('frame')
        det = inputs.get('detection')
        if frame is None or det is None or getattr(self, '_rect', None) is None:
            return {}

        rect = self._rect
        coord_state = self._build_coord_state()

        # ── State 3 — LOCKED: H + BEV snapshot frozen ─────────────────
        # Re-emit the captured image every tick. Downstream nodes treat
        # the frame as static; live aruco detections still flow through
        # the `detection` channel so localization keeps tracking robots.
        if rect.is_locked:
            return {
                'homography_state': {
                    'has_h':  True,
                    'fresh':  False,
                    'cached': True,
                    'locked': True,
                },
                'rectifier':    rect,
                'coord_state':  coord_state,
                'frame':        self._locked_bev,
                'preview':      self._locked_bev,
            }

        # ── State 1 — IDLE: not armed, no fit attempt at all ──────────
        # The recalage routine arms us via request_capture(). Until
        # then we emit nothing image-side, so downstream nodes
        # (localization, parallax, grid…) skip processing. Saves CPU
        # and visually signals "no homography yet" on the debug page.
        if not self._capture_pending:
            self._mode_active = None
            self._live_anchor_count = 0
            self._detected_anchor_ids = []
            self._detected_sim_ids = []
            return {
                'homography_state': {
                    'has_h':  False,
                    'fresh':  False,
                    'cached': False,
                    'locked': False,
                    'idle':   True,
                },
                'rectifier':   rect,
                'coord_state': coord_state,
            }

        # ── State 2 — CAPTURING: armed, try to fit H this tick ────────
        # Falls through to the legacy fit logic below. On success we
        # snapshot the BEV + lock; on failure we leave _capture_pending
        # set so the next tick retries until run.py's polling timeout.

        mode = str(self._params.get('homography_mode', 'auto'))
        sim_a = int(self._params.get('sim_tag_a_id', 20))
        sim_b = int(self._params.get('sim_tag_b_id', 22))
        # Resolve the effective list of anchor ids the corner-based fit
        # should use. Prefer the new `sim_tag_ids` list; fall back to the
        # legacy 2-id pair for back-compat.
        sim_ids_param = self._params.get('sim_tag_ids', [])
        if isinstance(sim_ids_param, str):
            try:
                sim_ids = [int(x.strip()) for x in sim_ids_param.split(',') if x.strip()]
            except ValueError:
                sim_ids = []
        else:
            try:
                sim_ids = [int(x) for x in (sim_ids_param or [])]
            except (TypeError, ValueError):
                sim_ids = []
        if not sim_ids:
            sim_ids = [sim_a, sim_b]

        def _do_sim2():
            # When intrinsics are loaded, solvePnP on the N centers is the
            # mathematically clean route (no yaw_deg, no tag_size_mm, lens
            # undistortion handled by the priority-1 rectify path). Falls
            # back to the 8-/12-corner findHomography or the 2-pt similarity
            # if PnP isn't available.
            r = self._rect
            if r.intrinsics is not None and len(sim_ids) >= 3:
                ok = r.update_centers_pose(det, sim_ids)
                if ok:
                    self._mode_active = 'centers_pnp'
                    return
                # PnP failed — clear stale _pose_H so the H-based fallbacks
                # below can take precedence in rectify().
                try:
                    r._pose_H = None
                    r._pose_rvec = None
                    r._pose_tvec = None
                except Exception:
                    pass
            else:
                # Not eligible for PnP — make sure _H wins in rectify().
                try:
                    r._pose_H = None
                    r._pose_rvec = None
                    r._pose_tvec = None
                except Exception:
                    pass
            tag_size = float(self._params.get('tag_size_mm', 100.0))
            if tag_size > 0:
                ok = r.update_corners_homography(det, sim_ids, tag_size)
                self._mode_active = 'corners_h' if ok else 'corners_h_fail'
                if not ok and len(sim_ids) >= 2:
                    r.update_2pt_similarity(det, sim_ids[0], sim_ids[1])
                    self._mode_active = 'sim2'
            else:
                if len(sim_ids) >= 2:
                    r.update_2pt_similarity(det, sim_ids[0], sim_ids[1])
                self._mode_active = 'sim2'

        def _do_h4():
            if self._rect.intrinsics is not None:
                self._rect.update_pose(det)
                self._mode_active = 'h4_pose'
            else:
                self._rect.update(det)
                self._mode_active = 'h4_findH'

        try:
            anchor_ids = [a.tag_id for a in self._rect.config.anchors()]
        except Exception:
            anchor_ids = []
        detected_anchor_ids = [tid for tid in anchor_ids
                               if det.get_center_for_id(tid) is not None]
        live = len(detected_anchor_ids)
        self._live_anchor_count = live
        self._detected_anchor_ids = list(detected_anchor_ids)

        # Which of the user-listed sim2 anchors are actually detected.
        detected_sim_ids = [int(tid) for tid in sim_ids
                            if det.get_center_for_id(int(tid)) is not None]
        self._detected_sim_ids = detected_sim_ids
        self._configured_sim_ids = list(sim_ids)
        live_sim_ids = len(detected_sim_ids)

        if mode == 'sim2':
            _do_sim2()
        elif mode == 'h4':
            _do_h4()
        else:
            # 'auto' — count live anchors detected this tick. >= 4 → 4-anchor
            # (perspective, more accurate when all are visible). Else if the
            # user-listed sim2 ids give us ≥ 2 detections, use the corner
            # homography. Otherwise try h4 cached.
            if live >= 4:
                _do_h4()
            elif live_sim_ids >= 2:
                _do_sim2()
            else:
                _do_h4()   # cache-driven fallback inside update()

        # ── Capture handshake ────────────────────────────────────────
        # has_homography is True iff a fit succeeded this tick (or in a
        # previous CAPTURING tick — but we cleared the cached H at
        # request_capture time). Lock + snapshot now, return frozen.
        # Re-check _capture_pending: release_capture() may have flipped
        # it False on a timeout path while we were still inside the fit.
        if rect.has_homography and self._capture_pending:
            bev = rect.rectify(frame)
            if bev is not None:
                self._locked_bev = bev.copy()
                # Save the live source frame too so the homography-debug
                # tile can show "this is the picture vision used".
                try:
                    self._locked_raw = frame.copy()
                except Exception:
                    self._locked_raw = None
            rect.lock()
            self._locked_at_t = time.monotonic()
            self._capture_pending = False
            self._capture_result = {
                'ok':                  True,
                'mode':                self._mode_active,
                'live_anchor_count':   self._live_anchor_count,
                'detected_anchor_ids': list(self._detected_anchor_ids),
                'detected_sim_ids':    list(self._detected_sim_ids),
            }
            return {
                'homography_state': {
                    'has_h':  True,
                    'fresh':  True,
                    'cached': False,
                    'locked': True,
                },
                'rectifier':   rect,
                'coord_state': coord_state,
                'frame':       self._locked_bev,
                'preview':     self._locked_bev,
            }

        # Fit failed this tick — keep _capture_pending=True so the
        # next tick retries. The polling thread in run.py will time
        # out after ~2s if anchors stay invisible and surface ok=0.
        return {
            'homography_state': {
                'has_h':  False,
                'fresh':  False,
                'cached': False,
                'locked': False,
                'armed':  True,
            },
            'rectifier':   rect,
            'coord_state': coord_state,
        }

    def get_state(self):
        r = getattr(self, '_rect', None)
        origin = str(self._params.get('world_origin_corner', 'top_left'))
        fx, fy = self._flips_for_origin(origin)
        # Surface a single human-readable phase string for the UI:
        #   'idle'      → no fit, no BEV emitted (pre-recalage default)
        #   'capturing' → recalage in progress, retrying H fit
        #   'locked'    → H + BEV frozen, no further work
        if r is None:
            phase = 'idle'
        elif r.is_locked:
            phase = 'locked'
        elif self._capture_pending:
            phase = 'capturing'
        else:
            phase = 'idle'
        return {
            'has_homography':    bool(r and r.has_homography),
            'has_intrinsics':    bool(r and r.has_intrinsics),
            'homography_locked': bool(r and r.is_locked),
            'capture_phase':     phase,
            'capture_pending':   bool(self._capture_pending),
            'capture_result':    dict(self._capture_result) if self._capture_result else None,
            'locked_age_s':      (time.monotonic() - self._locked_at_t)
                                  if self._locked_at_t is not None else None,
            # Which path actually ran on the latest tick. Useful to confirm
            # auto-mode is doing what you think: 'sim2' (2-anchor similarity),
            # 'h4_pose' (4-anchor + solvePnP), 'h4_findH' (4-anchor pure).
            'mode_param':           str(self._params.get('homography_mode', 'auto')),
            'mode_active':          self._mode_active,
            'live_anchor_count':    self._live_anchor_count,
            'detected_anchor_ids':  list(self._detected_anchor_ids),
            'configured_sim_ids':   list(self._configured_sim_ids),
            'detected_sim_ids':     list(self._detected_sim_ids),
            'origin_corner':     origin,
            'flip_x':            fx,
            'flip_y':            fy,
            'last_error':        self._last_error,
        }
