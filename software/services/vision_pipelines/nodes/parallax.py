"""
parallax node — applies a configurable parallax correction to a pose_list.

Use case: a tag mounted on a tall robot (z > 0) projects to a "naive"
ground-plane position that's biased away from the camera. This node maps
that naive xy back to where the tag's actual ground footprint is, given
the camera position and the tag height.

The math is the same as TwinVision's `correct_parallax` but exposed as a
standalone block so the pipeline can plug it BEFORE or AFTER a tracker /
filter — e.g. raw aruco → parallax → smoother → output.

    real_xy = camera_xy + factor * (naive_xy - camera_xy)
    factor  = (Zc - z_object) / Zc

Inputs
------
    pose_list (pose_list) : list of {tag_id, x_mm, y_mm, …}
                            Reads x_mm / y_mm (or naive_x_mm / naive_y_mm
                            if present) and writes back to x_mm / y_mm.

Outputs
-------
    pose_list (pose_list) : same shape, with corrected x_mm / y_mm.

Params
------
    cam_x_mm, cam_y_mm, cam_z_mm : camera position above the table (mm).
                                   z is height above the floor; set
                                   reasonably (typically 1500-2000 mm).
    object_z_mm                  : tag height above the floor (mm).
    use_naive_xy                 : if True, read naive_x_mm/naive_y_mm
                                   from the input (the localization node
                                   exposes those alongside the corrected
                                   pos_mm). Useful when the upstream
                                   tracker already did its own correction
                                   and you want to redo it with different
                                   parameters.
    write_back                   : 'overwrite' (default — replace x/y) or
                                   'add' (write corrected_x_mm /
                                   corrected_y_mm without touching x/y).
"""

from __future__ import annotations

import math

try:
    import cv2
    _CV2_OK = True
except Exception:
    _CV2_OK = False

from ..pipeline import NodeIO, Port, PortKind
from .base import Node, register_node
from ._coords_helpers import (
    coord_state_or_none, flip_pose_list, flip_cam_xyz,
    world_mm_to_bev_pixel,
)


@register_node('parallax')
class ParallaxCorrectionNode(Node):
    IO = NodeIO(
        inputs=[
            Port('pose_list', PortKind.POSE_LIST),
            Port('cam_xyz',   PortKind.JSON, optional=True,
                 description='wire camera.auto / camera.manual to override '
                             'the cam_x/y/z params'),
            Port('frame',     PortKind.FRAME, optional=True,
                 description='optional — wire a BEV frame to get a preview '
                             'output showing the corrected dots only'),
            Port('coord_state', PortKind.JSON, optional=True,
                 description='wire from rectify.coord_state — when wired, '
                             'cam_x/y_mm params are interpreted in the '
                             'world frame and the output pose_list stays '
                             'in world frame too'),
        ],
        outputs=[
            Port('pose_list', PortKind.POSE_LIST,
                 'pose_list with corrected x/y'),
            Port('preview',   PortKind.PREVIEW,
                 'BEV with only the parallax-corrected dots — '
                 'wire frame input to enable'),
        ],
    )
    params_schema = {
        'cam_x_mm':     {'type': 'float', 'default': 1500.0,
                         'help': 'camera X over the table (mm)'},
        'cam_y_mm':     {'type': 'float', 'default': -200.0,
                         'help': 'camera Y over the table (mm)'},
        'cam_z_mm':     {'type': 'float', 'default': 1600.0,
                         'help': 'camera height above the table (mm)'},
        'object_z_mm':  {'type': 'float', 'default': 490.0,
                         'help': 'tag height above the table (mm)'},
        'use_naive_xy': {'type': 'bool',  'default': True,
                         'label': 'use naive xy if available',
                         'description': 'when input pose_list has '
                                        'naive_x_mm/naive_y_mm fields '
                                        '(localization sets them), apply '
                                        'parallax to the NAIVE values. '
                                        'Otherwise localization\'s already-'
                                        'corrected x/y get re-corrected '
                                        '(double-correction). Default ON.'},
        'write_back':   {'type': 'str',   'default': 'overwrite',
                         'enum': ['overwrite', 'add']},
        'preview_bev_scale':     {'type': 'float', 'default': 0.25,
                                  'min': 0.05, 'max': 1.0, 'step': 0.05,
                                  'label': 'preview BEV scale', 'unit': 'px/mm',
                                  'description': 'must match the rectify '
                                                 'node (preview only)'},
        'preview_bev_margin_mm': {'type': 'float', 'default': 100.0,
                                  'min': 0, 'max': 500, 'step': 10,
                                  'label': 'preview BEV margin', 'unit': 'mm',
                                  'description': 'must match the rectify '
                                                 'node (preview only)'},
        'preview_dot_radius_px': {'type': 'int',   'default': 8,
                                  'min': 2, 'max': 30,
                                  'label': 'preview dot radius', 'unit': 'px'},
        'preview_draw_camera':   {'type': 'bool',  'default': True,
                                  'label': 'preview: draw camera',
                                  'description': 'plot the camera ground '
                                                 'projection (Xc, Yc) as a '
                                                 'magenta cross. Helps '
                                                 'debug parallax direction.'},
        'preview_draw_grid':     {'type': 'bool',  'default': False,
                                  'label': 'preview: draw mm grid',
                                  'description': 'overlay a 200 mm grid on '
                                                 'the BEV preview'},
    }

    def process(self, inputs):
        poses = inputs.get('pose_list')
        # Distinguish "no upstream" (None — propagate nothing) from "empty
        # list" (filter chain was active but no tag survived this frame —
        # propagate the empty list so downstream filters/overlays can react,
        # e.g. overlay.restrict_aruco_to_pose_list will hide every marker).
        if poses is None:
            return {}
        if len(poses) == 0:
            return {'pose_list': []}

        # Precedence: wired cam_xyz input > params. The wired path is the
        # debug-friendly one — a camera.auto / camera.manual node up
        # stream sets the truth, and changing it doesn't require editing
        # this node's params.
        #
        # Frame handling: pose_list comes in WORLD frame when upstream
        # localization has coord_state wired. cam_xyz must match. Camera
        # nodes label their dict with 'frame': 'twin_vision' (anchor-
        # aligned) or 'world' (already flipped by localization). When
        # the incoming cam_xyz is in twin_vision and we have cs, flip it.
        cs = coord_state_or_none(inputs.get('coord_state'))
        cam = inputs.get('cam_xyz') if isinstance(inputs.get('cam_xyz'), dict) else None
        if cam is not None and cs is not None:
            src_frame = str(cam.get('frame', 'twin_vision'))
            if src_frame != 'world':
                cam = flip_cam_xyz(cam, cs)
        if cam is not None:
            cx = float(cam.get('x_mm', self._params.get('cam_x_mm', 1500.0)))
            cy = float(cam.get('y_mm', self._params.get('cam_y_mm', -200.0)))
            cz = float(cam.get('z_mm', self._params.get('cam_z_mm', 1600.0)))
        else:
            cx = float(self._params.get('cam_x_mm',    1500.0))
            cy = float(self._params.get('cam_y_mm',    -200.0))
            cz = float(self._params.get('cam_z_mm',    1600.0))
            # Params are entered in TwinVision frame (anchor-aligned).
            # Mirror to world frame so cam-vs-pose math stays consistent.
            if cs is not None and (cs.get('flip_x') or cs.get('flip_y')):
                from ._coords_helpers import flip_mm
                cx, cy = flip_mm(cx, cy, cs)
        oz = float(self._params.get('object_z_mm', 490.0))
        use_naive  = bool(self._params.get('use_naive_xy', False))
        write_back = str(self._params.get('write_back', 'overwrite'))

        # factor = (Zc - z_object) / Zc — degenerates as Zc → z_object.
        if cz <= oz + 1e-3:
            self._last_error = (
                f'cam_z_mm ({cz:.0f}) must be > object_z_mm ({oz:.0f})'
            )
            # Pass through without correcting so downstream still gets data.
            return {'pose_list': poses}
        factor = (cz - oz) / cz

        out = []
        for q in poses:
            if not isinstance(q, dict):
                continue
            # Pick the source xy
            if use_naive and q.get('naive_x_mm') is not None \
                    and q.get('naive_y_mm') is not None:
                nx = q['naive_x_mm']; ny = q['naive_y_mm']
            else:
                nx = q.get('x_mm'); ny = q.get('y_mm')
            # Skip if either is None / NaN
            try:
                nxf = float(nx); nyf = float(ny)
                if math.isnan(nxf) or math.isnan(nyf):
                    out.append(dict(q))
                    continue
            except (TypeError, ValueError):
                out.append(dict(q))
                continue

            corr_x = cx + factor * (nxf - cx)
            corr_y = cy + factor * (nyf - cy)

            new_q = dict(q)   # don't mutate upstream's dict
            if write_back == 'add':
                new_q['corrected_x_mm'] = corr_x
                new_q['corrected_y_mm'] = corr_y
            else:   # overwrite
                # Preserve the un-corrected values under naive_* if not
                # already present, so downstream nodes can compare.
                new_q.setdefault('naive_x_mm', nxf)
                new_q.setdefault('naive_y_mm', nyf)
                new_q['x_mm'] = corr_x
                new_q['y_mm'] = corr_y
            out.append(new_q)

        # Reset error after a successful tick so the dashboard can clear it
        self._last_error = None

        result = {'pose_list': out}

        # Optional preview: draws BOTH the naive (yellow, on the BEV marker)
        # and the parallax-corrected (green, the true ground point) dots,
        # connected by a thin line. The line length is the magnitude of
        # the parallax correction — visual sanity-check that cam_xyz +
        # object_z_mm are sensible. If the line is huge or points the
        # wrong way, your camera position params are off.
        frame = inputs.get('frame')
        if frame is not None and _CV2_OK:
            try:
                vis = frame.copy()
                scale  = float(self._params.get('preview_bev_scale', 0.25))
                margin = float(self._params.get('preview_bev_margin_mm', 100.0))
                radius = int(self._params.get('preview_dot_radius_px', 8))
                # `cs` was parsed once at the top of process()
                # Project in BEV-native frame (used for grid lines that
                # span the BEV image directly).
                def _project_bev(x_mm, y_mm):
                    try:
                        return (int(round((float(x_mm) + margin) * scale)),
                                int(round((float(y_mm) + margin) * scale)))
                    except (TypeError, ValueError):
                        return None
                # Project from world frame (used for poses + cam marker —
                # they live in user-world frame when cs is wired).
                def _project(x_mm, y_mm):
                    return world_mm_to_bev_pixel(x_mm, y_mm, cs,
                                                 margin, scale)

                # Optional grid — drawn in BEV-native frame.
                if self._params.get('preview_draw_grid', False):
                    grid_color = (0, 200, 80)
                    h_px, w_px = vis.shape[:2]
                    x_min_mm = -margin
                    y_min_mm = -margin
                    x_max_mm = w_px / scale - margin
                    y_max_mm = h_px / scale - margin
                    step = 200
                    x_start = int(x_min_mm // step) * step
                    y_start = int(y_min_mm // step) * step
                    for xm in range(x_start, int(x_max_mm) + 1, step):
                        p0 = _project_bev(xm, y_min_mm)
                        p1 = _project_bev(xm, y_max_mm)
                        if p0 and p1:
                            cv2.line(vis, p0, p1, grid_color, 1, cv2.LINE_AA)
                    for ym in range(y_start, int(y_max_mm) + 1, step):
                        p0 = _project_bev(x_min_mm, ym)
                        p1 = _project_bev(x_max_mm, ym)
                        if p0 and p1:
                            cv2.line(vis, p0, p1, grid_color, 1, cv2.LINE_AA)

                # Camera ground projection — always shown as a text badge so
                # the user can sanity-check WHICH cam_xyz parallax is using.
                # Source tag tells whether the value came from a wired input
                # or from this node's params.
                cam_src = ('input' if cam is not None else 'params')
                badge = (f'cam=({int(cx)},{int(cy)},{int(cz)})  '
                         f'src={cam_src}  '
                         f'factor={factor:.3f}')
                cv2.rectangle(vis, (0, 0), (260, 22), (0, 0, 0), -1)
                cv2.putText(vis, badge, (4, 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                            (255, 255, 255), 1, cv2.LINE_AA)
                if self._params.get('preview_draw_camera', True):
                    h_px, w_px = vis.shape[:2]
                    cam_pt = _project(cx, cy)
                    if cam_pt is not None:
                        # Clamp to image edge so the cross is visible even
                        # when the camera is off-BEV (common when cam_x/y is
                        # wrong — that's exactly when the user needs to see
                        # which way it's pointing).
                        in_view = (0 <= cam_pt[0] < w_px and 0 <= cam_pt[1] < h_px)
                        x = max(2, min(w_px - 2, cam_pt[0]))
                        y = max(2, min(h_px - 2, cam_pt[1]))
                        col = (255, 0, 255) if in_view else (0, 0, 255)
                        cv2.drawMarker(vis, (x, y), col,
                                       cv2.MARKER_CROSS, radius * 3, 2)
                        if not in_view:
                            cv2.putText(vis, '⚠ cam off-BEV',
                                        (x + 8, y - 8),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                                        (0, 0, 255), 1, cv2.LINE_AA)

                for q in out:
                    # Corrected (green) — what parallax says is the truth
                    cx_mm, cy_mm = q.get('x_mm'), q.get('y_mm')
                    # Naive (yellow) — where the tag visually sits on BEV
                    nx_mm = q.get('naive_x_mm', cx_mm)
                    ny_mm = q.get('naive_y_mm', cy_mm)
                    p_corr  = _project(cx_mm, cy_mm) if cx_mm is not None else None
                    p_naive = _project(nx_mm, ny_mm) if nx_mm is not None else None
                    if p_naive is not None and p_corr is not None \
                            and p_naive != p_corr:
                        cv2.line(vis, p_naive, p_corr, (255, 255, 255),
                                 1, cv2.LINE_AA)
                    if p_naive is not None:
                        cv2.circle(vis, p_naive, radius, (0, 220, 220), -1)
                        cv2.circle(vis, p_naive, radius, (255, 255, 255), 1)
                    if p_corr is not None:
                        cv2.circle(vis, p_corr, radius, (0, 220, 0), -1)
                        cv2.circle(vis, p_corr, radius, (255, 255, 255), 1)
                        tid = q.get('tag_id')
                        if tid is not None:
                            cv2.putText(vis, f'#{tid}',
                                        (p_corr[0] + radius + 2, p_corr[1] + 4),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                                        (0, 220, 0), 1, cv2.LINE_AA)
                result['preview'] = vis
            except Exception as e:
                self._last_error = f'preview: {e}'

        return result

    def get_state(self):
        return {
            'cam_xyz':     [
                float(self._params.get('cam_x_mm', 1500.0)),
                float(self._params.get('cam_y_mm', -200.0)),
                float(self._params.get('cam_z_mm', 1600.0)),
            ],
            'object_z_mm': float(self._params.get('object_z_mm', 490.0)),
            'last_error':  self._last_error,
        }
