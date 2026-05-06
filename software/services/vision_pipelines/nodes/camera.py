"""
camera nodes — produce a camera-position estimate (cam_xyz JSON) that
downstream nodes (parallax correction, localization) can consume.

Two flavors:

  camera.manual   — user-typed (x, y, z). No inputs. Useful for known-good
                    rigs and as a debug baseline.

  camera.auto     — runs cv2.solvePnP on the 4 ArUco anchor centers
                    detected this frame. Reuses the same estimate
                    TwinVision's RobotTracker computes internally. Reports
                    its source ('K' = used loaded intrinsics, 'fx_estimate'
                    = guessed fx from image size, 'fail' = solvePnP didn't
                    converge).

Both emit a JSON payload of shape:
    {
        'x_mm':   float,
        'y_mm':   float,
        'z_mm':   float,
        'source': 'manual' | 'K' | 'fx_estimate' | 'fail',
    }
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
    from core.charuco_calib import CameraIntrinsics
    from core.robot_tracker import estimate_camera_position_pnp
    _CV2_OK = True
except Exception:
    _CV2_OK = False

from ..pipeline import NodeIO, Port, PortKind
from .base import Node, register_node

_DEFAULT_INTRINSICS = os.path.abspath(
    os.path.join(_HERE, '..', '..', '..', 'vision', 'calibrations',
                 'camera_intrinsics.json')
)
_DEFAULT_ANCHORS = {
    'top_left':     {'tag_id': 23, 'x_mm': 600,  'y_mm': 600},
    'top_right':    {'tag_id': 22, 'x_mm': 2400, 'y_mm': 600},
    'bottom_right': {'tag_id': 20, 'x_mm': 2400, 'y_mm': 1400},
    'bottom_left':  {'tag_id': 21, 'x_mm': 600,  'y_mm': 1400},
}


@register_node('camera.manual')
class CameraManualNode(Node):
    """Hand-set camera position. No inputs — emits the configured xyz
    every tick so downstream nodes always have a value to read."""
    IO = NodeIO(
        inputs=[],
        outputs=[Port('cam_xyz', PortKind.JSON,
                      'camera position {x_mm, y_mm, z_mm, source}')],
    )
    params_schema = {
        'cam_x_mm': {
            'type': 'float', 'default': 1500.0,
            'label': 'camera X', 'unit': 'mm',
            'description': 'horizontal position of the camera (in coord_frame)',
        },
        'cam_y_mm': {
            'type': 'float', 'default': -200.0,
            'label': 'camera Y', 'unit': 'mm',
            'description': 'vertical position of the camera (in coord_frame)',
        },
        'cam_z_mm': {
            'type': 'float', 'default': 1600.0,
            'label': 'camera Z (height)', 'unit': 'mm',
            'description': 'height of the camera above the floor',
        },
        'coord_frame': {
            'type': 'str', 'default': 'world',
            'enum': ['world', 'twin_vision'],
            'label': 'frame of cam_x/y_mm',
            'description': 'world = the xy you entered are already in your '
                           'world frame (most common — what you naturally '
                           'measure on the table). twin_vision = aligned '
                           'with anchor mm grid (TwinVision native, origin '
                           'top-left). Downstream nodes respect this tag.',
        },
    }

    def process(self, inputs):
        # The 'frame' tag tells downstream consumers (parallax,
        # localization) which mm convention these xy values are in.
        # They'll flip if needed to match their internal frame.
        return {'cam_xyz': {
            'x_mm':   float(self._params.get('cam_x_mm',  1500.0)),
            'y_mm':   float(self._params.get('cam_y_mm',  -200.0)),
            'z_mm':   float(self._params.get('cam_z_mm',  1600.0)),
            'source': 'manual',
            'frame':  str(self._params.get('coord_frame', 'world')),
        }}

    def get_state(self):
        return {
            'cam_xyz': [
                float(self._params.get('cam_x_mm', 1500.0)),
                float(self._params.get('cam_y_mm', -200.0)),
                float(self._params.get('cam_z_mm', 1600.0)),
            ],
            'last_error': self._last_error,
        }


@register_node('camera.auto')
class CameraAutoNode(Node):
    """Estimate camera position from the 4 ArUco anchor centers via
    solvePnP. Falls back to a `fx_estimate` heuristic if no intrinsics
    file is available — accurate enough to be useful but use camera.manual
    for production setups."""
    IO = NodeIO(
        inputs=[
            Port('detection', PortKind.DETECTION),
            Port('frame',     PortKind.FRAME, optional=True,
                 description='only used to read image size; an explicit '
                             'image_size param can replace it'),
        ],
        outputs=[Port('cam_xyz', PortKind.JSON,
                      'camera position {x_mm, y_mm, z_mm, source}')],
    )
    params_schema = {
        'intrinsics_path': {
            'type': 'str', 'default': _DEFAULT_INTRINSICS,
            'label': 'intrinsics file',
            'description': 'camera_intrinsics.json (improves estimate accuracy)',
        },
        'anchors': {
            'type': 'dict', 'default': _DEFAULT_ANCHORS,
            'label': 'anchor tag IDs + table positions',
        },
        'image_w': {
            'type': 'int', 'default': 1920, 'label': 'image width',
            'unit': 'px', 'min': 320,
            'description': 'used as a fallback when no frame is wired',
        },
        'image_h': {
            'type': 'int', 'default': 1080, 'label': 'image height',
            'unit': 'px', 'min': 240,
        },
    }

    def __init__(self, params=None):
        super().__init__(params)
        self._intr = None

    def start(self):
        if not _CV2_OK:
            self._last_error = 'cv2 unavailable'
            return
        path = self._params.get('intrinsics_path', _DEFAULT_INTRINSICS)
        if path and os.path.exists(path):
            try:
                self._intr = CameraIntrinsics.load(path)
            except Exception as e:
                self._last_error = f'intrinsics load: {e}'

    def process(self, inputs):
        det = inputs.get('detection')
        if det is None:
            return {}
        anchors_cfg = self._params.get('anchors', _DEFAULT_ANCHORS) or _DEFAULT_ANCHORS
        # Build the (pixel, table-mm) pairs for the 4 anchors. If any
        # anchor isn't visible this frame, we can't solve.
        pix, mm = [], []
        for corner in ('top_left', 'top_right', 'bottom_right', 'bottom_left'):
            entry = anchors_cfg.get(corner) if isinstance(anchors_cfg, dict) else None
            if not entry:
                return {}
            tid = int(entry['tag_id'])
            c = det.get_center_for_id(tid)
            if c is None:
                return {}   # anchor occluded — no estimate this frame
            pix.append([float(c[0]), float(c[1])])
            mm.append([float(entry['x_mm']), float(entry['y_mm'])])

        # Image size: prefer the wired frame, fall back to params.
        frame = inputs.get('frame')
        if frame is not None and hasattr(frame, 'shape'):
            h, w = frame.shape[:2]
        else:
            w = int(self._params.get('image_w', 1920))
            h = int(self._params.get('image_h', 1080))

        K = self._intr.K if self._intr is not None else None
        D = self._intr.dist if self._intr is not None else None
        try:
            cam_xyz, fx_used, source = estimate_camera_position_pnp(
                np.asarray(pix), np.asarray(mm),
                image_size=(w, h), K=K, dist=D,
                fx_estimate=None,
            )
        except Exception as e:
            self._last_error = f'solvePnP: {type(e).__name__}: {e}'
            return {}

        if cam_xyz is None or source == 'fail':
            return {}
        return {'cam_xyz': {
            'x_mm':   float(cam_xyz[0]),
            'y_mm':   float(cam_xyz[1]),
            'z_mm':   float(cam_xyz[2]),
            'source': source,
            # solvePnP runs in the anchor mm frame = TwinVision native.
            'frame':  'twin_vision',
        }}

    def get_state(self):
        return {
            'intrinsics_loaded': self._intr is not None,
            'last_error':        self._last_error,
        }
