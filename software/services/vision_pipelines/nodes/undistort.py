"""
undistort node — applies a CameraIntrinsics-driven undistort remap to the
input frame. The intrinsics file path is configurable so different sources
can use different camera models.
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
    from core.charuco_calib import CameraIntrinsics
    _CV2_OK = True
except Exception:
    _CV2_OK = False

from ..pipeline import NodeIO, Port, PortKind
from .base import Node, register_node
from ..cuda_util import CUDA_AVAILABLE, gpu_remap, upload_to_gpu

# Default intrinsics — same as the legacy backend used.
_DEFAULT_INTRINSICS = os.path.abspath(
    os.path.join(_HERE, '..', '..', '..', 'vision', 'calibrations', 'camera_intrinsics.json')
)


@register_node('undistort')
class UndistortNode(Node):
    IO = NodeIO(
        inputs=[Port('frame', PortKind.FRAME)],
        outputs=[
            Port('frame',   PortKind.FRAME,   'undistorted frame (clean)'),
            Port('preview', PortKind.PREVIEW, 'same, for inline display'),
        ],
    )
    params_schema = {
        'intrinsics_path': {
            'type': 'str', 'default': _DEFAULT_INTRINSICS,
            'label': 'intrinsics file',
            'description': 'camera_intrinsics.json from TwinVision calibration',
        },
        'use_gpu': {
            'type': 'bool', 'default': True,
            'label': 'use GPU (CUDA)',
            'description': 'offload remap to cv2.cuda when available '
                           '(takes effect on Jetson with CUDA-built OpenCV; '
                           'silently uses CPU otherwise)',
        },
    }

    def __init__(self, params=None):
        super().__init__(params)
        # Pre-init so get_state() never AttributeErrors before start().
        self._intr   = None
        self._mx     = None
        self._my     = None
        self._k_new  = None
        self._mx_gpu = None
        self._my_gpu = None

    def start(self):
        if not _CV2_OK:
            self._last_error = 'cv2 unavailable'
            return
        path = self._params.get('intrinsics_path', _DEFAULT_INTRINSICS)
        if not os.path.exists(path):
            self._last_error = f'intrinsics not found: {path}'
            return
        try:
            self._intr = CameraIntrinsics.load(path)
            self._mx, self._my, self._k_new = self._intr.build_undistort_maps()
        except Exception as e:
            self._last_error = f'load failed: {e}'
            return
        # Pre-upload the remap maps to GPU once. They never change after
        # load, so per-frame upload is wasted bandwidth.
        if CUDA_AVAILABLE and self._params.get('use_gpu', True):
            self._mx_gpu = upload_to_gpu(self._mx)
            self._my_gpu = upload_to_gpu(self._my)

    def process(self, inputs):
        frame = inputs.get('frame')
        if frame is None:
            return {}
        if self._intr is None:
            # No intrinsics: pass-through.
            return {'frame': frame, 'preview': frame}
        if CUDA_AVAILABLE and self._params.get('use_gpu', True):
            out = gpu_remap(frame, self._mx, self._my,
                            self._mx_gpu, self._my_gpu)
        else:
            out = cv2.remap(
                frame, self._mx, self._my, cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0),
            )
        # No annotations on preview yet — same as frame. Could draw a
        # boundary or distortion-rms badge here in a future pass.
        return {'frame': out, 'preview': out}

    def get_state(self):
        return {
            'loaded':     self._intr is not None,
            'using_gpu':  bool(CUDA_AVAILABLE
                               and self._params.get('use_gpu', True)
                               and self._mx_gpu is not None),
            'cuda_available': CUDA_AVAILABLE,
            'last_error': self._last_error,
        }
