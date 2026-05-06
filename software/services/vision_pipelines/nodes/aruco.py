"""
ArUco detection node — detects markers in the input frame, outputs a
DetectionResult (and optionally a frame with markers drawn on top).
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
    from core.aruco_detector import ArucoDetector, ARUCO_DICTS, DetectionResult
    _CV2_OK = True
except Exception:
    _CV2_OK = False

from ..pipeline import NodeIO, Port, PortKind
from .base import Node, register_node


@register_node('aruco')
class ArucoNode(Node):
    IO = NodeIO(
        inputs=[Port('frame', PortKind.FRAME)],
        outputs=[
            Port('detection', PortKind.DETECTION, 'DetectionResult'),
            Port('preview', PortKind.PREVIEW, 'frame with markers drawn'),
        ],
    )
    params_schema = {
        'dict': {
            'type': 'str', 'default': '4x4_50',
            'label': 'ArUco dictionary',
            'enum': list(ARUCO_DICTS.keys()) if _CV2_OK else [],
            'description': 'must match the markers printed on the table / robots',
        },
        'refine': {
            'type': 'str', 'default': 'subpix',
            'label': 'corner refinement',
            'enum': ['none', 'subpix', 'contour', 'apriltag'],
            'description': 'subpix = good default; contour = best accuracy; '
                           'apriltag = best for APRILTAG_* dictionaries',
        },
        'draw_markers': {
            'type': 'bool', 'default': True,
            'label': 'draw markers on preview',
            'description': 'when ON, preview shows every detected marker. '
                           'Turn OFF if you want a downstream overlay node '
                           'to do the (filtered) drawing — otherwise the '
                           'raw markers are baked into the preview before '
                           'any filter can subset them.',
        },
        'draw_ids': {
            'type': 'bool', 'default': True,
            'label': 'draw IDs',
            'description': 'numeric labels on each marker (preview output). '
                           'Ignored when draw_markers is OFF.',
        },
    }

    def start(self):
        if not _CV2_OK:
            self._last_error = 'cv2.aruco unavailable'
            return
        try:
            self._det = ArucoDetector(
                self._params.get('dict', '4x4_50'),
                self._params.get('refine', 'subpix'),
            )
        except Exception as e:
            self._last_error = f'detector init: {e}'

    def process(self, inputs):
        frame = inputs.get('frame')
        if frame is None or getattr(self, '_det', None) is None:
            return {}
        # Pick up param changes lazily.
        want_dict = self._params.get('dict', '4x4_50')
        if self._det.dictionary_name != want_dict:
            self._det.set_dictionary(want_dict)
        result = self._det.detect(frame)
        out = {'detection': result}
        # Preview output behaviour:
        #  - draw_markers ON  → raw frame with EVERY detected marker drawn
        #    (good for quick debug, BUT downstream filters can't subset what's
        #    already painted onto the image — use only when no filter chain
        #    follows).
        #  - draw_markers OFF → preview is the clean frame. Use this when
        #    you have a `filter` + `overlay` chain downstream that should
        #    do its own (subsetted) drawing.
        if self._params.get('draw_markers', True):
            out['preview'] = self._det.draw(
                frame, result, draw_ids=self._params.get('draw_ids', True),
            )
        else:
            out['preview'] = frame
        return out

    def get_state(self):
        det = getattr(self, '_det', None)
        return {
            'dict': det.dictionary_name if det else None,
            'last_error': self._last_error,
        }
