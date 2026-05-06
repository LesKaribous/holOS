"""
preprocess node — single configurable image-processing operation. The legacy
backend embedded all of these in one big switch; here each is selectable via
a `mode` param so the editor can plug in multiple preprocess nodes in series.

Modes: passthrough | grayscale | gaussian_blur | median_blur | clahe |
       binarize | canny | brightness | gamma
"""

from __future__ import annotations

try:
    import cv2
    import numpy as np
    _CV2_OK = True
except Exception:
    _CV2_OK = False

from ..pipeline import NodeIO, Port, PortKind
from .base import Node, register_node


@register_node('preprocess')
class PreprocessNode(Node):
    IO = NodeIO(
        inputs=[Port('frame', PortKind.FRAME)],
        outputs=[
            Port('frame',   PortKind.FRAME,   'processed frame (clean)'),
            Port('preview', PortKind.PREVIEW, 'same frame, for inline display'),
        ],
    )
    params_schema = {
        'mode':    {'type': 'str', 'default': 'passthrough',
                    'enum': ['passthrough', 'grayscale', 'gaussian_blur',
                             'median_blur', 'clahe', 'binarize', 'canny',
                             'brightness', 'gamma']},
        'k':       {'type': 'int',   'default': 5,    'help': 'kernel size (odd)'},
        'sigma':   {'type': 'float', 'default': 1.5},
        'thresh':  {'type': 'int',   'default': 128},
        'invert':  {'type': 'bool',  'default': False},
        'canny_lo':{'type': 'int',   'default': 50},
        'canny_hi':{'type': 'int',   'default': 150},
        'alpha':   {'type': 'float', 'default': 1.0,  'help': 'brightness gain'},
        'beta':    {'type': 'float', 'default': 0.0,  'help': 'brightness bias'},
        'gamma':   {'type': 'float', 'default': 1.0},
        'clip':    {'type': 'float', 'default': 3.0,  'help': 'CLAHE clip limit'},
        'tile':    {'type': 'int',   'default': 8,    'help': 'CLAHE tile grid'},
    }

    def process(self, inputs):
        frame = inputs.get('frame')
        if frame is None or not _CV2_OK:
            return {}
        mode = self._params.get('mode', 'passthrough')
        try:
            out = self._apply(frame, mode)
            return {'frame': out, 'preview': out}
        except Exception as e:
            self._last_error = f'{mode}: {e}'
            return {'frame': frame, 'preview': frame}

    def _apply(self, frame, mode: str):
        p = self._params
        if mode == 'passthrough':
            return frame
        if mode == 'grayscale':
            g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            return cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        if mode == 'gaussian_blur':
            k = max(1, int(p.get('k', 5)) | 1)   # ensure odd
            return cv2.GaussianBlur(frame, (k, k), float(p.get('sigma', 1.5)))
        if mode == 'median_blur':
            k = max(1, int(p.get('k', 5)) | 1)
            return cv2.medianBlur(frame, k)
        if mode == 'clahe':
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            cl = cv2.createCLAHE(
                clipLimit=float(p.get('clip', 3.0)),
                tileGridSize=(int(p.get('tile', 8)), int(p.get('tile', 8))),
            )
            l = cl.apply(l)
            return cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
        if mode == 'binarize':
            g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            flag = cv2.THRESH_BINARY_INV if p.get('invert') else cv2.THRESH_BINARY
            _, b = cv2.threshold(g, int(p.get('thresh', 128)), 255, flag)
            return cv2.cvtColor(b, cv2.COLOR_GRAY2BGR)
        if mode == 'canny':
            g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            e = cv2.Canny(g, int(p.get('canny_lo', 50)), int(p.get('canny_hi', 150)))
            return cv2.cvtColor(e, cv2.COLOR_GRAY2BGR)
        if mode == 'brightness':
            return cv2.convertScaleAbs(frame,
                                       alpha=float(p.get('alpha', 1.0)),
                                       beta=float(p.get('beta', 0.0)))
        if mode == 'gamma':
            g = max(0.01, float(p.get('gamma', 1.0)))
            inv = 1.0 / g
            table = np.array([((i / 255.0) ** inv) * 255
                              for i in range(256)]).astype('uint8')
            return cv2.LUT(frame, table)
        return frame
