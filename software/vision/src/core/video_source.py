"""
VideoSource — abstraction unifiée pour fichier vidéo ou caméra live.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


@dataclass
class VideoInfo:
    width: int
    height: int
    fps: float
    frame_count: int  # -1 pour caméra live
    source_name: str

    @property
    def duration_s(self) -> float:
        if self.frame_count < 0:
            return -1.0
        return self.frame_count / self.fps if self.fps > 0 else 0.0

    @property
    def is_live(self) -> bool:
        return self.frame_count < 0


class VideoSource:
    """
    Wrapper unifié autour de cv2.VideoCapture.
    Supporte : fichier mp4/mkv et caméra (index ou URL RTSP).
    """

    def __init__(self, source: str | int):
        self._source = source
        self._cap: Optional[cv2.VideoCapture] = None
        self._info: Optional[VideoInfo] = None
        self._current_frame_idx: int = 0
        self._last_read_time: float = 0.0

    def open(self) -> bool:
        if self._cap is not None:
            self._cap.release()

        self._cap = cv2.VideoCapture(self._source)
        if not self._cap.isOpened():
            return False

        w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self._cap.get(cv2.CAP_PROP_FPS)
        fc = int(self._cap.get(cv2.CAP_PROP_FRAME_COUNT))

        # fc <= 0 => source live (caméra)
        if fc <= 0:
            fc = -1

        name = str(self._source) if isinstance(self._source, int) else self._source
        self._info = VideoInfo(
            width=w, height=h, fps=fps if fps > 0 else 30.0,
            frame_count=fc, source_name=name
        )
        self._current_frame_idx = 0
        return True

    @property
    def is_open(self) -> bool:
        return self._cap is not None and self._cap.isOpened()

    @property
    def info(self) -> Optional[VideoInfo]:
        return self._info

    @property
    def current_frame_idx(self) -> int:
        return self._current_frame_idx

    def read(self) -> Optional[np.ndarray]:
        if not self.is_open:
            return None
        ret, frame = self._cap.read()
        if not ret:
            return None
        self._current_frame_idx = int(self._cap.get(cv2.CAP_PROP_POS_FRAMES))
        self._last_read_time = time.monotonic()
        return frame

    def seek(self, frame_idx: int) -> bool:
        """Seek à un frame précis (fichier seulement)."""
        if not self.is_open or self._info is None or self._info.is_live:
            return False
        frame_idx = max(0, min(frame_idx, self._info.frame_count - 1))
        self._cap.set(cv2.CAP_PROP_POS_FRAMES, float(frame_idx))
        self._current_frame_idx = frame_idx
        return True

    def seek_ms(self, ms: float) -> bool:
        if not self.is_open:
            return False
        self._cap.set(cv2.CAP_PROP_POS_MSEC, ms)
        self._current_frame_idx = int(self._cap.get(cv2.CAP_PROP_POS_FRAMES))
        return True

    def grab_current_frame(self) -> Optional[np.ndarray]:
        """Re-lit le frame courant sans avancer (utile pour pause)."""
        if not self.is_open or self._info is None or self._info.is_live:
            return None
        pos = self._current_frame_idx
        self.seek(pos)
        return self.read()

    def release(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        self._info = None
        self._current_frame_idx = 0
