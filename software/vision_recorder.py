"""
Recorder — taps FrameSource's latest BGR slot at its own rate and writes
an AVI directly from BGR, no JPEG round-trip.

The recorder runs entirely independently of the analysis pipeline: each
samples the same in-process slot at its own rate (e.g. recorder @ 16 Hz,
pipeline @ 8 Hz), and both always see the latest frame, never a queued
one. The pipeline never blocks the recorder, nor vice-versa.

cv2.VideoWriter with MJPG fourcc is the default — light CPU cost, decent
quality, plays everywhere. A future GStreamer-pipeline variant (nvjpegenc
on Jetson) could move encoding to the GPU; the API leaves room for it.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import cv2

import vision_source as _vs


class Recorder:
    """Single-session recorder. Lifecycle:
        start(out_path, fps) → spawns the writer thread
        stop()               → flushes + closes; returns summary dict
    Re-start by calling start() again with a new path."""

    def __init__(self):
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._out_path: Optional[str] = None
        self._fps: float = 16.0
        self._frames_written: int = 0
        self._started_at: float = 0.0
        self._last_error: Optional[str] = None
        self._lock = threading.Lock()

    def is_recording(self) -> bool:
        t = self._thread
        return t is not None and t.is_alive()

    def start(self, out_path: str, fps: float) -> dict:
        with self._lock:
            if self.is_recording():
                return {'ok': False, 'error': 'already recording',
                        'path': self._out_path}
            self._out_path = str(out_path)
            self._fps = max(1.0, float(fps))
            self._frames_written = 0
            self._started_at = time.monotonic()
            self._last_error = None
            self._stop.clear()
            self._thread = threading.Thread(
                target=self._record_loop, daemon=True, name='vision-recorder')
            self._thread.start()
        return {'ok': True, 'path': self._out_path, 'fps': self._fps}

    def stop(self) -> dict:
        with self._lock:
            if not self.is_recording():
                return {'ok': False, 'error': 'not recording'}
            self._stop.set()
            t = self._thread
        if t is not None:
            t.join(timeout=3.0)
        elapsed = time.monotonic() - self._started_at
        return {'ok': True, 'path': self._out_path,
                'frames_written': self._frames_written,
                'duration_s': round(elapsed, 2),
                'last_error': self._last_error}

    def status(self) -> dict:
        recording = self.is_recording()
        elapsed = (time.monotonic() - self._started_at) if recording else 0.0
        return {
            'recording':      recording,
            'path':           self._out_path,
            'fps':            self._fps,
            'frames_written': self._frames_written,
            'elapsed_s':      round(elapsed, 2) if recording else 0.0,
            'last_error':     self._last_error,
        }

    # ── internals ───────────────────────────────────────────────────────

    def _record_loop(self) -> None:
        src = _vs.get()
        if src is None:
            self._last_error = 'no FrameSource configured'
            return
        writer: Optional[cv2.VideoWriter] = None
        period = 1.0 / self._fps if self._fps > 0 else 0.25
        next_t = time.monotonic()
        try:
            while not self._stop.is_set():
                now = time.monotonic()
                wait = next_t - now
                if wait > 0:
                    time.sleep(min(wait, 0.05))
                    continue
                next_t += period
                f = src.read_latest()
                if f is None:
                    continue
                if writer is None:
                    h, w = f.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                    writer = cv2.VideoWriter(
                        self._out_path, fourcc, self._fps, (w, h))
                    if not writer.isOpened():
                        self._last_error = (
                            f'cv2.VideoWriter failed to open {self._out_path!r}')
                        return
                    print(f'[recorder] {self._out_path} '
                          f'({w}x{h} MJPG @ {self._fps}fps)')
                # Copy: the reader thread will overwrite this slot on its
                # next read. The encode runs without the slot lock so we
                # must own the bytes for the duration of write().
                try:
                    writer.write(f.copy())
                except Exception as e:
                    self._last_error = f'write: {type(e).__name__}: {e}'
                    continue
                self._frames_written += 1
        finally:
            if writer is not None:
                try:
                    writer.release()
                except Exception:
                    pass
            print(f'[recorder] closed: {self._frames_written} frames')


# ─── Module-level singleton ────────────────────────────────────────────────

_INSTANCE: Optional[Recorder] = None
_INSTANCE_LOCK = threading.Lock()


def get() -> Recorder:
    """Lazy-create the singleton. Safe to call from any thread."""
    global _INSTANCE
    with _INSTANCE_LOCK:
        if _INSTANCE is None:
            _INSTANCE = Recorder()
        return _INSTANCE


def start_recording(out_path: str, fps: float) -> dict:
    return get().start(out_path, fps)


def stop_recording() -> dict:
    return get().stop()


def status() -> dict:
    return get().status()
