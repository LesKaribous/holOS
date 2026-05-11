"""Match logger — captures UART, holOS events, ESPCam, and video frames
during a manually-started REC session for later replay.

Single source of truth: every log line carries `t_ms` (monotonic ms since
session start, used for sync with video frames) + `wall` (ISO timestamp).
Video frames are written as JPEGs named `NNNNNN_t=Xs.jpg` so any viewer
can match a frame back to log lines by timestamp.

Design constraints:
  - Caller-side overhead must be near zero. log() / log_video() push to
    a per-source bounded queue and return immediately. Writer threads
    drain queues and write to disk.
  - When no session is active, both log() and log_video() are O(1) checks.
  - On session stop, queues are drained before returning so no in-flight
    log is lost.
"""
from __future__ import annotations

import json
import os
import queue
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

try:
    import cv2
    _CV2_OK = True
except Exception:
    _CV2_OK = False


_DEFAULT_ROOT = Path(__file__).resolve().parents[1] / 'logs'


class MatchLogger:
    def __init__(self, root: 'Path | str | None' = None):
        self._root = Path(root) if root else _DEFAULT_ROOT
        self._session_dir: Optional[Path] = None
        self._session_id: Optional[str] = None
        self._t0_mono: Optional[float] = None
        self._t0_wall: Optional[float] = None
        self._files: dict = {}            # source -> open file
        self._queues: dict = {}           # source -> Queue
        self._writers: dict = {}          # source -> Thread
        self._video_q: 'queue.Queue | None' = None
        self._video_thread: Optional[threading.Thread] = None
        self._video_idx = 0
        self._stop_evt = threading.Event()
        self._lock = threading.Lock()

    # ── Public API ──────────────────────────────────────────────────────

    def is_recording(self) -> bool:
        return self._session_dir is not None

    def status(self) -> dict:
        if not self.is_recording():
            return {'recording': False}
        elapsed = time.monotonic() - (self._t0_mono or time.monotonic())
        return {
            'recording':  True,
            'session_id': self._session_id,
            'path':       str(self._session_dir),
            'elapsed_s':  round(elapsed, 1),
            'video_frames': self._video_idx,
        }

    def start(self, meta: 'dict | None' = None) -> str:
        with self._lock:
            if self.is_recording():
                return self._session_id  # already recording — no-op
            now = datetime.now()
            self._session_id = now.strftime('%Y-%m-%d_%H-%M-%S')
            self._session_dir = self._root / self._session_id
            (self._session_dir / 'video').mkdir(parents=True, exist_ok=True)
            self._t0_mono = time.monotonic()
            self._t0_wall = time.time()
            self._stop_evt.clear()
            self._video_idx = 0
            self._video_q = queue.Queue(maxsize=64)
            self._video_thread = threading.Thread(
                target=self._video_writer_loop, daemon=True,
                name='match-logger-video')
            self._video_thread.start()
            meta_doc = {
                'session_id':   self._session_id,
                'started_wall': now.isoformat(timespec='seconds'),
                't0_mono':      self._t0_mono,
                **(meta or {}),
            }
            (self._session_dir / 'meta.json').write_text(
                json.dumps(meta_doc, indent=2))
            return self._session_id

    def stop(self) -> 'dict | None':
        with self._lock:
            if not self.is_recording():
                return None
            sid = self._session_id
            sdir = self._session_dir
            self._stop_evt.set()
            for q in list(self._queues.values()):
                q.put(None)
            if self._video_q is not None:
                self._video_q.put(None)
            for t in list(self._writers.values()):
                t.join(timeout=2.0)
            if self._video_thread is not None:
                self._video_thread.join(timeout=3.0)
            for f in self._files.values():
                try: f.close()
                except Exception: pass
            self._files.clear()
            self._queues.clear()
            self._writers.clear()
            self._video_q = None
            self._video_thread = None
            elapsed = time.monotonic() - (self._t0_mono or time.monotonic())
            video_frames = self._video_idx
            try:
                meta_path = sdir / 'meta.json'
                meta = json.loads(meta_path.read_text())
                meta['ended_wall'] = datetime.now().isoformat(timespec='seconds')
                meta['duration_s'] = round(elapsed, 2)
                meta['video_frames'] = video_frames
                meta_path.write_text(json.dumps(meta, indent=2))
            except Exception:
                pass
            self._session_dir = None
            self._session_id = None
            self._t0_mono = None
            self._t0_wall = None
            return {'session_id': sid, 'path': str(sdir),
                    'duration_s': round(elapsed, 2),
                    'video_frames': video_frames}

    def log(self, source: str, line: str) -> None:
        if self._session_dir is None:
            return
        q = self._queues.get(source)
        if q is None:
            q = self._open_source(source)
        if q is None:
            return
        try:
            q.put_nowait(self._fmt(line))
        except queue.Full:
            pass  # drop on overflow rather than block the caller

    def log_video(self, frame_bgr, t_mono: 'float | None' = None) -> None:
        if self._session_dir is None or self._video_q is None or not _CV2_OK:
            return
        if frame_bgr is None:
            return
        t = (t_mono if t_mono is not None else time.monotonic())
        try:
            self._video_q.put_nowait((frame_bgr, t))
        except queue.Full:
            pass  # drop frame if writer is behind

    # ── Internals ──────────────────────────────────────────────────────

    def _fmt(self, line: str) -> str:
        t_ms = (time.monotonic() - self._t0_mono) * 1000.0
        wall = datetime.now().isoformat(timespec='milliseconds')
        return f'{t_ms:10.1f} {wall} {line}\n'

    def _open_source(self, source: str) -> 'queue.Queue | None':
        with self._lock:
            if not self.is_recording():
                return None
            if source in self._queues:
                return self._queues[source]
            safe = ''.join(c if c.isalnum() or c in '._-' else '_'
                           for c in source)
            path = self._session_dir / f'{safe}.log'
            try:
                f = open(path, 'a', buffering=8192)
            except Exception:
                return None
            self._files[source] = f
            q: 'queue.Queue' = queue.Queue(maxsize=4096)
            self._queues[source] = q
            t = threading.Thread(target=self._writer_loop, args=(source, q, f),
                                 daemon=True, name=f'match-logger-{safe}')
            self._writers[source] = t
            t.start()
            return q

    def _writer_loop(self, source: str, q: 'queue.Queue', f) -> None:
        while True:
            item = q.get()
            if item is None:
                break
            try:
                f.write(item)
            except Exception:
                pass
        try: f.flush()
        except Exception: pass

    def _video_writer_loop(self) -> None:
        while True:
            item = self._video_q.get() if self._video_q else None
            if item is None:
                break
            frame, t_mono = item
            if self._session_dir is None or self._t0_mono is None:
                continue
            t_rel = t_mono - self._t0_mono
            self._video_idx += 1
            name = f'{self._video_idx:06d}_t={t_rel:.3f}s.jpg'
            try:
                ok, buf = cv2.imencode('.jpg', frame,
                                       [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if ok:
                    (self._session_dir / 'video' / name).write_bytes(buf.tobytes())
            except Exception:
                pass


# Singleton — imported and reused everywhere.
MATCH_LOGGER = MatchLogger()
