"""Match logger — captures UART, holOS events, ESPCam text logs during a
manually-started REC session for later replay.

The VIDEO part of the recording is owned by the in-process Recorder
(vision_recorder.py): it samples the shared FrameSource's latest-BGR slot
at `record_fps` and writes an AVI directly from BGR. MatchLogger.start()/
stop() just kick the recorder.

Single source of truth for text logs: every line carries `t_ms` (monotonic
ms since session start) + `wall` (ISO timestamp). The video AVI's frame
indices line up with wall-clock via the session's `meta.json` (`started_wall`
+ `record_fps`).
"""
from __future__ import annotations

import json
import threading
import queue
import time
from datetime import datetime
from pathlib import Path
from typing import Optional


_DEFAULT_ROOT    = Path(__file__).resolve().parents[1] / 'logs'
_VISION_CFG      = Path(__file__).resolve().parents[1] / 'data' / 'vision_config.json'
_DEFAULT_REC_FPS = 16.0


def _read_record_fps() -> float:
    try:
        return float(json.loads(_VISION_CFG.read_text()).get(
            'record_fps', _DEFAULT_REC_FPS))
    except Exception:
        return _DEFAULT_REC_FPS


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
        self._video_summary: Optional[dict] = None  # filled at stop()
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
        }

    def start(self, meta: 'dict | None' = None) -> str:
        with self._lock:
            if self.is_recording():
                return self._session_id  # already recording — no-op
            now = datetime.now()
            self._session_id = now.strftime('%Y-%m-%d_%H-%M-%S')
            self._session_dir = self._root / self._session_id
            self._session_dir.mkdir(parents=True, exist_ok=True)
            self._t0_mono = time.monotonic()
            self._t0_wall = time.time()
            self._stop_evt.clear()
            self._video_summary = None
            fps = _read_record_fps()
            # Kick the in-process recorder; it samples the FrameSource's
            # latest-BGR slot and writes video.avi directly.
            try:
                import vision_recorder
                r = vision_recorder.start_recording(
                    str(self._session_dir / 'video.avi'), fps)
                if not r.get('ok'):
                    print(f'[match-logger] recorder start failed: {r}')
            except Exception as e:
                print(f'[match-logger] vision_recorder unavailable: {e}')
            meta_doc = {
                'session_id':   self._session_id,
                'started_wall': now.isoformat(timespec='seconds'),
                't0_mono':      self._t0_mono,
                'record_fps':   fps,
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
            for t in list(self._writers.values()):
                t.join(timeout=2.0)
            for f in self._files.values():
                try: f.close()
                except Exception: pass
            self._files.clear()
            self._queues.clear()
            self._writers.clear()
            try:
                import vision_recorder
                self._video_summary = vision_recorder.stop_recording()
            except Exception as e:
                self._video_summary = {'ok': False, 'error': f'{e}'}
            video_frames = int(self._video_summary.get('frames_written', 0)) \
                if self._video_summary else 0
            elapsed = time.monotonic() - (self._t0_mono or time.monotonic())
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


# Singleton — imported and reused everywhere.
MATCH_LOGGER = MatchLogger()
