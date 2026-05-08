#!/usr/bin/env python3
"""
Vision pipeline DEBUG runner launcher (vision.bat / vision.sh).

Defaults to reading from the vision_camera virtual-camera server
(http://127.0.0.1:5174/stream.mjpg) — launch vision_camera FIRST in
another terminal, then this one.

Modes:
  - No arguments      → read from the virtual camera (no prompt). Whatever
                        vision_camera is serving is what the runner sees.
  - --pick            → force the interactive source picker (browse local
                        videos/images/USB cams, bypass virtual camera).
  - One argument      → treat it as a source path. Kind auto-detected by
                        extension.
                            python vision.py vision/homography/data/video.3.mp4
                            python vision.py 0
                            python vision.py http://other-host:5174/stream.mjpg
  - Two arguments     → kind + path, same as the runner CLI:
                            python vision.py video path/to/clip.mp4
                            python vision.py camera 0
                            python vision.py image path/to/pic.png

Pass --port / --host / etc. through the same way as the runner.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_RUNNER_DIR = _HERE / 'vision_runner'
_LAST = _RUNNER_DIR / 'last_source.json'

VIDEO_EXTS = {'.mp4', '.mkv', '.avi', '.mov', '.webm', '.m4v'}
IMAGE_EXTS = {'.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff'}


def _kind_from_path(p: str) -> str:
    if p.isdigit():
        return 'camera'
    ext = Path(p).suffix.lower()
    if ext in VIDEO_EXTS: return 'video'
    if ext in IMAGE_EXTS: return 'image'
    raise SystemExit(f'unknown source type for {p!r} — pass kind explicitly: '
                     f'python vision.py video|camera|image PATH')


def _scan_sources() -> list[tuple[str, str, str]]:
    """Scan well-known media folders. Returns list of (kind, path, label)."""
    found = []
    candidates = [
        _HERE / 'vision' / 'data',
        _HERE / 'vision' / 'homography' / 'data',
    ]
    for root in candidates:
        if not root.is_dir():
            continue
        for f in sorted(root.iterdir()):
            ext = f.suffix.lower()
            if ext in VIDEO_EXTS:
                found.append(('video', str(f),
                              f.relative_to(_HERE).as_posix()))
            elif ext in IMAGE_EXTS:
                found.append(('image', str(f),
                              f.relative_to(_HERE).as_posix()))
    # Always offer camera 0 as last option
    found.append(('camera', '0', 'camera (device 0)'))
    return found


def _load_last() -> tuple[str, str] | None:
    try:
        d = json.loads(_LAST.read_text())
        return d.get('kind'), d.get('path')
    except Exception:
        return None


def _save_last(kind: str, path: str) -> None:
    try:
        _RUNNER_DIR.mkdir(parents=True, exist_ok=True)
        _LAST.write_text(json.dumps({'kind': kind, 'path': path}, indent=2))
    except Exception:
        pass


def _interactive_pick() -> tuple[str, str]:
    sources = _scan_sources()
    last = _load_last()
    print('\n  Vision pipeline launcher')
    print('  ' + '─' * 40)
    default_idx = 0
    for i, (kind, path, label) in enumerate(sources, start=1):
        marker = ''
        if last and last[0] == kind and last[1] == path:
            marker = '  ← last used'
            default_idx = i
        print(f'  {i:>2}. [{kind:<6}] {label}{marker}')
    print()
    prompt = (f'  Pick a source [1-{len(sources)}]'
              + (f' (default {default_idx})' if default_idx else '')
              + ': ')
    while True:
        raw = input(prompt).strip()
        if not raw and default_idx:
            choice = default_idx
            break
        try:
            choice = int(raw)
            if 1 <= choice <= len(sources):
                break
        except ValueError:
            pass
        print('  → enter a number from the list.')
    kind, path, label = sources[choice - 1]
    _save_last(kind, path)
    print(f'  → launching {kind}: {label}\n')
    return kind, path


# Default source: the vision_camera virtual-camera server. Skipping this
# means the user must launch vision_camera FIRST (or pass an explicit path
# / --pick to read a file directly).
DEFAULT_SOURCE_KIND = 'video'
DEFAULT_SOURCE_PATH = 'http://127.0.0.1:5174/stream.mjpg'


def main() -> None:
    # Splice off any --port / --host / extra args; pass them through to runner.
    extras = []
    positional = []
    force_pick = False
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        a = args[i]
        if a == '--pick':
            force_pick = True
        elif a.startswith('--'):
            # --key=val → keep as one token; --key val → two tokens
            extras.append(a)
            if '=' not in a and (i + 1) < len(args) and not args[i + 1].startswith('--'):
                extras.append(args[i + 1])
                i += 1
        else:
            positional.append(a)
        i += 1

    if force_pick:
        kind, path = _interactive_pick()
    elif len(positional) == 0:
        # No args → silently default to the virtual camera. The runner's
        # source.video node will retry-connect if vision_camera isn't up
        # yet, so launch order doesn't have to be strict.
        kind, path = DEFAULT_SOURCE_KIND, DEFAULT_SOURCE_PATH
        print(f'  → reading from virtual camera: {path}')
        print(f'    (pass --pick to choose a local file/camera instead)\n')
    elif len(positional) == 1:
        path = positional[0]
        kind = _kind_from_path(path) if not path.startswith(('http://', 'https://', 'rtsp://')) else 'video'
        _save_last(kind, path)
    elif len(positional) == 2:
        kind, path = positional[0], positional[1]
        _save_last(kind, path)
    else:
        raise SystemExit('usage: python vision.py [SOURCE_KIND] SOURCE_PATH '
                         '[--port N] [--host H] [--pick]')

    # Hand over to the runner
    sys.argv = ['vision_runner', kind, path, *extras]
    if str(_HERE) not in sys.path:
        sys.path.insert(0, str(_HERE))
    from vision_runner.runner import main as runner_main
    runner_main()


if __name__ == '__main__':
    main()
