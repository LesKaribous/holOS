#!/usr/bin/env python3
"""
Virtual-camera launcher with interactive source picker.

Usage:
    python vision_camera_launcher.py                       (interactive menu)
    python vision_camera_launcher.py path/to/clip.mp4      (kind auto-detected)
    python vision_camera_launcher.py video clip.mp4
    python vision_camera_launcher.py camera 0
    python vision_camera_launcher.py --port 5174 ...
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent     # software/tools
_SW   = _HERE.parent                         # software/
_LAST = _SW / 'vision_camera' / 'last_source.json'

VIDEO_EXTS = {'.mp4', '.mkv', '.avi', '.mov', '.webm', '.m4v'}
IMAGE_EXTS = {'.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff'}


def _kind_from_path(p: str) -> str:
    if p.isdigit(): return 'camera'
    ext = Path(p).suffix.lower()
    if ext in VIDEO_EXTS: return 'video'
    if ext in IMAGE_EXTS: return 'image'
    raise SystemExit(f'unknown source kind for {p!r} — pass kind explicitly')


def _scan_sources():
    found = []
    for root in [_SW / 'vision' / 'data',
                 _SW / 'vision' / 'homography' / 'data']:
        if root.is_dir():
            for f in sorted(root.iterdir()):
                ext = f.suffix.lower()
                if ext in VIDEO_EXTS:
                    found.append(('video', str(f), f.relative_to(_SW).as_posix()))
                elif ext in IMAGE_EXTS:
                    found.append(('image', str(f), f.relative_to(_SW).as_posix()))
    found.append(('camera', '0', 'camera (device 0)'))
    return found


def _load_last():
    try: return json.loads(_LAST.read_text())
    except: return None


def _save_last(kind, path):
    try:
        _LAST.parent.mkdir(parents=True, exist_ok=True)
        _LAST.write_text(json.dumps({'kind': kind, 'path': path}, indent=2))
    except: pass


def _interactive_pick():
    sources = _scan_sources()
    last = _load_last() or {}
    print('\n  Virtual camera launcher')
    print('  ' + '─' * 40)
    default_idx = 0
    for i, (kind, path, label) in enumerate(sources, 1):
        marker = ''
        if last.get('kind') == kind and last.get('path') == path:
            marker = '  ← last used'; default_idx = i
        print(f'  {i:>2}. [{kind:<6}] {label}{marker}')
    print()
    prompt = (f'  Pick a source [1-{len(sources)}]'
              + (f' (default {default_idx})' if default_idx else '') + ': ')
    while True:
        raw = input(prompt).strip()
        if not raw and default_idx: choice = default_idx; break
        try:
            n = int(raw)
            if 1 <= n <= len(sources): choice = n; break
        except ValueError: pass
        print('  → enter a number from the list.')
    kind, path, label = sources[choice - 1]
    _save_last(kind, path)
    print(f'  → launching {kind}: {label}\n')
    return kind, path


def main():
    extras, positional = [], []
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        a = args[i]
        if a.startswith('--'):
            extras.append(a)
            if '=' not in a and (i + 1) < len(args) and not args[i + 1].startswith('--'):
                extras.append(args[i + 1]); i += 1
        else:
            positional.append(a)
        i += 1

    if not positional:
        kind, path = _interactive_pick()
    elif len(positional) == 1:
        path = positional[0]; kind = _kind_from_path(path)
        _save_last(kind, path)
    else:
        kind, path = positional[0], positional[1]
        _save_last(kind, path)

    # vision_camera is a sibling package of tools/ at software/. Push
    # the software root onto sys.path so `from vision_camera.camera` resolves.
    sys.argv = ['vision_camera', kind, path, *extras]
    if str(_SW) not in sys.path:
        sys.path.insert(0, str(_SW))
    from vision_camera.camera import main as cam_main
    cam_main()


if __name__ == '__main__':
    main()
