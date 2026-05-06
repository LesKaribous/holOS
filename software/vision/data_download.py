#!/usr/bin/env python3
"""
data_download.py — Personal data fetcher for the holOS vision pipeline.

Big assets (match videos, calibration frames, …) don't belong in git. This
script reads `data_sources.json` (which IS gitignored — copy it from
`data_sources.example.json`) and downloads the listed entries into
`software/vision/data/`.

The TwinVision UI / holOS visio tab can then point at those files via
their relative path under `software/vision/data/…`.

Usage
-----
    python software/vision/data_download.py             # download everything
    python software/vision/data_download.py --list      # show what's configured
    python software/vision/data_download.py --section videos
    python software/vision/data_download.py --key match_2024_finale
    python software/vision/data_download.py --force     # re-download even if file exists
    python software/vision/data_download.py --check     # only verify checksums

Manifest format
---------------
See `data_sources.example.json`. Top-level keys are sections (`videos`,
`images`, …) — anything goes; each section is a list of entries with
`key`, `url`, optional `dest`, optional `sha256`.

Skips files that already exist (size > 0) unless --force or --check is
set. With `sha256` provided, verifies on every run.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Optional

HERE          = Path(__file__).resolve().parent
MANIFEST_PATH = HERE / 'data_sources.json'
EXAMPLE_PATH  = HERE / 'data_sources.example.json'
DATA_DIR      = HERE / 'data'

# Minimal terminal coloring without a library dep
def _c(s: str, code: str) -> str:
    return f'\033[{code}m{s}\033[0m' if sys.stdout.isatty() else s
def _green(s):  return _c(s, '32')
def _red(s):    return _c(s, '31')
def _yellow(s): return _c(s, '33')
def _blue(s):   return _c(s, '34')
def _dim(s):    return _c(s, '2')


# ── Manifest loading ────────────────────────────────────────────────────────

def load_manifest() -> dict:
    if not MANIFEST_PATH.exists():
        print(_red(f'No manifest at {MANIFEST_PATH}'))
        if EXAMPLE_PATH.exists():
            print(f'Hint: cp {EXAMPLE_PATH.name} {MANIFEST_PATH.name} '
                  f'and edit your private URLs.')
        sys.exit(2)
    with open(MANIFEST_PATH, 'r', encoding='utf-8') as f:
        return json.load(f)


def iter_entries(manifest: dict, section: Optional[str] = None,
                 key: Optional[str] = None):
    """Yield (section, entry_dict) pairs from the manifest."""
    for sec, items in manifest.items():
        if sec.startswith('_'):  # skip _comment / _format meta
            continue
        if section and sec != section:
            continue
        if not isinstance(items, list):
            continue
        for entry in items:
            if not isinstance(entry, dict):
                continue
            if key and entry.get('key') != key:
                continue
            yield sec, entry


# ── Path resolution ─────────────────────────────────────────────────────────

def entry_dest(entry: dict) -> Path:
    """Return the absolute destination path for an entry."""
    if entry.get('dest'):
        return DATA_DIR / entry['dest']
    # No explicit dest → derive from key + URL extension.
    url_path = urllib.parse.urlparse(entry['url']).path
    ext = Path(url_path).suffix or '.bin'
    key = entry.get('key', 'unnamed')
    return DATA_DIR / f'{key}{ext}'


# ── Hashing ─────────────────────────────────────────────────────────────────

def sha256_file(path: Path, chunk: int = 1 << 20) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        while True:
            buf = f.read(chunk)
            if not buf:
                break
            h.update(buf)
    return h.hexdigest()


# ── Download ────────────────────────────────────────────────────────────────

def _human_size(n: int) -> str:
    for unit in ('B', 'KB', 'MB', 'GB', 'TB'):
        if n < 1024:
            return f'{n:.1f} {unit}'
        n /= 1024
    return f'{n:.1f} PB'


def download(url: str, dest: Path, force: bool = False) -> tuple[bool, str]:
    """Download `url` to `dest`. Returns (ok, message). Skips if dest already
    has content unless force=True. Streams to a .part file then renames."""
    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.exists() and dest.stat().st_size > 0 and not force:
        return True, f'exists ({_human_size(dest.stat().st_size)})'

    tmp = dest.with_suffix(dest.suffix + '.part')
    try:
        req = urllib.request.Request(url, headers={'User-Agent': 'holOS-vision-downloader/1.0'})
        with urllib.request.urlopen(req, timeout=60) as resp:
            total = int(resp.headers.get('Content-Length', 0))
            written = 0
            last_pct = -1
            with open(tmp, 'wb') as f:
                while True:
                    chunk = resp.read(1 << 20)   # 1 MiB
                    if not chunk:
                        break
                    f.write(chunk)
                    written += len(chunk)
                    if total > 0:
                        pct = int(written * 100 / total)
                        if pct != last_pct and pct % 5 == 0:
                            sys.stdout.write(
                                f'\r  ↓ {pct:3d}%  '
                                f'{_human_size(written)}/{_human_size(total)}'
                            )
                            sys.stdout.flush()
                            last_pct = pct
            sys.stdout.write('\r' + ' ' * 60 + '\r')
            sys.stdout.flush()
        tmp.replace(dest)
        return True, f'downloaded ({_human_size(dest.stat().st_size)})'
    except Exception as e:
        if tmp.exists():
            try: tmp.unlink()
            except OSError: pass
        return False, f'{type(e).__name__}: {e}'


# ── Commands ────────────────────────────────────────────────────────────────

def cmd_list(manifest: dict, section: Optional[str], key: Optional[str]):
    """Print the manifest with current download status."""
    n = 0
    for sec, entry in iter_entries(manifest, section, key):
        n += 1
        dest = entry_dest(entry)
        if dest.exists() and dest.stat().st_size > 0:
            status = _green(f'OK  {_human_size(dest.stat().st_size):>10}')
        else:
            status = _yellow('MISSING')
        print(f'  [{sec:8s}] {entry.get("key", "?"):30s} {status}  '
              f'{_dim(str(dest.relative_to(HERE)))}')
    if n == 0:
        print(_yellow('Nothing matched the filter.'))


def cmd_check(manifest: dict, section: Optional[str], key: Optional[str]) -> int:
    """Verify checksums (where provided). Returns 0 if all OK, 1 otherwise."""
    fail = 0
    for sec, entry in iter_entries(manifest, section, key):
        dest = entry_dest(entry)
        want = entry.get('sha256')
        if not want:
            continue
        if not dest.exists():
            print(f'  {_red("MISSING")}  {entry["key"]}  ({dest})')
            fail += 1
            continue
        got = sha256_file(dest)
        if got != want:
            print(f'  {_red("BAD HASH")}  {entry["key"]}')
            print(f'    expected {want}')
            print(f'    actual   {got}')
            fail += 1
        else:
            print(f'  {_green("OK")}        {entry["key"]}')
    return 1 if fail else 0


def cmd_download(manifest: dict, section: Optional[str], key: Optional[str],
                 force: bool) -> int:
    """Download all matching entries."""
    fail = 0
    n = 0
    for sec, entry in iter_entries(manifest, section, key):
        n += 1
        dest = entry_dest(entry)
        print(f'  [{sec:8s}] {entry.get("key", "?"):30s} → '
              f'{_blue(str(dest.relative_to(HERE)))}')
        ok, msg = download(entry['url'], dest, force=force)
        if ok:
            print(f'           {_green(msg)}')
            # Verify hash if provided
            want = entry.get('sha256')
            if want:
                got = sha256_file(dest)
                if got != want:
                    print(f'           {_red("HASH MISMATCH")} — file kept; '
                          f'rerun with --force to retry')
                    print(f'             expected {want}')
                    print(f'             actual   {got}')
                    fail += 1
                else:
                    print(f'           {_green("hash verified")}')
        else:
            print(f'           {_red(msg)}')
            fail += 1
    if n == 0:
        print(_yellow('Nothing matched the filter.'))
    return 1 if fail else 0


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(
        description='Download personal vision data (videos / images) listed in data_sources.json.',
    )
    p.add_argument('--list', action='store_true',
                   help='Show manifest entries + download status, then exit.')
    p.add_argument('--check', action='store_true',
                   help='Verify SHA-256 hashes (only entries with sha256 set).')
    p.add_argument('--section',
                   help='Limit to a single manifest section (videos, images, …).')
    p.add_argument('--key',
                   help='Limit to a single entry by its "key" field.')
    p.add_argument('--force', action='store_true',
                   help='Re-download even if the destination already exists.')
    args = p.parse_args()

    manifest = load_manifest()

    if args.list:
        cmd_list(manifest, args.section, args.key)
        return 0

    if args.check:
        return cmd_check(manifest, args.section, args.key)

    return cmd_download(manifest, args.section, args.key, args.force)


if __name__ == '__main__':
    sys.exit(main())
