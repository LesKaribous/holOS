"""
shared/settings.py — holOS-side persistent settings store.

Replaces the firmware SD card RuntimeConfig persistence.
Settings are stored as a JSON file and pushed to the firmware
via cfg_set commands on every connect/reconnect.

Usage:
    from shared.settings import SettingsStore
    settings = SettingsStore()              # loads from default path
    settings.set("servo.CA.0.min", "110")   # set + auto-save
    settings.push_all(transport)            # push all to firmware
"""

import json
import os
import threading
from pathlib import Path
from typing import Optional, Callable

# Default location: next to the software/ root → software/data/settings.json
_DEFAULT_PATH = Path(__file__).resolve().parent.parent / "data" / "settings.json"


class SettingsStore:
    """
    JSON-backed key-value store that mirrors firmware RuntimeConfig.

    Thread-safe.  Auto-saves on every set().
    """

    def __init__(self, path: Optional[str] = None, log: Optional[Callable] = None):
        self._path = Path(path) if path else _DEFAULT_PATH
        self._lock = threading.Lock()
        self._data: dict[str, str] = {}
        self._log = log or print
        self._load()

    # ── Persistence ──────────────────────────────────────────────────────────

    def _load(self) -> None:
        if self._path.exists():
            try:
                with open(self._path, "r") as f:
                    raw = json.load(f)
                # Ensure all values are strings
                self._data = {str(k): str(v) for k, v in raw.items()}
                self._log(f"[Settings] Loaded {len(self._data)} entries from {self._path.name}")
            except Exception as e:
                self._log(f"[Settings] Failed to load {self._path}: {e}")
                self._data = {}
        else:
            self._log(f"[Settings] No settings file — starting empty")
            self._data = {}

    def _save(self) -> None:
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            with open(self._path, "w") as f:
                json.dump(self._data, f, indent=2, sort_keys=True)
        except Exception as e:
            self._log(f"[Settings] Failed to save: {e}")

    # ── Read / Write ─────────────────────────────────────────────────────────

    def get(self, key: str, default: str = "") -> str:
        with self._lock:
            return self._data.get(key, default)

    def get_int(self, key: str, default: int = 0) -> int:
        val = self.get(key)
        if not val:
            return default
        try:
            return int(val)
        except ValueError:
            return default

    def set(self, key: str, value: str) -> None:
        with self._lock:
            self._data[key] = str(value)
            self._save()

    def remove(self, key: str) -> None:
        with self._lock:
            self._data.pop(key, None)
            self._save()

    def all(self) -> dict[str, str]:
        with self._lock:
            return dict(self._data)

    def count(self) -> int:
        with self._lock:
            return len(self._data)

    # ── Firmware sync ────────────────────────────────────────────────────────

    def push_all(self, transport, timeout_ms: int = 2000) -> int:
        """
        Push all settings to firmware via cfg_set commands.
        Returns the number of successfully pushed entries.
        """
        with self._lock:
            entries = list(self._data.items())

        pushed = 0
        for key, val in entries:
            try:
                ok, _ = transport.execute(f"cfg_set({key},{val})",
                                          timeout_ms=timeout_ms)
                if ok:
                    pushed += 1
                else:
                    self._log(f"[Settings] cfg_set({key}) failed")
            except Exception as e:
                self._log(f"[Settings] cfg_set({key}) error: {e}")

        self._log(f"[Settings] Pushed {pushed}/{len(entries)} entries to firmware")
        return pushed

    def pull_from_firmware(self, transport, timeout_ms: int = 2000) -> bool:
        """
        Pull current config from firmware via cfg_list and merge into store.
        Only updates keys that firmware reports — does not delete local-only keys.
        """
        try:
            ok, raw = transport.execute("cfg_list", timeout_ms=timeout_ms)
            if not ok or not raw:
                return False

            # Parse "key1=val1;key2=val2;..."
            pairs = raw.split(";")
            count = 0
            with self._lock:
                for pair in pairs:
                    if "=" in pair:
                        k, v = pair.split("=", 1)
                        k, v = k.strip(), v.strip()
                        if k:
                            self._data[k] = v
                            count += 1
                self._save()

            self._log(f"[Settings] Pulled {count} entries from firmware")
            return True
        except Exception as e:
            self._log(f"[Settings] Pull failed: {e}")
            return False

    # ── Calibration warning helper ───────────────────────────────────────────

    CALIBRATION_KEYS = frozenset([
        "servo.CA.0.min", "servo.CA.0.max",
        "servo.CA.1.min", "servo.CA.1.max",
        "servo.CA.2.min", "servo.CA.2.max",
        "servo.AB.3.min", "servo.AB.3.max",
        "servo.AB.4.min", "servo.AB.4.max",
    ])

    def is_calibration_key(self, key: str) -> bool:
        """Return True if this key is a calibration value that should trigger a warning."""
        return key in self.CALIBRATION_KEYS or key.startswith("servo.")
