#!/bin/sh
# Virtual-camera launcher (Linux/macOS).
cd "$(dirname "$0")" || exit 1
if command -v python3 >/dev/null 2>&1; then
    exec python3 vision_camera_launcher.py "$@"
elif command -v python >/dev/null 2>&1; then
    exec python vision_camera_launcher.py "$@"
else
    echo "No Python interpreter on PATH." >&2
    exit 1
fi
