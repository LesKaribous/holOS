#!/bin/sh
# Virtual-camera launcher (Linux/macOS).
#
# Wrapper layout: this script lives in software/scripts/, the launcher
# is at software/tools/vision_camera.py. We cd to software/ then call
# tools/vision_camera.py so its sys.path resolution finds vision_camera/
# as a sibling.
cd "$(dirname "$0")/.." || exit 1
if command -v python3 >/dev/null 2>&1; then
    exec python3 tools/vision_camera.py "$@"
elif command -v python >/dev/null 2>&1; then
    exec python tools/vision_camera.py "$@"
else
    echo "No Python interpreter on PATH." >&2
    exit 1
fi
