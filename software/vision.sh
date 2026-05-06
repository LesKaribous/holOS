#!/bin/sh
# Vision pipeline launcher (Linux / macOS).
#   ./software/vision.sh                  (interactive menu)
#   ./software/vision.sh path/to/clip.mp4
#   ./software/vision.sh camera 0
cd "$(dirname "$0")" || exit 1

# Prefer python3, fall back to python.
if command -v python3 >/dev/null 2>&1; then
    exec python3 vision.py "$@"
elif command -v python >/dev/null 2>&1; then
    exec python vision.py "$@"
else
    echo "No Python interpreter found on PATH." >&2
    exit 1
fi
