#!/bin/sh
# Vision debug page launcher (Linux/macOS).
URL="http://127.0.0.1:5000/vision_debug"
if   command -v xdg-open >/dev/null 2>&1; then xdg-open "$URL"
elif command -v open     >/dev/null 2>&1; then open     "$URL"
else echo "Open in your browser: $URL"
fi
