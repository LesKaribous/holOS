#!/usr/bin/env bash
# =============================================================================
#  holOS — Quick update script (run after git pull on Jetson)
#
#  Usage:
#    ./deploy/update_jetson.sh
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY_DIR="$(cd "$SCRIPT_DIR/../py" && pwd)"
VENV="$PY_DIR/.venv"

GREEN='\033[0;32m'; CYAN='\033[0;36m'; NC='\033[0m'
info()    { echo -e "${CYAN}[INFO]${NC}  $*"; }
success() { echo -e "${GREEN}[OK]${NC}    $*"; }

info "Updating Python packages…"
"$VENV/bin/pip" install -r "$PY_DIR/requirements.txt" -q
success "Packages up to date"

info "Restarting holOS service…"
sudo systemctl restart holOS.service
sleep 1
sudo systemctl is-active --quiet holOS.service && success "holOS restarted ✓" || echo "Check: sudo journalctl -u holOS -n 20"
