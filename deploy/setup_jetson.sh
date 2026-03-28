#!/usr/bin/env bash
# =============================================================================
#  holOS — Jetson deployment setup
#  Run once on a fresh Jetson with JetPack installed.
#
#  Usage:
#    chmod +x deploy/setup_jetson.sh
#    ./deploy/setup_jetson.sh
#
#  What it does:
#    1. Updates apt and installs system dependencies
#    2. Installs Python packages from py/requirements.txt
#    3. Detects (or prompts for) the XBee serial port
#    4. Installs a systemd service  (holOS.service)
#    5. Enables and starts the service
#
#  After setup:
#    - holOS runs at boot on port 5000
#    - Access from PC:  http://<jetson-ip>:5000
#    - Connect to robot via the "Robot" panel in the UI
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PY_DIR="$PROJECT_ROOT/py"
SERVICE_FILE="$SCRIPT_DIR/holOS.service"
SERVICE_DEST="/etc/systemd/system/holOS.service"
PYTHON="python3"
PORT=5000

# ── Colours ───────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'
info()    { echo -e "${CYAN}[INFO]${NC}  $*"; }
success() { echo -e "${GREEN}[OK]${NC}    $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error()   { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

echo -e "\n${BOLD}holOS Jetson Setup${NC}"
echo "────────────────────────────────────────"
echo "Project : $PROJECT_ROOT"
echo "Python  : $($PYTHON --version 2>&1)"
echo "────────────────────────────────────────"

# ── 1. System dependencies ───────────────────────────────────────────────────
info "Updating apt and installing system packages…"
sudo apt-get update -qq
# libssl-dev needed by gevent/cryptography; python3-dev for C extensions
sudo apt-get install -y -qq \
    python3-pip python3-dev python3-venv \
    libssl-dev libffi-dev \
    2>/dev/null
success "System packages installed"

# ── 2. Python virtualenv + packages ──────────────────────────────────────────
VENV="$PY_DIR/.venv"
if [ ! -d "$VENV" ]; then
    info "Creating virtualenv at $VENV …"
    $PYTHON -m venv "$VENV"
fi

info "Installing Python requirements…"
"$VENV/bin/pip" install --upgrade pip -q
"$VENV/bin/pip" install -r "$PY_DIR/requirements.txt" -q
PYTHON_EXEC="$VENV/bin/python"
success "Python dependencies installed (venv: $VENV)"

# ── 3. XBee serial port detection ────────────────────────────────────────────
info "Scanning for XBee / serial ports…"
XBEE_PORT=""
# Typical Jetson ports: /dev/ttyUSB0 (USB XBee dongle), /dev/ttyTHS* (UART)
for port in /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyTHS1 /dev/ttyTHS0 /dev/ttyACM0; do
    if [ -c "$port" ]; then
        XBEE_PORT="$port"
        info "  Found: $port"
    fi
done

if [ -z "$XBEE_PORT" ]; then
    warn "No serial device detected. The Robot panel in the UI will let you select one at runtime."
else
    success "Default XBee port: $XBEE_PORT"
    # Add current user to dialout group for serial access
    sudo usermod -aG dialout "$USER" 2>/dev/null || true
    success "User '$USER' added to dialout group (re-login to apply)"
fi

# ── 4. Generate systemd service file ─────────────────────────────────────────
info "Generating systemd service file…"
cat > "$SERVICE_FILE" << EOF
[Unit]
Description=holOS Simulator / Dashboard
After=network.target
Wants=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$PY_DIR
ExecStart=$PYTHON_EXEC run_sim.py --host 0.0.0.0 --port $PORT
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal
# Allow access to serial / USB ports
SupplementaryGroups=dialout

[Install]
WantedBy=multi-user.target
EOF
success "Service file written: $SERVICE_FILE"

# ── 5. Install and enable service ────────────────────────────────────────────
info "Installing systemd service…"
sudo cp "$SERVICE_FILE" "$SERVICE_DEST"
sudo systemctl daemon-reload
sudo systemctl enable holOS.service
sudo systemctl restart holOS.service
sleep 2

if sudo systemctl is-active --quiet holOS.service; then
    success "holOS service is running ✓"
else
    warn "Service failed to start. Check logs with:  sudo journalctl -u holOS -n 40"
fi

# ── Summary ───────────────────────────────────────────────────────────────────
JETSON_IP=$(hostname -I | awk '{print $1}')
echo ""
echo -e "${BOLD}────────────────────────────────────────${NC}"
echo -e "${GREEN}Setup complete!${NC}"
echo ""
echo "  Dashboard URL (from your PC):"
echo -e "  ${BOLD}http://$JETSON_IP:$PORT${NC}"
echo ""
echo "  Useful commands:"
echo "    sudo systemctl status holOS     — check service status"
echo "    sudo systemctl restart holOS    — restart after code change"
echo "    sudo journalctl -u holOS -f     — follow logs"
echo "    sudo systemctl disable holOS    — disable autostart"
echo ""
if [ -n "$XBEE_PORT" ]; then
    echo "  XBee port detected: $XBEE_PORT (31250 baud)"
    echo "  → In the UI, click 'Robot' → select $XBEE_PORT → Connect"
fi
echo -e "${BOLD}────────────────────────────────────────${NC}"
