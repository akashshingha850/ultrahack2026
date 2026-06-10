#!/usr/bin/env bash
#
# Autostart for the ULTRAHACK 2026 mission.
#
# Brings up the MediaMTX RTSP server, waits for the flight-controller serial
# link, then launches main.py. Safe to run on boot (systemd / rc.local / cron
# @reboot) or by hand. All output is tee'd to logs/autostart_<timestamp>.log.
#
# Hook into systemd (recommended):
#   sudo tee /etc/systemd/system/ultrahack.service >/dev/null <<'UNIT'
#   [Unit]
#   Description=ULTRAHACK 2026 mission
#   After=network-online.target docker.service
#   Wants=network-online.target
#   [Service]
#   Type=simple
#   User=jetson
#   WorkingDirectory=/home/jetson/ultrahack2026
#   ExecStart=/home/jetson/ultrahack2026/autostart.sh
#   Restart=on-failure
#   RestartSec=5
#   [Install]
#   WantedBy=multi-user.target
#   UNIT
#   sudo systemctl daemon-reload && sudo systemctl enable --now ultrahack
#
set -uo pipefail

# ── Resolve project directory (works no matter where it's invoked from) ───────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

SERIAL_DEV="/dev/ttyTHS1"     # flight-controller link (matches config.yaml)
MEDIAMTX_DIR="$SCRIPT_DIR/mediamtx"
LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/autostart_$(date +%Y-%m-%d_%H-%M-%S).log"

# Mirror all stdout/stderr to the log file as well as the console.
exec > >(tee -a "$LOG_FILE") 2>&1

log() { echo "[autostart $(date +%H:%M:%S)] $*"; }

# ── Pick a Python: prefer .venv if present, else system python3 ───────────────
if [[ -x "$SCRIPT_DIR/.venv/bin/python" ]]; then
  PYTHON="$SCRIPT_DIR/.venv/bin/python"
else
  PYTHON="$(command -v python3)"
fi
log "Using Python: $PYTHON"

# ── Wait for the flight-controller serial device (up to 60 s) ─────────────────
log "Waiting for flight controller at $SERIAL_DEV …"
for _ in $(seq 1 60); do
  [[ -e "$SERIAL_DEV" ]] && break
  sleep 1
done
if [[ -e "$SERIAL_DEV" ]]; then
  log "Flight controller present at $SERIAL_DEV"
else
  log "WARNING: $SERIAL_DEV not found after 60 s — continuing anyway"
fi

# ── Bring up the MediaMTX RTSP server (best-effort, never fatal) ──────────────
if command -v docker >/dev/null 2>&1; then
  # Wait for the docker daemon to be ready (up to 30 s).
  for _ in $(seq 1 30); do
    docker info >/dev/null 2>&1 && break
    sleep 1
  done
  if docker info >/dev/null 2>&1; then
    log "Starting MediaMTX (docker compose up -d) …"
    ( cd "$MEDIAMTX_DIR" && docker compose up -d ) \
      || log "WARNING: failed to start MediaMTX — the annotated stream may be unavailable"
  else
    log "WARNING: docker daemon not ready — skipping MediaMTX"
  fi
else
  log "WARNING: docker not installed — skipping MediaMTX"
fi

# ── Launch the mission ────────────────────────────────────────────────────────
log "Launching mission: $PYTHON main.py"
exec "$PYTHON" main.py
