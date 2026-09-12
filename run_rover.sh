#!/usr/bin/env bash
# run_rover.sh -- launch mario on the real rover: RealSense D435i attached,
# Nucleo on a serial port, depth-camera occupancy map.
#
#   ./run_rover.sh <gnss_waypoints.txt> [extra mario flags]
#
# Environment:
#   SERIAL      Nucleo port                 (default /dev/ttyACM0)
#   BAUD        (default 115200)
#   RERUN_IP    Rerun viewer host:port      (default 127.0.0.1:9876). mario
#               exits at startup if nothing is listening there.
#   GRIDMAP     (default config/gridmap_rover.yaml -- MEASURE its offsets)
#   YOLO_MODEL  ONNX detector, optional. Without it the object waypoint
#               searches for 60 s and takes partial credit.
#   LINEAR / ANGULAR   speed caps, m/s and rad/s (default 0.5 / 0.8)
#   P / I / D          heading PID gains (default 1.2 / 0 / 0.05, sim-tuned)
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GNSS="${1:?usage: $0 <gnss_waypoints.txt> [extra flags]}"; shift || true

MARIO_BIN="${MARIO_BIN:-$REPO_DIR/build-rel/mario}"
[[ -x "$MARIO_BIN" ]] || { echo "!! $MARIO_BIN not built" >&2; exit 1; }
[[ -e "${SERIAL:-/dev/ttyACM0}" ]] || { echo "!! serial port ${SERIAL:-/dev/ttyACM0} not present" >&2; exit 1; }

cd "$REPO_DIR"
exec "$MARIO_BIN" \
  --serial "${SERIAL:-/dev/ttyACM0}" \
  --br "${BAUD:-115200}" \
  --slam_config "$REPO_DIR/stellaconf.yaml" \
  --slam_vocab "$REPO_DIR/orb_vocab.fbow" \
  --gridmap_config "${GRIDMAP:-$REPO_DIR/config/gridmap_rover.yaml}" \
  --gnss "$GNSS" \
  --rerun_ip "${RERUN_IP:-127.0.0.1:9876}" \
  --cloud_source depth \
  --p "${P:-1.2}" --i "${I:-0.0}" --d "${D:-0.05}" \
  --linear "${LINEAR:-0.5}" --angular "${ANGULAR:-0.8}" \
  --yolo_model "${YOLO_MODEL:-}" \
  --yolo_labels "$REPO_DIR/model/labels.names" \
  "$@"
