#!/usr/bin/env bash
# Launch the Webots sim and the mario autonomy binary against it.
#
#   ./run_sim.sh [--gui] [--mario-only] [--webots-only]
#
# Two processes, two transports:
#   Webots mario_bridge  --(pty /tmp/mario_serial)-->  mario   drive + geodetic
#                        --(tcp://127.0.0.1:5599)--->  mario   RGB-D + cloud
set -euo pipefail

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SIM_DIR")"

WORLD="$SIM_DIR/worlds/mars_yard.wbt"
PTY_LINK="/tmp/mario_serial"
ENDPOINT="tcp://127.0.0.1:5599"
MARIO_BIN="${MARIO_BIN:-$REPO_DIR/build/mario}"
RERUN_IP="${RERUN_IP:-127.0.0.1:9876}"

# Realtime, not fast. traverse() and approach() derive their PID dt from
# std::chrono::system_clock -- wall clock, not sim time -- so running the sim
# faster than realtime makes every control gain meaningless. Fast mode also
# floods the pty with geodetic frames faster than mario drains them, so plan()
# would read a backlog that is seconds stale.
WEBOTS_MODE="--minimize --batch --mode=realtime"
RUN_WEBOTS=1
RUN_MARIO=1
for arg in "$@"; do
  case "$arg" in
    --gui)         WEBOTS_MODE="--mode=realtime" ;;
    --mario-only)  RUN_WEBOTS=0 ;;
    --webots-only) RUN_MARIO=0 ;;
    *) echo "unknown option: $arg" >&2; exit 2 ;;
  esac
done

PIDS=()
cleanup() {
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ $RUN_WEBOTS -eq 1 ]]; then
  rm -f "$PTY_LINK"
  echo ">> starting webots: $WORLD"
  # shellcheck disable=SC2086
  webots $WEBOTS_MODE --stdout --stderr "$WORLD" &
  PIDS+=($!)

  echo ">> waiting for the bridge to create $PTY_LINK"
  for _ in $(seq 1 60); do
    [[ -e "$PTY_LINK" ]] && break
    sleep 0.5
  done
  if [[ ! -e "$PTY_LINK" ]]; then
    echo "!! bridge never created $PTY_LINK -- check the webots output above" >&2
    exit 1
  fi
  echo ">> bridge is up ($PTY_LINK -> $(readlink -f "$PTY_LINK"))"
fi

if [[ $RUN_MARIO -eq 0 ]]; then
  echo ">> --webots-only: leaving the sim running, Ctrl-C to stop"
  wait
  exit 0
fi

if [[ ! -x "$MARIO_BIN" ]]; then
  echo "!! $MARIO_BIN not found or not executable." >&2
  echo "   Build it first (see sim/README.md), or set MARIO_BIN=/path/to/mario." >&2
  exit 1
fi

# mario calls rec.connect_grpc(...).exit_on_failure(), so it aborts immediately
# if no Rerun viewer is listening. Warn early rather than let it die opaquely.
if ! (exec 3<>"/dev/tcp/${RERUN_IP%%:*}/${RERUN_IP##*:}") 2>/dev/null; then
  echo "!! nothing listening on $RERUN_IP -- mario will exit on startup." >&2
  echo "   Start a viewer first:  rerun --serve-web --port ${RERUN_IP##*:}" >&2
  echo "   (or set RERUN_IP=host:port)" >&2
  exit 1
fi

echo ">> starting mario against the sim"
cd "$REPO_DIR"
exec "$MARIO_BIN" \
  --sim \
  --zmq_endpoint "$ENDPOINT" \
  --serial "$PTY_LINK" \
  --br 115200 \
  --slam_config "$SIM_DIR/config/stellaconf_sim.yaml" \
  --slam_vocab "$REPO_DIR/orb_vocab.fbow" \
  --gridmap_config "$SIM_DIR/config/gridmap_sim.yaml" \
  --gnss "$SIM_DIR/config/gnss_waypoints.txt" \
  --rerun_ip "$RERUN_IP" \
  --marker_size 0.336 \
  --p 1.2 --i 0.0 --d 0.05 \
  --linear 0.6 --angular 1.0 \
  --yolo_model "${YOLO_MODEL:-$REPO_DIR/model/yolov8n.onnx}" \
  --yolo_labels "$REPO_DIR/model/labels.names"
