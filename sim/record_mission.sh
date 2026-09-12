#!/usr/bin/env bash
# record_mission.sh -- record the full autonomy mission, driven by the real
# mario binary rather than by sim/tools/fsm_drive.
#
#   ./sim/record_mission.sh [output.mp4]
#
# record_run.sh is the older, dependency-free equivalent: it drives fsm_drive,
# which follows GPS bearings and needs neither SLAM nor a grid map. This one
# runs the actual autonomy stack -- stella_vslam, the occupancy map, A* and
# the FSM -- so it needs the full build and something listening where a Rerun
# viewer would (mario exits at startup otherwise; a plain TCP accept is
# enough, see RERUN_IP below).
#
# The video is the rover's own camera, depth and lidar off the ZMQ bus
# (sim/tools/record_stream.py), not a screen grab: on an XWayland session
# x11grab of the Webots window records black.
#
# Environment:
#   MARIO_BIN     path to the binary          (default build-rel/mario)
#   RERUN_IP      viewer address              (default 127.0.0.1:9876)
#   CLOUD_SOURCE  depth | lidar               (default depth -- the D435i is
#                                              the sensor the real rover has)
#   YOLO_MODEL    ONNX detector, optional     (none: object waypoint times out)
#   REC_FPS       capture frame rate          (default 15)
#   MISSION_MAX   seconds before giving up    (default 600)
set -uo pipefail

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SIM_DIR")"
OUT="${1:-$REPO_DIR/mission_run.mp4}"
LOG="${OUT%.mp4}.log"

WORLD="$SIM_DIR/worlds/mars_yard.wbt"
PTY_LINK="/tmp/mario_serial"
ENDPOINT="tcp://127.0.0.1:5599"
MARIO_BIN="${MARIO_BIN:-$REPO_DIR/build-rel/mario}"
RERUN_IP="${RERUN_IP:-127.0.0.1:9876}"
CLOUD_SOURCE="${CLOUD_SOURCE:-depth}"
FPS="${REC_FPS:-15}"
MISSION_MAX="${MISSION_MAX:-600}"
PYTHON="${PYTHON:-python3}"

command -v ffmpeg >/dev/null || { echo "!! ffmpeg not installed" >&2; exit 1; }
[[ -x "$MARIO_BIN" ]] || { echo "!! $MARIO_BIN not built" >&2; exit 1; }

WEBOTS_PID=""; REC_PID=""; MARIO_PID=""
cleanup() {
  [[ -n "$MARIO_PID"  ]] && kill "$MARIO_PID"  2>/dev/null
  # SIGINT, not SIGKILL: the recorder has to close its ffmpeg or the mp4 has
  # no moov atom and is unplayable.
  [[ -n "$REC_PID" ]] && kill -INT "$REC_PID" 2>/dev/null
  sleep 3
  [[ -n "$WEBOTS_PID" ]] && kill "$WEBOTS_PID" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

# Webots rewrites the .wbt on exit, stripping comments and saving wherever the
# rover stopped. --batch prevents that; this puts back anything an earlier
# interactive session already saved.
git -C "$REPO_DIR" checkout -- sim/worlds/ 2>/dev/null || true

if ! (exec 3<>"/dev/tcp/${RERUN_IP%%:*}/${RERUN_IP##*:}") 2>/dev/null; then
  echo "!! nothing listening on $RERUN_IP -- mario exits on startup without it." >&2
  echo "   rerun --serve-web --port ${RERUN_IP##*:}" >&2
  exit 1
fi

echo ">> starting webots (realtime)"
rm -f "$PTY_LINK"
# Realtime. Not --mode=fast: traverse() and approach() take their PID dt from
# the wall clock, and fast mode floods the pty faster than mario drains it.
webots --batch --mode=realtime --stdout --stderr "$WORLD" >"$LOG.webots" 2>&1 &
WEBOTS_PID=$!

for _ in $(seq 1 90); do [[ -e "$PTY_LINK" ]] && break; sleep 0.5; done
[[ -e "$PTY_LINK" ]] || { echo "!! bridge never came up; see $LOG.webots" >&2; exit 1; }
sleep 5   # let the first frames and the lidar settle

echo ">> recording the camera stream @ ${FPS}fps -> $OUT"
"$PYTHON" "$SIM_DIR/tools/record_stream.py" "$OUT" --endpoint "$ENDPOINT" \
       --fps "$FPS" --seconds "$((MISSION_MAX + 30))" >"$LOG.rec" 2>&1 &
REC_PID=$!
sleep 2

echo ">> running the mission ($CLOUD_SOURCE map)"
cd "$REPO_DIR"
stdbuf -o0 "$MARIO_BIN" \
  --sim \
  --zmq_endpoint "$ENDPOINT" \
  --serial "$PTY_LINK" \
  --br 115200 \
  --slam_config "$SIM_DIR/config/stellaconf_sim.yaml" \
  --slam_vocab "$REPO_DIR/orb_vocab.fbow" \
  --gridmap_config "$SIM_DIR/config/gridmap_sim.yaml" \
  --gnss "$SIM_DIR/config/gnss_waypoints.txt" \
  --rerun_ip "$RERUN_IP" \
  --cloud_source "$CLOUD_SOURCE" \
  --marker_size 0.336 \
  --p 1.2 --i 0.0 --d 0.05 \
  --linear 0.6 --angular 1.0 \
  --yolo_model "${YOLO_MODEL:-}" \
  --yolo_labels "$REPO_DIR/model/labels.names" >"$LOG" 2>&1 &
MARIO_PID=$!

# mario exits on its own now, 0 for MISSION_DONE and 2 for an abort.
deadline=$((SECONDS + MISSION_MAX))
while (( SECONDS < deadline )) && kill -0 "$MARIO_PID" 2>/dev/null; do
  sleep 2
done
if kill -0 "$MARIO_PID" 2>/dev/null; then
  echo "!! mission still running after ${MISSION_MAX}s -- stopping it"
else
  wait "$MARIO_PID"; rc=$?
  case $rc in
    0) echo ">> mission complete" ;;
    2) echo "!! mission aborted -- see $LOG" ;;
    *) echo "!! mario exited with $rc -- see $LOG" ;;
  esac
fi
MARIO_PID=""
sleep 3

grep -E "State:|mission complete|aborted in" "$LOG" | tail -20
echo ">> video: $OUT"
echo ">> log:   $LOG"
