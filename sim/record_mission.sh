#!/usr/bin/env bash
# record_mission.sh -- record the full autonomy mission, driven by the real
# mario binary rather than by sim/tools/fsm_drive.
#
#   ./sim/record_mission.sh [output.mp4]
#
# record_run.sh is the older, dependency-free equivalent: it drives fsm_drive,
# which follows GPS bearings and needs neither SLAM nor a grid map. This one
# runs the actual autonomy stack -- stella_vslam, the lidar occupancy map, A*
# and the FSM -- so it needs the full build and a Rerun viewer.
#
# Environment:
#   MARIO_BIN     path to the binary          (default build-rel/mario)
#   RERUN_IP      viewer address              (default 127.0.0.1:9876)
#   CLOUD_SOURCE  lidar | depth               (default lidar)
#   YOLO_MODEL    ONNX detector               (see KNOWN_ISSUES issue 9)
#   REC_FPS       capture frame rate          (default 20)
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
CLOUD_SOURCE="${CLOUD_SOURCE:-lidar}"
DISPLAY_ID="${DISPLAY:-:0}"
FPS="${REC_FPS:-20}"
MISSION_MAX="${MISSION_MAX:-600}"
SIZE="${REC_SIZE:-$(xdpyinfo -display "$DISPLAY_ID" 2>/dev/null | awk '/dimensions:/{print $2}')}"
SIZE="${SIZE:-1920x1080}"

command -v ffmpeg >/dev/null || { echo "!! ffmpeg not installed" >&2; exit 1; }
[[ -x "$MARIO_BIN" ]] || { echo "!! $MARIO_BIN not built" >&2; exit 1; }

WEBOTS_PID=""; FFMPEG_PID=""; MARIO_PID=""
cleanup() {
  [[ -n "$MARIO_PID"  ]] && kill "$MARIO_PID"  2>/dev/null
  # SIGINT, not SIGKILL: ffmpeg has to write the moov atom or the mp4 is
  # unplayable.
  [[ -n "$FFMPEG_PID" ]] && kill -INT "$FFMPEG_PID" 2>/dev/null
  sleep 2
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

echo ">> starting webots (GUI, realtime)"
rm -f "$PTY_LINK"
# Realtime and windowed. Not --minimize: there is nothing to film if the 3D
# view is not being drawn. Not --mode=fast either -- traverse() and approach()
# take their PID dt from the wall clock.
webots --batch --mode=realtime --stdout --stderr "$WORLD" >"$LOG.webots" 2>&1 &
WEBOTS_PID=$!

for _ in $(seq 1 90); do [[ -e "$PTY_LINK" ]] && break; sleep 0.5; done
[[ -e "$PTY_LINK" ]] || { echo "!! bridge never came up; see $LOG.webots" >&2; exit 1; }
sleep 5   # let the first frames, the lidar and the GUI settle

echo ">> recording $SIZE @ ${FPS}fps -> $OUT"
ffmpeg -y -loglevel error -f x11grab -framerate "$FPS" -video_size "$SIZE" \
       -i "$DISPLAY_ID" -c:v libx264 -preset veryfast -crf 23 \
       -pix_fmt yuv420p "$OUT" &
FFMPEG_PID=$!
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
  --yolo_model "${YOLO_MODEL:-$REPO_DIR/model/yolov8n.onnx}" \
  --yolo_labels "$REPO_DIR/model/labels.names" >"$LOG" 2>&1 &
MARIO_PID=$!

# mario's worker threads loop forever, so it never returns even once the
# mission is done (KNOWN_ISSUES open issue 7). Watch the log for the end of
# the run instead of waiting on the process.
deadline=$((SECONDS + MISSION_MAX))
while (( SECONDS < deadline )); do
  if grep -q "mission complete" "$LOG" 2>/dev/null; then
    echo ">> mission complete"
    sleep 3
    break
  fi
  if grep -q "aborted in" "$LOG" 2>/dev/null; then
    echo "!! mission aborted -- see $LOG"
    sleep 3
    break
  fi
  kill -0 "$MARIO_PID" 2>/dev/null || { echo "!! mario exited early"; break; }
  sleep 2
done

grep -E "State:|mission complete|aborted in" "$LOG" | tail -20
echo ">> video: $OUT"
echo ">> log:   $LOG"
