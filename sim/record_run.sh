#!/usr/bin/env bash
# record_run.sh -- record a video of the FSM driving the mars yard course.
#
#   ./sim/record_run.sh [output.mp4]
#
# Resets the world, starts Webots, screen-records while sim/tools/fsm_drive
# runs the mission, and stops when the mission ends. The FSM transitions are
# printed to the terminal as it goes and also saved next to the video, so the
# recording and the state log line up.
set -uo pipefail

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SIM_DIR")"
OUT="${1:-$REPO_DIR/fsm_run.mp4}"
LOG="${OUT%.mp4}.log"

WORLD="$SIM_DIR/worlds/mars_yard.wbt"
PTY_LINK="/tmp/mario_serial"
DISPLAY_ID="${DISPLAY:-:0}"
SIZE="${REC_SIZE:-$(xdpyinfo -display "$DISPLAY_ID" 2>/dev/null | awk '/dimensions:/{print $2}')}"
SIZE="${SIZE:-1920x1080}"
FPS="${REC_FPS:-20}"

command -v ffmpeg >/dev/null || { echo "!! ffmpeg not installed" >&2; exit 1; }
[[ -x "$SIM_DIR/tools/fsm_drive" ]] || {
  echo "!! sim/tools/fsm_drive not built. See sim/tools/Makefile." >&2; exit 1; }

WEBOTS_PID=""; FFMPEG_PID=""
cleanup() {
  [[ -n "$FFMPEG_PID" ]] && kill -INT "$FFMPEG_PID" 2>/dev/null
  sleep 1
  [[ -n "$WEBOTS_PID" ]] && kill "$WEBOTS_PID" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

# Webots rewrites the .wbt on exit -- it strips the file's comments and saves
# wherever the rover happened to stop. --batch prevents that, and this puts
# back anything an earlier interactive session already saved.
git -C "$REPO_DIR" checkout -- sim/worlds/ 2>/dev/null || true

echo ">> starting webots"
rm -f "$PTY_LINK"
webots --batch --mode=realtime --stdout --stderr "$WORLD" >"$LOG.webots" 2>&1 &
WEBOTS_PID=$!

for _ in $(seq 1 80); do [[ -e "$PTY_LINK" ]] && break; sleep 0.5; done
[[ -e "$PTY_LINK" ]] || { echo "!! bridge never came up; see $LOG.webots" >&2; exit 1; }
sleep 4   # let the first frames and the GUI settle

echo ">> recording $SIZE @ ${FPS}fps -> $OUT"
ffmpeg -y -loglevel error -f x11grab -framerate "$FPS" -video_size "$SIZE" \
       -i "$DISPLAY_ID" -c:v libx264 -preset veryfast -pix_fmt yuv420p \
       "$OUT" &
FFMPEG_PID=$!
sleep 2

echo ">> running the mission"
( cd "$SIM_DIR/tools" && stdbuf -o0 ./fsm_drive \
    --waypoints="$SIM_DIR/config/gnss_waypoints.txt" ) | tee "$LOG"
RC=${PIPESTATUS[0]}

sleep 2
echo ">> mission exited $RC, stopping recording"
cleanup
trap - EXIT

echo
echo "video : $OUT"
echo "log   : $LOG"
exit "$RC"
