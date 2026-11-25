#!/usr/bin/env bash
set -euo pipefail
: "${DISPLAY:?The demo needs a local X11 display. Use run_webots_demo.sh.}"
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS
mkdir -p "$HOME"
export G1_RUN_DIR="$(mktemp -d "${G1_OUTPUT_PARENT:-/var/lib/g1-demo}/run.XXXXXXXX")"
set +u
source /opt/ros/jazzy/setup.bash
set -u
simulator_pid=''
live_pid=''
cleanup() {
    local rc=$?
    trap - EXIT
    for pid in "$live_pid" "$simulator_pid"; do
        if [[ -n "$pid" ]]; then kill -INT "$pid" 2>/dev/null || true; fi
    done
    for pid in "$live_pid" "$simulator_pid"; do
        if [[ -n "$pid" ]]; then wait "$pid" 2>/dev/null || true; fi
    done
    exit "$rc"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
printf 'G1 live Webots demo. Run data at %s\n' "$G1_RUN_DIR"
bash /opt/capstone/start_simulator.sh > "$G1_RUN_DIR/webots.log" 2>&1 &
simulator_pid=$!
bash /opt/g1/live.sh &
live_pid=$!
if wait -n "$simulator_pid" "$live_pid"; then
    status=0
else
    status=$?
fi
if [[ ! -f "$G1_RUN_DIR/finished" || $(cat "$G1_RUN_DIR/finished") != 0 ]]; then
    tail -15 "$G1_RUN_DIR/webots.log" >&2
    exit "${status:-1}"
fi
printf 'G1 route complete. Close the Webots window to end the demo.\n'
python3 - "$G1_RUN_DIR" <<'PY'
import json
from pathlib import Path
import sys
p=Path(sys.argv[1]); run=json.loads((p/'drive.json').read_text())
t=run['timing']; start=t['first_nonzero_command']; end=t['settled']
real=end['wall_s']-start['wall_s']; sim=end['sim_s']-start['sim_s']
print(f'Waypoints: {len(run["reached"])}. Motion and stop: {real:.2f} s wall, {sim:.2f} s simulated ({sim/real:.2f}x).')
if (p/'status.json').is_file():
    data=json.loads((p/'status.json').read_text())
    for method, row in data['methods'].items():
        print(f'{method}: ICP accepted {row["accepted"]}, fixed-noise fallback {row["fallback"]}.')
PY
if ! wait "$simulator_pid" && ! grep -Fq 'robot driver exited with code 0' "$G1_RUN_DIR/webots.log"; then
    tail -15 "$G1_RUN_DIR/webots.log" >&2
    exit 1
fi
