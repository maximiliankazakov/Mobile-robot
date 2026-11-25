#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /opt/g1/install/local_setup.bash
out=${G1_RUN_DIR:?}
localizer=/opt/g1/install/capstone_localization/lib/capstone_localization/localization_node
safety=/opt/g1/install/capstone_safety/lib/capstone_safety/safety_monitor_node
pids=()
cleanup() {
    local code=$?
    trap - EXIT
    printf '%s\n' "$code" > "$out/.finished"
    mv "$out/.finished" "$out/finished"
    if [[ -n "${observer:-}" ]]; then
        for ((i=0; i<12; i++)); do
            if ! kill -0 "$observer" 2>/dev/null; then break; fi
            sleep .05
        done
    fi
    for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
    for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
    exit "$code"
}
trap cleanup EXIT
trap 'exit 143' TERM INT
"$localizer" --ros-args -r __node:=g1_method_b -p use_sim_time:=true -p method:=learned \
  -p model_path:=/opt/g1/data/model.txt > "$out/g1-b.log" 2>&1 &
pids+=("$!")
"$localizer" --ros-args -r __node:=g1_method_a -p use_sim_time:=true -p method:=classical \
  -r /localization/pose:=/g1/a/pose -r /state/odom:=/g1/a/odom \
  -r /diagnostics:=/g1/a/diagnostics -r /tf:=/g1/a/tf > "$out/g1-a.log" 2>&1 &
pids+=("$!")
python3 /opt/g1/map.py > "$out/map.log" 2>&1 &
pids+=("$!")
"$safety" --ros-args -p use_sim_time:=true -p robot_radius:=0.272 -p margin:=0.35 \
  -p v_max:=0.15 -p w_max:=0.35 > "$out/safety.log" 2>&1 &
pids+=("$!")
python3 /opt/g1/observe.py > "$out/observe.log" 2>&1 &
observer=$!
pids+=("$observer")
timeout -k 4s 82s python3 /opt/g1/drive.py | tee "$out/drive.log"
