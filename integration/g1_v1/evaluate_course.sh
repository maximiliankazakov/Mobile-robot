#!/usr/bin/env bash
# Build C++ evaluators and keep the ground truth outside the inference container.
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
if (( $# < 2 )); then
  echo "Usage: $0 DATASET WORK_DIR [--scenario NAME | --case ID] [--jobs N]" >&2
  exit 2
fi
dataset="$(realpath -e "$1")"
mkdir -p "$2"
work="$(realpath -e "$2")"
shift 2
mode="${G1_EVALUATION_MODE:-replay}"
if [[ "$mode" != replay && "$mode" != archived ]]; then
  echo "G1_EVALUATION_MODE must be replay or archived" >&2
  exit 2
fi
image="${G1_COURSE_IMAGE:-capstone-dev:local}"
base=(docker run --rm --network none --cap-drop ALL
      --security-opt no-new-privileges --entrypoint /bin/bash
      -e PYTHONDONTWRITEBYTECODE=1
      --mount "type=bind,src=$root,dst=/workspace,readonly"
      --mount "type=bind,src=$work,dst=/work"
      --mount "type=bind,src=$dataset/inputs,dst=/inputs,readonly")
if [[ ! -x "$work/install/capstone_localization/lib/capstone_localization/localization_metrics" ]]; then
  "${base[@]}" "$image" -ec '
    source /opt/ros/jazzy/setup.bash
    cd /work
    MAKEFLAGS=-j2 colcon --log-base log build --base-paths /workspace/ros2_ws/src/capstone_localization       --packages-select capstone_localization --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
  '
fi
if [[ "$mode" == replay ]]; then
  "${base[@]}" --mount "type=bind,src=$dataset/models,dst=/models,readonly" "$image" -ec '
    source /opt/ros/jazzy/setup.bash
    source /work/install/local_setup.bash
    python3 /workspace/ml/localization/course_evaluate.py infer --inputs /inputs --model /models       --predictions /work/predictions --executable /work/install/capstone_localization/lib/capstone_localization/localization_replay "$@"
  ' g1 "$@"
  predictions=/work/predictions
  archive=()
else
  predictions=/archived
  archive=(--mount "type=bind,src=$dataset/archived-estimates,dst=/archived,readonly")
fi
"${base[@]}" "${archive[@]}" --mount "type=bind,src=$dataset/truth,dst=/truth,readonly" "$image" -ec '
  source /opt/ros/jazzy/setup.bash
  python3 /workspace/ml/localization/course_evaluate.py score --inputs /inputs --truth /truth     --predictions "$1" --results /work/results --executable /work/install/capstone_localization/lib/capstone_localization/localization_metrics "${@:2}"
' g1 "$predictions" "$@"
