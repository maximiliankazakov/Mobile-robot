#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'USAGE'
G1 localization on held-out Webots recordings

  demo                 Recompute A and B on the slip demonstration (default)
  case EPISODE         Recompute the methods on a chosen episode
  scenario SCENARIO   Recompute a 20-episode scenario and its table
  all                  Recompute all 100 episodes and five tables
  list                 List scenarios and episode identifiers
  ros-smoke            Check the C++ node's ROS topics, TF and simulation clock

Set G1_OUTPUT_DIR=/out and mount a host folder at /out to retain the results.
USAGE
}

root=/opt/g1
dataset="$root/dataset"
bin="$root/install/capstone_localization/lib/capstone_localization"
mode="${1:-demo}"
if (( $# > 0 )); then
    shift
fi
args=()
case "$mode" in
    demo) args=(--case g1-test_0-slip-extra-00) ;;
    case|scenario)
        if (( $# != 1 )); then
            usage >&2
            exit 2
        fi
        args=("--$mode" "$1")
        shift
        ;;
    all) ;;
    list)
        if (( $# != 0 )); then
            usage >&2
            exit 2
        fi
        exec python3 -c 'import json, sys
with open(sys.argv[1], encoding="utf-8") as source:
    for case in json.load(source)["cases"]:
        print(case["scenario"], case["id"], sep="\t")' "$dataset/inputs/index.json"
        ;;
    ros-smoke)
        if (( $# != 0 )); then
            usage >&2
            exit 2
        fi
        exec env G1_INSTALL_PREFIX="$root/install" bash "$root/smoke/smoke.sh" real
        ;;
    -h|--help|help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
esac
if (( $# != 0 )); then
    usage >&2
    exit 2
fi

work="$(mktemp -d)"
trap 'rm -rf -- "$work"' EXIT
chmod 0755 "$work"
install -d -m 0755 -o nobody -g nogroup "$work/predictions"

printf 'G1 %s. Running C++ localization A, B and the ablation.\n' "$mode"
# The unprivileged inference process cannot traverse the ground-truth directory.
runuser -u nobody -- python3 "$root/course_evaluate.py" infer \
    --inputs "$dataset/inputs" --model "$dataset/models" \
    --predictions "$work/predictions" --executable "$bin/localization_replay" \
    --jobs 2 "${args[@]}"
python3 "$root/course_evaluate.py" score \
    --inputs "$dataset/inputs" --truth "$dataset/truth" \
    --predictions "$work/predictions" --results "$work/results" \
    --executable "$bin/localization_metrics" --jobs 2 "${args[@]}"

python3 - "$work/results" <<'PY'
import json
from pathlib import Path
import sys

result = Path(sys.argv[1])
summary = json.loads((result / 'summary.json').read_text())
if summary:
    print(f"{'Scenario':<16} {'Pairs':>5} {'A (mm)':>10} {'B (mm)':>10} {'B-A (mm)':>10}")
    for scenario, output in summary.items():
        stats = output['primary']
        print(f"{scenario:<16} {stats['pairs']:>5} {1000 * stats['A_mean_m']:>10.3f} "
              f"{1000 * stats['B_mean_m']:>10.3f} {1000 * stats['B_minus_A_m']:>+10.3f}")
else:
    for row in json.loads((result / 'per-episode.json').read_text()):
        for variant, methods in row['variants'].items():
            metric = 'ate_rmse_window_m' if 'gap' in row['variants'] else 'ate_rmse_m'
            a = 1000 * methods['A'][metric]
            b = 1000 * methods['B'][metric]
            print(f"{row['id']} [{variant}]: ATE A = {a:.3f} mm, B = {b:.3f} mm, B-A = {b-a:+.3f} mm")
PY

if [[ -n "${G1_OUTPUT_DIR:-}" ]]; then
    [[ -d "$G1_OUTPUT_DIR" ]] || { echo "Output directory must be mounted and exist: $G1_OUTPUT_DIR" >&2; exit 2; }
    destination="$(mktemp -d -- "$G1_OUTPUT_DIR/g1.XXXXXXXX")"
    chmod 0755 "$destination"
    cp -R -- "$work/results" "$work/predictions" "$destination/"
    printf 'Results: %s\n' "$destination"
fi
