#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "$0")/../../.." && pwd)"
if (( $# != 1 )); then
    echo "Usage: $0 /path/to/extracted/bundle" >&2
    exit 2
fi
dataset="$(cd -- "$1" && pwd)"
if [[ ! -f "$dataset/inputs/index.json" || ! -f "$dataset/verification/release.json" ]]; then
    echo "Pass the bundle directory from g1_hw1_replay.tar.zst" >&2
    exit 2
fi
base_args=()
if [[ -n "${G1_ROS_BASE:-}" ]]; then
    base_args=(--build-arg "G1_ROS_BASE=$G1_ROS_BASE")
fi
revision="$(git -C "$root" rev-parse HEAD)"
exec docker build --platform linux/amd64 \
    --build-context "dataset=$dataset" \
    --build-arg "G1_SOURCE_REVISION=$revision" \
    "${base_args[@]}" \
    --file "$root/integration/g1_v1/portable/Dockerfile" \
    --tag "${G1_IMAGE:-g1-localization:hw1-v1}" "$root"
