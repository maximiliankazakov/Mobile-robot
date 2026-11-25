#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "$0")/../../.." && pwd)"
if (( $# != 1 )); then
    echo "Usage: $0 /path/to/extracted/bundle" >&2
    exit 2
fi
dataset="$(cd -- "$1" && pwd)"
base="${G1_SIM_IMAGE:-capstone-g1/webots-rootless:local}"
if ! docker image inspect "$base" >/dev/null 2>&1; then
    docker build --platform linux/amd64 --target simulator -f "$root/docker/Dockerfile" -t "$base" "$root"
fi
exec docker build --platform linux/amd64 --build-context "dataset=$dataset" \
    --build-arg "SIM_IMAGE=$base" --build-arg "G1_SOURCE_REVISION=$(git -C "$root" rev-parse HEAD)" \
    -f "$root/integration/g1_v1/webots-demo/Dockerfile" -t g1-webots-demo:1.0 "$root"
