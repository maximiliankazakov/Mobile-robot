#!/usr/bin/env bash
set -euo pipefail
if (( $# > 1 )); then
    echo "Usage: $0 [g1_webots_demo_linux_amd64.tar.gz]" >&2
    exit 2
fi
image=g1-webots-demo:1.0
here="$(cd -- "$(dirname -- "$0")" && pwd)"
if ! docker info >/dev/null 2>&1; then
    socket="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/docker.sock"
    if [[ -S "$socket" && -z "${DOCKER_HOST:-}" && -z "${DOCKER_CONTEXT:-}" ]]; then
        export DOCKER_HOST="unix://$socket"
    fi
    docker info >/dev/null
fi
if ! docker image inspect "$image" >/dev/null 2>&1; then
    archive="${1:-$here/g1_webots_demo_linux_amd64.tar.gz}"
    docker load -i "$archive"
fi
if [[ ! "${DISPLAY:-}" =~ ^:[0-9]+(\.[0-9]+)?$ ]]; then
    echo 'Run from the Linux desktop in an X11 or XWayland session.' >&2
    exit 2
fi
if ! command -v xauth >/dev/null; then
    echo 'Missing xauth. Install the xauth package on the Linux desktop.' >&2
    exit 2
fi
if pgrep -x webots-bin >/dev/null; then
    echo 'A Webots scene is already running. Close it before starting the demo.' >&2
    exit 3
fi
private="$(mktemp -d)"
cleanup() {
    if [[ -f "$private/xauth" ]]; then unlink "$private/xauth"; fi
    rmdir "$private"
}
trap cleanup EXIT
: > "$private/xauth"
xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$private/xauth" nmerge -
if [[ ! -s "$private/xauth" ]]; then
    echo 'No X11 authorization for this display. Start from your graphical session.' >&2
    exit 2
fi
chmod 644 "$private/xauth"
rootless=$(docker info --format '{{range .SecurityOptions}}{{println .}}{{end}}')
flags=()
if [[ "$rootless" == *name=rootless* ]]; then
    flags+=(--user 0:0)
fi
shopt -s nullglob
render_nodes=(/dev/dri/renderD*)
if [[ "${G1_SOFTWARE:-0}" != 1 && ${#render_nodes[@]} -gt 0 ]]; then
    flags+=(--mount 'type=bind,source=/dev/dri,target=/dev/dri,readonly')
    if [[ "$rootless" != *name=rootless* ]]; then
        flags+=(--group-add "$(stat -c %g "${render_nodes[0]}")")
    fi
else
    flags+=(-e LIBGL_ALWAYS_SOFTWARE=1)
fi
if [[ -n "${G1_OUTPUT_DIR:-}" ]]; then
    output="$(cd -- "$G1_OUTPUT_DIR" && pwd)"
    flags+=(--mount "type=bind,source=$output,target=/g1-output" -e G1_OUTPUT_PARENT=/g1-output)
fi
docker run --rm --init --pull=never --network none --shm-size 1g \
    --name g1-webots-demo -e "DISPLAY=$DISPLAY" -e XAUTHORITY=/g1-xauth \
    --mount 'type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix,readonly' \
    --mount "type=bind,source=$private/xauth,target=/g1-xauth,readonly" \
    "${flags[@]}" "$image"
