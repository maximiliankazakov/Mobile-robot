#!/usr/bin/env bash
# From the course checkout, use the same Ubuntu 24.04 / Jazzy dev image as CI.
set -eo pipefail
here="$(cd -- "$(dirname -- "$0")" && pwd)"
cd -- "$here/../.."
mode="${1:-mock}"
case "$mode" in
  mock) run_limit=45s ;;
  real) run_limit=150s ;;
  replay) run_limit=65s ;;
  *) echo "Usage: $0 [mock|real|replay]" >&2; exit 2 ;;
esac
if ! timeout -k 2s 10s docker compose version >/dev/null 2>&1; then
  echo "Course smoke needs Docker Compose. Nothing was installed or changed." >&2
  exit 2
fi
if ! timeout -k 2s 10s docker info >/dev/null; then
  echo "G1 course smoke: Docker daemon unavailable (preflight)" >&2
  exit 2
fi
export HOST_UID="$(id -u)" HOST_GID="$(id -g)"
# Keep this one-off container until its terminal state has been checked.
# Match cleanup by a per-run label so unrelated Compose jobs are never removed.
printf -v run_id 'g1-course-%s-%s-%s%s' "$mode" "$$" "$RANDOM" "$RANDOM"
label="capstone.g1.course-run=$run_id"
cleanup() {
  local status=$? ids remaining id cleanup_ok=yes
  local -a rm_args=()
  trap - EXIT
  trap '' INT TERM
  if ((status != 0)); then rm_args=(--force); fi
  if ids="$(timeout -k 2s 8s docker ps -aq --filter "label=$label")"; then
    while IFS= read -r id; do
      [[ -n "$id" ]] || continue
      timeout -k 2s 10s docker rm "${rm_args[@]}" "$id" || cleanup_ok=no
    done <<< "$ids"
  else
    cleanup_ok=no
  fi
  if ! remaining="$(timeout -k 2s 8s docker ps -aq --filter "label=$label")" || [[ -n "$remaining" ]]; then
    cleanup_ok=no
  fi
  echo "G1 course smoke: cleanup=$cleanup_ok run=$run_id"
  if [[ "$cleanup_ok" == no ]] && ((status == 0)); then status=1; fi
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

build_status=0
timeout -k 10s 600s docker compose --profile dev build dev </dev/null || build_status=$?
if ((build_status != 0)); then
  echo "G1 course smoke: dev build exit=$build_status" >&2
  exit "$build_status"
fi
run_status=0
# Remain attached to stdout and stderr, with no stdin stream or TTY.
timeout -k 8s "$run_limit" docker compose --profile dev run \
  --no-deps --interactive=false -T --name "$run_id" --label "$label" -w /workspace \
  -e G1_ROS_DISTRO=jazzy -e G1_INSTALL_PREFIX= -e PYTHONNOUSERSITE=1 \
  dev bash --noprofile --norc -c '
    set -eo pipefail
    source /etc/os-release
    test "$ID" = ubuntu && test "$VERSION_ID" = 24.04 || exit 2
    source /opt/ros/jazzy/setup.bash
    dir=/workspace/integration/g1_v1
    case "$1" in
      mock) exec bash "$dir/smoke.sh" mock ;;
      real)
        stage="$(mktemp -d /tmp/g1-course-build.XXXXXXXX)"
        MAKEFLAGS=-j2 colcon --log-base "$stage/log" build \
          --base-paths ros2_ws/src/capstone_localization \
          --build-base "$stage/build" --install-base "$stage/install" \
          --packages-select capstone_localization --executor sequential
        G1_INSTALL_PREFIX="$stage/install" exec bash "$dir/smoke.sh" real
        ;;
      replay) exec bash "$dir/replay_smoke.sh" ;;
    esac
  ' _ "$mode" </dev/null || run_status=$?
container_state="$(timeout -k 2s 8s docker inspect --type container \
  --format '{{.State.Status}} {{.State.ExitCode}} pid={{.State.Pid}}' "$run_id")" || container_state=unavailable
echo "G1 course smoke: attached_exit=$run_status container_state=$container_state"
if ((run_status != 0)); then
  if ((run_status == 124 || run_status == 137)); then
    echo "G1 course smoke: attached run hit its wall-time limit ($run_limit) or was killed" >&2
  fi
  if [[ "$container_state" != unavailable ]]; then
    echo "G1 course smoke: last container logs before cleanup" >&2
    timeout -k 2s 8s docker logs --tail 40 "$run_id" >&2 || true
  fi
  exit "$run_status"
fi
if [[ "$container_state" != "exited 0 pid=0" ]]; then
  echo "G1 course smoke: missing successful container exit" >&2
  exit 1
fi
