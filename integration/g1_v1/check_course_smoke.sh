#!/usr/bin/env bash
# Run the public attached command and check its output and project cleanup.
set -euo pipefail
here="$(cd -- "$(dirname -- "$0")" && pwd)"
cd -- "$here/../.."
case "${1:-mock}" in
  mock) modes=(mock) ;;
  all) modes=(mock real replay) ;;
  *) echo "Usage: $0 [mock|all]" >&2; exit 2 ;;
esac
timeout -k 2s 10s docker info >/dev/null || {
  echo "G1 regression: Docker daemon unavailable" >&2
  exit 2
}
out="$(mktemp -d "${TMPDIR:-/tmp}/g1-attached-evidence.XXXXXXXX")"
suffix="${out##*.}"
export COMPOSE_PROJECT_NAME="g1-attached-${suffix,,}"
echo "G1 attached evidence: $out"
git rev-parse HEAD > "$out/head.txt"
git diff HEAD > "$out/working-tree.patch"
sha256sum "$here"/{course_smoke,smoke,replay_smoke,stop_children,check_course_smoke}.sh > "$out/scripts.sha256"
printf 'mode\tcommand_exit\tfunctional_pass\tnatural_exit\tcleanup_pass\n' > "$out/results.tsv"

cleanup_project() {
  local status=$?
  trap - EXIT
  trap '' INT TERM
  if ! timeout -k 4s 25s docker compose --profile dev down --remove-orphans --timeout 8 > "$out/down.log" 2>&1; then
    echo "G1 regression: project teardown failed. See $out/down.log" >&2
    if ((status == 0)); then status=1; fi
  fi
  if ! remaining="$(timeout -k 2s 8s docker ps -aq --filter "label=com.docker.compose.project=$COMPOSE_PROJECT_NAME")" ||
      [[ -n "$remaining" ]]; then
    echo "G1 regression: project container cleanup could not be verified" >&2
    if ((status == 0)); then status=1; fi
  fi
  exit "$status"
}
trap cleanup_project EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

for mode in "${modes[@]}"; do
  rc=0 pass=no natural=no clean=no
  log="$out/$mode.log"
  timeout -k 45s 850s bash "$here/course_smoke.sh" "$mode" </dev/null > "$log" 2>&1 || rc=$?
  tail -n 24 "$log"
  grep -q '^PASS G1 v1:' "$log" && pass=yes
  grep -qx 'G1 course smoke: attached_exit=0 container_state=exited 0 pid=0' "$log" && natural=yes
  if grep -q '^G1 course smoke: cleanup=yes run=' "$log" &&
     remaining="$(timeout -k 2s 8s docker ps -aq --filter "label=com.docker.compose.project=$COMPOSE_PROJECT_NAME")" &&
     [[ -z "$remaining" ]]; then
    clean=yes
  fi
  printf '%s\t%s\t%s\t%s\t%s\n' "$mode" "$rc" "$pass" "$natural" "$clean" | tee -a "$out/results.tsv"
  if ((rc == 124 || rc == 137)); then
    echo "$mode: timeout or forced termination. See $log" >&2
  fi
  if ((rc != 0)); then exit "$rc"; fi
  if [[ "$pass/$natural/$clean" != yes/yes/yes ]]; then
    echo "$mode: smoke acceptance criteria incomplete. See $log" >&2
    exit 1
  fi
done
