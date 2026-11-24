#!/usr/bin/env bash
# Source only; receives only PIDs started directly by the caller.
g1_stop_children() {
  local p round any rc=0
  local -a children=()
  for p in "$@"; do
    [[ -n "$p" ]] || continue
    [[ "$p" =~ ^[1-9][0-9]*$ ]] && (( p > 1 )) || return 1
    children+=("$p")
  done
  if (( ${#children[@]} == 0 )); then return 0; fi
  for p in "${children[@]}"; do kill -TERM "$p" 2>/dev/null || true; done
  for round in {1..30}; do
    any=0
    for p in "${children[@]}"; do if kill -0 "$p" 2>/dev/null; then any=1; fi; done
    ((any)) || break
    sleep .1
  done
  for p in "${children[@]}"; do
    if kill -0 "$p" 2>/dev/null; then
      echo "G1 cleanup: PID $p ignored/overran TERM; sending KILL" >&2
      kill -KILL "$p" 2>/dev/null || true
    fi
  done
  for round in {1..10}; do
    any=0
    for p in "${children[@]}"; do if kill -0 "$p" 2>/dev/null; then any=1; fi; done
    ((any)) || break
    sleep .1
  done
  for p in "${children[@]}"; do
    if kill -0 "$p" 2>/dev/null; then
      echo "G1 cleanup: PID $p did not exit after KILL" >&2
      rc=1
    else
      wait "$p" 2>/dev/null || true
    fi
  done
  return "$rc"
}
