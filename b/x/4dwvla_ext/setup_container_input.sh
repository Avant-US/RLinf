#!/usr/bin/env bash
# Prepare keyboard/mouse nodes inside the Franky container so evdev programs
# can open a stable path.  Safe to run repeatedly after docker run, docker exec,
# or a USB unplug/replug.
#
# Inside the Franky container (recommended; also fixes this shell's env):
#   source /workspace/RLinf/b/x/4dwvla_ext/setup_container_input.sh
#
# Execute inside the container (nodes + watcher; does not change the parent env):
#   bash /workspace/RLinf/b/x/4dwvla_ext/setup_container_input.sh
#
# From the host (repairs the running container; cannot unset a stale export in
# an already-open interactive bash):
#   bash /home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/setup_container_input.sh
#
# Optional:
#   RLT_KEYBOARD_MATCH="Dell KB216"   default at this cell
#   RLT_MOUSE_MATCH=""                empty = unique mouse, else substring
#   CONTAINER_NAME=rlinf-4dwvla-franky
#   --restart-watch                   replace the existing --watch process

_setup_input_main() {
  local keyboard_match="${RLT_KEYBOARD_MATCH:-Dell KB216}"
  local mouse_match="${RLT_MOUSE_MATCH:-}"
  local container_name="${CONTAINER_NAME:-rlinf-4dwvla-franky}"
  local restart_watch=0
  local force_in_container=0
  local arg

  for arg in "$@"; do
    case "$arg" in
      --restart-watch) restart_watch=1 ;;
      --in-container) force_in_container=1 ;;
      -h|--help)
        sed -n '2,22p' "${_SETUP_INPUT_SELF}"
        return 0
        ;;
      *)
        echo "[setup-input] ERROR: unknown argument: $arg" >&2
        echo "[setup-input] usage: source setup_container_input.sh [--restart-watch]" >&2
        return 2
        ;;
    esac
  done

  if [[ "$force_in_container" -eq 0 ]] && ! _setup_input_in_container; then
    _setup_input_from_host "$container_name" "$restart_watch" "$keyboard_match" "$mouse_match"
    return $?
  fi

  _setup_input_in_franky "$restart_watch" "$keyboard_match" "$mouse_match"
}

_setup_input_in_container() {
  [[ -d /workspace/RLinf/b/x/4dwvla_ext ]] && [[ -d /sys/class/input ]]
}

_setup_input_from_host() {
  local container_name=$1
  local restart_watch=$2
  local keyboard_match=$3
  local mouse_match=$4
  local extra=()

  if ! command -v docker >/dev/null 2>&1; then
    echo "[setup-input] ERROR: not inside the Franky container, and docker is missing." >&2
    return 1
  fi
  if ! docker inspect -f '{{.State.Running}}' "$container_name" 2>/dev/null | grep -qx true; then
    echo "[setup-input] ERROR: container '$container_name' is not running." >&2
    echo "[setup-input] start it with docker_run_4dwvla_franky.sh, then run this script again." >&2
    return 1
  fi

  extra+=(--in-container)
  [[ "$restart_watch" -eq 1 ]] && extra+=(--restart-watch)

  echo "[setup-input] host -> docker exec -u 0 $container_name"
  echo "[setup-input] this cannot unset RLINF_KEYBOARD_DEVICE in an already-open container bash."
  echo "[setup-input] if that shell has a stale export, source this script there as well."

  RLT_KEYBOARD_MATCH="$keyboard_match" RLT_MOUSE_MATCH="$mouse_match" \
    docker exec -u 0 -e RLT_KEYBOARD_MATCH -e RLT_MOUSE_MATCH \
      "$container_name" \
      bash /workspace/RLinf/b/x/4dwvla_ext/setup_container_input.sh "${extra[@]}"
}

_setup_input_pick_python() {
  if [[ -x /opt/venv/franky-0.19.0/bin/python ]]; then
    printf '%s\n' /opt/venv/franky-0.19.0/bin/python
    return
  fi
  command -v python3
}

_setup_input_watch_running() {
  local pid_file=/run/rlt-input-sync/pid
  local pid=""
  if [[ -f "$pid_file" ]]; then
    pid=$(tr -d '[:space:]' <"$pid_file" || true)
  fi
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    if tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null | grep -q 'sync_input_devices.py'; then
      printf '%s\n' "$pid"
      return 0
    fi
  fi
  return 1
}

_setup_input_stop_watch() {
  local pid
  pid=$(_setup_input_watch_running || true)
  if [[ -n "$pid" ]]; then
    echo "[setup-input] stopping watcher pid $pid"
    kill "$pid" 2>/dev/null || true
    sleep 0.2
  fi
  rm -f /run/rlt-input-sync/pid
}

_setup_input_is_bad_keyboard_override() {
  local value=${1:-}
  [[ -z "$value" ]] && return 1
  [[ "$value" == /run/rlt-input/keyboard ]] && return 1
  [[ "$value" == /dev/input/by-id/*-event-kbd ]] && return 1
  return 0
}

_setup_input_verify_alias() {
  local role=$1
  local path=$2
  local required=$3
  local target=""

  if [[ ! -L "$path" ]]; then
    if [[ "$required" -eq 1 ]]; then
      echo "[setup-input] ERROR: missing $role alias $path" >&2
      return 1
    fi
    echo "[setup-input] WARN: no $role alias at $path"
    return 0
  fi
  target=$(readlink -f "$path" || true)
  if [[ -z "$target" || ! -e "$target" ]]; then
    echo "[setup-input] ERROR: $role alias $path is dangling" >&2
    return 1
  fi
  echo "[setup-input] $role alias: $path -> $target"
  return 0
}

_setup_input_try_evdev() {
  local python=$1
  local path=$2
  "$python" - "$path" <<'PY'
import sys
path = sys.argv[1]
try:
    from evdev import InputDevice
except ImportError:
    sys.exit(2)
try:
    dev = InputDevice(path)
except OSError as exc:
    print(f"[setup-input] ERROR: evdev cannot open {path}: {exc}", file=sys.stderr)
    sys.exit(1)
print(f"[setup-input] evdev ok: {path} name={dev.name!r} phys={dev.path!r}")
dev.close()
PY
}

_setup_input_in_franky() {
  local restart_watch=$1
  local keyboard_match=$2
  local mouse_match=$3
  local script_dir
  local sync_py
  local python
  local once_args=()
  local watch_args=()
  local pid=""
  local env_file=/run/rlt-input/env.sh
  local rc=0

  script_dir=$(cd "$(dirname "${_SETUP_INPUT_SELF}")" && pwd)
  sync_py="${script_dir}/sync_input_devices.py"
  if [[ ! -f "$sync_py" ]]; then
    echo "[setup-input] ERROR: missing $sync_py" >&2
    return 1
  fi

  python=$(_setup_input_pick_python) || {
    echo "[setup-input] ERROR: python3 not found" >&2
    return 1
  }

  mkdir -p /run/rlt-input /run/rlt-input-sync

  once_args=(--once --repair-nodes)
  watch_args=(--watch --repair-nodes)
  if [[ -n "$keyboard_match" ]]; then
    once_args+=(--keyboard-match "$keyboard_match")
    watch_args+=(--keyboard-match "$keyboard_match")
  fi
  if [[ -n "$mouse_match" ]]; then
    once_args+=(--mouse-match "$mouse_match")
    watch_args+=(--mouse-match "$mouse_match")
  fi

  echo "[setup-input] python=$python match=keyboard:${keyboard_match:-*} mouse:${mouse_match:-*}"
  echo "[setup-input] sync once..."
  if ! "$python" "$sync_py" "${once_args[@]}"; then
    echo "[setup-input] ERROR: sync --once failed" >&2
    return 1
  fi

  if [[ "$restart_watch" -eq 1 ]]; then
    _setup_input_stop_watch
  fi

  pid=$(_setup_input_watch_running || true)
  if [[ -n "$pid" ]]; then
    echo "[setup-input] watcher already running pid=$pid"
  else
    echo "[setup-input] starting watcher"
    nohup "$python" "$sync_py" "${watch_args[@]}" \
      >>/run/rlt-input-sync/sync.log 2>&1 &
    pid=$!
    echo "$pid" >/run/rlt-input-sync/pid
    disown "$pid" 2>/dev/null || true
    sleep 0.2
    if ! kill -0 "$pid" 2>/dev/null; then
      echo "[setup-input] ERROR: watcher exited immediately; see /run/rlt-input-sync/sync.log" >&2
      return 1
    fi
    echo "[setup-input] watcher pid=$pid"
  fi

  _setup_input_verify_alias keyboard /run/rlt-input/keyboard 1 || rc=1
  _setup_input_verify_alias mouse /run/rlt-input/mouse 0 || true

  if [[ -d /dev/input/by-id ]]; then
    echo "[setup-input] by-id keyboards:"
    ls -l /dev/input/by-id/*-event-kbd 2>/dev/null || echo "[setup-input]   (none)"
  fi

  _setup_input_try_evdev "$python" /run/rlt-input/keyboard
  case $? in
    0) ;;
    2) echo "[setup-input] WARN: python-evdev missing; symlink check only" ;;
    *) rc=1 ;;
  esac

  if [[ -L /run/rlt-input/mouse ]]; then
    _setup_input_try_evdev "$python" /run/rlt-input/mouse >/dev/null || true
  fi

  cat >"$env_file" <<'EOF'
# Generated by setup_container_input.sh. Source in the eval shell:
#   source /run/rlt-input/env.sh
unset RLINF_KEYBOARD_DEVICE
export RLINF_KEYBOARD_DEVICE=/run/rlt-input/keyboard
export RLINF_MOUSE_DEVICE=/run/rlt-input/mouse
EOF

  if [[ "${_SETUP_SOURCED}" -eq 1 ]]; then
    # shellcheck disable=SC1090
    source "$env_file"
    echo "[setup-input] sourced $env_file in this shell"
  else
    if _setup_input_is_bad_keyboard_override "${RLINF_KEYBOARD_DEVICE:-}"; then
      echo "[setup-input] WARN: this process inherited RLINF_KEYBOARD_DEVICE=${RLINF_KEYBOARD_DEVICE}" >&2
      echo "[setup-input] WARN: bash the script does not change your interactive shell." >&2
      echo "[setup-input] WARN: in the shell that will start the client, run:" >&2
      echo "[setup-input] WARN:   source /workspace/RLinf/b/x/4dwvla_ext/setup_container_input.sh" >&2
      echo "[setup-input] WARN: or: source /run/rlt-input/env.sh" >&2
    else
      echo "[setup-input] env file written: source $env_file"
    fi
  fi

  if command -v ss >/dev/null 2>&1; then
    if ss -ltnp 2>/dev/null | grep -q ':1337'; then
      echo "[setup-input] WARN: TCP 1337 is in use (Franka FCI). Do not start a second real-robot client."
      pgrep -af 'franka_vla_client|FrankyController' || true
    else
      echo "[setup-input] FCI port 1337 looks free"
    fi
  fi

  if [[ "$rc" -ne 0 ]]; then
    echo "[setup-input] FAILED. Do not start a real-robot client. See /run/rlt-input-sync/sync.log" >&2
    return 1
  fi
  echo "[setup-input] OK. Programs should open /run/rlt-input/keyboard and /run/rlt-input/mouse"
  echo "[setup-input] already-running evdev processes still hold old fds; restart those programs."
  return 0
}

_SETUP_INPUT_SELF=${BASH_SOURCE[0]:-$0}
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  _SETUP_SOURCED=1
  _setup_input_main "$@"
else
  _SETUP_SOURCED=0
  _setup_input_main "$@"
  exit $?
fi
