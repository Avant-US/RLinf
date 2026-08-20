#!/bin/bash
# Host-side pre-training gate for dmo_place_2.md S3.5 ("第 0 步").
#
# Run on the HOST before `run_cube_place_sac.sh`. Three layers, cheapest first;
# if a layer still fails AFTER its auto-fix attempts, later (more expensive /
# robot-touching) layers are skipped:
#
#   A. static YAML/file checks (host python3 + PyYAML only, no containers).
#      Auto-fixable: H1 -> YAML pose sync, RLINF_SKIP_CAMERA pin, camera serial
#      intra-YAML sync (hardware.configs is the anchor -> train/eval override +
#      camera_names; camera_detected.json may list EXTRA plugged-in cameras,
#      they are ignored by design), eval is_dummy, save_interval,
#      enable_camera_player. NOT auto-fixable: H1 calibration itself and the
#      cube width (both are physical measurements; a wrong auto value is a
#      safety hazard, not a convenience).
#   B. cluster: delegates to verify_ray_cluster.sh (14 checks, LOG-029).
#      Auto-fixable: start stopped containers, then a full ray restart of both
#      nodes in the S3.2 order (rank env exported BEFORE ray start). NOT
#      auto-fixable: FCI port busy, missing ResNet10 weights, broken venvs,
#      wrong python_interpreter_path (EDIT ME).
#   C. robot pre-flight (read-only, does NOT move the arm). NOT auto-fixable:
#      every check here involves the physical world (a process holding the
#      arm, arm pose, gripper holding) -- a human must do it, S3.4 steps 1-4.
#
# Behavior on FAIL (LOG-032): the failure is framed in a ***** banner with the
# doc pointer, an auto-fix is attempted if one exists, the check is re-run and
# reported as AUTO-FIXED or left FAIL with MANUAL ACTION REQUIRED. YAML edits
# keep a one-shot backup at realworld_cube_place_sac.yaml.preflight-bak.
#
# Usage:
#   bash b/x/scripts/preflight_cube_place_sac.sh                # full, incl. robot
#   bash b/x/scripts/preflight_cube_place_sac.sh --skip-robot   # A+B only, no FCI
#
# What this script deliberately does NOT check (human-only gates, see S3.5):
# 2.4b peak_overshoot data, whether the mark/cube moved since H1, and a person
# standing at the e-stop.
set -uo pipefail

REPO="${REPO_PATH:-/home/nvidia/bt/s/RLinf}"
SAC_YAML="${REPO}/b/x/configs/realworld_cube_place_sac.yaml"
H1_YAML="${REPO}/b/x/configs/cube_place_target_ee_pose.yaml"
CAMERA_JSON="${REPO}/b/x/configs/camera_detected.json"
VERIFY_SCRIPT="${REPO}/b/x/scripts/verify_ray_cluster.sh"
FRANKY_CONTAINER="${RLINF_FRANKA_CONTAINER:-rlinf-franky-5090}"
GPU_CONTAINER="${RLINF_GPU_CONTAINER:-rlinf-gpu-5090}"
export REPO_PATH="${REPO}" SAC_YAML H1_YAML CAMERA_JSON

SKIP_ROBOT=0
for arg in "$@"; do
  case "${arg}" in
    --skip-robot) SKIP_ROBOT=1 ;;
    -h|--help)
      sed -n '2,36p' "$0"
      exit 0
      ;;
    *)
      echo "unknown argument: ${arg} (only --skip-robot is supported)" >&2
      exit 2
      ;;
  esac
done

FAILED=0
STARS="*****************************************************************************"
banner() { # banner <name> <detail> <manual-pointer> <autofix-note>
  echo "${STARS}"
  echo "***** FAIL $1"
  [[ -n "${2:-}" ]] && echo "*****   detail: $2"
  [[ -n "${4:-}" ]] && echo "*****   auto-fix: $4"
  [[ -n "${3:-}" ]] && echo "*****   manual: $3"
  echo "${STARS}"
}
check() { # check <name> <ok:0/1> [detail] [AUTO-FIXED]
  local status="OK"
  [[ "$2" == "0" ]] || { status="FAIL"; FAILED=1; }
  [[ -n "${4:-}" ]] && status="${4}"
  if [[ -n "${3:-}" ]]; then
    echo "CHECK $1 ${status}  $3"
  else
    echo "CHECK $1 ${status}"
  fi
}

# ---------------------------------------------------------------- layer A ----
echo "== layer A: static YAML / file checks (host; auto-fix edits ${SAC_YAML##*/}) =="
layer_a_out="$(python3 - <<'PY'
import json
import os
import re
import shutil
import sys

import yaml

repo = os.environ.get("REPO_PATH", "/home/nvidia/bt/s/RLinf")
sys.path.insert(0, os.path.join(repo, "b", "x", "scripts"))
from step8_checks import is_placeholder_serial

sac_path = os.environ["SAC_YAML"]
h1_path = os.environ["H1_YAML"]
cam_path = os.environ["CAMERA_JSON"]

STARS = "*" * 77
state = {"text": open(sac_path, encoding="utf-8").read(), "sac": None, "backed_up": False}
n_fail = 0


def reload():
    state["sac"] = yaml.safe_load(state["text"])


def flush():
    if not state["backed_up"]:
        shutil.copy2(sac_path, sac_path + ".preflight-bak")
        state["backed_up"] = True
    with open(sac_path, "w", encoding="utf-8") as f:
        f.write(state["text"])
    reload()


def banner(name, detail, manual, autofix):
    print(STARS)
    print(f"***** FAIL {name}")
    if detail:
        print(f"*****   detail: {detail}")
    if autofix:
        print(f"*****   auto-fix: {autofix}")
    if manual:
        print(f"*****   manual: {manual}")
    print(STARS)


def record(name, status, detail=""):
    global n_fail
    if status == "FAIL":
        n_fail += 1
    print(f"CHECK {name} {status}" + (f"  {detail}" if detail else ""))


def run_check(name, evaluate, fixer=None, manual="", nofix_reason=""):
    """evaluate() -> (ok, detail); fixer() -> None (mutates state['text'])."""
    ok, detail = evaluate()
    if ok:
        record(name, "OK", detail)
        return
    if fixer is None:
        banner(name, detail, manual, f"not possible ({nofix_reason}) -- MANUAL ACTION REQUIRED")
        record(name, "FAIL", detail)
        return
    banner(name, detail, manual, "attempting ...")
    try:
        fixer()
        flush()
    except Exception as exc:  # a broken auto-fix must not hide the original failure
        print(f"*****   auto-fix raised: {exc!r} -- MANUAL ACTION REQUIRED")
        record(name, "FAIL", detail)
        return
    ok2, detail2 = evaluate()
    if ok2:
        record(name, "AUTO-FIXED", detail2)
    else:
        print("*****   auto-fix applied but the check still fails -- MANUAL ACTION REQUIRED")
        record(name, "FAIL", detail2)


reload()
sac = state["sac"]
h1 = yaml.safe_load(open(h1_path, encoding="utf-8")) or {}
pose = h1.get("target_ee_pose")
h1_ok = (
    bool(h1.get("calibrated"))
    and isinstance(pose, list)
    and len(pose) == 6
    and not all(abs(float(v)) < 1e-8 for v in pose)
)

# 1. H1 calibrated and non-zero -- physical measurement, never auto-fixed.
run_check(
    "h1_calibrated",
    lambda: (
        h1_ok,
        f"calibrated={h1.get('calibrated')} pose={pose}",
    ),
    fixer=None,
    manual="see dmo_place_2.md S2.5: redo H1 calibration (REPL getpos_euler with "
    "the cube touching the mark, then write_cube_place_pose.py)",
    nofix_reason="H1 is a physical robot measurement",
)

# 2. H1 identical to the training YAML (train and eval) -- auto-fix: sync YAML
# from the H1 file (H1 is written by the physical calibration, so it is the
# source of truth).
def eval_h1_match():
    if not h1_ok:
        return True, "skipped (H1 itself failed)"
    bad = []
    for split in ("train", "eval"):
        yaml_pose = (
            ((state["sac"].get("env") or {}).get(split) or {}).get("override_cfg") or {}
        ).get("target_ee_pose")
        same = isinstance(yaml_pose, list) and len(yaml_pose) == 6 and all(
            abs(float(a) - float(b)) < 1e-9 for a, b in zip(pose, yaml_pose)
        )
        if not same:
            bad.append(f"{split}={yaml_pose}")
    if bad:
        return False, f"mismatch: {'; '.join(bad)} (H1={pose})"
    return True, f"train/eval == H1 {pose}"


def fix_h1_match():
    block = "target_ee_pose:\n        [\n" + "".join(
        f"          {v},\n" for v in pose
    ) + "        ]"
    new, n = re.subn(r"target_ee_pose:\s*\[[^\]]*\]", block, state["text"])
    if n != 2:
        raise RuntimeError(f"expected 2 target_ee_pose blocks, found {n}")
    state["text"] = new


run_check(
    "h1_matches_sac_yaml",
    eval_h1_match,
    fixer=fix_h1_match if h1_ok else None,
    manual="see dmo_place_2.md S3.1 header note: paste the current H1 six-tuple "
    "into realworld_cube_place_sac.yaml (EDIT ME), cross-check with "
    "`run_cube_place_phase2.sh connect`",
    nofix_reason="H1 itself is not calibrated",
)


def franky_env_vars():
    env_vars = {}
    for group in (state["sac"].get("cluster") or {}).get("node_groups") or []:
        if group.get("label") != "franky":
            continue
        for ec in group.get("env_configs") or []:
            for item in ec.get("env_vars") or []:
                env_vars.update({str(k): str(v) for k, v in dict(item).items()})
    return env_vars


# 3. FRANKA_CUBE_WIDTH_M pinned and not the upstream default -- physical
# measurement, never auto-fixed (a wrong width breaks the holding check).
def eval_cube_width():
    width = franky_env_vars().get("FRANKA_CUBE_WIDTH_M")
    ok = width is not None and abs(float(width) - 0.046) > 1e-9
    return ok, f"FRANKA_CUBE_WIDTH_M={width}"


run_check(
    "cube_width_pinned",
    eval_cube_width,
    fixer=None,
    manual="see dmo_place_2.md S2.5 step 1: 0.046 is the upstream default, not "
    "any real cube (LOG-024); measure with the REPL (`close` prints measured "
    "width) and pin the measured value",
    nofix_reason="cube width is a physical measurement; a guessed value would "
    "silently break the gripper holding check",
)

# 4. RLINF_SKIP_CAMERA pinned to "0" -- auto-fix: set/insert the env_vars entry.
def eval_skip_camera():
    skip = franky_env_vars().get("RLINF_SKIP_CAMERA")
    return skip == "0", f"RLINF_SKIP_CAMERA={skip}"


def fix_skip_camera():
    new, n = re.subn(
        r'^(\s*)- RLINF_SKIP_CAMERA:.*$',
        r'\1- RLINF_SKIP_CAMERA: "0"',
        state["text"],
        flags=re.M,
    )
    if n == 0:  # key absent: insert right before the cube-width entry
        new, n = re.subn(
            r'^(\s*)- FRANKA_CUBE_WIDTH_M:.*$',
            r'\1- RLINF_SKIP_CAMERA: "0"\n\g<0>',
            state["text"],
            count=1,
            flags=re.M,
        )
    if n == 0:
        raise RuntimeError("no env_vars anchor line found to insert into")
    state["text"] = new


run_check(
    "skip_camera_pinned",
    eval_skip_camera,
    fixer=fix_skip_camera,
    manual="see dmo_place_2.md S3.10 item 1: pin RLINF_SKIP_CAMERA: \"0\" in the "
    "franky group env_configs.env_vars; anything else trains on black stub "
    "frames without error",
)

# 5. camera serials: the TRAINING YAML is the anchor (see the LOG entry "第二只
# 相机插着但不配"): camera_detected.json lists every camera physically plugged
# in, and extras are IGNORED (run_cube_place_camera_accept.sh whitelists from
# the YAML). FAIL only when the YAML's own serials are empty/placeholder,
# inconsistent across hardware/train/eval/camera_names, or not physically
# detected on USB.
def eval_camera_serials():
    if not os.path.isfile(cam_path):
        return False, f"{cam_path} missing", None
    det = json.load(open(cam_path, encoding="utf-8"))
    det_serials = [str(s) for s in det.get("camera_serials") or []]
    hw: list = []
    for group in (state["sac"].get("cluster") or {}).get("node_groups") or []:
        if group.get("label") != "franky":
            continue
        for cfg in ((group.get("hardware") or {}).get("configs")) or []:
            hw += [str(s) for s in cfg.get("camera_serials") or []]
    ovs, names = {}, {}
    for split in ("train", "eval"):
        ov = ((state["sac"].get("env") or {}).get(split) or {}).get("override_cfg") or {}
        ovs[split] = [str(s) for s in ov.get("camera_serials") or []]
        names[split] = [str(k) for k in (ov.get("camera_names") or {}).keys()]
    problems = []
    if not hw:
        problems.append("hardware camera_serials empty")
    bad_ph = [s for s in hw + ovs["train"] + ovs["eval"] if is_placeholder_serial(s)]
    if bad_ph:
        problems.append(f"placeholder serials: {bad_ph}")
    if hw and (ovs["train"] != hw or ovs["eval"] != hw):
        problems.append(f"override train={ovs['train']} eval={ovs['eval']} != hardware={hw}")
    if hw and (names["train"] != hw or names["eval"] != hw):
        problems.append(
            f"camera_names keys train={names['train']} eval={names['eval']} != hardware={hw}"
        )
    missing = [s for s in hw if s not in det_serials]
    if missing:
        problems.append(f"YAML serials NOT detected on USB: {missing}")
    extras = [s for s in det_serials if s not in hw]
    detail = f"yaml={hw} detected={det_serials}"
    if extras:
        detail += f" (extras plugged but ignored: {extras})"
    if problems:
        detail += "; " + "; ".join(problems)
    fix = None
    if (not hw or bad_ph) and len(det_serials) == 1:
        fix = list(det_serials)  # exactly one camera plugged in -> adopt it
    elif hw and not bad_ph and not missing and (
        ovs["train"] != hw or ovs["eval"] != hw
        or names["train"] != hw or names["eval"] != hw
    ):
        fix = list(hw)  # hardware anchor -> sync overrides + camera_names
    return (not problems), detail, fix


def fix_camera_serials(target):
    def _fix():
        state["text"] = re.sub(
            r"camera_serials:\s*\[[^\]]*\]",
            f"camera_serials: {json.dumps(target)}",
            state["text"],
        )
        # camera_names keys: wrist_i maps to target[i-1] (the YAML convention
        # is camera_names: {"<serials[i]>": wrist_{i+1}})

        def _name_sub(m):
            i = int(m.group(3)) - 1
            if i < 0 or i >= len(target):
                return m.group(0)
            return f'{m.group(1)}"{target[i]}": {m.group(2)}{m.group(3)}'

        state["text"] = re.sub(
            r'^(\s*)"[^"]*":\s*(wrist_)(\d+)[ \t]*$',
            _name_sub,
            state["text"],
            flags=re.M,
        )

    return _fix


cam_ok, cam_detail, cam_fix = eval_camera_serials()
run_check(
    "camera_serials_match",
    lambda: eval_camera_serials()[:2],
    fixer=(fix_camera_serials(cam_fix) if not cam_ok and cam_fix else None),
    manual="see dmo_place_2.md S3.3 (serial anchor = the training YAML): if a "
    "YAML serial is NOT detected, plug that camera in and re-run 8a; if the "
    "YAML is empty/placeholder while MULTIPLE cameras are plugged in, pick the "
    "wrist camera's serial by hand and paste it into hardware.configs + both "
    "override_cfg (EDIT ME) -- which one is the wrist camera is a physical "
    "question; extra plugged-in cameras need no action (ignored by design)",
    nofix_reason="a YAML serial is not physically detected (plugging it in is "
    "physical), or several cameras are plugged in and none is configured "
    "(which one is the wrist camera is a physical question)",
)

# 6. eval not dummy (S3.10 item 6) -- auto-fix: is_dummy True -> False.
def eval_not_dummy():
    d = bool(
        (((state["sac"].get("env") or {}).get("eval") or {}).get("override_cfg") or {}).get(
            "is_dummy", True
        )
    )
    return (not d), f"env.eval is_dummy={d}"


def fix_is_dummy():
    state["text"] = re.sub(
        r"^(\s*)is_dummy:\s*True", r"\1is_dummy: False", state["text"], flags=re.M
    )


run_check(
    "eval_not_dummy",
    eval_not_dummy,
    fixer=fix_is_dummy,
    manual="see dmo_place_2.md S3.10 item 6: copy train's override_cfg to eval, "
    "or keep val_check_interval=-1",
)

# 7. save_interval set (charger's -1 loses everything on e-stop) -- auto-fix to 50.
def eval_save_interval():
    si = (state["sac"].get("runner") or {}).get("save_interval")
    return (si is not None and int(si) > 0), f"save_interval={si}"


def fix_save_interval():
    state["text"] = re.sub(
        r"^(\s*)save_interval:\s*\S+",
        r"\1save_interval: 50",
        state["text"],
        count=1,
        flags=re.M,
    )


run_check(
    "save_interval_set",
    eval_save_interval,
    fixer=fix_save_interval,
    manual="see dmo_place_2.md S3.0 table: charger's -1 only saves on clean "
    "exit; a real robot gets e-stopped",
)

# 8. camera player off (headless containers) -- auto-fix True -> False.
def eval_camera_player():
    bad = []
    for split in ("train", "eval"):
        p = bool(
            (((state["sac"].get("env") or {}).get(split) or {}).get("override_cfg") or {}).get(
                "enable_camera_player", True
            )
        )
        if p:
            bad.append(split)
    return (not bad), f"enable_camera_player True in: {bad or 'none'}"


def fix_camera_player():
    state["text"] = re.sub(
        r"^(\s*)enable_camera_player:\s*True",
        r"\1enable_camera_player: False",
        state["text"],
        flags=re.M,
    )


run_check(
    "camera_player_off",
    eval_camera_player,
    fixer=fix_camera_player,
    manual="see dmo_place_2.md S3.1 item 1: the containers are headless; set "
    "enable_camera_player: false",
)

sys.exit(1 if n_fail else 0)
PY
)"
layer_a_rc=$?
echo "${layer_a_out}"
if [[ ${layer_a_rc} -ne 0 ]]; then
  echo "layer A still has FAIL after auto-fix attempts; skipping layers B and C (fix the static config first)."
  echo
  echo "RESULT preflight FAIL"
  exit 1
fi

# ---------------------------------------------------------------- layer B ----
echo
echo "== layer B: ray cluster (delegates to verify_ray_cluster.sh) =="
verify_out="$(bash "${VERIFY_SCRIPT}" 2>&1)"
verify_rc=$?
echo "${verify_out}"
if [[ ${verify_rc} -ne 0 ]]; then
  failed_names="$(grep -oE '^CHECK [a-z0-9_]+ FAIL' <<<"${verify_out}" | awk '{print $2}' | tr '\n' ' ')"
  autofixable=1
  for n in ${failed_names}; do
    case "${n}" in
      container_*|raylet_*|ray_status_*|e2e_pinned_tasks|gym_id_*) ;;
      *) autofixable=0 ;;
    esac
  done
  if [[ "${autofixable}" == "1" ]]; then
    banner "cluster_verify" "failed checks: ${failed_names}" \
      "if the auto-fix below does not stick, see dmo_place_2.md S3.2 (restart the two-container cluster in the documented order) / S3.1 (EDIT ME values)" \
      "attempting: docker start stopped containers, then full ray restart of both nodes in the S3.2 order"
    for c in "${FRANKY_CONTAINER}" "${GPU_CONTAINER}"; do
      if ! docker ps --format '{{.Names}}' | grep -qx "${c}"; then
        echo "auto-fix: docker start ${c}"
        docker start "${c}"
      fi
    done
    sleep 3
    host_ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
    echo "auto-fix: ray stop on both nodes, then ray start (head rank 0 -> worker rank 1), host IP ${host_ip}"
    docker exec "${GPU_CONTAINER}" bash -lc \
      'source b/x/configs/setup_before_ray_gpu_5090.sh >/dev/null 2>&1; ray stop --force >/dev/null 2>&1' || true
    docker exec "${FRANKY_CONTAINER}" bash -lc \
      'source b/x/configs/setup_before_ray_5090.sh >/dev/null 2>&1; ray stop --force >/dev/null 2>&1' || true
    sleep 3
    docker exec "${GPU_CONTAINER}" bash -lc \
      "source b/x/configs/setup_before_ray_gpu_5090.sh >/dev/null 2>&1; export RLINF_NODE_RANK=0; ray start --head --port=6379 --node-ip-address=${host_ip} --disable-usage-stats"
    sleep 5
    docker exec "${FRANKY_CONTAINER}" bash -lc \
      "source b/x/configs/setup_before_ray_5090.sh >/dev/null 2>&1; export RLINF_NODE_RANK=1 ROBOT_IP=172.16.0.2; ray start --address=${host_ip}:6379"
    sleep 5
    echo "auto-fix: re-running verify_ray_cluster.sh"
    verify_out="$(bash "${VERIFY_SCRIPT}" 2>&1)"
    verify_rc=$?
    echo "${verify_out}"
    if [[ ${verify_rc} -eq 0 ]]; then
      check "cluster_verify" 0 "verify_ray_cluster.sh PASS after auto-fix" "AUTO-FIXED"
    else
      echo "*****   auto-fix applied but the cluster still fails -- MANUAL ACTION REQUIRED (see dmo_place_2.md S3.2)"
      check "cluster_verify" 1 "failed checks after auto-fix: $(grep -oE '^CHECK [a-z0-9_]+ FAIL' <<<"${verify_out}" | awk '{print $2}' | tr '\n' ' ')"
    fi
  else
    banner "cluster_verify" "failed checks: ${failed_names}" \
      "see dmo_place_2.md: fci_free -> S6 'Couldn't connect' (stop whatever holds :1337); resnet10_weights -> download the weights per S3.1; interpreter_* / franky_import / gpu_cuda_torch -> S3.1 EDIT ME / rebuild the image venv" \
      "not possible for these checks (not a cluster-start ordering issue) -- MANUAL ACTION REQUIRED"
    check "cluster_verify" 1 "failed checks: ${failed_names}"
  fi
  if [[ "${FAILED}" != "0" ]]; then
    echo
    echo "layer B failed; skipping layer C (robot checks need the cluster up)."
    echo
    echo "RESULT preflight FAIL"
    exit 1
  fi
else
  check "cluster_verify" 0 "verify_ray_cluster.sh PASS"
fi

# ---------------------------------------------------------------- layer C ----
if [[ "${SKIP_ROBOT}" == "1" ]]; then
  echo
  echo "== layer C: SKIPPED (--skip-robot); the robot gates in dmo_place_2.md S3.4 steps 1-3 are NOT verified =="
else
  echo
  echo "== layer C: robot pre-flight (read-only; arm does not move; no auto-fix -- the physical world needs a human) =="

  # 9. no live controller actor
  actors="$(docker exec "${GPU_CONTAINER}" bash -lc \
    'source b/x/configs/setup_before_ray_gpu_5090.sh >/dev/null 2>&1; ray list actors --filter "class_name=FrankyControllerExtended" --filter "state=ALIVE" 2>/dev/null' || true)"
  if grep -q ALIVE <<<"${actors}"; then
    banner "no_live_controller" "a live FrankyControllerExtended actor holds the robot" \
      "see dmo_place_2.md S3.4 step 4: stop the owning process cleanly (Ctrl+C / let it finish), confirm the arm is still, then re-run" \
      "refused: killing a process that holds the arm is a human decision"
    check "no_live_controller" 1
  else
    check "no_live_controller" 0
  fi

  # 10. FCI port free
  if ss -tn state established '( dport = :1337 or sport = :1337 )' 2>/dev/null | grep -q 1337; then
    banner "fci_free" "something holds :1337" \
      "see dmo_place_2.md S6 'Couldn't connect': stop all python, REPL must exit with q, ray stop where needed" \
      "refused: killing the holder of :1337 may drop the arm's connection mid-motion"
    check "fci_free" 1
  else
    check "fci_free" 0
  fi

  # 11+12. connect pre-flight inside the franky container (mode / start pose /
  # gripper gates live in step_cube_place_robot.py --connect-only), plus the
  # authority echo from its output.
  connect_out="$(docker exec "${FRANKY_CONTAINER}" bash -lc \
    'source b/x/configs/setup_before_ray_5090.sh >/dev/null 2>&1; bash b/x/scripts/run_cube_place_phase2.sh connect 2>&1' || true)"
  if grep -q "connect-only OK" <<<"${connect_out}"; then
    check "connect_preflight" 0 "robot_mode / start pose / gripper gates all passed"
  else
    banner "connect_preflight" "$(grep -E 'PROBLEM|warning|Error|error' <<<"${connect_out}" | head -3 | tr '\n' '|')" \
      "see dmo_place_2.md S3.4 steps 1-3: REPL re-grasp (measured width -> FRANKA_CUBE_WIDTH_M), guide the arm to 3-5 cm above the mark, then re-run connect" \
      "refused: re-grasping the cube and guiding the arm are physical actions"
    check "connect_preflight" 1
  fi
  authority="$(grep 'authority:' <<<"${connect_out}" | head -1)"
  if grep -q '20\.0N/axis' <<<"${authority}" && ! grep -q '100\.0N/axis' <<<"${authority}"; then
    check "authority_echo" 0 "${authority#authority: }"
  else
    banner "authority_echo" "got '${authority:-<no authority line>}'" \
      "see dmo_place_2.md S6 'authority: ... 100.0N/axis': motion_limits not in effect (stale code or RLINF_CUBE_FORCE_CEILING_N overridden); STOP, this is the LOG-019 authority" \
      "refused: a wrong authority means the running code/env is not what you think it is -- find out why, don't patch over it"
    check "authority_echo" 1
  fi
fi

echo
if [[ "${FAILED}" == "0" ]]; then
  echo "RESULT preflight PASS"
  exit 0
else
  echo "RESULT preflight FAIL"
  exit 1
fi
