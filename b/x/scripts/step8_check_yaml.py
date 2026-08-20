#!/usr/bin/env python3
"""Step 8b: accept camera YAML against 8a JSON (no robot).

Exit 0 only if YAML has real serials matching camera_detected.json
and is_dummy is false.

Usage:
  python b/x/scripts/step8_check_yaml.py
  python b/x/scripts/step8_check_yaml.py --json-in ... --yaml-in ...
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

REPO = os.environ.get(
    "REPO_PATH",
    os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")),
)
sys.path.insert(0, os.path.join(REPO, "b", "x", "scripts"))

from step8_checks import (  # noqa: E402
    check,
    collect_yaml_serials,
    is_placeholder_serial,
    result,
)

DEFAULT_JSON = os.path.join(REPO, "b", "x", "configs", "camera_detected.json")
DEFAULT_YAML = os.path.join(REPO, "b", "x", "configs", "realworld_franky_camera.yaml")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Step 8b: validate camera YAML")
    parser.add_argument("--json-in", default=DEFAULT_JSON)
    parser.add_argument("--yaml-in", default=DEFAULT_YAML)
    parser.add_argument(
        "--expect-gym-id",
        default="FrankyFrankaEnv-v1",
        help="expected env.eval.init_params.id (default: FrankyFrankaEnv-v1; "
        "the cube-place carrier uses FrankyCubePlaceEnv-v1)",
    )
    parser.add_argument(
        "--expect-serials-from",
        default=None,
        help="second YAML whose camera serials must equal the detected set, "
        "order included (e.g. the training config realworld_cube_place_sac.yaml); "
        "catches 'detected 2 cameras but training expects 1' (LOG-031)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print("Step8b: check camera YAML wiring (no FCI)")
    failed = False

    json_ok = os.path.isfile(args.json_in)
    check("json_exists", json_ok, args.json_in)
    if not json_ok:
        return result("8b", False, reason="run step8_detect_cameras.py first")

    detected = json.loads(Path(args.json_in).read_text(encoding="utf-8"))
    det_type = str(detected.get("camera_type") or "")
    det_serials = [str(s) for s in detected.get("camera_serials") or []]
    check("json_has_serials", bool(det_serials), str(det_serials))
    ph_json = [s for s in det_serials if is_placeholder_serial(s)]
    check("json_serials_not_placeholder", not ph_json, str(ph_json or det_serials))
    if not det_serials or ph_json:
        failed = True

    yaml_ok = os.path.isfile(args.yaml_in)
    check("yaml_exists", yaml_ok, args.yaml_in)
    if not yaml_ok:
        return result("8b", False, reason="missing YAML")

    import yaml

    data = yaml.safe_load(Path(args.yaml_in).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        check("yaml_parse", False, "not a mapping")
        return result("8b", False, reason="bad YAML")
    check("yaml_parse", True)

    y_type, y_serials = collect_yaml_serials(data)
    check("yaml_has_serials", bool(y_serials), str(y_serials))
    ph_yaml = [s for s in y_serials if is_placeholder_serial(s)]
    check("yaml_serials_not_placeholder", not ph_yaml, str(ph_yaml or y_serials))
    match = list(y_serials) == list(det_serials)
    check("yaml_serials_match_json", match, f"yaml={y_serials} json={det_serials}")
    type_ok = (y_type or "") == det_type or not det_type
    check("yaml_camera_type_match_json", type_ok, f"yaml={y_type} json={det_type}")

    override = ((data.get("env") or {}).get("eval") or {}).get("override_cfg") or {}
    dummy = bool(override.get("is_dummy", True))
    check("yaml_is_dummy_false", dummy is False, f"is_dummy={dummy}")
    gym_id = ((data.get("env") or {}).get("eval") or {}).get("init_params", {}).get(
        "id"
    )
    check(
        "yaml_gym_id_franky",
        str(gym_id) == args.expect_gym_id,
        f"{gym_id} (expected {args.expect_gym_id})",
    )
    names = override.get("camera_names") or {}
    wrist_ok = (not names) or ("wrist_1" in names.values())
    check("yaml_wrist_1_name", wrist_ok, str(names))
    dup_names = len(set(names.values())) != len(names.values())
    check("yaml_camera_names_unique", not dup_names, str(names))
    if dup_names:
        failed = True

    if args.expect_serials_from:
        train_ok = os.path.isfile(args.expect_serials_from)
        check("train_yaml_exists", train_ok, args.expect_serials_from)
        if not train_ok:
            failed = True
        else:
            train_data = yaml.safe_load(
                Path(args.expect_serials_from).read_text(encoding="utf-8")
            )
            _, t_serials = collect_yaml_serials(train_data or {})
            # Order matters: the first serial becomes wrist_1.
            match_train = list(t_serials) == list(det_serials)
            check(
                "train_yaml_serials_match",
                match_train,
                f"train={t_serials} detected={det_serials}",
            )
            if not match_train:
                failed = True

    if not y_serials or ph_yaml or not match or dummy or not type_ok:
        failed = True

    return result(
        "8b",
        not failed,
        camera_type=y_type,
        camera_serials=y_serials,
        yaml=args.yaml_in,
        json=args.json_in,
    )


if __name__ == "__main__":
    raise SystemExit(main())
