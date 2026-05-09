# Copyright 2026 The RLinf Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Command-line input parser for the R1 Pro joint+gripper CLI.

Per design doc ``test_galaxea_r1_pro_cli_controller.md`` §4.1.

A line of REPL input is tokenized and translated into a
:class:`ParsedCommand` dataclass.  The parser is **deliberately
hand-written** (rather than ``argparse``) because:

* The verb-then-side abbreviation pattern (``j r 0.5 ...``) is awkward
  to express in argparse subparsers.
* Each verb has its own argument shape (some verbs take 7 floats,
  some take an index + a float, some take none); argparse's mutually
  exclusive groups become spaghetti.
* The error messages we want -- "did you mean ...?", "side defaulted
  to right" -- are easier to control by hand.

The grammar and the verb -> ParsedCommand mapping is the source of
truth -- :class:`~ActionComposer` and :class:`~ReplLoop` only consume
``ParsedCommand``, never raw lines.  Adding a new verb is therefore a
two-step change: register here, branch in the composer / repl.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Optional


class CliInputError(ValueError):
    """Raised by :meth:`InputParser.parse` for invalid syntax.

    The REPL main loop catches this and prints a one-line red error
    + a hint, then re-prompts.  Carries the offending raw line and
    a brief hint string.
    """

    def __init__(self, line: str, hint: str):
        super().__init__(f"{hint}  (line: {line!r})")
        self.line = line
        self.hint = hint


@dataclass
class ParsedCommand:
    """Result of :meth:`InputParser.parse`.

    Attributes:
        verb: Canonical verb name -- one of ``joint, jn, jd, home,
            sweep, traj, gripper, gn, pose, pn, state, safety, topo,
            mode, brake, set, help, quit, noop``.  Aliases are
            normalised to canonical form.
        side: ``"right"``, ``"left"``, or ``None`` (meta commands).
        payload: verb-specific dict.  Schema documented per verb in
            the parser.
        raw: The original input line (for logging / debugging).
    """

    verb: str
    side: Optional[str] = None
    payload: dict = field(default_factory=dict)
    raw: str = ""


# ─────────────────── Verb / side aliases ───────────────────────


_VERB_ALIASES = {
    "joint": "joint",
    "j": "joint",
    "jn": "jn",
    "jd": "jd",
    "home": "home",
    "h": "home",
    "sweep": "sweep",
    "sw": "sweep",
    "traj": "traj",
    "t": "traj",
    "g": "gripper",
    "grip": "gripper",
    "gripper": "gripper",
    "gn": "gn",
    "pose": "pose",
    "p": "pose",
    "pn": "pn",
    "state": "state",
    "st": "state",
    "safety": "safety",
    "sa": "safety",
    "topo": "topo",
    "to": "topo",
    "mode": "mode",
    "brake": "brake",
    "br": "brake",
    "set": "set",
    "help": "help",
    "?": "help",
    "quit": "quit",
    "q": "quit",
    "exit": "quit",
}

_SIDE_ALIASES = {
    "right": "right",
    "r": "right",
    "left": "left",
    "l": "left",
}

# Verbs whose first argument is an arm side (``r`` / ``l``).
_VERBS_TAKE_SIDE = frozenset(
    {
        "joint",
        "jn",
        "jd",
        "home",
        "sweep",
        "traj",
        "gripper",
        "gn",
        "pose",
        "pn",
        "state",
    }
)

# Verbs that NEVER take a side -- if the user types one, treat it
# as part of the payload.  ``brake on/off``, ``mode joint``, ``set k=v``
# all fall in this category.
_VERBS_NO_SIDE = frozenset(
    {
        "safety",
        "topo",
        "mode",
        "brake",
        "set",
        "help",
        "quit",
        "noop",
    }
)


# ─────────────────── Parser implementation ─────────────────────


def _is_floatish(token: str) -> bool:
    """Cheap lookahead: would this token parse as a float?"""
    try:
        float(token)
        return True
    except ValueError:
        return False


def _to_float(token: str, ctx: str, line: str) -> float:
    try:
        return float(token)
    except ValueError as exc:
        raise CliInputError(line, f"{ctx}: '{token}' is not a number") from exc


def _to_int(token: str, ctx: str, line: str) -> int:
    try:
        return int(token)
    except ValueError as exc:
        raise CliInputError(line, f"{ctx}: '{token}' is not an int") from exc


class InputParser:
    """Stateless line -> ParsedCommand translator.

    The parser is intentionally instance-less in spirit; we still
    instantiate one per CLI session so a future ``--alias-file``
    flag can carry per-user verb / side overrides without touching
    module-level state.
    """

    def __init__(
        self,
        *,
        custom_verb_aliases: Optional[dict] = None,
    ):
        self._verb_map = dict(_VERB_ALIASES)
        if custom_verb_aliases:
            for raw, canon in custom_verb_aliases.items():
                self._verb_map[raw.lower()] = canon

    # ── Top-level entry ────────────────────────────────────────

    def parse(self, line: str) -> ParsedCommand:
        """Tokenise *line* and return a :class:`ParsedCommand`.

        Raises:
            CliInputError: For any malformed input.  Empty / whitespace
                lines return ``ParsedCommand(verb='noop', raw=line)``
                (no exception) so the REPL can just re-prompt.
        """
        raw = line
        if line is None:
            return ParsedCommand(verb="noop", raw="")
        line = line.strip()
        if not line:
            return ParsedCommand(verb="noop", raw=raw)

        # Strip inline comments (`# anything`); useful for command files.
        if "#" in line:
            line = line.split("#", 1)[0].rstrip()
        if not line:
            return ParsedCommand(verb="noop", raw=raw)

        tokens = re.split(r"\s+", line)
        head = tokens[0].lower()
        if head not in self._verb_map:
            raise CliInputError(
                raw,
                f"unknown verb '{head}'; type '?' for help",
            )
        verb = self._verb_map[head]
        rest = tokens[1:]

        # Decide whether the next token is a side or a payload value.
        side: Optional[str] = None
        if verb in _VERBS_TAKE_SIDE and rest:
            head1 = rest[0].lower()
            if head1 in _SIDE_ALIASES:
                side = _SIDE_ALIASES[head1]
                rest = rest[1:]
        if side is None and verb in _VERBS_TAKE_SIDE and verb != "state":
            # Default side is right (covers single-arm bring-up).
            side = "right"

        # Per-verb payload extraction.
        if verb == "joint":
            payload = self._parse_joint_abs(rest, raw)
        elif verb == "jn":
            payload = self._parse_joint_norm(rest, raw)
        elif verb == "jd":
            payload = self._parse_joint_delta(rest, raw)
        elif verb == "home":
            payload = self._parse_home(rest, raw)
        elif verb == "sweep":
            payload = self._parse_sweep(rest, raw)
        elif verb == "traj":
            payload = self._parse_traj(rest, raw)
        elif verb == "gripper":
            payload = self._parse_gripper_pct(rest, raw)
        elif verb == "gn":
            payload = self._parse_gripper_norm(rest, raw)
        elif verb == "pose":
            payload = self._parse_pose_abs(rest, raw)
        elif verb == "pn":
            payload = self._parse_pose_norm(rest, raw)
        elif verb == "state":
            payload = {}
        elif verb == "safety":
            payload = {}
        elif verb == "topo":
            payload = {}
        elif verb == "mode":
            payload = self._parse_mode(rest, raw)
        elif verb == "brake":
            payload = self._parse_brake(rest, raw)
        elif verb == "set":
            payload = self._parse_set(rest, raw)
        elif verb == "help":
            payload = {}
        elif verb == "quit":
            payload = {}
        else:
            raise CliInputError(raw, f"BUG: unhandled verb '{verb}'")

        return ParsedCommand(verb=verb, side=side, payload=payload, raw=raw)

    # ── Per-verb sub-parsers ───────────────────────────────────

    def _parse_joint_abs(self, rest: list[str], raw: str) -> dict:
        # 7-token branch (full-arm abs): convert one-by-one so that a
        # non-numeric in any slot produces the precise "not a number"
        # error instead of the generic "wrong arg count" hint.
        if len(rest) == 7:
            return {
                "mode": "abs",
                "q_rad": [_to_float(t, "joint", raw) for t in rest],
            }
        if len(rest) == 2:
            idx = _to_int(rest[0], "joint idx", raw)
            val = _to_float(rest[1], "joint q (rad)", raw)
            if not (0 <= idx <= 6):
                raise CliInputError(
                    raw,
                    f"joint idx must be in [0, 6], got {idx}",
                )
            return {"mode": "single_abs", "idx": idx, "q_rad": val}
        raise CliInputError(
            raw,
            "joint takes 7 floats (full-arm abs) or "
            "<idx 0..6> <q_rad> (single-joint abs)",
        )

    def _parse_joint_norm(self, rest: list[str], raw: str) -> dict:
        if len(rest) == 7:
            return {
                "mode": "norm",
                "q_norm": [_to_float(t, "jn", raw) for t in rest],
            }
        if len(rest) == 2:
            idx = _to_int(rest[0], "jn idx", raw)
            val = _to_float(rest[1], "jn q_norm", raw)
            if not (0 <= idx <= 6):
                raise CliInputError(
                    raw,
                    f"jn idx must be in [0, 6], got {idx}",
                )
            return {"mode": "single_norm", "idx": idx, "q_norm": val}
        raise CliInputError(
            raw,
            "jn takes 7 floats (full-arm norm) or "
            "<idx 0..6> <q_norm> (single-joint norm)",
        )

    def _parse_joint_delta(self, rest: list[str], raw: str) -> dict:
        if len(rest) != 2:
            raise CliInputError(
                raw,
                "jd takes <idx 0..6> <delta_rad>",
            )
        idx = _to_int(rest[0], "jd idx", raw)
        delta = _to_float(rest[1], "jd delta", raw)
        if not (0 <= idx <= 6):
            raise CliInputError(
                raw,
                f"jd idx must be in [0, 6], got {idx}",
            )
        return {"mode": "delta", "idx": idx, "delta_rad": delta}

    def _parse_home(self, rest: list[str], raw: str) -> dict:
        if rest:
            raise CliInputError(
                raw,
                "home takes no arguments (just a side, optional)",
            )
        return {}

    def _parse_sweep(self, rest: list[str], raw: str) -> dict:
        if len(rest) != 5:
            raise CliInputError(
                raw,
                "sweep takes <idx 0..6> <lo> <hi> <step> <dwell_s>",
            )
        idx = _to_int(rest[0], "sweep idx", raw)
        lo = _to_float(rest[1], "sweep lo", raw)
        hi = _to_float(rest[2], "sweep hi", raw)
        step = _to_float(rest[3], "sweep step", raw)
        dwell = _to_float(rest[4], "sweep dwell_s", raw)
        if not (0 <= idx <= 6):
            raise CliInputError(
                raw,
                f"sweep idx must be in [0, 6], got {idx}",
            )
        if step <= 0:
            raise CliInputError(raw, "sweep step must be > 0")
        if dwell < 0:
            raise CliInputError(raw, "sweep dwell must be >= 0")
        return {
            "idx": idx,
            "lo": lo,
            "hi": hi,
            "step": step,
            "dwell_s": dwell,
        }

    def _parse_traj(self, rest: list[str], raw: str) -> dict:
        if len(rest) != 1:
            raise CliInputError(
                raw,
                "traj takes <path-to-csv>",
            )
        return {"path": rest[0]}

    def _parse_gripper_pct(self, rest: list[str], raw: str) -> dict:
        if len(rest) != 1:
            raise CliInputError(
                raw,
                "g takes <pct 0..100> or 'open' / 'close'",
            )
        token = rest[0].lower()
        if token in ("open", "o"):
            return {"mode": "open"}
        if token in ("close", "c"):
            return {"mode": "close"}
        try:
            pct = float(token)
        except ValueError as exc:
            raise CliInputError(
                raw,
                f"g: '{token}' is not 'open' / 'close' / a number",
            ) from exc
        return {"mode": "pct", "pct": pct}

    def _parse_gripper_norm(self, rest: list[str], raw: str) -> dict:
        if len(rest) != 1:
            raise CliInputError(
                raw,
                "gn takes <value -1..1>",
            )
        return {"mode": "norm", "value": _to_float(rest[0], "gn value", raw)}

    def _parse_pose_abs(self, rest: list[str], raw: str) -> dict:
        if len(rest) != 7:
            raise CliInputError(
                raw,
                "pose takes 7 floats: x y z qx qy qz qw",
            )
        vals = [_to_float(t, "pose", raw) for t in rest]
        return {"mode": "abs", "pose": vals}

    def _parse_pose_norm(self, rest: list[str], raw: str) -> dict:
        if len(rest) != 7:
            raise CliInputError(
                raw,
                "pn takes 7 floats in [-1, 1]",
            )
        return {
            "mode": "norm",
            "pose_norm": [_to_float(t, "pn", raw) for t in rest],
        }

    def _parse_mode(self, rest: list[str], raw: str) -> dict:
        if len(rest) != 1 or rest[0].lower() not in ("joint", "ee"):
            raise CliInputError(
                raw,
                "mode takes 'joint' or 'ee'",
            )
        return {"mode": rest[0].lower()}

    def _parse_brake(self, rest: list[str], raw: str) -> dict:
        if len(rest) != 1 or rest[0].lower() not in ("on", "off"):
            raise CliInputError(
                raw,
                "brake takes 'on' or 'off'",
            )
        return {"on": rest[0].lower() == "on"}

    def _parse_set(self, rest: list[str], raw: str) -> dict:
        if len(rest) != 1 or "=" not in rest[0]:
            raise CliInputError(
                raw,
                "set takes <key>=<value>, e.g. 'set heartbeat_ms=5000'",
            )
        k, v = rest[0].split("=", 1)
        # Try int -> float -> bool -> str fallback so users don't
        # have to quote anything.
        v_parsed: object
        try:
            v_parsed = int(v)
        except ValueError:
            try:
                v_parsed = float(v)
            except ValueError:
                if v.lower() in ("true", "false"):
                    v_parsed = v.lower() == "true"
                else:
                    v_parsed = v
        return {"k": k, "v": v_parsed}

    # ── Help string ────────────────────────────────────────────

    def help(self) -> str:
        """One-screen help text printed on '?' / 'help' / parse error."""
        return _HELP_TEXT


_HELP_TEXT = """\
R1 Pro joint+gripper CLI commands
=================================

Joint mode:
  joint|j  [side]  q0 q1 q2 q3 q4 q5 q6   # 7 absolute joint angles in rad
  joint|j  [side]  IDX VALUE              # single joint absolute (idx 0..6)
  jn       [side]  v0 v1 v2 v3 v4 v5 v6   # 7 normalised values [-1, 1]
  jn       [side]  IDX VALUE              # single joint normalised
  jd       [side]  IDX DELTA              # single joint increment in rad
  home|h   [side]                         # reset to home pose
  sweep|sw [side]  IDX LO HI STEP DWELL_S
  traj|t   [side]  PATH                   # CSV trajectory file

Gripper:
  g|grip|gripper [side] PCT               # absolute pct in [0, 100]
  g              [side] open|close        # gmax_pct / gmin_pct
  gn             [side] VALUE             # normalised [-1, 1]

EE pose (only when mode=ee):
  pose|p   [side]  X Y Z QX QY QZ QW
  pn       [side]  v0..v6                 # normalised [-1, 1]

Meta:
  state|st  [side]                        # show current state
  safety|sa                               # show last SafetyInfo details
  topo|to                                 # ROS2 topology health check
  mode      joint|ee                      # switch action mode
  brake|br  on|off                        # apply / release brake
  set       KEY=VALUE                     # change runtime config
  help|?                                  # show this help
  quit|q|exit                             # exit CLI

Side defaults to 'right' for action verbs; aliases r/l for right/left.
Lines starting with '#' (or after '#') are ignored as comments.
"""
