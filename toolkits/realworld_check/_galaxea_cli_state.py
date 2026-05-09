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

"""State / safety / topology pretty-printer for the R1 Pro CLI.

Per design doc ``test_galaxea_r1_pro_cli_controller.md`` §4.7.

All output goes through fixed-prefix tags (``[STATE]``, ``[SAFETY]``,
``[TOPO]``, ``[INFO]``, ``[WARN]``, ``[TX]``) so an operator can grep
sessions later.  Each pretty-printer returns a multi-line string;
the REPL is responsible for actually printing it (kept here so unit
tests can introspect the format without capturing stdout).
"""

from __future__ import annotations

import datetime as _dt
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

# ─────────────────────── Tag prefixes ──────────────────────────


class Tag:
    INFO = "[INFO]   "
    WARN = "[WARN]   "
    ERROR = "[ERROR]  "
    STATE = "[STATE]  "
    SAFETY = "[SAFETY] "
    TOPO = "[TOPO]   "
    TX = "[TX]     "
    DRY = "[TX-DRY] "
    CRITICAL = "[CRITICAL]"


def now_iso() -> str:
    """ISO-ish timestamp with millisecond precision."""
    return _dt.datetime.now().strftime("%Y-%m-%dT%H:%M:%S.") + (
        f"{_dt.datetime.now().microsecond // 1000:03d}"
    )


def fmt_vec(vec: Sequence[float], precision: int = 4) -> str:
    """Format a numeric vector compactly, signed, fixed precision."""
    return (
        "["
        + ", ".join(f"{x:+.{precision}f}" for x in np.asarray(vec, dtype=np.float64))
        + "]"
    )


# ─────────────────────── Topology spec ─────────────────────────


@dataclass
class TopologySpec:
    """What topics the current CLI session needs alive.

    Constructed by :class:`StateMonitor.topology_report` from the cfg
    + the active dispatcher's ``get_required_topics``.  Separated out
    so unit tests can build one explicitly without spinning up a real
    backend.
    """

    publishers: list[str] = field(default_factory=list)
    subscribers: list[str] = field(default_factory=list)
    optional: list[str] = field(default_factory=list)


# ─────────────────────── StateMonitor ──────────────────────────


class StateMonitor:
    """Pretty-print state, safety info, topology, TX events.

    Receives a :class:`~rlinf.envs.realworld.galaxear.r1_pro_gripper_mixer.GripperMixer`
    so the gripper line can show both physical pct and policy-side
    obs in ``[-1, 1]``.  Stateless apart from the mixer ref.
    """

    def __init__(self, gripper_mixer):
        self._mixer = gripper_mixer

    # ── State pretty-printer ───────────────────────────────────

    def pretty_state(
        self,
        state,
        side: Optional[str] = None,
    ) -> str:
        """Return a multi-line ``[STATE]`` block.

        Args:
            state: :class:`GalaxeaR1ProRobotState` snapshot.
            side: ``"right"`` / ``"left"`` to print only one arm; pass
                ``None`` to print whichever arms have non-zero qpos
                (or both right and left as a global query).
        """
        ts = now_iso()
        lines: list[str] = []

        sides = (side,) if side is not None else ("right", "left")

        for s in sides:
            qpos = state.get_arm_qpos(s)
            qvel = state.get_arm_qvel(s)
            ee = state.get_ee_pose(s)
            grip_mm = state.get_gripper_pos(s)
            grip_obs = self._mixer.pct_to_obs11(grip_mm)
            lines.append(
                f"{Tag.STATE}side={s}\n"
                f"  q   (rad): {fmt_vec(qpos)}\n"
                f"  qvel (rad/s): {fmt_vec(qvel)}\n"
                f"  ee_xyz (m):   {fmt_vec(ee[:3])} (frame torso_link4)\n"
                f"  ee_quat:      {fmt_vec(ee[3:])}\n"
                f"  gripper:      pos={grip_mm:6.2f}%  "
                f"obs_norm={grip_obs:+.3f}  "
                f"(gmin={self._mixer.gmin_pct:.0f}, "
                f"gmax={self._mixer.gmax_pct:.0f})"
            )

        bms_pct = float(state.bms.get("capital_pct", 100.0))
        n_stale = sum(
            1 for age in (state.feedback_age_ms or {}).values() if age > 200.0
        )
        n_status_errs = sum(1 for errs in (state.status_errors or {}).values() if errs)
        lines.append(
            f"  health:       bms={bms_pct:5.1f}%  "
            f"alive={state.is_alive}  "
            f"stale_topics={n_stale}  status_errors={n_status_errs}"
        )

        return f"{ts} " + ("\n  " + ts + " ").join(lines)

    # ── SafetyInfo overlay ─────────────────────────────────────

    def safety_overlay(self, info) -> str:
        """Return a multi-line ``[SAFETY]`` block.

        The first line is a one-shot summary; subsequent lines list
        each ``info.reason`` element verbatim so the operator sees
        every gate that triggered."""
        ts = now_iso()
        head = (
            f"{Tag.SAFETY}reason_count={len(info.reason)}  "
            f"clipped={info.clipped}  "
            f"soft_hold={info.soft_hold}  "
            f"safe_stop={info.safe_stop}  "
            f"emergency_stop={info.emergency_stop}"
        )
        out = [f"{ts} {head}"]
        for r in info.reason:
            out.append(f"{ts}   {r}")
        # Compact metrics line for grepping.
        if info.metrics:
            metric_str = "  ".join(
                f"{k}={float(v):.3g}" for k, v in info.metrics.items()
            )
            out.append(f"{ts}   metrics: {metric_str}")
        return "\n".join(out)

    # ── Topology report ───────────────────────────────────────

    def topology_report(
        self,
        backend,
        spec: TopologySpec,
    ) -> str:
        """Run ``get_subscription_count`` / ``get_publisher_count`` for
        every topic in *spec* and format a table.

        Returns a multi-line string; the caller decides whether to
        ``print()`` it or feed to the strict-topo guard.
        """
        ts = now_iso()
        out = [
            f"{ts} {Tag.TOPO}backend={backend.kind}",
            f"{ts}   REQUIRED publishers (need >=1 subscriber):",
        ]
        any_warn = False
        for topic in spec.publishers:
            n = int(backend.get_subscription_count(topic))
            ok = "OK" if n >= 1 else "WARN"
            if n < 1:
                any_warn = True
            out.append(f"{ts}     {topic:55s}  subs={n:3d}  {ok}")
        out.append(f"{ts}   REQUIRED subscribers (need >=1 publisher):")
        for topic in spec.subscribers:
            n = int(backend.get_publisher_count(topic))
            ok = "OK" if n >= 1 else "WARN"
            if n < 1:
                any_warn = True
            out.append(f"{ts}     {topic:55s}  pubs={n:3d}  {ok}")
        if spec.optional:
            out.append(f"{ts}   OPTIONAL subscribers (informational):")
            for topic in spec.optional:
                n = int(backend.get_publisher_count(topic))
                tag = "OK" if n >= 1 else "missing"
                out.append(f"{ts}     {topic:55s}  pubs={n:3d}  {tag}")
        out.append(
            f"{ts}   summary: {'WARN -- missing topics' if any_warn else 'all OK'}"
        )
        return "\n".join(out)

    # ── TX log line ───────────────────────────────────────────

    def tx_line(self, record: dict, dry: bool = False) -> str:
        """Format a single backend TX record (from ``backend.tx_log``).

        Used by the REPL to echo what was just sent so the operator
        can correlate command -> dispatch -> ROS2 publish.
        """
        ts = now_iso()
        tag = Tag.DRY if dry else Tag.TX
        t = record.get("type", "?")
        side = record.get("side", "?")
        if t == "arm_joints":
            return (
                f"{ts} {tag}/motion_target/target_joint_state_arm_{side}  "
                f"pos={fmt_vec(record['q_target'])}  "
                f"vmax={fmt_vec(record['qvel_max'], precision=2)}"
            )
        if t == "arm_pose":
            return (
                f"{ts} {tag}/motion_target/target_pose_arm_{side}  "
                f"pose={fmt_vec(record['pose'])}"
            )
        if t == "gripper":
            return (
                f"{ts} {tag}/motion_target/target_position_gripper_{side}  "
                f"pct=[{record['pct']:6.2f}]"
            )
        if t == "brake":
            return f"{ts} {tag}/motion_target/brake_mode  data={record['on']}"
        return f"{ts} {tag}<unknown record type {t}>"

    # ── Convenience info / warn lines ──────────────────────────

    @staticmethod
    def info(msg: str) -> str:
        return f"{now_iso()} {Tag.INFO}{msg}"

    @staticmethod
    def warn(msg: str) -> str:
        return f"{now_iso()} {Tag.WARN}{msg}"

    @staticmethod
    def error(msg: str) -> str:
        return f"{now_iso()} {Tag.ERROR}{msg}"
