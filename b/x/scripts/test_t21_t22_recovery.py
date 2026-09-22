#!/usr/bin/env python3
"""Acceptance test for LOG-040's T21 (trip recovery) and T22 (elbow self-check).

Both fixes exist because of the same run: a ``[watchdog:dq]`` trip braked the arm
cleanly and correctly, and then killed the entire training job -- and the reason
the arm was in a bad configuration in the first place had gone unnoticed for
minutes, because nothing looks at joint configuration.

Three halves, by what they can catch:

* **Math** (no hardware): ``jacobian_conditioning`` against matrices whose
  singular values are known by construction, and the placement of the
  ``sigma_min`` warning threshold between the measured good and bad poses.
* **Behaviour** (no hardware): the recovery path driven through a stub
  controller -- budget exhaustion, a refused recovery, and the property the whole
  design rests on, that ``step()``/``reset()`` catch a *guard trip* and nothing
  else. If they caught bare ``RuntimeError`` they would also swallow the
  interpolation refusals, which must stay fatal.
* **Real** (``--robot``, read-only): ``sigma_min`` at the poses LOG-037/LOG-040
  recorded, to confirm the threshold still sits where the log says it does.

Run inside the franky container after ``source b/x/configs/setup_before_ray_5090.sh``.
``--robot`` connects but commands NO motion.
"""

from __future__ import annotations

import argparse
import inspect
import os
import sys

import numpy as np

REPO = os.environ.get(
    "REPO_PATH", os.path.abspath(os.path.join(__file__, "../../../.."))
)
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "b", "x"))

from franky_ext.motion_limits import (  # noqa: E402
    GUARD_RECOVERY_BUDGET_DEFAULT,
    SIGMA_MIN_WARN_DEFAULT,
    guard_recovery_budget,
    jacobian_conditioning,
    sigma_min_warn,
)

# Measured on this arm; every number here appears in the LOG entry named beside it.
TRIP_Q = [0.0330, 1.1360, -0.0341, -0.4233, 0.0853, 1.4967, 0.5404]  # LOG-036
HOVER_Q = [0.2304, 0.5462, -0.2855, -1.2968, 0.1800, 1.7952, 1.0502]  # LOG-037
DRIFT_Q = [0.0726, 1.0085, -0.0485, -0.4606, 0.0656, 1.3530, 0.9832]  # LOG-040 §40.1
RECOVERED_Q = [0.1187, 0.6011, -0.0995, -1.3072, 0.0458, 1.8885, 0.7611]  # LOG-040 §40.2

SIGMA_HOVER = 0.1115
SIGMA_TRIP = 0.0062
SIGMA_DRIFT = 0.0009

_failures: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  {detail}" if detail else ""))
    if not ok:
        _failures.append(name)


# ----------------------------------------------------------------------
# Stubs. Enough surface for the recovery path, and nothing else.
# ----------------------------------------------------------------------


class _FakeFuture:
    def __init__(self, value):
        self._value = value

    def wait(self):
        return [self._value]


class _FakeState:
    def __init__(self, jacobian=None, q=None):
        self.arm_jacobian = jacobian
        self.arm_joint_position = q
        self.tcp_pose = np.zeros(7)


class _FakeController:
    """Returns queued recovery reports, then keeps returning the last one."""

    def __init__(self, reports=None, state=None):
        self._reports = list(reports or [{"recovered": True, "robot_mode": "Idle"}])
        self.state = state or _FakeState()
        self.recover_calls = 0

    def recover_from_guard_trip(self):
        self.recover_calls += 1
        report = self._reports[0] if len(self._reports) == 1 else self._reports.pop(0)
        return _FakeFuture(report)

    def get_state(self):
        return _FakeFuture(self.state)


class _FakeLogger:
    def __init__(self):
        self.lines: list[tuple[str, str]] = []

    def _record(self, level, msg, *args):
        try:
            self.lines.append((level, msg % args if args else msg))
        except Exception:  # noqa: BLE001 - a logging stub must never be the failure
            self.lines.append((level, msg))

    def info(self, msg, *args, **kw):
        self._record("info", msg, *args)

    def warning(self, msg, *args, **kw):
        self._record("warning", msg, *args)

    def error(self, msg, *args, **kw):
        self._record("error", msg, *args)

    def text(self) -> str:
        return "\n".join(line for _lvl, line in self.lines)


class _FakeConfig:
    is_dummy = False


def _stub_env(controller=None, logger=None):
    """A real mixin instance carrying only the attributes the recovery path touches.

    The mixin's own methods have to be genuinely bound (``_recover_from_trip``
    calls ``_warn_if_ill_conditioned``, which calls ``_safe_state``), so this
    subclasses it for real and skips ``__init__`` instead of faking the methods
    -- constructing the real thing would launch a Ray actor and talk to an arm.
    Setting only these five attributes is itself a check: it fails loudly if the
    recovery path starts depending on state only ``FrankaEnv.__init__`` provides.
    """
    from franky_ext.franky_single_franka_env import FrankySingleFrankaEnvMixin

    class _StubEnv(FrankySingleFrankaEnvMixin):
        pass

    env = object.__new__(_StubEnv)
    env.config = _FakeConfig()
    env._controller = controller or _FakeController()
    env._logger = logger or _FakeLogger()
    env._guard_recoveries_used = 0
    env._num_steps = 0
    return env


# ----------------------------------------------------------------------


def test_conditioning_math() -> None:
    print("\n=== T22 math: jacobian_conditioning ===")

    # No data must mean no opinion, same convention as joint_demand_scale: the
    # dataclass default for arm_jacobian is zeros, which a dummy env carries.
    check("zeros -> None (no data, not 'perfectly singular')",
          jacobian_conditioning(np.zeros((6, 7))) is None)
    check("wrong shape -> None", jacobian_conditioning(np.eye(6)) is None)
    check("None -> None", jacobian_conditioning(None) is None)

    # Known singular values by construction: diag(1..1, s) on the first six rows.
    J = np.zeros((6, 7))
    J[:6, :6] = np.eye(6)
    J[5, 5] = 0.01
    cond = jacobian_conditioning(J)
    check(
        "sigma_min is the constructed one",
        abs(cond["sigma_min"] - 0.01) < 1e-12,
        f"{cond['sigma_min']:.6f}",
    )
    check("cond = sigma_max/sigma_min", abs(cond["cond"] - 100.0) < 1e-9,
          f"{cond['cond']:.3f}")
    check(
        "manipulability = product of singular values",
        abs(cond["manipulability"] - 0.01) < 1e-9,
        f"{cond['manipulability']:.6f}",
    )

    # An exactly singular Jacobian must not divide by zero: this is the shape a
    # warning has to survive, since it is the worst case it exists to report.
    J_sing = np.zeros((6, 7))
    J_sing[:5, :5] = np.eye(5)
    sing = jacobian_conditioning(J_sing)
    check("exact singularity -> cond=inf, not a crash", sing["cond"] == float("inf"))

    # Row subsets, because the diagnostic reports translation and rotation
    # blocks separately and now shares this function to do it.
    block = jacobian_conditioning(J[:3])
    check("accepts a 3x7 block", block is not None and block["sigma_min"] > 0)


def test_threshold_placement() -> None:
    print("\n=== T22: sigma_min warning threshold ===")
    thr = sigma_min_warn()
    check("default is the documented value", thr == SIGMA_MIN_WARN_DEFAULT, f"{thr}")
    # The threshold is only worth anything if it separates the measured
    # populations. Below the good poses so it is quiet in normal operation;
    # above the bad ones so it actually fires where it mattered.
    check(
        f"below the hover pose ({SIGMA_HOVER}, known-good, LOG-037)",
        thr < SIGMA_HOVER,
        f"{thr} < {SIGMA_HOVER}",
    )
    check(
        f"above the LOG-036 trip pose ({SIGMA_TRIP})",
        thr > SIGMA_TRIP,
        f"{thr} > {SIGMA_TRIP}",
    )
    check(
        f"above LOG-040's drifted pose ({SIGMA_DRIFT}, could not be reset out of)",
        thr > SIGMA_DRIFT,
    )


def test_recovery_budget() -> None:
    print("\n=== T21: recovery budget ===")
    from franky_ext.franky_single_franka_env import (
        FrankySingleFrankaEnvMixin,
        MotionGuardTripped,
    )

    recover = FrankySingleFrankaEnvMixin._recover_from_trip
    exc = MotionGuardTripped("[watchdog:dq] joint runaway: |dq|=1.2942 rad/s > 1.2000")

    check(
        "default budget is the documented value",
        guard_recovery_budget() == GUARD_RECOVERY_BUDGET_DEFAULT,
        f"{guard_recovery_budget()}",
    )

    previous = os.environ.get("RLINF_CUBE_GUARD_RECOVERY_BUDGET")
    try:
        os.environ["RLINF_CUBE_GUARD_RECOVERY_BUDGET"] = "2"
        env = _stub_env()
        results = [recover(env, exc, "step()") for _ in range(3)]
        check(
            "recovers up to the budget, then stops",
            results == [True, True, False],
            f"{results} (controller called {env._controller.recover_calls}x)",
        )
        check(
            "the refused attempt does not call the controller again",
            env._controller.recover_calls == 2,
            f"{env._controller.recover_calls} calls for 3 trips",
        )

        # 0 must restore the pre-LOG-040 behaviour exactly: any trip ends the run.
        os.environ["RLINF_CUBE_GUARD_RECOVERY_BUDGET"] = "0"
        env0 = _stub_env()
        check(
            "budget 0 disables recovery entirely (old behaviour)",
            recover(env0, exc, "step()") is False
            and env0._controller.recover_calls == 0,
        )
    finally:
        if previous is None:
            os.environ.pop("RLINF_CUBE_GUARD_RECOVERY_BUDGET", None)
        else:
            os.environ["RLINF_CUBE_GUARD_RECOVERY_BUDGET"] = previous

    # A controller that refuses (arm still moving, Desk fault that will not
    # clear) must end the run, not be retried around.
    refusing = _FakeController(
        [{"recovered": False, "refusal": "the arm is still moving after the brake"}]
    )
    env_r = _stub_env(controller=refusing)
    logger = env_r._logger
    check("a refused recovery ends the run", recover(env_r, exc, "step()") is False)
    check(
        "the refusal tells the operator what to do by hand",
        "MANUAL ACTION" in logger.text(),
    )


def test_self_check_behaviour() -> None:
    print("\n=== T22: the self-check warns, and only when it should ===")
    from franky_ext.franky_single_franka_env import FrankySingleFrankaEnvMixin

    warn = FrankySingleFrankaEnvMixin._warn_if_ill_conditioned

    good = np.zeros((6, 7))
    good[:6, :6] = np.eye(6)  # sigma_min = 1.0, far above any sane threshold
    env_good = _stub_env(
        controller=_FakeController(state=_FakeState(jacobian=good, q=RECOVERED_Q))
    )
    result = warn(env_good, "before reset")
    check("well-conditioned: no warning", not any(
        lvl == "warning" for lvl, _ in env_good._logger.lines
    ), f"sigma_min={result['sigma_min']:.4f}")

    bad = good.copy()
    bad[5, 5] = SIGMA_DRIFT  # LOG-040's measured drift
    env_bad = _stub_env(
        controller=_FakeController(state=_FakeState(jacobian=bad, q=DRIFT_Q))
    )
    warn(env_bad, "before reset")
    text = env_bad._logger.text()
    check("ill-conditioned: warns", "ILL-CONDITIONED" in text)
    check("the warning names the manual action", "MANUAL ACTION" in text)
    check("the warning quotes the known-good elbow to aim at", "0.23, 0.55" in text)
    check("the warning prints the actual q", "1.0085" in text or "0.0726" in text)

    # Advisory means advisory: a controller that cannot be read must not be the
    # reason a reset fails, particularly right after a trip.
    class _DeadController:
        def get_state(self):
            raise RuntimeError("actor is gone")

    env_dead = _stub_env(controller=_DeadController())
    check(
        "an unreadable controller degrades to a skip, not an exception",
        warn(env_dead, "after a guard trip") is None,
    )

    # No jacobian (dummy env) must be silent rather than warning about zeros.
    env_nodata = _stub_env(
        controller=_FakeController(state=_FakeState(jacobian=np.zeros((6, 7))))
    )
    check("no jacobian data: silent", warn(env_nodata, "before reset") is None)


def test_step_reset_wiring() -> None:
    print("\n=== T21: step()/reset() catch a trip and nothing else ===")
    from franky_ext.franky_single_franka_env import (
        RESET_TRIP_ATTEMPTS,
        FrankySingleFrankaEnvMixin,
        MotionGuardTripped,
    )

    check(
        "MotionGuardTripped is a RuntimeError (existing handlers still work)",
        issubclass(MotionGuardTripped, RuntimeError),
    )
    raise_src = inspect.getsource(FrankySingleFrankaEnvMixin._raise_if_guard_tripped)
    check(
        "_raise_if_guard_tripped raises the specific type",
        "raise MotionGuardTripped(" in raise_src,
    )

    obs_sentinel = {"observation": "post-brake"}

    class _Base:
        """Stands in for FrankaEnv: raises what a trip raises, on demand."""

        def __init__(self):
            self.step_calls = 0
            self.reset_calls = 0
            self.trips_left = 0
            self.raise_other = False

        def step(self, action):
            self.step_calls += 1
            if self.raise_other:
                raise RuntimeError("refusing to interpolate from this configuration")
            if self.trips_left > 0:
                self.trips_left -= 1
                raise MotionGuardTripped("[watchdog:dq] joint runaway")
            return obs_sentinel, 1.0, False, False, {}

        def reset(self, *a, **kw):
            self.reset_calls += 1
            if self.trips_left > 0:
                self.trips_left -= 1
                raise MotionGuardTripped("[watchdog:dq] joint runaway during reset")
            return obs_sentinel, {}

        def _get_observation(self):
            return obs_sentinel

    class _Env(FrankySingleFrankaEnvMixin, _Base):
        pass

    def make_env(trips: int, reports=None):
        # Bypass __init__ on purpose: the real one launches a Ray actor.
        env = object.__new__(_Env)
        _Base.__init__(env)
        env.trips_left = trips
        env.config = _FakeConfig()
        env._controller = _FakeController(
            reports, state=_FakeState(jacobian=None, q=None)
        )
        env._logger = _FakeLogger()
        env._guard_recoveries_used = 0
        env._num_steps = 0
        return env

    # A trip during step() becomes a truncated episode, not a dead job.
    env = make_env(trips=1)
    obs, reward, terminated, truncated, info = env.step(np.zeros(7))
    check("step(): a trip returns instead of propagating", obs is obs_sentinel)
    check("step(): the episode is truncated, not terminated",
          truncated is True and terminated is False)
    check("step(): reward is 0 for a step that did not execute", reward == 0.0)
    check("step(): info records the trip", "motion_guard_trip" in info)
    check("step(): the step is still counted", env._num_steps == 1)

    # The distinction the whole design rests on: an interpolation refusal is not
    # a trip, and must stay fatal.
    env_other = make_env(trips=0)
    env_other.raise_other = True
    try:
        env_other.step(np.zeros(7))
        check("step(): a non-trip RuntimeError still propagates", False)
    except MotionGuardTripped:
        check("step(): a non-trip RuntimeError still propagates", False,
              "caught as a trip")
    except RuntimeError as exc:
        check(
            "step(): a non-trip RuntimeError still propagates",
            "refusing to interpolate" in str(exc),
        )
    check(
        "step(): no recovery was attempted for a non-trip error",
        env_other._controller.recover_calls == 0,
    )

    # reset() retries past a recovered trip...
    env_reset = make_env(trips=1)
    out = env_reset.reset()
    check("reset(): retries after a recovered trip", out[0] is obs_sentinel)
    check("reset(): the retry actually re-ran the reset", env_reset.reset_calls == 2)

    # ...but a configuration that keeps tripping is the LOG-040 §40.1 case, and
    # must end up in front of a human rather than looping.
    env_stuck = make_env(trips=99)
    try:
        env_stuck.reset()
        check("reset(): gives up after the attempt limit", False, "no exception")
    except MotionGuardTripped as exc:
        check(
            "reset(): gives up after the attempt limit",
            "enabling device" in str(exc),
            f"after {env_stuck.reset_calls} attempts",
        )
    check(
        "reset(): the attempt limit is respected",
        env_stuck.reset_calls == RESET_TRIP_ATTEMPTS,
        f"{env_stuck.reset_calls} of {RESET_TRIP_ATTEMPTS}",
    )


def test_controller_recovery_wiring() -> None:
    print("\n=== T21: controller-side recovery ordering ===")
    from franky_ext.controller_extended import FrankyControllerExtended

    check(
        "the controller exposes an explicit recovery API",
        hasattr(FrankyControllerExtended, "recover_from_guard_trip"),
    )
    src = inspect.getsource(FrankyControllerExtended.recover_from_guard_trip)

    # Ordering is the safety property here: the latch may only be cleared after
    # the arm is confirmed stopped and the fault confirmed cleared. Clearing it
    # first would re-authorise motion on top of a runaway.
    settle_at = src.find("_BRAKE_SETTLED_RAD_S")
    recover_at = src.find("recover_from_errors()")
    clear_at = src.find("self._guard_trip_reason = None")
    check("checks the arm has settled before clearing", 0 <= settle_at < clear_at)
    check("clears the Desk fault before clearing the latch", 0 <= recover_at < clear_at)
    check(
        "refuses while the arm is still moving",
        "refusing to clear a trip on a moving arm" in src,
    )
    check("never raises (returns a refusal report)", "raise" not in src.split('"""')[2])

    # The fence must survive recovery: resuming without one would be worse than
    # not resuming at all.
    check(
        "does not disarm the fence",
        "_guard_enabled = False" not in src,
    )
    check(
        "refuses to recover when no fence is armed",
        "the motion guard is not armed" in src,
    )


def test_real(robot_ip: str) -> None:
    print("\n=== real Jacobians (READ-ONLY, no motion commanded) ===")
    import franky

    robot = franky.Robot(robot_ip)
    state = robot.state
    model = robot.model
    frame = franky.Frame.EndEffector
    F_T_EE, EE_T_K = state.F_T_EE, state.EE_T_K
    print(f"  connected to {robot_ip}")

    def sigma_at(q):
        J = np.asarray(
            model.zero_jacobian(frame, np.asarray(q), F_T_EE, EE_T_K), dtype=np.float64
        ).reshape(6, 7)
        return jacobian_conditioning(J)

    thr = sigma_min_warn()
    for label, q, expected, should_warn in (
        ("hover (LOG-037, known-good)", HOVER_Q, SIGMA_HOVER, False),
        ("recovered (LOG-040 §40.2)", RECOVERED_Q, 0.1141, False),
        ("trip (LOG-036)", TRIP_Q, SIGMA_TRIP, True),
        ("drifted (LOG-040 §40.1)", DRIFT_Q, SIGMA_DRIFT, True),
    ):
        cond = sigma_at(q)
        warns = cond["sigma_min"] < thr
        print(
            f"  {label}: sigma_min={cond['sigma_min']:.4f} "
            f"(logged {expected}), cond={cond['cond']:.1f}"
        )
        check(
            f"{label}: matches the value the LOG recorded",
            abs(cond["sigma_min"] - expected) < max(0.002, expected * 0.1),
        )
        check(
            f"{label}: verdict is {'warn' if should_warn else 'ok'}",
            warns == should_warn,
        )

    live = sigma_at(np.asarray(state.q, dtype=np.float64))
    print(
        f"  live: q={np.round(np.asarray(state.q, dtype=np.float64), 4).tolist()} "
        f"sigma_min={live['sigma_min']:.4f} cond={live['cond']:.1f} -> "
        f"{'WARN' if live['sigma_min'] < thr else 'ok'}"
    )


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--robot",
        action="store_true",
        help="also evaluate real Jacobians (connects read-only; commands no motion)",
    )
    p.add_argument("--robot-ip", default=os.environ.get("FRANKA_ROBOT_IP", "172.16.0.2"))
    args = p.parse_args()

    print(
        f"T21/T22 acceptance: sigma_min warn < {sigma_min_warn():.4f}, "
        f"recovery budget = {guard_recovery_budget()} trips per env"
    )
    test_conditioning_math()
    test_threshold_placement()
    test_recovery_budget()
    test_self_check_behaviour()
    test_step_reset_wiring()
    test_controller_recovery_wiring()
    if args.robot:
        test_real(args.robot_ip)
    else:
        print("\n(skipping real-Jacobian checks; pass --robot to run them)")

    print()
    if _failures:
        print(f"FAILED {len(_failures)}: {', '.join(_failures)}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
