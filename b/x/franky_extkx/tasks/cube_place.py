"""Franky-backed cube-touch env: gripper stays closed, success is TCP at mark."""

from __future__ import annotations

import copy
import time
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation as R

from franky_ext.franky_single_franka_env import FrankySingleFrankaEnvMixin
from franky_ext.motion_limits import (
    interp_duration_s,
    max_action_scale_rot,
    max_action_scale_xyz,
    orientation_fence_rad,
    quat_angle_rad,
    step_rot_speed_rad_s,
    step_speed_m_s,
)
from rlinf.envs.realworld.franka.tasks.peg_insertion_env import (
    PegInsertionConfig,
    PegInsertionEnv,
)

#: Convergence tolerance for the final move to ``reset_ee_pose``, metres.
#:
#: Tighter than the smoke script's hover gate (0.03 xy / 0.025 z) so a PASS means
#: the arm actually arrived rather than merely landing inside the acceptance band.
REST_POSE_TOL_M = 0.01

#: Attempts at the rest move before giving up (upstream also uses 3).
REST_POSE_ATTEMPTS = 3

#: Placeholder IPs that mean "this config is not meant to reach a robot".
_PLACEHOLDER_IPS = ("0.0.0.0", "127.0.0.1", "localhost", "none", "")


@dataclass
class CubePlaceConfig(PegInsertionConfig):
    """Defaults from dmo_place_1.md §5: closed-gripper touch, not release."""

    task_description: str = "touch the marked place with a grasped cube"
    clip_x_range: float = 0.05
    clip_y_range: float = 0.05
    clip_z_range_low: float = 0.005
    clip_z_range_high: float = 0.08
    random_xy_range: float = 0.03
    random_rz_range: float = 0.35
    clip_rz_range: float = 0.35
    # NOTE: these two mirror FrankySingleFrankaEnvConfig. They are re-declared
    # rather than inherited because this class must derive from
    # PegInsertionConfig. Phase 1A asserts that the two stay in sync, so a new
    # safety field added there cannot silently be missing here.
    safe_smoke_hold: bool = False
    clear_error_per_waypoint: bool = True
    # Lift relative to the *current* TCP, to get the cube off the mark before
    # travelling to hover. NOT ``clip_z_range_high``.
    #
    # 0.03, not PegInsertion's 0.10: charger lifts 10 cm because it has to pull a
    # plug out of a socket, and this task only has to break contact with a flat
    # mark. The old 0.10 also pushed the lift target 10 cm above the safety-box
    # top, outside anything the geometry had authorised, and -- via
    # ``_interpolate_move(timeout=1)`` at ``step_frequency=10`` -- commanded it at
    # 10 cm/s, ten times the validated speed. That combination is LOG-019.
    reset_z_lift_m: float = 0.03

    def __post_init__(self):
        # PegInsertionConfig.__post_init__ derives reset_ee_pose, ee_pose_limit_*,
        # action_scale and the compliance dicts from target_ee_pose + the clip
        # ranges. Anything explicitly passed for those keys IS overwritten here --
        # that is upstream's design and it is the *tighter* choice (roll and pitch
        # get +/-0.01 rad rather than +/-clip_rz), so it is deliberately not
        # restored. What must not happen is code elsewhere printing the
        # pre-construction candidate as if it were enforced; see
        # tcp_probe.effective_ee_pose_limits.
        super().__post_init__()

        # The policy path never sees the interpolation speed cap: FrankaEnv.step
        # adds ``action[:3] * action_scale[0]`` to the measured pose and calls
        # _move_action directly. PegInsertion's action_scale[0]=0.02 at
        # step_frequency=10 is a **20 cm/s** command -- twice LOG-019's 10 cm/s and
        # 20x the validated 1 cm/s -- and nothing downstream throttled it:
        # _CART_MAX_STEP_M=0.10 does not bite at 2 cm, and because step recomputes
        # its target from the freshly measured pose the lag never accumulates, so
        # the guard's lag test cannot see it either.
        #
        # It is also above what the arm can achieve under the commanded force
        # ceiling (~13 cm/s terminal against ~155 N.s/m damping), so most of the
        # old action range was physically unreachable -- bad for the policy as well
        # as unsafe. Phase 3 should revisit the value with data.
        cap = max_action_scale_xyz(self.step_frequency)
        scale = np.asarray(self.action_scale, dtype=np.float64)
        if scale.size >= 1 and float(scale[0]) > cap:
            print(
                f"WARNING: action_scale[0]={float(scale[0]):.4f} m/step at "
                f"{self.step_frequency:.0f} Hz is "
                f"{float(scale[0]) * self.step_frequency * 100:.0f} cm/s; clamping to "
                f"{cap:.4f} m/step ({step_speed_m_s() * 100:.0f} cm/s). "
                "Set RLINF_CUBE_STEP_SPEED to change the cap."
            )
            scale[0] = cap
            self.action_scale = scale

        # LOG-034: the same gap, unnoticed until it tripped the motion guard.
        # action_scale[1]=0.1 rad/step at step_frequency=10 is a 1.0 rad/s per-axis
        # rotation command (1.73 rad/s worst-case 3-axis) -- nothing on the step()
        # path clamped it, and near a reach boundary that demands a joint velocity
        # the arm cannot supply. See motion_limits.STEP_ROT_SPEED_RAD_S_DEFAULT for
        # why this cap is provisional pending a measured 2.4b-rot ladder.
        rot_cap = max_action_scale_rot(self.step_frequency)
        if scale.size >= 2 and float(scale[1]) > rot_cap:
            print(
                f"WARNING: action_scale[1]={float(scale[1]):.4f} rad/step at "
                f"{self.step_frequency:.0f} Hz is "
                f"{float(scale[1]) * self.step_frequency:.2f} rad/s; clamping to "
                f"{rot_cap:.4f} rad/step ({step_rot_speed_rad_s():.2f} rad/s). "
                "Set RLINF_CUBE_STEP_ROT_SPEED to change the cap."
            )
            scale[1] = rot_cap
            self.action_scale = scale

        ip = str(self.robot_ip).strip().lower() if self.robot_ip is not None else "none"
        placeholder = ip in _PLACEHOLDER_IPS
        if not self.is_dummy and placeholder:
            raise ValueError(
                f"is_dummy=False but robot_ip={self.robot_ip!r} is a placeholder. "
                "Set a real robot IP, or set is_dummy=True."
            )
        if self.is_dummy and not placeholder:
            # Harmless (dummy never connects) but it means someone edited one of
            # the pair and not the other, and is_dummy is the ONLY thing keeping
            # the 1A regression off the hardware.
            print(
                f"WARNING: CubePlaceConfig has is_dummy=True with a real "
                f"robot_ip={self.robot_ip!r}; these should agree."
            )


class FrankyCubePlaceEnv(FrankySingleFrankaEnvMixin, PegInsertionEnv):
    """Closed-gripper cube touch on Franky (PegInsertion-style lift, not open)."""

    CONFIG_CLS = CubePlaceConfig

    def _check_start_pose(self) -> None:
        """Warn when reset starts from outside the box it is supposed to live in.

        The safety box floor is ``target_z - clip_z_range_low``. LOG-019 started
        6.6 mm *below* that, i.e. with the cube pressed into the mark, because the
        manual's "guide the arm a few cm above the mark" step was skipped and
        nothing checked it. This is a warning rather than a refusal because during
        training a millimetre of sag below the floor after a successful touch is
        normal; the hard refusal belongs in the smoke script, where a human is
        present and can fix the setup.
        """
        target = np.asarray(self.config.target_ee_pose, dtype=np.float64)
        tcp = np.asarray(self._franka_state.tcp_pose[:3], dtype=np.float64)
        floor = float(target[2]) - float(self.config.clip_z_range_low)
        top = float(target[2]) + float(self.config.clip_z_range_high)
        if tcp[2] < floor - 1e-4:
            self._logger.warning(
                "reset starts %.4fm BELOW the box floor (z=%.4f < %.4f): the cube "
                "is pressed into the mark. Guide the arm a few cm above the mark "
                "before reset (dmo_place_2 §2.1 rule 3).",
                floor - float(tcp[2]),
                float(tcp[2]),
                floor,
            )
        elif tcp[2] > top + 1e-4:
            self._logger.warning(
                "reset starts %.4fm ABOVE the box top (z=%.4f > %.4f); the lift "
                "will be commanded from there.",
                float(tcp[2]) - top,
                float(tcp[2]),
                top,
            )
        xy_err = float(np.linalg.norm(tcp[:2] - target[:2]))
        box_xy = float(max(self.config.clip_x_range, self.config.clip_y_range))
        if xy_err > box_xy:
            self._logger.warning(
                "reset starts %.4fm from the mark in xy, outside the %.3fm box; "
                "the arm will travel there under impedance. Guide it closer first.",
                xy_err,
                box_xy,
            )
        # Orientation matters as much as position here, because the very first
        # command of go_to_rest is "stay exactly where you are" -- so a wrist far
        # from the target orientation trips the guard's orientation fence with a
        # motionless arm and a message about whipping.
        lim_min = np.asarray(self.config.ee_pose_limit_min, dtype=np.float64).reshape(-1)
        lim_max = np.asarray(self.config.ee_pose_limit_max, dtype=np.float64).reshape(-1)
        if lim_min.size >= 6 and lim_max.size >= 6:
            halves = [float(abs(lim_max[3 + i] - lim_min[3 + i]) / 2.0) for i in range(3)]
            fence = orientation_fence_rad(*halves)
            live_quat = np.asarray(self._franka_state.tcp_pose[3:], dtype=np.float64)
            target_quat = R.from_euler("xyz", target[3:6]).as_quat()
            ang = quat_angle_rad(live_quat, target_quat)
            if ang > fence * 0.8:
                self._logger.warning(
                    "reset starts %.3frad (%.0f deg) from the target orientation, "
                    "against a %.3frad fence: the guard may abort before the arm "
                    "moves. Re-orient the wrist with the enabling device.",
                    ang,
                    ang * 180.0 / np.pi,
                    fence,
                )

    def _go_to_rest_pose(self, joint_reset: bool) -> None:
        """Move to ``reset_ee_pose``, converging on an absolute tolerance.

        Replaces ``FrankaEnv.go_to_rest``'s loop, whose condition is
        ``np.allclose(tcp[:3], reset_pose[:3], 0.02)`` -- and ``np.allclose``'s
        third positional argument is **rtol**, not atol. With ``target_y ~ 0`` the
        tolerance on that axis collapses to ``atol=1e-8``, so the loop always runs
        its full 3 iterations. That was merely wasteful upstream; with the
        displacement-derived duration it costs two extra ~1.5 s interpolations per
        reset that re-command a pose the arm is already at, which in a training run
        is minutes of nothing and looks like a hang.

        Everything else upstream does here is reproduced, except the dexterous-hand
        branch, which cannot apply to a parallel-jaw gripper.
        """
        if joint_reset:
            self._controller.reset_joint(self.config.joint_reset_qpos).wait()
            time.sleep(0.5)

        reset_pose = self._reset_pose.copy()
        if self.config.enable_random_reset:
            reset_pose[:2] += np.random.uniform(
                -self.config.random_xy_range, self.config.random_xy_range, (2,)
            )
            euler_random = np.asarray(
                self.config.target_ee_pose[3:], dtype=np.float64
            ).copy()
            euler_random[-1] += np.random.uniform(
                -self.config.random_rz_range, self.config.random_rz_range
            )
            reset_pose[3:] = R.from_euler("xyz", euler_random).as_quat()

        for attempt in range(1, REST_POSE_ATTEMPTS + 1):
            self._franka_state = self._controller.get_state().wait()[0]
            err = float(
                np.linalg.norm(
                    np.asarray(self._franka_state.tcp_pose[:3], dtype=np.float64)
                    - np.asarray(reset_pose[:3], dtype=np.float64)
                )
            )
            if err <= REST_POSE_TOL_M:
                self._logger.info(
                    "rest pose reached on attempt %d (err=%.4fm <= %.4fm)",
                    attempt,
                    err,
                    REST_POSE_TOL_M,
                )
                return
            self._logger.info(
                "rest move attempt %d/%d: err=%.4fm -> %s",
                attempt,
                REST_POSE_ATTEMPTS,
                err,
                np.round(reset_pose[:3], 4).tolist(),
            )
            self._interpolate_move(reset_pose)

        # Re-check after the LAST move: the loop tests before moving, so without this
        # a successful final attempt still logged "NOT reached".
        self._franka_state = self._controller.get_state().wait()[0]
        final = float(
            np.linalg.norm(
                np.asarray(self._franka_state.tcp_pose[:3], dtype=np.float64)
                - np.asarray(reset_pose[:3], dtype=np.float64)
            )
        )
        if final <= REST_POSE_TOL_M:
            self._logger.info(
                "rest pose reached on the final attempt (err=%.4fm <= %.4fm)",
                final,
                REST_POSE_TOL_M,
            )
            return
        self._logger.warning(
            "rest pose NOT reached after %d attempts: err=%.4fm > %.4fm. If the arm "
            "was still creeping toward it, the impedance is tracking but slowly: "
            "check for a 'step slew clamped' flood (that clamp must not run during "
            "interpolation) and raise --force-ceiling if lag is the limit.",
            REST_POSE_ATTEMPTS,
            final,
            REST_POSE_TOL_M,
        )

    def go_to_rest(self, joint_reset=False):
        """PegInsertionEnv / charger sequence, gripper stays closed.

        1. Close gripper (``-1``).
        2. Sanity-check the start pose (warn only; the smoke script refuses).
        3. ``_move_action(current)`` so cartesian impedance tracks *here*
           (ROS publishes equilibrium; franky starts the tracker after
           ``reconfigure_compliance_params`` stopped it).
        4. Lift ``reset_z_lift_m`` along **current** TCP z, same orientation.
           This is not ``target + clip_z_range_high``.
        5. Move to ``reset_ee_pose`` (target + ``clip_z_range_high``).

        Steps 4 and 5 both go through the mixin's speed- and distance-capped
        ``_interpolate_move``, and every waypoint plus every ``get_state`` is
        fenced by the controller's motion guard (with a watchdog covering the gaps
        in between), so neither the amplitude nor the speed of this sequence is set
        by a hardcoded constant any more.
        """
        self._end_effector_action(np.array([-1.0]))
        self._franka_state = self._controller.get_state().wait()[0]
        self._check_start_pose()
        self._move_action(self._franka_state.tcp_pose)
        self._franka_state = self._controller.get_state().wait()[0]
        before = np.asarray(self._franka_state.tcp_pose[:3], dtype=np.float64)
        reset_pose = copy.deepcopy(self._franka_state.tcp_pose)
        z_lift = float(getattr(self.config, "reset_z_lift_m", 0.03))
        reset_pose[2] += z_lift
        lift_s = interp_duration_s(self._franka_state.tcp_pose, reset_pose)
        self._logger.info(
            "cube_place go_to_rest: current %s +z=%.3f -> %s over %.2fs; "
            "then rest target+clip_z_high",
            np.round(before, 4).tolist(),
            z_lift,
            np.round(reset_pose[:3], 4).tolist(),
            lift_s,
        )
        self._interpolate_move(reset_pose, timeout=lift_s)
        self._go_to_rest_pose(joint_reset)
        self._franka_state = self._controller.get_state().wait()[0]
        after = np.asarray(self._franka_state.tcp_pose[:3], dtype=np.float64)
        self._logger.info(
            "cube_place go_to_rest done: tcp_xyz=%s dz=%.4f",
            np.round(after, 4).tolist(),
            float(after[2] - before[2]),
        )
