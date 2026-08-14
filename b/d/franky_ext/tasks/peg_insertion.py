"""Franky-backed PegInsertion env."""

from franky_ext.franky_single_franka_env import FrankySingleFrankaEnvMixin
from rlinf.envs.realworld.franka.tasks.peg_insertion_env import PegInsertionEnv


class FrankyPegInsertionEnv(FrankySingleFrankaEnvMixin, PegInsertionEnv):
    pass
