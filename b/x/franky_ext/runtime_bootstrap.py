"""Runtime fixes for franky venv inside Docker (Step 7+ CPU / NO_ACCEL)."""

from __future__ import annotations

import sys as _sys

import os

import torch.library

_real_register_fake = torch.library.register_fake


def _safe_register_fake(*args, **kwargs):
    if len(args) == 2 and callable(args[1]):
        try:
            return _real_register_fake(args[0])(args[1])
        except RuntimeError:
            return args[1]
    if len(args) == 1 and isinstance(args[0], str):
        op_name = args[0]

        def decorator(fn):
            try:
                return _real_register_fake(op_name)(fn)
            except RuntimeError:
                return fn

        return decorator
    return _real_register_fake(*args, **kwargs)


torch.library.register_fake = _safe_register_fake  # type: ignore[misc]


class _DummyStream:
    @staticmethod
    def synchronize() -> None:
        return None


class _CpuTorchPlatform:
    """Minimal torch.cuda-like API for NO_ACCEL / CPU-only Step 7 runs."""

    @staticmethod
    def is_available() -> bool:
        return True

    @staticmethod
    def is_initialized() -> bool:
        return True

    @staticmethod
    def current_device():
        import torch

        return torch.device("cpu")

    @staticmethod
    def set_device(_device) -> None:
        return None

    @staticmethod
    def empty_cache() -> None:
        return None

    @staticmethod
    def synchronize() -> None:
        return None

    @staticmethod
    def ipc_collect() -> None:
        return None

    @staticmethod
    def current_stream(device=None):
        return _DummyStream()

    @staticmethod
    def get_rng_state():
        import torch

        return torch.get_rng_state()

    @staticmethod
    def set_rng_state(state) -> None:
        import torch

        torch.set_rng_state(state)

    @staticmethod
    def device_count() -> int:
        return 0


def _patch_no_accel_platform() -> None:
    from rlinf.scheduler.hardware.accelerators.accelerator import (
        AcceleratorType,
        AcceleratorUtil,
    )

    _orig_get_torch_platform = AcceleratorUtil.get_torch_platform
    _orig_get_device_type = AcceleratorUtil.get_device_type

    @staticmethod
    def get_torch_platform(accelerator_type):
        if accelerator_type == AcceleratorType.NO_ACCEL:
            return _CpuTorchPlatform
        return _orig_get_torch_platform(accelerator_type)

    @staticmethod
    def get_device_type(accelerator_type):
        if accelerator_type == AcceleratorType.NO_ACCEL:
            return "cpu"
        return _orig_get_device_type(accelerator_type)

    AcceleratorUtil.get_torch_platform = get_torch_platform  # type: ignore[assignment]
    AcceleratorUtil.get_device_type = get_device_type  # type: ignore[assignment]

    from rlinf.scheduler.worker import worker as worker_mod

    accel_type = AcceleratorUtil.get_accelerator_type()
    worker_mod.Worker.torch_platform = AcceleratorUtil.get_torch_platform(accel_type)
    worker_mod.Worker.torch_device_type = AcceleratorUtil.get_device_type(accel_type)


def _patch_worker_env_setup() -> None:
    """Re-apply torch_platform on worker instances after RLINF_EXT_MODULE register()."""
    from rlinf.scheduler.hardware.accelerators.accelerator import AcceleratorUtil
    from rlinf.scheduler.worker import worker as worker_mod

    if getattr(worker_mod.Worker, "_franky_cpu_platform_patched", False):
        return

    orig = worker_mod.Worker._env_setup_before_init

    def patched_env_setup(self):
        orig(self)
        self.torch_platform = AcceleratorUtil.get_torch_platform(self._accelerator_type)
        self.torch_device_type = AcceleratorUtil.get_device_type(self._accelerator_type)
        worker_mod.Worker.torch_platform = self.torch_platform
        worker_mod.Worker.torch_device_type = self.torch_device_type

    worker_mod.Worker._env_setup_before_init = patched_env_setup
    worker_mod.Worker._franky_cpu_platform_patched = True


def _patch_fsdp_for_cpu_smoke() -> None:
    """Skip CUDA FSDP wrap when ACCELERATOR_TYPE=NO_ACCEL (Step 7 CPU smoke)."""
    try:
        from rlinf.hybrid_engines.fsdp.strategy.fsdp import FSDPStrategy
    except ModuleNotFoundError:
        return

    if getattr(FSDPStrategy, "_franky_cpu_smoke_patched", False):
        return

    orig_wrap = FSDPStrategy.wrap_model

    def wrap_model(self, model, device_mesh):
        import os
        from rlinf.scheduler.hardware.accelerators.accelerator import AcceleratorType

        if os.environ.get("ACCELERATOR_TYPE") == AcceleratorType.NO_ACCEL.value:
            return model.to("cpu")
        return orig_wrap(self, model, device_mesh)

    FSDPStrategy.wrap_model = wrap_model  # type: ignore[method-assign]
    FSDPStrategy._franky_cpu_smoke_patched = True


def _patch_pin_memory_for_cpu() -> None:
    import torch

    if getattr(torch.Tensor, "_franky_pin_memory_patched", False):
        return
    orig = torch.Tensor.pin_memory

    def pin_memory(self, device=None):
        try:
            return orig(self, device)
        except RuntimeError:
            return self

    torch.Tensor.pin_memory = pin_memory  # type: ignore[method-assign]
    torch.Tensor._franky_pin_memory_patched = True


def register() -> None:
    """RLINF_EXT_MODULE hook: patch NO_ACCEL torch platform in Ray workers."""
    _patch_no_accel_platform()
    _patch_worker_env_setup()
    _patch_fsdp_for_cpu_smoke()
    _patch_pin_memory_for_cpu()
    from rlinf.scheduler.hardware.accelerators.accelerator import (
        AcceleratorType,
        AcceleratorUtil,
    )
    from rlinf.scheduler.worker import worker as worker_mod

    accel_type = AcceleratorType(
        os.environ.get("ACCELERATOR_TYPE", str(AcceleratorType.NO_ACCEL.value))
    )
    worker_mod.Worker.torch_platform = AcceleratorUtil.get_torch_platform(accel_type)
    worker_mod.Worker.torch_device_type = AcceleratorUtil.get_device_type(accel_type)

    try:
        import franky_ext.tasks.register  # noqa: F401
    except Exception:
        # Reported, not swallowed: an unregistered gym id fails safe (it cannot
        # command a robot), but the failure used to surface much later as an
        # unrelated gym.make error.
        import traceback as _tb

        print(
            "runtime_bootstrap: franky gym registration FAILED inside the worker "
            "env patch:\n" + _tb.format_exc(),
            file=_sys.stderr,
        )


_patch_no_accel_platform()
_patch_worker_env_setup()
try:
    _patch_fsdp_for_cpu_smoke()
    _patch_pin_memory_for_cpu()
except Exception:
    pass

try:
    import franky_ext.tasks.register  # noqa: F401
except Exception:
    import traceback as _tb

    print(
        "runtime_bootstrap: franky gym registration FAILED at import:\n"
        + _tb.format_exc(),
        file=_sys.stderr,
    )
