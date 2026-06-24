"""Unit tests for ModelEMA (openpi_au/ema.py)."""

import sys
from pathlib import Path

import pytest
import torch
import torch.nn as nn

# Allow direct import of the module without pulling in the entire rlinf dep chain
_mod_path = str(Path(__file__).resolve().parents[2] / "rlinf" / "models" / "embodiment" / "openpi_au")
if _mod_path not in sys.path:
    sys.path.insert(0, _mod_path)

from ema import ModelEMA


class TinyModel(nn.Module):
    def __init__(self):
        super().__init__()
        self.fc = nn.Linear(4, 2, bias=True)
        self.frozen = nn.Linear(2, 1, bias=False)
        self.frozen.weight.requires_grad = False

    def forward(self, x):
        return self.frozen(self.fc(x))


@pytest.fixture
def model():
    torch.manual_seed(42)
    return TinyModel()


def test_only_trainable_tracked(model):
    ema = ModelEMA(model, decay=0.99)
    assert "fc.weight" in ema.shadow
    assert "fc.bias" in ema.shadow
    assert "frozen.weight" not in ema.shadow


def test_ema_formula(model):
    decay = 0.9
    ema = ModelEMA(model, decay=decay)
    old_shadow_w = ema.shadow["fc.weight"].clone()
    with torch.no_grad():
        model.fc.weight.add_(torch.ones_like(model.fc.weight))
    ema.update(model)
    expected = decay * old_shadow_w + (1 - decay) * model.fc.weight.detach()
    torch.testing.assert_close(ema.shadow["fc.weight"], expected)


def test_multi_update(model):
    ema = ModelEMA(model, decay=0.99)
    for _ in range(10):
        with torch.no_grad():
            model.fc.weight.add_(0.1)
        ema.update(model)
    assert ema.num_updates == 10
    final_param = model.fc.weight.detach()
    assert (ema.shadow["fc.weight"] - final_param).abs().mean() > 0


def test_swap_roundtrip(model):
    ema = ModelEMA(model, decay=0.99)
    with torch.no_grad():
        model.fc.weight.add_(5.0)
    ema.update(model)

    original_params = {n: p.detach().clone() for n, p in model.named_parameters()}
    backup = ema.swap_in(model)

    torch.testing.assert_close(model.fc.weight, ema.shadow["fc.weight"])
    torch.testing.assert_close(backup["fc.weight"], original_params["fc.weight"])

    ema.swap_out(model, backup)
    torch.testing.assert_close(model.fc.weight, original_params["fc.weight"])


def test_state_dict_roundtrip(model):
    ema = ModelEMA(model, decay=0.999)
    with torch.no_grad():
        model.fc.weight.add_(1.0)
    ema.update(model)
    ema.update(model)

    sd = ema.state_dict()
    assert sd["decay"] == 0.999
    assert sd["num_updates"] == 2
    assert "fc.weight" in sd["shadow"]
    for v in sd["shadow"].values():
        assert v.device == torch.device("cpu")

    ema2 = ModelEMA(model, decay=0.5)
    ema2.load_state_dict(sd)
    assert ema2.decay == 0.999
    assert ema2.num_updates == 2
    torch.testing.assert_close(ema2.shadow["fc.weight"], ema.shadow["fc.weight"])


def test_zero_decay(model):
    ema = ModelEMA(model, decay=0.0)
    old = ema.shadow["fc.weight"].clone()
    with torch.no_grad():
        model.fc.weight.add_(1.0)
    ema.update(model)
    torch.testing.assert_close(ema.shadow["fc.weight"], model.fc.weight.detach())


@pytest.mark.gpu
def test_cpu_offload():
    model = TinyModel().cuda()
    ema = ModelEMA(model, decay=0.99, device=torch.device("cpu"))
    for v in ema.shadow.values():
        assert v.device == torch.device("cpu")
    with torch.no_grad():
        model.fc.weight.add_(1.0)
    ema.update(model)
    backup = ema.swap_in(model)
    assert model.fc.weight.device.type == "cuda"
    ema.swap_out(model, backup)


@pytest.mark.gpu
def test_large_model_smoke():
    big = nn.Sequential(nn.Linear(1024, 1024), nn.ReLU(), nn.Linear(1024, 1024)).cuda()
    ema = ModelEMA(big, decay=0.999)
    for _ in range(5):
        with torch.no_grad():
            for p in big.parameters():
                if p.requires_grad:
                    p.add_(torch.randn_like(p) * 0.01)
        ema.update(big)
    backup = ema.swap_in(big)
    ema.swap_out(big, backup)
