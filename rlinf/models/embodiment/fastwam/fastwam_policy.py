import torch

from rlinf.models.embodiment.base_policy import BasePolicy, ForwardType


class FastWAMPolicy(torch.nn.Module, BasePolicy):
    _no_split_modules = ["DiTBlock"]

    def __init__(self, fastwam_model, config):
        torch.nn.Module.__init__(self)
        self.fastwam = fastwam_model
        self.config = config

    def forward(self, forward_type=ForwardType.DEFAULT, **kwargs):
        if forward_type == ForwardType.SFT:
            return self.sft_forward(**kwargs)
        elif forward_type == ForwardType.DEFAULT:
            return self.default_forward(**kwargs)
        raise NotImplementedError(f"Unsupported forward type: {forward_type}")

    def sft_forward(self, data=None, **kwargs):
        torch.compiler.cudagraph_mark_step_begin()
        if data is None:
            data = kwargs.get("data")
        param_device = next(self.fastwam.mot.parameters()).device
        if self.fastwam.device != param_device:
            self.fastwam.device = param_device
        loss_total, loss_dict = self.fastwam.training_loss(data)
        return {
            "loss": loss_total,
            "dynamics_loss": torch.tensor(loss_dict.get("loss_video", 0.0)),
            "action_loss": torch.tensor(loss_dict.get("loss_action", 0.0)),
        }

    def train(self, mode=True):
        if mode:
            self.fastwam.eval()
            self.fastwam.requires_grad_(False)
            self.fastwam.dit.train()
            self.fastwam.dit.requires_grad_(True)
            if self.fastwam.proprio_encoder is not None:
                self.fastwam.proprio_encoder.train()
                self.fastwam.proprio_encoder.requires_grad_(True)
        else:
            self.fastwam.eval()
        return self

    def default_forward(self, **kwargs):
        raise NotImplementedError("V1 does not support default_forward.")

    def predict_action_batch(self, **kwargs):
        raise NotImplementedError("V1 does not support rollout inference.")

    def gradient_checkpointing_enable(self, gradient_checkpointing_kwargs=None):
        self.fastwam.video_expert.use_gradient_checkpointing = True
        self.fastwam.action_expert.use_gradient_checkpointing = True
