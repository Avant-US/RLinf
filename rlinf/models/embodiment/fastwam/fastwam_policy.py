import torch

from rlinf.models.embodiment.base_policy import BasePolicy, ForwardType


class FastWAMPolicy(torch.nn.Module, BasePolicy):
    # MoT reads block.modulation outside DiTBlock.forward(); per-block FSDP
    # wrapping would expose sharded/empty params. Keep this None so
    # get_fsdp_wrap_policy returns no auto_wrap_policy (single root FSDP).
    #@# So change `_no_split_modules = ["DiTBlock"]` into below line
    _no_split_modules = None

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
            #@# 如果想用 _ExpertMixtures 解决多次引用 video_expert 和 action_expert 作为 nn.Module 的问题(见 FastWAM/ 的 fastwam2.py 和 mot2.py )
            # 请注释掉上面`self.fastwam.dit`相关代码块, 而使用如下代码块.
            # MoT stores experts in _ExpertMixtures (not nn.Module), so
            # dit.train()/requires_grad_() won't propagate to them.
            # for expert in [self.fastwam.video_expert, self.fastwam.action_expert]:
            #     expert.train()
            #     expert.requires_grad_(True)

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
