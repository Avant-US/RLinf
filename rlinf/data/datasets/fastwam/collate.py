import numpy as np
import torch


def fastwam_collate_fn(features):
    batch = {}
    for key in features[0]:
        values = [f[key] for f in features]
        if isinstance(values[0], torch.Tensor):
            batch[key] = torch.stack(values)
        elif isinstance(values[0], np.ndarray):
            batch[key] = torch.from_numpy(np.stack(values))
        else:
            batch[key] = values
    return batch
