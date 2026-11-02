"""Select inference hardware once per compute process; preserve precision settings."""
from functools import lru_cache


@lru_cache(maxsize=1)
def inference_device():
    try:
        import torch
    except ImportError:
        return 'cpu'
    return 'cuda:0' if torch.cuda.is_available() else 'cpu'
