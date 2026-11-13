"""GPU memory preflight for the reconstruction scripts. It reads nvidia-smi, so a script that must not create
a torch CUDA context (fuse.py) can call it too."""
from __future__ import annotations

import subprocess
import sys

MIN_FREE_MB = 7000


class VramError(RuntimeError):
    pass


def parse_smi_free(text: str) -> int:
    return int(text.strip().splitlines()[0].strip())


def free_vram_mb(device: int = 0) -> int:
    out = subprocess.run(["nvidia-smi", "--query-gpu=memory.free", "--format=csv,noheader,nounits", "-i",
                          str(device)], capture_output=True, text=True, check=True).stdout
    return parse_smi_free(out)


def require_free_vram(min_mb: int = MIN_FREE_MB, device: int = 0) -> int:
    free = free_vram_mb(device)
    if free < min_mb:
        raise VramError(f"{free} MiB free, {min_mb} MiB needed on GPU {device}: another process holds it "
                        "(nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv)")
    return free


def peak_vram_mb() -> int | None:
    torch = sys.modules.get("torch")
    if torch is None or not torch.cuda.is_initialized():
        return None
    return int(torch.cuda.max_memory_allocated() // (1 << 20))
