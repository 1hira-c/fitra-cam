"""Run the released indoor resolution, saving bidirectional fields for three-view geometry."""

import argparse
import gc
import importlib
import itertools
import json
import os
import time
from pathlib import Path
from unittest.mock import patch

DINOV3_REVISION = "adc254450203739c8149213a7a69d8d905b4fcfa"
BACKBONE = Path("hub") / f"facebookresearch_dinov3_{DINOV3_REVISION}"
CHECKPOINT = Path("hub/checkpoints/romav2.0.1.pt")
WEIGHTS_URL = (
    "https://github.com/Parskatt/RoMaV2/releases/download/v2.0.1/romav2.0.1.pt"
)


def load_model(trial):
    import torch
    from romav2 import RoMaV2

    hub_load = torch.hub.load

    def load_backbone(*, repo_or_dir, model, pretrained, weights, skip_validation):
        if (repo_or_dir, model, pretrained, weights) != (
            f"facebookresearch/dinov3:{DINOV3_REVISION}",
            "dinov3_vitl16",
            False,
            None,
        ):
            raise ValueError("Unexpected backbone requested by RoMa v2.0.1")
        return hub_load(
            str(trial / BACKBONE), model, source="local", pretrained=False, weights=None
        )

    def load_weights(url, map_location):
        if url != WEIGHTS_URL:
            raise ValueError("Unexpected checkpoint requested by RoMa v2.0.1")
        return torch.load(
            trial / CHECKPOINT, map_location="cpu", weights_only=True, mmap=True
        )

    with (
        patch.object(torch.hub, "load", load_backbone),
        patch.object(torch.hub, "load_state_dict_from_url", load_weights),
    ):
        model = RoMaV2(RoMaV2.Cfg(setting="scannet1500", compile=False))
    # Keep raw overlap probabilities for the geometric consistency check.
    model.threshold = None
    return model


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog="Set ROMA_TRIAL to your local model directory. For the checkpoint "
        "and Meta DINOv3 license terms, see THIRD_PARTY_NOTICES.md#roma-v2.",
    )
    parser.add_argument(
        "frames", type=Path, help="directory containing input0.png to input2.png"
    )
    args = parser.parse_args()
    trial_dir = os.environ.get("ROMA_TRIAL")
    if not trial_dir:
        parser.error(
            "set ROMA_TRIAL to the local RoMa v2.0.1 and DINOv3 assets (see THIRD_PARTY_NOTICES.md#roma-v2)"
        )
    trial = Path(trial_dir).expanduser()
    for asset in (trial / CHECKPOINT, trial / BACKBONE / "hubconf.py"):
        if not asset.is_file():
            parser.error(
                f"missing local file {asset} (see THIRD_PARTY_NOTICES.md#roma-v2)"
            )

    import numpy as np
    import torch
    from tiled_correlation import tiled, check

    if not torch.cuda.is_available():
        parser.error("RoMa v2 calibration requires CUDA")
    corr = importlib.import_module("romav2.local_correlation")
    torch.manual_seed(66)
    torch.cuda.manual_seed_all(66)
    print(
        torch.__version__, torch.version.cuda, torch.cuda.get_device_name(), flush=True
    )
    error = check(corr.native_torch_local_corr)
    corr.native_torch_local_corr = tiled
    print("tiled correlation max absolute difference", error, flush=True)
    start = time.monotonic()
    model = load_model(trial)
    torch.cuda.synchronize()
    print("loaded", round(time.monotonic() - start, 2), flush=True)
    stats = {"tiled_reference_max_abs_difference": error}
    for i, j in itertools.combinations(range(3), 2):
        torch.cuda.reset_peak_memory_stats()
        start = time.monotonic()
        with torch.inference_mode():
            pred = model.match(
                args.frames / f"input{i}.png", args.frames / f"input{j}.png"
            )
        torch.cuda.synchronize()
        stats[f"{i}{j}"] = {
            "seconds": time.monotonic() - start,
            "peak_allocated_MiB": torch.cuda.max_memory_allocated() / 2**20,
        }
        print(i, j, stats[f"{i}{j}"], flush=True)
        fields = {
            k: pred[k][0].float().cpu().numpy()
            for k in ("warp_AB", "warp_BA", "overlap_AB", "overlap_BA")
        }
        np.savez(args.frames / f"fields{i}{j}.npz", **fields)
        del pred, fields
        gc.collect()
        torch.cuda.empty_cache()
    (args.frames / "gpu.json").write_text(json.dumps(stats, indent=2) + "\n")


if __name__ == "__main__":
    main()
