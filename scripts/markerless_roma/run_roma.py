"""Run the released indoor resolution, saving bidirectional fields for three-view geometry."""

import argparse
import gc
import itertools
import json
import os
import time
from pathlib import Path
from unittest.mock import patch

import importlib
import numpy as np
from tiled_correlation import tiled, check
import torch
from romav2 import RoMaV2


def load_model(trial):
    # Use only the verified local release and author's pinned backbone definition.
    tag = "adc254450203739c8149213a7a69d8d905b4fcfa"
    checkpoint = trial / "hub/checkpoints/romav2.0.1.pt"
    repo = trial / ("hub/facebookresearch_dinov3_" + tag)
    hub_load = torch.hub.load

    def load_backbone(*, repo_or_dir, model, pretrained, weights, skip_validation):
        assert (
            repo_or_dir == "facebookresearch/dinov3:" + tag
            and not pretrained
            and weights is None
        )
        return hub_load(
            str(repo), model, source="local", pretrained=False, weights=None
        )

    def load_weights(url, map_location):
        assert url.endswith("/v2.0.1/romav2.0.1.pt")
        return torch.load(checkpoint, map_location="cpu", weights_only=True, mmap=True)

    with (
        patch.object(torch.hub, "load", load_backbone),
        patch.object(torch.hub, "load_state_dict_from_url", load_weights),
    ):
        model = RoMaV2(RoMaV2.Cfg(setting="scannet1500", compile=False))
    # Keep raw overlap probabilities for the geometric consistency check.
    model.threshold = None
    return model


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("frames", type=Path)
    args = parser.parse_args()
    assert torch.cuda.is_available()
    torch.manual_seed(66)
    torch.cuda.manual_seed_all(66)
    print(
        torch.__version__, torch.version.cuda, torch.cuda.get_device_name(), flush=True
    )
    corr = importlib.import_module("romav2.local_correlation")
    error = check(corr.native_torch_local_corr)
    corr.native_torch_local_corr = tiled
    print("tiled correlation max absolute difference", error, flush=True)
    start = time.monotonic()
    model = load_model(Path(os.environ["ROMA_TRIAL"]))
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
