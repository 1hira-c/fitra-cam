"""Same per-pixel correlation as upstream, with bounded workspace on the Orin."""

import torch
from torch.nn import functional as F


def tiled(
    feature0,
    feature1,
    warp,
    local_window,
    B,
    K,
    c,
    r,
    h,
    w,
    device,
    padding_mode="zeros",
    sample_mode="bilinear",
    dtype=torch.float32,
):
    rows = max(1, min(h, (24 * 1024**2) // (w * K * c * feature1.element_size())))
    corr = torch.empty((B, K, h, w), device=device, dtype=dtype)
    for batch in range(B):
        for top in range(0, h, rows):
            bottom = min(h, top + rows)
            coords = (
                warp[batch, top:bottom, :, None] + local_window[:, None, None]
            ).reshape(1, bottom - top, w * K, 2)
            sampled = F.grid_sample(
                feature1[batch : batch + 1],
                coords,
                padding_mode=padding_mode,
                align_corners=False,
                mode=sample_mode,
            )
            sampled = sampled.reshape(c, bottom - top, w, K)
            values = (feature0[batch, :, top:bottom, :, None] / (c**0.5) * sampled).sum(
                0
            )
            corr[batch, :, top:bottom] = values.permute(2, 0, 1)
    return corr


def check(reference):
    device = torch.device("cuda")
    b, c, h, w, k = 1, 72, 71, 73, 49
    a = torch.randn(b, c, h, w, device=device)
    v = torch.randn_like(a)
    p = torch.rand(b, h, w, 2, device=device) * 2 - 1
    offsets = torch.randn(b, k, 2, device=device) / 100
    args = (a, v, p, offsets, b, k, c, 3, h, w, device)
    expected, actual = reference(*args), tiled(*args)
    torch.testing.assert_close(actual, expected, rtol=1e-5, atol=3e-6)
    return float((actual - expected).abs().max())
