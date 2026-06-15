// 2D pose overlay rendering for a single camera canvas. Ported from drawCamera
// in the legacy viewer.

import { CAM_COLORS, KP_THR, skeletonFor } from "./skeleton";
import type { CameraBundle, KpFormat } from "../types/bundle";

export function drawCamera(
  canvas: HTMLCanvasElement,
  bundle: CameraBundle | undefined,
  kpFormat: KpFormat,
): void {
  const ctx = canvas.getContext("2d");
  if (!ctx) return;
  // Resize BEFORE any painting: assigning canvas.width/height clears the bitmap
  // and resets the 2D context state, so doing it after the black fill would wipe
  // the background on every frame the dimensions change.
  if (bundle && (canvas.width !== bundle.w || canvas.height !== bundle.h)) {
    canvas.width = bundle.w;
    canvas.height = bundle.h;
  }
  ctx.fillStyle = "#000";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  if (!bundle) {
    ctx.fillStyle = "#666";
    ctx.font = "16px monospace";
    ctx.fillText("(no data)", 16, 32);
    return;
  }
  const color = CAM_COLORS[bundle.id % CAM_COLORS.length];
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 2;
  const skeleton = skeletonFor(kpFormat);
  for (const person of bundle.persons || []) {
    if (person.bbox) {
      const [x1, y1, x2, y2] = person.bbox;
      ctx.strokeStyle = "#444";
      ctx.lineWidth = 1;
      ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
    }
    const kpts = person.kpts || [];
    for (const [a, b] of skeleton) {
      if (!kpts[a] || !kpts[b]) continue;
      if (kpts[a][2] < KP_THR || kpts[b][2] < KP_THR) continue;
      ctx.beginPath();
      ctx.moveTo(kpts[a][0], kpts[a][1]);
      ctx.lineTo(kpts[b][0], kpts[b][1]);
      ctx.stroke();
    }
    for (const kp of kpts) {
      if (!kp || kp[2] < KP_THR) continue;
      ctx.beginPath();
      ctx.arc(kp[0], kp[1], 3, 0, Math.PI * 2);
      ctx.fill();
    }
  }
}
