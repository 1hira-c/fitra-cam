// Keypoint topology + drawing constants shared by the 2D canvas and the 3D
// viewer. Ported verbatim from the legacy web/dual_rtmpose/app.js.

import type { KpFormat } from "../types/bundle";

export const CAM_COLORS = ["#00dc00", "#ffb400", "#48aaff", "#ff6f6f"];
export const PERSON_3D_COLORS = ["#ff4cff", "#48aaff", "#ffd166", "#5cff8d"];
export const KP_THR = 0.3;

// COCO17 edge table — the original Phase 6 viewer expected exactly these
// keypoints.
export const SKELETON_COCO17: Array<[number, number]> = [
  [0, 1], [0, 2], [1, 3], [2, 4],
  [5, 7], [7, 9], [6, 8], [8, 10],
  [5, 6], [5, 11], [6, 12], [11, 12],
  [11, 13], [13, 15], [12, 14], [14, 16],
];

// Halpe26 adds neck (18), hip-center (19) and per-side toes/heels. Indices
// 0–16 still match COCO17, so the upper-body edges are shared.
export const SKELETON_HALPE26: Array<[number, number]> = [
  [17, 18], [0, 17], [0, 1], [0, 2], [1, 3], [2, 4],
  [18, 5], [18, 6], [18, 19], [11, 19], [12, 19], [5, 6], [11, 12],
  [5, 7], [7, 9], [6, 8], [8, 10],
  [11, 13], [13, 15], [12, 14], [14, 16],
  [15, 24], [16, 25],
];

const KP_COUNT_BY_FORMAT: Record<KpFormat, number> = { coco17: 17, halpe26: 26 };

export function skeletonFor(format: KpFormat): Array<[number, number]> {
  return format === "halpe26" ? SKELETON_HALPE26 : SKELETON_COCO17;
}

export function kpCountFor(format: KpFormat): number {
  return KP_COUNT_BY_FORMAT[format] ?? 17;
}

// 10 SlimeVR trackers in role order (sensor_id 0..9) — matches the wire
// ordering in the C++ TrackerRole enum.
export const TRACKER_ROLES = [
  "LeftUpperArm", "RightUpperArm",
  "Chest", "Waist",
  "LeftUpperLeg", "RightUpperLeg",
  "LeftLowerLeg", "RightLowerLeg",
  "LeftFoot", "RightFoot",
];
export const TRACKER_COUNT = TRACKER_ROLES.length;
