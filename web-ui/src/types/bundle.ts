// Wire schemas emitted by the C++ Crow server (snapshot.cpp). The frontend
// consumes these unchanged — do not rename fields. Numeric arrays mirror the
// compact JSON the backend produces.

export type KpFormat = "coco17" | "halpe26";

// ---- 2D bundle (/ws) -------------------------------------------------------

/** [x, y, confidence] */
export type Keypoint2D = [number, number, number];

export interface Person2D {
  kpts: Keypoint2D[];
  /** [x1, y1, x2, y2, score] (optional). */
  bbox?: number[];
}

export interface CameraStats {
  recv_fps?: number;
  recent_pose_fps?: number;
  avg_pose_fps?: number;
  pending?: number;
  stage_ms?: number;
  processed?: number;
  captured_at_ms?: number;
}

export interface CameraBundle {
  id: number;
  w: number;
  h: number;
  persons: Person2D[];
  stats?: CameraStats;
}

export interface Bundle2D {
  seq: number;
  ts_ms: number;
  kp_format?: KpFormat;
  cameras: CameraBundle[];
}

// ---- 3D bundle (/ws3d) -----------------------------------------------------

/** [x, y, z, confidence, valid] */
export type Joint3D = [number, number, number, number, boolean];

export interface Person3D {
  id: number;
  joints: Joint3D[];
}

export interface TrackerStats {
  freeze_current_ms?: number;
  leakage_pct?: number;
  freeze_pct?: number;
  ang_vel_p50?: number;
  ang_vel_p95?: number;
  conf_avg?: number;
  freeze_max_ms?: number;
  dropouts?: number;
}

export interface Tracker {
  role: string;
  pos: [number, number, number];
  quat_wxyz: [number, number, number, number];
  valid: boolean;
  roll_confidence?: number;
  stats?: TrackerStats;
}

export interface VmtAlignment {
  x: number;
  y: number;
  z: number;
  yaw_deg: number;
}

export interface VmtBlock {
  sent_bundles?: number;
  disabled_count?: number;
  alignment?: VmtAlignment;
  [k: string]: unknown;
}

export interface HmdBlock {
  enabled?: boolean;
  have_any?: boolean;
  stale?: boolean;
  valid?: boolean;
  age_ms?: number;
  pos?: [number, number, number];
  yaw_deg?: number;
  [k: string]: unknown;
}

export interface ContinuousAlignBlock {
  running?: boolean;
  enabled?: boolean;
  locked?: boolean;
  occupied_cells?: number;
  min_cells?: number;
  head_samples?: number;
  chest_samples?: number;
  last_status?: string;
  last_residual_m?: number;
  resolves?: number;
  updates?: number;
  [k: string]: unknown;
}

export interface Stats3D {
  enabled?: boolean;
  tri_fps?: number;
  reproj_err_med_px?: number;
  bone_len_drift_pct?: number;
  valid_joints?: number;
  sync_dt_ms?: number;
  stage_ms?: number;
  subject_height_m?: number;
  profile_loaded?: boolean;
  subject_id?: string;
  quality_status?: string;
  processed?: number;
  sync_miss?: number;
  ik_locked?: boolean;
}

export interface Bundle3D {
  seq: number;
  ts_ms: number;
  enabled?: boolean;
  kp_format?: KpFormat;
  persons_3d?: Person3D[];
  trackers?: Tracker[];
  stats?: Stats3D;
  vmt?: VmtBlock;
  hmd?: HmdBlock;
  continuous_align?: ContinuousAlignBlock;
}

// ---- REST payloads ---------------------------------------------------------

export type FlowMode =
  | "run"
  | "calib-subject"
  | "calib-extrinsic"
  | "calib-extrinsic-floor";

export interface FlowState {
  mode: FlowMode;
  managed?: boolean;
}

export interface FlowSwitchResponse {
  ok?: boolean;
  err?: string;
}

export interface VmtAlignmentResponse {
  alignment?: VmtAlignment;
  enabled?: boolean;
  ok?: boolean;
  err?: string;
}

export interface AutoAlignResult {
  status: string;
  err?: string;
  alignment?: VmtAlignment;
  residual_m?: number;
  n_samples?: number;
}

export interface AutoAlignResponse {
  ok?: boolean;
  err?: string;
  result?: AutoAlignResult;
}

export interface CorrectionRole {
  role: string;
  yaw_quarters?: number;
  pitch_quarters?: number;
  roll_quarters?: number;
}

export interface CorrectionsResponse {
  ok?: boolean;
  err?: string;
  roles?: CorrectionRole[];
  preview_no_reset?: boolean;
}
