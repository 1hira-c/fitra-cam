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

/** Static camera placement in the world frame (Z-up). */
export interface Camera3D {
  id: string;
  /** Camera center [x, y, z] in world coords. */
  pos: [number, number, number];
  /** Camera->world rotation as a unit quaternion (w, x, y, z). */
  quat_wxyz: [number, number, number, number];
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
  preset?: string;
  alignment?: VmtAlignment;
  [k: string]: unknown;
}

export interface VmtPresetResponse {
  enabled?: boolean;
  preset?: string;
  ok?: boolean;
  err?: string;
}

export interface HmdBlock {
  enabled?: boolean;
  have_any?: boolean;
  stale?: boolean;
  valid?: boolean;
  age_ms?: number;
  /** Raw HMD position in VMT Driver frame (Y-up). */
  pos?: [number, number, number];
  yaw_deg?: number;
  /** HMD position in fitra world frame (Z-up); present when VMT alignment known. */
  pos_world?: [number, number, number];
  /** HMD orientation in fitra world frame, quaternion (w, x, y, z). */
  quat_wxyz?: [number, number, number, number];
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

export interface DiscoveryPeer {
  name?: string;
  id?: string;
  ip?: string;
  port?: number;
  age_ms?: number;
}

/** Zeroconf discovery status. The block is present only when discovery is
 *  active (a pinned vmt.host runs no beacon — the viewer then shows "manual"). */
export interface DiscoveryBlock {
  mode?: "discovery" | "manual";
  socket_up?: boolean;
  self_id?: string;
  group?: string;
  discovery_port?: number;
  announces_sent?: number;
  announces_recv?: number;
  peer_count?: number;
  resolved?: DiscoveryPeer & { have?: boolean };
  peers?: DiscoveryPeer[];
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
  cameras?: Camera3D[];
  trackers?: Tracker[];
  stats?: Stats3D;
  vmt?: VmtBlock;
  hmd?: HmdBlock;
  continuous_align?: ContinuousAlignBlock;
  discovery?: DiscoveryBlock;
}

// ---- REST payloads ---------------------------------------------------------

export type FlowMode =
  | "run"
  | "setup"
  | "calib-subject"
  | "calib-extrinsic"
  | "calib-extrinsic-floor"
  | "calib-intrinsic";

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

// ---- Intrinsic calibration (/api/incal/*) ----------------------------------

export interface IncalCamera {
  cam: number;
  views: number;
  coverage: number; // 0..1
  rms_px: number;
  solved: boolean;
}

export interface IncalState {
  method?: "intrinsic";
  state: "idle" | "collecting" | "solving" | "solved" | "failed";
  model?: string;
  num_cams?: number;
  min_views?: number;
  min_corners?: number;
  cameras?: IncalCamera[];
  error?: string;
}

export interface CalibActionResponse {
  ok?: boolean;
  err?: string;
  state?: string;
  samples?: number;
  next_step?: string;
}

// ---- Extrinsic calibration (/api/excal/*) ----------------------------------

export type ExcalMethod = "controller" | "floor";

/** A detected face (controller method) or tag (floor method) within a frame. */
export interface ExcalFaceDetection {
  id: number;
  reproj: number;
  ok: boolean;
}

export interface ExcalDetection {
  cam: number;
  /** controller method only */
  ctrl_ok?: boolean;
  /** controller method */
  faces?: ExcalFaceDetection[];
  /** floor method */
  tags?: ExcalFaceDetection[];
  age_ms: number;
}

export interface ExcalCoverage {
  cam: number;
  /** controller method */
  face?: number;
  /** floor method */
  tag?: number;
  count: number;
}

export interface ExcalGate {
  lin_max?: number;
  ang_max?: number;
}

/** Per-camera solve result. Fields differ by method. */
export interface ExcalResultCamera {
  cam: number;
  // controller
  n_faces?: number;
  n_samples?: number;
  face_spread_trans_m?: number;
  face_spread_rot_deg?: number;
  // floor
  n_tags?: number;
  reproj_rms_px?: number;
  planar_degenerate?: boolean;
  plane_thickness_m?: number;
}

export interface ExcalState {
  method?: ExcalMethod;
  state: string;
  samples?: number;
  /** controller method */
  min_samples?: number;
  /** floor method */
  burst_min?: number;
  num_cams?: number;
  // controller-only motion gate
  lin_vel_mps?: number;
  ang_vel_dps?: number;
  gate?: ExcalGate;
  gate_reason?: string;
  /** controller method face ids */
  faces?: number[];
  /** floor method tag ids */
  tags?: number[];
  detections?: ExcalDetection[];
  coverage?: ExcalCoverage[];
  cameras?: ExcalResultCamera[];
  error?: string;
}

// ---- Setup (/api/cameras*, /api/config*, /api/setup/*) ---------------------

export interface CameraFormatSize {
  width: number;
  height: number;
  fps: number[];
}

export interface CameraFormat {
  fourcc: string;
  description: string;
  sizes: CameraFormatSize[];
}

export interface DetectedCamera {
  by_path: string;
  dev_node: string;
  card: string;
  driver: string;
  formats: CameraFormat[];
}

export interface CamerasResponse {
  cameras: DetectedCamera[];
}

// Per-camera override (slot index 0..2). Each field's "unset" sentinel means
// "use the global setting / leave the camera default", matching the YAML
// cam{N}_* keys and the MainOptions arrays.
export interface ConfigCameraOverride {
  capture_width: number;   // 0 = use global width (capture at full sensor then downscale)
  capture_height: number;  // 0 = use global height
  pixel_format: string;    // "" = use global pixel_format
  exposure_mode: string;   // "" / "auto" = untouched | "manual" | "assist"
  exposure: number;        // V4L2 exposure_absolute, 100us units; 0 = leave
  gain: number;            // V4L2 gain; -1 = leave at camera default
  ae_target: number;       // assist target mean luma (0..255)
}

export interface ConfigCameras {
  cam0: string;
  cam1: string;
  cam2: string;
  width: number;
  height: number;
  fps: number;
  pixel_format: string;
  n_buffers: number;
  /** Per-slot overrides; index 0=cam0, 1=cam1, 2=cam2. Always length 3. */
  overrides: ConfigCameraOverride[];
}

export interface ConfigInference {
  det_engine: string;
  pose_engine: string;
  keypoint_format: KpFormat;
  det_frequency: number;
  det_score: number;
  multi_person: boolean;
}

export interface ConfigWeb {
  host: string;
  port: number;
}

export interface ConfigThreeD {
  enable_3d: boolean;
  calib: string;
}

export interface ConfigVmt {
  vmt_out: boolean;
  // Runtime auto-discovery (zeroconf): true + empty host = resolve the VMT
  // target on the LAN at runtime; false (or a non-empty host) = manual fixed IP.
  discovery: boolean;
  host: string;
  port: number;
  hmd_listen_enabled: boolean;
}

export interface ConfigIntrinsicCalib {
  enabled: boolean;
  out: string;   // where the intrinsic step writes (= the extrinsic step's intrinsics input)
  model: string; // "pinhole" | "fisheye" — fisheye lenses (ELP AR0234) need fisheye
}

export interface ConfigExtrinsicCalib {
  method: string;          // "controller" | "floor"
  out: string;             // where extrinsics are written
  intrinsics: string;      // controller PnP intrinsics ("" → reuse three_d.calib)
  floor_map: string;       // floor: known AprilTag layout YAML (required for floor)
  floor_intrinsics: string; // floor PnP intrinsics ("" → reuse three_d.calib)
  floor_fisheye: boolean;  // floor: intrinsics use the fisheye model
}

export interface ConfigDraft {
  cameras: ConfigCameras;
  inference: ConfigInference;
  web: ConfigWeb;
  three_d: ConfigThreeD;
  vmt: ConfigVmt;
  intrinsic_calib: ConfigIntrinsicCalib;
  extrinsic_calib: ConfigExtrinsicCalib;
}

export interface ConfigResponse {
  config: ConfigDraft;
  named: string[];
}

export interface ConfigListResponse {
  named: string[];
}

export interface ConfigOkResponse {
  ok?: boolean;
  err?: string;
}

export interface ConfigLoadResponse {
  ok?: boolean;
  err?: string;
  config?: ConfigDraft;
}

export interface SetupProceedResponse {
  ok?: boolean;
  next?: string;
  err?: string;
}

export interface CameraPreviewResponse {
  ok?: boolean;
  err?: string;
}

export interface PathCheckResponse {
  path: string;
  abs: string;      // resolved against the backend CWD (where engines are opened)
  exists: boolean;
  is_file: boolean;
}
