// Build the monospaced stats blocks shown under each camera and the 3D view.
// Ported verbatim (text layout preserved) from updateStats/update3DStats in the
// legacy viewer.

import { formatInputNumber } from "./format";
import type { Bundle3D, CameraBundle } from "../types/bundle";

export function build2dStatsText(
  cam: CameraBundle | undefined,
  renderFps: number,
  serverLastMs: number,
  serverSeq: number,
): string {
  if (!cam) return "waiting…";
  const s = cam.stats || {};
  const latency =
    s.captured_at_ms && serverLastMs ? Math.max(0, serverLastMs - s.captured_at_ms) : 0;
  return (
    `recv_fps        ${(s.recv_fps ?? 0).toFixed(2)}\n` +
    `render_fps      ${renderFps.toFixed(1)}\n` +
    `recent_pose_fps ${(s.recent_pose_fps ?? 0).toFixed(2)}\n` +
    `avg_pose_fps    ${(s.avg_pose_fps ?? 0).toFixed(2)}\n` +
    `stage_ms        ${(s.stage_ms ?? 0).toFixed(1)}\n` +
    `pending         ${s.pending ?? 0}\n` +
    `processed       ${s.processed ?? 0}\n` +
    `latency_ms      ${latency}\n` +
    `bundle_seq      ${serverSeq}`
  );
}

export interface HmdStatus {
  text: string;
  cls: string;
}

export function build3dStatsText(
  bundle: Bundle3D | null,
  server3dSeq: number,
): { text: string; hmdStatus: HmdStatus } {
  if (!bundle) {
    return { text: "waiting…", hmdStatus: { text: "no hmd", cls: "" } };
  }
  if (bundle.enabled === false) {
    return { text: "enabled         false", hmdStatus: { text: "no hmd", cls: "" } };
  }
  const s = bundle.stats || {};
  const vmt = bundle.vmt || null;
  const a = vmt && vmt.alignment ? vmt.alignment : null;
  const vmtLine = vmt
    ? `\nvmt_bundles    ${vmt.sent_bundles ?? 0}` +
      `\nvmt_disabled   ${vmt.disabled_count ?? 0}` +
      `\nvmt_align      x=${formatInputNumber(a?.x ?? 0)} ` +
      `y=${formatInputNumber(a?.y ?? 0)} ` +
      `z=${formatInputNumber(a?.z ?? 0)} ` +
      `yaw=${formatInputNumber(a?.yaw_deg ?? 0)}`
    : "\nvmt            off";

  const hmd = bundle.hmd || null;
  let hmdLine = "";
  let hmdStatus: HmdStatus = { text: "no hmd", cls: "" };
  if (hmd && hmd.enabled) {
    if (!hmd.have_any) {
      hmdStatus = { text: "waiting for hmd", cls: "" };
      hmdLine = "\nhmd            waiting";
    } else if (hmd.stale) {
      hmdStatus = { text: `stale (${Math.round(hmd.age_ms ?? 0)}ms)`, cls: "dead" };
      hmdLine = `\nhmd            stale (${(hmd.age_ms ?? 0).toFixed(0)}ms)`;
    } else if (hmd.valid === false) {
      hmdStatus = { text: "lost", cls: "dead" };
      hmdLine = "\nhmd            lost";
    } else {
      hmdStatus = { text: `tracking (${Math.round(hmd.age_ms ?? 0)}ms)`, cls: "live" };
      const pos = hmd.pos || [0, 0, 0];
      hmdLine =
        `\nhmd_pos        [${pos[0]?.toFixed(3)}, ${pos[1]?.toFixed(3)}, ${pos[2]?.toFixed(3)}]` +
        `\nhmd_yaw_deg    ${(hmd.yaw_deg ?? 0).toFixed(2)}` +
        `\nhmd_age_ms     ${(hmd.age_ms ?? 0).toFixed(0)}`;
    }
  }

  const text =
    `tri_fps         ${(s.tri_fps ?? 0).toFixed(2)}\n` +
    `reproj_med_px  ${(s.reproj_err_med_px ?? 0).toFixed(2)}\n` +
    `bone_drift_pct ${(s.bone_len_drift_pct ?? 0).toFixed(2)}\n` +
    `valid_joints   ${s.valid_joints ?? 0}\n` +
    `sync_dt_ms     ${(s.sync_dt_ms ?? 0).toFixed(1)}\n` +
    `stage_ms       ${(s.stage_ms ?? 0).toFixed(2)}\n` +
    `height_m       ${(s.subject_height_m ?? 0).toFixed(2)}\n` +
    `profile_loaded ${s.profile_loaded ? "true" : "false"}\n` +
    `subject_id     ${s.subject_id || "-"}\n` +
    `quality        ${s.quality_status || "-"}\n` +
    `processed      ${s.processed ?? 0}\n` +
    `sync_miss      ${s.sync_miss ?? 0}\n` +
    `ik_locked      ${s.ik_locked ? "true" : "false"}\n` +
    `bundle_seq     ${server3dSeq}` +
    vmtLine +
    hmdLine;
  return { text, hmdStatus };
}
