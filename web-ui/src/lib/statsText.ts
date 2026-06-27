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

// At-a-glance VMT output-target status for the viewer header, derived from the
// same /ws3d discovery fragment that build3dStatsText spells out in the stats
// block. Returns null when there is no VMT output to report (so no chip shows).
//   - discovery resolved → "出力先 <name> <ip>:<port>" (live)
//   - discovery searching → "出力先 検索中… (N)"
//   - no discovery block but vmt active → "出力先 手動" (a pinned host runs no beacon)
// Gated on `bundle.vmt`: the backend emits that block only when VMT output is
// enabled, whereas the discovery beacon also runs for the HMD-listen punch path
// (vmt_out=false, see pose_relay_builder.cpp). Without the gate a punch-only rig
// would falsely advertise an 出力先 (output target) when nothing is sent there.
export function discoveryStatus(bundle: Bundle3D | null): HmdStatus | null {
  if (!bundle || bundle.enabled === false || !bundle.vmt) return null;
  const disc = bundle.discovery || null;
  if (disc) {
    const r = disc.resolved;
    if (r && r.have) {
      return {
        text: `出力先 ${r.name || r.id || "peer"} ${r.ip ?? "?"}:${r.port ?? 0}`,
        cls: "live",
      };
    }
    return { text: `出力先 検索中… (${disc.peer_count ?? 0})`, cls: "" };
  }
  return { text: "出力先 手動", cls: "" };
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

  // Zeroconf discovery status. The block is present only in discovery mode; a
  // pinned vmt.host runs no beacon, so an absent block (with VMT active) means
  // the destination was configured manually.
  const disc = bundle.discovery || null;
  let discLine = "";
  if (disc) {
    const r = disc.resolved;
    if (r && r.have) {
      discLine =
        `\ndiscovery      ${r.name || r.id || "peer"} ` +
        `${r.ip ?? "?"}:${r.port ?? 0} (${Math.round(r.age_ms ?? 0)}ms)`;
    } else {
      discLine = `\ndiscovery      searching… (${disc.peer_count ?? 0} peers)`;
    }
  } else if (vmt) {
    discLine = "\ndiscovery      manual";
  }

  // Continuous HMD-driven alignment status (block exists iff aligner attached).
  const cont = bundle.continuous_align || null;
  let contLine = "";
  if (cont) {
    const srcMix = `head=${cont.head_samples ?? 0} chest=${cont.chest_samples ?? 0}`;
    const phase = cont.locked ? "fine" : "coarse";
    contLine =
      `\ncont_align     ${cont.enabled ? "on" : "off"} [${phase}] (${cont.last_status || "-"})` +
      `\ncont_cells     ${cont.occupied_cells ?? 0}/${cont.min_cells ?? 0} ${srcMix}` +
      `\ncont_resid_m   ${(cont.last_residual_m ?? 0).toFixed(3)}` +
      `\ncont_updates   ${cont.updates ?? 0}/${cont.resolves ?? 0}`;
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
    discLine +
    hmdLine +
    contLine;
  return { text, hmdStatus };
}
