// Typed REST wrappers for the Crow endpoints. All routed through transport.ts
// (which honours the configured backend origin).

import { getJson, postJson } from "./transport";
import type {
  AutoAlignResponse,
  CorrectionsResponse,
  VmtAlignment,
  VmtAlignmentResponse,
} from "../types/bundle";

// ---- VMT alignment ---------------------------------------------------------

export function fetchVmtAlignment(): Promise<VmtAlignmentResponse> {
  return getJson<VmtAlignmentResponse>("/api/vmt/alignment");
}

/** POST a (possibly partial) alignment; throws on failure. Returns applied alignment. */
export async function postVmtAlignment(
  alignment: Partial<VmtAlignment>,
): Promise<VmtAlignment | undefined> {
  const { res, data } = await postJson<VmtAlignmentResponse>(
    "/api/vmt/alignment",
    alignment,
  );
  if (!res.ok || !data || data.ok === false) {
    throw new Error(data?.err || `HTTP ${res.status}`);
  }
  return data.alignment;
}

export async function postAutoTpose(): Promise<AutoAlignResponse> {
  const { res, data } = await postJson<AutoAlignResponse>(
    "/api/vmt/alignment/auto/tpose",
  );
  return data ?? { ok: false, err: `HTTP ${res.status}` };
}

export async function postMotionStart(
  durationS: number,
  sampleHz: number,
): Promise<AutoAlignResponse> {
  const { res, data } = await postJson<AutoAlignResponse>(
    "/api/vmt/alignment/auto/motion/start",
    { duration_s: durationS, sample_hz: sampleHz },
  );
  return data ?? { ok: false, err: `HTTP ${res.status}` };
}

export async function postMotionStop(): Promise<AutoAlignResponse> {
  const { res, data } = await postJson<AutoAlignResponse>(
    "/api/vmt/alignment/auto/motion/stop",
  );
  return data ?? { ok: false, err: `HTTP ${res.status}` };
}

/** Start/stop continuous HMD-driven auto-alignment. Returns an error string on failure, else null. */
export async function postContinuousAlign(enabled: boolean): Promise<string | null> {
  const path = enabled
    ? "/api/vmt/alignment/auto/continuous/start"
    : "/api/vmt/alignment/auto/continuous/stop";
  const { res, data } = await postJson<{ ok?: boolean; err?: string }>(path);
  if (!res.ok || (data && data.ok === false)) {
    return data?.err || `HTTP ${res.status}`;
  }
  return null;
}

// ---- SlimeVR corrections ---------------------------------------------------

export function fetchCorrections(): Promise<CorrectionsResponse> {
  return getJson<CorrectionsResponse>("/api/slimevr/corrections");
}

export async function postCorrection(
  body: Record<string, unknown>,
): Promise<CorrectionsResponse> {
  const { res, data } = await postJson<CorrectionsResponse>(
    "/api/slimevr/corrections",
    body,
  );
  if (!res.ok || !data || data.ok === false) {
    return { ok: false, err: data?.err || `HTTP ${res.status}` };
  }
  return data;
}

// ---- Subject calibration ---------------------------------------------------

export function fetchCalibState<T = Record<string, unknown>>(): Promise<T> {
  return getJson<T>("/api/calib/state");
}

export async function postCalib<T = { ok?: boolean; err?: string }>(
  action: "preflight" | "start" | "cancel" | "retake" | "approve",
  body?: unknown,
): Promise<T> {
  const { data } = await postJson<T>(`/api/calib/${action}`, body);
  return (data ?? ({} as T));
}
