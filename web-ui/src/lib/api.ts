// Typed REST wrappers for the Crow endpoints. All routed through transport.ts
// (which honours the configured backend origin).

import { getJson, postJson } from "./transport";
import type {
  AutoAlignResponse,
  CalibActionResponse,
  CamerasResponse,
  CameraPreviewResponse,
  ConfigDraft,
  ConfigListResponse,
  ConfigLoadResponse,
  ConfigOkResponse,
  ConfigResponse,
  CorrectionsResponse,
  ExcalState,
  FlowMode,
  FlowState,
  FlowSwitchResponse,
  IncalState,
  SetupProceedResponse,
  VmtAlignment,
  VmtAlignmentResponse,
} from "../types/bundle";

// ---- Flow daemon -----------------------------------------------------------

export function fetchFlowState(): Promise<FlowState> {
  return getJson<FlowState>("/api/state");
}

export async function requestFlowSwitch(mode: FlowMode): Promise<FlowSwitchResponse> {
  const { res, data } = await postJson<FlowSwitchResponse>(
    "/api/flow/switch",
    { mode },
  );
  if (!res.ok || (data && data.ok === false)) {
    return { ok: false, err: data?.err || `HTTP ${res.status}` };
  }
  return data ?? { ok: true };
}

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

export async function postCalib<T = { ok?: boolean; err?: string; next_step?: string }>(
  action: "preflight" | "start" | "cancel" | "retake" | "approve",
  body?: unknown,
): Promise<T> {
  const { data } = await postJson<T>(`/api/calib/${action}`, body);
  return (data ?? ({} as T));
}

// ---- Intrinsic calibration (/api/incal/*) ----------------------------------

export function fetchIncalState(): Promise<IncalState> {
  return getJson<IncalState>("/api/incal/state");
}

/** POST an intrinsic-calib action. Tolerates a non-OK / module-swap response. */
export async function postIncal(
  action: "start" | "stop" | "solve",
): Promise<CalibActionResponse> {
  try {
    const { res, data } = await postJson<CalibActionResponse>(`/api/incal/${action}`);
    if (!res.ok || (data && data.ok === false)) {
      return { ok: false, err: data?.err || `HTTP ${res.status}`, ...(data ?? {}) };
    }
    return data ?? { ok: true };
  } catch (e) {
    return { ok: false, err: (e as Error).message || "request failed" };
  }
}

// ---- Extrinsic calibration (/api/excal/*) ----------------------------------

export function fetchExcalState(): Promise<ExcalState> {
  return getJson<ExcalState>("/api/excal/state");
}

/** POST an extrinsic-calib action. Tolerates a non-OK / module-swap response. */
export async function postExcal(
  action: "start" | "stop" | "solve",
): Promise<CalibActionResponse> {
  try {
    const { res, data } = await postJson<CalibActionResponse>(`/api/excal/${action}`);
    if (!res.ok || (data && data.ok === false)) {
      return { ok: false, err: data?.err || `HTTP ${res.status}`, ...(data ?? {}) };
    }
    return data ?? { ok: true };
  } catch (e) {
    return { ok: false, err: (e as Error).message || "request failed" };
  }
}

// ---- Setup (cameras / config / proceed) ------------------------------------

export function fetchCameras(): Promise<CamerasResponse> {
  return getJson<CamerasResponse>("/api/cameras");
}

export async function startCameraPreview(body: {
  device: string;
  width: number;
  height: number;
  fps: number;
  pixel_format: string;
}): Promise<CameraPreviewResponse> {
  const { res, data } = await postJson<CameraPreviewResponse>(
    "/api/cameras/preview/start",
    body,
  );
  if (!res.ok || (data && data.ok === false)) {
    return { ok: false, err: data?.err || `HTTP ${res.status}` };
  }
  return data ?? { ok: true };
}

export async function stopCameraPreview(device: string): Promise<CameraPreviewResponse> {
  const { res, data } = await postJson<CameraPreviewResponse>(
    "/api/cameras/preview/stop",
    { device },
  );
  if (!res.ok) return { ok: false, err: data?.err || `HTTP ${res.status}` };
  return data ?? { ok: true };
}

export function fetchConfig(): Promise<ConfigResponse> {
  return getJson<ConfigResponse>("/api/config");
}

/** Persist a (partial) config; the backend merges only the present keys. */
export async function postConfig(partial: Partial<ConfigDraft>): Promise<ConfigOkResponse> {
  const { res, data } = await postJson<ConfigOkResponse>("/api/config", {
    config: partial,
  });
  if (!res.ok || (data && data.ok === false)) {
    return { ok: false, err: data?.err || `HTTP ${res.status}` };
  }
  return data ?? { ok: true };
}

export async function validateConfig(): Promise<ConfigOkResponse> {
  const { res, data } = await postJson<ConfigOkResponse>("/api/config/validate");
  if (!res.ok) return { ok: false, err: data?.err || `HTTP ${res.status}` };
  return data ?? { ok: true };
}

export function listNamedConfigs(): Promise<ConfigListResponse> {
  return getJson<ConfigListResponse>("/api/config/list");
}

export async function saveNamedConfig(name: string): Promise<ConfigOkResponse> {
  const { res, data } = await postJson<ConfigOkResponse>("/api/config/save", { name });
  if (!res.ok || (data && data.ok === false)) {
    return { ok: false, err: data?.err || `HTTP ${res.status}` };
  }
  return data ?? { ok: true };
}

export async function loadNamedConfig(name: string): Promise<ConfigLoadResponse> {
  const { res, data } = await postJson<ConfigLoadResponse>("/api/config/load", { name });
  if (!res.ok || (data && data.ok === false)) {
    return { ok: false, err: data?.err || `HTTP ${res.status}` };
  }
  return data ?? { ok: true };
}

export async function proceedSetup(): Promise<SetupProceedResponse> {
  const { res, data } = await postJson<SetupProceedResponse>("/api/setup/proceed");
  if (!res.ok || (data && data.ok === false)) {
    return { ok: false, err: data?.err || `HTTP ${res.status}` };
  }
  return data ?? { ok: true };
}
