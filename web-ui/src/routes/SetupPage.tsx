// First-run setup page (FlowMode "setup"). Camera enumeration + assignment,
// one-at-a-time MJPEG preview, config editor (engines / output targets /
// keypoint format / 3D / calib), named-config load/save, validate, and proceed.
// All editable config lives in one ConfigDraft state with controlled inputs.

import { useCallback, useEffect, useRef, useState, type ReactNode } from "react";
import { Link } from "react-router-dom";
import { WizardLayout } from "../components/WizardLayout";
import { useFlowWatch } from "../hooks/useFlowWatch";
import {
  checkPath,
  fetchCameras,
  fetchConfig,
  loadNamedConfig,
  postConfig,
  proceedSetup,
  saveNamedConfig,
  startCameraPreview,
  stopCameraPreview,
  validateConfig,
} from "../lib/api";
import { httpUrl } from "../lib/config";
import type {
  CameraFormat,
  ConfigCameraOverride,
  ConfigCameras,
  ConfigDraft,
  ConfigExtrinsicCalib,
  ConfigInference,
  ConfigSlimevr,
  ConfigThreeD,
  ConfigVmt,
  ConfigWeb,
  DetectedCamera,
  KpFormat,
  PathCheckResponse,
} from "../types/bundle";
import "../styles/setup.css";

const PREVIEW_REFRESH_MS = 150;
const SLOTS: Array<keyof Pick<ConfigCameras, "cam0" | "cam1" | "cam2">> = ["cam0", "cam1", "cam2"];
const PIXEL_FORMATS = ["mjpeg", "yuyv"];
// Per-camera pixel_format override: "" = use the global format; nvjpeg (HW
// decode) is valid per-camera even though the global selector omits it.
const OVERRIDE_PIXEL_FORMATS = ["", "mjpeg", "yuyv", "nvjpeg"];

type Msg = { text: string; err: boolean };

// Map a v4l2 pixel_format string to the fourcc used by the enumerated formats.
function fourccForPixelFormat(pf: string): string {
  const p = pf.toLowerCase();
  if (p === "mjpeg" || p === "mjpg") return "MJPG";
  if (p === "yuyv") return "YUYV";
  return pf.toUpperCase();
}

// Pick the enumerated format matching the draft pixel_format (fall back to first).
function formatFor(cam: DetectedCamera | undefined, pixelFormat: string): CameraFormat | undefined {
  if (!cam) return undefined;
  const want = fourccForPixelFormat(pixelFormat);
  return cam.formats.find((f) => f.fourcc.toUpperCase() === want) ?? cam.formats[0];
}

interface PreviewSettings {
  width: number;
  height: number;
  fps: number;
  pixel_format: string;
  exposure_mode: string;
  exposure: number;
  gain: number;
  ae_target: number;
}

// Effective capture settings for previewing `cam`: the per-slot override (if the
// camera is assigned to a slot) merged over the global capture format, matching
// how the runtime builds each camera. The preview captures with exactly these so
// an override can be reflected and confirmed before committing.
function effectiveSettings(cam: DetectedCamera, draft: ConfigDraft): PreviewSettings {
  const c = draft.cameras;
  const i = SLOTS.findIndex((s) => c[s] === cam.by_path);
  const ov = i >= 0 ? c.overrides[i] : undefined;
  return {
    width: ov && ov.capture_width > 0 ? ov.capture_width : c.width,
    height: ov && ov.capture_height > 0 ? ov.capture_height : c.height,
    fps: c.fps,
    pixel_format: ov && ov.pixel_format ? ov.pixel_format : c.pixel_format,
    exposure_mode: ov ? ov.exposure_mode : "",
    exposure: ov ? ov.exposure : 0,
    gain: ov ? ov.gain : -1,
    ae_target: ov ? ov.ae_target : 110,
  };
}

// A text input for a filesystem path with an on-the-spot existence check. The
// path is resolved by the backend against the daemon CWD (where engines/calib
// are actually opened), so relative and absolute inputs both report correctly.
function PathField({
  label,
  value,
  onChange,
  placeholder,
}: {
  label: string;
  value: string;
  onChange: (v: string) => void;
  placeholder?: string;
}) {
  const [check, setCheck] = useState<PathCheckResponse | null>(null);
  const [checking, setChecking] = useState(false);

  useEffect(() => {
    if (!value.trim()) {
      setCheck(null);
      return;
    }
    let cancelled = false;
    setChecking(true);
    const t = setTimeout(async () => {
      try {
        const r = await checkPath(value);
        if (!cancelled) setCheck(r);
      } catch {
        if (!cancelled) setCheck(null);
      } finally {
        if (!cancelled) setChecking(false);
      }
    }, 500);
    return () => {
      cancelled = true;
      clearTimeout(t);
    };
  }, [value]);

  let status: ReactNode = null;
  if (!value.trim()) status = null;
  else if (checking) status = <span className="path-status">確認中…</span>;
  else if (check?.is_file) status = <span className="path-status ok">✓ {check.abs}</span>;
  else if (check?.exists)
    status = <span className="path-status warn">△ 存在（ファイルではない）: {check.abs}</span>;
  else if (check)
    status = <span className="path-status err">✗ 見つかりません: {check.abs}</span>;

  return (
    <label className="path-field">
      {label}
      <input
        type="text"
        value={value}
        placeholder={placeholder}
        onChange={(e) => onChange(e.target.value)}
      />
      {status}
    </label>
  );
}

export function SetupPage() {
  const flow = useFlowWatch({ page: "setup" });

  const [cameras, setCameras] = useState<DetectedCamera[]>([]);
  const [named, setNamed] = useState<string[]>([]);
  const [draft, setDraft] = useState<ConfigDraft | null>(null);

  const [scanMsg, setScanMsg] = useState<Msg>({ text: "", err: false });
  const [cfgMsg, setCfgMsg] = useState<Msg>({ text: "", err: false });
  const [validateMsg, setValidateMsg] = useState<Msg>({ text: "", err: false });
  const [proceedMsg, setProceedMsg] = useState<Msg>({ text: "", err: false });

  const [saveName, setSaveName] = useState("");
  const [selectedNamed, setSelectedNamed] = useState("");

  // Active preview: the by_path of the previewing camera, or null.
  const [previewing, setPreviewing] = useState<string | null>(null);
  const previewImg = useRef<HTMLImageElement | null>(null);
  const previewTimer = useRef<ReturnType<typeof setInterval> | null>(null);
  // The device the refresh tick currently points at (updated without a flash on
  // restart) and the effective-settings JSON last sent to the backend (so the
  // live-reflect effect only restarts the stream when something actually changed).
  const previewDevice = useRef<string>("");
  const appliedPreviewKey = useRef<string>("");

  // --- initial load ---------------------------------------------------------
  const loadCameras = useCallback(async () => {
    setScanMsg({ text: "scanning…", err: false });
    try {
      const r = await fetchCameras();
      setCameras(r.cameras ?? []);
      setScanMsg({ text: `${(r.cameras ?? []).length} cameras found`, err: false });
    } catch (e) {
      setScanMsg({ text: (e as Error).message || "scan failed", err: true });
    }
  }, []);

  const loadConfig = useCallback(async () => {
    try {
      const r = await fetchConfig();
      setDraft(r.config);
      setNamed(r.named ?? []);
    } catch (e) {
      setCfgMsg({ text: (e as Error).message || "config load failed", err: true });
    }
  }, []);

  useEffect(() => {
    void loadCameras();
    void loadConfig();
  }, [loadCameras, loadConfig]);

  // --- preview lifecycle ----------------------------------------------------
  const stopPreviewTimer = useCallback(() => {
    if (previewTimer.current !== null) {
      clearInterval(previewTimer.current);
      previewTimer.current = null;
    }
    if (previewImg.current) previewImg.current.src = "";
  }, []);

  // Tear down preview on unmount (and stop the backend stream).
  useEffect(() => {
    return () => {
      stopPreviewTimer();
      if (previewing) void stopCameraPreview(previewing);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // Start (or restart) the preview for `cam` using its effective settings. One
  // camera at a time: switching devices stops the previous backend stream. The
  // refresh tick reads previewDevice (a ref) so a same-camera restart (override
  // edit) does not blank/recreate the <img>.
  const applyPreview = useCallback(
    async (cam: DetectedCamera) => {
      if (!draft) return;
      const eff = effectiveSettings(cam, draft);
      if (previewing && previewing !== cam.by_path) {
        await stopCameraPreview(previewing);
      }
      const r = await startCameraPreview({ device: cam.by_path, ...eff });
      if (!r.ok) {
        setScanMsg({ text: r.err || "preview failed", err: true });
        return;
      }
      appliedPreviewKey.current = JSON.stringify(eff);
      previewDevice.current = cam.by_path;
      setPreviewing(cam.by_path);
      if (previewTimer.current === null) {
        const tick = () => {
          if (previewImg.current && previewDevice.current) {
            previewImg.current.src = httpUrl(
              "/api/cameras/preview.jpg?cam=" +
                encodeURIComponent(previewDevice.current) +
                "&t=" + Date.now(),
            );
          }
        };
        tick();
        previewTimer.current = setInterval(tick, PREVIEW_REFRESH_MS);
      }
    },
    [draft, previewing],
  );

  const stopPreview = useCallback(async () => {
    stopPreviewTimer();
    const dev = previewing;
    previewDevice.current = "";
    appliedPreviewKey.current = "";
    setPreviewing(null);
    if (dev) await stopCameraPreview(dev);
  }, [previewing, stopPreviewTimer]);

  // Live reflect: when the previewing camera's effective settings change (an
  // override or global edit), restart its stream after a short debounce so the
  // preview shows the new settings. The key-compare keeps it from restarting
  // when nothing relevant changed.
  useEffect(() => {
    if (!previewing || !draft) return;
    const cam = cameras.find((c) => c.by_path === previewing);
    if (!cam) return;
    const key = JSON.stringify(effectiveSettings(cam, draft));
    if (key === appliedPreviewKey.current) return;
    const t = setTimeout(() => void applyPreview(cam), 400);
    return () => clearTimeout(t);
  }, [previewing, draft, cameras, applyPreview]);

  // --- draft mutation helpers ----------------------------------------------
  const setCameras_ = (patch: Partial<ConfigCameras>) =>
    setDraft((d) => (d ? { ...d, cameras: { ...d.cameras, ...patch } } : d));
  const setInference = (patch: Partial<ConfigInference>) =>
    setDraft((d) => (d ? { ...d, inference: { ...d.inference, ...patch } } : d));
  const setWeb = (patch: Partial<ConfigWeb>) =>
    setDraft((d) => (d ? { ...d, web: { ...d.web, ...patch } } : d));
  const setThreeD = (patch: Partial<ConfigThreeD>) =>
    setDraft((d) => (d ? { ...d, three_d: { ...d.three_d, ...patch } } : d));
  const setVmt = (patch: Partial<ConfigVmt>) =>
    setDraft((d) => (d ? { ...d, vmt: { ...d.vmt, ...patch } } : d));
  const setSlimevr = (patch: Partial<ConfigSlimevr>) =>
    setDraft((d) => (d ? { ...d, slimevr: { ...d.slimevr, ...patch } } : d));
  const setExtrinsic = (patch: Partial<ConfigExtrinsicCalib>) =>
    setDraft((d) => (d ? { ...d, extrinsic_calib: { ...d.extrinsic_calib, ...patch } } : d));
  // Patch one slot's per-camera override (index 0=cam0, 1=cam1, 2=cam2).
  const setOverride = (i: number, patch: Partial<ConfigCameraOverride>) =>
    setDraft((d) => {
      if (!d) return d;
      const overrides = d.cameras.overrides.map((o, idx) => (idx === i ? { ...o, ...patch } : o));
      return { ...d, cameras: { ...d.cameras, overrides } };
    });

  // Assign a detected camera to a slot (toggle off if already assigned there).
  const assignSlot = (slot: keyof ConfigCameras, byPath: string) => {
    setDraft((d) => {
      if (!d) return d;
      const cur = (d.cameras[slot] as string) || "";
      return { ...d, cameras: { ...d.cameras, [slot]: cur === byPath ? "" : byPath } };
    });
  };

  // --- named configs --------------------------------------------------------
  const doLoadNamed = async () => {
    if (!selectedNamed) return;
    const r = await loadNamedConfig(selectedNamed);
    if (r.ok && r.config) {
      setDraft(r.config);
      setCfgMsg({ text: `loaded "${selectedNamed}"`, err: false });
    } else {
      setCfgMsg({ text: r.err || "load failed", err: true });
    }
  };
  const doSaveNamed = async () => {
    const name = saveName.trim();
    if (!name) {
      setCfgMsg({ text: "name is required", err: true });
      return;
    }
    if (draft) await postConfig(draft);
    const r = await saveNamedConfig(name);
    if (r.ok) {
      setCfgMsg({ text: `saved "${name}"`, err: false });
      if (!named.includes(name)) setNamed((n) => [...n, name]);
      setSaveName("");
    } else {
      setCfgMsg({ text: r.err || "save failed", err: true });
    }
  };

  // --- validate / proceed ---------------------------------------------------
  const doValidate = async () => {
    setValidateMsg({ text: "validating…", err: false });
    if (draft) await postConfig(draft);
    const r = await validateConfig();
    setValidateMsg({ text: r.ok ? "OK" : r.err || "invalid", err: !r.ok });
  };
  const doProceed = async () => {
    setProceedMsg({ text: "proceeding…", err: false });
    if (draft) {
      const saved = await postConfig(draft);
      if (!saved.ok) {
        setProceedMsg({ text: saved.err || "config save failed", err: true });
        return;
      }
    }
    const r = await proceedSetup();
    setProceedMsg({
      text: r.ok ? `next: ${r.next || "starting…"}` : r.err || "proceed failed",
      err: !r.ok,
    });
  };

  const conn = flow.status === "down" ? "disconnected — waiting for restart…" : draft ? "ok" : "connecting…";

  return (
    <div className="setup-page">
      <header>
        <h1>fitra-cam Setup</h1>
        <div className="conn-group">
          <div className="conn">{conn}</div>
          <Link className="conn link" to="/">live</Link>
        </div>
      </header>

      <WizardLayout current="setup" flow={flow.state}>
        <main>
          {/* 1. Camera enumeration + assignment + preview */}
          <section className="card">
            <h2>1. カメラ</h2>
            <p>検出されたカメラを cam0/cam1/cam2 スロットに割り当て、解像度・fps・ピクセル形式を選びます。</p>
            <div className="actions">
              <button type="button" onClick={() => void loadCameras()}>再スキャン</button>
              {previewing && <button type="button" onClick={() => void stopPreview()}>プレビュー停止</button>}
            </div>
            <div className={`msg ${scanMsg.err ? "err" : ""}`.trim()}>{scanMsg.text}</div>

            {draft && (
              <table className="cam-table">
                <thead>
                  <tr>
                    <th>device</th>
                    <th>card / driver</th>
                    <th>slot</th>
                    <th>preview</th>
                  </tr>
                </thead>
                <tbody>
                  {cameras.length === 0 ? (
                    <tr><td className="muted" colSpan={4}>no cameras detected</td></tr>
                  ) : (
                    cameras.map((cam) => {
                      const isPreview = previewing === cam.by_path;
                      return (
                        <tr key={cam.by_path}>
                          <td>
                            <code>{cam.by_path}</code>
                            <div className="muted">{cam.dev_node}</div>
                          </td>
                          <td>
                            {cam.card}
                            <div className="muted">{cam.driver}</div>
                          </td>
                          <td>
                            <div className="slot-buttons">
                              {SLOTS.map((slot) => {
                                const assigned = draft.cameras[slot] === cam.by_path;
                                return (
                                  <button
                                    key={slot}
                                    type="button"
                                    className={assigned ? "assigned" : ""}
                                    onClick={() => assignSlot(slot, cam.by_path)}
                                  >
                                    {slot}
                                  </button>
                                );
                              })}
                            </div>
                          </td>
                          <td>
                            <button
                              type="button"
                              onClick={() => (isPreview ? void stopPreview() : void applyPreview(cam))}
                            >
                              {isPreview ? "停止" : "プレビュー"}
                            </button>
                          </td>
                        </tr>
                      );
                    })
                  )}
                </tbody>
              </table>
            )}

            {previewing && (
              <div className="preview-wrap">
                <img ref={previewImg} alt="camera preview" />
                {draft && (() => {
                  const cam = cameras.find((c) => c.by_path === previewing);
                  if (!cam) return null;
                  const e = effectiveSettings(cam, draft);
                  const exp =
                    e.exposure_mode === "manual" || e.exposure_mode === "assist"
                      ? ` / 露出 ${e.exposure_mode} exp=${e.exposure} gain=${e.gain}` +
                        (e.exposure_mode === "assist" ? ` ae=${e.ae_target}` : "")
                      : "";
                  return (
                    <div className="preview-caption muted">
                      プレビュー: {e.width}×{e.height} {e.pixel_format} @{e.fps}fps{exp}
                    </div>
                  );
                })()}
              </div>
            )}

            {/* Shared capture format applied to all slots. */}
            {draft && (
              <>
                <h3>キャプチャ形式</h3>
                <div className="form-grid">
                  <label>
                    pixel_format
                    <select
                      value={draft.cameras.pixel_format}
                      onChange={(e) => setCameras_({ pixel_format: e.target.value })}
                    >
                      {PIXEL_FORMATS.map((pf) => <option key={pf} value={pf}>{pf}</option>)}
                    </select>
                  </label>
                  <label>
                    resolution
                    <select
                      value={`${draft.cameras.width}x${draft.cameras.height}`}
                      onChange={(e) => {
                        const [w, h] = e.target.value.split("x").map(Number);
                        setCameras_({ width: w, height: h });
                      }}
                    >
                      {(() => {
                        // Offer sizes from the first assigned camera's chosen format.
                        const firstAssigned = SLOTS.map((s) => draft.cameras[s])
                          .filter(Boolean)
                          .map((bp) => cameras.find((c) => c.by_path === bp))
                          .find(Boolean);
                        const fmt = formatFor(firstAssigned, draft.cameras.pixel_format);
                        const sizes = fmt?.sizes ?? [];
                        const cur = `${draft.cameras.width}x${draft.cameras.height}`;
                        const opts = sizes.map((s) => `${s.width}x${s.height}`);
                        if (!opts.includes(cur)) opts.unshift(cur);
                        return opts.map((o) => <option key={o} value={o}>{o}</option>);
                      })()}
                    </select>
                  </label>
                  <label>
                    fps
                    <select
                      value={String(draft.cameras.fps)}
                      onChange={(e) => setCameras_({ fps: Number(e.target.value) })}
                    >
                      {(() => {
                        const firstAssigned = SLOTS.map((s) => draft.cameras[s])
                          .filter(Boolean)
                          .map((bp) => cameras.find((c) => c.by_path === bp))
                          .find(Boolean);
                        const fmt = formatFor(firstAssigned, draft.cameras.pixel_format);
                        const size = fmt?.sizes.find(
                          (s) => s.width === draft.cameras.width && s.height === draft.cameras.height,
                        );
                        const fpsList = size?.fps ?? [];
                        const opts = fpsList.map((f) => Math.round(f));
                        if (!opts.includes(draft.cameras.fps)) opts.unshift(draft.cameras.fps);
                        return opts.map((f) => <option key={f} value={String(f)}>{f}</option>);
                      })()}
                    </select>
                  </label>
                  <label>
                    n_buffers
                    <input
                      type="number"
                      min={1}
                      max={16}
                      value={draft.cameras.n_buffers}
                      onChange={(e) => setCameras_({ n_buffers: Number(e.target.value) })}
                    />
                  </label>
                </div>

                <h3>カメラ別オーバーライド（任意）</h3>
                <p className="muted">
                  スロットごとにキャプチャ解像度・ピクセル形式・露出をグローバル設定から上書きします。
                  0 / 空欄 / gain -1 はグローバルまたはカメラ既定を使用。
                </p>
                {SLOTS.every((s) => !draft.cameras[s]) ? (
                  <p className="muted">スロットにカメラを割り当てると表示されます。</p>
                ) : (
                  SLOTS.map((slot, i) => {
                    const dev = draft.cameras[slot] as string;
                    const ov = draft.cameras.overrides[i];
                    if (!dev || !ov) return null;
                    const manualOrAssist = ov.exposure_mode === "manual" || ov.exposure_mode === "assist";
                    return (
                      <div key={slot} className="override-block">
                        <h4>{slot} <code className="muted">{dev}</code></h4>
                        <div className="form-grid">
                          <label>
                            capture override (w×h, 0=グローバル)
                            <span className="wh">
                              <input
                                type="number"
                                min={0}
                                value={ov.capture_width}
                                onChange={(e) => setOverride(i, { capture_width: Number(e.target.value) })}
                              />
                              <span>×</span>
                              <input
                                type="number"
                                min={0}
                                value={ov.capture_height}
                                onChange={(e) => setOverride(i, { capture_height: Number(e.target.value) })}
                              />
                            </span>
                          </label>
                          <label>
                            pixel_format
                            <select
                              value={ov.pixel_format}
                              onChange={(e) => setOverride(i, { pixel_format: e.target.value })}
                            >
                              {OVERRIDE_PIXEL_FORMATS.map((pf) => (
                                <option key={pf || "global"} value={pf}>{pf || "(グローバル)"}</option>
                              ))}
                            </select>
                          </label>
                          <label>
                            exposure_mode
                            <select
                              value={ov.exposure_mode}
                              onChange={(e) => setOverride(i, { exposure_mode: e.target.value })}
                            >
                              <option value="">auto（既定）</option>
                              <option value="manual">manual</option>
                              <option value="assist">assist</option>
                            </select>
                          </label>
                          {manualOrAssist && (
                            <>
                              <label>
                                exposure (×100µs)
                                <input
                                  type="number"
                                  min={0}
                                  value={ov.exposure}
                                  onChange={(e) => setOverride(i, { exposure: Number(e.target.value) })}
                                />
                              </label>
                              <label>
                                gain (-1=既定)
                                <input
                                  type="number"
                                  min={-1}
                                  value={ov.gain}
                                  onChange={(e) => setOverride(i, { gain: Number(e.target.value) })}
                                />
                              </label>
                            </>
                          )}
                          {ov.exposure_mode === "assist" && (
                            <label>
                              ae_target (0-255)
                              <input
                                type="number"
                                min={0}
                                max={255}
                                value={ov.ae_target}
                                onChange={(e) => setOverride(i, { ae_target: Number(e.target.value) })}
                              />
                            </label>
                          )}
                        </div>
                      </div>
                    );
                  })
                )}
              </>
            )}
          </section>

          {/* 2. Engines */}
          {draft && (
            <section className="card">
              <h2>2. 推論エンジン</h2>
              <div className="form-grid">
                <PathField
                  label="det_engine"
                  value={draft.inference.det_engine}
                  onChange={(v) => setInference({ det_engine: v })}
                  placeholder="outputs/tensorrt_engines/yolox.engine"
                />
                <PathField
                  label="pose_engine"
                  value={draft.inference.pose_engine}
                  onChange={(v) => setInference({ pose_engine: v })}
                  placeholder="outputs/tensorrt_engines/rtmpose.engine"
                />
                <label>
                  keypoint_format
                  <select
                    value={draft.inference.keypoint_format}
                    onChange={(e) => setInference({ keypoint_format: e.target.value as KpFormat })}
                  >
                    <option value="coco17">coco17</option>
                    <option value="halpe26">halpe26</option>
                  </select>
                </label>
                <label>
                  det_frequency
                  <input
                    type="number"
                    min={1}
                    value={draft.inference.det_frequency}
                    onChange={(e) => setInference({ det_frequency: Number(e.target.value) })}
                  />
                </label>
                <label>
                  det_score
                  <input
                    type="number"
                    min={0}
                    max={1}
                    step={0.01}
                    value={draft.inference.det_score}
                    onChange={(e) => setInference({ det_score: Number(e.target.value) })}
                  />
                </label>
                <label className="inline">
                  <input
                    type="checkbox"
                    checked={draft.inference.multi_person}
                    onChange={(e) => setInference({ multi_person: e.target.checked })}
                  />
                  multi_person
                </label>
              </div>
            </section>
          )}

          {/* 3. Output targets */}
          {draft && (
            <section className="card">
              <h2>3. 出力ターゲット</h2>
              <h3>VMT</h3>
              <div className="row">
                <label className="inline">
                  <input
                    type="checkbox"
                    checked={draft.vmt.vmt_out}
                    onChange={(e) => setVmt({ vmt_out: e.target.checked })}
                  />
                  enable
                </label>
                <label>
                  host
                  <input
                    type="text"
                    value={draft.vmt.host}
                    onChange={(e) => setVmt({ host: e.target.value })}
                  />
                </label>
                <label>
                  port
                  <input
                    type="number"
                    value={draft.vmt.port}
                    onChange={(e) => setVmt({ port: Number(e.target.value) })}
                  />
                </label>
                <label className="inline">
                  <input
                    type="checkbox"
                    checked={draft.vmt.hmd_listen_enabled}
                    onChange={(e) => setVmt({ hmd_listen_enabled: e.target.checked })}
                  />
                  hmd_listen
                </label>
              </div>
              <h3>SlimeVR</h3>
              <div className="row">
                <label className="inline">
                  <input
                    type="checkbox"
                    checked={draft.slimevr.slimevr_out}
                    onChange={(e) => setSlimevr({ slimevr_out: e.target.checked })}
                  />
                  enable
                </label>
                <label>
                  host
                  <input
                    type="text"
                    value={draft.slimevr.host}
                    onChange={(e) => setSlimevr({ host: e.target.value })}
                  />
                </label>
                <label>
                  port
                  <input
                    type="number"
                    value={draft.slimevr.port}
                    onChange={(e) => setSlimevr({ port: Number(e.target.value) })}
                  />
                </label>
              </div>
            </section>
          )}

          {/* 4. 3D + calibration */}
          {draft && (
            <section className="card">
              <h2>4. 3D / 校正</h2>
              <div className="form-grid">
                <label className="inline">
                  <input
                    type="checkbox"
                    checked={draft.three_d.enable_3d}
                    onChange={(e) => setThreeD({ enable_3d: e.target.checked })}
                  />
                  enable_3d
                </label>
                <PathField
                  label="calib path"
                  value={draft.three_d.calib}
                  onChange={(v) => setThreeD({ calib: v })}
                  placeholder="calibrations/extrinsics.yaml"
                />
                <label className="inline">
                  <input
                    type="checkbox"
                    checked={draft.intrinsic_calib.enabled}
                    onChange={(e) =>
                      setDraft((d) =>
                        d ? { ...d, intrinsic_calib: { enabled: e.target.checked } } : d,
                      )
                    }
                  />
                  intrinsic_calib
                </label>
                <label>
                  extrinsic method
                  <select
                    value={draft.extrinsic_calib.method}
                    onChange={(e) => setExtrinsic({ method: e.target.value })}
                  >
                    <option value="controller">controller</option>
                    <option value="floor">floor</option>
                  </select>
                </label>
                <label>
                  extrinsic out
                  <input
                    type="text"
                    value={draft.extrinsic_calib.out}
                    onChange={(e) => setExtrinsic({ out: e.target.value })}
                  />
                </label>
                <label>
                  web host
                  <input
                    type="text"
                    value={draft.web.host}
                    onChange={(e) => setWeb({ host: e.target.value })}
                  />
                </label>
                <label>
                  web port
                  <input
                    type="number"
                    value={draft.web.port}
                    onChange={(e) => setWeb({ port: Number(e.target.value) })}
                  />
                </label>
              </div>
            </section>
          )}

          {/* 5. Named configs */}
          {draft && (
            <section className="card">
              <h2>5. 名前付き設定</h2>
              <div className="row">
                <label>
                  load
                  <select value={selectedNamed} onChange={(e) => setSelectedNamed(e.target.value)}>
                    <option value="">(選択)</option>
                    {named.map((n) => <option key={n} value={n}>{n}</option>)}
                  </select>
                </label>
                <button type="button" disabled={!selectedNamed} onClick={() => void doLoadNamed()}>読込</button>
                <label>
                  名前を付けて保存
                  <input
                    type="text"
                    value={saveName}
                    placeholder="config name"
                    onChange={(e) => setSaveName(e.target.value)}
                  />
                </label>
                <button type="button" onClick={() => void doSaveNamed()}>保存</button>
              </div>
              <div className={`msg ${cfgMsg.err ? "err" : ""}`.trim()}>{cfgMsg.text}</div>
            </section>
          )}

          {/* 6. Validate + proceed */}
          {draft && (
            <section className="card">
              <h2>6. 検証して進む</h2>
              <div className="actions">
                <button type="button" onClick={() => void doValidate()}>検証</button>
                <button type="button" className="primary" onClick={() => void doProceed()}>次へ進む</button>
              </div>
              <div className={`msg ${validateMsg.err ? "err" : ""}`.trim()}>{validateMsg.text}</div>
              <div className={`msg ${proceedMsg.err ? "err" : ""}`.trim()}>{proceedMsg.text}</div>
            </section>
          )}
        </main>
      </WizardLayout>
    </div>
  );
}
