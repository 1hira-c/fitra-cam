// Extrinsic calibration page (TABLE UI only — the Three.js 3D scene is
// deferred). Polls /api/excal/state @ ~200ms; method-polymorphic (controller
// 案C / floor 案D). Port of legacy web/extrinsic_calibration/app.js. The method
// toggle is a flow switch (useFlowSwitch); the rest mirrors SubjectCalibPage.

import { useState } from "react";
import { Link } from "react-router-dom";
import { WizardLayout } from "../components/WizardLayout";
import { useFlowSwitch } from "../hooks/useFlowSwitch";
import { useFlowWatch } from "../hooks/useFlowWatch";
import { usePolling } from "../hooks/usePolling";
import { fetchExcalState, postExcal } from "../lib/api";
import type {
  ExcalDetection,
  ExcalMethod,
  ExcalResultCamera,
  ExcalState,
} from "../types/bundle";
import "../styles/calib.css";

const fmt = (n: unknown, d = 2): string =>
  typeof n === "number" && isFinite(n) ? n.toFixed(d) : "-";
const clamp01 = (x: number) => Math.max(0, Math.min(1, x));

const BADGE: Record<string, string> = {
  idle: "",
  collecting: "ok",
  solving: "warn",
  solved: "pass",
  failed: "fail",
};

// Switching method = switching mode: both modes serve this one page.
const MODE_FOR_METHOD: Record<ExcalMethod, "calib-extrinsic" | "calib-extrinsic-floor"> = {
  controller: "calib-extrinsic",
  floor: "calib-extrinsic-floor",
};

// gate_reason -> [label text, css class]
const GATE_LABEL: Record<string, [string, string]> = {
  NO_TAG: ["NO TAG — point a camera at the marker", "moving"],
  NO_POSE: ["NO POSE — controller tracking lost", "moving"],
  MOVING: ["MOVING — hold still", "moving"],
  GOOD: ["GOOD — capturing", "good"],
};

// Build a coverage map keyed "cam:face|tag" -> count (controller reports `face`,
// floor reports `tag`; accept either).
function coverageMap(s: ExcalState): Record<string, number> {
  const m: Record<string, number> = {};
  for (const c of s.coverage ?? []) {
    m[`${c.cam}:${c.face ?? c.tag}`] = c.count;
  }
  return m;
}

export function ExtrinsicCalibPage() {
  const flow = useFlowWatch({ page: "calib-extrinsic" });
  const { switchTo, banner } = useFlowSwitch();
  const { data, error } = usePolling<ExcalState>(fetchExcalState, 200);
  const s: ExcalState = data ?? { state: "idle" };

  const [msg, setMsg] = useState<{ text: string; err: boolean }>({ text: "", err: false });

  const method: ExcalMethod = s.method === "floor" ? "floor" : "controller";
  const floor = method === "floor";
  const managed = !!flow.state?.managed;

  const conn =
    flow.status === "down" || error
      ? "disconnected — waiting for restart…"
      : data
        ? "ok"
        : "connecting…";

  const stateName = s.state ?? "idle";
  const collecting = stateName === "collecting";
  const startDisabled = collecting;
  const stopDisabled = !collecting;
  const minN = s.min_samples ?? s.burst_min ?? 1;

  const switchMethod = (m: ExcalMethod) => {
    if (!managed) {
      setMsg({ text: "method switching needs the flow daemon", err: true });
      return;
    }
    setMsg({ text: `switching to ${m}…`, err: false });
    const label = m === "floor" ? "Floor AprilTag (案D)" : "Controller marker (案C)";
    void switchTo(MODE_FOR_METHOD[m], label, { confirm: false });
  };

  const doStart = async () => {
    const r = await postExcal("start");
    setMsg({ text: r.ok ? "collecting" : r.err || "start failed", err: !r.ok });
  };
  const doStop = async () => {
    const r = await postExcal("stop");
    setMsg({ text: `stopped (${r.samples ?? 0} samples)`, err: false });
  };
  const doSolve = async () => {
    setMsg({ text: "solving…", err: false });
    const r = await postExcal("solve");
    setMsg({
      text: r.ok ? `solved — ${r.next_step || "extrinsics written"}` : r.err || "solve failed",
      err: !r.ok,
    });
  };

  // --- Capture gate (controller only) ---------------------------------------
  const lin = s.lin_vel_mps ?? NaN;
  const ang = s.ang_vel_dps ?? NaN;
  const linMax = s.gate?.lin_max ?? 0;
  const angMax = s.gate?.ang_max ?? 0;
  const linOver = lin > linMax;
  const angOver = ang > angMax;
  const gateReason =
    s.gate_reason ||
    (stateName !== "collecting" ? "IDLE" : linOver || angOver ? "MOVING" : "GOOD");
  const [gateText, gateCls] =
    gateReason === "IDLE"
      ? [stateName.toUpperCase(), "idle"]
      : GATE_LABEL[gateReason] || [gateReason, "idle"];

  // --- Coverage matrix ------------------------------------------------------
  const faces = (floor ? s.tags : s.faces) ?? [];
  const nCams = s.num_cams ?? 0;
  const cov = coverageMap(s);
  const corner = floor ? "cam＼tag" : "cam＼face";

  // --- Result table ---------------------------------------------------------
  const resultCams = s.cameras ?? [];
  const resultMsg =
    resultCams.length === 0
      ? stateName === "failed"
        ? "solve failed — see raw state below"
        : "(no solution yet)"
      : "";

  return (
    <div className="calib-page">
      <header>
        <h1>{floor ? "Floor AprilTag Extrinsic Calibration" : "Controller-Marker Extrinsic Calibration"}</h1>
        <div className="conn-group">
          <div className="conn">{conn}</div>
          <Link className="conn link" to="/">live</Link>
        </div>
      </header>

      <WizardLayout current="extrinsic" flow={flow.state}>
        {banner && <div className={`wizard-switch-banner ${banner.cls}`.trim()}>{banner.text}</div>}
        <main>
          <section className="card">
            <h2>Method</h2>
            <div className="actions">
              <button
                type="button"
                className={!floor ? "active" : ""}
                disabled={!managed || !floor}
                onClick={() => switchMethod("controller")}
              >
                Controller marker (案C)
              </button>
              <button
                type="button"
                className={floor ? "active" : ""}
                disabled={!managed || floor}
                onClick={() => switchMethod("floor")}
              >
                Floor AprilTag (案D)
              </button>
            </div>
            <p className="hint">
              現在の方式: <b>{floor ? "Floor AprilTag (案D)" : "Controller marker (案C)"}</b>。
              方式を切り替えると flow daemon がそのモードで再起動します。
            </p>
          </section>

          <section className="card">
            <h2>1. Control</h2>
            {floor ? (
              <p>
                床に既知配置した AprilTag を各カメラから安定検出させ、<b>各 (cam,tag) を burst 数以上</b>
                蓄積したら Solve。カメラ・タグは静止前提（VR 不要）。広く散らしつつ最悪カメラから
                edge ≥ 30px を維持、遠い位置ほど大きいタグを。面外スタンドのタグで平面縮退を回避。
              </p>
            ) : (
              <p>
                コントローラ固定 AprilTag を各カメラの近くに持って行き、<b>一瞬止める</b>と
                その姿勢のサンプルが溜まります。全カメラ×全 face を姿勢多様性込みで被覆したら Solve。
                取得は短時間 1 セッションで（recenter 厳禁）。
              </p>
            )}
            <div className="status-line">
              state: <span className={`badge ${BADGE[stateName] || ""}`.trim()}>{stateName}</span> &middot;
              samples: <span>{s.samples ?? 0}</span> &middot;
              cams: <span>{s.num_cams ?? "-"}</span> &middot;
              min/group: <span>{s.min_samples ?? s.burst_min ?? "-"}</span>
            </div>
            <div className="actions">
              <button type="button" disabled={startDisabled} onClick={doStart}>Start collecting</button>
              <button type="button" disabled={stopDisabled} onClick={doStop}>Stop</button>
              <button type="button" onClick={doSolve}>Solve &amp; write</button>
            </div>
            <div className={`msg ${msg.err ? "err" : ""}`.trim()}>{msg.text}</div>
          </section>

          {!floor && (
            <section className="card">
              <h2>2. Capture gate <span className="hint">(glanceable)</span></h2>
              <div className={`gate ${gateCls}`.trim()}>{gateText}</div>
              <div className="bar">
                <div
                  className={`bar-fill ${linOver ? "over" : ""}`.trim()}
                  style={{ width: `${clamp01(lin / (linMax || 1)) * 100}%` }}
                />
              </div>
              <div className="bar-label">linear <span>{fmt(lin, 3)}</span> / <span>{fmt(linMax, 3)}</span> m/s</div>
              <div className="bar">
                <div
                  className={`bar-fill ${angOver ? "over" : ""}`.trim()}
                  style={{ width: `${clamp01(ang / (angMax || 1)) * 100}%` }}
                />
              </div>
              <div className="bar-label">angular <span>{fmt(ang, 2)}</span> / <span>{fmt(angMax, 1)}</span> deg/s</div>
              <p className="hint">
                理由: <b>NO_TAG</b> どのカメラもタグ未検出 / <b>NO_POSE</b> コントローラ tracking 不良 /
                <b>MOVING</b> 動いている / <b>GOOD</b> 撮影中。
              </p>
            </section>
          )}

          <section className="card">
            <h2>Live detections <span className="hint">(per camera)</span></h2>
            <table className="detections">
              <thead>
                <tr>
                  <th>cam</th>
                  <th>controller</th>
                  <th>{floor ? "tags (id · reproj px)" : "faces (id · reproj px)"}</th>
                  <th>age</th>
                </tr>
              </thead>
              <tbody>
                {(s.detections ?? []).length === 0 ? (
                  <tr><td className="muted" colSpan={4}>no frames yet</td></tr>
                ) : (
                  (s.detections ?? []).map((d: ExcalDetection) => {
                    const items = (floor ? d.tags : d.faces) ?? [];
                    const stale = d.age_ms > 750;
                    return (
                      <tr key={d.cam}>
                        <td>cam{d.cam}</td>
                        {floor ? (
                          <td className="muted">—</td>
                        ) : (
                          <td className={d.ctrl_ok ? "ok-txt" : "bad-txt"}>{d.ctrl_ok ? "OK" : "—"}</td>
                        )}
                        <td>
                          {items.length === 0 ? (
                            <span className="muted">none</span>
                          ) : (
                            items.map((f, i) => (
                              <span key={i} className={`tag ${f.ok ? "ok" : "bad"}`}>
                                {f.id}·{fmt(f.reproj, 2)}
                              </span>
                            ))
                          )}
                        </td>
                        <td className={stale ? "muted" : ""}>{fmt(d.age_ms, 0)} ms</td>
                      </tr>
                    );
                  })
                )}
              </tbody>
            </table>
          </section>

          <section className="card">
            <h2>3. Coverage <span className="hint">(emitted samples per cam × {floor ? "tag" : "face"})</span></h2>
            <table className="matrix">
              <tbody>
                {nCams === 0 || faces.length === 0 ? (
                  <tr><td className="muted">no cameras/{floor ? "tags" : "faces"} configured</td></tr>
                ) : (
                  <>
                    <tr>
                      <th>{corner}</th>
                      {faces.map((f) => <th key={f}>{f}</th>)}
                    </tr>
                    {Array.from({ length: nCams }, (_, cam) => (
                      <tr key={cam}>
                        <th>cam{cam}</th>
                        {faces.map((f) => {
                          const n = cov[`${cam}:${f}`] || 0;
                          const cls = n === 0 ? "empty" : n >= minN ? "ok" : "low";
                          return <td key={f} className={`cell ${cls}`}>{n}</td>;
                        })}
                      </tr>
                    ))}
                  </>
                )}
              </tbody>
            </table>
            <div className="bar-label">
              セル色: <span className="cell empty">0</span>{" "}
              <span className="cell low">1..min-1</span>{" "}
              <span className="cell ok">&ge; min</span>
            </div>
          </section>

          <section className="card">
            <h2>4. Result <span className="hint">(after solve)</span></h2>
            <table className="result">
              <thead>
                {floor ? (
                  <tr><th>cam</th><th>tags</th><th>reproj (px)</th><th>planar?</th><th>plane thickness (mm)</th></tr>
                ) : (
                  <tr><th>cam</th><th>faces</th><th>samples</th><th>face spread (mm)</th><th>face spread (deg)</th></tr>
                )}
              </thead>
              <tbody>
                {resultCams.map((c: ExcalResultCamera) =>
                  floor ? (
                    <tr key={c.cam}>
                      <td>cam{c.cam}</td>
                      <td>{c.n_tags ?? "-"}</td>
                      <td>{fmt(c.reproj_rms_px, 3)}</td>
                      <td>{c.planar_degenerate ? "DEGENERATE" : "ok"}</td>
                      <td>{fmt((c.plane_thickness_m || 0) * 1000, 1)}</td>
                    </tr>
                  ) : (
                    <tr key={c.cam}>
                      <td>cam{c.cam}</td>
                      <td>{c.n_faces ?? "-"}</td>
                      <td>{c.n_samples ?? "-"}</td>
                      <td>{fmt((c.face_spread_trans_m || 0) * 1000, 2)}</td>
                      <td>{fmt(c.face_spread_rot_deg, 3)}</td>
                    </tr>
                  ),
                )}
              </tbody>
            </table>
            <div className={`msg ${stateName === "failed" ? "err" : ""}`.trim()}>{resultMsg}</div>
          </section>

          <section className="card">
            <h2>Raw state</h2>
            <pre className="log">{JSON.stringify(s, null, 2)}</pre>
          </section>
        </main>
      </WizardLayout>
    </div>
  );
}
