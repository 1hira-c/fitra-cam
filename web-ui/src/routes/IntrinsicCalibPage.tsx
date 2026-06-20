// Intrinsic (ChArUco) calibration page. Polls /api/incal/state @ ~300ms; POSTs
// start / stop / solve. Port of legacy web/intrinsic_calibration/. Mirrors the
// SubjectCalibPage structure (usePolling + {text,err} message + conn string).

import { useState } from "react";
import { Link } from "react-router-dom";
import { WizardLayout } from "../components/WizardLayout";
import { useFlowWatch } from "../hooks/useFlowWatch";
import { usePolling } from "../hooks/usePolling";
import { fetchIncalState, postIncal } from "../lib/api";
import type { IncalState } from "../types/bundle";
import "../styles/calib.css";

const fmt = (n: unknown, d = 2): string =>
  typeof n === "number" && isFinite(n) ? n.toFixed(d) : "-";

// state -> badge class, mirroring legacy BADGE map.
const BADGE: Record<string, string> = {
  idle: "",
  collecting: "ok",
  solving: "warn",
  solved: "pass",
  failed: "fail",
};

export function IntrinsicCalibPage() {
  const flow = useFlowWatch({ page: "calib-intrinsic" });
  const { data, error } = usePolling<IncalState>(fetchIncalState, 300);
  const s: IncalState = data ?? { state: "idle" };

  const [msg, setMsg] = useState<{ text: string; err: boolean }>({ text: "", err: false });

  const conn =
    flow.status === "down"
      ? "disconnected — waiting for restart…"
      : error
        ? "disconnected — waiting for restart…"
        : data
          ? "ok"
          : "connecting…";

  const stateName = s.state ?? "idle";
  const collecting = stateName === "collecting";
  // Start allowed only when idle or failed; Stop only while collecting.
  const startDisabled = !(stateName === "idle" || stateName === "failed");
  const stopDisabled = !collecting;

  const cams = s.cameras ?? [];
  const minViews = s.min_views ?? 0;

  const doStart = async () => {
    const r = await postIncal("start");
    setMsg({ text: r.ok ? "collecting" : r.err || "start failed", err: !r.ok });
  };
  const doStop = async () => {
    await postIncal("stop");
    setMsg({ text: "stopped", err: false });
  };
  const doSolve = async () => {
    setMsg({ text: "solving…", err: false });
    const r = await postIncal("solve");
    setMsg({
      text: r.ok ? `solved — ${r.next_step || "intrinsics written"}` : r.err || "solve failed",
      err: !r.ok,
    });
  };

  return (
    <div className="calib-page">
      <header>
        <h1>Intrinsic Calibration (ChArUco)</h1>
        <div className="conn-group">
          <div className="conn">{conn}</div>
          <Link className="conn link" to="/">live</Link>
        </div>
      </header>

      <WizardLayout current="intrinsic" flow={flow.state}>
        <main>
          <section className="card">
            <h2>1. Control</h2>
            <p>
              ChArUco ボードを各カメラに見せ、<b>画像の隅々・近遠・傾き</b>を変えながら
              多様なビューを溜めます。各カメラが <b>min views</b> 以上・被覆が埋まったら Solve。
              モデル: <b>{s.model || "-"}</b>（fisheye は強い魚眼レンズ向け）。
            </p>
            <div className="status-line">
              state: <span className={`badge ${BADGE[stateName] || ""}`.trim()}>{stateName}</span> &middot;
              cams: <span>{s.num_cams ?? "-"}</span> &middot;
              min views: <span>{s.min_views ?? "-"}</span>
            </div>
            <div className="actions">
              <button type="button" disabled={startDisabled} onClick={doStart}>Start collecting</button>
              <button type="button" disabled={stopDisabled} onClick={doStop}>Stop</button>
              <button type="button" onClick={doSolve}>Solve &amp; write</button>
            </div>
            <div className={`msg ${msg.err ? "err" : ""}`.trim()}>{msg.text}</div>
          </section>

          <section className="card">
            <h2>2. Per-camera coverage <span className="hint">(views · coverage · rms)</span></h2>
            <table className="result">
              <thead>
                <tr><th>cam</th><th>views</th><th>coverage</th><th>rms (px)</th><th>solved</th></tr>
              </thead>
              <tbody>
                {cams.length === 0 ? (
                  <tr><td className="muted" colSpan={5}>no cameras</td></tr>
                ) : (
                  cams.map((c) => {
                    const enough = c.views >= minViews;
                    return (
                      <tr key={c.cam}>
                        <td>cam{c.cam}</td>
                        <td className={enough ? "ok-txt" : ""}>{c.views}</td>
                        <td>{fmt((c.coverage || 0) * 100, 0)}%</td>
                        <td>{c.solved ? fmt(c.rms_px, 3) : "-"}</td>
                        <td className={c.solved ? "ok-txt" : "muted"}>{c.solved ? "yes" : "—"}</td>
                      </tr>
                    );
                  })
                )}
              </tbody>
            </table>
            <p className="hint">
              coverage は画像を 3×3 に分けた被覆率。隅・中央・近遠・傾きを変えて 1.0 に近づける。
              rms は solve 後に表示（小さいほど良い、目安 &lt; 0.5px）。
            </p>
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
