import { useState } from "react";
import { Link } from "react-router-dom";
import { useFlowWatch } from "../hooks/useFlowWatch";
import { WizardLayout } from "../components/WizardLayout";
import { usePolling } from "../hooks/usePolling";
import { fetchCalibState, postCalib } from "../lib/api";
import "../styles/subject-calib.css";

interface Pose {
  name: string;
  recorded?: boolean;
  buffered?: number[];
  fps?: number[];
}
interface Angles {
  l_elbow?: number;
  r_elbow?: number;
  l_sh_abd?: number;
  r_sh_abd?: number;
  l_knee?: number;
  r_knee?: number;
  torso_tilt?: number;
}
interface CalibState {
  state?: string;
  target_pose?: string;
  target_pose_idx?: number;
  bone_drift_pct?: number;
  in_band?: boolean;
  failing_axis?: string;
  hold_progress?: number;
  hold_elapsed_sec?: number;
  required_hold_sec?: number;
  recording_frames_per_cam?: number;
  poses?: Pose[];
  angles_valid?: boolean;
  angles?: Angles;
  quality_status?: string;
  analyze_exit?: number | string;
  analyze_log_tail?: string;
  last_error?: string;
  session_dir?: string;
  latest_profile?: string;
}

const clamp01 = (x: number) => Math.max(0, Math.min(1, x));
const pct = (x: number) => `${(clamp01(x) * 100).toFixed(0)}%`;
const fmt = (n: unknown, d = 1) =>
  typeof n === "number" && isFinite(n) ? n.toFixed(d) : "-";

const ANGLE_ROWS: Array<[string, keyof Angles]> = [
  ["L elbow flex", "l_elbow"],
  ["R elbow flex", "r_elbow"],
  ["L shoulder abd", "l_sh_abd"],
  ["R shoulder abd", "r_sh_abd"],
  ["L knee flex", "l_knee"],
  ["R knee flex", "r_knee"],
  ["torso tilt", "torso_tilt"],
];

export function SubjectCalibPage() {
  const flow = useFlowWatch({ page: "calib-subject" });
  const { data, error } = usePolling<CalibState>(fetchCalibState, 200);
  const s: CalibState = data ?? {};

  const [subjectId, setSubjectId] = useState("subject01");
  const [heightCm, setHeightCm] = useState("170");
  const [holdSec, setHoldSec] = useState("1.5");
  const [frames, setFrames] = useState("75");
  const [force, setForce] = useState(false);
  const [preflightMsg, setPreflightMsg] = useState<{ text: string; err: boolean }>({ text: "", err: false });
  const [reviewMsg, setReviewMsg] = useState<{ text: string; err: boolean }>({ text: "", err: false });

  const conn =
    flow.status === "down"
      ? "disconnected - waiting for restart..."
      : error
        ? "unavailable - restart main with --calibrate"
        : data
          ? "ok"
          : "connecting…";
  const stateName = s.state ?? "idle";
  const pidx = s.target_pose_idx ?? 0;
  const tgt = (s.poses || [])[pidx];
  const cap = s.recording_frames_per_cam || 75;

  // hold/recording bar
  let holdBarWidth = "0%";
  let holdLabel = "0%";
  if (stateName === "recording") {
    const buf0 = tgt?.buffered?.[0] ?? 0;
    const buf1 = tgt?.buffered?.[1] ?? 0;
    const minBuf = Math.min(buf0, buf1);
    holdBarWidth = pct(minBuf / cap);
    holdLabel = `REC ${minBuf}/${cap}`;
  } else {
    const hold = clamp01(s.hold_progress || 0);
    holdBarWidth = pct(hold);
    const holdSecLabel =
      typeof s.hold_elapsed_sec === "number"
        ? ` (${fmt(s.hold_elapsed_sec, 1)}/${fmt(s.required_hold_sec, 1)}s)`
        : "";
    holdLabel = `${(hold * 100).toFixed(0)}%${holdSecLabel}`;
  }

  const c0 = tgt?.buffered?.[0] ?? 0;
  const c1 = tgt?.buffered?.[1] ?? 0;

  const q = s.quality_status || "";
  const badgeClass = q === "pass" ? "pass" : q === "warn" ? "warn" : q === "fail" ? "fail" : "";
  let qualitySummary: string;
  if (q) {
    qualitySummary = `quality_status=${q}\nanalyze_log_tail (last 8KB):\n${s.analyze_log_tail || ""}`;
  } else if (s.last_error) {
    qualitySummary = `state=${stateName}\nerror=${s.last_error}\nanalyze_log_tail (last 8KB):\n${s.analyze_log_tail || ""}`;
  } else if (stateName === "finalizing" || stateName === "analyzing") {
    qualitySummary = `state=${stateName}\nanalyze_log_tail (last 8KB):\n${s.analyze_log_tail || ""}`;
  } else {
    qualitySummary = "(no analysis yet)";
  }

  const inactive = ["idle", "ready", "approved", "canceled", "failed"].includes(stateName);
  const preflightDisabled = !inactive || stateName === "approving";
  const startDisabled = stateName !== "ready";
  const cancelDisabled = inactive;
  const approveDisabled = !(stateName === "review" && (q === "pass" || q === "warn"));

  const doPreflight = async () => {
    setPreflightMsg({ text: "", err: false });
    const sid = subjectId.trim();
    const h = parseFloat(heightCm) / 100.0;
    const hold = parseFloat(holdSec);
    const f = parseInt(frames, 10);
    if (!sid) return setPreflightMsg({ text: "subject_id is required", err: true });
    if (!(h > 0)) return setPreflightMsg({ text: "height invalid", err: true });
    if (!(hold > 0)) return setPreflightMsg({ text: "hold time invalid", err: true });
    if (!(f > 0)) return setPreflightMsg({ text: "frames invalid", err: true });
    try {
      const res = await postCalib("preflight", {
        subject_id: sid,
        subject_height_m: h,
        required_hold_sec: hold,
        recording_frames_per_cam: f,
      });
      setPreflightMsg({ text: res.ok ? "ready" : res.err || "failed", err: !res.ok });
    } catch (e) {
      setPreflightMsg({ text: (e as Error).message || "network error", err: true });
    }
  };

  const doStart = async () => {
    try {
      const res = await postCalib("start", {});
      if (!res.ok) setPreflightMsg({ text: res.err || "start failed", err: true });
    } catch (e) {
      setPreflightMsg({ text: (e as Error).message || "start failed", err: true });
    }
  };

  const doCancel = () => void postCalib("cancel", {});
  const doRetake = (pose: string) => void postCalib("retake", { pose });

  const doApprove = async () => {
    setReviewMsg({ text: "", err: false });
    try {
      const res = await postCalib<{ ok?: boolean; err?: string; next_step?: string }>(
        "approve",
        { force },
      );
      setReviewMsg({
        text: res.ok ? `approved - ${res.next_step || "profile written"}` : res.err || "approve failed",
        err: !res.ok,
      });
    } catch (e) {
      setReviewMsg({ text: (e as Error).message || "approve failed", err: true });
    }
  };

  return (
    <WizardLayout current="subject" flow={flow.state}>
    <div className="subject-page">
      <header>
        <h1>Subject Profile Calibration</h1>
        <div className="conn-group">
          <div className="conn">{conn}</div>
          <Link className="conn link" to="/">live</Link>
        </div>
      </header>

      <main>
        <section className="card">
          <h2>1. Preflight</h2>
          <p>身長を入力して開始します。3D 角度ベース自動ポーズ認識に必須です。</p>
          <div className="form-row">
            <label>Subject ID
              <input value={subjectId} autoComplete="off" onChange={(e) => setSubjectId(e.target.value)} />
            </label>
            <label>Height (cm)
              <input type="number" min={100} max={230} step={0.1} value={heightCm}
                     onChange={(e) => setHeightCm(e.target.value)} />
            </label>
            <label>Hold (s)
              <input type="number" min={0.5} max={5} step={0.1} value={holdSec}
                     onChange={(e) => setHoldSec(e.target.value)} />
            </label>
            <label>Frames/cam
              <input type="number" min={30} max={300} step={5} value={frames}
                     onChange={(e) => setFrames(e.target.value)} />
            </label>
          </div>
          <div className="actions">
            <button type="button" disabled={preflightDisabled} onClick={doPreflight}>Preflight</button>
            <button type="button" disabled={startDisabled} onClick={doStart}>Start (auto)</button>
            <button type="button" disabled={cancelDisabled} onClick={doCancel}>Cancel</button>
          </div>
          <div className={`msg ${preflightMsg.err ? "err" : ""}`.trim()}>{preflightMsg.text}</div>
        </section>

        <section className="card">
          <h2>2. Capture</h2>
          <div className="capture-grid">
            <div>
              <h3>Target: <span>{s.target_pose || "-"}</span></h3>
              <div className="bar"><div className="bar-fill hold" style={{ width: holdBarWidth }} /></div>
              <div className="bar-label">hold <span>{holdLabel}</span></div>
              <div className="bar"><div className="bar-fill rec" style={{ width: pct(c0 / cap) }} /></div>
              <div className="bar-label">cam0 <span>{c0}</span> frames</div>
              <div className="bar"><div className="bar-fill rec" style={{ width: pct(c1 / cap) }} /></div>
              <div className="bar-label">cam1 <span>{c1}</span> frames</div>
              <div className="status-line">
                state: <span>{stateName}</span> &middot;
                drift: <span>{fmt(s.bone_drift_pct, 2)}</span>% &middot;
                in_band: <span>{s.in_band ? "yes" : "no"}</span> &middot;
                failing: <span>{s.failing_axis || "-"}</span>
              </div>
            </div>
            <div>
              <h3>3D angles (deg)</h3>
              <table className="angles">
                <tbody>
                  {ANGLE_ROWS.map(([label, key]) => (
                    <tr key={key}>
                      <td>{label}</td>
                      <td>{s.angles_valid && s.angles ? fmt(s.angles[key]) : "-"}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          </div>
          <div className="poses-list">
            {(s.poses || []).map((p, i) => (
              <div
                key={p.name}
                className={`pose-item${p.recorded ? " recorded" : ""}${i === pidx ? " current" : ""}`}
              >
                <div className="name">{p.name}</div>
                <div>buf {p.buffered?.[0] || 0} / {p.buffered?.[1] || 0}</div>
                {p.recorded && (
                  <>
                    <div>fps {fmt(p.fps?.[0], 1)} / {fmt(p.fps?.[1], 1)}</div>
                    <button type="button" className="retake" onClick={() => doRetake(p.name)}>Retake</button>
                  </>
                )}
              </div>
            ))}
          </div>
        </section>

        <section className="card">
          <h2>3. Review</h2>
          <div className="status-line">
            quality: <span className={`badge ${badgeClass}`.trim()}>{q || "-"}</span> &middot;
            analyze exit: <span>{s.analyze_exit ?? "-"}</span>
          </div>
          <pre className="log">{qualitySummary}</pre>
          <div className="actions">
            <label className="checkbox">
              <input type="checkbox" checked={force} onChange={(e) => setForce(e.target.checked)} /> force (approve warn)
            </label>
            <button type="button" disabled={approveDisabled} onClick={doApprove}>Approve &amp; write profile</button>
          </div>
          <div className={`msg ${reviewMsg.err ? "err" : ""}`.trim()}>{reviewMsg.text}</div>
        </section>

        <section className="card">
          <h2>Log</h2>
          <pre className="log">{s.analyze_log_tail || ""}</pre>
          <div className="status-line">
            session dir: <span>{s.session_dir || "-"}</span> &middot;
            latest_profile: <span>{s.latest_profile || "-"}</span>
          </div>
        </section>
      </main>
    </div>
    </WizardLayout>
  );
}
