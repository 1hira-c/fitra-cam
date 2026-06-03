import { useEffect, useRef, useState } from "react";
import {
  postAutoTpose,
  postContinuousAlign,
  postMotionStart,
  postMotionStop,
} from "../lib/api";
import type { AutoAlignResult, ContinuousAlignBlock, VmtAlignment } from "../types/bundle";

const DURATION_S = 3.0;
const SAMPLE_HZ = 30.0;

interface Props {
  hmdStatus: { text: string; cls: string };
  continuousAlign: ContinuousAlignBlock | null;
  onAlignmentResolved: (alignment: VmtAlignment) => void;
}

function describeAutoResult(result: AutoAlignResult | undefined): string {
  if (!result) return "—";
  if (result.status !== "ok") {
    return `${result.status}${result.err ? `: ${result.err}` : ""}`;
  }
  const a = result.alignment || ({} as VmtAlignment);
  const yaw = Number.isFinite(a.yaw_deg) ? a.yaw_deg : 0;
  const tx = Number.isFinite(a.x) ? a.x : 0;
  const tz = Number.isFinite(a.z) ? a.z : 0;
  const res = Number.isFinite(result.residual_m) ? (result.residual_m as number) : 0;
  return `yaw=${yaw.toFixed(2)}° tx=${tx.toFixed(3)} tz=${tz.toFixed(3)} residual=${res.toFixed(4)}m (n=${result.n_samples})`;
}

export function VmtAutoForm({ hmdStatus, continuousAlign, onAlignmentResolved }: Props) {
  const [result, setResult] = useState("—");
  const [collecting, setCollecting] = useState(false);
  const motionTimer = useRef<ReturnType<typeof setTimeout> | null>(null);

  const onContinuousToggle = async (enabled: boolean) => {
    const err = await postContinuousAlign(enabled);
    if (err) setResult(err);
  };

  useEffect(() => () => {
    if (motionTimer.current !== null) clearTimeout(motionTimer.current);
  }, []);

  const onTpose = async () => {
    setResult("solving…");
    const data = await postAutoTpose();
    if (data.ok === false) {
      setResult(data.err || "request failed");
      return;
    }
    setResult(describeAutoResult(data.result));
    if (data.result?.alignment) onAlignmentResolved(data.result.alignment);
  };

  const stopMotion = async () => {
    if (motionTimer.current !== null) {
      clearTimeout(motionTimer.current);
      motionTimer.current = null;
    }
    const data = await postMotionStop();
    if (data.ok === false) {
      setResult(data.err || "request failed");
    } else {
      setResult(describeAutoResult(data.result));
      if (data.result?.status === "ok" && data.result.alignment) {
        onAlignmentResolved(data.result.alignment);
      }
    }
    setCollecting(false);
  };

  const startMotion = async () => {
    setCollecting(true);
    setResult("collecting…");
    const data = await postMotionStart(DURATION_S, SAMPLE_HZ);
    if (data.ok === false) {
      setCollecting(false);
      setResult(data.err || "request failed");
      return;
    }
    motionTimer.current = setTimeout(
      () => void stopMotion(),
      Math.round((DURATION_S + 0.4) * 1000),
    );
  };

  return (
    <form className="vmt-align" aria-label="VMT auto alignment" onSubmit={(e) => e.preventDefault()}>
      <div className="vmt-align-head">
        <h3>VMT auto-alignment</h3>
        <span className={`vmt-align-status ${hmdStatus.cls}`.trim()}>{hmdStatus.text}</span>
      </div>
      <div className="vmt-axis-row">
        <button type="button" onClick={onTpose} disabled={collecting}>Tポーズで合わせる</button>
        <button type="button" onClick={startMotion} disabled={collecting}>移動キャリブ開始</button>
        <button type="button" onClick={stopMotion} disabled={!collecting}>停止</button>
        <output className="vmt-align-total">{result}</output>
      </div>
      <div className="vmt-axis-row">
        <label>
          <input
            type="checkbox"
            disabled={!continuousAlign}
            checked={!!continuousAlign?.enabled}
            onChange={(e) => void onContinuousToggle(e.target.checked)}
          />
          <span>
            {!continuousAlign
              ? "自動追従 (未接続)"
              : continuousAlign.enabled
                ? `自動追従 ON (cells ${continuousAlign.occupied_cells ?? 0}/${continuousAlign.min_cells ?? 0})`
                : "自動追従 OFF"}
          </span>
        </label>
      </div>
    </form>
  );
}
