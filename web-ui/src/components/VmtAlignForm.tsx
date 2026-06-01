import {
  forwardRef,
  useCallback,
  useEffect,
  useImperativeHandle,
  useRef,
  useState,
  type FormEvent,
} from "react";
import { fetchVmtAlignment, postVmtAlignment } from "../lib/api";
import { formatInputNumber } from "../lib/format";
import type { VmtAlignment } from "../types/bundle";

type AxisKey = keyof VmtAlignment; // "x" | "y" | "z" | "yaw_deg"
const KEYS: AxisKey[] = ["x", "y", "z", "yaw_deg"];
const LABELS: Record<AxisKey, string> = { x: "X", y: "Y", z: "Z", yaw_deg: "yaw" };
const BASE_STEP: Record<AxisKey, number> = { x: 1, y: 1, z: 1, yaw_deg: 45 };
const NUMBER_STEP: Record<AxisKey, number> = { x: 1, y: 1, z: 1, yaw_deg: 45 };
// [min, max, step] for the fine-adjust slider.
const SLIDER: Record<AxisKey, [number, number, number]> = {
  x: [-0.5, 0.5, 0.005],
  y: [-0.5, 0.5, 0.005],
  z: [-0.5, 0.5, 0.005],
  yaw_deg: [-45, 45, 0.5],
};

export interface VmtAlignHandle {
  writeForm(alignment: Partial<VmtAlignment>): void;
}

type Axes = Record<AxisKey, number>;
const ZERO: Axes = { x: 0, y: 0, z: 0, yaw_deg: 0 };

function splitValue(key: AxisKey, total: number): { base: number; fine: number } {
  const step = BASE_STEP[key];
  const value = Number.isFinite(total) ? total : 0;
  let base = Math.trunc(value / step) * step;
  let fine = value - base;
  const [min, max] = SLIDER[key];
  if (fine > max) {
    base += step;
    fine -= step;
  } else if (fine < min) {
    base -= step;
    fine += step;
  }
  return { base, fine };
}

export const VmtAlignForm = forwardRef<VmtAlignHandle>(function VmtAlignForm(_props, ref) {
  const [enabled, setEnabled] = useState(false);
  const [status, setStatus] = useState<{ text: string; cls: string }>({ text: "loading", cls: "" });
  const [base, setBase] = useState<Axes>({ ...ZERO });
  const [fine, setFine] = useState<Axes>({ ...ZERO });
  const baseRef = useRef(base);
  const fineRef = useRef(fine);
  baseRef.current = base;
  fineRef.current = fine;
  const enabledRef = useRef(enabled);
  enabledRef.current = enabled;
  const postTimer = useRef<ReturnType<typeof setTimeout> | null>(null);

  const total = (key: AxisKey): number => base[key] + fine[key];

  const writeForm = useCallback((alignment: Partial<VmtAlignment>) => {
    const nb: Axes = { ...ZERO };
    const nf: Axes = { ...ZERO };
    for (const key of KEYS) {
      const { base: b, fine: f } = splitValue(key, Number(alignment[key] ?? 0));
      nb[key] = b;
      nf[key] = f;
    }
    setBase(nb);
    setFine(nf);
  }, []);

  useImperativeHandle(ref, () => ({ writeForm }), [writeForm]);

  useEffect(() => {
    let cancelled = false;
    (async () => {
      try {
        const data = await fetchVmtAlignment();
        if (cancelled) return;
        if (data.alignment) writeForm(data.alignment);
        setEnabled(!!data.enabled);
        setStatus(data.enabled ? { text: "ready", cls: "live" } : { text: "vmt off", cls: "" });
      } catch {
        if (cancelled) return;
        setEnabled(false);
        setStatus({ text: "api error", cls: "dead" });
      }
    })();
    return () => {
      cancelled = true;
      if (postTimer.current !== null) clearTimeout(postTimer.current);
    };
  }, [writeForm]);

  const readTotals = (): VmtAlignment => ({
    x: baseRef.current.x + fineRef.current.x,
    y: baseRef.current.y + fineRef.current.y,
    z: baseRef.current.z + fineRef.current.z,
    yaw_deg: baseRef.current.yaw_deg + fineRef.current.yaw_deg,
  });

  const schedulePost = (delayMs = 70) => {
    if (postTimer.current !== null) clearTimeout(postTimer.current);
    postTimer.current = setTimeout(async () => {
      postTimer.current = null;
      if (!enabledRef.current) return;
      setStatus({ text: "applying", cls: "" });
      try {
        await postVmtAlignment(readTotals());
        setStatus({ text: "applied", cls: "live" });
      } catch (e) {
        setStatus({ text: (e as Error).message || "apply failed", cls: "dead" });
      }
    }, delayMs);
  };

  const onSubmit = async (ev: FormEvent) => {
    ev.preventDefault();
    if (!enabled) return;
    if (postTimer.current !== null) {
      clearTimeout(postTimer.current);
      postTimer.current = null;
    }
    setStatus({ text: "applying", cls: "" });
    try {
      const applied = await postVmtAlignment(readTotals());
      if (applied) writeForm(applied);
      setStatus({ text: "applied", cls: "live" });
    } catch (e) {
      setStatus({ text: (e as Error).message || "apply failed", cls: "dead" });
    }
  };

  const onReset = async () => {
    if (!enabled) return;
    if (postTimer.current !== null) {
      clearTimeout(postTimer.current);
      postTimer.current = null;
    }
    writeForm(ZERO);
    setStatus({ text: "resetting", cls: "" });
    try {
      await postVmtAlignment(ZERO);
      setStatus({ text: "reset", cls: "live" });
    } catch (e) {
      setStatus({ text: (e as Error).message || "reset failed", cls: "dead" });
    }
  };

  return (
    <form className="vmt-align" aria-label="VMT alignment" onSubmit={onSubmit}>
      <div className="vmt-align-head">
        <h3>VMT alignment</h3>
        <button type="submit" disabled={!enabled}>Apply</button>
        <button type="button" disabled={!enabled} onClick={onReset}>Reset</button>
        <span className={`vmt-align-status ${status.cls}`.trim()}>{status.text}</span>
      </div>
      {KEYS.map((key) => {
        const [min, max, step] = SLIDER[key];
        return (
          <div className="vmt-axis-row" key={key}>
            <label htmlFor={`vmt-align-${key}-slider`}>{LABELS[key]}</label>
            <input
              id={`vmt-align-${key}-slider`}
              className="vmt-align-slider"
              type="range"
              min={min}
              max={max}
              step={step}
              value={fine[key]}
              disabled={!enabled}
              onChange={(e) => {
                setFine((f) => ({ ...f, [key]: Number(e.target.value) }));
                if (enabled) schedulePost();
              }}
              // mouseup/keyup commit → immediate post
              onPointerUp={() => enabled && schedulePost(0)}
              onKeyUp={() => enabled && schedulePost(0)}
            />
            <input
              className="vmt-align-number"
              type="number"
              step={NUMBER_STEP[key]}
              value={base[key]}
              disabled={!enabled}
              aria-label={`${LABELS[key]} base`}
              onChange={(e) => {
                setBase((b) => ({ ...b, [key]: Number(e.target.value) }));
                if (enabled) schedulePost(0);
              }}
            />
            <output className="vmt-align-total">{formatInputNumber(total(key))}</output>
          </div>
        );
      })}
    </form>
  );
});
