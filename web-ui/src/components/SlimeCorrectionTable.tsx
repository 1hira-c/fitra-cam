import { useEffect, useRef, useState } from "react";
import { fetchCorrections, postCorrection } from "../lib/api";
import { TRACKER_ROLES } from "../lib/skeleton";
import type { CorrectionsResponse } from "../types/bundle";

const AXES = ["yaw", "pitch", "roll"] as const;
type Axis = (typeof AXES)[number];
type Quarters = Record<Axis, number>;

function normalizeQuarters(q: number): number {
  let v = Number.isFinite(q) ? Math.trunc(q) : 0;
  v %= 4;
  if (v < 0) v += 4;
  return v === 3 ? -1 : v;
}

function emptyValues(): Record<string, Quarters> {
  const out: Record<string, Quarters> = {};
  for (const role of TRACKER_ROLES) out[role] = { yaw: 0, pitch: 0, roll: 0 };
  return out;
}

export function SlimeCorrectionTable() {
  const [values, setValues] = useState<Record<string, Quarters>>(emptyValues);
  const [enabled, setEnabled] = useState(false);
  const [status, setStatus] = useState<{ text: string; cls: string }>({ text: "not loaded", cls: "" });
  const valuesRef = useRef(values);
  valuesRef.current = values;

  const applyPayload = (payload: CorrectionsResponse) => {
    if (!Array.isArray(payload.roles)) return;
    const next = emptyValues();
    for (const role of TRACKER_ROLES) {
      const c = payload.roles.find((entry) => entry.role === role) || {};
      next[role] = {
        yaw: normalizeQuarters(Number((c as Record<string, number>).yaw_quarters ?? 0)),
        pitch: normalizeQuarters(Number((c as Record<string, number>).pitch_quarters ?? 0)),
        roll: normalizeQuarters(Number((c as Record<string, number>).roll_quarters ?? 0)),
      };
    }
    setValues(next);
    setEnabled(true);
    setStatus({ text: payload.preview_no_reset === false ? "preview off" : "ready", cls: "live" });
  };

  useEffect(() => {
    let cancelled = false;
    (async () => {
      try {
        const payload = await fetchCorrections();
        if (cancelled) return;
        if (!payload.ok) {
          setEnabled(false);
          setStatus({ text: payload.err || "unavailable", cls: "dead" });
          return;
        }
        applyPayload(payload);
      } catch {
        if (cancelled) return;
        setEnabled(false);
        setStatus({ text: "unavailable", cls: "dead" });
      }
    })();
    return () => {
      cancelled = true;
    };
  }, []);

  const send = async (body: Record<string, unknown>) => {
    setEnabled(false);
    setStatus({ text: "applying", cls: "" });
    const payload = await postCorrection(body);
    if (!payload.ok) {
      setStatus({ text: payload.err || "failed", cls: "dead" });
      setEnabled(true);
      return;
    }
    applyPayload(payload);
  };

  const adjust = (role: string, axis: Axis, delta: number) => {
    const cur = valuesRef.current[role];
    const next: Quarters = { ...cur, [axis]: normalizeQuarters(cur[axis] + delta) };
    void send({
      role,
      yaw_quarters: next.yaw,
      pitch_quarters: next.pitch,
      roll_quarters: next.roll,
    });
  };

  const resetRole = (role: string) => void send({ role, reset: true });
  const resetAll = () => void send({ reset: true });

  return (
    <div className="slimevr-corrections">
      <div className="slimevr-corrections-head">
        <h3>SlimeVR correction</h3>
        <button type="button" disabled={!enabled} onClick={resetAll}>reset all</button>
        <span className={`slimevr-correction-status ${status.cls}`.trim()}>{status.text}</span>
      </div>
      <table className="trackers-table slimevr-table" aria-label="SlimeVR correction controls">
        <thead>
          <tr>
            <th>tracker</th>
            <th>yaw</th>
            <th>pitch</th>
            <th>roll</th>
            <th>reset</th>
          </tr>
        </thead>
        <tbody>
          {TRACKER_ROLES.map((role) => (
            <tr key={role} data-role={role}>
              <td>{role}</td>
              {AXES.map((axis) => (
                <td key={axis}>
                  <span className="slimevr-axis-control">
                    <button type="button" disabled={!enabled} onClick={() => adjust(role, axis, -1)}>-90</button>
                    <span className="slimevr-axis-value">{values[role][axis] * 90}</span>
                    <button type="button" disabled={!enabled} onClick={() => adjust(role, axis, 1)}>+90</button>
                  </span>
                </td>
              ))}
              <td>
                <button type="button" disabled={!enabled} onClick={() => resetRole(role)}>0</button>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}
