import { TRACKER_COUNT, TRACKER_ROLES } from "../lib/skeleton";
import { fmtPct, fmtRad } from "../lib/format";
import type { Tracker } from "../types/bundle";

interface Props {
  trackers: Tracker[];
}

const HEADERS = [
  "tracker",
  "state",
  "ang vel p50 (rad/s)",
  "ang vel p95",
  "conf avg",
  "leakage",
  "freeze",
  "freeze max (ms)",
  "dropouts",
];

export function TrackerStatsTable({ trackers }: Props) {
  return (
    <table className="trackers-table" aria-label="tracker stats">
      <thead>
        <tr>{HEADERS.map((h) => <th key={h}>{h}</th>)}</tr>
      </thead>
      <tbody>
        {Array.from({ length: TRACKER_COUNT }, (_, i) => {
          const role = TRACKER_ROLES[i];
          const t = trackers[i];
          if (!t) {
            return (
              <tr key={role} data-role={role}>
                <td>{role}</td>
                {Array.from({ length: 8 }, (_, c) => <td key={c}>-</td>)}
              </tr>
            );
          }
          const st = t.stats || {};
          const leak = Number(st.leakage_pct ?? 0);
          const frz = Number(st.freeze_pct ?? 0);
          const freezeMs = Number(st.freeze_current_ms ?? 0);

          let rowClass = "state-active";
          let label = "active";
          if (frz >= 0.5) {
            rowClass = "state-frozen";
            label = "frozen";
          } else if (leak >= 0.5) {
            rowClass = "state-leakage";
            label = "leakage";
          } else if (freezeMs >= 200) {
            label = "held";
          }

          const leakCls = leak >= 0.5 ? "bad" : leak >= 0.3 ? "warn" : undefined;
          const frzCls = frz >= 0.5 ? "bad" : frz >= 0.3 ? "warn" : undefined;

          return (
            <tr key={role} data-role={role} className={rowClass}>
              <td>{role}</td>
              <td>{label}</td>
              <td>{fmtRad(st.ang_vel_p50)}</td>
              <td>{fmtRad(st.ang_vel_p95)}</td>
              <td>{Number(st.conf_avg ?? 0).toFixed(2)}</td>
              <td className={leakCls}>{fmtPct(leak)}</td>
              <td className={frzCls}>{fmtPct(frz)}</td>
              <td>{st.freeze_max_ms ?? 0}</td>
              <td>{st.dropouts ?? 0}</td>
            </tr>
          );
        })}
      </tbody>
    </table>
  );
}
