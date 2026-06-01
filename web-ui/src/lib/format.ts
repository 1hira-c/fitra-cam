// Shared numeric formatting helpers ported from the legacy viewer.

/** Trim trailing zeros; "-0" → "0". Used by the VMT alignment controls. */
export function formatInputNumber(v: unknown): string {
  const n = Number.isFinite(Number(v)) ? Number(v) : 0;
  const s = n.toFixed(3).replace(/\.?0+$/, "");
  return s === "-0" ? "0" : s;
}

export function fmtPct(v: number | undefined): string {
  return `${(Math.max(0, Math.min(1, v ?? 0)) * 100).toFixed(0)}%`;
}

export function fmtRad(v: number | undefined): string {
  return (Number.isFinite(v) ? (v as number) : 0).toFixed(2);
}

export function fixed(v: number | undefined, d = 2): string {
  return (Number.isFinite(v) ? (v as number) : 0).toFixed(d);
}
