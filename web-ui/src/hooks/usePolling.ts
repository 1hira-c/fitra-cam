// Poll an async function on an interval and expose its latest resolved value.
// Used by the subject-calibration wizard (legacy: setInterval(refresh, 200)).

import { useEffect, useRef, useState } from "react";

export function usePolling<T>(
  fn: () => Promise<T>,
  intervalMs: number,
): { data: T | null; error: unknown } {
  const [data, setData] = useState<T | null>(null);
  const [error, setError] = useState<unknown>(null);
  const fnRef = useRef(fn);
  fnRef.current = fn;

  useEffect(() => {
    let cancelled = false;
    let timer: ReturnType<typeof setInterval> | null = null;
    const run = async () => {
      try {
        const v = await fnRef.current();
        if (!cancelled) {
          setData(v);
          setError(null);
        }
      } catch (e) {
        if (!cancelled) setError(e);
      }
    };
    run();
    timer = setInterval(run, intervalMs);
    return () => {
      cancelled = true;
      if (timer !== null) clearInterval(timer);
    };
  }, [intervalMs]);

  return { data, error };
}
