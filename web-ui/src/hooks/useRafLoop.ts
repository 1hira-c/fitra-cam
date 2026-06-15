// Drive a per-frame callback with requestAnimationFrame. The callback is held
// in a ref so changing it doesn't restart the loop. This is where imperative
// canvas / Three.js rendering happens, kept out of React's render cycle so the
// 30Hz data stream never triggers component re-renders.

import { useEffect, useRef } from "react";

export function useRafLoop(cb: (nowMs: number) => void, enabled = true): void {
  const cbRef = useRef(cb);
  cbRef.current = cb;

  useEffect(() => {
    if (!enabled) return;
    let raf = 0;
    const tick = (now: number) => {
      cbRef.current(now);
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, [enabled]);
}
