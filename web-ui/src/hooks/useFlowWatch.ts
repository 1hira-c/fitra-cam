import { useEffect, useState } from "react";
import { httpUrl } from "../lib/config";
import type { FlowMode, FlowState } from "../types/bundle";

export const PAGE_FOR_MODE: Record<FlowMode, string> = {
  run: "/",
  "calib-subject": "/subject-calib",
  "calib-extrinsic": "/extrinsic-calib",
  // 案D shares the /extrinsic-calib page with 案C; the page branches on method.
  "calib-extrinsic-floor": "/extrinsic-calib",
};

export type FlowWatchStatus = "unknown" | "unsupported" | "down" | "up";

interface FlowWatchOptions {
  page: FlowMode;
  redirect?: boolean;
  intervalMs?: number;
}

interface FlowWatchResult {
  state: FlowState | null;
  status: FlowWatchStatus;
}

function isFlowMode(mode: string): mode is FlowMode {
  return (
    mode === "run" ||
    mode === "calib-subject" ||
    mode === "calib-extrinsic" ||
    mode === "calib-extrinsic-floor"
  );
}

export function useFlowWatch({
  page,
  redirect = true,
  intervalMs = 1000,
}: FlowWatchOptions): FlowWatchResult {
  const [state, setState] = useState<FlowState | null>(null);
  const [status, setStatus] = useState<FlowWatchStatus>("unknown");

  useEffect(() => {
    let cancelled = false;
    let timer: ReturnType<typeof setInterval> | null = null;

    const stopTimer = () => {
      if (timer !== null) {
        clearInterval(timer);
        timer = null;
      }
    };

    const tick = async () => {
      try {
        const res = await fetch(httpUrl("/api/state"), { cache: "no-store" });
        if (res.status === 404) {
          if (!cancelled) {
            setStatus("unsupported");
            setState(null);
          }
          stopTimer();
          return;
        }
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        const raw = (await res.json()) as Partial<FlowState>;
        if (typeof raw.mode !== "string" || !isFlowMode(raw.mode)) {
          throw new Error("invalid flow state");
        }
        const next: FlowState = {
          mode: raw.mode,
          managed: !!raw.managed,
        };
        if (cancelled) return;
        setState(next);
        setStatus("up");
        // Redirect by target page, not mode label: calib-extrinsic and
        // calib-extrinsic-floor share a page, so compare the target against the
        // current path to avoid a reload loop when only the method differs.
        const target = PAGE_FOR_MODE[next.mode];
        if (redirect && target !== window.location.pathname &&
            target !== PAGE_FOR_MODE[page]) {
          window.location.href = target;
        }
      } catch {
        if (!cancelled) {
          setStatus("down");
        }
      }
    };

    void tick();
    timer = setInterval(() => void tick(), intervalMs);
    return () => {
      cancelled = true;
      stopTimer();
    };
  }, [intervalMs, page, redirect]);

  return { state, status };
}
