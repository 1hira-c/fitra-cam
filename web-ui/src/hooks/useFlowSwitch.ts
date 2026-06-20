// Shared flow-switch action (lifted from ViewerPage's switchMode so the viewer,
// the wizard stepper, and the extrinsic method toggle all use one path). A
// switch asks the daemon to respawn into another mode; useFlowWatch then
// follows the live mode. The request only stops the current module — the new
// one comes up asynchronously, so `pending` stays true until the page changes.

import { useCallback, useState } from "react";
import { requestFlowSwitch } from "../lib/api";
import type { FlowMode } from "../types/bundle";

export interface Banner {
  text: string;
  cls: string;
}

export interface FlowSwitch {
  pending: boolean;
  banner: Banner | null;
  /** Request a switch to `mode`. `confirm` (default true) gates on window.confirm. */
  switchTo: (mode: FlowMode, label: string, opts?: { confirm?: boolean }) => Promise<void>;
  clear: () => void;
}

export function useFlowSwitch(): FlowSwitch {
  const [pending, setPending] = useState(false);
  const [banner, setBanner] = useState<Banner | null>(null);

  const switchTo = useCallback(
    async (mode: FlowMode, label: string, opts?: { confirm?: boolean }) => {
      const needConfirm = opts?.confirm ?? true;
      if (
        needConfirm &&
        !window.confirm(`${label} に切り替えますか? 完了するまでトラッカー出力は停止します。`)
      ) {
        return;
      }
      setBanner(null);
      setPending(true);
      try {
        const res = await requestFlowSwitch(mode);
        if (res.ok) {
          setPending(true);
        } else {
          setPending(false);
          setBanner({ text: `切替失敗: ${res.err || "unknown error"}`, cls: "err" });
        }
      } catch (e) {
        setPending(false);
        setBanner({ text: `切替失敗: ${(e as Error).message || "unknown error"}`, cls: "err" });
      }
    },
    [],
  );

  const clear = useCallback(() => {
    setPending(false);
    setBanner(null);
  }, []);

  return { pending, banner, switchTo, clear };
}
