// Auto-reconnecting JSON WebSocket. Mirrors the legacy connect()/connect3d()
// loops: 5s keep-alive ping, 1.5s reconnect backoff, JSON parse with silent
// drop on malformed frames. Messages are delivered via a ref'd callback so the
// socket isn't torn down on every render.

import { useEffect, useRef, useState } from "react";
import { openWs } from "../lib/transport";

export type WsStatus = "connecting" | "open" | "closed" | "error";

export function useWebSocketJson<T>(
  path: string,
  onMessage: (msg: T) => void,
): WsStatus {
  const [status, setStatus] = useState<WsStatus>("connecting");
  const cbRef = useRef(onMessage);
  cbRef.current = onMessage;

  useEffect(() => {
    let ws: WebSocket | null = null;
    let pingTimer: ReturnType<typeof setInterval> | null = null;
    let reconnectTimer: ReturnType<typeof setTimeout> | null = null;
    let closed = false;

    const clearPing = () => {
      if (pingTimer !== null) {
        clearInterval(pingTimer);
        pingTimer = null;
      }
    };

    const connect = () => {
      if (closed) return;
      setStatus("connecting");
      ws = openWs(path);
      ws.onopen = () => {
        setStatus("open");
        clearPing();
        pingTimer = setInterval(() => {
          if (ws && ws.readyState === WebSocket.OPEN) ws.send("ping");
        }, 5000);
      };
      ws.onclose = () => {
        clearPing();
        if (closed) return;
        setStatus("closed");
        reconnectTimer = setTimeout(connect, 1500);
      };
      ws.onerror = () => {
        clearPing();
        if (!closed) setStatus("error");
      };
      ws.onmessage = (ev) => {
        let msg: T;
        try {
          msg = JSON.parse(ev.data as string) as T;
        } catch {
          return;
        }
        cbRef.current(msg);
      };
    };

    connect();
    return () => {
      closed = true;
      clearPing();
      if (reconnectTimer !== null) clearTimeout(reconnectTimer);
      if (ws) {
        ws.onclose = null;
        ws.close();
      }
    };
  }, [path]);

  return status;
}
