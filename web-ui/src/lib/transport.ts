// Thin REST/WebSocket helpers routed through lib/config so every network call
// honours the configured backend origin. These are the only places that touch
// `fetch` / `WebSocket` directly.

import { httpUrl, wsUrl } from "./config";

export async function getJson<T = unknown>(path: string): Promise<T> {
  const res = await fetch(httpUrl(path), { cache: "no-store" });
  return res.json() as Promise<T>;
}

/**
 * POST JSON and return the parsed response together with the raw Response so
 * callers can inspect `res.ok`. Mirrors the legacy pattern where some handlers
 * tolerate non-OK responses with an `{ok:false, err}` body.
 */
export async function postJson<T = unknown>(
  path: string,
  body?: unknown,
): Promise<{ res: Response; data: T | null }> {
  const res = await fetch(httpUrl(path), {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body ?? {}),
  });
  let data: T | null = null;
  try {
    data = (await res.json()) as T;
  } catch {
    data = null;
  }
  return { res, data };
}

/** Open a WebSocket to a backend path (scheme/host resolved via config). */
export function openWs(path: string): WebSocket {
  return new WebSocket(wsUrl(path));
}
