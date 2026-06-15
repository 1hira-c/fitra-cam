// Single source of truth for the backend connection target.
//
// Three deployment modes are served by one codebase:
//  1. Crow-served (production): the dist is served by the C++ Crow server, so
//     the backend is same-origin. apiBase = "".
//  2. Vite dev: same-origin paths are proxied to Crow (see vite.config.ts).
//     apiBase = "".
//  3. Separate host / future Tauri/Wails desktop app: the frontend is loaded
//     from an app bundle, not from Crow, so it needs an absolute backend URL.
//     Set via localStorage['fitra.apiBase'] (runtime) or VITE_API_BASE (build).
//
// Resolution priority: runtime override > build-time env > same-origin.
//
// All WebSocket/REST URLs MUST be built through httpUrl()/wsUrl() below — never
// hand-roll `new WebSocket(...)` or `fetch("/api/...")`. This module is the
// single seam where a future Tauri/Wails shell can inject the remote backend
// origin (or swap REST/WS for native IPC).

function resolveBase(): string {
  try {
    const rt = localStorage.getItem("fitra.apiBase");
    if (rt) return rt.replace(/\/+$/, "");
  } catch {
    // localStorage may be unavailable (SSR / sandboxed); fall through.
  }
  const env = import.meta.env.VITE_API_BASE;
  if (env) return env.replace(/\/+$/, "");
  return "";
}

export const apiBase = resolveBase();

/** Build an absolute HTTP(S) URL for a backend path (e.g. "/api/vmt/alignment"). */
export function httpUrl(path: string): string {
  return apiBase + path;
}

/** Build a WebSocket URL for a backend path (e.g. "/ws", "/ws3d"). */
export function wsUrl(path: string): string {
  if (!apiBase) {
    const proto = location.protocol === "https:" ? "wss:" : "ws:";
    return `${proto}//${location.host}${path}`;
  }
  return apiBase.replace(/^http/, "ws") + path;
}
