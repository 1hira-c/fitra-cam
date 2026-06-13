// Mode-flow helper shared by every page (viewer + both calib frontends).
//
// Served from the root static dir, which is registered in every mode, so the
// calib pages can load it as /flow.js even though their own static groups
// only exist in their dedicated modes.
//
// Watches GET /api/state to follow the flow daemon's module swaps: while a
// module is restarting the fetch fails (onDown); once the next module is up,
// a page whose mode no longer matches navigates to the right page. The
// viewer opts out of the redirect and renders a banner instead
// (docs/design/pose-3d-flow-daemon.md).

window.FitraFlow = (() => {
  const PAGE_FOR_MODE = {
    "run": "/",
    "calib-subject": "/subject-calib",
    "calib-extrinsic": "/extrinsic-calib",
  };

  // watch({page, redirect, onState, onDown}) -> interval id
  //  - page: this page's mode ("run" | "calib-subject" | "calib-extrinsic")
  //  - redirect (default true): navigate to PAGE_FOR_MODE[state.mode] when
  //    the live module's mode differs from `page`
  //  - onState(state, recovered): every successful poll; `recovered` is true
  //    on the first poll after a down period
  //  - onDown(): every failed poll (module restarting / process gone)
  function watch(opts) {
    let down = false;
    let timer = null;
    const tick = async () => {
      let s = null;
      let unsupported = false;
      try {
        const res = await fetch("/api/state", { cache: "no-store" });
        if (res.ok) s = await res.json();
        // 404 = the server serves these static files but has no /api/state:
        // the Python fallback (dual_rtmpose_web.py) is not flow-aware. That is
        // a healthy server, not a restart — disable the watcher rather than
        // flashing the "restarting" banner forever.
        else if (res.status === 404) unsupported = true;
      } catch (e) { /* connection refused while the daemon swaps modules */ }
      if (unsupported) {
        if (timer) clearInterval(timer);
        return;
      }
      if (!s) {
        down = true;
        if (opts.onDown) opts.onDown();
        return;
      }
      const recovered = down;
      down = false;
      if (s.mode !== opts.page && opts.redirect !== false
          && PAGE_FOR_MODE[s.mode]) {
        location.href = PAGE_FOR_MODE[s.mode];
        return;
      }
      if (opts.onState) opts.onState(s, recovered);
    };
    tick();
    timer = setInterval(tick, 1000);
    return timer;
  }

  // POST /api/flow/switch — daemon-managed modules only (404/405 otherwise).
  async function requestSwitch(mode) {
    try {
      const res = await fetch("/api/flow/switch", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ mode }),
      });
      if (!res.ok) return { ok: false, err: `HTTP ${res.status}` };
      return await res.json();
    } catch (e) {
      return { ok: false, err: e.message || "request failed" };
    }
  }

  return { watch, requestSwitch, PAGE_FOR_MODE };
})();
