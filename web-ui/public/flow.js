// Mode-flow helper shared by the legacy extrinsic-calib page.
//
// The React SPA implements the same behavior in useFlowWatch.ts. This file is
// kept as a root static asset because web/extrinsic_calibration is still a
// vanilla frontend and loads /flow.js directly.

window.FitraFlow = (() => {
  const PAGE_FOR_MODE = {
    "run": "/",
    "calib-subject": "/subject-calib",
    "calib-extrinsic": "/extrinsic-calib",
    // The floor (案D) and controller (案C) extrinsic methods share one page;
    // it branches on the state JSON's "method" field.
    "calib-extrinsic-floor": "/extrinsic-calib",
    "calib-intrinsic": "/intrinsic-calib",
  };

  function watch(opts) {
    let down = false;
    let timer = null;
    const tick = async () => {
      let state = null;
      let unsupported = false;
      try {
        const res = await fetch("/api/state", { cache: "no-store" });
        if (res.ok) state = await res.json();
        else if (res.status === 404) unsupported = true;
      } catch (e) {
        // Connection refused while the daemon swaps modules.
      }
      if (unsupported) {
        if (timer) clearInterval(timer);
        return;
      }
      if (!state) {
        down = true;
        if (opts.onDown) opts.onDown();
        return;
      }
      const recovered = down;
      down = false;
      // Redirect by target page, not mode label: two modes (calib-extrinsic /
      // calib-extrinsic-floor) map to the same page, so comparing the target
      // against the current path avoids a reload loop when only the method
      // differs.
      const target = PAGE_FOR_MODE[state.mode];
      if (opts.redirect !== false && target && target !== location.pathname) {
        location.href = target;
        return;
      }
      if (opts.onState) opts.onState(state, recovered);
    };
    tick();
    timer = setInterval(tick, 1000);
    return timer;
  }

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
