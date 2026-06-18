// Controller-marker extrinsic calibration frontend.
//
// Talks to the C++ Crow server's /api/excal/* routes. Polls /api/excal/state
// at 5 Hz to drive the UI; POSTs start / stop / solve.

const $ = (id) => document.getElementById(id);

// Set by the flow watcher; read by the method selector + subject button. Hoisted
// above refresh() (called on load) to avoid a temporal-dead-zone reference.
let flowManaged = false;

async function postJSON(path) {
  // Tolerate a non-JSON / unreachable response (e.g. a flow-daemon module swap):
  // return a structured failure so callers always get {ok:false} instead of an
  // unhandled rejection that leaves the UI stuck (e.g. on "solving…").
  try {
    const res = await fetch(path, { method: "POST",
      headers: { "Content-Type": "application/json" }, body: "{}" });
    return await res.json();
  } catch (e) {
    return { ok: false, err: e.message || "request failed" };
  }
}
async function getJSON(path) { return (await fetch(path)).json(); }

function fmt(n, d = 2) {
  return typeof n === "number" && isFinite(n) ? n.toFixed(d) : "-";
}
function clamp01(x) { return Math.max(0, Math.min(1, x)); }
function setMsg(el, s, isErr = false) {
  el.textContent = s || "";
  el.classList.toggle("err", isErr);
}

const BADGE = {
  idle: "", collecting: "ok", solving: "warn", solved: "pass", failed: "fail",
};

// Coverage map keyed "cam:face|tag" -> count. The controller path reports
// `face`, the floor path `tag` — accept either.
function coverageMap(s) {
  const m = {};
  (s.coverage || []).forEach((c) => { m[`${c.cam}:${c.face ?? c.tag}`] = c.count; });
  return m;
}

function renderMatrix(s) {
  const faces = s.faces || s.tags || [];
  const nCams = s.num_cams || 0;
  const minN = s.min_samples || s.burst_min || 1;
  const cov = coverageMap(s);
  const tb = $("matrix").querySelector("tbody");
  tb.innerHTML = "";

  // header row: face/tag ids
  const hr = document.createElement("tr");
  const corner = s.method === "floor" ? "cam＼tag" : "cam＼face";
  hr.appendChild(Object.assign(document.createElement("th"), { textContent: corner }));
  faces.forEach((f) => {
    hr.appendChild(Object.assign(document.createElement("th"), { textContent: f }));
  });
  tb.appendChild(hr);

  for (let cam = 0; cam < nCams; cam++) {
    const tr = document.createElement("tr");
    tr.appendChild(Object.assign(document.createElement("th"), { textContent: `cam${cam}` }));
    faces.forEach((f) => {
      const n = cov[`${cam}:${f}`] || 0;
      const td = document.createElement("td");
      td.textContent = n;
      td.className = "cell " + (n === 0 ? "empty" : n >= minN ? "ok" : "low");
      tr.appendChild(td);
    });
    tb.appendChild(tr);
  }
  if (nCams === 0 || faces.length === 0) {
    tb.innerHTML = `<tr><td class="muted">no cameras/faces configured</td></tr>`;
  }
}

const RESULT_HEAD = {
  controller: "<tr><th>cam</th><th>faces</th><th>samples</th>" +
              "<th>face spread (mm)</th><th>face spread (deg)</th></tr>",
  floor: "<tr><th>cam</th><th>tags</th><th>reproj (px)</th>" +
         "<th>planar?</th><th>plane thickness (mm)</th></tr>",
};

function renderResult(s) {
  const floor = s.method === "floor";
  $("result_head").innerHTML = floor ? RESULT_HEAD.floor : RESULT_HEAD.controller;
  const tb = $("result").querySelector("tbody");
  tb.innerHTML = "";
  const cams = s.cameras || [];
  if (cams.length === 0) {
    setMsg($("result_msg"),
      s.state === "failed" ? "solve failed — see raw state below" : "(no solution yet)",
      s.state === "failed");
    return;
  }
  setMsg($("result_msg"), "");
  cams.forEach((c) => {
    const tr = document.createElement("tr");
    const cells = floor
      ? [`cam${c.cam}`, c.n_tags, fmt(c.reproj_rms_px, 3),
         c.planar_degenerate ? "DEGENERATE" : "ok",
         fmt((c.plane_thickness_m || 0) * 1000, 1)]
      : [`cam${c.cam}`, c.n_faces, c.n_samples,
         fmt((c.face_spread_trans_m || 0) * 1000, 2),
         fmt(c.face_spread_rot_deg, 3)];
    cells.forEach((v) => {
      tr.appendChild(Object.assign(document.createElement("td"), { textContent: v }));
    });
    tb.appendChild(tr);
  });
}

function renderGate(s) {
  const gate = $("gate");
  const lin = s.lin_vel_mps, ang = s.ang_vel_dps;
  const linMax = s.gate?.lin_max ?? 0, angMax = s.gate?.ang_max ?? 0;
  $("lin_val").textContent = fmt(lin, 3);
  $("ang_val").textContent = fmt(ang, 2);
  $("lin_max").textContent = fmt(linMax, 3);
  $("ang_max").textContent = fmt(angMax, 1);
  $("lin_bar").style.width = `${clamp01(lin / (linMax || 1)) * 100}%`;
  $("ang_bar").style.width = `${clamp01(ang / (angMax || 1)) * 100}%`;

  $("lin_bar").classList.toggle("over", lin > linMax);
  $("ang_bar").classList.toggle("over", ang > angMax);

  // The backend computes the authoritative reason (folds tag visibility +
  // controller tracking + motion). Fall back to motion-only if absent.
  const reason = s.gate_reason
    || (s.state !== "collecting" ? "IDLE"
        : (lin > linMax || ang > angMax) ? "MOVING" : "GOOD");
  const LABEL = {
    IDLE: [s.state.toUpperCase(), "idle"],
    NO_TAG: ["NO TAG — point a camera at the marker", "moving"],
    NO_POSE: ["NO POSE — controller tracking lost", "moving"],
    MOVING: ["MOVING — hold still", "moving"],
    GOOD: ["GOOD — capturing", "good"],
  };
  const [text, cls] = LABEL[reason] || [reason, "idle"];
  gate.textContent = text;
  gate.className = "gate " + cls;
}

function renderDetections(s) {
  const tb = $("detections").querySelector("tbody");
  tb.innerHTML = "";
  const dets = s.detections || [];
  if (dets.length === 0) {
    tb.innerHTML = `<tr><td class="muted" colspan="4">no frames yet</td></tr>`;
    return;
  }
  const floor = s.method === "floor";
  dets.forEach((d) => {
    const tr = document.createElement("tr");
    const items = floor ? (d.tags || []) : (d.faces || []);
    const tags = items
      .map((f) => `<span class="tag ${f.ok ? "ok" : "bad"}">${f.id}·${fmt(f.reproj, 2)}</span>`)
      .join(" ") || '<span class="muted">none</span>';
    const stale = d.age_ms > 750;
    // Floor has no controller; show a static dash in that column.
    const ctrlCell = floor
      ? `<td class="muted">—</td>`
      : `<td class="${d.ctrl_ok ? "ok-txt" : "bad-txt"}">${d.ctrl_ok ? "OK" : "—"}</td>`;
    tr.innerHTML =
      `<td>cam${d.cam}</td>` +
      ctrlCell +
      `<td>${tags}</td>` +
      `<td class="${stale ? "muted" : ""}">${fmt(d.age_ms, 0)} ms</td>`;
    tb.appendChild(tr);
  });
}

async function refresh() {
  let s;
  try { s = await getJSON("/api/excal/state"); }
  catch (e) {
    // Down during a flow-daemon module swap (or a manual restart). flow.js
    // navigates away once the next module is up with a different mode.
    $("conn").textContent = "disconnected — waiting for restart…";
    return;
  }
  $("conn").textContent = "ok";

  const method = s.method === "floor" ? "floor" : "controller";
  applyMethodUI(method);

  const badge = $("state");
  badge.textContent = s.state;
  badge.className = "badge " + (BADGE[s.state] || "");
  $("samples").textContent = s.samples ?? 0;
  $("num_cams").textContent = s.num_cams ?? "-";
  $("min_samples").textContent = s.min_samples ?? s.burst_min ?? "-";

  // The floor path is static — no motion gate. Hide the gate card; the rest of
  // the rendering is method-aware.
  $("gate_card").hidden = method === "floor";
  if (method !== "floor") renderGate(s);
  renderDetections(s);
  renderMatrix(s);
  renderResult(s);
  $("raw").textContent = JSON.stringify(s, null, 2);

  const collecting = s.state === "collecting";
  $("btn_start").disabled = collecting;
  $("btn_stop").disabled = !collecting;
  $("btn_subject").disabled = s.state !== "solved";
}

// --- method selector --------------------------------------------------------
// Switching method = switching mode: the same /extrinsic-calib page is served by
// both the controller (案C) and floor (案D) modes. A switch POSTs /api/flow/switch
// and the flow daemon respawns into the chosen mode; flow.js then keeps the page.
const MODE_FOR_METHOD = {
  controller: "calib-extrinsic",
  floor: "calib-extrinsic-floor",
};

function applyMethodUI(method) {
  $("title").textContent = method === "floor"
    ? "Floor AprilTag Extrinsic Calibration"
    : "Controller-Marker Extrinsic Calibration";
  $("method_label").textContent = method === "floor"
    ? "Floor AprilTag (案D)" : "Controller marker (案C)";
  $("instr_controller").hidden = method === "floor";
  $("instr_floor").hidden = method !== "floor";
  $("btn_method_controller").classList.toggle("active", method !== "floor");
  $("btn_method_floor").classList.toggle("active", method === "floor");
  // Without the flow daemon, mode switching is unavailable.
  $("btn_method_controller").disabled = !flowManaged || method !== "floor";
  $("btn_method_floor").disabled = !flowManaged || method === "floor";
}

async function switchMethod(method) {
  if (!flowManaged) {
    setMsg($("ctrl_msg"), "method switching needs the flow daemon", true);
    return;
  }
  setMsg($("ctrl_msg"), `switching to ${method}…`);
  const r = await FitraFlow.requestSwitch(MODE_FOR_METHOD[method]);
  if (!r.ok) setMsg($("ctrl_msg"), `switch failed: ${r.err || "?"}`, true);
  // On success the daemon respawns; flow.js keeps us on /extrinsic-calib and
  // refresh() picks up the new method.
}

$("btn_method_controller").addEventListener("click", () => switchMethod("controller"));
$("btn_method_floor").addEventListener("click", () => switchMethod("floor"));

$("btn_start").addEventListener("click", async () => {
  const r = await postJSON("/api/excal/start");
  setMsg($("ctrl_msg"), r.ok ? "collecting" : "start failed", !r.ok);
  refresh();
});
$("btn_stop").addEventListener("click", async () => {
  const r = await postJSON("/api/excal/stop");
  setMsg($("ctrl_msg"), `stopped (${r.samples ?? 0} samples)`);
  refresh();
});
$("btn_solve").addEventListener("click", async () => {
  setMsg($("ctrl_msg"), "solving…");
  const r = await postJSON("/api/excal/solve");
  // On success the process writes the extrinsics YAML and auto-exits;
  // r.next_step carries what happens next: the flow daemon's auto-switch
  // notice, or the manual restart command when running standalone
  // (docs/design/pose-3d-flow-daemon.md). On failure r.err is
  // self-describing ("solve/write failed: …") — surface it verbatim.
  setMsg($("ctrl_msg"),
    r.ok ? `solved — ${r.next_step || "extrinsics written"}`
         : (r.err || "solve failed"),
    !r.ok);
  refresh();
});
$("btn_subject").addEventListener("click", () => {
  // The dedicated calib-extrinsic process has (or is about to) exit after a
  // successful solve. Under the flow daemon the watcher below follows the
  // swap automatically; standalone needs a manual restart.
  setMsg($("ctrl_msg"), flowManaged
    ? "waiting for the flow daemon to switch to subject calibration…"
    : "restart main with --calibrate (subject-calib mode) — see the solve message");
});

refresh();
setInterval(refresh, 200);

// Mode-flow watcher (flow.js): once the next module is up with a different
// mode, navigate there (solve success → /subject-calib under the daemon).
// The connection display stays owned by refresh() above.
FitraFlow.watch({
  page: "calib-extrinsic",
  onState: (s) => { flowManaged = !!s.managed; },
});
