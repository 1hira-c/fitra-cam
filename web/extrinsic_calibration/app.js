// Controller-marker extrinsic calibration frontend.
//
// Talks to the C++ Crow server's /api/excal/* routes. Polls /api/excal/state
// at 5 Hz to drive the UI; POSTs start / stop / solve.

const $ = (id) => document.getElementById(id);

async function postJSON(path) {
  const res = await fetch(path, { method: "POST",
    headers: { "Content-Type": "application/json" }, body: "{}" });
  return res.json();
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

// Coverage map keyed "cam:face" -> count.
function coverageMap(s) {
  const m = {};
  (s.coverage || []).forEach((c) => { m[`${c.cam}:${c.face}`] = c.count; });
  return m;
}

function renderMatrix(s) {
  const faces = s.faces || [];
  const nCams = s.num_cams || 0;
  const minN = s.min_samples || 1;
  const cov = coverageMap(s);
  const tb = $("matrix").querySelector("tbody");
  tb.innerHTML = "";

  // header row: face ids
  const hr = document.createElement("tr");
  hr.appendChild(Object.assign(document.createElement("th"), { textContent: "cam＼face" }));
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

function renderResult(s) {
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
    const cells = [
      `cam${c.cam}`, c.n_faces, c.n_samples,
      fmt((c.face_spread_trans_m || 0) * 1000, 2),
      fmt(c.face_spread_rot_deg, 3),
    ];
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
  dets.forEach((d) => {
    const tr = document.createElement("tr");
    const faces = (d.faces || [])
      .map((f) => `<span class="tag ${f.ok ? "ok" : "bad"}">${f.id}·${fmt(f.reproj, 2)}</span>`)
      .join(" ") || '<span class="muted">none</span>';
    const stale = d.age_ms > 750;
    tr.innerHTML =
      `<td>cam${d.cam}</td>` +
      `<td class="${d.ctrl_ok ? "ok-txt" : "bad-txt"}">${d.ctrl_ok ? "OK" : "—"}</td>` +
      `<td>${faces}</td>` +
      `<td class="${stale ? "muted" : ""}">${fmt(d.age_ms, 0)} ms</td>`;
    tb.appendChild(tr);
  });
}

async function refresh() {
  let s;
  try { s = await getJSON("/api/excal/state"); }
  catch (e) { $("conn").textContent = "disconnected"; return; }
  $("conn").textContent = "ok";

  const badge = $("state");
  badge.textContent = s.state;
  badge.className = "badge " + (BADGE[s.state] || "");
  $("samples").textContent = s.samples ?? 0;
  $("num_cams").textContent = s.num_cams ?? "-";
  $("min_samples").textContent = s.min_samples ?? "-";

  renderGate(s);
  renderDetections(s);
  renderMatrix(s);
  renderResult(s);
  $("raw").textContent = JSON.stringify(s, null, 2);

  const collecting = s.state === "collecting";
  $("btn_start").disabled = collecting;
  $("btn_stop").disabled = !collecting;
}

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
  setMsg($("ctrl_msg"),
    r.ok ? "solved — extrinsics written" : `solve failed: ${r.err || ""}`, !r.ok);
  refresh();
});

refresh();
setInterval(refresh, 200);
