// Intrinsic (ChArUco) calibration frontend. Polls /api/incal/state; POSTs
// start / stop / solve. Mirrors the extrinsic-calib page's structure.

const $ = (id) => document.getElementById(id);
let flowManaged = false;

async function postJSON(path) {
  // Tolerate a non-JSON / unreachable response (e.g. a flow-daemon module swap):
  // return a structured failure so callers always get {ok:false} instead of an
  // unhandled rejection that leaves the UI stuck.
  try {
    const res = await fetch(path, { method: "POST",
      headers: { "Content-Type": "application/json" }, body: "{}" });
    return await res.json();
  } catch (e) {
    return { ok: false, err: e.message || "request failed" };
  }
}
function fmt(n, d = 2) {
  return typeof n === "number" && isFinite(n) ? n.toFixed(d) : "-";
}
function setMsg(el, s, isErr = false) {
  el.textContent = s || "";
  el.classList.toggle("err", isErr);
}

const BADGE = {
  idle: "", collecting: "ok", solving: "warn", solved: "pass", failed: "fail",
};

function renderCams(s) {
  const tb = $("cams").querySelector("tbody");
  tb.innerHTML = "";
  const cams = s.cameras || [];
  if (cams.length === 0) {
    tb.innerHTML = `<tr><td class="muted" colspan="5">no cameras</td></tr>`;
    return;
  }
  cams.forEach((c) => {
    const enough = c.views >= (s.min_views || 0);
    const tr = document.createElement("tr");
    tr.innerHTML =
      `<td>cam${c.cam}</td>` +
      `<td class="${enough ? "ok-txt" : ""}">${c.views}</td>` +
      `<td>${fmt((c.coverage || 0) * 100, 0)}%</td>` +
      `<td>${c.solved ? fmt(c.rms_px, 3) : "-"}</td>` +
      `<td class="${c.solved ? "ok-txt" : "muted"}">${c.solved ? "yes" : "—"}</td>`;
    tb.appendChild(tr);
  });
}

async function refresh() {
  let s;
  try { s = await fetch("/api/incal/state").then((r) => r.json()); }
  catch (e) {
    $("conn").textContent = "disconnected — waiting for restart…";
    return;
  }
  $("conn").textContent = "ok";
  $("model").textContent = s.model || "-";
  const badge = $("state");
  badge.textContent = s.state;
  badge.className = "badge " + (BADGE[s.state] || "");
  $("num_cams").textContent = s.num_cams ?? "-";
  $("min_views").textContent = s.min_views ?? "-";
  renderCams(s);
  $("raw").textContent = JSON.stringify(s, null, 2);

  const collecting = s.state === "collecting";
  $("btn_start").disabled = collecting;
  $("btn_stop").disabled = !collecting;
}

$("btn_start").addEventListener("click", async () => {
  const r = await postJSON("/api/incal/start");
  setMsg($("ctrl_msg"), r.ok ? "collecting" : "start failed", !r.ok);
  refresh();
});
$("btn_stop").addEventListener("click", async () => {
  await postJSON("/api/incal/stop");
  setMsg($("ctrl_msg"), "stopped");
  refresh();
});
$("btn_solve").addEventListener("click", async () => {
  setMsg($("ctrl_msg"), "solving…");
  const r = await postJSON("/api/incal/solve");
  setMsg($("ctrl_msg"),
    r.ok ? `solved — ${r.next_step || "intrinsics written"}`
         : (r.err || "solve failed"),
    !r.ok);
  refresh();
});

refresh();
setInterval(refresh, 300);

FitraFlow.watch({
  page: "calib-intrinsic",
  onState: (s) => { flowManaged = !!s.managed; },
});
