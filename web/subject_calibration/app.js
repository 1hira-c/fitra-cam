// Subject profile calibration wizard frontend.
//
// Talks to the C++ Crow server's /api/calib/* routes. Polls /api/calib/state
// at ~5 Hz to drive the UI; sends POST for preflight / start / retake / cancel
// / approve.

const $ = (id) => document.getElementById(id);

async function postJSON(path, body) {
  const res = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body || {}),
  });
  return res.json();
}

function clamp01(x) { return Math.max(0, Math.min(1, x)); }
function pct(x) { return `${(clamp01(x) * 100).toFixed(0)}%`; }
function fmt(n, d = 1) {
  return typeof n === "number" && isFinite(n) ? n.toFixed(d) : "-";
}

function setMsg(el, s, isErr = false) {
  el.textContent = s || "";
  el.classList.toggle("err", isErr);
}

let lastState = null;

async function refresh() {
  let s;
  try {
    const res = await fetch("/api/calib/state");
    if (res.status === 404) {
      // The /api/calib/* routes only exist in calib-subject mode.
      $("conn").textContent =
        "unavailable — restart main with --calibrate (calib-subject mode)";
      return;
    }
    s = await res.json();
  } catch (e) {
    $("conn").textContent = "disconnected";
    return;
  }
  $("conn").textContent = "ok";
  lastState = s;

  $("state").textContent = s.state;
  $("target_pose").textContent = s.target_pose || "-";
  $("drift").textContent = fmt(s.bone_drift_pct, 2);
  $("in_band").textContent = s.in_band ? "yes" : "no";
  $("failing").textContent = s.failing_axis || "-";

  // During recording, the live pose estimator is paused, so hold_progress
  // would otherwise look stuck/zero. Repurpose the bar as a recording
  // progress indicator (X / N frames -> approx seconds) instead.
  if (s.state === "recording") {
    const pidx2 = s.target_pose_idx ?? 0;
    const tgt2 = (s.poses || [])[pidx2];
    const cap2 = s.recording_frames_per_cam || 75;
    const buf0 = tgt2?.buffered?.[0] ?? 0;
    const buf1 = tgt2?.buffered?.[1] ?? 0;
    const minBuf = Math.min(buf0, buf1);
    const recProg = clamp01(minBuf / cap2);
    $("hold_bar").style.width = pct(recProg);
    $("hold_pct").textContent = `REC ${minBuf}/${cap2}`;
  } else {
    const hold = clamp01(s.hold_progress || 0);
    $("hold_bar").style.width = pct(hold);
    const holdSec = typeof s.hold_elapsed_sec === "number"
      ? ` (${fmt(s.hold_elapsed_sec, 1)}/${fmt(s.required_hold_sec, 1)}s)`
      : "";
    $("hold_pct").textContent = `${(hold * 100).toFixed(0)}%${holdSec}`;
  }

  const pidx = s.target_pose_idx ?? 0;
  const tgt = (s.poses || [])[pidx];
  if (tgt) {
    const cap = s.recording_frames_per_cam || 75;
    const c0 = tgt.buffered?.[0] ?? 0;
    const c1 = tgt.buffered?.[1] ?? 0;
    $("rec_bar0").style.width = pct(c0 / cap);
    $("rec_bar1").style.width = pct(c1 / cap);
    $("rec_count0").textContent = c0;
    $("rec_count1").textContent = c1;
  } else {
    $("rec_bar0").style.width = "0%";
    $("rec_bar1").style.width = "0%";
  }

  if (s.angles_valid && s.angles) {
    $("a_le").textContent = fmt(s.angles.l_elbow);
    $("a_re").textContent = fmt(s.angles.r_elbow);
    $("a_ls").textContent = fmt(s.angles.l_sh_abd);
    $("a_rs").textContent = fmt(s.angles.r_sh_abd);
    $("a_lk").textContent = fmt(s.angles.l_knee);
    $("a_rk").textContent = fmt(s.angles.r_knee);
    $("a_tt").textContent = fmt(s.angles.torso_tilt);
  } else {
    for (const id of ["a_le","a_re","a_ls","a_rs","a_lk","a_rk","a_tt"]) {
      $(id).textContent = "-";
    }
  }

  // Pose tiles
  const list = $("poses_list");
  list.innerHTML = "";
  (s.poses || []).forEach((p, i) => {
    const div = document.createElement("div");
    div.className = "pose-item" + (p.recorded ? " recorded" : "")
                                 + (i === pidx ? " current" : "");
    const lines = [
      `<div class="name">${p.name}</div>`,
      `<div>buf ${p.buffered?.[0]||0} / ${p.buffered?.[1]||0}</div>`,
    ];
    if (p.recorded) {
      lines.push(`<div>fps ${fmt(p.fps?.[0],1)} / ${fmt(p.fps?.[1],1)}</div>`);
      lines.push(`<button data-pose="${p.name}" class="retake">Retake</button>`);
    }
    div.innerHTML = lines.join("");
    list.appendChild(div);
  });
  document.querySelectorAll(".retake").forEach(b => {
    b.addEventListener("click", () => doRetake(b.dataset.pose));
  });

  // Review
  const q = s.quality_status || "";
  const badge = $("quality");
  badge.textContent = q || "-";
  badge.className = "badge " + (q === "pass" ? "pass" : q === "warn" ? "warn" : q === "fail" ? "fail" : "");
  $("analyze_exit").textContent = s.analyze_exit ?? "-";
  if (q) {
    $("quality_summary").textContent =
      `quality_status=${q}\nanalyze_log_tail (last 8KB):\n${s.analyze_log_tail || ""}`;
  } else if (s.last_error) {
    $("quality_summary").textContent =
      `state=${s.state}\nerror=${s.last_error}\nanalyze_log_tail (last 8KB):\n${s.analyze_log_tail || ""}`;
  } else if (s.state === "finalizing" || s.state === "analyzing") {
    $("quality_summary").textContent =
      `state=${s.state}\nanalyze_log_tail (last 8KB):\n${s.analyze_log_tail || ""}`;
  } else {
    $("quality_summary").textContent = "(no analysis yet)";
  }
  $("btn_approve").disabled = !(s.state === "review" && (q === "pass" || q === "warn"));
  $("log_tail").textContent = s.analyze_log_tail || "";
  $("session_dir").textContent = s.session_dir || "-";
  $("latest_profile").textContent = s.latest_profile || "-";

  // Button enable state
  const inactive = ["idle","ready","approved","canceled","failed"].includes(s.state);
  $("btn_preflight").disabled = !inactive || s.state === "approving";
  $("btn_start").disabled = !(s.state === "ready");
  $("btn_cancel").disabled = inactive;
}

async function doPreflight() {
  setMsg($("preflight_msg"), "");
  const sid = $("subject_id").value.trim();
  const h = parseFloat($("subject_height_cm").value) / 100.0;
  const hold = parseFloat($("hold_sec").value);
  const frames = parseInt($("frames").value, 10);
  if (!sid) { setMsg($("preflight_msg"), "subject_id is required", true); return; }
  if (!(h > 0)) { setMsg($("preflight_msg"), "height invalid", true); return; }
  const res = await postJSON("/api/calib/preflight", {
    subject_id: sid,
    subject_height_m: h,
    required_hold_sec: hold,
    recording_frames_per_cam: frames,
  });
  setMsg($("preflight_msg"), res.ok ? "ready" : (res.err || "failed"), !res.ok);
  refresh();
}

async function doStart() {
  const res = await postJSON("/api/calib/start", {});
  if (!res.ok) setMsg($("preflight_msg"), res.err || "start failed", true);
  refresh();
}

async function doCancel() {
  await postJSON("/api/calib/cancel", {});
  refresh();
}

async function doRetake(pose) {
  await postJSON("/api/calib/retake", { pose });
  refresh();
}

async function doApprove() {
  setMsg($("review_msg"), "");
  const force = $("force_chk").checked;
  const res = await postJSON("/api/calib/approve", { force });
  setMsg($("review_msg"), res.ok ? "approved & applied to live IK" : (res.err || "approve failed"), !res.ok);
  refresh();
}

$("btn_preflight").addEventListener("click", doPreflight);
$("btn_start").addEventListener("click", doStart);
$("btn_cancel").addEventListener("click", doCancel);
$("btn_approve").addEventListener("click", doApprove);

refresh();
setInterval(refresh, 200);
