const statusEl = document.getElementById("status");
const poseList = document.getElementById("poseList");
const qualityEl = document.getElementById("quality");
const pathsEl = document.getElementById("paths");
const analyzeBtn = document.getElementById("analyzeBtn");
const approveBtn = document.getElementById("approveBtn");

let state = null;

async function api(path, opts = {}) {
  const res = await fetch(path, opts);
  if (!res.ok) {
    let message = `${res.status} ${res.statusText}`;
    try {
      const body = await res.json();
      message = body.detail || message;
    } catch (_) {
      // keep HTTP message
    }
    throw new Error(message);
  }
  return res.json();
}

function poseStatus(name) {
  const pose = state?.poses?.[name];
  if (!pose) return "not recorded";
  const frames = Array.isArray(pose.frames) ? pose.frames.join("/") : "-";
  const fps = Array.isArray(pose.fps) ? pose.fps.map((v) => Number(v).toFixed(1)).join("/") : "-";
  return `${pose.status || "recorded"}  frames ${frames}  fps ${fps}`;
}

function render() {
  if (!state) return;
  statusEl.textContent = state.busy
    ? `${state.message} (${state.current})`
    : state.message;
  analyzeBtn.disabled = state.busy;
  approveBtn.disabled = state.busy || !state.last_analyze;

  poseList.innerHTML = "";
  for (const pose of state.sequence || []) {
    const row = document.createElement("div");
    row.className = "poseRow";
    const info = document.createElement("div");
    const title = document.createElement("div");
    title.className = "poseName";
    title.textContent = pose.name;
    const meta = document.createElement("div");
    meta.className = "poseMeta";
    meta.textContent = `${pose.duration_s}s  ${poseStatus(pose.name)}`;
    info.appendChild(title);
    info.appendChild(meta);
    const btn = document.createElement("button");
    btn.type = "button";
    btn.textContent = state.poses?.[pose.name] ? "Retake" : "Record";
    btn.disabled = state.busy;
    btn.addEventListener("click", () => recordPose(pose.name));
    row.appendChild(info);
    row.appendChild(btn);
    poseList.appendChild(row);
  }

  const q = state.last_analyze?.quality_data || null;
  qualityEl.textContent = q ? JSON.stringify(q, null, 2) : "waiting";
  pathsEl.textContent = [
    `subject_id: ${state.subject_id}`,
    `session: ${state.session_dir}`,
    `pose_session: ${state.pose_session}`,
    `latest_profile: ${state.latest_profile}`,
  ].join("\n");
}

async function refresh() {
  try {
    state = await api("/api/state");
    render();
  } catch (err) {
    statusEl.textContent = err.message;
  }
}

async function recordPose(name) {
  statusEl.textContent = `recording ${name}`;
  try {
    await api(`/api/record/${encodeURIComponent(name)}`, { method: "POST" });
  } catch (err) {
    alert(err.message);
  } finally {
    await refresh();
  }
}

analyzeBtn.addEventListener("click", async () => {
  statusEl.textContent = "analyzing";
  try {
    await api("/api/analyze", { method: "POST" });
  } catch (err) {
    alert(err.message);
  } finally {
    await refresh();
  }
});

approveBtn.addEventListener("click", async () => {
  try {
    await api("/api/approve", { method: "POST" });
  } catch (err) {
    alert(err.message);
  } finally {
    await refresh();
  }
});

refresh();
setInterval(refresh, 1000);
