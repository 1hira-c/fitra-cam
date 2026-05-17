const statusEl = document.getElementById("status");
const gridEl = document.getElementById("charucoGrid");
const sessionEl = document.getElementById("session");
const resultEl = document.getElementById("result");
const startBtn = document.getElementById("startBtn");
const pauseBtn = document.getElementById("pauseBtn");
const resetBtn = document.getElementById("resetBtn");
const solveBtn = document.getElementById("solveBtn");

let knownCameras = [];

function setStatus(text) {
  statusEl.textContent = text;
}

async function api(path, options = {}) {
  const res = await fetch(path, options);
  const body = await res.json();
  if (!res.ok) throw new Error(body.detail || JSON.stringify(body));
  return body;
}

function renderCameras(cameras) {
  const ids = cameras.map((cam) => cam.id).join(",");
  if (ids === knownCameras.join(",")) return;
  knownCameras = cameras.map((cam) => cam.id);
  gridEl.innerHTML = "";
  cameras.forEach((cam) => {
    const card = document.createElement("article");
    card.className = "camera";
    card.innerHTML = `
      <h2><span>${cam.id}</span><span>${cam.capture_width}x${cam.capture_height}</span></h2>
      <div class="streamBox">
        <img src="/stream/${encodeURIComponent(cam.id)}" alt="${cam.id}">
      </div>
    `;
    gridEl.appendChild(card);
  });
}

async function refresh() {
  const session = await api("/api/charuco/session");
  renderCameras(session.cameras);
  sessionEl.textContent = JSON.stringify(session, null, 2);
  const counts = session.cameras.map((cam) => `${cam.id}:${cam.accepted}/${cam.target_samples}`).join(" ");
  setStatus(counts || "ready");
}

startBtn.addEventListener("click", async () => {
  try {
    await api("/api/charuco/start", { method: "POST" });
    await refresh();
  } catch (err) {
    setStatus(`error: ${err.message}`);
  }
});

pauseBtn.addEventListener("click", async () => {
  try {
    await api("/api/charuco/pause", { method: "POST" });
    await refresh();
  } catch (err) {
    setStatus(`error: ${err.message}`);
  }
});

resetBtn.addEventListener("click", async () => {
  try {
    await api("/api/charuco/reset", { method: "POST" });
    resultEl.textContent = "{}";
    await refresh();
  } catch (err) {
    setStatus(`error: ${err.message}`);
  }
});

solveBtn.addEventListener("click", async () => {
  try {
    setStatus("solving");
    const result = await api("/api/charuco/solve", { method: "POST" });
    resultEl.textContent = JSON.stringify(result, null, 2);
    await refresh();
  } catch (err) {
    setStatus(`error: ${err.message}`);
  }
});

refresh().catch((err) => setStatus(`error: ${err.message}`));
setInterval(() => {
  refresh().catch((err) => setStatus(`error: ${err.message}`));
}, 1000);
