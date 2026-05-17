const state = {
  cameras: [],
  points: [],
  pointById: {},
  floorGrid: null,
  annotations: { cameras: {}, grid_observations: {} },
  activePoint: null,
  mode: "grid",
  zoom: {},
  dragging: null,
};

const statusEl = document.getElementById("status");
const pointList = document.getElementById("pointList");
const cameraGrid = document.getElementById("cameraGrid");
const qualityEl = document.getElementById("quality");
const saveBtn = document.getElementById("saveBtn");
const solveBtn = document.getElementById("solveBtn");
const pointModeBtn = document.getElementById("pointModeBtn");
const gridModeBtn = document.getElementById("gridModeBtn");
const fitGridBtn = document.getElementById("fitGridBtn");

function setStatus(text) {
  statusEl.textContent = text;
}

function annotationFor(camId) {
  if (!state.annotations.cameras) state.annotations.cameras = {};
  if (!state.annotations.cameras[camId]) state.annotations.cameras[camId] = {};
  return state.annotations.cameras[camId];
}

function gridRows() {
  return state.floorGrid?.point_ids || [];
}

function gridIds() {
  return gridRows().flat();
}

function isGridPoint(pointId) {
  return gridIds().includes(pointId);
}

function orderedPointIds() {
  if (state.mode === "grid" && gridIds().length > 0) return gridIds();
  return state.points.map((point) => point.id);
}

function selectPointByOffset(offset) {
  const ids = orderedPointIds();
  if (!ids.length) return;
  const current = ids.indexOf(state.activePoint);
  const base = current >= 0 ? current : 0;
  const next = (base + offset + ids.length) % ids.length;
  state.activePoint = ids[next];
  renderPoints();
  setStatus(`selected ${state.activePoint}`);
}

function selectPointByIndex(index) {
  const ids = orderedPointIds();
  if (!ids.length) return;
  const clamped = Math.max(0, Math.min(index, ids.length - 1));
  state.activePoint = ids[clamped];
  renderPoints();
  setStatus(`selected ${state.activePoint}`);
}

function setMode(mode) {
  state.mode = mode;
  pointModeBtn.classList.toggle("active", mode === "point");
  gridModeBtn.classList.toggle("active", mode === "grid");
  document.body.dataset.mode = mode;
  if (!orderedPointIds().includes(state.activePoint)) {
    state.activePoint = orderedPointIds()[0] || null;
    renderPoints();
  }
}

function scrollActivePointIntoList() {
  const active = pointList.querySelector(".point.active");
  if (!active) return;
  const listRect = pointList.getBoundingClientRect();
  const activeRect = active.getBoundingClientRect();
  if (activeRect.top < listRect.top) {
    pointList.scrollTop -= listRect.top - activeRect.top;
  } else if (activeRect.bottom > listRect.bottom) {
    pointList.scrollTop += activeRect.bottom - listRect.bottom;
  }
}

function renderPoints() {
  const previousScrollTop = pointList.scrollTop;
  pointList.innerHTML = "";
  if (state.activePoint === null && state.points.length > 0) {
    state.activePoint = state.points[0].id;
  }
  state.points.forEach((point) => {
    const button = document.createElement("button");
    button.type = "button";
    button.className = `point ${state.activePoint === point.id ? "active" : ""}`;
    const tag = isGridPoint(point.id) ? "grid" : "point";
    button.innerHTML = `${point.id}<span>${tag} ${point.x.toFixed(3)}, ${point.y.toFixed(3)}, ${point.z.toFixed(3)} m</span>`;
    button.addEventListener("click", () => {
      state.activePoint = point.id;
      renderPoints();
    });
    pointList.appendChild(button);
  });
  pointList.scrollTop = previousScrollTop;
  scrollActivePointIntoList();
}

function svgEl(name) {
  return document.createElementNS("http://www.w3.org/2000/svg", name);
}

function addLine(svg, a, b, className) {
  const line = svgEl("line");
  line.setAttribute("x1", a[0]);
  line.setAttribute("y1", a[1]);
  line.setAttribute("x2", b[0]);
  line.setAttribute("y2", b[1]);
  line.setAttribute("class", className);
  svg.appendChild(line);
}

function addCircle(svg, camId, pointId, xy, className) {
  const group = svgEl("g");
  group.setAttribute("class", `pointNode ${className}`);
  group.dataset.pointId = pointId;
  const circle = svgEl("circle");
  circle.setAttribute("cx", xy[0]);
  circle.setAttribute("cy", xy[1]);
  circle.setAttribute("r", isGridPoint(pointId) ? 5 : 6);
  const label = svgEl("text");
  label.setAttribute("x", xy[0] + 7);
  label.setAttribute("y", xy[1] - 7);
  label.textContent = pointId;
  group.appendChild(circle);
  group.appendChild(label);
  group.addEventListener("mousedown", (event) => {
    event.preventDefault();
    event.stopPropagation();
    if (event.shiftKey) {
      annotationFor(camId)[pointId] = null;
      renderCameras();
      return;
    }
    state.activePoint = pointId;
    state.dragging = { camId, pointId };
    renderPoints();
  });
  group.addEventListener("contextmenu", (event) => {
    event.preventDefault();
    annotationFor(camId)[pointId] = null;
    renderCameras();
  });
  svg.appendChild(group);
}

function eventImagePoint(event, wrap, camId) {
  const rect = wrap.getBoundingClientRect();
  const zoom = state.zoom[camId] || 1;
  return [
    (event.clientX - rect.left) / zoom,
    (event.clientY - rect.top) / zoom,
  ];
}

function renderOverlay(cam, wrap) {
  let svg = wrap.querySelector("svg.overlay");
  if (!svg) {
    svg = svgEl("svg");
    svg.classList.add("overlay");
    wrap.appendChild(svg);
  }
  svg.setAttribute("width", cam.width);
  svg.setAttribute("height", cam.height);
  svg.setAttribute("viewBox", `0 0 ${cam.width} ${cam.height}`);
  svg.innerHTML = "";
  const ann = annotationFor(cam.id);

  const rows = gridRows();
  if (rows.length) {
    for (const row of rows) {
      for (let i = 0; i + 1 < row.length; i += 1) {
        const a = ann[row[i]];
        const b = ann[row[i + 1]];
        if (Array.isArray(a) && Array.isArray(b)) addLine(svg, a, b, "gridLine observed");
      }
    }
    const cols = rows[0]?.length || 0;
    for (let col = 0; col < cols; col += 1) {
      for (let row = 0; row + 1 < rows.length; row += 1) {
        const a = ann[rows[row][col]];
        const b = ann[rows[row + 1][col]];
        if (Array.isArray(a) && Array.isArray(b)) addLine(svg, a, b, "gridLine observed");
      }
    }
  }

  Object.entries(ann).forEach(([pointId, xy]) => {
    if (!Array.isArray(xy) || xy.length < 2) return;
    const cls = isGridPoint(pointId) ? "gridPoint" : "manualPoint";
    addCircle(svg, cam.id, pointId, xy, cls);
  });
}

function renderCameras() {
  const gridScroll = {
    left: cameraGrid.scrollLeft,
    top: cameraGrid.scrollTop,
  };
  const viewportScroll = {};
  cameraGrid.querySelectorAll("[data-camera-card]").forEach((card) => {
    const camId = card.dataset.cameraCard;
    const viewport = card.querySelector(".viewport");
    if (camId && viewport) {
      viewportScroll[camId] = {
        left: viewport.scrollLeft,
        top: viewport.scrollTop,
      };
    }
  });

  cameraGrid.innerHTML = "";
  state.cameras.forEach((cam) => {
    if (!state.zoom[cam.id]) state.zoom[cam.id] = 1;
    const section = document.createElement("article");
    section.className = "camera";
    section.dataset.cameraCard = cam.id;
    section.innerHTML = `
      <h2><span>${cam.id}</span><span>${cam.width}x${cam.height}</span></h2>
      <div class="cameraControls">
        <label>Zoom <input type="range" min="0.25" max="2.5" step="0.05" value="${state.zoom[cam.id]}"></label>
        <button type="button" data-clear="${cam.id}">Clear</button>
        <button type="button" data-disable="${cam.id}">Disable selected</button>
      </div>
      <div class="viewport">
        <div class="imageWrap" data-wrap="${cam.id}" style="width:${cam.width}px;height:${cam.height}px">
          <img src="${cam.image_url}" alt="${cam.id}" width="${cam.width}" height="${cam.height}">
        </div>
      </div>
    `;
    const slider = section.querySelector("input");
    const wrap = section.querySelector(".imageWrap");
    const clear = section.querySelector("[data-clear]");
    const disable = section.querySelector("[data-disable]");
    const applyZoom = () => {
      wrap.style.transform = `scale(${state.zoom[cam.id]})`;
    };
    slider.addEventListener("input", () => {
      state.zoom[cam.id] = Number(slider.value);
      applyZoom();
    });
    clear.addEventListener("click", () => {
      state.annotations.cameras[cam.id] = {};
      renderOverlay(cam, wrap);
    });
    disable.addEventListener("click", () => {
      if (!state.activePoint) return;
      annotationFor(cam.id)[state.activePoint] = null;
      renderOverlay(cam, wrap);
    });
    wrap.addEventListener("click", (event) => {
      if (!state.activePoint || state.dragging) return;
      annotationFor(cam.id)[state.activePoint] = eventImagePoint(event, wrap, cam.id);
      setStatus(`${cam.id} ${state.activePoint}`);
      renderOverlay(cam, wrap);
    });
    cameraGrid.appendChild(section);
    applyZoom();
    renderOverlay(cam, wrap);
    const saved = viewportScroll[cam.id];
    if (saved) {
      const viewport = section.querySelector(".viewport");
      viewport.scrollLeft = saved.left;
      viewport.scrollTop = saved.top;
    }
  });
  cameraGrid.scrollLeft = gridScroll.left;
  cameraGrid.scrollTop = gridScroll.top;
}

function solveLinearSystem(A, b) {
  const n = b.length;
  const M = A.map((row, i) => [...row, b[i]]);
  for (let col = 0; col < n; col += 1) {
    let pivot = col;
    for (let row = col + 1; row < n; row += 1) {
      if (Math.abs(M[row][col]) > Math.abs(M[pivot][col])) pivot = row;
    }
    if (Math.abs(M[pivot][col]) < 1e-9) throw new Error("grid fit is singular");
    [M[col], M[pivot]] = [M[pivot], M[col]];
    const div = M[col][col];
    for (let k = col; k <= n; k += 1) M[col][k] /= div;
    for (let row = 0; row < n; row += 1) {
      if (row === col) continue;
      const factor = M[row][col];
      for (let k = col; k <= n; k += 1) M[row][k] -= factor * M[col][k];
    }
  }
  return M.map((row) => row[n]);
}

function fitHomography(worldPts, imagePts) {
  const normal = Array.from({ length: 8 }, () => Array(8).fill(0));
  const rhs = Array(8).fill(0);
  const addRow = (row, value) => {
    for (let i = 0; i < 8; i += 1) {
      rhs[i] += row[i] * value;
      for (let j = 0; j < 8; j += 1) normal[i][j] += row[i] * row[j];
    }
  };
  for (let i = 0; i < worldPts.length; i += 1) {
    const [X, Y] = worldPts[i];
    const [u, v] = imagePts[i];
    addRow([X, Y, 1, 0, 0, 0, -u * X, -u * Y], u);
    addRow([0, 0, 0, X, Y, 1, -v * X, -v * Y], v);
  }
  const h = solveLinearSystem(normal, rhs);
  return [...h, 1];
}

function projectHomography(H, x, y) {
  const den = H[6] * x + H[7] * y + H[8];
  return [
    (H[0] * x + H[1] * y + H[2]) / den,
    (H[3] * x + H[4] * y + H[5]) / den,
  ];
}

function fitGridForCamera(camId) {
  const ann = annotationFor(camId);
  const disabled = new Set(gridIds().filter((pointId) => ann[pointId] === null));
  const anchors = gridIds()
    .filter((pointId) => Array.isArray(ann[pointId]) && state.pointById[pointId])
    .map((pointId) => ({ pointId, world: state.pointById[pointId], image: ann[pointId] }));
  if (anchors.length < 4) throw new Error(`${camId}: grid fit needs at least 4 observed grid points`);
  const H = fitHomography(
    anchors.map((p) => [p.world.x, p.world.y]),
    anchors.map((p) => p.image),
  );
  gridIds().forEach((pointId) => {
    if (disabled.has(pointId)) return;
    const p = state.pointById[pointId];
    if (!p) return;
    ann[pointId] = projectHomography(H, p.x, p.y);
  });
  anchors.forEach((p) => {
    ann[p.pointId] = p.image;
  });
  if (!state.annotations.grid_observations) state.annotations.grid_observations = {};
  state.annotations.grid_observations[camId] = {
    fitted_at_ms: Date.now(),
    anchors: anchors.map((p) => p.pointId),
  };
}

async function saveAnnotations() {
  const res = await fetch("/api/annotations", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(state.annotations),
  });
  if (!res.ok) throw new Error(await res.text());
  setStatus("saved");
}

async function solve() {
  await saveAnnotations();
  setStatus("solving");
  const res = await fetch("/api/solve", { method: "POST" });
  const body = await res.json();
  if (!res.ok) throw new Error(body.detail || JSON.stringify(body));
  qualityEl.textContent = JSON.stringify(body.quality, null, 2);
  setStatus(`wrote ${body.calibration}`);
}

async function loadSession() {
  const res = await fetch("/api/session");
  if (!res.ok) throw new Error(await res.text());
  const body = await res.json();
  state.cameras = body.cameras;
  state.points = body.points;
  state.floorGrid = body.floor_grid || null;
  state.pointById = Object.fromEntries(state.points.map((p) => [p.id, p]));
  state.annotations = body.annotations || { cameras: {}, grid_observations: {} };
  if (!state.annotations.grid_observations) state.annotations.grid_observations = {};
  const firstGrid = gridIds()[0];
  state.activePoint = firstGrid || state.points[0]?.id || null;
  qualityEl.textContent = JSON.stringify(body.quality || {}, null, 2);
  setMode(state.floorGrid ? "grid" : "point");
  renderPoints();
  renderCameras();
  setStatus("ready");
}

window.addEventListener("mousemove", (event) => {
  if (!state.dragging) return;
  const { camId, pointId } = state.dragging;
  const wrap = document.querySelector(`[data-wrap="${camId}"]`);
  if (!wrap) return;
  annotationFor(camId)[pointId] = eventImagePoint(event, wrap, camId);
  const cam = state.cameras.find((item) => item.id === camId);
  if (cam) renderOverlay(cam, wrap);
});

window.addEventListener("mouseup", () => {
  if (!state.dragging) return;
  state.dragging = null;
  renderCameras();
});

window.addEventListener("keydown", (event) => {
  if (state.dragging) return;
  const tag = event.target?.tagName?.toLowerCase();
  if (tag === "input" || tag === "textarea" || tag === "select" || event.target?.isContentEditable) {
    return;
  }
  const key = event.key.toLowerCase();
  if (key === "arrowdown" || key === "arrowright" || key === "n" || key === "j") {
    event.preventDefault();
    selectPointByOffset(1);
  } else if (key === "arrowup" || key === "arrowleft" || key === "p" || key === "k") {
    event.preventDefault();
    selectPointByOffset(-1);
  } else if (key === "home") {
    event.preventDefault();
    selectPointByIndex(0);
  } else if (key === "end") {
    event.preventDefault();
    selectPointByIndex(orderedPointIds().length - 1);
  } else if (key === "g") {
    event.preventDefault();
    setMode("grid");
  } else if (key === "o") {
    event.preventDefault();
    setMode("point");
  }
});

pointModeBtn.addEventListener("click", () => setMode("point"));
gridModeBtn.addEventListener("click", () => setMode("grid"));

fitGridBtn.addEventListener("click", () => {
  try {
    state.cameras.forEach((cam) => fitGridForCamera(cam.id));
    renderCameras();
    setStatus("grid fitted");
  } catch (err) {
    setStatus(`error: ${err.message}`);
  }
});

saveBtn.addEventListener("click", () => {
  saveAnnotations().catch((err) => setStatus(`error: ${err.message}`));
});

solveBtn.addEventListener("click", () => {
  solve().catch((err) => setStatus(`error: ${err.message}`));
});

loadSession().catch((err) => setStatus(`error: ${err.message}`));
