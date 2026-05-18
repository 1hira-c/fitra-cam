import * as THREE from "three";
import { OrbitControls } from "OrbitControls";

const CAM_COLORS = ["#00dc00", "#ffb400", "#48aaff", "#ff6f6f"];
const PERSON_3D_COLORS = ["#ff4cff", "#48aaff", "#ffd166", "#5cff8d"];
const KP_THR = 0.3;
const SKELETON = [
  [0, 1], [0, 2], [1, 3], [2, 4],
  [5, 7], [7, 9], [6, 8], [8, 10],
  [5, 6], [5, 11], [6, 12], [11, 12],
  [11, 13], [13, 15], [12, 14], [14, 16],
];

const main = document.querySelector("main");
const conn = document.getElementById("conn");
const conn3d = document.getElementById("conn3d");
const canvas3d = document.getElementById("canvas3d");
const status3d = document.getElementById("status3d");
const stats3d = document.getElementById("stats3d");

const state = {
  // per-camera latest snapshot (sparse — keyed by camera id)
  bundles: {},
  // per-camera render fps state
  renderTimes: {},
  renderFps: {},
  panes: {},          // { camId: { canvas, stats } }
  serverSeq: 0,
  serverLastMs: 0,
  view3d: "front",
  bundle3d: null,
  server3dSeq: 0,
  server3dLastMs: 0,
};

function isVisible3DJoint(joint) {
  if (!Array.isArray(joint) || joint.length < 3) return false;
  if (joint[4] === false) return false;
  const score = Number(joint[3] ?? 1);
  if (!Number.isFinite(score) || score < KP_THR) return false;
  return [joint[0], joint[1], joint[2]].every((v) => Number.isFinite(Number(v)));
}

function jointToVector(joint, target) {
  // Backend joints are [x, y, z]; Three.js uses Y as up, so map z -> Y.
  target.set(Number(joint[0]), Number(joint[2]), -Number(joint[1]));
  return target;
}

class ThreeDViewer {
  constructor(canvas, statusEl) {
    this.canvas = canvas;
    this.statusEl = statusEl;
    this.personViews = [];
    this.view = "front";
    this.sceneRadius = 1.2;
    this.hasData = false;
    this.bounds = new THREE.Box3();
    this.boundsCenter = new THREE.Vector3(0, 0.9, 0);
    this.boundsSize = new THREE.Vector3();

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x050505);
    this.camera = new THREE.PerspectiveCamera(45, 4 / 3, 0.01, 100);
    this.camera.up.set(0, 1, 0);

    try {
      this.renderer = new THREE.WebGLRenderer({
        canvas,
        antialias: true,
        alpha: false,
      });
    } catch (e) {
      this.renderer = null;
      this.setStatus("WebGL unavailable");
      return;
    }

    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setClearColor(0x050505, 1);
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.enableZoom = true;
    this.controls.enablePan = true;
    this.controls.target.copy(this.boundsCenter);

    this.peopleRoot = new THREE.Group();
    this.scene.add(this.peopleRoot);

    const grid = new THREE.GridHelper(4, 20, 0x335577, 0x2b2f36);
    grid.material.opacity = 0.75;
    grid.material.transparent = true;
    this.scene.add(grid);
    this.scene.add(new THREE.AxesHelper(0.75));

    window.addEventListener("resize", () => this.resize());
    this.setView("front");
    this.setStatus("(no 3D data)");
    this.resize();
    this.render();
  }

  setStatus(text) {
    if (!this.statusEl) return;
    this.statusEl.textContent = text;
    this.statusEl.hidden = !text;
  }

  ensurePersonView(index) {
    if (this.personViews[index]) return this.personViews[index];

    const color = PERSON_3D_COLORS[index % PERSON_3D_COLORS.length];
    const group = new THREE.Group();
    const jointGeometry = new THREE.SphereGeometry(0.025, 12, 8);
    const jointMaterial = new THREE.MeshBasicMaterial({ color });
    const boneMaterial = new THREE.LineBasicMaterial({ color });

    const joints = Array.from({ length: 17 }, () => {
      const mesh = new THREE.Mesh(jointGeometry, jointMaterial);
      mesh.visible = false;
      mesh.frustumCulled = false;
      group.add(mesh);
      return mesh;
    });

    const bones = SKELETON.map(() => {
      const positions = new Float32Array(6);
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
      const line = new THREE.Line(geometry, boneMaterial);
      line.visible = false;
      line.frustumCulled = false;
      group.add(line);
      return { line, positions };
    });

    this.peopleRoot.add(group);
    const view = { group, joints, bones };
    this.personViews[index] = view;
    return view;
  }

  update(bundle) {
    if (!this.renderer) return;

    if (!bundle || bundle.enabled === false) {
      this.setStatus(bundle && bundle.enabled === false ? "3D disabled" : "(no 3D data)");
      this.updatePeople([]);
      return;
    }

    const persons = Array.isArray(bundle.persons_3d) ? bundle.persons_3d : [];
    if (!persons.length) {
      this.setStatus("(no 3D person)");
      this.updatePeople([]);
      return;
    }

    this.setStatus("");
    this.updatePeople(persons);
  }

  updatePeople(persons) {
    const visiblePoints = [];
    const scratch = new THREE.Vector3();

    persons.forEach((person, personIndex) => {
      const view = this.ensurePersonView(personIndex);
      view.group.visible = true;
      const joints = Array.isArray(person.joints) ? person.joints : [];
      const jointVectors = Array.from({ length: 17 }, () => null);

      for (let i = 0; i < 17; i += 1) {
        const joint = joints[i];
        const mesh = view.joints[i];
        if (!isVisible3DJoint(joint)) {
          mesh.visible = false;
          continue;
        }
        const pos = jointToVector(joint, scratch);
        mesh.position.copy(pos);
        mesh.visible = true;
        jointVectors[i] = pos.clone();
        visiblePoints.push(pos.clone());
      }

      SKELETON.forEach(([a, b], boneIndex) => {
        const bone = view.bones[boneIndex];
        const pa = jointVectors[a];
        const pb = jointVectors[b];
        if (!pa || !pb) {
          bone.line.visible = false;
          return;
        }
        bone.positions[0] = pa.x;
        bone.positions[1] = pa.y;
        bone.positions[2] = pa.z;
        bone.positions[3] = pb.x;
        bone.positions[4] = pb.y;
        bone.positions[5] = pb.z;
        bone.line.geometry.attributes.position.needsUpdate = true;
        bone.line.visible = true;
      });
    });

    for (let i = persons.length; i < this.personViews.length; i += 1) {
      this.personViews[i].group.visible = false;
    }

    this.updateBounds(visiblePoints);
  }

  updateBounds(points) {
    if (!points.length) {
      this.hasData = false;
      return;
    }

    this.bounds.setFromPoints(points);
    this.bounds.getCenter(this.boundsCenter);
    this.bounds.getSize(this.boundsSize);
    this.sceneRadius = Math.max(this.boundsSize.length() * 0.5, 0.5);

    if (!this.hasData) {
      this.controls.target.copy(this.boundsCenter);
      this.setView(this.view);
      this.hasData = true;
      return;
    }

    this.controls.target.lerp(this.boundsCenter, 0.15);
  }

  setView(view) {
    if (!this.renderer) return;
    this.view = view;
    const target = this.controls.target;
    const distance = Math.max(this.sceneRadius * 3.2, 2.2);
    const lift = Math.max(this.sceneRadius * 0.15, 0.15);

    if (view === "top") {
      this.camera.up.set(0, 0, -1);
    } else {
      this.camera.up.set(0, 1, 0);
    }

    if (view === "side") {
      this.camera.position.set(target.x + distance, target.y + lift, target.z);
    } else if (view === "top") {
      this.camera.position.set(target.x, target.y + distance, target.z + 0.001);
    } else {
      this.camera.position.set(target.x, target.y + lift, target.z + distance);
    }

    this.camera.near = Math.max(distance / 1000, 0.001);
    this.camera.far = Math.max(distance * 20, 20);
    this.camera.updateProjectionMatrix();
    this.camera.lookAt(target);
    this.controls.update();
  }

  resize() {
    if (!this.renderer) return;
    const width = Math.max(1, Math.round(this.canvas.clientWidth || this.canvas.width));
    const height = Math.max(1, Math.round(this.canvas.clientHeight || this.canvas.height));
    const current = this.renderer.getSize(new THREE.Vector2());
    if (current.x === width && current.y === height) return;

    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }

  render() {
    if (!this.renderer) return;
    this.resize();
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}

const viewer3d = canvas3d ? new ThreeDViewer(canvas3d, status3d) : null;

// Build (or reuse) the pane for a given camera id.
function ensurePane(camId) {
  if (state.panes[camId]) return state.panes[camId];

  const section = document.createElement("section");
  section.className = "pane";
  section.dataset.cam = String(camId);
  const h2 = document.createElement("h2");
  h2.textContent = `cam${camId}`;
  const canvas = document.createElement("canvas");
  canvas.dataset.cam = String(camId);
  canvas.width = 640;
  canvas.height = 480;
  const stats = document.createElement("pre");
  stats.className = "stats";
  stats.dataset.cam = String(camId);
  stats.textContent = "waiting…";
  section.appendChild(h2);
  section.appendChild(canvas);
  section.appendChild(stats);

  // Insert sorted by cam id
  let inserted = false;
  for (const sib of main.querySelectorAll("section.pane")) {
    if (Number(sib.dataset.cam) > camId) {
      main.insertBefore(section, sib);
      inserted = true;
      break;
    }
  }
  if (!inserted) main.appendChild(section);

  state.panes[camId] = { canvas, stats };
  state.renderTimes[camId] = [];
  state.renderFps[camId] = 0;
  return state.panes[camId];
}

function connect() {
  const wsProto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${wsProto}://${location.host}/ws`);
  let pingTimer = null;
  const clearPing = () => {
    if (pingTimer !== null) {
      clearInterval(pingTimer);
      pingTimer = null;
    }
  };
  ws.onopen = () => {
    conn.textContent = "2D live";
    conn.className = "conn live";
    clearPing();
    pingTimer = setInterval(() => {
      if (ws.readyState === WebSocket.OPEN) ws.send("ping");
    }, 5000);
  };
  ws.onclose = () => {
    clearPing();
    conn.textContent = "2D disconnected — retrying";
    conn.className = "conn dead";
    setTimeout(connect, 1500);
  };
  ws.onerror = () => {
    clearPing();
    conn.textContent = "2D error";
    conn.className = "conn dead";
  };
  ws.onmessage = (ev) => {
    let bundle;
    try {
      bundle = JSON.parse(ev.data);
    } catch (e) {
      return;
    }
    state.serverSeq = bundle.seq;
    state.serverLastMs = bundle.ts_ms;
    for (const cam of bundle.cameras || []) {
      ensurePane(cam.id);
      state.bundles[cam.id] = cam;
    }
  };
}

function connect3d() {
  const wsProto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${wsProto}://${location.host}/ws3d`);
  let pingTimer = null;
  const clearPing = () => {
    if (pingTimer !== null) {
      clearInterval(pingTimer);
      pingTimer = null;
    }
  };
  ws.onopen = () => {
    conn3d.textContent = "3D live";
    conn3d.className = "conn live";
    clearPing();
    pingTimer = setInterval(() => {
      if (ws.readyState === WebSocket.OPEN) ws.send("ping");
    }, 5000);
  };
  ws.onclose = () => {
    clearPing();
    conn3d.textContent = "3D disconnected — retrying";
    conn3d.className = "conn dead";
    setTimeout(connect3d, 1500);
  };
  ws.onerror = () => {
    clearPing();
    conn3d.textContent = "3D error";
    conn3d.className = "conn dead";
  };
  ws.onmessage = (ev) => {
    let bundle;
    try {
      bundle = JSON.parse(ev.data);
    } catch (e) {
      return;
    }
    state.bundle3d = bundle;
    state.server3dSeq = bundle.seq || 0;
    state.server3dLastMs = bundle.ts_ms || 0;
    if (bundle.enabled === false) {
      conn3d.textContent = "3D disabled";
      conn3d.className = "conn";
    } else {
      conn3d.textContent = "3D live";
      conn3d.className = "conn live";
    }
    viewer3d?.update(bundle);
  };
}

function drawCamera(camId) {
  const pane = state.panes[camId];
  if (!pane) return;
  const canvas = pane.canvas;
  const bundle = state.bundles[camId];
  const ctx = canvas.getContext("2d");
  ctx.fillStyle = "#000";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  if (!bundle) {
    ctx.fillStyle = "#666";
    ctx.font = "16px monospace";
    ctx.fillText("(no data)", 16, 32);
    return;
  }
  if (canvas.width !== bundle.w || canvas.height !== bundle.h) {
    canvas.width = bundle.w;
    canvas.height = bundle.h;
  }
  const color = CAM_COLORS[camId % CAM_COLORS.length];
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 2;
  for (const person of bundle.persons || []) {
    if (person.bbox) {
      const [x1, y1, x2, y2] = person.bbox;
      ctx.strokeStyle = "#444";
      ctx.lineWidth = 1;
      ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
    }
    const kpts = person.kpts || [];
    for (const [a, b] of SKELETON) {
      if (!kpts[a] || !kpts[b]) continue;
      if (kpts[a][2] < KP_THR || kpts[b][2] < KP_THR) continue;
      ctx.beginPath();
      ctx.moveTo(kpts[a][0], kpts[a][1]);
      ctx.lineTo(kpts[b][0], kpts[b][1]);
      ctx.stroke();
    }
    for (const kp of kpts) {
      if (!kp || kp[2] < KP_THR) continue;
      ctx.beginPath();
      ctx.arc(kp[0], kp[1], 3, 0, Math.PI * 2);
      ctx.fill();
    }
  }
}

function updateStats(camId) {
  const pane = state.panes[camId];
  if (!pane) return;
  const el = pane.stats;
  const bundle = state.bundles[camId];
  if (!bundle) {
    el.textContent = "waiting…";
    return;
  }
  const s = bundle.stats || {};
  const renderFps = state.renderFps[camId].toFixed(1);
  const latency = bundle.stats && bundle.stats.captured_at_ms && state.serverLastMs
    ? Math.max(0, state.serverLastMs - bundle.stats.captured_at_ms)
    : 0;
  el.textContent =
    `recv_fps        ${(s.recv_fps ?? 0).toFixed(2)}\n` +
    `render_fps      ${renderFps}\n` +
    `recent_pose_fps ${(s.recent_pose_fps ?? 0).toFixed(2)}\n` +
    `avg_pose_fps    ${(s.avg_pose_fps ?? 0).toFixed(2)}\n` +
    `stage_ms        ${(s.stage_ms ?? 0).toFixed(1)}\n` +
    `pending         ${s.pending ?? 0}\n` +
    `processed       ${s.processed ?? 0}\n` +
    `latency_ms      ${latency}\n` +
    `bundle_seq      ${state.serverSeq}`;
}

function update3DStats() {
  if (!stats3d) return;
  const bundle = state.bundle3d;
  if (!bundle) {
    stats3d.textContent = "waiting…";
    return;
  }
  if (bundle.enabled === false) {
    stats3d.textContent = "enabled         false";
    return;
  }
  const s = bundle.stats || {};
  stats3d.textContent =
    `tri_fps         ${(s.tri_fps ?? 0).toFixed(2)}\n` +
    `reproj_med_px  ${(s.reproj_err_med_px ?? 0).toFixed(2)}\n` +
    `bone_drift_pct ${(s.bone_len_drift_pct ?? 0).toFixed(2)}\n` +
    `valid_joints   ${s.valid_joints ?? 0}\n` +
    `sync_dt_ms     ${(s.sync_dt_ms ?? 0).toFixed(1)}\n` +
    `stage_ms       ${(s.stage_ms ?? 0).toFixed(2)}\n` +
    `height_m       ${(s.subject_height_m ?? 0).toFixed(2)}\n` +
    `processed      ${s.processed ?? 0}\n` +
    `sync_miss      ${s.sync_miss ?? 0}\n` +
    `ik_locked      ${s.ik_locked ? "true" : "false"}\n` +
    `bundle_seq     ${state.server3dSeq}`;
}

function renderTick() {
  const now = performance.now();
  for (const camIdStr of Object.keys(state.panes)) {
    const camId = Number(camIdStr);
    drawCamera(camId);
    const times = state.renderTimes[camId];
    times.push(now);
    while (times.length && now - times[0] > 1000) times.shift();
    state.renderFps[camId] = times.length;
    updateStats(camId);
  }
  viewer3d?.render();
  update3DStats();
  requestAnimationFrame(renderTick);
}

for (const btn of document.querySelectorAll(".view3d-tabs button")) {
  btn.addEventListener("click", () => {
    state.view3d = btn.dataset.view || "front";
    viewer3d?.setView(state.view3d);
    for (const other of document.querySelectorAll(".view3d-tabs button")) {
      other.classList.toggle("active", other === btn);
    }
  });
}

connect();
connect3d();
requestAnimationFrame(renderTick);
