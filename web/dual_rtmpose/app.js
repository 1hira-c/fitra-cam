import * as THREE from "three";
import { OrbitControls } from "OrbitControls";

const CAM_COLORS = ["#00dc00", "#ffb400", "#48aaff", "#ff6f6f"];
const PERSON_3D_COLORS = ["#ff4cff", "#48aaff", "#ffd166", "#5cff8d"];
const KP_THR = 0.3;
// COCO17 edge table — the original Phase 6 viewer expected exactly these
// keypoints.
const SKELETON_COCO17 = [
  [0, 1], [0, 2], [1, 3], [2, 4],
  [5, 7], [7, 9], [6, 8], [8, 10],
  [5, 6], [5, 11], [6, 12], [11, 12],
  [11, 13], [13, 15], [12, 14], [14, 16],
];
// Halpe26 adds neck (18), hip-center (19) and per-side toes/heels. Indices
// 0–16 still match COCO17, so the upper-body edges are shared. Lower body and
// feet are extended.
const SKELETON_HALPE26 = [
  // Head
  [17, 18], [0, 17], [0, 1], [0, 2], [1, 3], [2, 4],
  // Torso
  [18, 5], [18, 6], [18, 19], [11, 19], [12, 19], [5, 6], [11, 12],
  // Arms
  [5, 7], [7, 9], [6, 8], [8, 10],
  // Legs
  [11, 13], [13, 15], [12, 14], [14, 16],
  // Feet (representative line per side)
  [15, 24], [16, 25],
];
const KP_COUNT_BY_FORMAT = { coco17: 17, halpe26: 26 };

// Phase 13 M1: SlimeVR tracker visualization.
// 10 trackers in role order (sensor_id 0..9) — matches the wire ordering in
// the C++ TrackerRole enum (cpp/src/slimevr/tracker_extract.hpp).
const TRACKER_ROLES = [
  "LeftUpperArm", "RightUpperArm",
  "Chest", "Waist",
  "LeftUpperLeg", "RightUpperLeg",
  "LeftLowerLeg", "RightLowerLeg",
  "LeftFoot", "RightFoot",
];
const TRACKER_COUNT = TRACKER_ROLES.length;
// Default axis length (meters) in WORLD frame, scales to (0.3 + 0.7*conf) × this.
const TRACKER_AXIS_BASE_LEN = 0.15;
// World (Z-up, X-right, Y-forward) → Three.js (Y-up, X-right, Z-back)
// basis change quaternion = Rx(-90°). In xyzw: (sin(-45°), 0, 0, cos(-45°)).
// We pre-build this constant once for the per-frame quaternion sandwich.
const WORLD_TO_THREE_QUAT = (() => {
  const k = 0.7071067811865475;  // 1/√2
  return new THREE.Quaternion(-k, 0, 0, k);
})();
const WORLD_TO_THREE_QUAT_INV = WORLD_TO_THREE_QUAT.clone().invert();

function skeletonFor(format) {
  return format === "halpe26" ? SKELETON_HALPE26 : SKELETON_COCO17;
}
function kpCountFor(format) {
  return KP_COUNT_BY_FORMAT[format] ?? 17;
}

const main = document.querySelector("main");
const conn = document.getElementById("conn");
const conn3d = document.getElementById("conn3d");
const canvas3d = document.getElementById("canvas3d");
const status3d = document.getElementById("status3d");
const stats3d = document.getElementById("stats3d");
const vmtAlignForm = document.getElementById("vmt-align-form");
const vmtAlignStatus = document.getElementById("vmt-align-status");
const vmtAlignReset = document.getElementById("vmt-align-reset");
const vmtAlignInputs = {
  x: document.getElementById("vmt-align-x"),
  y: document.getElementById("vmt-align-y"),
  z: document.getElementById("vmt-align-z"),
  yaw_deg: document.getElementById("vmt-align-yaw"),
};
const vmtAlignSliders = {
  x: document.getElementById("vmt-align-x-slider"),
  y: document.getElementById("vmt-align-y-slider"),
  z: document.getElementById("vmt-align-z-slider"),
  yaw_deg: document.getElementById("vmt-align-yaw-slider"),
};
const vmtAlignTotals = {
  x: document.getElementById("vmt-align-x-total"),
  y: document.getElementById("vmt-align-y-total"),
  z: document.getElementById("vmt-align-z-total"),
  yaw_deg: document.getElementById("vmt-align-yaw-total"),
};
const VMT_ALIGN_KEYS = ["x", "y", "z", "yaw_deg"];
const VMT_ALIGN_BASE_STEP = { x: 1, y: 1, z: 1, yaw_deg: 45 };

// Phase 15: auto alignment elements.
const vmtAutoForm        = document.getElementById("vmt-auto-form");
const vmtHmdStatus       = document.getElementById("vmt-hmd-status");
const vmtAutoTposeBtn    = document.getElementById("vmt-auto-tpose");
const vmtAutoStartBtn    = document.getElementById("vmt-auto-motion-start");
const vmtAutoStopBtn     = document.getElementById("vmt-auto-motion-stop");
const vmtAutoResult      = document.getElementById("vmt-auto-result");
const VMT_AUTO_DURATION_S = 3.0;
const VMT_AUTO_SAMPLE_HZ  = 30.0;

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
  // Active keypoint topology from the most recent /ws bundle ("coco17" or
  // "halpe26"). Backend started emitting `kp_format` in Phase 9; older
  // servers omit it, so we default to coco17 for compatibility.
  kpFormat2D: "coco17",
  kpFormat3D: "coco17",
  vmtAlignmentEnabled: false,
  vmtPostTimer: null,
};

function formatInputNumber(v) {
  const n = Number.isFinite(Number(v)) ? Number(v) : 0;
  const s = n.toFixed(3).replace(/\.?0+$/, "");
  return s === "-0" ? "0" : s;
}

function setVmtAlignmentStatus(text, className = "") {
  if (!vmtAlignStatus) return;
  vmtAlignStatus.textContent = text;
  vmtAlignStatus.className = `vmt-align-status ${className}`.trim();
}

function setVmtAlignmentEnabled(enabled) {
  state.vmtAlignmentEnabled = !!enabled;
  for (const input of Object.values(vmtAlignInputs)) {
    if (input) input.disabled = !enabled;
  }
  for (const slider of Object.values(vmtAlignSliders)) {
    if (slider) slider.disabled = !enabled;
  }
  if (vmtAlignForm) {
    for (const button of vmtAlignForm.querySelectorAll("button")) {
      button.disabled = !enabled;
    }
  }
}

function splitVmtAlignmentValue(key, total) {
  const step = VMT_ALIGN_BASE_STEP[key] ?? 1;
  const value = Number.isFinite(Number(total)) ? Number(total) : 0;
  let base = Math.trunc(value / step) * step;
  let fine = value - base;
  const min = Number(vmtAlignSliders[key]?.min ?? -step * 0.5);
  const max = Number(vmtAlignSliders[key]?.max ??  step * 0.5);
  if (fine > max) {
    base += step;
    fine -= step;
  } else if (fine < min) {
    base -= step;
    fine += step;
  }
  return { base, fine };
}

function totalVmtAlignmentValue(key) {
  const base = Number(vmtAlignInputs[key]?.value ?? 0);
  const fine = Number(vmtAlignSliders[key]?.value ?? 0);
  if (!Number.isFinite(base) || !Number.isFinite(fine)) {
    throw new Error(`invalid ${key}`);
  }
  return base + fine;
}

function updateVmtAlignmentTotals() {
  for (const key of VMT_ALIGN_KEYS) {
    const output = vmtAlignTotals[key];
    if (!output) continue;
    let total = 0;
    try {
      total = totalVmtAlignmentValue(key);
    } catch (e) {
      output.textContent = "-";
      continue;
    }
    output.textContent = formatInputNumber(total);
  }
}

function writeVmtAlignmentForm(alignment, opts = {}) {
  if (!alignment) return;
  const splitControls = opts.splitControls !== false;
  for (const key of VMT_ALIGN_KEYS) {
    const total = Number(alignment[key] ?? 0);
    const parts = splitControls ? splitVmtAlignmentValue(key, total)
                                : { base: total, fine: Number(vmtAlignSliders[key]?.value ?? 0) };
    if (vmtAlignInputs[key]) {
      vmtAlignInputs[key].value = formatInputNumber(parts.base);
    }
    if (vmtAlignSliders[key]) {
      vmtAlignSliders[key].value = formatInputNumber(parts.fine);
    }
  }
  updateVmtAlignmentTotals();
}

function readVmtAlignmentForm() {
  const alignment = {};
  for (const key of VMT_ALIGN_KEYS) {
    alignment[key] = totalVmtAlignmentValue(key);
  }
  return alignment;
}

async function loadVmtAlignment() {
  if (!vmtAlignForm) return;
  setVmtAlignmentEnabled(false);
  setVmtAlignmentStatus("loading");
  try {
    const resp = await fetch("/api/vmt/alignment", { cache: "no-store" });
    const data = await resp.json();
    writeVmtAlignmentForm(data.alignment);
    setVmtAlignmentEnabled(!!data.enabled);
    setVmtAlignmentStatus(data.enabled ? "ready" : "vmt off", data.enabled ? "live" : "");
  } catch (e) {
    setVmtAlignmentEnabled(false);
    setVmtAlignmentStatus("api error", "dead");
  }
}

async function postVmtAlignment(alignment, opts = {}) {
  const resp = await fetch("/api/vmt/alignment", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(alignment),
  });
  let data = null;
  try {
    data = await resp.json();
  } catch (e) {
    // fall through to status handling below
  }
  if (!resp.ok || !data || data.ok === false) {
    throw new Error((data && data.err) || `HTTP ${resp.status}`);
  }
  if (opts.syncForm !== false) {
    writeVmtAlignmentForm(data.alignment);
  } else {
    updateVmtAlignmentTotals();
  }
  return data.alignment;
}

function scheduleVmtAlignmentPost(delayMs = 70) {
  if (state.vmtPostTimer !== null) {
    clearTimeout(state.vmtPostTimer);
  }
  state.vmtPostTimer = setTimeout(async () => {
    state.vmtPostTimer = null;
    if (!state.vmtAlignmentEnabled) return;
    setVmtAlignmentStatus("applying");
    try {
      await postVmtAlignment(readVmtAlignmentForm(), { syncForm: false });
      setVmtAlignmentStatus("applied", "live");
    } catch (e) {
      setVmtAlignmentStatus(e.message || "apply failed", "dead");
    }
  }, delayMs);
}

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
    this.lastDataAtMs = 0;
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

    // Phase 13 M1: per-tracker AxesHelper. 10 groups (one per SlimeVR
    // tracker role); each group's quaternion expresses the tracker's
    // orientation in Three.js Y-up frame. Group's scale encodes confidence,
    // material opacity encodes valid. Toggleable via the show-trackers
    // checkbox.
    this.trackersRoot = new THREE.Group();
    this.trackersVisible = true;
    this.scene.add(this.trackersRoot);
    this.trackerViews = [];
    for (let i = 0; i < TRACKER_COUNT; i += 1) {
      const group = new THREE.Group();
      const axes = new THREE.AxesHelper(TRACKER_AXIS_BASE_LEN);
      // AxesHelper uses a LineBasicMaterial that supports transparency.
      axes.material.transparent = true;
      axes.material.opacity = 1.0;
      // The AxesHelper line draws each axis as 2 vertices, but vertex colors
      // are set on construction. We don't need to recolor per tracker — the
      // red/green/blue per-axis convention is more readable than per-tracker
      // colors here.
      group.add(axes);
      group.visible = false;          // hidden until first valid update
      group.frustumCulled = false;
      this.trackersRoot.add(group);
      // Phase 13 (post-review): cache the last position/scale we got from a
      // valid frame so brief sync misses (= bundle says tracker is invalid
      // and ships pos=(0,0,0) per the Codex P2 fix) don't snap the
      // AxesHelper to the world origin. Quaternion held quat is already
      // produced by apply_quat_smoothing on the C++ side, so we use the
      // bundle's quat as-is even during held frames.
      const lastGood = {
        position: new THREE.Vector3(),
        scale: 1.0,
        hasData: false,
      };
      this.trackerViews.push({ group, axes, lastGood });
    }

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

    // Allocate the maximum we might receive; unused slots stay hidden.
    const joints = Array.from({ length: kpCountFor("halpe26") }, () => {
      const mesh = new THREE.Mesh(jointGeometry, jointMaterial);
      mesh.visible = false;
      mesh.frustumCulled = false;
      group.add(mesh);
      return mesh;
    });

    // Bone lines are rebuilt lazily when the active topology changes; start
    // empty so the first update() pass populates them for the current format.
    const view = { group, joints, bones: [], boneMaterial, boneFormat: null };
    this.peopleRoot.add(group);
    this.personViews[index] = view;
    return view;
  }

  ensureBones(view, format) {
    if (view.boneFormat === format) return;
    for (const bone of view.bones) {
      view.group.remove(bone.line);
      bone.line.geometry.dispose();
    }
    view.bones = skeletonFor(format).map(() => {
      const positions = new Float32Array(6);
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
      const line = new THREE.Line(geometry, view.boneMaterial);
      line.visible = false;
      line.frustumCulled = false;
      view.group.add(line);
      return { line, positions };
    });
    view.boneFormat = format;
  }

  update(bundle) {
    if (!this.renderer) return;

    if (!bundle || bundle.enabled === false) {
      this.setStatus(bundle && bundle.enabled === false ? "3D disabled" : "(no 3D data)");
      this.updatePeople([]);
      this.updateTrackers([]);
      return;
    }

    const persons = Array.isArray(bundle.persons_3d) ? bundle.persons_3d : [];
    const trackers = Array.isArray(bundle.trackers) ? bundle.trackers : [];
    if (!persons.length) {
      // Hold the last skeleton on screen across short data dropouts. 3.5s
      // covers a Phase 8 calibration recording window (~3s) during which the
      // live pose estimator is intentionally paused.
      if (this.hasData && performance.now() - this.lastDataAtMs < 3500) {
        // Still refresh trackers — they may be frozen with the held pose.
        this.updateTrackers(trackers);
        return;
      }
      this.setStatus("(no 3D person)");
      this.updatePeople([]);
      this.updateTrackers(trackers);
      return;
    }

    this.lastDataAtMs = performance.now();
    this.setStatus("");
    this.updatePeople(persons);
    this.updateTrackers(trackers);
  }

  setTrackersVisible(visible) {
    this.trackersVisible = !!visible;
    if (this.trackersRoot) this.trackersRoot.visible = this.trackersVisible;
  }

  updateTrackers(trackers) {
    if (!this.trackersRoot) return;
    this.trackersRoot.visible = this.trackersVisible;

    // Bundle may carry fewer than TRACKER_COUNT entries (e.g. extractor
    // hasn't produced its first snapshot yet) — hide the rest. The wire
    // payload always emits exactly 10 in role order, so the index IS the
    // role's sensor_id.
    for (let i = 0; i < TRACKER_COUNT; i += 1) {
      const view = this.trackerViews[i];
      const t = trackers[i];
      if (!t || !Array.isArray(t.pos) || !Array.isArray(t.quat_wxyz)) {
        view.group.visible = false;
        continue;
      }

      // Quaternion: backend sends wxyz in world frame. Three.js uses xyzw and
      // Y-up frame. Apply the basis change Rx(-90°) on both sides:
      //   q_three = B · q_world · B⁻¹
      // where B is the world→three rotation expressed as a quaternion.
      // (Even on held frames the quat is the smoothed prev_quat — usable
      // as-is.)
      const qw = Number(t.quat_wxyz[0]);
      const qx = Number(t.quat_wxyz[1]);
      const qy = Number(t.quat_wxyz[2]);
      const qz = Number(t.quat_wxyz[3]);
      const qWorld = new THREE.Quaternion(qx, qy, qz, qw);
      const qThree = WORLD_TO_THREE_QUAT.clone().multiply(qWorld).multiply(WORLD_TO_THREE_QUAT_INV);
      view.group.quaternion.copy(qThree);

      // Phase 13 (post-review): position/scale use last-good caching to
      // avoid snapping to the world origin during the 1-3 frame sync
      // misses that the Codex P2 fix now reports as valid=false. The
      // cache is only updated on confirmed valid frames; held trackers
      // freeze visually in place until they recover.
      const st = t.stats || {};
      const freezeMs = Number(st.freeze_current_ms ?? 0);
      const conf = Number.isFinite(t.roll_confidence) ? Number(t.roll_confidence) : 1.0;
      const scaleFromConf = 0.3 + 0.7 * Math.max(0, Math.min(1, conf));

      if (t.valid) {
        view.group.position.set(
          Number(t.pos[0]), Number(t.pos[2]), -Number(t.pos[1])
        );
        view.group.scale.setScalar(scaleFromConf);
        view.lastGood.position.copy(view.group.position);
        view.lastGood.scale = scaleFromConf;
        view.lastGood.hasData = true;
      } else if (view.lastGood.hasData) {
        // Held: keep last-good position/scale so the axes hover where
        // they last were, with the smoothed quat continuing to track.
        view.group.position.copy(view.lastGood.position);
        view.group.scale.setScalar(view.lastGood.scale);
      } else {
        // Never had data — hide rather than draw at origin.
        view.group.visible = false;
        continue;
      }

      // Opacity uses the same 200ms sustained threshold as the state-label
      // debounce in updateTrackerTable, so the row state column and the
      // 3D axes fade together (or not at all for sync-miss transients).
      view.axes.material.opacity = (freezeMs >= 200) ? 0.3 : 1.0;
      view.group.visible = true;
    }
  }

  updatePeople(persons) {
    const visiblePoints = [];
    const scratch = new THREE.Vector3();
    const format = state.kpFormat3D;
    const kpCount = kpCountFor(format);
    const skeleton = skeletonFor(format);

    persons.forEach((person, personIndex) => {
      const view = this.ensurePersonView(personIndex);
      this.ensureBones(view, format);
      view.group.visible = true;
      const joints = Array.isArray(person.joints) ? person.joints : [];
      const jointVectors = Array.from({ length: kpCount }, () => null);

      for (let i = 0; i < view.joints.length; i += 1) {
        if (i >= kpCount) {
          view.joints[i].visible = false;
          continue;
        }
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

      skeleton.forEach(([a, b], boneIndex) => {
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
    if (typeof bundle.kp_format === "string") {
      state.kpFormat2D = bundle.kp_format;
    }
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
    if (typeof bundle.kp_format === "string") {
      state.kpFormat3D = bundle.kp_format;
    }
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
  const skeleton = skeletonFor(state.kpFormat2D);
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
    for (const [a, b] of skeleton) {
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
    updateTrackerTable(null);
    return;
  }
  if (bundle.enabled === false) {
    stats3d.textContent = "enabled         false";
    updateTrackerTable(null);
    return;
  }
  const s = bundle.stats || {};
  const vmt = bundle.vmt || null;
  const vmtAlignment = vmt && vmt.alignment ? vmt.alignment : null;
  const vmtLine = vmt
    ? `\nvmt_bundles    ${vmt.sent_bundles ?? 0}` +
      `\nvmt_disabled   ${vmt.disabled_count ?? 0}` +
      `\nvmt_align      x=${formatInputNumber(vmtAlignment?.x ?? 0)} ` +
      `y=${formatInputNumber(vmtAlignment?.y ?? 0)} ` +
      `z=${formatInputNumber(vmtAlignment?.z ?? 0)} ` +
      `yaw=${formatInputNumber(vmtAlignment?.yaw_deg ?? 0)}`
    : "\nvmt            off";
  // Phase 15: HMD pose receiver status (block exists iff bus is attached).
  const hmd = bundle.hmd || null;
  let hmdLine = "";
  let hmdStatusText = "no hmd";
  let hmdStatusClass = "";
  if (hmd && hmd.enabled) {
    if (!hmd.have_any) {
      hmdStatusText  = "waiting for hmd";
      hmdStatusClass = "";
      hmdLine = "\nhmd            waiting";
    } else if (hmd.stale) {
      hmdStatusText  = `stale (${Math.round(hmd.age_ms ?? 0)}ms)`;
      hmdStatusClass = "dead";
      hmdLine = `\nhmd            stale (${(hmd.age_ms ?? 0).toFixed(0)}ms)`;
    } else if (hmd.valid === false) {
      hmdStatusText  = "lost";
      hmdStatusClass = "dead";
      hmdLine = "\nhmd            lost";
    } else {
      hmdStatusText  = `tracking (${Math.round(hmd.age_ms ?? 0)}ms)`;
      hmdStatusClass = "live";
      const pos = hmd.pos || [0, 0, 0];
      hmdLine =
        `\nhmd_pos        [${pos[0]?.toFixed(3)}, ${pos[1]?.toFixed(3)}, ${pos[2]?.toFixed(3)}]` +
        `\nhmd_yaw_deg    ${(hmd.yaw_deg ?? 0).toFixed(2)}` +
        `\nhmd_age_ms     ${(hmd.age_ms ?? 0).toFixed(0)}`;
    }
  }
  if (vmtHmdStatus) {
    vmtHmdStatus.textContent = hmdStatusText;
    vmtHmdStatus.className   = `vmt-align-status ${hmdStatusClass}`.trim();
  }
  stats3d.textContent =
    `tri_fps         ${(s.tri_fps ?? 0).toFixed(2)}\n` +
    `reproj_med_px  ${(s.reproj_err_med_px ?? 0).toFixed(2)}\n` +
    `bone_drift_pct ${(s.bone_len_drift_pct ?? 0).toFixed(2)}\n` +
    `valid_joints   ${s.valid_joints ?? 0}\n` +
    `sync_dt_ms     ${(s.sync_dt_ms ?? 0).toFixed(1)}\n` +
    `stage_ms       ${(s.stage_ms ?? 0).toFixed(2)}\n` +
    `height_m       ${(s.subject_height_m ?? 0).toFixed(2)}\n` +
    `profile_loaded ${s.profile_loaded ? "true" : "false"}\n` +
    `subject_id     ${s.subject_id || "-"}\n` +
    `quality        ${s.quality_status || "-"}\n` +
    `processed      ${s.processed ?? 0}\n` +
    `sync_miss      ${s.sync_miss ?? 0}\n` +
    `ik_locked      ${s.ik_locked ? "true" : "false"}\n` +
    `bundle_seq     ${state.server3dSeq}` +
    vmtLine +
    hmdLine;
  updateTrackerTable(bundle);
}

const trackerTbody = document.getElementById("trackers-tbody");
const correctionTbody = document.getElementById("slimevr-corrections-tbody");
const correctionStatus = document.getElementById("slimevr-correction-status");
const correctionResetAll = document.getElementById("slimevr-reset-all");
const CORRECTION_AXES = ["yaw", "pitch", "roll"];

function normalizeQuarters(q) {
  let v = Number.isFinite(q) ? Math.trunc(q) : 0;
  v %= 4;
  if (v < 0) v += 4;
  return v === 3 ? -1 : v;
}

function setCorrectionStatus(text, cls = "") {
  if (!correctionStatus) return;
  correctionStatus.textContent = text;
  correctionStatus.className = cls;
}

function updateCorrectionRowsEnabled(enabled) {
  if (!correctionTbody) return;
  for (const button of correctionTbody.querySelectorAll("button")) {
    button.disabled = !enabled;
  }
  if (correctionResetAll) correctionResetAll.disabled = !enabled;
}

function ensureCorrectionRows() {
  if (!correctionTbody) return false;
  if (correctionTbody.dataset.built === "1") return true;
  correctionTbody.replaceChildren();
  for (const role of TRACKER_ROLES) {
    const tr = document.createElement("tr");
    tr.dataset.role = role;

    const roleCell = document.createElement("td");
    roleCell.textContent = role;
    tr.appendChild(roleCell);

    for (const axis of CORRECTION_AXES) {
      const td = document.createElement("td");
      const wrap = document.createElement("span");
      wrap.className = "slimevr-axis-control";

      const minus = document.createElement("button");
      minus.type = "button";
      minus.textContent = "-90";
      minus.addEventListener("click", () => adjustCorrection(role, axis, -1));

      const value = document.createElement("span");
      value.className = "slimevr-axis-value";
      value.dataset.axis = axis;
      value.textContent = "0";

      const plus = document.createElement("button");
      plus.type = "button";
      plus.textContent = "+90";
      plus.addEventListener("click", () => adjustCorrection(role, axis, 1));

      wrap.appendChild(minus);
      wrap.appendChild(value);
      wrap.appendChild(plus);
      td.appendChild(wrap);
      tr.appendChild(td);
    }

    const resetCell = document.createElement("td");
    const reset = document.createElement("button");
    reset.type = "button";
    reset.textContent = "0";
    reset.addEventListener("click", () => resetCorrection(role));
    resetCell.appendChild(reset);
    tr.appendChild(resetCell);

    correctionTbody.appendChild(tr);
  }
  correctionTbody.dataset.built = "1";
  updateCorrectionRowsEnabled(false);
  return true;
}

function applyCorrectionsPayload(payload) {
  if (!ensureCorrectionRows() || !payload || !Array.isArray(payload.roles)) return;
  for (const row of correctionTbody.children) {
    const role = row.dataset.role;
    const c = payload.roles.find((entry) => entry.role === role) || {};
    for (const axis of CORRECTION_AXES) {
      const q = normalizeQuarters(Number(c[`${axis}_quarters`] ?? 0));
      const value = row.querySelector(`[data-axis="${axis}"]`);
      if (value) value.textContent = String(q * 90);
    }
  }
  updateCorrectionRowsEnabled(true);
  setCorrectionStatus(payload.preview_no_reset === false ? "preview off" : "ready", "live");
}

async function loadCorrections() {
  if (!ensureCorrectionRows()) return;
  try {
    const res = await fetch("/api/slimevr/corrections", { cache: "no-store" });
    const payload = await res.json();
    if (!res.ok || !payload.ok) {
      updateCorrectionRowsEnabled(false);
      setCorrectionStatus(payload.err || "unavailable", "dead");
      return;
    }
    applyCorrectionsPayload(payload);
  } catch (e) {
    updateCorrectionRowsEnabled(false);
    setCorrectionStatus("unavailable", "dead");
  }
}

function rowCorrection(role) {
  const row = correctionTbody?.querySelector(`tr[data-role="${role}"]`);
  const out = {};
  for (const axis of CORRECTION_AXES) {
    const value = row?.querySelector(`[data-axis="${axis}"]`);
    out[`${axis}_quarters`] = normalizeQuarters(Number(value?.textContent ?? 0) / 90);
  }
  return out;
}

async function postCorrection(body) {
  updateCorrectionRowsEnabled(false);
  setCorrectionStatus("applying", "");
  try {
    const res = await fetch("/api/slimevr/corrections", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const payload = await res.json();
    if (!res.ok || !payload.ok) {
      setCorrectionStatus(payload.err || "failed", "dead");
      updateCorrectionRowsEnabled(true);
      return;
    }
    applyCorrectionsPayload(payload);
  } catch (e) {
    setCorrectionStatus("failed", "dead");
    updateCorrectionRowsEnabled(true);
  }
}

function adjustCorrection(role, axis, delta) {
  const current = rowCorrection(role);
  current[`${axis}_quarters`] = normalizeQuarters(current[`${axis}_quarters`] + delta);
  postCorrection({ role, ...current });
}

function resetCorrection(role) {
  postCorrection({ role, reset: true });
}

if (correctionResetAll) {
  correctionResetAll.addEventListener("click", () => postCorrection({ reset: true }));
}

// Phase 13 M2: live per-tracker stats table. Built once (lazy on first
// bundle), then cells are updated in place to avoid DOM churn at 30Hz.
function ensureTrackerTableRows() {
  if (!trackerTbody) return false;
  if (trackerTbody.dataset.built === "1") return true;
  trackerTbody.replaceChildren();
  for (let i = 0; i < TRACKER_COUNT; i += 1) {
    const tr = document.createElement("tr");
    tr.dataset.role = TRACKER_ROLES[i];
    // 9 columns: role / state / ang_vel_p50 / p95 / conf_avg / leakage_pct /
    // freeze_pct / freeze_max_ms / dropouts.
    for (let c = 0; c < 9; c += 1) {
      tr.appendChild(document.createElement("td"));
    }
    tr.firstChild.textContent = TRACKER_ROLES[i];
    trackerTbody.appendChild(tr);
  }
  trackerTbody.dataset.built = "1";
  return true;
}

function fmtPct(v) { return `${(Math.max(0, Math.min(1, v ?? 0)) * 100).toFixed(0)}%`; }
function fmtRad(v) { return (Number.isFinite(v) ? v : 0).toFixed(2); }

function updateTrackerTable(bundle) {
  if (!ensureTrackerTableRows()) return;
  const trackers = (bundle && Array.isArray(bundle.trackers)) ? bundle.trackers : [];
  for (let i = 0; i < TRACKER_COUNT; i += 1) {
    const tr = trackerTbody.children[i];
    if (!tr) continue;
    const t = trackers[i];
    const cells = tr.children;
    if (!t) {
      tr.classList.remove("state-frozen", "state-leakage", "state-active");
      // Phase 13 (Copilot): also clear the per-cell warn/bad classes on
      // the leakage/freeze columns, otherwise a row that was orange/red
      // before a disconnect keeps that text color after the row resets
      // to "-".
      cells[5].classList.remove("warn", "bad");
      cells[6].classList.remove("warn", "bad");
      for (let c = 1; c < cells.length; c += 1) cells[c].textContent = "-";
      continue;
    }
    const st = t.stats || {};
    const leak = Number(st.leakage_pct ?? 0);
    const frz  = Number(st.freeze_pct ?? 0);

    // State color: red if mostly frozen, yellow if mostly in leakage zone,
    // else neutral. Thresholds picked so a single transient drop doesn't
    // light up red — the test is "majority of recent frames".
    tr.classList.toggle("state-frozen",  frz  >= 0.5);
    tr.classList.toggle("state-leakage", frz  <  0.5 && leak >= 0.5);
    tr.classList.toggle("state-active",  frz  <  0.5 && leak <  0.5);

    // Phase 13 (post-review): held label requires a sustained invalid run,
    // not a single-frame drop. Aligns with the "majority of rolling window"
    // threshold used by frozen/leakage above. Without this, every
    // 1-3 frame sync miss in the 3D pipeline (= 10 trackers go invalid
    // simultaneously per the Codex P2 fix) made the state column flicker
    // through "held" on every tracker. 200 ms ≈ 12 frames at 60 Hz is the
    // shortest "intentional pause" a human perceives, below which we treat
    // the gap as a transient that doesn't deserve a label change.
    const freezeMs = Number(st.freeze_current_ms ?? 0);
    let stateLabel;
    if (frz  >= 0.5) stateLabel = "frozen";
    else if (leak >= 0.5) stateLabel = "leakage";
    else if (freezeMs >= 200) stateLabel = "held";
    else stateLabel = "active";

    cells[1].textContent = stateLabel;
    cells[2].textContent = fmtRad(st.ang_vel_p50);
    cells[3].textContent = fmtRad(st.ang_vel_p95);
    cells[4].textContent = (Number(st.conf_avg ?? 0)).toFixed(2);
    cells[5].textContent = fmtPct(leak);
    cells[5].classList.toggle("warn", leak >= 0.3 && leak < 0.5);
    cells[5].classList.toggle("bad",  leak >= 0.5);
    cells[6].textContent = fmtPct(frz);
    cells[6].classList.toggle("warn", frz >= 0.3 && frz < 0.5);
    cells[6].classList.toggle("bad",  frz >= 0.5);
    cells[7].textContent = String(st.freeze_max_ms ?? 0);
    cells[8].textContent = String(st.dropouts ?? 0);
  }
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

const trackerToggle = document.getElementById("toggle-trackers");
if (trackerToggle) {
  trackerToggle.addEventListener("change", () => {
    viewer3d?.setTrackersVisible(trackerToggle.checked);
  });
}

if (vmtAlignForm) {
  setVmtAlignmentEnabled(false);
  vmtAlignForm.addEventListener("submit", async (ev) => {
    ev.preventDefault();
    if (!state.vmtAlignmentEnabled) return;
    if (state.vmtPostTimer !== null) {
      clearTimeout(state.vmtPostTimer);
      state.vmtPostTimer = null;
    }
    setVmtAlignmentStatus("applying");
    try {
      await postVmtAlignment(readVmtAlignmentForm());
      setVmtAlignmentStatus("applied", "live");
    } catch (e) {
      setVmtAlignmentStatus(e.message || "apply failed", "dead");
    }
  });

  for (const key of Object.keys(vmtAlignInputs)) {
    const input = vmtAlignInputs[key];
    if (!input) continue;
    input.addEventListener("change", () => {
      if (!state.vmtAlignmentEnabled) return;
      updateVmtAlignmentTotals();
      scheduleVmtAlignmentPost(0);
    });
  }

  for (const key of Object.keys(vmtAlignSliders)) {
    const slider = vmtAlignSliders[key];
    if (!slider) continue;
    slider.addEventListener("input", () => {
      if (!state.vmtAlignmentEnabled) return;
      updateVmtAlignmentTotals();
      scheduleVmtAlignmentPost();
    });
    slider.addEventListener("change", () => {
      if (!state.vmtAlignmentEnabled) return;
      updateVmtAlignmentTotals();
      scheduleVmtAlignmentPost(0);
    });
  }
}

if (vmtAlignReset) {
  vmtAlignReset.addEventListener("click", async () => {
    if (!state.vmtAlignmentEnabled) return;
    if (state.vmtPostTimer !== null) {
      clearTimeout(state.vmtPostTimer);
      state.vmtPostTimer = null;
    }
    const zero = { x: 0, y: 0, z: 0, yaw_deg: 0 };
    writeVmtAlignmentForm(zero);
    setVmtAlignmentStatus("resetting");
    try {
      await postVmtAlignment(zero);
      setVmtAlignmentStatus("reset", "live");
    } catch (e) {
      setVmtAlignmentStatus(e.message || "reset failed", "dead");
    }
  });
}

// Phase 15: auto alignment helpers.
function setAutoResultText(text) {
  if (vmtAutoResult) vmtAutoResult.textContent = text || "—";
}

function describeAutoResult(result) {
  if (!result) return "—";
  if (result.status !== "ok") {
    return `${result.status}${result.err ? `: ${result.err}` : ""}`;
  }
  const a = result.alignment || {};
  const yaw = Number.isFinite(a.yaw_deg) ? a.yaw_deg : 0;
  const tx  = Number.isFinite(a.x) ? a.x : 0;
  const tz  = Number.isFinite(a.z) ? a.z : 0;
  const res = Number.isFinite(result.residual_m) ? result.residual_m : 0;
  return `yaw=${yaw.toFixed(2)}° tx=${tx.toFixed(3)} tz=${tz.toFixed(3)} residual=${res.toFixed(4)}m (n=${result.n_samples})`;
}

async function postAutoTpose() {
  if (!vmtAutoTposeBtn) return;
  vmtAutoTposeBtn.disabled = true;
  setAutoResultText("solving…");
  try {
    const resp = await fetch("/api/vmt/alignment/auto/tpose", { method: "POST" });
    const data = await resp.json();
    if (!resp.ok || data.ok === false) {
      setAutoResultText((data && data.err) || `HTTP ${resp.status}`);
      return;
    }
    setAutoResultText(describeAutoResult(data.result));
    // Reflect the new alignment in the manual form so subsequent slider
    // tweaks start from the auto-derived baseline.
    if (data.result && data.result.alignment) {
      writeVmtAlignmentForm(data.result.alignment);
    }
  } catch (e) {
    setAutoResultText(e.message || "request failed");
  } finally {
    vmtAutoTposeBtn.disabled = false;
  }
}

let vmtAutoMotionTimer = null;

function setAutoMotionUiCollecting(on) {
  if (vmtAutoStartBtn) vmtAutoStartBtn.disabled = on;
  if (vmtAutoStopBtn)  vmtAutoStopBtn.disabled  = !on;
  if (vmtAutoTposeBtn) vmtAutoTposeBtn.disabled = on;
}

async function startMotionCalib() {
  if (!vmtAutoStartBtn) return;
  setAutoMotionUiCollecting(true);
  setAutoResultText("collecting…");
  try {
    const resp = await fetch("/api/vmt/alignment/auto/motion/start", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ duration_s: VMT_AUTO_DURATION_S, sample_hz: VMT_AUTO_SAMPLE_HZ }),
    });
    const data = await resp.json();
    if (!resp.ok || data.ok === false) {
      setAutoMotionUiCollecting(false);
      setAutoResultText((data && data.err) || `HTTP ${resp.status}`);
      return;
    }
    // Auto-stop after duration + small slack so the server is guaranteed
    // to have finished the solve.
    vmtAutoMotionTimer = setTimeout(() => { stopMotionCalib(); },
                                    Math.round((VMT_AUTO_DURATION_S + 0.4) * 1000));
  } catch (e) {
    setAutoMotionUiCollecting(false);
    setAutoResultText(e.message || "request failed");
  }
}

async function stopMotionCalib() {
  if (vmtAutoMotionTimer !== null) { clearTimeout(vmtAutoMotionTimer); vmtAutoMotionTimer = null; }
  try {
    const resp = await fetch("/api/vmt/alignment/auto/motion/stop", { method: "POST" });
    const data = await resp.json();
    if (!resp.ok || data.ok === false) {
      setAutoResultText((data && data.err) || `HTTP ${resp.status}`);
    } else {
      setAutoResultText(describeAutoResult(data.result));
      if (data.result && data.result.status === "ok" && data.result.alignment) {
        writeVmtAlignmentForm(data.result.alignment);
      }
    }
  } catch (e) {
    setAutoResultText(e.message || "request failed");
  } finally {
    setAutoMotionUiCollecting(false);
  }
}

if (vmtAutoTposeBtn) vmtAutoTposeBtn.addEventListener("click", postAutoTpose);
if (vmtAutoStartBtn) vmtAutoStartBtn.addEventListener("click", startMotionCalib);
if (vmtAutoStopBtn)  vmtAutoStopBtn.addEventListener("click",  stopMotionCalib);

connect();
connect3d();
loadVmtAlignment();
loadCorrections();
requestAnimationFrame(renderTick);
