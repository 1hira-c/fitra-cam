// 3D verification scene for the controller-marker extrinsic calibration.
//
// Polls /api/excal/extrinsics for solved camera frustums and /api/excal/poses
// for the live HMD + selected calibration controller pose. All are already in
// the shared SteamVR Standing / VMT world frame (Y-up RH, metres), so positions
// and quaternions are applied directly to Three.js objects.

import * as THREE from "three";
import { OrbitControls } from "OrbitControls";

const canvas = document.getElementById("view");
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0e13);

const camera = new THREE.PerspectiveCamera(55, 2, 0.01, 100);
camera.position.set(2.5, 2.0, 3.0);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 1.0, 1.5);

scene.add(new THREE.GridHelper(10, 20, 0x335, 0x223));  // floor at Y=0
scene.add(new THREE.AxesHelper(0.5));                   // world origin
const hemi = new THREE.HemisphereLight(0xbbccff, 0x223344, 1.2);
scene.add(hemi);

const CAM_COLORS = [0x66ccff, 0xffaa66, 0x88ff88, 0xff88cc, 0xffff66, 0xcc99ff];
let camGroup = new THREE.Group();
scene.add(camGroup);

const poseGroup = new THREE.Group();
scene.add(poseGroup);

let hmdViz = null;
let controllerViz = null;
let extrinsicsStatus = "waiting for a solve...";
let poseStatus = "waiting for pose relay...";

function resize() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== w || canvas.height !== h) {
    renderer.setSize(w, h, false);
    camera.aspect = w / Math.max(h, 1);
    camera.updateProjectionMatrix();
  }
}

// Build a frustum (apex at origin, opening toward +Z = OpenCV camera forward)
// from intrinsics, plus a small axis triad, as a Group whose local→world
// matrix is cam→world = [Rᵀ | center].
function makeCameraViz(cam, color) {
  const g = new THREE.Group();
  const depth = 0.35;
  const hw = ((cam.width / 2) / cam.fx) * depth;
  const hh = ((cam.height / 2) / cam.fy) * depth;
  const c = [
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(-hw, -hh, depth),
    new THREE.Vector3(hw, -hh, depth),
    new THREE.Vector3(hw, hh, depth),
    new THREE.Vector3(-hw, hh, depth),
  ];
  const lines = [
    c[0], c[1], c[0], c[2], c[0], c[3], c[0], c[4],   // apex → corners
    c[1], c[2], c[2], c[3], c[3], c[4], c[4], c[1],   // base rectangle
  ];
  const geo = new THREE.BufferGeometry().setFromPoints(lines);
  g.add(new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color })));
  g.add(new THREE.AxesHelper(0.12));  // camera axes (R=x, G=y, B=z forward)

  // cam→world matrix from T_cam_world (world→cam, row-major) + center.
  const T = cam.T_cam_world;        // 16 row-major
  const ctr = cam.center;           // [x,y,z] = -Rᵀ t
  const m = new THREE.Matrix4();
  // row i = (Rᵀ row i) = R column i = [T[i], T[4+i], T[8+i]]; translation = center.
  m.set(
    T[0], T[4], T[8],  ctr[0],
    T[1], T[5], T[9],  ctr[1],
    T[2], T[6], T[10], ctr[2],
    0, 0, 0, 1
  );
  g.matrixAutoUpdate = false;
  g.matrix.copy(m);
  return g;
}

function makePoseViz(color, radius) {
  const g = new THREE.Group();
  const sphere = new THREE.Mesh(
    new THREE.SphereGeometry(radius, 24, 16),
    new THREE.MeshStandardMaterial({
      color,
      roughness: 0.55,
      metalness: 0.05,
      emissive: color,
      emissiveIntensity: 0.12,
    })
  );
  g.add(sphere);
  g.add(new THREE.AxesHelper(radius * 3.5));
  return g;
}

function updatePoseViz(current, snap, color, radius) {
  if (!snap || !snap.have_any || snap.stale || !snap.valid || !snap.pos || !snap.quat_xyzw) {
    if (current) {
      poseGroup.remove(current);
    }
    return null;
  }
  const viz = current || makePoseViz(color, radius);
  if (!current) {
    poseGroup.add(viz);
  }
  viz.position.set(snap.pos[0], snap.pos[1], snap.pos[2]);
  viz.quaternion.set(
    snap.quat_xyzw[0],
    snap.quat_xyzw[1],
    snap.quat_xyzw[2],
    snap.quat_xyzw[3]
  );
  return viz;
}

function poseLabel(name, snap) {
  if (!snap || !snap.enabled) return `${name}: disabled`;
  if (!snap.have_any) return `${name}: missing`;
  if (snap.stale) return `${name}: stale ${Math.round(snap.age_ms)} ms`;
  if (!snap.valid) return `${name}: invalid`;
  if ("running_ok" in snap && !snap.running_ok) {
    return `${name}: tracking ${snap.tracking_result}`;
  }
  const age = Number.isFinite(snap.age_ms) ? `${Math.round(snap.age_ms)} ms` : "? ms";
  return `${name}: ok ${age}`;
}

function updateStatus() {
  document.getElementById("status").textContent =
    `${extrinsicsStatus}  |  ${poseStatus}`;
}

async function refreshExtrinsics() {
  let data;
  try { data = await (await fetch("/api/excal/extrinsics")).json(); }
  catch (e) { document.getElementById("conn").textContent = "disconnected"; return; }
  document.getElementById("conn").textContent = "ok";

  scene.remove(camGroup);
  camGroup = new THREE.Group();
  scene.add(camGroup);

  const cams = data.cameras || [];
  (cams).forEach((cam, i) => {
    camGroup.add(makeCameraViz(cam, CAM_COLORS[i % CAM_COLORS.length]));
  });

  if (!data.solved || cams.length === 0) {
    extrinsicsStatus = "cameras: no solution yet";
  } else {
    extrinsicsStatus = `${cams.length} camera(s): `
      + cams.map((c) => `${c.id} @ (${c.center.map((v) => v.toFixed(2)).join(", ")})`).join("  ·  ");
  }
  updateStatus();
}

async function refreshPoses() {
  let data;
  try { data = await (await fetch("/api/excal/poses")).json(); }
  catch (e) { document.getElementById("conn").textContent = "disconnected"; return; }
  document.getElementById("conn").textContent = "ok";

  hmdViz = updatePoseViz(hmdViz, data.hmd, 0x4da3ff, 0.055);
  controllerViz = updatePoseViz(controllerViz, data.controller, 0xffd166, 0.045);

  const controllerName = data.controller?.role
    ? `${data.controller.role} controller`
    : "controller";
  poseStatus = `${poseLabel("HMD", data.hmd)}  ·  ${poseLabel(controllerName, data.controller)}`;
  updateStatus();
}

function animate() {
  resize();
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}

refreshExtrinsics();
refreshPoses();
setInterval(refreshExtrinsics, 1500);
setInterval(refreshPoses, 100);
animate();
