// 3D verification scene for the controller-marker extrinsic calibration.
//
// Polls /api/excal/extrinsics and places one camera frustum per solved camera
// in the shared world frame (VMT Standing, Y-up RH, metres). Each frustum's
// apex is the camera centre and its opening reflects the real intrinsics FoV,
// so you can eyeball where the cameras sit in the room and whether their
// relative geometry looks right.

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

async function refresh() {
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

  const status = document.getElementById("status");
  if (!data.solved || cams.length === 0) {
    status.textContent = "no solution yet — run a solve on the collect page.";
  } else {
    status.textContent = `${cams.length} camera(s): `
      + cams.map((c) => `${c.id} @ (${c.center.map((v) => v.toFixed(2)).join(", ")})`).join("  ·  ");
  }
}

function animate() {
  resize();
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}

refresh();
setInterval(refresh, 1000);
animate();
