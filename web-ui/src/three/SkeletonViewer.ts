// Imperative Three.js skeleton + SlimeVR tracker viewer. Ported from the
// ThreeDViewer class in the legacy web/dual_rtmpose/app.js with minimal
// changes: the COCO17/Halpe26 topology constants come from lib/skeleton, the
// active keypoint format is held on the instance (set from the bundle) instead
// of a module-global, and a dispose() tears down the resize listener.

import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import {
  HALPE_ANKLE_INDICES,
  PERSON_3D_COLORS,
  KP_THR,
  TRACKER_COUNT,
  kpCountFor,
  skeletonFor,
} from "../lib/skeleton";
import type {
  Bundle3D,
  Camera3D,
  HmdBlock,
  Joint3D,
  KpFormat,
  Person3D,
  Tracker,
} from "../types/bundle";

const TRACKER_AXIS_BASE_LEN = 0.15;
// HMD marker: a small wireframe headset box + a forward gaze line. Drawn at the
// HMD's fitra-world pose (hmd.pos_world) so it shares the skeleton's space.
const HMD_COLOR = 0x33ddff;
const HMD_BOX = { w: 0.18, h: 0.1, d: 0.1 };
const HMD_GAZE_LEN = 0.25; // forward line length (camera -Z in HMD local frame)
// Camera frustum (wireframe pyramid) drawn at each calibrated camera. Apex sits
// at the camera center; the opening points along the camera's view direction.
const CAMERA_FRUSTUM_COLOR = 0xffc233;
const CAMERA_FRUSTUM_DEPTH = 0.2; // metres from apex to opening
const CAMERA_FRUSTUM_HALF_W = 0.13;
const CAMERA_FRUSTUM_HALF_H = 0.1;
const FLOOR_CONTACT_COLOR = 0x33ee77;
const FLOOR_CONTACT_RING_INNER_M = 0.055;
const FLOOR_CONTACT_RING_OUTER_M = 0.075;
// World (Z-up, X-right, Y-forward) → Three.js (Y-up) basis change = Rx(-90°).
const WORLD_TO_THREE_QUAT = (() => {
  const k = 0.7071067811865475; // 1/√2
  return new THREE.Quaternion(-k, 0, 0, k);
})();
const WORLD_TO_THREE_QUAT_INV = WORLD_TO_THREE_QUAT.clone().invert();
// Scratch quaternion reused by the per-frame camera/HMD update paths to keep
// those hot loops allocation-free.
const tempQuat = new THREE.Quaternion();

function isVisible3DJoint(joint: Joint3D | undefined): boolean {
  if (!Array.isArray(joint) || joint.length < 3) return false;
  if (joint[4] === false) return false;
  const score = Number(joint[3] ?? 1);
  if (!Number.isFinite(score) || score < KP_THR) return false;
  return [joint[0], joint[1], joint[2]].every((v) => Number.isFinite(Number(v)));
}

function jointToVector(joint: Joint3D, target: THREE.Vector3): THREE.Vector3 {
  // Backend joints are [x, y, z]; Three.js uses Y as up, so map z -> Y.
  target.set(Number(joint[0]), Number(joint[2]), -Number(joint[1]));
  return target;
}

interface PersonView {
  group: THREE.Group;
  joints: THREE.Mesh[];
  bones: Array<{ line: THREE.Line; positions: Float32Array }>;
  boneMaterial: THREE.LineBasicMaterial;
  boneFormat: KpFormat | null;
}

interface TrackerView {
  group: THREE.Group;
  axes: THREE.AxesHelper;
  lastGood: { position: THREE.Vector3; scale: number; hasData: boolean };
}

interface CameraView {
  group: THREE.Group;
}

export type ViewName = "front" | "side" | "top";

export class SkeletonViewer {
  private canvas: HTMLCanvasElement;
  private statusEl: HTMLElement | null;
  private personViews: PersonView[] = [];
  private view: ViewName = "front";
  private sceneRadius = 1.2;
  private hasData = false;
  private lastDataAtMs = 0;
  private kpFormat: KpFormat = "coco17";

  private bounds = new THREE.Box3();
  private boundsCenter = new THREE.Vector3(0, 0.9, 0);
  private boundsSize = new THREE.Vector3();

  private scene: THREE.Scene;
  private camera: THREE.PerspectiveCamera;
  private renderer: THREE.WebGLRenderer | null = null;
  private controls!: OrbitControls;
  private peopleRoot!: THREE.Group;
  private floorContactRoot!: THREE.Group;
  private floorContactRings: THREE.Mesh[] = [];
  private trackersRoot!: THREE.Group;
  private trackersVisible = true;
  private trackerViews: TrackerView[] = [];
  private camerasRoot!: THREE.Group;
  private camerasVisible = true;
  private cameraMaterial!: THREE.LineBasicMaterial;
  private cameraViews = new Map<string, CameraView>();
  private hmdGroup!: THREE.Group;
  private hmdVisible = true;
  private onResize = () => this.resize();

  constructor(canvas: HTMLCanvasElement, statusEl: HTMLElement | null) {
    this.canvas = canvas;
    this.statusEl = statusEl;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x050505);
    this.camera = new THREE.PerspectiveCamera(45, 4 / 3, 0.01, 100);
    this.camera.up.set(0, 1, 0);

    try {
      this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
    } catch {
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

    this.floorContactRoot = new THREE.Group();
    this.scene.add(this.floorContactRoot);
    const contactGeometry = new THREE.RingGeometry(
      FLOOR_CONTACT_RING_INNER_M,
      FLOOR_CONTACT_RING_OUTER_M,
      32,
    );
    const contactMaterial = new THREE.MeshBasicMaterial({
      color: FLOOR_CONTACT_COLOR,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.9,
      depthWrite: false,
    });
    for (let side = 0; side < 2; side += 1) {
      const ring = new THREE.Mesh(contactGeometry, contactMaterial);
      ring.rotation.x = -Math.PI / 2;
      ring.visible = false;
      ring.frustumCulled = false;
      this.floorContactRoot.add(ring);
      this.floorContactRings.push(ring);
    }

    this.trackersRoot = new THREE.Group();
    this.scene.add(this.trackersRoot);
    for (let i = 0; i < TRACKER_COUNT; i += 1) {
      const group = new THREE.Group();
      const axes = new THREE.AxesHelper(TRACKER_AXIS_BASE_LEN);
      (axes.material as THREE.LineBasicMaterial).transparent = true;
      (axes.material as THREE.LineBasicMaterial).opacity = 1.0;
      group.add(axes);
      group.visible = false;
      group.frustumCulled = false;
      this.trackersRoot.add(group);
      this.trackerViews.push({
        group,
        axes,
        lastGood: { position: new THREE.Vector3(), scale: 1.0, hasData: false },
      });
    }

    this.camerasRoot = new THREE.Group();
    this.scene.add(this.camerasRoot);
    this.cameraMaterial = new THREE.LineBasicMaterial({ color: CAMERA_FRUSTUM_COLOR });

    this.hmdGroup = this.buildHmdMarker();
    this.hmdGroup.visible = false;
    this.scene.add(this.hmdGroup);

    const grid = new THREE.GridHelper(4, 20, 0x335577, 0x2b2f36);
    (grid.material as THREE.Material).opacity = 0.75;
    (grid.material as THREE.Material).transparent = true;
    this.scene.add(grid);
    this.scene.add(new THREE.AxesHelper(0.75));

    window.addEventListener("resize", this.onResize);
    this.setView("front");
    this.setStatus("(no 3D data)");
    this.resize();
    this.render();
  }

  dispose(): void {
    window.removeEventListener("resize", this.onResize);
    this.controls?.dispose();

    // Geometries/materials are GPU resources that survive scene removal; on a
    // shared-memory device (Jetson) leaking them across route remounts can OOM
    // the tab. Walk the scene and dispose each unique resource once.
    const geometries = new Set<THREE.BufferGeometry>();
    const materials = new Set<THREE.Material>();
    this.scene.traverse((obj) => {
      const mesh = obj as Partial<THREE.Mesh> & Partial<THREE.Line>;
      if (mesh.geometry) geometries.add(mesh.geometry as THREE.BufferGeometry);
      const mat = mesh.material;
      if (Array.isArray(mat)) for (const m of mat) materials.add(m);
      else if (mat) materials.add(mat as THREE.Material);
    });
    for (const g of geometries) g.dispose();
    for (const m of materials) m.dispose();

    this.renderer?.dispose();
  }

  private setStatus(text: string): void {
    if (!this.statusEl) return;
    this.statusEl.textContent = text;
    this.statusEl.hidden = !text;
  }

  private ensurePersonView(index: number): PersonView {
    if (this.personViews[index]) return this.personViews[index];

    const color = PERSON_3D_COLORS[index % PERSON_3D_COLORS.length];
    const group = new THREE.Group();
    const jointGeometry = new THREE.SphereGeometry(0.025, 12, 8);
    const jointMaterial = new THREE.MeshBasicMaterial({ color });
    const boneMaterial = new THREE.LineBasicMaterial({ color });

    const joints = Array.from({ length: kpCountFor("halpe26") }, () => {
      const mesh = new THREE.Mesh(jointGeometry, jointMaterial);
      mesh.visible = false;
      mesh.frustumCulled = false;
      group.add(mesh);
      return mesh;
    });

    const view: PersonView = { group, joints, bones: [], boneMaterial, boneFormat: null };
    this.peopleRoot.add(group);
    this.personViews[index] = view;
    return view;
  }

  private ensureBones(view: PersonView, format: KpFormat): void {
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

  update(bundle: Bundle3D | null): void {
    if (!this.renderer) return;

    if (typeof bundle?.kp_format === "string") {
      this.kpFormat = bundle.kp_format;
    }

    // Camera frustums are static placement data, independent of whether a person
    // is currently triangulated; update them regardless of the branches below.
    this.updateCameras(Array.isArray(bundle?.cameras) ? bundle.cameras : []);
    // HMD marker is likewise independent of the person/disabled branches.
    this.updateHmd(bundle?.hmd);
    this.updateFloorContacts(bundle);

    if (!bundle || bundle.enabled === false) {
      this.setStatus(bundle && bundle.enabled === false ? "3D disabled" : "(no 3D data)");
      this.updatePeople([]);
      this.updateTrackers([]);
      return;
    }

    const persons = Array.isArray(bundle.persons_3d) ? bundle.persons_3d : [];
    const trackers = Array.isArray(bundle.trackers) ? bundle.trackers : [];
    if (!persons.length) {
      if (this.hasData && performance.now() - this.lastDataAtMs < 3500) {
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

  setTrackersVisible(visible: boolean): void {
    this.trackersVisible = !!visible;
    if (this.trackersRoot) this.trackersRoot.visible = this.trackersVisible;
  }

  setCamerasVisible(visible: boolean): void {
    this.camerasVisible = !!visible;
    if (this.camerasRoot) this.camerasRoot.visible = this.camerasVisible;
  }

  private updateFloorContacts(bundle: Bundle3D | null): void {
    const stats = bundle?.stats;
    const person = Array.isArray(bundle?.persons_3d) ? bundle.persons_3d[0] : undefined;
    const joints = Array.isArray(person?.joints) ? person.joints : [];
    const floorZ = Number(stats?.floor_z_m ?? 0);
    const active = [stats?.floor_contact_left === true, stats?.floor_contact_right === true];
    const stale = stats?.floor_contact_fresh === false;

    for (let side = 0; side < this.floorContactRings.length; side += 1) {
      const ring = this.floorContactRings[side];
      if (stale && active[side]
          && performance.now() - this.lastDataAtMs < 3500) {
        // Match the viewer's short last-person hold across sync misses so the
        // ring does not flash plant -> air -> plant on a transport gap.
        continue;
      }
      const ankle = joints[HALPE_ANKLE_INDICES[side]];
      if (stats?.floor_stability_enabled !== true || !active[side]
          || !Number.isFinite(floorZ) || !isVisible3DJoint(ankle)) {
        ring.visible = false;
        continue;
      }
      ring.position.set(Number(ankle[0]), floorZ + 0.002, -Number(ankle[1]));
      ring.visible = true;
    }
  }

  // Wireframe pyramid with the apex at the local origin (camera center) opening
  // toward local +Z (the camera's view direction in camera-frame coords). The
  // group's quaternion then rotates +Z to the actual view direction in world.
  private buildCameraFrustum(): THREE.LineSegments {
    const d = CAMERA_FRUSTUM_DEPTH;
    const w = CAMERA_FRUSTUM_HALF_W;
    const h = CAMERA_FRUSTUM_HALF_H;
    // 4 apex->corner edges + 4 rectangle edges = 8 segments (16 vertices).
    const corners = [
      [-w, -h, d],
      [w, -h, d],
      [w, h, d],
      [-w, h, d],
    ];
    const verts: number[] = [];
    for (const c of corners) verts.push(0, 0, 0, c[0], c[1], c[2]);
    for (let i = 0; i < 4; i += 1) {
      const a = corners[i];
      const b = corners[(i + 1) % 4];
      verts.push(a[0], a[1], a[2], b[0], b[1], b[2]);
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(verts, 3));
    const lines = new THREE.LineSegments(geometry, this.cameraMaterial);
    lines.frustumCulled = false;
    return lines;
  }

  private updateCameras(cameras: Camera3D[]): void {
    if (!this.camerasRoot) return;
    this.camerasRoot.visible = this.camerasVisible;

    const seen = new Set<string>();
    for (const cam of cameras) {
      if (!cam || !Array.isArray(cam.pos) || !Array.isArray(cam.quat_wxyz)) continue;
      const id = String(cam.id);
      seen.add(id);

      let view = this.cameraViews.get(id);
      if (!view) {
        const group = new THREE.Group();
        group.frustumCulled = false;
        group.add(this.buildCameraFrustum());
        this.camerasRoot.add(group);
        view = { group };
        this.cameraViews.set(id, view);
      }

      // Position: world (x, y, z) -> Three.js (x, z, -y), matching jointToVector.
      view.group.position.set(Number(cam.pos[0]), Number(cam.pos[2]), -Number(cam.pos[1]));

      // Orientation: B·R (compose), NOT the similarity transform B·R·B⁻¹.
      // Camera quat_wxyz comes from the raw camera->world extrinsic (Rᵀ), whose
      // local frame is still the OpenCV camera convention (+Z = optical axis),
      // and the frustum opening is authored along three.js-local +Z to match it
      // directly. Because the rotation's local frame is NOT rebased into the
      // three.js basis, only the world side needs B; conjugating would
      // double-apply the basis change and tilt a forward frustum up 90°.
      // (The HMD path differs — see updateHmd — because its quat goes through
      // vmt_pose_to_world, which rebases the local frame like a tracker.)
      tempQuat.set(
        Number(cam.quat_wxyz[1]),
        Number(cam.quat_wxyz[2]),
        Number(cam.quat_wxyz[3]),
        Number(cam.quat_wxyz[0]),
      );
      view.group.quaternion.multiplyQuaternions(WORLD_TO_THREE_QUAT, tempQuat);
      view.group.visible = true;
    }

    for (const [id, view] of this.cameraViews) {
      if (!seen.has(id)) view.group.visible = false;
    }
  }

  setHmdVisible(visible: boolean): void {
    this.hmdVisible = !!visible;
    // Actual visibility also depends on whether a pose is present; updateHmd
    // re-applies hmdVisible each frame, so just refresh now.
    if (this.hmdGroup && !this.hmdVisible) this.hmdGroup.visible = false;
  }

  // Wireframe headset box centred at the local origin plus a forward gaze line
  // along local -Z (the HMD's view direction in OpenVR's device frame).
  private buildHmdMarker(): THREE.Group {
    const group = new THREE.Group();
    group.frustumCulled = false;
    const material = new THREE.LineBasicMaterial({ color: HMD_COLOR });

    const box = new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.BoxGeometry(HMD_BOX.w, HMD_BOX.h, HMD_BOX.d)),
      material,
    );
    box.frustumCulled = false;
    group.add(box);

    const gaze = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(0, 0, 0),
        new THREE.Vector3(0, 0, -HMD_GAZE_LEN),
      ]),
      material,
    );
    gaze.frustumCulled = false;
    group.add(gaze);
    return group;
  }

  private updateHmd(hmd: HmdBlock | undefined): void {
    if (!this.hmdGroup) return;
    // Need the world-frame pose (only present when VMT alignment is known) and a
    // live, valid HMD reading.
    const pos = hmd?.pos_world;
    if (
      !this.hmdVisible ||
      !hmd ||
      hmd.have_any !== true ||
      hmd.valid === false ||
      hmd.stale === true ||
      !Array.isArray(pos)
    ) {
      this.hmdGroup.visible = false;
      return;
    }

    // World (x, y, z) -> Three.js (x, z, -y), matching jointToVector/trackers.
    this.hmdGroup.position.set(Number(pos[0]), Number(pos[2]), -Number(pos[1]));

    const q = hmd.quat_wxyz;
    if (Array.isArray(q) && q.length === 4) {
      // Conjugate into the Three.js basis (B·R·B⁻¹), same as the tracker path —
      // NOT the camera path's B·R. The HMD quat_wxyz is delivered by
      // vmt_pose_to_world, which rebases the rotation from the VMT frame into
      // the fitra world frame on BOTH sides (local + world), so its local frame
      // is fitra-convention just like a tracker's. With identity orientation
      // the correct forward is fitra +Y → three.js -Z; B·R alone would tilt the
      // gaze line down to -Y (a 90° error), B·R·B⁻¹ keeps it at -Z.
      tempQuat.set(Number(q[1]), Number(q[2]), Number(q[3]), Number(q[0]));
      this.hmdGroup.quaternion
        .multiplyQuaternions(WORLD_TO_THREE_QUAT, tempQuat)
        .multiply(WORLD_TO_THREE_QUAT_INV);
    }
    this.hmdGroup.visible = true;
  }

  private updateTrackers(trackers: Tracker[]): void {
    if (!this.trackersRoot) return;
    this.trackersRoot.visible = this.trackersVisible;

    for (let i = 0; i < TRACKER_COUNT; i += 1) {
      const view = this.trackerViews[i];
      const t = trackers[i];
      if (!t || !Array.isArray(t.pos) || !Array.isArray(t.quat_wxyz)) {
        view.group.visible = false;
        continue;
      }

      const qw = Number(t.quat_wxyz[0]);
      const qx = Number(t.quat_wxyz[1]);
      const qy = Number(t.quat_wxyz[2]);
      const qz = Number(t.quat_wxyz[3]);
      const qWorld = new THREE.Quaternion(qx, qy, qz, qw);
      const qThree = WORLD_TO_THREE_QUAT.clone()
        .multiply(qWorld)
        .multiply(WORLD_TO_THREE_QUAT_INV);
      view.group.quaternion.copy(qThree);

      const st = t.stats || {};
      const freezeMs = Number(st.freeze_current_ms ?? 0);
      const conf = Number.isFinite(t.roll_confidence) ? Number(t.roll_confidence) : 1.0;
      const scaleFromConf = 0.3 + 0.7 * Math.max(0, Math.min(1, conf));

      if (t.valid) {
        view.group.position.set(Number(t.pos[0]), Number(t.pos[2]), -Number(t.pos[1]));
        view.group.scale.setScalar(scaleFromConf);
        view.lastGood.position.copy(view.group.position);
        view.lastGood.scale = scaleFromConf;
        view.lastGood.hasData = true;
      } else if (view.lastGood.hasData) {
        view.group.position.copy(view.lastGood.position);
        view.group.scale.setScalar(view.lastGood.scale);
      } else {
        view.group.visible = false;
        continue;
      }

      (view.axes.material as THREE.LineBasicMaterial).opacity = freezeMs >= 200 ? 0.3 : 1.0;
      view.group.visible = true;
    }
  }

  private updatePeople(persons: Person3D[]): void {
    const visiblePoints: THREE.Vector3[] = [];
    const scratch = new THREE.Vector3();
    const format = this.kpFormat;
    const kpCount = kpCountFor(format);
    const skeleton = skeletonFor(format);

    persons.forEach((person, personIndex) => {
      const view = this.ensurePersonView(personIndex);
      this.ensureBones(view, format);
      view.group.visible = true;
      const joints = Array.isArray(person.joints) ? person.joints : [];
      const jointVectors: Array<THREE.Vector3 | null> = Array.from({ length: kpCount }, () => null);

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
        (bone.line.geometry.attributes.position as THREE.BufferAttribute).needsUpdate = true;
        bone.line.visible = true;
      });
    });

    for (let i = persons.length; i < this.personViews.length; i += 1) {
      this.personViews[i].group.visible = false;
    }

    this.updateBounds(visiblePoints);
  }

  private updateBounds(points: THREE.Vector3[]): void {
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

  setView(view: ViewName): void {
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

  private resize(): void {
    if (!this.renderer) return;
    const width = Math.max(1, Math.round(this.canvas.clientWidth || this.canvas.width));
    const height = Math.max(1, Math.round(this.canvas.clientHeight || this.canvas.height));
    const current = this.renderer.getSize(new THREE.Vector2());
    if (current.x === width && current.y === height) return;

    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }

  render(): void {
    if (!this.renderer) return;
    this.resize();
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}
