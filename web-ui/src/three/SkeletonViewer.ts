// Imperative Three.js skeleton + SlimeVR tracker viewer. Ported from the
// ThreeDViewer class in the legacy web/dual_rtmpose/app.js with minimal
// changes: the COCO17/Halpe26 topology constants come from lib/skeleton, the
// active keypoint format is held on the instance (set from the bundle) instead
// of a module-global, and a dispose() tears down the resize listener.

import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import {
  PERSON_3D_COLORS,
  KP_THR,
  TRACKER_COUNT,
  kpCountFor,
  skeletonFor,
} from "../lib/skeleton";
import type { Bundle3D, Joint3D, KpFormat, Person3D, Tracker } from "../types/bundle";

const TRACKER_AXIS_BASE_LEN = 0.15;
// World (Z-up, X-right, Y-forward) → Three.js (Y-up) basis change = Rx(-90°).
const WORLD_TO_THREE_QUAT = (() => {
  const k = 0.7071067811865475; // 1/√2
  return new THREE.Quaternion(-k, 0, 0, k);
})();
const WORLD_TO_THREE_QUAT_INV = WORLD_TO_THREE_QUAT.clone().invert();

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
  private trackersRoot!: THREE.Group;
  private trackersVisible = true;
  private trackerViews: TrackerView[] = [];
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
