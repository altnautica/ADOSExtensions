/**
 * @module world/spark-stage
 * @description The world viewer's 3D stage: a three.js scene rendering a
 * Gaussian splat with Spark (SPZ / PLY bytes, or a `.rad` level-of-detail file
 * streamed by URL), an optional collision mesh (GLB) used for picking, the
 * floorplan's walls drawn on the floor, and measurement markers. Three view
 * modes: orbit, dollhouse (orbit from above) and first-person walk.
 *
 * Framework-free so the plugin page and the standalone tour viewer share it.
 * Everything is placed through `world/placement`: artifacts by their
 * placement, overlays from local ENU metres.
 */

import type * as T from "three";
import type * as SparkNs from "@sparkjsdev/spark";
import type { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import type { SparkRenderer, SplatMesh } from "@sparkjsdev/spark";

import type { Placement } from "../../lib/contract/generated";
import type { FloorplanFloor } from "../../lib/world/floorplan";
import { floorLinesViewer } from "../../lib/world/floorplan";
import { artifactToViewerMatrix, type Vec3 } from "../../lib/world/placement";
import { followCanvasSize } from "../atlas/viewers/cloud-view";
import { WalkControls } from "./walk-controls";

export type ViewMode = "orbit" | "dollhouse" | "walk";

/** The splat to show: whole-file bytes, or a LOD root streamed by URL. */
export type SplatInput = { bytes: Uint8Array; fileName: string } | { url: string; fileName: string };

type Three = typeof T;
type Spark = typeof SparkNs;

const PLAN_COLOUR = 0x38bdf8;
const MEASURE_COLOUR = 0xfacc15;

export class SparkStage {
  readonly camera: T.PerspectiveCamera;
  mode: ViewMode = "orbit";
  private readonly renderer: T.WebGLRenderer;
  private readonly scene: T.Scene;
  private readonly spark: SparkRenderer;
  private readonly orbit: OrbitControls;
  private readonly walk: WalkControls;
  private readonly raycaster: T.Raycaster;
  private splat: SplatMesh | null = null;
  private collision: T.Object3D | null = null;
  private plan: T.LineSegments | null = null;
  private measure: T.Group;
  private bounds: T.Box3;
  private floorY = 0;
  private raf = 0;
  private last = 0;
  private readonly stopResize: () => void;
  private readonly frameListeners = new Set<() => void>();

  static async create(canvas: HTMLCanvasElement): Promise<SparkStage> {
    // Loaded on demand: three, Spark (and its wasm + workers) stay out of the
    // module graph of every page and test that never opens a 3D view.
    const [THREE, SPARK, orbit] = await Promise.all([
      import("three"),
      import("@sparkjsdev/spark"),
      import("three/examples/jsm/controls/OrbitControls.js"),
    ]);
    return new SparkStage(canvas, THREE, SPARK, orbit.OrbitControls);
  }

  private constructor(
    private readonly canvas: HTMLCanvasElement,
    private readonly THREE: Three,
    private readonly SPARK: Spark,
    Orbit: typeof OrbitControls,
  ) {
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: false, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setClearColor(0x0b0d10, 1);
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(60, 1, 0.05, 2000);
    this.camera.position.set(4, 4, 6);
    this.spark = new SPARK.SparkRenderer({ renderer: this.renderer });
    this.scene.add(this.spark);
    this.orbit = new Orbit(this.camera, canvas);
    this.orbit.enableDamping = true;
    this.walk = new WalkControls(this.camera, canvas);
    this.raycaster = new THREE.Raycaster();
    this.measure = new THREE.Group();
    this.scene.add(this.measure);
    this.bounds = new THREE.Box3();
    this.stopResize = followCanvasSize(canvas, this.renderer, this.camera);
    this.loop(performance.now());
  }

  /** Called after every rendered frame (overlay positions follow the camera). */
  onFrame(listener: () => void): () => void {
    this.frameListeners.add(listener);
    return () => this.frameListeners.delete(listener);
  }

  private loop = (now: number) => {
    const dt = Math.min(0.1, (now - (this.last || now)) / 1000);
    this.last = now;
    if (this.mode === "walk") this.walk.update(dt);
    else this.orbit.update();
    this.renderer.render(this.scene, this.camera);
    for (const l of this.frameListeners) l();
    this.raf = requestAnimationFrame(this.loop);
  };

  /** Load the splat, placed by `placement` (null = already local ENU). */
  async loadSplat(input: SplatInput, placement: Placement | null): Promise<void> {
    const { SplatMesh, SplatFileType } = this.SPARK;
    const ext = input.fileName.toLowerCase().split(".").pop() ?? "";
    const fileType = Object.values(SplatFileType).find((t) => t === ext);
    const mesh =
      "bytes" in input
        ? new SplatMesh({ fileBytes: input.bytes, fileType, fileName: input.fileName })
        : new SplatMesh({ url: input.url, fileType, paged: true });
    mesh.matrixAutoUpdate = false;
    mesh.matrix.set(...(artifactToViewerMatrix(placement) as Parameters<T.Matrix4["set"]>));
    mesh.updateMatrixWorld(true);
    this.splat?.dispose();
    if (this.splat) this.scene.remove(this.splat);
    this.splat = mesh;
    this.scene.add(mesh);
    await mesh.initialized;
    const box = mesh.getBoundingBox(true);
    if (!box.isEmpty()) this.bounds.union(box.applyMatrix4(mesh.matrix));
    this.frame();
  }

  /** Load the collision mesh (GLB, local ENU, Z up) used for picking. */
  async loadCollision(glb: ArrayBuffer, show = false): Promise<void> {
    const { GLTFLoader } = await import("three/examples/jsm/loaders/GLTFLoader.js");
    const gltf = await new GLTFLoader().parseAsync(glb, "");
    const root = gltf.scene;
    root.matrixAutoUpdate = false;
    root.matrix.set(...(artifactToViewerMatrix(null) as Parameters<T.Matrix4["set"]>));
    root.traverse((o) => {
      const mesh = o as T.Mesh;
      if (mesh.isMesh) {
        mesh.material = new this.THREE.MeshBasicMaterial({ color: 0x94a3b8, wireframe: true, opacity: 0.25, transparent: true });
        mesh.visible = show;
      }
    });
    root.updateMatrixWorld(true);
    if (this.collision) this.scene.remove(this.collision);
    this.collision = root;
    this.scene.add(root);
    if (this.bounds.isEmpty()) {
      this.bounds.setFromObject(root);
      this.frame();
    }
  }

  setCollisionVisible(show: boolean): void {
    this.collision?.traverse((o) => {
      if ((o as T.Mesh).isMesh) o.visible = show;
    });
  }

  /** Draw one floor's walls on the floor (null clears the plan). */
  setFloor(floor: FloorplanFloor | null): void {
    if (this.plan) {
      this.scene.remove(this.plan);
      this.plan.geometry.dispose();
      (this.plan.material as T.Material).dispose();
      this.plan = null;
    }
    if (!floor) return;
    this.floorY = floor.zM;
    const geom = new this.THREE.BufferGeometry();
    geom.setAttribute("position", new this.THREE.BufferAttribute(floorLinesViewer(floor), 3));
    // Drawn after the (transparent, depth-less) splat so the plan stays visible.
    this.plan = new this.THREE.LineSegments(
      geom,
      new this.THREE.LineBasicMaterial({ color: PLAN_COLOUR, depthTest: false, transparent: true }),
    );
    this.plan.renderOrder = 10;
    this.scene.add(this.plan);
    if (this.bounds.isEmpty()) {
      geom.computeBoundingBox();
      if (geom.boundingBox) this.bounds.copy(geom.boundingBox);
      this.frame();
    }
  }

  setMode(mode: ViewMode): void {
    this.mode = mode;
    this.orbit.enabled = mode !== "walk";
    if (mode === "walk") this.walk.enter(this.floorY);
    else this.frame(mode === "dollhouse");
  }

  setStick(forward: number, strafe: number): void {
    this.walk.stick = { forward, strafe };
  }

  /** Fit the camera to the scene: a three-quarter view, or from above. */
  frame(fromAbove = this.mode === "dollhouse"): void {
    if (this.bounds.isEmpty() || this.mode === "walk") return;
    const sphere = this.bounds.getBoundingSphere(new this.THREE.Sphere());
    const r = Math.max(sphere.radius, 0.5);
    const dist = (r / Math.tan((this.camera.fov * Math.PI) / 360)) * 1.1;
    const dir = fromAbove ? new this.THREE.Vector3(0.25, 1.6, 0.6) : new this.THREE.Vector3(0.6, 0.45, 1);
    this.camera.near = Math.max(dist / 2000, 0.02);
    this.camera.far = dist * 20;
    this.camera.updateProjectionMatrix();
    this.camera.position.copy(sphere.center).addScaledVector(dir.normalize(), dist);
    this.orbit.target.copy(sphere.center);
    this.orbit.maxPolarAngle = fromAbove ? Math.PI / 3 : Math.PI;
    this.orbit.update();
  }

  /** The viewer-space point under client pixel `(x, y)`: the collision mesh
   * when loaded, else the current floor plane; null when nothing is hit. */
  pick(clientX: number, clientY: number): Vec3 | null {
    const rect = this.canvas.getBoundingClientRect();
    const ndc = new this.THREE.Vector2(
      ((clientX - rect.left) / rect.width) * 2 - 1,
      -((clientY - rect.top) / rect.height) * 2 + 1,
    );
    this.raycaster.setFromCamera(ndc, this.camera);
    if (this.collision) {
      const hit = this.raycaster.intersectObject(this.collision, true)[0];
      if (hit) return [hit.point.x, hit.point.y, hit.point.z];
    }
    const plane = new this.THREE.Plane(new this.THREE.Vector3(0, 1, 0), -this.floorY);
    const p = this.raycaster.ray.intersectPlane(plane, new this.THREE.Vector3());
    return p ? [p.x, p.y, p.z] : null;
  }

  /** Where a viewer-space point lands on the canvas, in CSS pixels. */
  project(p: Vec3): { x: number; y: number; visible: boolean } {
    const v = new this.THREE.Vector3(...p).project(this.camera);
    const rect = this.canvas.getBoundingClientRect();
    return { x: ((v.x + 1) / 2) * rect.width, y: ((1 - v.y) / 2) * rect.height, visible: v.z > -1 && v.z < 1 };
  }

  /** Draw measurement points (viewer space) joined by a line. */
  setMeasure(points: readonly Vec3[]): void {
    for (const c of [...this.measure.children]) {
      this.measure.remove(c);
      const m = c as T.Mesh;
      m.geometry?.dispose();
      (m.material as T.Material | undefined)?.dispose();
    }
    const overlay = { color: MEASURE_COLOUR, depthTest: false, transparent: true };
    for (const p of points) {
      const dot = new this.THREE.Mesh(new this.THREE.SphereGeometry(0.04, 12, 8), new this.THREE.MeshBasicMaterial(overlay));
      dot.position.set(...p);
      dot.renderOrder = 11;
      this.measure.add(dot);
    }
    if (points.length === 2) {
      const geom = new this.THREE.BufferGeometry().setFromPoints(points.map((p) => new this.THREE.Vector3(...p)));
      const line = new this.THREE.Line(geom, new this.THREE.LineBasicMaterial(overlay));
      line.renderOrder = 11;
      this.measure.add(line);
    }
  }

  dispose(): void {
    cancelAnimationFrame(this.raf);
    this.frameListeners.clear();
    this.stopResize();
    this.orbit.dispose();
    this.walk.dispose();
    this.setMeasure([]);
    this.setFloor(null);
    this.splat?.dispose();
    this.collision?.traverse((o) => {
      const m = o as T.Mesh;
      if (m.isMesh) {
        m.geometry.dispose();
        (m.material as T.Material).dispose();
      }
    });
    this.renderer.dispose();
  }
}
