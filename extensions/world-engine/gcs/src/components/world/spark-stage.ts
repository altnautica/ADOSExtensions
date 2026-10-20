/**
 * @module world/spark-stage
 * @description The world viewer's 3D stage: a three.js scene rendering a
 * Gaussian splat with Spark (SPZ / PLY bytes, or a `.rad` level-of-detail file
 * streamed by URL), an optional collision mesh (GLB) used for picking, the
 * floorplan's walls and warehouse aisles drawn on the floor, and measurement
 * markers. Three view modes: orbit, dollhouse (orbit from above) and
 * first-person walk.
 *
 * Framework-free so the plugin page and the standalone tour viewer share it.
 * Everything is placed through `world/placement`: artifacts by their frame,
 * overlays from local ENU metres. Every replaced object and the stage itself
 * free their GPU resources (`scene-resources`), Spark's renderer included.
 */

import type * as T from "three";
import type * as SparkNs from "@sparkjsdev/spark";
import type { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import type { SparkRenderer, SplatMesh } from "@sparkjsdev/spark";

import { aisleLinesViewer, type Aisle } from "../../lib/world/aisles";
import type { FloorplanFloor } from "../../lib/world/floorplan";
import { floorLinesViewer } from "../../lib/world/floorplan";
import { artifactToViewerMatrix, type ArtifactFrame, type Vec3 } from "../../lib/world/placement";
import { splatFramingBounds } from "../../lib/world/splat-bounds";
import { followCanvasSize } from "../atlas/viewers/cloud-view";
import { disposeMaterials, disposeObject } from "./scene-resources";
import { WalkControls } from "./walk-controls";

export type ViewMode = "orbit" | "dollhouse" | "walk";

/** The splat to show: whole-file bytes, or a LOD root streamed by URL. */
export type SplatInput = { bytes: Uint8Array; fileName: string } | { url: string; fileName: string };

type Three = typeof T;
type Spark = typeof SparkNs;
type Matrix16 = Parameters<T.Matrix4["set"]>;

const PLAN_COLOUR = 0x38bdf8;
const AISLE_COLOUR = 0xf97316;
const MEASURE_COLOUR = 0xfacc15;
/** Overlays draw after the (transparent, depth-less) splat so they stay visible. */
const OVERLAY_ORDER = 10;

/** The Spark mesh for `input`, its file type from the name's extension:
 * bytes are decoded whole, a URL is streamed as a paged LOD file. */
export function splatMesh(SPARK: Spark, input: SplatInput): SplatMesh {
  const ext = input.fileName.toLowerCase().split(".").pop() ?? "";
  const fileType = Object.values(SPARK.SplatFileType).find((t) => t === ext);
  return "bytes" in input
    ? new SPARK.SplatMesh({ fileBytes: input.bytes, fileType, fileName: input.fileName })
    : new SPARK.SplatMesh({ url: input.url, fileType, paged: true });
}

/** The framing box of an initialized mesh in viewer coordinates (empty when
 * the mesh has no finite centre; floaters trimmed, see `splat-bounds`) and
 * whether any of its splats is solid; `"streaming"` for a streamed LoD mesh
 * the renderer has paged no splats into yet (its RAD header carries no
 * bounds), to be framed once it has. */
export function splatViewerBounds(THREE: Three, mesh: SplatMesh): { box: T.Box3; solid: boolean } | "streaming" {
  const count = mesh.splats?.getNumSplats() ?? 0;
  if (mesh.paged && count === 0) return "streaming";
  const f = splatFramingBounds((cb) => mesh.forEachSplat(cb), count);
  if (!f) return { box: new THREE.Box3(), solid: false };
  const box = new THREE.Box3(new THREE.Vector3(...f.min), new THREE.Vector3(...f.max)).applyMatrix4(mesh.matrix);
  return { box, solid: f.solid };
}

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
  /** The streamed splat still waiting for its first pages to be framed. */
  private streaming: SplatMesh | null = null;
  private collision: T.Object3D | null = null;
  private plan: T.LineSegments | null = null;
  private aisles: T.LineSegments | null = null;
  private readonly measure: T.Group;
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
    if (this.streaming) this.frameStreamed(this.streaming);
    for (const l of this.frameListeners) l();
    this.raf = requestAnimationFrame(this.loop);
  };

  /** Frame a streamed splat on the first splats the renderer paged in. */
  private frameStreamed(mesh: SplatMesh): void {
    if (mesh !== this.splat) {
      this.streaming = null;
      return;
    }
    const f = splatViewerBounds(this.THREE, mesh);
    if (f === "streaming") return;
    this.streaming = null;
    if (f.box.isEmpty()) return;
    this.bounds = f.box;
    this.frame();
  }

  /** Load the splat in `frame`; it replaces any previous splat, and the
   * camera frames the new one alone (a streamed LoD splat once its first
   * pages arrive). Resolves whether any splat in it is solid: false means an
   * empty or fully transparent model, nothing to see (true for a streamed
   * splat, and when a later load replaced this one). */
  async loadSplat(input: SplatInput, frame: ArtifactFrame): Promise<boolean> {
    const mesh = splatMesh(this.SPARK, input);
    mesh.matrixAutoUpdate = false;
    mesh.matrix.set(...(artifactToViewerMatrix(frame) as Matrix16));
    mesh.updateMatrixWorld(true);
    if (this.splat) {
      this.scene.remove(this.splat);
      this.splat.dispose();
    }
    this.splat = mesh;
    this.streaming = null;
    this.scene.add(mesh);
    await mesh.initialized;
    if (this.splat !== mesh) return true;
    const f = splatViewerBounds(this.THREE, mesh);
    if (f === "streaming") {
      this.streaming = mesh;
      return true;
    }
    this.bounds = f.box;
    this.frame();
    return f.solid;
  }

  /** Load the collision mesh (GLB, local ENU, Z up) used for picking. */
  async loadCollision(glb: ArrayBuffer, show = false): Promise<void> {
    const { GLTFLoader } = await import("three/examples/jsm/loaders/GLTFLoader.js");
    const root = (await new GLTFLoader().parseAsync(glb, "")).scene;
    root.matrixAutoUpdate = false;
    root.matrix.set(...(artifactToViewerMatrix(null) as Matrix16));
    root.traverse((o) => {
      const mesh = o as T.Mesh;
      if (!mesh.isMesh) return;
      // The loader's own materials are replaced, so free them now.
      disposeMaterials(mesh.material);
      mesh.material = new this.THREE.MeshBasicMaterial({ color: 0x94a3b8, wireframe: true, opacity: 0.25, transparent: true });
      mesh.visible = show;
    });
    root.updateMatrixWorld(true);
    disposeObject(this.collision);
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

  private lines(positions: Float32Array | null, colour: number): T.LineSegments | null {
    if (!positions || positions.length === 0) return null;
    const geom = new this.THREE.BufferGeometry();
    geom.setAttribute("position", new this.THREE.BufferAttribute(positions, 3));
    const seg = new this.THREE.LineSegments(
      geom,
      new this.THREE.LineBasicMaterial({ color: colour, depthTest: false, transparent: true }),
    );
    seg.renderOrder = OVERLAY_ORDER;
    this.scene.add(seg);
    if (this.bounds.isEmpty()) {
      geom.computeBoundingBox();
      if (geom.boundingBox) this.bounds.copy(geom.boundingBox);
      this.frame();
    }
    return seg;
  }

  /** Draw one floor's walls on the floor (null clears the plan). */
  setFloor(floor: FloorplanFloor | null): void {
    disposeObject(this.plan);
    if (floor) this.floorY = floor.zM;
    this.plan = this.lines(floor ? floorLinesViewer(floor) : null, PLAN_COLOUR);
  }

  /** Draw warehouse aisles (centrelines and edges) on the current floor. */
  setAisles(aisles: readonly Aisle[]): void {
    disposeObject(this.aisles);
    this.aisles = this.lines(aisles.length ? aisleLinesViewer(aisles, this.floorY + 0.03) : null, AISLE_COLOUR);
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
    for (const c of [...this.measure.children]) disposeObject(c);
    const overlay = { color: MEASURE_COLOUR, depthTest: false, transparent: true };
    for (const p of points) {
      const dot = new this.THREE.Mesh(new this.THREE.SphereGeometry(0.04, 12, 8), new this.THREE.MeshBasicMaterial(overlay));
      dot.position.set(...p);
      dot.renderOrder = OVERLAY_ORDER + 1;
      this.measure.add(dot);
    }
    if (points.length === 2) {
      const geom = new this.THREE.BufferGeometry().setFromPoints(points.map((p) => new this.THREE.Vector3(...p)));
      const line = new this.THREE.Line(geom, new this.THREE.LineBasicMaterial(overlay));
      line.renderOrder = OVERLAY_ORDER + 1;
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
    disposeObject(this.plan);
    disposeObject(this.aisles);
    disposeObject(this.collision);
    this.plan = this.aisles = this.collision = null;
    if (this.splat) {
      this.scene.remove(this.splat);
      this.splat.dispose();
      this.splat = null;
    }
    this.scene.remove(this.spark);
    this.spark.dispose();
    this.renderer.dispose();
  }
}
