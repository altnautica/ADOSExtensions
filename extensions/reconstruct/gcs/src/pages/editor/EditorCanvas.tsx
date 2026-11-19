/**
 * @module editor/EditorCanvas
 * @description The editor's canvas: the shown points of the visible layers,
 * in the product's frame (z up), seen from above (orthographic, the screen is
 * the ground plane) or in 3D, with the selection, the polygon being drawn,
 * a patch's source and placed boxes and the measurement picks drawn over it.
 * Pointer input becomes ground-plane geometry in top view; a measurement
 * pick snaps to the nearest shown point under the pointer. Renders on demand
 * (a camera move, a resize, a layer change), never on a loop.
 */

import { useEffect, useRef, useState, type Dispatch, type PointerEvent } from "react";
import type * as Three from "three";
import type { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

import type { EditorCloud } from "./editor-cloud";
import type { Action, EditorState } from "./editor-state";
import { place, rectPolygon, type Xy, type Xyz } from "./geometry";

/** Screen distance (px) a pick snaps within. */
const PICK_RADIUS_PX = 10;

interface Scene {
  three: typeof Three;
  renderer: Three.WebGLRenderer;
  scene: Three.Scene;
  geometry: Three.BufferGeometry;
  top: Three.OrthographicCamera;
  persp: Three.PerspectiveCamera;
  controls: OrbitControls;
  render: () => void;
}

function visibleIndex(cloud: EditorCloud, hidden: readonly number[]): Uint32Array {
  const out = new Uint32Array(cloud.layers.length);
  let n = 0;
  for (let i = 0; i < cloud.layers.length; i++) if (!hidden.includes(cloud.layers[i]!)) out[n++] = i;
  return out.slice(0, n);
}

export function EditorCanvas({
  cloud,
  state,
  dispatch,
}: {
  cloud: EditorCloud;
  state: EditorState;
  dispatch: Dispatch<Action>;
}) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const sceneRef = useRef<Scene | null>(null);
  // Bumped on every render of the scene, so the overlay re-projects with the
  // camera.
  const [, setFrame] = useState(0);
  const [ready, setReady] = useState(false);
  const [drag, setDrag] = useState<[Xy, Xy] | null>(null);

  // One renderer per cloud.
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    let disposed = false;
    let observer: ResizeObserver | null = null;
    void (async () => {
      const THREE = await import("three");
      const { OrbitControls: Orbit } = await import("three/examples/jsm/controls/OrbitControls.js");
      if (disposed) return;
      const w = canvas.clientWidth || 640;
      const h = canvas.clientHeight || 400;
      const renderer = new THREE.WebGLRenderer({ canvas, antialias: false });
      renderer.setSize(w, h, false);
      const scene = new THREE.Scene();
      scene.background = new THREE.Color(0x0a0a0a);
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute("position", new THREE.BufferAttribute(cloud.positions, 3));
      if (cloud.colors) geometry.setAttribute("color", new THREE.BufferAttribute(cloud.colors, 3));
      geometry.computeBoundingSphere();
      const material = new THREE.PointsMaterial({
        size: 2,
        sizeAttenuation: false,
        vertexColors: !!cloud.colors,
        color: cloud.colors ? 0xffffff : 0x88ccff,
      });
      scene.add(new THREE.Points(geometry, material));
      const bs = geometry.boundingSphere!;
      const r = Math.max(bs.radius, 0.5);
      const top = new THREE.OrthographicCamera(-r * (w / h), r * (w / h), r, -r, -r * 20, r * 20);
      top.position.set(bs.center.x, bs.center.y, bs.center.z + r * 4);
      top.up.set(0, 1, 0);
      top.lookAt(bs.center);
      const persp = new THREE.PerspectiveCamera(55, w / h, r / 1000, r * 40);
      persp.up.set(0, 0, 1);
      persp.position.set(bs.center.x - r * 1.4, bs.center.y - r * 1.4, bs.center.z + r * 1.2);
      const controls = new Orbit(top, canvas);
      // The controls aim the camera at their target on every update, the
      // first one included: aim them at the cloud.
      controls.target.copy(bs.center);
      controls.update();
      const s: Scene = {
        three: THREE,
        renderer,
        scene,
        geometry,
        top,
        persp,
        controls,
        render: () => {
          renderer.render(scene, controls.object as Three.Camera);
          setFrame((f) => f + 1);
        },
      };
      controls.addEventListener("change", s.render);
      sceneRef.current = s;
      observer = new ResizeObserver(() => {
        const cw = canvas.clientWidth;
        const ch = canvas.clientHeight;
        if (!cw || !ch) return;
        renderer.setSize(cw, ch, false);
        const half = (top.top - top.bottom) / 2;
        top.left = -half * (cw / ch);
        top.right = half * (cw / ch);
        top.updateProjectionMatrix();
        persp.aspect = cw / ch;
        persp.updateProjectionMatrix();
        s.render();
      });
      observer.observe(canvas);
      setReady(true);
      s.render();
    })();
    return () => {
      disposed = true;
      setReady(false);
      observer?.disconnect();
      const s = sceneRef.current;
      sceneRef.current = null;
      if (s) {
        s.controls.dispose();
        s.geometry.dispose();
        s.renderer.dispose();
      }
    };
  }, [cloud]);

  // Visible layers.
  useEffect(() => {
    const s = sceneRef.current;
    if (!s) return;
    s.geometry.setIndex(new s.three.BufferAttribute(visibleIndex(cloud, state.hidden), 1));
    s.render();
  }, [cloud, state.hidden, ready]);

  // View and which mouse button pans.
  useEffect(() => {
    const s = sceneRef.current;
    if (!s) return;
    const camera = state.view === "top" ? s.top : s.persp;
    if (s.controls.object !== camera) {
      s.controls.object = camera;
      s.controls.update();
    }
    const { MOUSE } = s.three;
    const drawing = state.mode !== "select" && state.mode !== "measure";
    s.controls.enableRotate = state.view === "3d";
    s.controls.mouseButtons = {
      LEFT: drawing ? (-1 as Three.MOUSE) : state.view === "top" ? MOUSE.PAN : MOUSE.ROTATE,
      MIDDLE: MOUSE.DOLLY,
      RIGHT: MOUSE.PAN,
    };
    s.render();
  }, [state.view, state.mode, ready]);

  const s = sceneRef.current;
  const size = () => {
    const c = canvasRef.current;
    return [c?.clientWidth || 1, c?.clientHeight || 1] as const;
  };
  const toScreen = (p: Xyz): Xy => {
    if (!s) return [0, 0];
    const v = new s.three.Vector3(...p).project(s.controls.object as Three.Camera);
    const [w, h] = size();
    return [((v.x + 1) / 2) * w, ((1 - v.y) / 2) * h];
  };
  const ground = (e: PointerEvent): Xy | null => {
    if (!s || state.view !== "top" || !canvasRef.current) return null;
    const rect = canvasRef.current.getBoundingClientRect();
    const v = new s.three.Vector3(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
      0,
    ).unproject(s.top);
    return [v.x, v.y];
  };
  const nearest = (e: PointerEvent): Xyz | null => {
    if (!s || !canvasRef.current) return null;
    const rect = canvasRef.current.getBoundingClientRect();
    const [mx, my] = [e.clientX - rect.left, e.clientY - rect.top];
    const cam = s.controls.object as Three.Camera;
    const v = new s.three.Vector3();
    let best = -1;
    let bestD = PICK_RADIUS_PX * PICK_RADIUS_PX;
    const p = cloud.positions;
    for (let i = 0; i < cloud.layers.length; i++) {
      if (state.hidden.includes(cloud.layers[i]!)) continue;
      v.set(p[i * 3]!, p[i * 3 + 1]!, p[i * 3 + 2]!).project(cam);
      if (v.z < -1 || v.z > 1) continue;
      const dx = ((v.x + 1) / 2) * rect.width - mx;
      const dy = ((1 - v.y) / 2) * rect.height - my;
      const d = dx * dx + dy * dy;
      if (d < bestD) {
        bestD = d;
        best = i;
      }
    }
    return best < 0 ? null : [p[best * 3]!, p[best * 3 + 1]!, p[best * 3 + 2]!];
  };

  const onDown = (e: PointerEvent) => {
    if (e.button !== 0) return;
    const g = ground(e);
    if (state.mode === "box" || (state.mode === "patch" && !state.patch?.box)) {
      if (g) setDrag([g, g]);
    } else if (state.mode === "polygon" && g) {
      dispatch({ type: "vertex", at: g });
    } else if (state.mode === "patch" && state.patch?.box && g) {
      const [a, b] = state.patch.box;
      dispatch({ type: "patchPlace", translate: [g[0] - (a[0] + b[0]) / 2, g[1] - (a[1] + b[1]) / 2] });
    } else if (state.mode === "measure") {
      const at = nearest(e);
      if (at) dispatch({ type: "pick", at });
      else dispatch({ type: "refuse", message: "Pick a point of a visible layer." });
    }
  };
  const onMove = (e: PointerEvent) => {
    if (!drag) return;
    const g = ground(e);
    if (g) setDrag([drag[0], g]);
  };
  const onUp = () => {
    if (!drag) return;
    const [a, b] = drag;
    setDrag(null);
    if (Math.abs(a[0] - b[0]) < 1e-6 || Math.abs(a[1] - b[1]) < 1e-6) return;
    if (state.mode === "box") dispatch({ type: "select", polygon: rectPolygon(a, b) });
    else dispatch({ type: "patchBox", box: [a, b] });
  };

  const zMid = (cloud.zRange[0] + cloud.zRange[1]) / 2;
  const path = (poly: readonly Xy[], close: boolean) =>
    poly.map((q, i) => `${i ? "L" : "M"}${toScreen([q[0], q[1], zMid]).join(",")}`).join(" ") + (close ? " Z" : "");
  const patch = state.patch;
  const placed =
    patch?.box &&
    rectPolygon(patch.box[0], patch.box[1]).map((c) => place(patch.box!, patch.rotateDeg, patch.translate, c));

  return (
    <div className="rec:relative rec:h-full rec:min-h-[360px] rec:w-full">
      <canvas ref={canvasRef} className="rec:h-full rec:w-full" />
      <svg
        className="rec:absolute rec:inset-0 rec:h-full rec:w-full"
        style={{ cursor: state.mode === "select" ? "grab" : "crosshair", pointerEvents: "none" }}
        aria-hidden="true"
      >
        {state.selection && (
          <path d={path(state.selection, true)} fill="rgba(56,189,248,0.12)" stroke="rgb(56,189,248)" strokeWidth={1.5} />
        )}
        {state.draft.length > 0 && <path d={path(state.draft, false)} fill="none" stroke="rgb(251,191,36)" strokeWidth={1.5} />}
        {drag && <path d={path(rectPolygon(drag[0], drag[1]), true)} fill="none" stroke="rgb(251,191,36)" strokeDasharray="4 3" />}
        {patch?.box && <path d={path(rectPolygon(patch.box[0], patch.box[1]), true)} fill="none" stroke="rgb(167,139,250)" />}
        {placed && <path d={path(placed, true)} fill="rgba(167,139,250,0.12)" stroke="rgb(167,139,250)" strokeDasharray="4 3" />}
        {state.picks.map((p, i) => {
          const [x, y] = toScreen(p);
          return <circle key={i} cx={x} cy={y} r={4} fill="none" stroke="rgb(52,211,153)" strokeWidth={2} />;
        })}
      </svg>
      <div
        className="rec:absolute rec:inset-0"
        style={{ pointerEvents: state.mode === "select" ? "none" : "auto" }}
        onPointerDown={onDown}
        onPointerMove={onMove}
        onPointerUp={onUp}
        onDoubleClick={() => state.mode === "polygon" && dispatch({ type: "close" })}
        data-testid="editor-input"
      />
    </div>
  );
}
