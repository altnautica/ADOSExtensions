/**
 * @module world/scene-resources
 * @description GPU resource bookkeeping for the world viewer: every geometry
 * and material under an object is disposed exactly once when the object is
 * replaced or the viewer closes, including the materials a loader created
 * that the viewer swapped out.
 */

import type { BufferGeometry, Material, Object3D } from "three";

/** Dispose one material or a material array; returns how many were freed. */
export function disposeMaterials(material: Material | Material[] | undefined | null): number {
  if (!material) return 0;
  const list = Array.isArray(material) ? material : [material];
  for (const m of list) m.dispose();
  return list.length;
}

/** Dispose every geometry and material under `root` (each shared resource
 * once). Returns the counts freed. */
export function disposeObject(root: Object3D | null): { geometries: number; materials: number } {
  const geometries = new Set<BufferGeometry>();
  const materials = new Set<Material>();
  root?.traverse((o) => {
    const node = o as Object3D & { geometry?: BufferGeometry; material?: Material | Material[] };
    if (node.geometry) geometries.add(node.geometry);
    if (node.material) for (const m of Array.isArray(node.material) ? node.material : [node.material]) materials.add(m);
  });
  for (const g of geometries) g.dispose();
  for (const m of materials) m.dispose();
  root?.removeFromParent();
  return { geometries: geometries.size, materials: materials.size };
}
