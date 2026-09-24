/**
 * @module atlas/viewers/decimate-in-worker
 * @description Runs `ply-decimate-worker` off the main thread. The worker is
 * bundled separately at build time and started from a blob URL of its source,
 * which the host page's CSP admits (`worker-src blob:`).
 */

import workerSource from "./ply-decimate-worker.ts?worker-source";
import type { DecimatedCloud } from "./decimate-cloud";
import type { PlyDecimateRequest, PlyDecimateResponse } from "./ply-decimate-worker";

export interface DecimateWorker {
  /** Parse and decimate `buffer` to `budget` points; the buffer is transferred away. */
  run(buffer: ArrayBuffer, budget: number): Promise<DecimatedCloud>;
  /** Terminate the worker and release its script URL. */
  stop(): void;
}

export function startDecimateWorker(): DecimateWorker {
  const url = URL.createObjectURL(new Blob([workerSource], { type: "text/javascript" }));
  const worker = new Worker(url, { type: "module" });
  return {
    run(buffer, budget) {
      const { promise, resolve, reject } = Promise.withResolvers<DecimatedCloud>();
      worker.onmessage = (event: MessageEvent<PlyDecimateResponse>) => {
        if (event.data.ok) resolve(event.data.cloud);
        else reject(new Error(event.data.error));
      };
      worker.onerror = () => reject(new Error("cloud worker failed"));
      const request: PlyDecimateRequest = { buffer, budget };
      worker.postMessage(request, [buffer]);
      return promise;
    },
    stop() {
      worker.terminate();
      URL.revokeObjectURL(url);
    },
  };
}
