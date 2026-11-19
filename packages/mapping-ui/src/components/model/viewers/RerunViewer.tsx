/**
 * @module viewers/RerunViewer
 * @description Mounts the Rerun web viewer on a world artifact (an `.rrd`
 * recording). The viewer's wasm is the extension's own asset (see
 * `rerun-start`).
 *
 * We do NOT hand Rerun the recording URL to fetch itself: the artifact is read
 * through the node's extension server with the host's authentication, which
 * Rerun's own loader cannot send. Instead we fetch the recording bytes the way
 * the other viewers do (which also gives a real download progress bar for a
 * large recording) and push them into the viewer over a log channel
 * (`open_channel().send_rrd`). A failed fetch/start surfaces an error overlay
 * rather than a silent blank.
 */

import { useEffect, useRef, useState } from "react";
import { WebViewer, type LogChannel } from "@rerun-io/web-viewer";

import { viewerT as t } from "../../../host/i18n";
import type { ArtifactSource } from "../../../lib/net/artifact-source";
import { fetchArrayBufferWithProgress, type FetchProgress } from "../../../lib/net/fetch-with-progress";
import { startRerunViewer } from "./rerun-start";
import { ViewerError } from "./ViewerError";
import { ViewerLoading } from "./ViewerLoading";


/** Poll a getter until it is true (Rerun exposes readiness as a getter, with no
 * guaranteed event), bounded so a viewer that never becomes ready gives up. */
async function waitReady(
  check: () => boolean,
  tries = 300,
  stepMs = 50,
): Promise<boolean> {
  for (let i = 0; i < tries; i++) {
    if (check()) return true;
    await new Promise((r) => setTimeout(r, stepMs));
  }
  return check();
}

/** `readAsset` reads a file the extension ships (the host's `readAsset`):
 * the viewer's wasm. */
export function RerunViewer({
  source,
  readAsset,
}: {
  source: ArtifactSource;
  readAsset: (path: string) => Promise<Blob>;
}) {
  const hostRef = useRef<HTMLDivElement>(null);
  // Serializes construct-after-stop across effect runs so a StrictMode
  // double-mount or a source change can't stack two WebViewer canvases on the host.
  const lifecycle = useRef<Promise<void>>(Promise.resolve());
  const [failed, setFailed] = useState(false);
  const [loading, setLoading] = useState(true);
  const [progress, setProgress] = useState<FetchProgress | null>(null);

  useEffect(() => {
    const host = hostRef.current;
    if (!host) return;
    setFailed(false);
    setLoading(true);
    setProgress(null);
    const abort = new AbortController();
    let cancelled = false;
    let viewer: WebViewer | null = null;
    let channel: LogChannel | null = null;

    const run = lifecycle.current.then(async () => {
      if (cancelled || !hostRef.current) return;
      try {
        // Rerun sets its parent to `position: relative` and sizes its canvas to
        // 640x360 unless told otherwise. Give it an inner box that fills the
        // absolute host, so the override cannot collapse the sized box, and
        // ask for a canvas that fills that box.
        const parent = document.createElement("div");
        parent.style.width = "100%";
        parent.style.height = "100%";
        host.replaceChildren(parent);
        const v = new WebViewer();
        viewer = v;
        // Start empty; the recording is pushed as bytes below.
        await startRerunViewer(v, parent, readAsset);
        if (cancelled) return;

        // Fetch the recording with a determinate progress bar for the large
        // download.
        const buffer = await fetchArrayBufferWithProgress(source, {
          signal: abort.signal,
          onProgress: (p) => {
            if (!cancelled) setProgress(p);
          },
        });
        if (cancelled) return;

        await waitReady(() => v.ready);
        const ch = v.open_channel("reconstruction");
        channel = ch;
        await waitReady(() => ch.ready);
        if (cancelled) return;
        ch.send_rrd(new Uint8Array(buffer));
        setLoading(false);
      } catch {
        if (!cancelled) {
          setLoading(false);
          setFailed(true);
        }
      }
    });
    lifecycle.current = run;

    return () => {
      cancelled = true;
      abort.abort();
      // Tear down only after this run settles; the next effect's start is chained
      // on it, so it waits for the teardown.
      lifecycle.current = run.then(() => {
        try {
          channel?.close();
        } catch {
          /* already gone */
        }
        try {
          viewer?.stop();
        } catch {
          /* already gone */
        }
      });
    };
    // Keyed by the source's identity, not the object: a re-render that hands
    // over an equal source must not restart the download.
  }, [source.key]);

  return (
    <div className="map:relative map:w-full map:h-full map:min-h-[320px]">
      {/* `absolute inset-0` — a definite-size box (see SplatViewer); Rerun's
          canvas needs a real height, not a collapsing percentage height. */}
      <div ref={hostRef} className="map:absolute map:inset-0" />
      {loading && !failed && (
        <ViewerLoading
          percent={progress?.percent ?? undefined}
          receivedBytes={progress?.receivedBytes}
          totalBytes={progress?.totalBytes ?? undefined}
          label={t("viewerStatus.loadingRecording")}
        />
      )}
      {failed && <ViewerError what={t("viewerStatus.what.rerun")} />}
    </div>
  );
}
