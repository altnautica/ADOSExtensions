/**
 * @module capture/CameraTile
 * @description One capture camera's live preview over WHEP (WebRTC receive
 * only), negotiated through the device's capture surface so the request
 * carries the host's authentication. A preview that cannot start says so;
 * it never shows a frozen or stand-in frame.
 */

import { useEffect, useRef, useState } from "react";
import { VideoOff } from "lucide-react";

import { translator } from "../../host/i18n";
import type { CaptureClient } from "../../lib/agent/capture-client";
import type { CameraFrames } from "../../lib/capture/capture-state";

const t = translator("capture");

/** How long to wait for ICE candidates before sending the offer anyway. */
const ICE_GATHER_MS = 1500;

function gathered(pc: RTCPeerConnection): Promise<void> {
  if (pc.iceGatheringState === "complete") return Promise.resolve();
  const { promise, resolve } = Promise.withResolvers<void>();
  const check = () => pc.iceGatheringState === "complete" && resolve();
  pc.addEventListener("icegatheringstatechange", check);
  setTimeout(resolve, ICE_GATHER_MS);
  return promise.finally(() => pc.removeEventListener("icegatheringstatechange", check));
}

export function CameraTile({ client, camera }: { client: CaptureClient; camera: CameraFrames }) {
  const videoRef = useRef<HTMLVideoElement>(null);
  const [status, setStatus] = useState<"connecting" | "live" | "failed">("connecting");

  useEffect(() => {
    if (typeof RTCPeerConnection === "undefined") {
      setStatus("failed");
      return;
    }
    let closed = false;
    let location: string | null = null;
    const pc = new RTCPeerConnection();
    pc.addTransceiver("video", { direction: "recvonly" });
    pc.ontrack = (e) => {
      const video = videoRef.current;
      if (!video || closed) return;
      video.srcObject = e.streams[0] ?? new MediaStream([e.track]);
      void video.play().catch(() => undefined);
      setStatus("live");
    };
    pc.onconnectionstatechange = () => {
      if (pc.connectionState === "failed") setStatus("failed");
    };
    setStatus("connecting");
    void (async () => {
      try {
        await pc.setLocalDescription(await pc.createOffer());
        await gathered(pc);
        const sdp = pc.localDescription?.sdp;
        if (!sdp || closed) return;
        const answer = await client.whepOffer(camera.id, sdp);
        if (closed) {
          // Unmounted while the offer was in flight: the device opened a
          // session nobody will read, so close it now.
          if (answer?.location) void client.whepClose(answer.location);
          return;
        }
        if (!answer) {
          setStatus("failed");
          return;
        }
        location = answer.location;
        await pc.setRemoteDescription({ type: "answer", sdp: answer.answer });
      } catch {
        if (!closed) setStatus("failed");
      }
    })();
    return () => {
      closed = true;
      pc.close();
      if (location) void client.whepClose(location);
    };
  }, [client, camera.id]);

  return (
    <figure className="we:relative we:aspect-video we:rounded we:bg-media we:overflow-hidden">
      <video ref={videoRef} muted playsInline className="we:w-full we:h-full we:object-contain" />
      {status !== "live" && (
        <div className="we:absolute we:inset-0 we:flex we:flex-col we:items-center we:justify-center we:gap-1 we:text-on-media/70">
          {status === "failed" && <VideoOff size={16} />}
          <span className="we:text-[11px]">{status === "failed" ? t("previewUnavailable") : t("previewConnecting")}</span>
        </div>
      )}
      <figcaption className="we:absolute we:left-1 we:top-1 we:rounded we:bg-scrim/60 we:px-1.5 we:py-0.5 we:text-[10px] we:font-mono we:text-on-media">
        {camera.id}
        {camera.fps !== null && ` · ${camera.fps.toFixed(0)} fps`}
        {camera.dropped !== null && camera.dropped > 0 && ` · ${t("dropped", { n: camera.dropped })}`}
      </figcaption>
    </figure>
  );
}
