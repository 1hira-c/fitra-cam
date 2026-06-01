import { useCallback, useRef, useState } from "react";
import { Link } from "react-router-dom";
import { CameraPane } from "../components/CameraPane";
import { ThreeDView } from "../components/ThreeDView";
import { VmtAlignForm, type VmtAlignHandle } from "../components/VmtAlignForm";
import { VmtAutoForm } from "../components/VmtAutoForm";
import { SlimeCorrectionTable } from "../components/SlimeCorrectionTable";
import { TrackerStatsTable } from "../components/TrackerStatsTable";
import { useWebSocketJson, type WsStatus } from "../hooks/useWebSocketJson";
import { useRafLoop } from "../hooks/useRafLoop";
import { drawCamera } from "../lib/draw2d";
import { build2dStatsText, build3dStatsText, type HmdStatus } from "../lib/statsText";
import type { SkeletonViewer } from "../three/SkeletonViewer";
import type { Bundle2D, Bundle3D, CameraBundle, KpFormat, Tracker } from "../types/bundle";
import "../styles/viewer.css";

const STATS_THROTTLE_MS = 160;

function conn2dLabel(s: WsStatus): { text: string; cls: string } {
  switch (s) {
    case "open": return { text: "2D live", cls: "live" };
    case "closed": return { text: "2D disconnected — retrying", cls: "dead" };
    case "error": return { text: "2D error", cls: "dead" };
    default: return { text: "2D connecting…", cls: "" };
  }
}

function arraysEqual(a: number[], b: number[]): boolean {
  return a.length === b.length && a.every((v, i) => v === b[i]);
}

export function ViewerPage() {
  // Incoming high-frequency data lives in refs (no re-render per frame).
  const bundles2d = useRef(new Map<number, CameraBundle>());
  const kpFormat2d = useRef<KpFormat>("coco17");
  const serverSeq = useRef(0);
  const serverLastMs = useRef(0);
  const bundle3d = useRef<Bundle3D | null>(null);
  const server3dSeq = useRef(0);

  const canvases = useRef(new Map<number, HTMLCanvasElement>());
  const renderTimes = useRef(new Map<number, number[]>());
  const renderFps = useRef(new Map<number, number>());
  const viewer = useRef<SkeletonViewer | null>(null);
  const vmtAlignRef = useRef<VmtAlignHandle>(null);
  const lastStatsAt = useRef(0);

  const [camIds, setCamIds] = useState<number[]>([]);
  const [stats2d, setStats2d] = useState<Record<number, string>>({});
  const [stats3dText, setStats3dText] = useState("waiting…");
  const [hmdStatus, setHmdStatus] = useState<HmdStatus>({ text: "no hmd", cls: "" });
  const [trackers, setTrackers] = useState<Tracker[]>([]);

  const status2d = useWebSocketJson<Bundle2D>(
    "/ws",
    useCallback((b: Bundle2D) => {
      serverSeq.current = b.seq;
      serverLastMs.current = b.ts_ms;
      if (typeof b.kp_format === "string") kpFormat2d.current = b.kp_format;
      for (const cam of b.cameras || []) bundles2d.current.set(cam.id, cam);
      const ids = [...bundles2d.current.keys()].sort((x, y) => x - y);
      setCamIds((prev) => (arraysEqual(prev, ids) ? prev : ids));
    }, []),
  );

  const status3dWs = useWebSocketJson<Bundle3D>(
    "/ws3d",
    useCallback((b: Bundle3D) => {
      bundle3d.current = b;
      server3dSeq.current = b.seq || 0;
    }, []),
  );

  const onCanvas = useCallback((camId: number, el: HTMLCanvasElement | null) => {
    if (el) {
      canvases.current.set(camId, el);
      renderTimes.current.set(camId, []);
      renderFps.current.set(camId, 0);
    } else {
      canvases.current.delete(camId);
      renderTimes.current.delete(camId);
      renderFps.current.delete(camId);
    }
  }, []);

  const onViewer = useCallback((v: SkeletonViewer | null) => {
    viewer.current = v;
  }, []);

  useRafLoop(
    useCallback((now: number) => {
      for (const [id, canvas] of canvases.current) {
        drawCamera(canvas, bundles2d.current.get(id), kpFormat2d.current);
        const times = renderTimes.current.get(id) ?? [];
        times.push(now);
        while (times.length && now - times[0] > 1000) times.shift();
        renderTimes.current.set(id, times);
        renderFps.current.set(id, times.length);
      }
      viewer.current?.update(bundle3d.current);
      viewer.current?.render();

      if (now - lastStatsAt.current > STATS_THROTTLE_MS) {
        lastStatsAt.current = now;
        const s2: Record<number, string> = {};
        for (const id of canvases.current.keys()) {
          s2[id] = build2dStatsText(
            bundles2d.current.get(id),
            renderFps.current.get(id) ?? 0,
            serverLastMs.current,
            serverSeq.current,
          );
        }
        setStats2d(s2);
        const { text, hmdStatus: hs } = build3dStatsText(bundle3d.current, server3dSeq.current);
        setStats3dText(text);
        setHmdStatus(hs);
        setTrackers(bundle3d.current?.trackers ?? []);
      }
    }, []),
  );

  const conn2d = conn2dLabel(status2d);
  const enabled3d = bundle3d.current?.enabled;
  const conn3d =
    enabled3d === false
      ? { text: "3D disabled", cls: "" }
      : status3dWs === "open"
        ? { text: "3D live", cls: "live" }
        : status3dWs === "closed"
          ? { text: "3D disconnected — retrying", cls: "dead" }
          : status3dWs === "error"
            ? { text: "3D error", cls: "dead" }
            : { text: "3D connecting…", cls: "" };

  // Auto-align results feed back into the manual alignment form.
  const onAlignmentResolved = useCallback((alignment: Parameters<VmtAlignHandle["writeForm"]>[0]) => {
    vmtAlignRef.current?.writeForm(alignment);
  }, []);

  return (
    <div className="viewer-page">
      <header>
        <h1>fitra-cam RTMPose</h1>
        <div className="conn-group">
          <div className={`conn ${conn2d.cls}`.trim()}>{conn2d.text}</div>
          <div className={`conn ${conn3d.cls}`.trim()}>{conn3d.text}</div>
          <Link className="conn link" to="/subject-calib">subject calib</Link>
        </div>
      </header>

      <div className="cams">
        {camIds.map((id) => (
          <CameraPane key={id} camId={id} statsText={stats2d[id] ?? "waiting…"} onCanvas={onCanvas} />
        ))}
      </div>

      <section className="view3d">
        <ThreeDView onViewer={onViewer} />
        <pre className="stats">{stats3dText}</pre>
        <VmtAlignForm ref={vmtAlignRef} />
        <VmtAutoForm hmdStatus={hmdStatus} onAlignmentResolved={onAlignmentResolved} />
        <SlimeCorrectionTable />
        <TrackerStatsTable trackers={trackers} />
      </section>
    </div>
  );
}
