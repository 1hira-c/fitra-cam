import { useEffect, useRef, useState } from "react";
import { SkeletonViewer, type ViewName } from "../three/SkeletonViewer";

const VIEWS: ViewName[] = ["front", "side", "top"];

interface Props {
  /** Receives the viewer instance so the parent rAF loop can drive update()/render(). */
  onViewer: (v: SkeletonViewer | null) => void;
}

export function ThreeDView({ onViewer }: Props) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const statusRef = useRef<HTMLDivElement | null>(null);
  const viewerRef = useRef<SkeletonViewer | null>(null);
  const [view, setView] = useState<ViewName>("front");
  const [showTrackers, setShowTrackers] = useState(true);
  const [showCameras, setShowCameras] = useState(true);
  const [showHmd, setShowHmd] = useState(true);

  useEffect(() => {
    if (!canvasRef.current) return;
    const viewer = new SkeletonViewer(canvasRef.current, statusRef.current);
    viewerRef.current = viewer;
    onViewer(viewer);
    return () => {
      onViewer(null);
      viewer.dispose();
      viewerRef.current = null;
    };
    // onViewer is stable (useCallback in parent); run once on mount.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const pickView = (v: ViewName) => {
    setView(v);
    viewerRef.current?.setView(v);
  };

  const toggleTrackers = (checked: boolean) => {
    setShowTrackers(checked);
    viewerRef.current?.setTrackersVisible(checked);
  };

  const toggleCameras = (checked: boolean) => {
    setShowCameras(checked);
    viewerRef.current?.setCamerasVisible(checked);
  };

  const toggleHmd = (checked: boolean) => {
    setShowHmd(checked);
    viewerRef.current?.setHmdVisible(checked);
  };

  return (
    <>
      <div className="view3d-head">
        <h2>3D</h2>
        <div className="view3d-tabs" role="group" aria-label="3D view">
          {VIEWS.map((v) => (
            <button
              key={v}
              type="button"
              className={v === view ? "active" : undefined}
              onClick={() => pickView(v)}
            >
              {v}
            </button>
          ))}
        </div>
        <label className="view3d-toggle">
          <input
            type="checkbox"
            checked={showTrackers}
            onChange={(e) => toggleTrackers(e.target.checked)}
          />
          <span>show trackers</span>
        </label>
        <label className="view3d-toggle">
          <input
            type="checkbox"
            checked={showCameras}
            onChange={(e) => toggleCameras(e.target.checked)}
          />
          <span>show cameras</span>
        </label>
        <label className="view3d-toggle">
          <input
            type="checkbox"
            checked={showHmd}
            onChange={(e) => toggleHmd(e.target.checked)}
          />
          <span>show hmd</span>
        </label>
      </div>
      <div className="canvas3d-wrap">
        <canvas id="canvas3d" ref={canvasRef} width={640} height={480} />
        <div className="canvas-status" ref={statusRef}>
          (no 3D data)
        </div>
      </div>
    </>
  );
}
