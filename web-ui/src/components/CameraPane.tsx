import { useEffect, useRef } from "react";

interface Props {
  camId: number;
  /** Stats text block (throttled by the parent). */
  statsText: string;
  /** Register/unregister this pane's canvas with the parent rAF loop. */
  onCanvas: (camId: number, el: HTMLCanvasElement | null) => void;
}

export function CameraPane({ camId, statsText, onCanvas }: Props) {
  const ref = useRef<HTMLCanvasElement | null>(null);

  useEffect(() => {
    onCanvas(camId, ref.current);
    return () => onCanvas(camId, null);
  }, [camId, onCanvas]);

  return (
    <section className="pane" data-cam={camId}>
      <h2>cam{camId}</h2>
      <canvas ref={ref} width={640} height={480} data-cam={camId} />
      <pre className="stats">{statsText || "waiting…"}</pre>
    </section>
  );
}
