import { useEffect, useState } from "react";
import { fetchVmtPreset, postVmtPreset } from "../lib/api";

// VRChat FBT consumes at most 8 trackers. Fewer can give a more stable IK
// solve, so the published set is selectable. Changing it requires re-running
// VRChat full-body-tracking calibration.
const PRESETS: { value: string; label: string }[] = [
  { value: "p3", label: "3点: 腰 + 両足" },
  { value: "p6", label: "6点: + 胸 + 両膝" },
  { value: "p8", label: "8点: + 両肘 (VRChat標準)" },
  { value: "full", label: "10点: + 両脛" },
];

export function VmtPresetForm() {
  const [preset, setPreset] = useState<string>("p8");
  const [enabled, setEnabled] = useState(false);
  const [status, setStatus] = useState("—");

  useEffect(() => {
    let alive = true;
    void (async () => {
      try {
        const data = await fetchVmtPreset();
        if (!alive) return;
        setEnabled(!!data.enabled);
        if (data.preset) setPreset(data.preset);
      } catch {
        // VMT publisher may be disabled; leave defaults.
      }
    })();
    return () => {
      alive = false;
    };
  }, []);

  const onChange = async (value: string) => {
    setPreset(value);
    setStatus("適用中…");
    const data = await postVmtPreset(value);
    if (data.ok === false) {
      setStatus(data.err || "適用失敗");
      return;
    }
    setStatus(`適用: ${data.preset}（要VRChat再キャリブ）`);
  };

  return (
    <form className="vmt-align" aria-label="VMT tracker preset" onSubmit={(e) => e.preventDefault()}>
      <div className="vmt-align-head">
        <h3>VMT トラッカー構成</h3>
        <span className="vmt-align-status">{enabled ? "" : "(未接続)"}</span>
      </div>
      <div className="vmt-axis-row">
        <label>
          プリセット{" "}
          <select
            value={preset}
            disabled={!enabled}
            onChange={(e) => void onChange(e.target.value)}
          >
            {PRESETS.map((p) => (
              <option key={p.value} value={p.value}>
                {p.label}
              </option>
            ))}
          </select>
        </label>
        <output className="vmt-align-total">{status}</output>
      </div>
    </form>
  );
}
