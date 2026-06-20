import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// All API/WS calls are same-origin paths (/ws, /ws3d, /api/*) so that the
// production build can be served verbatim by the C++ Crow server. During
// `vite dev` we proxy those prefixes to a running Crow instance so the HMR
// dev server shows real data. Point VITE_CROW at the Jetson when developing
// against the real device, e.g. VITE_CROW=http://192.168.0.42:8000.
const CROW = process.env.VITE_CROW ?? "http://localhost:8000";

export default defineConfig({
  plugins: [react()],
  // Absolute base: the SPA is served at both `/` and `/subject-calib` by Crow,
  // and assets are fetched from the dist root via the `/<path>` catch-all.
  base: "/",
  build: {
    outDir: "dist",
    // Crow's static dir IS this dist; keep it self-contained.
    assetsDir: "assets",
    sourcemap: false,
  },
  server: {
    port: 5173,
    proxy: {
      "/ws": { target: CROW, ws: true },
      "/ws3d": { target: CROW, ws: true },
      "/stats": { target: CROW },
      "/stats3d": { target: CROW },
      "/api/state": { target: CROW },
      "/api/flow": { target: CROW },
      "/api/vmt": { target: CROW },
      "/api/slimevr": { target: CROW },
      "/api/calib": { target: CROW },
      "/api/excal": { target: CROW },
      // Intrinsic + setup (cameras / config) REST live on Crow too. The
      // calibration pages and /setup are now React routes served by Vite during
      // HMR dev, so /extrinsic-calib and /intrinsic-calib are NOT proxied — only
      // their /api/* backends are.
      "/api/incal": { target: CROW },
      "/api/cameras": { target: CROW },
      "/api/config": { target: CROW },
      "/api/setup": { target: CROW },
    },
  },
});
