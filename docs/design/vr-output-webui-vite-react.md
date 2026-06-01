# vr-output: WebUI を Vite/React (TypeScript) へ移行

(着手日 2026-06-01 / 旧フロント = `web/dual_rtmpose`・`web/subject_calibration`)

## 背景 / 動機

WebUI はバニラ JS/Canvas/Three.js の静的ファイル群で、ビルドツールも型もなかった:

- `web/dual_rtmpose/`（2D/3D ビューア, Three.js, VMT alignment, SlimeVR correction, trackers stats）= `app.js` 1450 行。
- `web/subject_calibration/`（IK プロファイル取得ウィザード, REST ポーリング）。
- 両者とも C++ Crow サーバ (:8000) が静的配信（`/` と `/subject-calib`）。

問題: WS/3D バンドルの複雑なスキーマを型なしで手動パース、Three.js を手動 vendoring、HMR がなく開発が遅い、コンポーネント再利用なし。

**最終目標**（今回は実装しないが設計をこれに合わせる）: ブラウザ WebUI と Windows 側 VMT Manager
（`refs/VirtualMotionTracker/vmt_manager.sln`、C# WPF。OpenVR client + OSC + Room Matrix + 登録ゲート + HMD pose 中継）を、
**単一の Tauri or Wails デスクトップアプリ (Windows)** に統合し、Jetson バックエンドとネットワーク越しに通信する。

完了条件: Vite/React/TS の単一 SPA に viewer + subject-calib を移植し、(1) HMR 開発時に proxy 経由で実データ表示、
(2) 本番は dist を Crow が配信、(3) WS/REST スキーマ不変、(4) デスクトップ統合に備えて接続先を絶対 URL に集約。

対象外: `web/calibration/`（測定外部キャリブ :8010 / ChArUco :8020, Python FastAPI）。将来 C++ 再実装予定のため今回触らない。

## 検討した案

### A. ページ構成: MPA (エントリ分割) vs **SPA + React Router** ← 採用
- **MPA 没**: viewer.html / subject-calib.html を別エントリにすると、最終目標の「単一アプリに VMT Manager も束ねる」と相性が悪い。
  さらに shared な `assets/` をサブパス配信 (`/subject-calib`) で解決させるのが面倒（相対 base だとアセット解決が崩れる）。
- **SPA 採用**: 1 つの `index.html` + `BrowserRouter`。Crow は `/` と `/subject-calib` の両方で**同じ dist/index.html** を返し、
  React Router が `location.pathname` でページを出し分ける。将来 `/vmt-manager` route を足すだけで拡張できる。
  `base: '/'`（絶対）にしてアセットは `/assets/*` 絶対参照 → dist ルートの `/<path>` catch-all が配る。

### B. 接続先: 同一オリジン直叩き vs **config で絶対 URL 抽象化** ← 採用
- **直叩き 没**: 現行の `new WebSocket(location...)` / `fetch("/api/...")` のままだと、Crow が同一ホストで配信する前提に縛られる。
  デスクトップアプリではフロントはアプリバンドル (`tauri://`) から読まれ Crow からは配信されない → 同一オリジン前提が破綻。
- **config 採用**: `lib/config.ts` の `httpUrl()` / `wsUrl()` に**全 URL を集約**。解決優先度
  `localStorage['fitra.apiBase']`（ランタイム）> `VITE_API_BASE`（ビルド時）> `""`（同一オリジン）。
  空なら現行と同じ相対挙動（Crow 配信・dev proxy）、絶対指定すれば別ホスト/デスクトップから Jetson `http://<ip>:8000` に向く。
  ここが将来 Tauri/Wails の IPC 差し替えの**唯一の接合点**。

### C. リアルタイム描画: React state vs **ref + rAF** ← 採用
- 30Hz の WS バンドルを毎フレーム React state に入れると再レンダ過剰。受信データは ref に保持し、
  Canvas(2D)/Three.js(3D) は `requestAnimationFrame` で命令的に更新（現行 `renderTick`/`drawCamera`/`ThreeDViewer` を踏襲）。
  統計テーブル等の DOM だけ ~6Hz にスロットルした React state で更新。状態管理ライブラリは導入せず hooks + ref で完結。

## 採用設計

```
web-ui/  (Vite + React + TS, web/ と並置, dist は .gitignore)
  vite.config.ts  base:'/' / build.outDir=dist / server.proxy(→ Crow :8000, ws:true)
  src/
    main.tsx          BrowserRouter: "/"→ViewerPage, "/subject-calib"→SubjectCalibPage, "*"→/
    lib/config.ts     ★httpUrl()/wsUrl() = 接続先の単一の真実
    lib/transport.ts  getJson/postJson/openWs（config 経由のみが fetch/WebSocket に触れる）
    lib/api.ts        /api/vmt・/api/slimevr・/api/calib の型付きラッパ
    lib/skeleton.ts   SKELETON_COCO17/HALPE26, KP_THR, TRACKER_ROLES, colors
    lib/draw2d.ts     drawCamera()（2D overlay）
    lib/statsText.ts  build2dStatsText/build3dStatsText（旧テキストレイアウト保持）
    three/SkeletonViewer.ts  旧 ThreeDViewer を TS 化（kp_format は instance 保持, dispose() 追加）
    hooks/useWebSocketJson.ts  自動再接続 + 5s ping + 1.5s backoff
    hooks/useRafLoop.ts        ref'd callback を rAF 駆動
    hooks/usePolling.ts        subject-calib の 200ms ポーリング
    routes/ViewerPage.tsx      ref に WS データ保持 → rAF で 2D 描画 + viewer.update/render + ~6Hz で stats state 更新
    routes/SubjectCalibPage.tsx 状態機械をポーリング結果から純関数的に描画
    components/  CameraPane / ThreeDView / VmtAlignForm(imperative writeForm) / VmtAutoForm / SlimeCorrectionTable / TrackerStatsTable
```

不変条件/要点:
- WS/REST スキーマは `types/bundle.ts` に固定（フィールド名は backend `snapshot.cpp` と一致, リネーム禁止）。
- VMT auto-align の結果は `VmtAlignForm` の `writeForm()`（`useImperativeHandle`）経由で手動フォームに反映（旧 `writeVmtAlignmentForm`）。
- Crow 側変更は最小: `guess_static_dir()`/`guess_subject_calib_static_dir()` を共に `<repo>/web-ui/dist` に向けるのみ
  （`crow_server.cpp` のルートは無改修。`/subject-calib` は `calib_root/index.html` を読むので dist/index.html を返す）。
- Python 2D フォールバック (`dual_rtmpose_web.py`) も `WEB_DIR` を `web-ui/dist` に（3D 無しでも graceful degrade）。

## 将来計画（今回スコープ外・設計だけ通す）
- **Tauri vs Wails の選定**は別途。フロントは純 web のまま据え置き、`lib/config.ts`+`lib/transport.ts` がシェル統合の seam。
- **VMT Manager 統合**: C# WPF の OpenVR/OSC/Room Matrix/登録ゲートを Tauri(Rust)/Wails(Go) ホストに移植し IPC で React に公開。
  `/vmt-manager` route を追加。今回は route slot のみ。後続トラックで設計する。
- `web/calibration/`（Python キャリブ）も将来 C++ 化に合わせて本 SPA に取り込む候補。

## Milestone（コミット境界）
- M1: web-ui scaffold（Vite/React/TS, BrowserRouter, proxy, lib/config・transport・api・skeleton, types/bundle）+ 本ドキュメント。
- M2: ViewerPage 2D/3D 描画（CameraPane, SkeletonViewer, WS フック, rAF）。
- M3: VMT/SlimeVR 制御 UI（VmtAlignForm, VmtAutoForm, SlimeCorrectionTable, TrackerStatsTable）。
- M4: SubjectCalibPage。
- M5: Crow static_dir を web-ui/dist に + Python フォールバック追従 + .gitignore。
- M6: 旧 `web/{dual_rtmpose,subject_calibration}` 削除 + 本トラック changelog 確定。

## 検証
- **dev (HMR + 実データ)**: Crow(:8000) 起動 → `cd web-ui && pnpm install && pnpm dev` → `http://localhost:5173/`(viewer) /
  `/subject-calib` で 2D/3D 実データ描画、`/ws`・`/ws3d` が proxy 経由で接続（WS 昇格）、編集で HMR が効く。
  別マシン実機は `VITE_CROW=http://<jetson-ip>:8000`。
- **別ホスト/デスクトップ想定**: `localStorage['fitra.apiBase']='http://<jetson-ip>:8000'` を設定し、proxy 無しでも実 Jetson に接続できる。
- **prod (dist 配信)**: `pnpm build` → Crow 再起動し `/` と `/subject-calib` が同一 SPA を配信、全機能
  （VMT alignment 適用 / SlimeVR correction / calib state 遷移）が動作。
- ビルド: `cd web-ui && pnpm build`（tsc -b + vite build）が通る / `cmake --build cpp/build -j` がパス差し替え後も通る。
- スキーマ互換: 旧 UI と同一バンドルで 2D/3D 描画・stats 値が一致（目視 + 主要数値突合）。
