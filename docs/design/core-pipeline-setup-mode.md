# core-pipeline: WebUI 主導セットアップ (RunMode::Setup daemon モジュール)

(着手 2026-06-20 / 状態: **M1–M7 実装済み**。実装ブランチ `core-pipeline/webui-setup-mode`。
前提: [pose-3d-flow-daemon.md](pose-3d-flow-daemon.md) (flow daemon)、
[vr-output-webui-vite-react.md](vr-output-webui-vite-react.md) (React SPA) が `Develop` 済み。
本 doc は未マージの旧設計 `core-pipeline-launcher-daemon.md`
(ブランチ `core-pipeline/launcher-setup-design` のみに存在) を **supersede** する。)

## 背景 / 動機

「初回セットアップから実推論まで」をほぼ全部ブラウザから回したい。残っていた gap:

1. **カメラ選択 UI が無い** — `cameras.cam0/1/2` (paths/解像度/fps/pixfmt)、engine パス、
   出力先 (VMT host:port) はいまだ `session.yaml` 手編集。`/api/cameras` も preview も無い。
2. **bootstrap の鶏卵** — flow daemon は portless で、WebUI は推論モジュール内にしか無い。
   config が無いと何も起動できず、初回は SSH で YAML を書く必要があった。
3. **名前付き config 管理が無い** (save/load/list)。
4. **UI 分裂** — intrinsic/extrinsic は vanilla-JS (`web/`)、viewer/subject は React (`web-ui/`)。

旧 `launcher-daemon` 設計はこれを **別 launcher プロセス + reverse-proxy** で解こうとしたが、
その後 `Develop` は **portless flow daemon** を採用し reverse-proxy を明示的に却下した
(flow-daemon doc 参照)。本設計は flow daemon の「モード=モジュール」パターンを再利用して
同じ目標を達成する。

達成目標: cameras 未設定の config で `./main --daemon` を 1 回起動すれば、ブラウザのみで
setup (カメラ/config 合成) → intrinsic → extrinsic → subject → run まで到達できる。

## 検討した案

### セットアップ基盤のアーキ (採用: Setup を daemon モジュール化)

| 案 | 内容 | 判定 |
|---|---|---|
| **採用: `RunMode::Setup` モジュール** | daemon が最初に spawn する軽量モジュール。Crow + V4L2 列挙/preview のみ構築 (TRT/CUDA/3D/publisher 不使用)。config を書き出し exit code で次段へ連鎖 | flow daemon を丸ごと再利用。reverse-proxy 不要。新バイナリ不要 (同一 `./main`)。起動が軽い (CUDA 初期化しない) |
| 没: 別 launcher プロセス + reverse-proxy | 旧 launcher-daemon 設計。常駐 launcher が子 fitra-cam を proxy | `Develop` が flow-daemon で reverse-proxy を却下済み。Crow に HTTP/WS proxy が無く工数・障害点大 |
| 没: 単一プロセス内再構成 | 実行時にパイプライン段を停止・再構築 | main() の teardown 経路が無く C++ 改修過大 (flow-daemon と同じ理由で却下済み) |

### config の永続と次段への受け渡し (採用: union YAML 書き出し + flow exit code)

Setup モジュールは合成した config を **daemon が起動時に渡した `--config` パスへ書き戻す**。
proceed すると flow exit code (85 で setup、または次段の 84/82/...) を返し、daemon が次の子を
spawn する。**子は `--config` を再ロードする**ので、Setup が書いた cameras 入り config を読む
(daemon の in-memory opts は stale でも問題ない)。reverse-proxy も IPC も不要。

### config シリアライザ (新規 emit_main_config)

`main_config.cpp` には loader しか無かった。`emit_main_config`/`save_main_config` を新設し、
loader の各キーと 1:1 で往復させる。要点:

- **非デフォルト値のみ emit** (ファイルを汚さない)。
- 否定キー `no_3d_kalman`/`no_3d_ik` は positive predicate の反転を emit。
- `vmt:` は bare な `host`/`port`/... キー (loader 規約)。
- **run-mode 派生フラグ (`calibration.calibrate`/`extrinsic_calib.enabled`/`*.replay_dir`) と
  launch-form フラグ (daemon/flow_managed/setup — YAML キー無し) は emit しない** → 書いた config は
  常に daemon が食える union config。
- atomic 書き込み (tmp + rename)。ctest `emit_load_round_trip` で全フィールド往復一致を固定。

### パス解決 (採用: 保存時に絶対化 + その場 existence チェック)

エンジン (`det_engine`/`pose_engine`) や calib 成果物は実行時に **CWD 相対**で `std::ifstream`
で開かれる (基準ディレクトリの付与なし、`trt_engine.cpp`)。daemon は各モード子を **daemon の
CWD のまま** `execvp` するので、相対パスは「daemon を起動したディレクトリ」基準。これが
config ファイルの場所・リポジトリルート・バイナリ位置のいずれとも違い事故りやすい
(静的 UI だけは `paths.cpp` がバイナリ位置基準で別系統)。対策:

- **保存時に絶対化**: `absolutize_config_paths(MainOptions&)` が path フィールド
  (engines / calib / intrinsic_out / extrinsic out / floor_* / subjects_dir / subject_profile) を
  `std::filesystem::absolute`(CWD 基準) で絶対化。Setup モジュールは daemon と CWD を共有するので、
  ここで絶対化した先 = 後段の run/calib 子が相対解決する先と一致する。`SetupConfigStore::write_union`
  と `save_named` で適用 (出力パスも対象。存在不要。絶対パスは不変)。WebUI 入力は相対/絶対どちらでも可。
- **その場 existence チェック**: `GET /api/setup/check-path?path=` が CWD 基準で絶対化 +
  `exists`/`is_file` を返す。SetupPage の `PathField` がエンジン/calib 入力をデバウンス(500ms)で
  チェックし ✓/✗ と絶対パスをインライン表示。ブラウザは Jetson の FS を直接見られないため
  バックエンド経由。

### 設定編集サーフェス (採用: 編集サブセットの JSON、全量は YAML)

draft は完全な `MainOptions`。`/api/config` の GET/POST はウィザードが触る **サブセット**だけを
JSON で出し入れし、それ以外の seed 値は draft に保持され write_union で全量 YAML に書かれる。
JSON⇄MainOptions マッピングは web 層 (`crow_routes_setup_mode.cpp`) に置き、`fitra_config` を
Crow から切り離す (TensorRT/CUDA 非依存を維持)。

**外部校正の method 別フィールド**も編集サブセットに含む: controller は `intrinsics`、floor は
`floor_map`(タグ配置・floor 必須) / `floor_intrinsics` / `floor_fisheye`。SetupPage は method に
応じて出し分け、path は `PathField` で存在チェック。これが無いと floor を選んでも
`precheck_mode_switch(CalibExtrinsicFloor)` が floor_map を要求して proceed が止まる。

**out/calib の鶏卵問題と 2-file モデル**: `extrinsic_calib.out`(外部校正の出力) と
`three_d.calib`(run が読む) は daemon 規約で一致必須。さらに外部校正の intrinsics 入力が
未設定だと `precheck_mode_switch` は `three_d.calib` に fallback するが、それは「まだ存在しない
外部校正の出力」なので初回に `floor PnP intrinsics ... not found: calibrations/extrinsics.yaml`
で落ちる(循環)。対策として SetupPage は **2 ファイルモデル**で提示する:
- **intrinsics ファイル** = `intrinsic_calib.out`(内部校正の出力) かつ外部校正の入力。1 つの入力欄が
  `intrinsic_calib.out` / `excal_intrinsics` / `floor_intrinsics` を同時に駆動。
- **extrinsics ファイル** = `extrinsic_calib.out` = `three_d.calib`(外部校正の出力 = run の calib)。
  1 つの入力欄が両方を駆動。

`normalizeCalibPaths` がロード時に不変条件を確立し(intrinsics 系を一致、extrinsics 系を一致、
intrinsics 入力が空なら intrinsic 出力で埋める)、編集中も coupled setter が両者を同期する。
これで「外部校正の入力 intrinsics が、まだ無い外部校正の出力を指す」循環が起きない。
バックエンドの fallback 挙動 (pose-3d トラック) は変更せず、UI 層で解消している。

**カメラ別オーバーライド**は `cameras.overrides[]` (長さ 3、index=スロット cam0/1/2) として
JSON に乗せる。各要素は `{capture_width, capture_height, pixel_format, exposure_mode, exposure,
gain, ae_target}` で、MainOptions の `cam{N}_*` 配列・YAML の `cam{N}_capture_width` 等と 1:1。
未設定センチネル (capture 0 / pixel_format・exposure_mode "" / gain -1) はグローバルまたは
カメラ既定を意味し、emit_main_config が非デフォルトのみ書くので未設定スロットは YAML に出ない。
用途: 低解像でセンタークロップする USB3.0 カメラの capture 解像度上書き
([per-camera-capture-downscale](core-pipeline-per-camera-capture-downscale.md))、nvjpeg/mjpeg の
デコード経路混在、露出固定によるブレ低減
([camera-exposure-control](core-pipeline-camera-exposure-control.md))。

### カメラ preview (採用: 単発 JPEG スナップショットのポーリング)

Crow は handler から `multipart/x-mixed-replace` を綺麗にストリームできない (body を 1 回 flush)。
そこで preview は `GET /api/cameras/preview.jpg` の単発スナップショットをブラウザが `<img>` で
~150ms ポーリングする方式。MJPEG はペイロードを verbatim 返却 (再エンコード無)、YUYV のみ
OpenCV で BGR→JPEG。背景スレッド無し (Crow worker が pull + cache)。

### トップページからの導線 (採用: ビューワも flow を追従)

ブラウザで `/` (ViewerPage) を開いたとき、daemon が run 以外 (setup / 各 calib) なら
その step ページへ自動遷移する。これにより「トップを開く → そのままウィザードを進める」が
成立する。実装は他の step ページと同じ `useFlowWatch`(redirect 既定 ON) を ViewerPage にも
適用しただけ (旧来 ViewerPage だけ redirect:false で、setup モードで `/` を開くと
"connecting…" のまま行き止まりだった)。run モードでは `/` に留まる。さらに ViewerPage も
`WizardLayout` でラップし、run 中も step バーで進捗表示 + 任意 step への再校正 (flow-switch) が
できる (旧「recalibrate extrinsic/subject」ボタンと calib モード時のリンク/バナーは step バーに
集約して撤去)。calib 中のカメラ監視は各 calib ページ自身のライブ表示で代替。

### calib UI の統合 (採用: 全部 React + 静的 dir を web-ui/dist に向ける)

intrinsic/extrinsic ページを React SPA へ移植。バックエンドは
`guess_{extrinsic,intrinsic}_calib_static_dir()` を `web-ui/dist` に向け、`/extrinsic-calib`・
`/intrinsic-calib` ページ route が SPA index を返す (subject-calib と同じ)。`/api/excal/*`・
`/api/incal/*` は不変。さらに Crow 静的 catch-all に **SPA history-fallback** (拡張子の無い
未知パス → index.html) を入れ、deep-link/再読込で React Router が解決できるようにした。
legacy `web/{extrinsic,intrinsic}_calibration` は配信されなくなり superseded (削除可)。

## 採用設計

```
ブラウザ ── :8000 ──▶ Setup module (./main --setup --flow-managed --config X)
                          │ /api/cameras /api/cameras/preview.jpg /api/config* /api/setup/proceed
                          │ config 書き出し → exit code (85=setup / 84=intrinsic / 82=extrinsic / ...)
                          ▲ fork/exec + waitpid (exit code = 次モード)
                      daemon (./main --daemon --config X、ソケット/CUDA/TRT なし)
                          │ initial_mode: cam_paths[0] 空 → Setup
                          ▼
                      intrinsic → extrinsic → subject → run (既存モジュール無改変、子が config 再ロード)
```

### 不変条件

- daemon/setup 系フラグ (`--daemon`/`--setup`/`--flow-managed`/`--daemon-initial`) は **CLI 専権、
  YAML キーを作らない** (union YAML をどのモードにも食わせられる性質を保つ)。
- `validate_options`: Setup は port のみ検査して早期 return (cam/engine/3D 不要)。run-form 必須
  チェックは `!opts.daemon` でゲート — daemon 親は空 config を許容し、検証は各子 + initial_mode の
  precheck に委ねる (cameras 未設定の初回でも daemon が起動できる)。
- Setup モジュールは TRT/CUDA/3D/publisher を一切構築しない (起動が軽い、GPU を触らない)。
- proceed は cameras 未設定なら拒否 (次段が Setup に戻るのを防ぐ)。

### 新規エンドポイント (Setup モードのみ登録)

- `GET /api/cameras` — `/dev/v4l/by-path/*` を列挙 (fourcc/解像度/fps)。
- `POST /api/cameras/preview/{start,stop}`、`GET /api/cameras/preview.jpg?cam=`。
- `GET/POST /api/config`、`POST /api/config/validate`、`GET /api/config/list`、
  `POST /api/config/{save,load}`、`POST /api/setup/proceed`。

### exit code (flow.hpp)

`kExitFlowToSetup = 85` を追加。`flow_exit_code(Setup)`、`module_argv(Setup)→--setup`、
`next_action` の 85 ケース、`initial_mode` の「cam_paths[0] 空 → Setup」。

## Milestone (各 = 1 コミット)

- **M1** `feat(core-pipeline)`: `RunMode::Setup` + daemon 配線 + validate 緩和 + `mode_setup` runner
  (Crow のみ) + ctest。
- **M2** `feat(core-pipeline)`: `emit/save_main_config` (往復 ctest) + `SetupConfigStore` +
  `/api/config*` + `/api/setup/proceed` + register_setup_mode_routes。
- **M3** `feat(core-pipeline)`: `v4l2_enumerate` + `setup_camera_manager` (JPEG preview) +
  `/api/cameras*`。
- **M4** `feat(core-pipeline)`: frontend 足場 (`FlowMode` に setup、`lib/wizard.ts`、
  `WizardSteps`/`WizardLayout`、`useFlowSwitch`) + vite proxy + C++ SPA history-fallback。
- **M5** `feat(vr-output)`: intrinsic/extrinsic を React 移植 (`/api/incal*`・`/api/excal*` 消費)。
- **M6** `feat(vr-output)`: Setup ページ (カメラ選択/preview/config/名前付き保存/proceed)。
- **M7** `feat(*)`: legacy calib 静的配信を `web-ui/dist` へ retire + SubjectCalibPage を
  WizardLayout 化 + 本 doc + track changelog + flow-daemon exit-code 表更新。

## 検証

- **ctest** 全 29 pass。`test_main_config`: `--setup`→Setup・空 config の validate 非 throw・
  `emit→load` 往復一致 (否定キー/bare host port 含む)。`test_flow_daemon`: `module_argv(Setup)`・
  `next_action(85)→Setup`・`initial_mode` 空 cam→Setup・explicit override 優先。
- **backend smoke**: `--daemon --config setup_first` が setup に着地 → `/api/state`=setup →
  REST で cameras/engines/halpe26/enable_3d を merge → validate ok → proceed が session.yaml に
  cameras を書き出し exit 84 (calib-intrinsic) で連鎖。3 カメラリグで `/api/cameras` が
  ELP×2 (90fps) + USB3.0 (120fps) を列挙、preview.jpg が有効 JPEG をライブ更新。
- **frontend**: `pnpm build` (tsc strict + vite) 通過。SPA history-fallback で /setup・
  /intrinsic-calib が index.html を 200 で返し /assets/missing.js は 404。
- **実機 一気通し (未・ユーザー実施)**: 3 カメラ + SteamVR で SSH 1 回
  `./main --daemon --config configs/session.yaml` (非存在なら `setup_first.yaml.example` から
  自動 bootstrap、gitignore 済み) → ブラウザのみで setup → intrinsic → extrinsic → subject → run、
  各 calib ページが旧 vanilla と機能 parity であることを確認。`.example` を直接 `--config` に渡すと
  proceed が「テンプレートは上書き不可」で明確に拒否 (詳細は `core-pipeline-setup-mode-fixes.md`)。

## 残課題

- **extrinsic 3D scene** (`web/extrinsic_calibration/scene.js`) の React 移植は未 (テーブル UI のみ
  移植済み)。`/api/excal/{extrinsics,poses}` を消費する `components/ExtrinsicScene.tsx` を tab として
  追加するのが backlog。
- ViewerPage への WizardSteps 表示は見送り (既存の switch ボタン nav があるため)。
- legacy `web/{extrinsic,intrinsic}_calibration` dir は配信されなくなったが残置 (実機 parity 確認後に削除)。
- engine ビルド自体は WebUI 外 (Python/TRT)。Setup は engine パスを受け取るだけ。
- daemon の systemd 常駐化 (初回 SSH ゼロ化) は別軸 (今回スコープ外、vr-output の運用整備と合流)。
- named config の export/import (`.fitrabundle`) は見送り (旧 launcher 設計の bundle 機構は未採用)。
