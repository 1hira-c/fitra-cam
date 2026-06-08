# core-pipeline: ランチャー常駐 + 推論子プロセスによる WebUI 主導セットアップ基盤

(起票 2026-06-08 / 状態: **計画・未着手**。着手は `vr-output/webui-vite-react` と
`pose-3d/controller-marker-extrinsic-impl` の `Develop` マージ後。
関連: [vr-output-webui-vite-react](#) (webui ブランチ上の設計 doc), [pose-3d-controller-marker-extrinsic.md](./pose-3d-controller-marker-extrinsic.md))

## 背景 / 動機

現状 fitra-cam は **全設定が起動時固定**。CLI 引数または `--config <yaml>`
(`fitra_main_config_v1`) で `MainOptions` を一度組み立てたら、推論グラフ
(TRT エンジン・FrameSource・driver・publisher・Crow) は再構築されない (`cpp/src/main.cpp:268-919`)。
校正モード (`--calibrate` / `--extrinsic-calib` / live) は単一の frame tap を奪い合うため
**起動時に排他選択**される (`main_config.cpp:533-537`)。カメラ列挙機能は無く、ユーザは
`/dev/v4l/by-path` を手で指定する。ChArUco intrinsics 校正は Python offline ツール
(`python/scripts/calibrate_intrinsics_charuco.py`) にしか存在しない。

達成目標:

- **引数無し起動 → WebUI 上でステップを踏んでセットアップ**(①カメラ選択 ②ChArUco 単カメラ校正
  ③VMT/SteamVR 連携案内・チェック ④AprilTag+コントローラのマルチカメラ校正 ⑤リアルタイム推定チェック
  ⑥IK 用 subject 校正)。
- **全設定を WebUI から実施**。設定はその場で反映 — 即時反映可能なものは純リアルタイム、
  再起動が必要なものは自動で再起動。
- **名前付き設定(bundle)の保存/読込/エクスポート/インポート**(校正成果物も同梱)。

完了の定義: zero-arg 起動で①〜⑥を WebUI 操作のみで通せ、名前付き bundle を保存/適用/可搬でき、
intrinsics を含む全校正が Python 非依存で完結すること。

## 検討した案

### プロセス構成(採用: ランチャー常駐 + 推論子プロセス)

- **採用**: 軽量な常駐ランチャーが WebUI・設定管理・ウィザード・カメラ列挙・子プロセス制御をホストし、
  重い推論本体 (`fitra-cam`) を子プロセスとして起動/停止/再起動する。zero-arg 起動はランチャーに着地。
- **没: 単一プロセス内で再構成** — 今の `fitra-cam` のまま、パイプライン段 (カメラ/モード/VR 出力) を
  実行時に停止・再構築できるよう C++ を改修。`main()` が全グラフを一度きり構築し teardown 経路を
  持たない現状から、C++ 側の改修規模が過大。モード排他 (frame tap) の本質も解けない。
- **没: systemd / ラッパースクリプトで再起動** — WebUI が config を書き、外部スーパーバイザで再起動。
  実装最小だが、自動再起動の制御・状態引き継ぎ・ヘルス監視が外部依存に散り、ウィザードの
  「子を順次別モードで起動」制御が組めない。

### ポート/プロキシモデル(採用: 単一安定ポート + ランチャーが子へリバースプロキシ)

- **採用**: ランチャーが唯一のユーザ向けポート (既定 8000) を持ち、子 (`127.0.0.1:8011` 固定) へ
  推論系 REST と WS をリバースプロキシ。ブラウザの origin が一つで不変なため、子再起動を跨いでも
  `web-ui/src/lib/config.ts` 無改修・CORS 不要・Tauri/Wails 化とも整合。子再起動中は REST が `503`
  (`transport.ts` は `{ok:false}` を許容)、WS は `useWebSocketJson.ts` の 1.5s 再接続で復帰。
- **没: 完全 2 ポート(フロントが両方に接続)** — WS プロキシ実装は不要だが、ブラウザが子ポートを
  追跡し再起動中の connection-refused を処理する必要があり、single-origin 不変条件 (webui 設計 doc) を崩す。
  → **フォールバック**として保持。
- **没: SO_REUSEPORT / fd 受け渡しで 1 ポート共有** — 再起動オーバーラップ時の接続振り分けが不安定で、
  Crow も listener-fd 注入を持たない。複雑性高・便益低。
- **ハイブリッド(レイテンシ次第で採用)**: REST は 8000 でプロキシしつつ `/ws`・`/ws3d` は
  `/api/process/status` が広告する子ポートへ直結。Jetson のレイテンシ感度を踏まえ、WS プロキシが
  許容外なら本案を採る (M0 で計測決定)。

## 採用設計

### コンポーネント所有権

- **新規軽量バイナリ `fitra-launcher`** (`cpp/src/launcher/`、ターゲット `fitra_launcher`)。
  Crow + yaml-cpp + 既存 `fitra_config` ライブラリのみリンクし、**TensorRT/CUDA/OpenCV/pipeline は
  リンクしない**(起動 1 秒未満・低 RSS で常駐する前提)。`MainOptions` のスキーマ機構
  (`load_main_config`/`apply_cli_overrides`/`validate_options`) を再利用して config 検証する。
  ランチャーが所有するもの: ①`web-ui/dist` 静的配信 ②config bundle ストア ③カメラ列挙/preview
  ④子プロセスのライフサイクル ⑤ウィザード状態機械 ⑥子へのリバースプロキシ。
- **推論子プロセス = 既存 `fitra-cam`(役割不変)**。ランチャーがモードに応じた完全 YAML を
  `calibrations/.launcher/run.yaml` に書き出し、`posix_spawn`/`fork+execv` で
  `fitra-cam --config <run.yaml> --host 127.0.0.1 --port 8011 [--<mode-flag>]` として起動
  (pid・シグナル制御のため `popen` は使わない。既存の検証経路を丸ごと再利用)。

### 子プロセスのライフサイクル & ヘルス (`ChildSupervisor`)

状態機械: `Stopped → Starting → Healthy →(Stopping → Stopped)`、異常終了で `Crashed`。

- **停止**: `SIGINT`(main.cpp は orderly shutdown を実装済 `main.cpp:264,900-913`、
  extrinsic-calib は終了時 solve)→ 猶予 (既定 10s、extrinsic は長め) 後 `SIGKILL`。
- **異常終了**: 非 0 終了は stderr tail を UI へ。校正モードは自動再起動しない。
  live のみ限定的自動再起動 (例 60s 内 3 回まで)。
- **Readiness**: `GET 127.0.0.1:8011/stats` の初回 200 で `Healthy`。加えて子に **`GET /healthz`** を
  新設し実効 config (`mode, n_cams, enable_3d, kp_format, uptime_s`) をエコー (ドリフト検出 + UI 表示)。
- **Liveness**: `SIGCHLD`/`waitpid(WNOHANG)` で即時クラッシュ検知 + 終了コード捕捉。
- 子 stdout/stderr をリングバッファへ取り込み `/api/process/logs?tail=N` で公開。

### 設定反映: ホットリロード vs 再起動の分類

ランチャーが「希望 config」と「稼働中の実効 config」を差分し、フィールド毎に判定。**変更フィールドに
1 つでも再起動必須があれば全体を再起動**(部分 in-process 変更 + 再起動の混在は避ける)。分類器は
純関数 `classify_change(running, desired) -> {hot_fields, restart_required}` として子無しで単体テスト可能にする。

| 設定 | 判定 | 根拠 / 機構 |
|---|---|---|
| `cam_paths`, `width/height/fps`, `pixel_format`, `n_buffers` | **再起動** | FrameSource は構築時 1 回 (`main.cpp:422-463`) |
| `det_engine`, `pose_engine` | **再起動** | TRT エンジンは構築時ロード |
| `keypoint_format` | **再起動** | プロセス全体トポロジが起動前に確定 (`main.cpp:295-307`) |
| `enable_3d` | **再起動** | 3D グラフ全体を構築時にゲート |
| `calib`(intrinsics/extrinsics パス) | **再起動** | Triangulator が構築時に校正から生成 |
| `multi_person`,`det_score`,`det_frequency` | **再起動**(初期) | 構築時に焼き込み。将来 hot 化可 |
| `subject_id`/`subject_profile`/`subject_height_m` | **HOT** | `ik().reload_from_profile()`/`apply_subject_height()` 既存 (`main.cpp:563,572`)。新 `/api/ik/profile` |
| `vr_one_euro`+`vr_pos_*`/`vr_quat_*`, event-driven, `vmt_pos_smooth`, `slimevr_quat_smooth` | **HOT(目標)** | `TrackerExtractor::set_options()` 新設 + `/api/extractor/tuning`。実装まで暫定再起動。最も価値の高い live チューニング |
| `kalman_3d`, `ik_3d` | **HOT(目標)** | driver に live トグル新設まで暫定再起動 |
| VMT アライメント(manual/tpose/motion/continuous) | **HOT** | `/api/vmt/*` で既に live (`crow_server.cpp:520-852`) |
| SlimeVR 補正 | **HOT** | `/api/slimevr/corrections` で既に live |
| `vmt_out`,`slimevr_out`,`hmd_listen_enabled`, ports/hosts/rates 等 | **再起動** | publisher/receiver は構築時条件生成・socket 1 回 bind |
| モード(`calibrate`/`excal`/live) | **再起動** | frame-tap 排他 = ウィザード順序機構そのもの |
| `port/host/static_dir/no_web` | **N/A** | ランチャー所有(子は固定 loopback) |

「目標」行が M6 のホットリロード化バックログ。各々が独立 1 オブジェクトへの `set_*` + 子 REST 追加で実現。

### ウィザード状態機械(ランチャー所有)

進捗は子再起動を跨いで保持する必要があるためランチャーが保持。ステップ実行は子へ委譲
(該当モードで起動し、子の既存セッション状態機械をプロキシ)。

| ステップ | id | 子モード | プロキシ先 | 成果物 |
|---|---|---|---|---|
| ①カメラ選択 | `cameras` | 子無し / preview | `/api/cameras/*`(ランチャー) | `cam_paths`, w/h/fps/pixfmt |
| ②intrinsics 校正 | `intrinsics` | **新規** `--intrinsic-calib --intrinsic-cam <i>` | `/api/incal/*`(新規) | `intrinsics.yaml`(per-cam) |
| ③VMT/SteamVR チェック | `vmt_check` | live + `--vmt-out --hmd-listen-enabled` | `/stats3d`, `/api/vmt/.../status` | なし(チェックのみ) |
| ④extrinsics 校正 | `extrinsics` | `--extrinsic-calib`(既存) | `/api/excal/*`(既存) | `extrinsics.yaml` |
| ⑤リアルタイムチェック | `live_check` | live(`--enable-3d`, intr+extr) | `/ws3d`, `/stats3d` | なし(目視確認) |
| ⑥subject IK 校正 | `subject` | `--calibrate ...`(既存) | `/api/calib/*`(既存) | `subjects/<id>/latest_profile.yaml` |

- **排他は順序で解消**: ステップ遷移ごとに `stop()`(graceful、extrinsic は終了時 solve を待つ)→
  次モード config 合成→`spawn()`→`Healthy` 待ち→当該セッション API をプロキシ。既存 `validate_options`
  の排他チェックは防御として残す。
- **進捗永続化**: `calibrations/.launcher/wizard_state.yaml`(`schema: fitra_wizard_state_v1`、
  ステップ毎 `pending|in_progress|done|failed|skipped`)。上流再実行は下流を `stale` 化して再実行を促す。
  ランチャー再起動時は `current_step` から再開(子は Stopped)。
- **②は新規 C++ モード**: Python `calibrate_intrinsics_charuco.py` を移植参照に、`IntrinsicCalibSession`
  (`cpp/src/pipeline/`、`extrinsic_calib_session.hpp` がテンプレ)+ `--intrinsic-calib`/`MainOptions`
  グループ + `validate_options` 排他ゲート + `register_intrinsic_calib_routes_()` + 静的パスを新設。
  単カメラ固有なので `--intrinsic-cam <idx>` を取り、ウィザードが選択カメラ毎にループして
  `intrinsics.yaml` へマージ。

### 名前付き設定 bundle

config + 依存校正成果物の **コピー**(参照でなく)を同梱した自己完結スナップショット。
コピー理由: intrinsics/extrinsics/subject は cameras+config と密結合で、runtime 変更が共有
`calibrations/*.yaml` の変異を黙って拾うのを防ぐ。

```
configs/bundles/<name>/
  bundle.yaml          # schema: fitra_config_bundle_v1。name/メタ/wizard provenance/artifacts index
  config.yaml          # fitra_main_config_v1(パスは bundle 相対に書換)
  calib/{intrinsics.yaml, extrinsics.yaml, cam_params.yaml}
  subjects/<id>/latest_profile.yaml   # 承認済プロファイルのみ(sessions/ は同梱しない)
  preview/cam0.jpg ...               # 任意サムネ
```

- `config.yaml` はスキーマ無変更(`three_d.calib`/`extrinsic_calib.intrinsics`/`subject.*` を
  bundle 相対に書換えるだけ)。「bundle 適用」= bundle 相対パスを絶対化して run config 合成 →
  live モードで子起動。
- `calibrations/` は子が校正中に読み書きする **作業領域**のまま。ステップ完了時にランチャーが
  成果物を bundle へコピー。bundle が durable/named/可搬な形。
- **CRUD/Export/Import**: list = `configs/bundles/*/bundle.yaml` 走査。Export = `<name>.fitrabundle`
  (`.tar.gz`)。Import = 展開 → `bundle.yaml`/各成果物/`fitra_main_config_v1` を既存ローダで検証 →
  絶対パスを bundle 相対に書換。`tar` は shell out 可(レイテンシ経路外)。
- 後方互換: bundle 無しの素の `configs/*.yaml`(flat `calibrations/` 参照)もそのまま適用可。

### 新規/プロキシ エンドポイント

**ランチャー新規**:

- カメラ: `GET /api/cameras`(`/dev/v4l/by-path/*` 列挙 + `VIDIOC_ENUM_*` で format/size/fps 能力)、
  `POST /api/cameras/preview/{start,stop}` + `GET /api/cameras/preview/<cam>.{mjpg,jpg}`。
  **preview は子が `Stopped` の時のみ許可**(カメラ占有競合回避、ランチャーが thin V4L2 で直接 open。
  TRT 非リンク)。
- config: `GET/POST /api/config/bundles`, `GET/PUT/DELETE /api/config/bundles/<name>`,
  `GET .../export`, `POST /api/config/import`(multipart), `POST /api/config/validate`。
- process: `GET /api/process/status`(`state,pid,mode,effective_config_summary,uptime_s,
  restart_count,child_ws_base`)、`POST /api/process/{start,stop,restart,apply}`、
  `GET /api/process/logs?tail=N`、ランチャー自身 `GET /healthz`。
- wizard: `GET /api/wizard/state`、`POST /api/wizard/step/<id>/{enter,complete,skip}`、
  `POST /api/wizard/reset`。

**子へプロキシ(既存・不変)**: `/stats`, `/stats3d`, WS `/ws`,`/ws3d`, `/api/calib/*`,
`/api/excal/*`, `/api/vmt/*`, `/api/slimevr/*`。

**子へ新規追加**: `GET /healthz`、`register_intrinsic_calib_routes_()`
(`/api/incal/{state,start,capture,solve,cancel}`、excal 形を踏襲)、(M6 で)
`POST /api/ik/profile`・`/api/extractor/tuning`・`/api/three_d/toggles`。

## Milestone

各 M はビルド可能・利用可能な状態を保つ。リスク先行で順序付け。

- **M0 — プロキシ de-risk(スパイク)**: Crow で HTTP + `/ws3d` の upgrade/双方向ポンプを子(loopback)から
  プロキシし、Orin Nano で許容レイテンシか計測(proxied vs direct)。**判定ゲート**: 許容なら Model 1、
  不可ならハイブリッド(REST プロキシ + WS は子ポート直結)。**最優先**。
- **M1 — ランチャー骨格 + supervisor + passthrough**: `fitra_launcher` 新設(8000 bind、`web-ui/dist`
  配信、固定 loopback で既存 `fitra-cam` を合成 run.yaml 起動、`ChildSupervisor`、`/stats` readiness、
  log リングバッファ、全既存ルート + WS プロキシ)、子 `/healthz`。利用可: zero-arg 起動で WebUI 表示、
  ハードコード config の start/stop、現行機能がランチャー経由で動作。
- **M2 — bundle ストア + apply + 分類器**: `fitra_config_bundle_v1` 構成・CRUD・`validate`・
  `classify_change`(単体テスト)。apply は当面再起動のみ。export/import(`.fitrabundle`)。利用可:
  名前付き設定の保存/読込/適用(再起動ベース)。
- **M3 — カメラ列挙 + preview + ウィザード①③⑤(可能なら④)**: V4L2 列挙、子停止時 preview、
  `WizardController` + `fitra_wizard_state_v1` 永続化、既存子モードを使うステップを配線。
- **M4 — ウィザード④(extrinsic)本配線 + ⑥(subject)**: 既存子セッション(`/api/excal/*`,
  `/api/calib/*`)を使用。順序制御・成果物の bundle コピー・stale 下流無効化が作業。利用可:
  intrinsics 以外のフルウィザード。
- **M5 — C++ ChArUco intrinsics モード + ウィザード②**: Python ツールを `IntrinsicCalibSession` へ
  移植 + `--intrinsic-calib`/`MainOptions`/`validate_options`/`register_intrinsic_calib_routes_()` +
  per-camera ループで `intrinsics.yaml` マージ。利用可: Python 依存無しのエンドツーエンド。
- **M6 — ホットリロード化(逐次・任意)**: 分類器「目標」行を真の hot 化(extractor 平滑化チューニング・
  kalman/ik トグル・IK プロファイル push)。各々独立出荷可。
- **M7 — ドキュメント + 仕上げ**: 本 doc を実装着地メモで更新、`docs/tracks/core-pipeline.md` changelog、
  main.cpp ヘルプの port/static 記述整理、`web-ui/dist` 所有移管を明記。

### 最優先で潰すべき未知数

1. **WS リバースプロキシのレイテンシ(M0)** — ポートモデル全体を左右。
2. **カメラ preview の占有**(子停止時のみで解決、ランチャー V4L2 が TRT 非依存か確認、M3)。
3. **extrinsic-calib の graceful stop 時間**(終了時 solve に数秒、supervisor 猶予で吸収、M4 検証)。
4. **C++ ChArUco の Python パリティ**(board 幾何/スケール/RMS、M5、既存 `intrinsics.yaml` 出力と照合)。

## 検証

- **M0**: `/ws3d` proxied vs direct の片道レイテンシを Orin 実機で計測、レイテンシ基準
  ([core-pipeline-e2e-latency.md](./core-pipeline-e2e-latency.md)) と比較。
- **分類器**: `classify_change` を子無し単体テスト(各フィールド変更 → hot/restart 期待値)。ctest 追加。
- **ライフサイクル**: ランチャー経由で start→stop→restart、子クラッシュ注入で `Crashed` 表示と stderr
  tail を確認。SIGINT 後の extrinsic 終了時 solve が猶予内で完了することを確認。
- **ウィザード E2E**: zero-arg 起動 → ①〜⑥ を WebUI 操作のみで通し、各ステップ成果物が bundle に
  コピーされ、再起動跨ぎで `wizard_state.yaml` から再開できることを確認。
- **bundle 可搬性**: export → 別ディレクトリ(または別 Orin)へ import → live 適用で完全再現を確認。
- **ChArUco パリティ(M5)**: 同一観測で C++ 出力 `intrinsics.yaml` の K/dist/RMS を Python ツール
  出力と照合(許容誤差内)。
- **回帰**: ランチャー経由で現行の live 推論・VMT/SlimeVR 出力・既存校正フローが従来通り動作すること。

## 残課題

- 着手は両依存ブランチ (`vr-output/webui-vite-react`, `pose-3d/controller-marker-extrinsic-impl`) の
  `Develop` マージ後。webui ブランチ上の設計 doc `docs/design/vr-output-webui-vite-react.md` を
  マージ後に相互リンクする。
- フロント側(`web-ui/`)の新規ページ(wizard / camera / config manager)の UI 詳細は本 doc では未確定
  (M3 以降で詰める)。
- M6 のホットリロード化対象は便益順に取捨選択(One Euro 平滑化が最優先候補)。
