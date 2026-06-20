# core-pipeline: Setup モード コードレビュー指摘の修正

(着手日 2026-06-20 / 親設計: [`core-pipeline-setup-mode.md`](core-pipeline-setup-mode.md) /
`/code-review xhigh` で confirmed の 15 件)

## 背景 / 動機

`core-pipeline/webui-setup-mode` の WebUI ウィザードに対するコードレビューで、ウィザードを
実運用に乗せる前に潰すべき 15 件の不具合が confirmed となった。大別すると:

1. **config 往復のデータ消失**: `emit_main_config` が `calibration:` ブロックを出力せず、save
   往復で `calib_subject_id` 等が消える。`web.static` が絶対化されず別 CWD で 404。
2. **フロー routing/validation の穴**: 新規リグで subject 校正がスキップされる、engine 未設定でも
   validate が「OK」を返す、別タブからの run 切替がカメラ無し config で run を起動して落とす、
   同一カメラを 2 スロットに割当てると run child が EBUSY で死ぬ。
3. **書き戻し汚染**: daemon の `--config` が bootstrap テンプレ兼書込先で、追跡中の
   `setup_first.yaml.example` が機械固有値で上書きされる (実際に worktree で発生していた)。
4. **Web サーバ**: 未登録 `/api/*` が SPA fallback で index.html(200) を返し frontend の JSON parse が
   例外に。`/api/setup/check-path` が 0.0.0.0 bind で任意パス存在オラクルに。
5. **カメラ backend**: V4L2 列挙が stepwise カメラでサイズ空。YUYV のバッファ padding で
   プレビューが永久 503。
6. **React フォーム/プレビュー**: 数値欄を空にすると 0 に潰れる (gain -1 sentinel/port/det_score)、
   解像度を拒否されると編集ごとに無限再 POST、アンマウントでカメラ fd が解放されない、
   pixel_format 変更で非対応解像度のまま送信可能。

完了基準 = 15 件すべて修正 + ctest 緑 + `pnpm build` 通過。

## 検討した案

- **#11 書き戻し汚染**: (a) `.example` を `git restore` + doc 修正だけ / (b) `--config` を読取専用に
  保ち派生パス (`<stem>.local.yaml`) へ書き出す 2 ファイル方式 / **(c, 採用) bootstrap + 上書き拒否**
  — `--config` が非存在なら `.example` から seed し、`write_union` は `.example` 末尾パスを拒否。
  (b) は daemon が in-process で再ロードせず各 child が forward された `--config` を読む構造のため、
  「seed 先と書込先と child の読込先」を 1 パスに保つ (c) が最小改修。`.example` を直接渡すと
  proceed 時に明確エラーを返すので汚染が構造的に起きない。
- **#2 subject 校正 routing**: 単に `profile_exists=false` にするだけでは不足。daemon の CalibSubject
  child は `--config` のみ再ロードし `--calib-subject-id` を渡さないため、id/height が空だと child が
  `validate_options` と boot preflight の両方で落ち daemon が run に fallback する。よって **routing
  修正に加え、`next==CalibSubject` のとき id/height を SubjectCalibPage 既定 (subject01/170cm) で
  seed** し、A1 の emit で永続化する。operator は subject-calib ページで id を上書き可能。
- **#4 validate 偽 OK**: target mode を導出して mode 別 run-form で検証する案も検討したが、
  `validate_draft` は config 層にあり app 層の `initial_mode` を呼べず、CalibSubject form は seed 前の
  id 空で偽 NG になる。**採用: web 層の app コールバック `on_validate` で「cam0 割当済みなら engine
  必須」を追加チェック** — #4 の核 (engine 空の偽 OK) を最小・偽 NG なしで解消。
- **#5 flow-switch**: stale な bootstrap `opts` を precheck していたのを、Setup mode では **live draft を
  write_union → draft 基準で precheck → switch** に変更 (proceed と `compose_and_switch` で共有)。
  非 setup mode は従来の captured-opts precheck を維持。
- **#10 check-path オラクル**: localhost bind 案はリモートブラウザ運用を壊すため不採用。**daemon CWD
  (リポジトリルート) 配下に限定** — 実 engine/calib は `outputs/`・`configs/` 配下なので正規用途は
  通る。static handler と同じ `weakly_canonical` + prefix 判定を流用。

## 採用設計

- **A1** `emit_main_config` に `calibration:` 節を追加。`calibrate` は run-mode 派生のため **emit しない**
  (既存「enabled/replay は emit しない」規約踏襲)。loader は既読なので load 変更不要。
- **A2** `absolutize_config_paths` に `static_dir` を追加。
- **A3** bootstrap = `main.cpp` の `load_main_config` 直前 (親 daemon プロセスで `--config` 非存在なら
  `configs/setup_first.yaml.example` を `copy_file`)。拒否 = `SetupConfigStore::write_union` 冒頭で
  `union_path_` が `.example` 末尾なら明確 err を返す。runtime 既定は `configs/session.yaml`
  (`.gitignore` の `configs/*.yaml` で ignore 済み)。
- **B1/B2/B3** `mode_setup.cpp` を `compose_and_switch` (live draft 基準で seed/precheck/write/switch)・
  `do_proceed` (auto mode 導出)・`do_switch` (要求 mode を同経路へ)・`do_validate` (緩和 + engine 必須)
  に整理。`on_validate` を `SetupRouteDeps`/`CrowServer` に追加。flow-switch handler は `run_mode_setup`
  で `do_switch` に差し替え (make_server の stale-opts 版を上書き)。
  **重要 (レビュー指摘で修正)**: cam0+engine 必須と subject id/height の seed は `next==CalibSubject`
  だけでなく **`next != Setup` の全 chain 入口** で行う。初回フローは setup→calib-intrinsic から入るため、
  CalibSubject 限定だと union config の `calib_subject_id` が空のまま intrinsic→extrinsic へ進み、
  extrinsic 子 (`has_subject_stage = !calib_subject_id.empty()`) が空判定で run へ直行し subject 校正を
  飛ばす。engine も同様で、空のまま離脱すると後段 run/subject 子が即死し UI から直せなくなる。
- **B4** frontend `assignSlot` が割当時に他スロットの同 device をクリア + `validate_options` が非空
  `cam_paths` の重複を reject。
- **C1** catch-all で `api/` 始まりパスは index.html でなく JSON 404。
- **C2** `check-path` は解決後の絶対パスが CWD 配下のときだけ existence を返す (`allowed` フィールド追加)。
  包含判定は prefix 一致でなく **完全一致 or 直後が `/`** の component 単位 (`<root>-secret` の sibling
  バイパスを塞ぐ。bot レビュー指摘)。
- **D1** `enum_frame_sizes`/`enum_frame_rates` が STEPWISE/CONTINUOUS で min/max + レンジ内標準解像度/fps を
  合成。候補は **step grid 上 (`(w-min)%step==0`) のものだけ** に限定 (driver が S_FMT で蹴る非整列解像度を
  出さない。bot レビュー指摘)。
- **追加堅牢化 (bot レビュー指摘)**: V4L2 enum の `::open` に `O_CLOEXEC` (子へ fd 漏洩→EBUSY 防止)、
  `directory_iterator` を明示 increment + error_code (例外でデーモンを落とさない)、`cap.card/driver` と
  fmtdesc description を `strnlen` 長で構築 (非終端ドライバの範囲外読み防止)、`write_union` は空 `--config`
  パスを明確 err で拒否。
- **D2** YUYV ガードを `==` → `>=` (padding を許容し先頭 `w*h*2` を decode)、短すぎる場合のみ 1 回 warn。
- **E1** `lib/format.ts` に `numOr(value, fallback)` を追加し全数値 input に適用。
- **E2** `applyPreview` が成功/失敗を問わず `appliedPreviewKey` を更新 (失敗設定の連打再 POST を停止)。
- **E3** アンマウント cleanup を state でなく `previewDevice` ref から読む。
- **E4** pixel_format 変更時に新 format の有効解像度へ snap。
- **subject identity の一元化 (レビュー後の追加修正)**: `subject.subject_id`/`subject_height_m`(run 用)と
  `calibration.calib_subject_id`/`calib_subject_height_m`(校正用)が**別物なのに重複**し、自然な
  `subject:` 配下に書いても subject 校正が走らない / `subject.calib_subject_height_m` が unknown key で
  弾かれる罠があった (daemon フローでは同一被験者なのに二重定義)。
  - **段階1 (PR #42 / c2e143c)**: ローダーに双方向ブリッジを入れ、暫定的に `subject.subject_id` 1か所で
    両方を駆動できるようにした(構造は温存=ごまかし)。
  - **段階2 (本修正)**: 構造そのものを `intrinsic_calib`/`extrinsic_calib` と同じ「1関心1ブロック」へ整理。
    `calib_subject_id`/`calib_subject_height_m` フィールドを**撤去**し、identity は `subject:` の
    `subject_id`/`subject_height_m` に一本化(run/校正/daemon すべてこれを参照)。校正プロセス調整は
    `calibration:` → **`subject_calib:`** に改名し `calib_` 接頭辞を落とした(`frames_per_cam` 等)。
    全使用箇所(daemon `profile_path`/`profile_now`/`module_argv`、calib-subject/extrinsic の
    `has_subject_stage`、validate、CLI、emit、ブリッジ)を `subject_id`/`subject_height_m` へ移行し
    ブリッジは撤去。**schema 非破壊・旧 config 互換**: 旧 `calibration:` ブロック(`calib_subject_*` 含む)は
    deprecated read alias として引き続き load 可(`subject:` が優先)。`--calib-subject-id` 等の CLI も
    `--subject-id` の alias に。emit は `subject:` + `subject_calib:` のみ(旧 `calibration:` は書かない)。

## Milestone

単一トピック (レビュー指摘の一括修正)。コミット境界の目安: A 群 (config) / B 群 (flow) / C+D 群
(web+camera) / E 群 (React) / tests+docs。

## 検証

- **ctest** 全 29 pass。`test_main_config` に追加: calibration 往復一致 (calibrate=true でも emit されない
  ことを確認)、`cam_paths` 重複 reject、`.example` への `write_union` 拒否 + `.yaml` 成功。`initial_mode`
  の `profile_exists=false→CalibSubject` は既存 `test_flow_daemon` で担保。
- **frontend** `pnpm build` (tsc strict + vite) 通過。
- **実機 (未・ユーザー実施)**: `--config configs/session.yaml` で bootstrap → ウィザードで数値欄空が 0 に
  潰れない / 同一カメラ 2 スロット不可 / 別ページ遷移でカメラ解放 / pixel_format 変更で解像度更新 /
  受けない解像度で連打再 POST なし / engine 未設定で検証 NG / 新規リグで「次へ」が subject 校正へ /
  別タブ run 切替でカメラ無し run が起動しない / `.example` 直指定で proceed 拒否 / 未登録 `/api/foo` が
  JSON 404 / `check-path?path=/etc/shadow` が拒否。

## 2巡目レビュー (2026-06-21)

PR #42 全体 (`Develop...HEAD`) に `/code-review xhigh` を再実行。新規に確認できた指摘を修正:

- **R1 (security)** 静的配信 catch-all (`crow_server.cpp`) の containment が bare-prefix
  (`canon_req.rfind(root, 0)`) で、`<root>-secret` のような兄弟ディレクトリへ脱出可能だった
  (check-path と serve_static_sub は既に境界アンカー済みなのにここだけ穴)。**採用**: 境界判定を
  `crow_util.hpp` の共有 `path_within(root, abs)` に一本化し、static handler / serve_static_sub /
  check-path の3コピーを統合。static dir 未設定 (`canon_root` 空) のときは 403 を出さず file-not-found
  経路へ通す (旧 `rfind("",0)==0` の挙動を保持、`test_crow_excal` の 404 期待に整合)。
- **R2 (routing)** ウィザードの「外部校正」ステップは方式非依存 (`MODE_FOR_STEP['extrinsic'] =
  calib-extrinsic`) なため、floor リグで手動ステップ移動すると controller 版を起動し precheck で失敗/誤起動。
  **採用 (altitude)**: frontend を方式対応にするのでなく、方式を知っている backend で正規化 —
  `do_switch` と viewer の flow-switch ハンドラ (`server_builder.cpp`) で extrinsic target を
  `excal_method` に合わせて floor/controller へ寄せる (auto-chain の `initial_mode` と一致)。
- **R3 (routing)** `do_switch` (手動ステップ移動) が `do_proceed` の `validate_draft` を通らず、
  range/enum 不正 draft を書いて子が落ち daemon が run へ fallback。**採用**: validate を
  `compose_and_switch` の冒頭に集約し両経路で実行 (`do_proceed` 側の重複呼出は除去)。
- **R4 (info-disc)** `check-path` が `deps.store` ガード外で全モードに登録され、run/calib でも
  CWD 配下の存在オラクルになっていた。**採用**: ガード内へ移し Setup モジュール専用化。
- **R5 (frontend)** `getJson` が `res.ok` を見ず body を parse するため、/api/config の JSON-404
  (R 系で意図通り返る) で `r.config` undefined → `normalizeCalibPaths` が throw し SetupPage が
  「connecting…」固着。**採用**: `loadConfig` で `!r.config` を guard し retry へ。
- **R6 (UX)** 解像度 select 変更時に fps を再 snap せず、非対応の size+fps 組合せを送信可能
  (pixel_format select は snap 済)。**採用**: pixel_format と同じ snap を解像度変更にも適用。
- **cleanup**: `clamp01(NaN)` の `width:'NaN%'` を NaN-safe 0% 化、手書き `ends_with` を C++20
  `std::string::ends_with` へ、exposure_mode の string→enum 重複を `v4l2_capture.hpp` の
  `parse_exposure_mode` に共有化 (pixfmt は preview=CPU/runtime=HW で意図的に差異があるため統合しない)、
  `useFlowSwitch` 成功枝の no-op `setPending(true)` 除去。

**見送り** (意図的設計 / 別軸 / 軽微): `compose_and_switch` の engine 必須 (intrinsic/extrinsic でも要求)
は前回承認済みの意図的設計、subject01/1.70m の seed も意図的 (subject-calib preflight で上書き) ——
ただしウィザードに subject_id/身長の入力欄が無いのは真の UX ギャップで別トピック候補。daemon の
blank-config クラッシュ→Run fallback→giveup、bootstrap の CWD 相対パス、`do_validate` が calib-stage
precheck を通さない偽 OK、`jint` の exponent/範囲、`latest_jpeg` の mutex は本巡もスコープ外。

## 残課題 (今回スコープ外・cap で落とした confirmed)

applyPreview の in-flight 競合、`floor_out` の往復消失 (`--floor-out` 経由のみ)、useFlowSwitch の
banner 未クリア、ExtrinsicCalibPage の二重 banner、POST /api/config の RMW race (単一ユーザでは
非現実的)、jint の範囲チェック。必要なら別トピックで対応。

bot レビューで指摘された **`SetupCameraManager::latest_jpeg` のグローバル `mu_` を encode 中も保持** する
contention は見送り。プレビューは 1 台ずつ (SetupPage は単一 `previewing`) なので実害は poll-encode と
start/stop の競合のみで軽微。本格対応は `streams_` を `shared_ptr<Stream>` + per-stream mutex 化して
encode を `mu_` 外に出す改修だが、別トピック化する。
