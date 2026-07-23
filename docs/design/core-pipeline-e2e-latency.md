# core-pipeline: E2E レイテンシ計測と削減 (YUYV 切替含む)

(着手日 2026-05-29 / 関連: [`research/integration-benchmark.md`](../research/integration-benchmark.md) =
ステージ別レイテンシ計測の要望元、[`research/multi-camera-ingest.md`](../research/multi-camera-ingest.md) =
MJPEG/YUYV トレードオフ)

## 背景 / 動機

throughput (aggregate 170 fps) は達成済みだが、**E2E レイテンシ (photon → VR 出力) は計測も
最適化もされていなかった**。唯一の遅延指標は `stage_ms` (capture→pose) のみで、
`integration-benchmark.md` が求めるステージ別タイムスタンプ (t_capture/t_decode/t_detect/
t_pose/t_publish) は未実装だった。

ピクセル形式 (`V4L2_PIX_FMT_MJPEG`) と V4L2 バッファ数 (4) はコードにハードコードされ、
CLI/YAML で切替できなかった。利用カメラは低解像度/低fps なら YUYV にも対応するため、
MJPEG のカメラ内エンコード遅延 + CPU `cv::imdecode` (~5–15ms) を消せる可能性があった。

**スコープ (確定)**: 単一カメラ中心・VR 出力 (SlimeVR/VMT) 優先・「計測基盤 → 形式切替実装 →
実測比較」まで。完了基準 = 各ステージのレイテンシが数値で見え、MJPEG/YUYV・sleep 除去・
VR ペーシングの効果を実機で比較できる状態。

遅延連鎖: sensor + USB + カメラ内 MJPEG エンコード → V4L2 dequeue → デコード(CPU) →
decode_loop の 2ms poll sleep + YOLOX + RTMPose prebake → central loop の 2ms poll sleep +
バッチ RTMPose(GPU 同期) → **TrackerExtractor 60Hz ホップ → Publisher 60Hz ホップ** (VR は 2 ホップ)。

## 検討した案

### 計測の持たせ方: フレーム構造体 inline TS vs seq キーの side-map
採用 = **構造体に steady_clock タイムスタンプを inline で持たせる**。latest-frame-wins で
各ホップでフレームがドロップされるため、seq キーの side-map はリーク + 掃除が必要になり没。
inline TS は生き残ったフレームに相乗りでコストほぼゼロ。`steady_clock` 固定 — `system_clock`
は NTP ジャンプで delta が壊れるため delta 計算には混ぜない。スキップしたステージは前段の
値で埋め、epoch を残さない (巨大 delta で平均が壊れるのを防ぐ)。

### YUYV vs MJPEG: 帯域トレードオフ
YUYV は非圧縮なのでデコード (entropy 復号) もカメラ内エンコード遅延も消えるが、USB 帯域を
食う (640×480@30 ≈ 18MB/s)。共有 USB2.0 で 2 台だと低解像度/低fps に強く制約される。
スコープを単一カメラに絞ったので YUYV が現実的に通る。**MJPEG をデフォルト維持**し YUYV を
opt-in にして既存デプロイを壊さない。形式は `enum class PixFmt` で持ち、ホットパスで文字列
比較しない (`main.cpp` で 1 回パース)。

### sleep 除去: condition_variable vs spin/短縮 sleep
採用 = **CV 通知**。プロデューサが slot 更新と同じロック内で `notify_one`、コンシューマは
`wait_for(timeout, predicate)`。spin は CPU を焼き、sleep 短縮は jitter を残す。lost-wakeup は
`wait_for` の述語再評価で回避 (notify をロック外に出さない)。shutdown は flag→wake→join 順で、
コンシューマの stop flag を述語に含め timeout (100ms) を belt-and-suspenders に。
初回実装は単一カメラのみで、多カメラは 2ms poll を維持していた。2026-07-21 の M5 で、全
`FrameSource` が共有する世代番号付き CV を追加し、1台/複数台ともイベント駆動へ統一した。

### 多カメラ wakeup: CV vs semaphore / atomic::wait
採用 = **condition_variable + 世代番号**。consumer は前回観測した generation を持ったまま wait し、
通知が RTMPose/3D 処理中に来ていれば次の wait は即時 return する。この順序を
`wait → 全 slot を1回 scan` とすることで lost-wakeup を防ぎつつ空 scan を除去する。

C++20 `counting_semaphore::try_acquire_for` と `atomic::wait/notify` も同一ベンチで比較したが、Jetson の
libstdc++ では待機前 spin が高くつき、consumer CPU は CV 約14ms/10秒に対し semaphore 約65ms、
atomic wait 約42msまで増えたため不採用。レイテンシだけなら僅かに短いが、CPU削減という目的に反する。

### VR ペーシング: レート引き上げ vs イベント駆動
レート引き上げ (Option A) は `apply_quat_smoothing` の EMA alpha が dt 非依存の固定値のため、
レートを倍にすると平滑が倍効きレイテンシゲインを相殺する罠がある。採用 = **イベント駆動
extractor (Option B)** を opt-in で追加。`Skeleton3DBus` の更新を CV で待ち、三角測量 1 frame
ごとに 1 平滑ステップを回す。固定 60Hz では ~30Hz の三角測量に対し 2 ステップ回って過平滑/
遅延していたのが、ソースレート同期で**レイテンシ・平滑の両方が改善**する。publisher 側 (hop2)
は SteamVR の定レート期待を尊重し `sleep_until` 据え置き。subject 退出時の stale クリアは
timeout fallback tick で維持。VR 挙動は実機検証が要るため**デフォルト off**。

## 採用設計

### M1 計測 (挙動変更なし)
- `camera::DecodedFrame` (`frame_source.hpp`) に `t_decode`/`t_detect`/`t_prebake` を追加。
  `captured_at` (=t_capture, V4L2 DQBUF 時刻) は既存。`decode_loop` の各ステージ直後でセット、
  スキップ時は前段値で埋める。
- `MultiCameraDriver::loop` (`multi_pipeline.cpp`) の既存 3 秒ローリング breakdown を拡張し、
  `cap->dec / dec->det / det->bake / bake->pose / pose->pub | cap->pub` の per-frame 平均を
  別行で出力 (central スレッド自身の poll/rtm/snap wall time とは別物として)。
- VR: `pipeline::Skeleton3DSnapshot` に `t_capture_oldest` (steady, =`min_ts`) を追加。
  `NativePublisher`/`VmtPublisher` が送信成功時に `now - t_capture_oldest` を EMA 集計し
  `e2e_capture_to_send_ms` として stats に出す (web `/stats3d` JSON に追加)。

### M2 形式 / バッファ config
- `MainOptions` に `pixel_format` ("mjpeg"|"yuyv") / `n_buffers` を追加。CLI
  (`--pixel-format`/`--n-buffers`) + YAML (`cameras.*`, `allowed` set 更新) + `validate_options`
  (format 列挙 / n_buffers>=2)。`V4l2Options` は `enum class PixFmt`。
- `v4l2_capture.cpp`: `S_FMT`/受理チェック/ログを fourcc で分岐。YUYV は `bytesused==0` 時に
  mmap 長へフォールバック。`Frame::jpeg` → `Frame::data` (format 中立名) にリネーム
  (`frame_source.cpp`/`pose_pipeline.cpp` の消費側も更新)。
- `frame_source.cpp` decode 分岐: MJPEG=`cv::imdecode`、YUYV=`cv::cvtColor(COLOR_YUV2BGR_YUYV)`。
  分岐は calib short-circuit より前 (BGR を壊さない)。web プレビューは JSON のみで生フレーム
  非依存、calib 録画は既に BGR ベースなので YUYV は透過。

### M3 sleep 除去 (単一カメラ)
- `V4l2Capture` / `FrameSource` に slot CV + `wait_pop_latest`/`wait_available`/`wake` を追加。
  `decode_loop` は `capture_->wait_pop_latest(stop_, 100ms)`、central loop は単一カメラ時に
  `sources_[0]->wait_available(stop_, 100ms)`。各 `stop()` は flag→wake→join 順。
  `try_pop_latest*` は pose_bench / multi-cam 用に残す。

### M4 VR イベント駆動 (opt-in)
- `Skeleton3DBus` に内部 `update_seq_` + CV + `wait_for_update`/`wake`。`update()` で bump+notify。
- `TrackerExtractorOptions::event_driven`。`run_loop` は event_driven 時に bus 更新を待ち、
  実測 dt (clamp [1e-3, 0.5]s) を ang-vel/freeze stats に使う。fixed 時は nominal dt のまま。
- config `three_d.vr_extract_event_driven` / `--vr-extract-event-driven` (default off)、
  `main.cpp` で `tex_opts.event_driven` に配線。

### M5 多カメラ wakeup + decoded-frame zero-copy handoff
- `FrameReadySignal`: 全 `FrameSource` の publish を世代番号付き共有 CV へ集約。central loop は
  2ms polling を廃止し、通知を待ってから全 slot を1回だけ走査。stop は flag→lock付き wake→join。
- `LatestSlot<DecodedFrame>`: size-1/drop-old を保ったまま、consumer の `out = *latest_` deep copy を
  ownership exchange に変更。consumer の前フレーム storage を producer に返し、`chw_concat` /
  `M_invs` / 必要時 BGR (`copyTo`) の capacity を定常再利用する。
- frame tap のmutex/std::function snapshotをready cameraごとではなくbatchごと1回に集約する。
- `frame_handoff_bench` と `frame_ready_bench` を追加し、カメラ/GPUなしでもhandoff CPUと通知遅延を
  再計測可能にした。`test_latest_slot` はmove-only payload、latest-wins、storage recycle、
  scan→wait race、stop wakeを回帰確認する。

## Milestone
- **M1**: 計測基盤 (TS + breakdown 拡張 + VR e2e stat)。挙動変更なし。
- **M2**: pixel_format/n_buffers config + YUYV 経路。default MJPEG。
- **M3**: 2ms poll sleep → CV (単一カメラ)。
- **M4**: イベント駆動 extractor (opt-in)。
- **M5**: 多カメラ central loop を共有通知化し、decoded-frame handoff の copy/allocation を除去。

## 検証
- ビルド: `cmake --build cpp/build -j` クリーン、ctest 9/9 pass、`--help` に新フラグ表示。
- ビルド/ctest/`--help` は実施済み (9/9 pass)。
- **実機計測 (2026-05-29)**: 単一カメラ `/dev/video0` (640×480@30)、
  `yolox_s.fp16` + `rtmpose_m.fp16`、`--no-web --bench-fake-bbox` (人なし、合成 bbox で
  pose まで毎フレーム実行)、M1+M3 込みの定常値。3 秒 breakdown の `e2e_ms` 行より。

| 指標 (定常) | MJPEG 640×480@30 | YUYV 640×480@30 | 差 |
|---|---|---|---|
| cap→dec (ms) | 6.7 | **0.95** | −5.8 |
| dec→det (ms) | 2.8 | 2.8 | — |
| det→bake (ms) | 4.1 | 4.1 | — |
| bake→pose (ms) | 8.9 | 9.2 | — |
| **cap→pub (ms)** | **22.6** | **17.1** | **−5.5** |
| recv fps | 30.0 | 30.0 | 維持 |
| stage_ms (cam0) | 22–26 | 14.0 | −約9 |

  - **YUYV の知見**: このカメラ (ELP 系 UVC) は YUYV でも 640×480@30 に対応し、単一カメラなら
    USB 帯域に余裕で 30fps を完全維持。MJPEG の CPU `cv::imdecode` (~5.8ms) がそっくり消え
    E2E が −5.5ms。低解像度/低fps を強いられるのは複数台で共有 USB2.0 帯域を超える場合のみ。
  - **M3 停止経路**: SIGINT で 0.35s 内にクリーン終了 (ハングなし)。breakdown の `poll=0.12ms`
    が central のポーリング税ゼロを示す。
  - **M4 `e2e_capture_to_send_ms`**: `ik_locked` (=被写体が映って 3D が成立) が必要なため、
    人なしの本計測では未取得。被写体ありで `--enable-3d` + カメラ 2 台 + `--vr-extract-event-driven`
    の有無で要比較 (残課題)。
  - **M5 handoff microbench (2026-07-21, Jetson)**: RTMPose-M 1人分 CHW 0.562MiB、20,000回で
    deep-copy 41.908us/frame → exchange 0.028us/frame (**約1494x**)。3人分 1.688MiB は
    140.765us/frame → 0.029us/frame (**約4838x**)。3cam×60fps×1人なら少なくとも約101MiB/sの
    memcpyと約7.5ms CPU time/sを除去 (旧pathの毎フレームheap確保分はこの比較に含めず)。
  - **M5 ready microbench (3cam×60fps模擬、1,800 frame、3回)**: 2ms poll → 共有CVで slot scan
    19,992–19,995 → 5,403、consumer CPU 16.2–17.5ms → 13.6–14.2ms (**13–22%減**)、通知→pop
    平均 1.02–1.04ms → 0.013–0.014ms、p95 1.95–1.97ms → 0.017ms。全frame処理数は1,800維持。

## 残課題
- GPU 前処理 / NVJPEG decode (migration-plan の Phase 6 残課題) — decode を CPU から剥がす。
- イベント駆動 extractor の実機チューニング後、デフォルト化を検討。
- pose-side TRT FP16 drift は本作業のスコープ外 (migration-plan 参照)。
