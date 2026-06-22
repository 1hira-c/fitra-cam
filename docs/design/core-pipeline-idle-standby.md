# core-pipeline: WebUI/VMT 未接続時の待機 (idle) モード

(着手日 2026-06-18 / 関連: `vr-output-vmt-pose-relay-wire-spec.md`,
`pose-3d-flow-daemon.md`, memory `project-vmt-ip-learning-punch`)

> **ステータス: 実装済み (2026-06-22)。** M1〜M5 を `core-pipeline/idle-standby`
> ブランチで実装。仕様 (2026-06-18 起票) からの差分:
> - 消費者プレゼンスは `IdleState` (`cpp/src/app/idle_state.hpp`、header-only な
>   atomic 群) を mode_run が所有し、各コンポーネントへ raw pointer 配布
>   (`std::shared_ptr<IdleState>` でなく所有 1 箇所 + 生ポインタ。下位層には
>   `app::IdleState` 型を持ち込まず素の `const std::atomic<bool>*` を渡してレイヤ
>   依存を避けた)。
> - 評価器は専用 ~10Hz スレッド `IdleEvaluator` (`cpp/src/app/idle_evaluator.*`)。
>   ヒステリシス / VR 観測可否 / 安全既定は header-only 純関数
>   (`IdleDecision::step` / `idle_consumer_present` / `idle_vr_observable`) に分離し
>   `tools/test_idle_evaluator.cpp` で単体テスト。
> - 復帰リセットは所有スレッド内で実施 (driver: `SkeletonKalman::reset()` +
>   `has_last_3d_update_=false`、TrackerExtractor: `reset_smoothing()`)。One Euro は
>   固定レートで dt が常に nominal のため自己回復せず、明示リセットが必須だった。

## 背景 / 動機

fitra-cam は起動中、消費者 (WebUI の WS ビューア / VR 側 VMT) が誰も繋がっていなくても、
全カメラで YOLOX + RTMPose + 3D 三角測量/IK を回し続け GPU をフルに使う。Jetson Orin Nano
での常時稼働・他ユーザー配布を想定すると、「誰も見ていない時は自動で省電力に落ちる」挙動が欲しい。

**完了条件**: 消費者がゼロ (WS クライアント無し **かつ** ライブ VR ピア無し) になったら、重い
GPU 推論をスキップして待機する。消費者が戻ったら即フル稼働に復帰する (体感ラグなし)。

**確定した方針 (ユーザー判断)**:
- **待機深度 = 推論スキップのみ**。YOLOX/RTMPose/3D をスキップし、V4L2 capture・JPEG decode・
  TRT コンテキストは温いまま保持 → 復帰は atomic 反転で次フレーム即推論 (<100ms)。
- **既定 ON**。放置で自動待機、`--no-idle` で無効化。calib 系モードは対象外。

## 検討した案

| 案 | 採否 | 理由 |
|---|---|---|
| **A. in-process throttle (採用)** | ✅ | `RunMode::Run` プロセスを生かしたまま既存スレッド内で重い処理をゲート。カメラ・V4L2 バッファ・全 TRT コンテキストが常駐し、復帰は atomic 1 個の反転 (<100ms)。既存の `calib_recording_flag` (`FrameSource::Options` の `shared_ptr<atomic<bool>>`、`camera_builder.cpp:52` で設定し YOLOX+RTMPose prebake をフレーム単位スキップ済) と同型の実績ある仕組みを流用できる。 |
| B. 新 `RunMode::Idle` (flow daemon モード) | ❌ | flow daemon のモード切替はプロセス exit-code + 再起動で、TRT 再デシリアライズ + N 台カメラ再 open + NVJPEG/EGL 再初期化に数秒かかる。「即復帰」を満たせない。idle 専用 YAML も増える。 |
| C. 待機時に decode も間引き / 深い teardown (カメラ・TRT 停止) | ❌ (今回) | 省電力は最大だが復帰がサブ秒〜数秒に伸びる。本タスクは復帰速度優先と決定。GPU 推論負荷が主コストなので「推論スキップのみ」で大半削減でき、decode 間引きは将来の追加ノブ候補として残す。 |

### idle 判定の消費者シグナル

- **WS**: `crow_server.cpp` の `/ws`・`/ws3d` クライアント数 (`clients2d`/`clients3d.conns`)。
- **VR**: `HmdPoseBus::snapshot(hmd_stale_ms).have_any && !stale` をライブ VR ピアの代理にする
  (SteamVR/VMT 稼働中は HMD pose が戻ってくる。memory `project-vmt-ip-learning-punch`)。
- 没にした代替: 「VMT publisher の送信成功」は片方向 OSC で ack が無く消費者有無を判定できない。

## 採用設計

### 1. 消費者プレゼンスの共有状態

新規 `cpp/src/app/idle_state.hpp` (`fitra::app`)。`calib_recording_flag` と同じく
`std::shared_ptr<IdleState>` で各コンポーネントへ配る。

```cpp
struct IdleState {
    std::atomic<int>  ws_client_count{0};   // CrowServer onopen/onclose
    std::atomic<bool> vr_peer_live{false};  // 評価器 <- HmdPoseBus freshness
    std::atomic<bool> vr_observable{false}; // hmd_listen_enabled かどうか
    std::atomic<bool> idle{false};          // 評価器が確定させる現在状態
    bool any_consumer() const {
        return ws_client_count.load(std::memory_order_relaxed) > 0
            || vr_peer_live.load(std::memory_order_relaxed);
    }
};
```

- **Writer (WS)**: `/ws`・`/ws3d` の onopen/onclose (`crow_server.cpp:359-389`) で
  `ws_client_count` を inc/dec。`CrowServer::set_idle_state(IdleState*)` を追加。
- **Writer / 評価器 (VR + idle 確定)**: `HmdPoseBus::snapshot()` から `vr_peer_live` を導出し、
  ヒステリシス付きで `idle` を確定。`run_stats_loop` スレッド (`mode_run.cpp:87`) 相乗り、
  または小さな ~10Hz スレッド。
- **Reader**: `FrameSource::decode_loop` と `MultiCameraDriver::loop` (relaxed load)。

**ヒステリシス**: 非対称。消費者ゼロが `enter_after_s` 継続して初めて idle 突入、復帰は次ティックで
即時 (遅延なし)。既定 `enter_after_s = 10.0`、評価器 10Hz。

### 2. ゲート挿入箇所 (推論スキップのみ)

- **`FrameSource::decode_loop()`** (`frame_source.cpp:128-328`) — 削減の主役。`calib_recording`
  と同じパターンで idle を読み、YOLOX ガード (~195) を `&& !idle` 相当に拡張、bbox を空に倒す
  (~255 と同様) ことで RTMPose prebake ブロック (~259-313) も自然にスキップ。
  **JPEG decode・HW decoder・EGL・TRT は触らない** (温い)。
- **`MultiCameraDriver::loop()`** (`multi_pipeline.cpp:108-285`) — while 先頭で idle を読む。
  RTMPose バッチ (~197) は `reqs` 空で自動スキップ (or `if(!idle && !reqs.empty())`)。3D は
  `if (!idle) maybe_update_3d(...)` (~246) でスキップ。idle 中はループを `idle.tick_hz` (既定 2Hz)
  へスロットル (`pending.empty()` 分岐 ~177-190 と末尾で sleep)。

### 3. Publisher は無改修で degrade

VMT (`vmt_publisher.cpp:136-141`) / SlimeVR は `!ik_locked`・データ無しで既に `continue`。
WS `publisher_loop` はクライアント空セットで no-op。**推奨 1 点**: idle 突入時に
`enabled=true, ik_locked=false` の Skeleton3D snapshot を 1 回流し (sync-miss 経路 ~360-377 と同様)、
凍結 pose を掴み続けないクリーンな「鮮度なし」状態へ落とす。

### 4. 復帰時のジャンプ対策

idle→active 遷移で Kalman を明示リセット (`has_last_3d_update_=false`) + One Euro スムーザを
リセットし、復帰初フレームを新アンカーにする。IK ロック/ボーン長 (較正値) は**温存**。
`SkeletonKalman::reset_after_missing=30` (`kalman.hpp:48`) が既存の安全網だが belt-and-suspenders。

### 5. Config / CLI

`MainOptions` (`main_config.hpp`) に追加: `idle_enabled=true`, `idle_enter_after_s=10.0`,
`idle_tick_hz=2.0`。YAML パース + `apply_cli_overrides` (`main_config.cpp` ~394) で
`--no-idle` / `--idle-enter-after-s` / `--idle-tick-hz`、`main.cpp` help (~62/177)、
`validate_options` を更新。

### 6. 不変条件 / エッジケース

- **VMT 出力 ON だが HMD-listen OFF** (VR 戻り信号無し): VR プレゼンス観測不能 → 安全側で
  **VR 軸では idle に入れない**。評価器で `(vmt_out||slimevr_out) && !hmd_listen_enabled` なら
  present=true 固定。`vr_observable=false` を WebUI に出して理由を可視化。
- **calib モード**: idle させない → そのモードでは `IdleState` を null 共有 (null → idle 常に false)。
- **WebUI タブ開きっぱなし**: WS 接続が生きている限り「視聴中」= active (仕様どおり)。

### 7. 可観測性

`/ws3d` extra フラグメント (`make_vmt_stats_fragment` 等の隣、`crow_server.cpp:1040-1062`) と
`/stats3d` / `/api/state` に `idle` オブジェクトを追加:
`enabled, active, ws_clients, vr_observable, vr_peer_live, enter_after_s, tick_hz`。
enter/exit 遷移を 1 行ずつログ。

## Milestone

- **M1**: `IdleState` + 評価器 + CrowServer クライアント計数 + config/CLI + status フラグメント。
  ゲート未挿入で、WebUI 開閉により `/stats3d` の `active` が切り替わることを確認。
- **M2**: `MultiCameraDriver::loop` ゲート (idle 中スロットル + 3D スキップ)。
- **M3**: `FrameSource::decode_loop` ゲート (YOLOX/RTMPose スキップ) — 省電力の本体。
- **M4**: 復帰ジャンプ対策 (Kalman/One Euro リセット + idle マーカー snapshot)。
- **M5**: VR 観測不能の安全既定 + track changelog 更新。

## 検証

- **ctest**: `ctest --test-dir cpp/build` 全体 + 評価器述語の純ロジック単体テスト
  (ヒステリシス: `enter_after` 継続でのみ突入 / 即時復帰、VR 観測不能の安全既定)。
  既存 `parse_hmd_pose_packet` 系テストのスタイルに合わせる。
- **実機**: `tegrastats`/`jtop` で GPU% と `/stats3d` の pose/tri fps を見ながら
  - WebUI を閉じる → `enter_after_s` 後に idle 突入・GPU 低下、再度開く → <100ms で復帰・pose lurch 無し。
  - `vmt_hmd_pose_sender.exe` (`--hmd-listen-enabled`) を停止/再開 → VR 軸で idle 出入り。
  - calib モードでは idle に入らない / VMT-out かつ HMD-listen 無しでは idle に入らないことを確認。

## 残課題

- **decode 間引き / 深い teardown** (案 C): さらなる省電力が必要になったら `idle.tick_hz` 連動の
  decode decimation、最終的にはカメラ/TRT teardown を追加ノブとして検討 (復帰速度とのトレードオフ)。
- **WebUI 表示**: idle 状態のバッジ表示 / 手動 wake ボタンは UI 側の別作業。
- **省電力の定量**: idle 時の実測 W / GPU% を計測し、`nvpmodel` 連動の是非を評価。
