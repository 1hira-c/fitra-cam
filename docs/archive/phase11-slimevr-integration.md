# Phase 11 — SlimeVR ネイティブ Firmware UDP 連携

> **方向転換 (2026-05-21)**: VMC over OSC で 8 トラッカーを送る初版を実装したが、SlimeVR Server 上で **OSC 経由のトラッカーが連番表示にしかならず、画面上での body-part assign が極めて煩雑** であることが運用で判明。OSC は諦めて **SlimeVR Firmware UDP プロトコル (port 6969)** へ移行。トラッカー構成も 8→10 (二の腕×2/胸/腰/腿×2/脛×2/足×2) に変更。
>
> - **Bridge protocol は不採用**: 位置+回転を送れる SlimeVR の native IPC 経路 (Unix socket / Named pipe + Protobuf) は同一マシン専用で、SlimeVR Server が別 PC の Windows で動く我々の環境ではネットワーク越しに使えない。Windows 側にリレー常駐を置く案 (B 案) は [`backlog-slimevr-bridge-relay.md`](../backlog-slimevr-bridge-relay.md) に積み課題として残す。
> - **Firmware UDP は回転のみ**: 位置は SlimeVR の IK が骨格 + 回転 + HMD から再構築する。カメラ由来の絶対位置は wire に乗らない。サーバ側 `UDPPacket27Position` は実装上存在するが UDP-created tracker は `hasPosition=false` で固定されており、送っても IK には反映されないことを確認済み (SlimeVR-Server 2026-05 時点)。
> - **トラッカー命名は SensorInfo packet 15 で解決**: `TrackerPosition` enum に `LEFT_UPPER_ARM(15)/RIGHT_UPPER_ARM(16)/CHEST(4)/HIP(6)/LEFT_UPPER_LEG(7)/RIGHT_UPPER_LEG(8)/LEFT_LOWER_LEG(9)/RIGHT_LOWER_LEG(10)/LEFT_FOOT(11)/RIGHT_FOOT(12)` の 10 個が完全に乗る (骨盤は当初 WAIST(5) を指定していたが SlimeVR Server で auto-assign が走らず手動割り当てが必要だったため HIP(6) に変更。`pos = hip_center` から作っているので Hip の方が解剖学的にも整合)。MAC は hostname ハッシュから決定論的に生成し再起動を跨いで同一にする (SlimeVR 側の trackerPosition 設定が persistence される)。
> - **`--slimevr-port` のデフォルトは 6969** (旧 VMC の 39539 から変更)。

## Context

Phase 7 で多視点三角測量 + Kalman + IK による 3D skeleton 出力が安定し、Phase 8 で被験者プロファイルによるボーン長固定、Phase 9 で Halpe26 移行が揃った時点で、Phase 11 のゴールは **3D skeleton を SlimeVR Server に流し込み、SteamVR / VRChat の Full-Body Tracking (FBT) 用 tracker として使えるようにする** ことだった。これにより IMU ベースの SlimeVR トラッカーを補完する「カメラ駆動 FBT ソース」が成立する。

初版は **VMC over OSC** で 8 トラッカー (waist/chest/L/R knee/L/R elbow/L/R foot) を 60 Hz で送る実装をリリースしたが、SlimeVR Server の OSC 受信ハンドラは VMC trackers を匿名の連番デバイスとして扱うため、GUI 上で 8 つの "Tracker #1..#8" を被験者の体の部位 (左足 / 右足 / 腰 等) に手で割り当て直す必要があった。被験者を入れ替える / Jetson を再起動するたびにこれが要るので、運用上 NG。

調査で確定した方針 (2026-05-21):

- **プロトコル: SlimeVR Firmware UDP (port 6969)**。SlimeVR-Tracker-ESP firmware と同じ wire format で `[4-byte tag][8-byte sequence][payload]` フレームを送る。Handshake + SensorInfo の `TrackerPosition` 指定により、SlimeVR GUI が起動と同時に **名前付きトラッカー** として認識 (連番にならない / 手動 assign 不要)。
- **Tracker 構成: 10 本** (両二の腕 / 胸 / 腰 / 両腿 / 両脛 / 両足先)。SlimeVR の `TrackerPosition` enum に完全一致。腕は二の腕のみ (前腕はコントローラ前提)、脚は thigh / shin / foot の 3 段で詳細化することで FBT のレッグ品質を上げる。
- **位置は捨てる**: Firmware UDP は回転のみ。SlimeVR の IK が HMD + 骨格 + 回転から位置を再構築する。カメラ由来の絶対位置が活かせない代償はあるが、ローカル PC で完結する Bridge protocol (Protobuf over Unix socket / Named pipe) は別 PC Windows 構成の我々には使えないため、現実解は Firmware UDP のみ。
- **依存追加なし**: 既存方針 (JSON を `std::ostringstream` で手書き、OSC も自前で書く) に揃え、Firmware UDP の wire-format シリアライザも `cpp/src/slimevr/firmware_protocol.{hpp,cpp}` で自前実装 (~250 行)。FetchContent も apt も追加なし。
- **MAC 決定論**: SlimeVR Server は MAC アドレスを device 識別キーとして persistence する。`gethostname()` → SHA-1 先頭 6 byte (locally-administered + unicast bit set) で派生させると、同じ Jetson が再起動しても同じ MAC を出すので、ユーザが GUI 上で承認した tracker 配置がそのまま残る。
- **座標変換**: 我々の world frame (Z-up, X-right, Y-forward, meters) → SlimeVR (Y-up Unity LH)。Quaternion は `(qx, qz, -qy, -qw)` の入れ替え + 符号反転。位置は送らないので変換不要。
- **Ping reply**: SlimeVR Server は ~500 ms 周期で `tag=10` の Ping を送り、応答が無い device を disconnected 扱いにする。recv ループで Ping をデコードし、同じ `ping_id` を載せて即返送。

## ゴール / 完了条件

1. `--slimevr-out` フラグで native UDP publisher が立ち上がり、handshake + SensorInfo × 10 + RotationData 60 Hz + Heartbeat 1 Hz を `<slimevr-host>:6969` に送出。
2. SlimeVR Server GUI で **10 個のトラッカー** が `Left Upper Arm` / `Right Upper Arm` / `Chest` / `Hip` / `Left Upper Leg` / `Right Upper Leg` / `Left Lower Leg` / `Right Lower Leg` / `Left Foot` / `Right Foot` の名前で **自動表示** される (連番ならない / 手動 assign 不要)。
3. 各トラッカーの回転が、被験者の歩行 / しゃがみ / 腕の上げ下げに対応してアバター上で破綻なく追従する (静止 T ポーズで identity rotation 二乗誤差 < 5°)。
4. `--enable-3d` が無効 / IK locked = false の間は RotationData の送出が止まり、Heartbeat のみ流れる。
5. 既存 Phase 6b の 170 fps ベンチに対し、aggregate `recent_pose_fps` 低下 < 2% (publisher は別スレッドで polling、データ生成パスに介入しない)。
6. Jetson 再起動後、SlimeVR 側で前回の trackerPosition 設定が引き継がれる (MAC 決定論)。
7. Ping count が steady-state で > 0 (= 双方向通信成立)。

## アーキテクチャ

```
        ┌──── pipeline::Skeleton3DBus (既存) ────┐
        │  ::snapshot() lock-protected getter    │
        └─────────────────┬──────────────────────┘
                          │
                          ▼
   slimevr::extract_trackers(snapshot) → std::array<SlimeTracker, 10>
                          │  (world-frame pos + wxyz quat + valid)
                          ▼
   slimevr::NativePublisher (thread pair)
     ├── start(): Handshake (tag 3) → 10× SensorInfo (tag 15)
     ├── send_thread (60 Hz pacing):
     │     ├─ bus_.snapshot() → extract_trackers → apply_quat_smoothing
     │     ├─ world_quat_to_slime per tracker (Y-up Unity)
     │     ├─ encode_rotation_data × N → sendto
     │     └─ 1 Hz Heartbeat (tag 0)
     └── recv_thread:
           └─ recv → decode_ping → encode_ping_reply (mirror id)
```

`send_thread` は `CrowServer::publisher_loop` の pacing 構造を写経 (`steady_clock + sleep_until`)。データパスへの介入なし → スループット影響ゼロ。

## トラッカー定義 (10 本)

| # | TrackerRole | sensor_id | SlimeVR `TrackerPosition` (id) | Pos joint (世界フレーム) | Forward | Up |
|---|---|---|---|---|---|---|
| 0 | LeftUpperArm  | 0 | LEFT_UPPER_ARM (15)  | midpoint(l_shoulder, l_elbow) | elbow − shoulder | neck − shoulder |
| 1 | RightUpperArm | 1 | RIGHT_UPPER_ARM (16) | midpoint(r_shoulder, r_elbow) | elbow − shoulder | neck − shoulder |
| 2 | Chest         | 2 | CHEST (4)            | midpoint(neck, hip_center)    | ⊥(shoulder_axis × spine) | neck − hip_center |
| 3 | Waist         | 3 | HIP (6)              | hip_center                    | ⊥(hip_axis × spine)      | neck − hip_center |
| 4 | LeftUpperLeg  | 4 | LEFT_UPPER_LEG (7)   | midpoint(l_hip, l_knee)       | knee − hip       | hip − hip_center |
| 5 | RightUpperLeg | 5 | RIGHT_UPPER_LEG (8)  | midpoint(r_hip, r_knee)       | knee − hip       | hip − hip_center |
| 6 | LeftLowerLeg  | 6 | LEFT_LOWER_LEG (9)   | midpoint(l_knee, l_ankle)     | ankle − knee     | hip − knee |
| 7 | RightLowerLeg | 7 | RIGHT_LOWER_LEG (10) | midpoint(r_knee, r_ankle)     | ankle − knee     | hip − knee |
| 8 | LeftFoot      | 8 | LEFT_FOOT (11)       | midpoint(l_heel, l_big_toe)   | toe − heel       | world Z-up |
| 9 | RightFoot     | 9 | RIGHT_FOOT (12)      | midpoint(r_heel, r_big_toe)   | toe − heel       | world Z-up |

Halpe26 必須。COCO17 では `neck(18)` / `hip_center(19)` / `heel(24,25)` / `big_toe(20,21)` が無いので extract が `runtime_error` を投げる。

位置情報は wire に乗らないので "pos" 列はトラッカー識別とテスト検証用 (publisher は無視)。回転の forward/up は (right, up, forward) の右手系基底を Shoemake のトレースベースで wxyz quaternion に変換する。forward と up が並行 / forward が零ベクトル / 必要 joint が欠落 → `valid=false` で publisher がそのトラッカーをスキップ。

## Firmware UDP プロトコル詳細

参考: `~/Documents/refs/slimevr/SlimeVR-Tracker-ESP/src/network/{connection.cpp,packets.h}` と `~/Documents/refs/slimevr/SlimeVR-Server/server/core/src/main/java/dev/slimevr/tracking/trackers/udp/{UDPProtocolParser,UDPPacket,FirmwareConstants}.kt`。

### 共通ヘッダ (全パケット)

```
[4 bytes BE u32]  packet tag
[8 bytes BE u64]  sequence number
```

シーケンスは publisher 内で単調増加 (handshake で 0 を override 送出してから 1 起算)。サーバ側は `connection.isNextPacket(seq)` で out-of-order を弾くが、ping は seq=0 で送り返しても受け付ける (実装確認済)。

### Handshake (tag = 3)

起動時に 1 回。サーバ側は `if (buf.remaining() > 3) { ... }` で各フィールドを optional に読むので、MAC まで送れば十分:

```
[4 BE i32]   board_type     = 4 (BoardType.CUSTOM)
[4 BE i32]   imu_type       = 0 (UNKNOWN)
[4 BE i32]   mcu_type       = 0 (UNKNOWN)
[4×3 BE i32] imu_info       = 0, 0, 0  (reserved)
[4 BE i32]   protocol_ver   = 18  (現行)
[1 u8]       fw_string_len
[N u8]       firmware_string  ("fitra-cam 0.1" 等)
[6 u8]       mac_bytes
```

### SensorInfo (tag = 15)

Handshake 直後にトラッカー 10 個分まとめて送る:

```
[1 u8]  sensor_id        = 0..9
[1 u8]  sensor_status    = 1 (OK)
[1 u8]  sensor_type      = 0 (IMUType.UNKNOWN)
[2 BE u16] sensor_config = 0  (no magnetometer)
[1 u8]  rest_calib_done  = 0
[1 u8]  tracker_position = TrackerPosition value (4/6/7/8/9/10/11/12/15/16)
[1 u8]  tracker_data_type= 0 (ROTATION)
```

サーバ側は `tracker_position` を読んだ時点で GUI display name が決まる。後段の `tracker_data_type` は optional だが送っておく。

### RotationData (tag = 17)

毎周期 60 Hz × 10 sensor:

```
[1 u8]  sensor_id
[1 u8]  data_type = 1 (NORMAL)
[4 BE f32] qx
[4 BE f32] qy
[4 BE f32] qz
[4 BE f32] qw
[1 u8]  accuracy_info = 0
```

Quaternion は xyzw on wire で、SlimeVR Server が受信時に `AXES_OFFSET = Rx(-90°)` を左から掛ける。通常は `world_quat_to_slime()` で world wxyz (Z-up RH) を wire xyzw に変換し、Server 適用後の raw rotation が SlimeVR/Unity bone space になるようにする。

SlimeVR GUI の skeleton preview は raw rotation ではなく `Tracker.getRotation()` を使うため、未 reset でも `raw * mountingOrientation` が掛かる。`--slimevr-preview-no-reset` を使うと `world_quat_to_slime_no_reset_preview_adjusted()` で SlimeVR の既定 `mountingOrientation = Quaternion.SLIMEVR.FRONT` を送信側で逆算し、さらに preview 上のズレを role 別に補正する (胸/腰/足は `Rx(+90°)`、上腕は `Rx(-180°)`、大腿/脛は `Rz(180°)`)。これにより full/yaw/mounting reset 前でも preview が bone-space の向きに近づく。WebUI の `SlimeVR correction` はこの固定補正に対する一時的な上乗せで、再起動時は 0 に戻る。

### Heartbeat (tag = 0)

1 Hz でヘッダのみ送出。SlimeVR 側の per-device timeout (~500 ms) で disconnected 扱いにならないための保険。

### PingPong (tag = 10) — receive

SlimeVR Server が ~500 ms 周期で:

```
[4 BE u32] tag = 10
[8 BE u64] seq = 0   (サーバ側 writer は常に 0)
[4 BE u32] ping_id
```

を投げてくる。`recv_thread` が `decode_ping` で id を取り出し、`encode_ping_reply(ping_id)` で同じ id を載せ替えて返送 (こちらも seq=0)。

## ファイル構成

| ファイル | 役割 |
|---|---|
| `cpp/src/slimevr/firmware_protocol.{hpp,cpp}` | パケット enum / wire-format シリアライザ / 簡易 SHA-1 / MAC 派生 / 座標変換 |
| `cpp/src/slimevr/tracker_extract.{hpp,cpp}`   | Halpe26 → 10 トラッカー (世界フレーム) + slerp smoothing |
| `cpp/src/slimevr/native_publisher.{hpp,cpp}`  | UDP socket + send/recv 2 スレッド + handshake state + stats |
| `cpp/tools/test_firmware_protocol.cpp`        | wire-format byte-level golden test (handshake / sensor_info / rotation_data / ping_reply / MAC / quat 変換) |
| `cpp/tools/test_tracker_extract.cpp`          | T-pose / degeneracy / smoothing / TrackerPosition mapping |

CMake:
- `fitra_slimevr` lib に 3 つの `.cpp` (firmware_protocol / native_publisher / tracker_extract) をまとめる
- `fitra_web` は publisher の stats を `/stats3d` に splice するため `fitra_slimevr` に依存

## CLI

```
--slimevr-out             ネイティブ UDP publisher 起動 (bool, default off)
--slimevr-host ADDR       SlimeVR Server host (default 127.0.0.1; 別 PC Windows なら IP を指定)
--slimevr-port N          UDP port (default 6969 — SlimeVR firmware port)
--slimevr-rate-hz F       RotationData 送出周期 (default 60.0、最大 240)
--slimevr-quat-smooth F   per-tracker slerp alpha 0..1 (default 0.5)
--slimevr-preview-no-reset  SlimeVR GUI preview 用に既定 mountingOrientation を相殺
```

Gating (main.cpp で early-fail):

1. `--slimevr-out` ⇒ `--enable-3d` 必須
2. `--slimevr-out` ⇒ `--keypoint-format=halpe26` 必須
3. `--slimevr-out` ⇒ `--calibrate` と排他
4. port ∈ [1, 65535], rate ∈ (0, 240], smooth ∈ [0, 1]

## /stats3d スプライス

`crow_server.cpp` の `/stats3d` ハンドラは publisher が attach されている時のみ、bundle JSON の末尾 `}` を `,"slimevr":{...}}` に書き換えて返す:

```json
"slimevr": {
  "sent_handshakes":  1,
  "sent_sensor_info": 10,
  "sent_rotations":   12345,
  "sent_heartbeats":  21,
  "skipped_invalid":  3,
  "ping_count":       42,
  "last_send_ms":     1715310123456.789
}
```

`/stats` (2D bundle) は変更なし。

## マイルストーン

| M | 内容 | 完了 |
|---|---|---|
| M1 | VMC/OSC ソース削除 + CMake/CLI クリーンアップ + Phase 11 doc preamble 修正。ビルド通過、test (triangulator + 旧 tracker_extract) pass | ✅ |
| M2 | `firmware_protocol.{hpp,cpp}` + `test_firmware_protocol` (9 cases: header / handshake / sensor_info / rotation_data / ping reply / ping decode / MAC determinism / quat transform / encoder purity) | ✅ |
| M3 | `tracker_extract.{hpp,cpp}` を 10 本構成へ書き換え + `test_tracker_extract` 更新 (T-pose で 10 全 valid, missing joint → invalid, role→position mapping, smoothing) | ✅ |
| M4 | `native_publisher.{hpp,cpp}` — UDP socket + send thread + recv thread + handshake/SensorInfo 起動シーケンス + heartbeat + ping reply | ✅ |
| M5 | `main.cpp` への CLI 配線 + `crow_server.{hpp,cpp}` の `/stats3d` splice | ✅ |
| M6 | 実機 SlimeVR Server (別 PC Windows) との E2E 確認: 10 トラッカーが名前付きで表示 / avatar が破綻なく動く / fps regression < 2% | ⏳ ユーザ手動 |
| M7 | `docs/phase11-slimevr-integration.md` 全面書き換え + `cpp-migration-plan.md` 更新 + `docs/backlog-slimevr-bridge-relay.md` 新規 | ✅ |

## 検証

### 単体 (実機不要)

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build -j
ctest --test-dir cpp/build --output-on-failure
# → test_triangulator / test_firmware_protocol / test_tracker_extract が全 pass
```

### 統合 (Jetson 単体、SlimeVR 機の代わりに netcat dump)

```bash
# Terminal A — fake SlimeVR Server (receive only, dump hex):
nc -u -l 6969 | xxd

# Terminal B — Jetson 実機:
./cpp/build/main --enable-3d --keypoint-format=halpe26 \
    --slimevr-out --slimevr-host=127.0.0.1 --slimevr-port=6969 \
    --cam0 /dev/v4l/by-path/... --cam1 /dev/v4l/by-path/... \
    --calib calibrations/<id>/calibration.yaml \
    --det-engine engines/yolox.engine --pose-engine engines/rtmpose.engine
```

Terminal A 側で:
- 起動直後の handshake (~50 byte) と sensor_info × 10 (各 18 byte)
- 60 Hz の rotation_data ×~10 (各 23 byte)
- 1 Hz の 12-byte heartbeat

が観測できれば wire は OK (`nc` は ping を投げてこないので ping_count は 0 のまま)。

### E2E (Windows SlimeVR 機が必要 — M6)

1. Windows 機の IP を確認 (例 `192.168.1.50`)。SlimeVR Server を起動。
2. Jetson 側:
   ```bash
   ./cpp/build/main --enable-3d --keypoint-format=halpe26 \
       --slimevr-out --slimevr-preview-no-reset --slimevr-host=192.168.1.50 \
       --cam0 ... --cam1 ... --calib ... --det-engine ... --pose-engine ...
   ```
3. SlimeVR GUI で **10 個のトラッカーが Left Upper Arm / Right Upper Arm / Chest / Hip / Left Upper Leg / Right Upper Leg / Left Lower Leg / Right Lower Leg / Left Foot / Right Foot の名前で自動表示** されること (連番ならない、手動 assign 不要)。
4. `--slimevr-preview-no-reset` を付けた場合、SlimeVR 側で full/yaw/mounting reset を実行しなくても GUI skeleton preview が被験者の向きで動くこと。過去に SlimeVR GUI で manual mounting を変更している tracker は、SlimeVR 側 config の mounting orientation と送信側の前提がズレるので clear/reset して確認する。
5. 被験者の腕上げ / しゃがみ / 歩行で各トラッカーの回転が破綻なく追従し、SteamVR Avatar の FBT が成立すること。
6. `curl http://<jetson-ip>:8000/stats3d` の `"slimevr"` ブロックで:
   - `sent_handshakes` = 1
   - `sent_sensor_info` = 10
   - `sent_rotations` が steady-state で 60×10 = 600/s 程度に増加
   - `ping_count` > 0 (双方向通信成立)
   - aggregate `recent_pose_fps` 低下 < 2%
6. Jetson を再起動して再接続 → SlimeVR 側のトラッカー配置が前回のまま保持されることを確認 (MAC 決定論)。

## リスク・残課題

| ID | 内容 | 対応 |
|---|---|---|
| R1 | quat 合成の forward/up vector が短い / 並行に近い | `quat_from_forward_up` が `valid=false` + identity を返す → publisher がそのフレームをスキップ。前周期の smoothing バッファが残るので avatar はフリーズせず、最後の有効値を保持 |
| R2 | SlimeVR 側 IK が回転だけでは想定外の体型に解いてしまう | M6 で実機目視。被験者の身長を SlimeVR 側でも合わせる (HMD 高さ調整) ことが前提 |
| R3 | Server からの ping に応答しない / 遅延 | recv ループの SO_RCVTIMEO=250ms。stop 中は shutdown(SHUT_RDWR) でブロックを解除 |
| R4 | Jetson 内で gethostname() が変わると MAC が変わって SlimeVR 設定が無効化 | hostname を固定して運用。Docker コンテナ起動時は `--hostname fitra-jetson` を渡す |
| R5 | 位置情報を VR 側で活用したい用途 | Bridge protocol over network のリレー実装が必要。[`backlog-slimevr-bridge-relay.md`](../backlog-slimevr-bridge-relay.md) に積み課題として記録 |

## 関連ドキュメント

- [`backlog-slimevr-bridge-relay.md`](../backlog-slimevr-bridge-relay.md) — 位置を含めて送りたい場合の Bridge protocol relay 構想 (B 案)
- [`cpp-migration-plan.md`](../cpp-migration-plan.md) — 全体ロードマップと Phase 11 検証行
- [`phase9-halpe26-migration.md`](phase9-halpe26-migration.md) — 前提となる Halpe26 移行
