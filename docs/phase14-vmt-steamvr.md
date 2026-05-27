# Phase 14 — Virtual Motion Tracker (VMT) 経由 SteamVR 直結

> **Phase 12 没経緯の構造的解消**: 2026-05-22 起票の Bridge relay 経路は **(a) SlimeVR Server `\\.\pipe\SlimeVRInput` の Named Pipe を SteamVR が排他占有して接続不能 (FBT 運用は SteamVR + SlimeVR 同時起動が前提)** + **(b) SlimeVR avatar 上で位置と回転の整合が運用上安定しない** という 2 つの理由で 2026-05-23 に没にした (`archive/botsu-phase12-bridge-relay` 凍結)。Phase 14 では SlimeVR Server を完全に経由しない **Virtual Motion Tracker (VMT)** ― SteamVR Driver として直接登録される OSC ベースの仮想トラッカードライバ ― 経由で同じ目的 (絶対座標 + 回転を VR に流す) を達成する。
>
> - **Named Pipe 問題**: VMT は UDP `/VMT/Room/Driver` で受け取るので Named Pipe を一切使わない → (a) 構造的に解消
> - **座標系問題**: VMT の `/VMT/Room/Driver` は SteamVR Driver Y-up RH 座標系を直接指定し、Room Matrix calibration を VMT Manager で 1 回測れば確定 → (b) 構造的に解消 (中間レイヤなし)
> - **VRChat FBT は 10-point 構成**: Phase 11 と同じ 10 trackers (LeftUpperArm / RightUpperArm / Chest / Hip / LeftUpperLeg / RightUpperLeg / LeftLowerLeg / RightLowerLeg / LeftFoot / RightFoot) を流す。VRChat 標準 FBT role に乗らない LowerLeg は SteamVR Manage Trackers で未割当のままにし、Knee と機能重複させない運用

## Context

Phase 11 で Firmware UDP 経路 (回転のみ 10 trackers, port 6969) が安定し、Phase 13 で per-tracker degeneracy gate + confidence-modulated smoothing + WebUI viz + rolling stats まで品質が詰め切れた。残課題は **絶対位置を VR 空間に流すこと**。Phase 12 でこれを SlimeVR Server 経由 (Bridge relay) でやろうとして失敗したので、Phase 14 では **SteamVR Driver 側へ直接** 流す経路に切り替える。

VMT (Virtual Motion Tracker, gpsnmeajp/VirtualMotionTracker, v0.15) は SteamVR Driver として `vrpathreg --install` で登録され、Windows 上で SteamVR が起動すると同時に立ち上がる。受信は UDP port 39570 で、`/VMT/Room/Driver` OSC アドレスに `(index:i, enable:i, timeoffset:f, x:f, y:f, z:f, qx:f, qy:f, qz:f, qw:f)` を投げると、`vmt_<index>` という仮想トラッカーとして SteamVR Settings → Manage Trackers に出現する。Room Matrix (12 パラメータの affine) を Manager GUI で取れば、Jetson カメラ座標 → SteamVR playspace の変換が永続化される。

調査で確定した方針 (2026-05-25):

- **プロトコル: VMT OSC over UDP (`/VMT/Room/Driver`, port 39570)**。OSC 1.0 wire-format シリアライザは Phase 11 で commit `a64becf` に書いたものを github 履歴から復元 (Phase 11 撤去は `14ec5d4`)。namespace を `fitra::vmt` に再配置
- **Tracker 構成: 10 本** (Phase 11 と同じ TrackerRole enum 順)。既定は `vmt_index = 10 + static_cast<int>(TrackerRole)` で **VMT_10..VMT_19**。VMT/SteamVR の `VMT_0..VMT_2` は HMD/左右手の TrackingOverrides や過去の controller 登録と衝突しやすいため避ける。互換性が必要なら `--vmt-index-base=0` で従来の 0..9 に戻せる
- **位置 + 回転 両方** wire に乗せる。Phase 11 までは `SlimeTracker.pos` が wire に乗らなかったので、Phase 14 で初めて使う
- **座標変換**: world (Z-up RH, X-right, Y-forward) → VMT Driver (Y-up RH, X-right, Z-back)。Phase 12 Bridge transform (`world_pos_to_slime` / `world_quat_to_bridge_slime`) と完全同型 (Bridge も SteamVR の `TrackerYaw.kt` が固定する frame に揃えていた)。archive ブランチの関数本体 4 行 + 7 行を `cpp/src/vmt/vmt_protocol.cpp` にコピー
- **位置 smoothing**: Phase 11 (回転のみ) では未着手だった位置 EMA を `TrackerExtractor` に追加 (`apply_pos_smoothing`)。Phase 13 「state を一箇所で持つ」原則を保ち、`prev_quat_` の隣で `prev_pos_` を持つ。confidence modulation は採用しない (pos のノイズ源は roll の observability と独立)
- **degeneracy 時の挙動**: default `--vmt-degeneracy-mode=hold` (`enable=1` で前周期 pos/quat 保持)。CLI で `disable` / `skip` に切り替え可
- **Room Matrix calibration は Manager GUI 任せ**: CLI から `/VMT/SetRoomMatrix` を打つ機能は Phase 14 では入れない (Phase 15 候補)
- **HMD あり前提**: Quest / Index 等の実 HMD で Manager の Room Matrix calibration を取る運用。HMD なし (Null HMD Driver) は Phase 15 候補
- **依存追加なし**: OSC writer は自前 (既存方針)。`cpp/src/vmt/` 配下に 3 .cpp + 3 test
- **Phase 11/13 と独立にトグル可能**: `--slimevr-out` と `--vmt-out` は両方同時 enable も可。TrackerExtractor が唯一の producer (Phase 13 原則) なので extractor state 競合は構造的に発生しない

## ゴール / 完了条件

1. `--vmt-out` で 10 trackers を `/VMT/Room/Driver` 60 Hz で送信
2. SteamVR の Manage Trackers に `vmt_10..vmt_19` が出現し、role 手動割当後 VRChat avatar が 10-point FBT で追従
3. `/stats3d` の `"vmt":{...}` が steady-state で `sent_bundles ~ 60/s` / `disabled_count` 定常 0
4. `ctest -R 'vmt|tracker_extract|firmware_protocol'` が全 pass (新 12 ケース + 既存退行なし)
5. `--slimevr-out --vmt-out` 同時起動で extractor の state が 1 個だけ存在し二重 smoothing にならない (コードレビュー + smoke test で確認済)
6. `docs/cpp-migration-plan.md` の段階実装 + 検証戦略表に Phase 14 行追加
7. VMT Manager セットアップ手順が本 doc 内で完結 (操作者が phase14 doc 1 つで通電できる)

## アーキテクチャ

```
   Skeleton3DBus
        │
        ▼  (60 Hz, TrackerExtractor が唯一の producer)
   SlimeTrackerBus  (pos world Z-up + quat_wxyz, Phase 13 で導入済)
        │
   ┌────┴────┬──────────────┐
   ▼         ▼              ▼
 NativePub  VmtPublisher  CrowServer
 (UDP 6969) (UDP 39570)   (/ws3d, /stats3d, WebUI viz)
 回転のみ   位置+回転     ↑ "slimevr":{...} と "vmt":{...} 並列
   │         │
   ▼         ▼
 SlimeVR    SteamVR Driver (VMT)
 Server     └ VRChat FBT (10-point)
```

設計の肝:
- **`SlimeTrackerBus` を value-copy で読むだけの consumer**: 新 bus は作らない (Phase 13 原則)
- **位置 smoothing は TrackerExtractor 側**: VMT publisher は smoothed pos/quat をそのまま wire に乗せる
- **send_thread のみ**: VMT は ack なしの一方向 OSC なので recv_thread 不要 (`native_publisher` の半分の構造)

## ファイル構成

| ファイル | 役割 |
|---|---|
| `cpp/src/vmt/osc_writer.{hpp,cpp}` | OSC 1.0 wire-format serializer (commit `a64becf` から復元) |
| `cpp/src/vmt/vmt_protocol.{hpp,cpp}` | world → VMT Driver 座標変換 + `vmt_index_for(TrackerRole)` + `encode_vmt_room_driver()` |
| `cpp/src/vmt/vmt_publisher.{hpp,cpp}` | UDP socket + send_thread + 60 Hz pacing + degeneracy mode + stats |
| `cpp/tools/test_vmt_osc_writer.cpp` | 5 case (OSC 1.0 wire bytes golden + `/VMT/Room/Driver` full packet) |
| `cpp/tools/test_vmt_protocol.cpp` | 3 case (pos 3 軸 cardinal / quat 4 cardinal / index mapping) |
| `cpp/tools/test_tracker_extract_pos.cpp` | 4 case (alpha=1 passthrough / 6-frame 収束 / valid=false hold / dropout 復帰) |

修正:

| ファイル | 変更 |
|---|---|
| `cpp/src/slimevr/tracker_extract.{hpp,cpp}` | `apply_pos_smoothing(curr, prev_pos, base_alpha)` 追加 |
| `cpp/src/slimevr/tracker_extractor.{hpp,cpp}` | `prev_pos_` array + `pos_smooth` option + `run_loop` で `apply_quat_smoothing` 直後に呼ぶ |
| `cpp/src/config/main_config.{hpp,cpp}` | `vmt_*` 7 フィールド + YAML `vmt:` セクション + `--vmt-*` CLI + `validate_options` |
| `cpp/src/main.cpp` | `--vmt-*` help + binding alias + `VmtPublisher` ライフサイクル + `tex_opts.pos_smooth` |
| `cpp/src/web/crow_server.{hpp,cpp}` | `set_vmt_publisher()` + `/stats3d` の splice 2 段化 |

## CLI flags

| flag | 型 | default | 説明 |
|---|---|---|---|
| `--vmt-out` | bool | false | VMT publisher 起動 |
| `--vmt-host ADDR` | string | `127.0.0.1` | VMT Manager host (別 PC Windows なら IP) |
| `--vmt-port N` | int | 39570 | VMT 受信 UDP port |
| `--vmt-rate-hz F` | double | 60.0 | 送信レート (0, 240] |
| `--vmt-index-base N` | int | 10 | 先頭 VMT index。既定は `VMT_10..VMT_19`、範囲は 0..48 |
| `--vmt-pos-smooth F` | double | 0.5 | 位置 EMA alpha [0, 1]、0=freeze、1=smoothing なし |
| `--vmt-degeneracy-mode S` | enum | `hold` | `hold` / `disable` / `skip` |
| `--vmt-disable-below-floor` | bool | false | `pos.z < 0` の tracker は `enable=0` (room matrix sanity, debug) |

`validate_options`:
- `--vmt-out` ⇒ `--enable-3d` + `--keypoint-format=halpe26` 必須
- `--vmt-out` ⇒ `--calibrate` と排他
- `--vmt-port` ∈ [1, 65535] / `--vmt-rate-hz` ∈ (0, 240] / `--vmt-index-base` ∈ [0, 48] / `--vmt-pos-smooth` ∈ [0, 1]
- `--vmt-degeneracy-mode` ∈ {hold, disable, skip}
- `--vmt-out` と `--slimevr-out` の併用は許可

YAML 例 (`configs/<host>.yaml`):

```yaml
vmt:
  vmt_out: true
  host: 192.168.1.50
  port: 39570
  rate_hz: 60.0
  index_base: 10
  pos_smooth: 0.5
  degeneracy_mode: hold
  disable_below_floor: false
```

## 暫定 WebUI 手動位置合わせ

VMT 経由では SteamVR/HMD playspace と Jetson 側 tracker 座標の原点・向きを合わせる必要がある。HMD pose を使った自動位置合わせは別フェーズとし、当面は WebUI の 3D panel から手動で VMT 送信値へオフセットをかける。

- UI: `http://<jetson>:8000/` の `VMT alignment`
- API:
  - `GET /api/vmt/alignment`
  - `POST /api/vmt/alignment`
- JSON:

```json
{"x":0.0,"y":0.0,"z":0.0,"yaw_deg":0.0}
```

座標系は VMT/SteamVR Driver frame で、`X=right`, `Y=up`, `Z=back`、単位は meter。`yaw_deg` は `+Y` 軸回りの degree。適用順は `p' = R_y(yaw) * p + offset`、`q' = q_yaw * q`。WebUI は `base number + fine slider = 送信値` として扱う。`X/Y/Z` の数値 input は 1m 単位の粗調整、横幅いっぱいの slider は `-0.5..+0.5m` の微調整。`yaw` も数値 input と slider を分け、slider は `-45..+45deg` の微調整。値はプロセス内メモリだけに保持し、再起動で 0 に戻る。CLI/YAML 永続化や HMD pose 由来の自動推定は入れない。

## VMT プロトコル詳細

OSC 1.0 wire format で UDP 39570 へ 1 datagram per send-loop tick:

```
#bundle\0                                  // 8 byte bundle header
<8 byte NTP timetag, big-endian>           // OSC 1.0 timetag
<4 byte big-endian length=N1> <message 1>  // /VMT/Room/Driver msg
<4 byte big-endian length=N2> <message 2>
...
<4 byte big-endian length=N10> <message 10>
```

各 `/VMT/Room/Driver` メッセージ:

```
"/VMT/Room/Driver\0\0\0\0"                 // OSC string (NUL + 3 byte pad to 16)
",iiffffffff\0"                            // typetag: 2 ints + 8 floats + NUL + 4 byte pad (16 byte)
<i32 BE> index                             // index_base..index_base+9 (default 10..19)
<i32 BE> enable                            // 1 = active, 0 = disabled
<f32 BE> timeoffset                        // 0.0 (now)
<f32 BE> x, <f32 BE> y, <f32 BE> z         // pos in VMT Driver Y-up RH (meters)
<f32 BE> qx, <f32 BE> qy, <f32 BE> qz, <f32 BE> qw   // quat xyzw order
```

1 メッセージは 80 byte 強。10 メッセージ × bundle ヘッダで 1 bundle ≈ 780 byte → MTU (一般 LAN 1472 byte) に余裕で収まる。

## 座標変換マップ

`cpp/src/vmt/vmt_protocol.hpp` で:

```cpp
inline VmtPos world_pos_to_vmt(float x, float y, float z) {
    return {x, z, -y};  // (X-right, Y-up, Z-back)
}

inline VmtQuat world_quat_to_vmt(float qw, float qx, float qy, float qz) {
    // Rx(-90°) basis change. Closed form, xyzw on wire:
    return {qx, qz, -qy, qw};
}
```

Phase 12 Bridge の `world_quat_to_bridge_slime` と完全同型 (Bridge コメントに `TrackerYaw.kt fixes the server world frame at x-right, y-up, z-back, right-handed` と記載済)。SteamVR Driver も同じ frame を使うので Bridge 関数の式をそのままコピー (本体 4 行 + 7 行)。`test_vmt_protocol.cpp` の golden 値は archive `test_firmware_protocol.cpp` の Bridge transform ケースと同じ数値。

## Tracker index mapping (VRChat FBT role 手動割当の目安)

| default vmt_index | TrackerRole | SteamVR Manage Trackers role (推奨) | VRChat IK |
|---|---|---|---|
| 10 | LeftUpperArm  | LeftShoulder (拡張) / LeftElbow | VRChat IK 2.0 / 拡張 |
| 11 | RightUpperArm | RightShoulder (拡張) / RightElbow | 同上 |
| 12 | Chest         | Chest | VRChat 標準 |
| 13 | Waist (HIP)   | Waist | VRChat 標準 |
| 14 | LeftUpperLeg  | LeftKnee | VRChat 標準 |
| 15 | RightUpperLeg | RightKnee | VRChat 標準 |
| 16 | LeftLowerLeg  | (未割当推奨 — Knee と機能重複) | - |
| 17 | RightLowerLeg | 同上 | - |
| 18 | LeftFoot      | LeftFoot | VRChat 標準 |
| 19 | RightFoot     | RightFoot | VRChat 標準 |

VRChat 標準 FBT は Chest + Waist + 両足 + 両膝 + 両肘 の 8-point IK。LowerLeg (6, 7) は VRChat 標準 role に対応がないので、SteamVR Manage Trackers では未割当のままにし、知覚情報として送るだけ (avatar への影響なし)。10 個全部を avatar に反映させたい場合は VRChat 側 IK 拡張 (例えば Custom Animator) を別途用意する。

## smoothing 設計

```cpp
// cpp/src/slimevr/tracker_extract.cpp
void apply_pos_smoothing(std::array<SlimeTracker, kTrackerCount>& curr,
                         std::array<cv::Vec3f, kTrackerCount>& prev_pos,
                         float base_alpha) {
    base_alpha = std::clamp(base_alpha, 0.0f, 1.0f);
    for (std::size_t i = 0; i < kTrackerCount; ++i) {
        if (!curr[i].valid) {
            curr[i].pos = prev_pos[i];  // hold prev, leave prev unchanged
            continue;
        }
        cv::Vec3f& p = prev_pos[i];
        const cv::Vec3f q = curr[i].pos;
        p[0] += base_alpha * (q[0] - p[0]);
        p[1] += base_alpha * (q[1] - p[1]);
        p[2] += base_alpha * (q[2] - p[2]);
        curr[i].pos = p;
    }
}
```

`tracker_extractor.cpp::run_loop` の `apply_quat_smoothing(...)` 直後で呼ぶ。

設計判断:

- **`TrackerExtractor` 側に state を持たせる理由**: Phase 13 「一箇所で state を持つ」原則の踏襲。`prev_quat_` と `prev_pos_` が同じ場所にあれば WebUI viz と publisher が同じ post-smoothing pos を見る (AxesHelper のジッター低減も副産物)
- **confidence-modulated を採用しない理由**: `SlimeTracker.roll_confidence` は roll 軸の reliability であり、pos のノイズ源 (三角測量 reprojection error / keypoint score) とは独立。pos 専用の `pos_confidence` を入れるのは Phase 14 スコープ外 (Phase 15 候補)
- **One-Euro filter を採用しない理由**: 3 パラメータ (β, fc, dcut) のチューニングコスト + 実装コストが EMA より高い。実機評価で EMA が物足りなければ Phase 15 候補

## degeneracy mode

| mode | wire 動作 | 用途・期待挙動 |
|---|---|---|
| `hold` (default) | `enable=1, pos=prev, quat=prev` | 1〜2 frame の occlusion で誰も気づかない。VRChat IK は前周期保持の上で動く |
| `disable` | `enable=0, pos=prev, quat=prev` | 長期 occlusion で SteamVR 側 IK が "lost" 扱い。Avatar が一瞬不自然に飛ぶ可能性あり、気付きやすさ重視 |
| `skip` | message を送らない | VMT Driver の internal timeout (~250 ms) で自動的に disabled。debug 用 |

default `hold` の根拠: Phase 11 で `valid=false` 時の SlimeVR Server 側保持挙動が実用上 OK だった実績ベース。M5 (本 milestone) の実機検証で `disable` も試して挙動の docs 化を行う方針。

## /stats3d スプライス

```json
{
  "enabled": true,
  "stats": {...},
  "trackers": [...],
  "slimevr": {"sent_handshakes": ..., ...},
  "vmt": {
    "sent_bundles": 1234,
    "sent_trackers": 12340,
    "disabled_count": 0,
    "skipped_invalid_bundles": 5,
    "last_send_ms": 1748086342123.0,
    "rate_hz": 60.0,
    "index_base": 10,
    "port": 39570,
    "host": "192.168.1.50",
    "degeneracy_mode": "hold",
    "alignment": {"x":0.0,"y":0.0,"z":0.0,"yaw_deg":0.0}
  }
}
```

`--slimevr-out` と `--vmt-out` の組合せに応じて `slimevr` / `vmt` キーが付いたり付かなかったりする (publisher 構築時のみ splice)。

## Milestones (実装履歴)

| M | commit | 内容 |
|---|---|---|
| M1 | `feat(phase14): M1 — OSC 1.0 writer 復元 + VMT wire format + golden tests` | `cpp/src/vmt/{osc_writer,vmt_protocol}.{hpp,cpp}` + 8 test case |
| M2 | `feat(phase14): M2 — VmtPublisher + CLI 配線 + main 統合` | `cpp/src/vmt/vmt_publisher.{hpp,cpp}` + `--vmt-*` CLI + YAML + main wiring |
| M3 | `feat(phase14): M3 — 位置 EMA を TrackerExtractor に追加 + ctest` | `apply_pos_smoothing` + `prev_pos_` + 4 test case |
| M4 | `feat(phase14): M4 — /stats3d への VMT publisher stats splice` | `set_vmt_publisher` + JSON splice 2 段化 |
| M5 | `docs(phase14): M5 — phase14-vmt-steamvr.md + migration plan 行追加` | 本 doc + `cpp-migration-plan.md` 反映 |

`cpp-phase14` ブランチを `cpp-yolox-int8-eval` (Phase 13 tip) から切る (CLAUDE.md ルール準拠)。1 milestone 1 commit。

## 検証

### 単体 (ctest, 実機不要)

```bash
ctest --test-dir cpp/build --output-on-failure -R 'vmt|tracker_extract|firmware_protocol'
```

カバー範囲 (12 新規 + 既存退行なし):

| test | 何を検証するか |
|---|---|
| `test_vmt_osc_writer` (5 case) | OSC 1.0 wire bytes golden + `/VMT/Room/Driver` full packet shape |
| `test_vmt_protocol` (4 case) | TrackerRole→vmt_index 順 + index_base offset / `world_pos_to_vmt` 3 軸 cardinal / `world_quat_to_vmt` 4 cardinal |
| `test_tracker_extract_pos` (4 case) | alpha=1 passthrough / 6-frame 収束 / valid=false hold + prev 不変 / dropout 復帰時 prev が (0,0,0) に戻らない |
| 既存 `test_firmware_protocol` / `test_tracker_extract` | 退行なし |

### 統合 (Jetson 単体、wire 確認)

```bash
# Terminal A — fake VMT receiver:
nc -u -l 39570 | xxd

# Terminal B — Jetson:
./cpp/build/main --enable-3d --keypoint-format=halpe26 \
    --vmt-out --vmt-host=127.0.0.1 --vmt-port=39570 \
    --cam0 ... --cam1 ... --calib ... --det-engine ... --pose-engine ...
```

Terminal A 側で `#bundle\0` + 10 個の `/VMT/Room/Driver` メッセージ (1 bundle ~780 byte) が 60 Hz で観測されれば wire OK。

```bash
curl http://<jetson>:8000/stats3d | jq .vmt
# {
#   "sent_bundles": ...,
#   "sent_trackers": ...,
#   ...
# }
```

### E2E (Windows + Quest/Index + SteamVR + VMT Manager + VRChat)

#### VMT Manager セットアップ手順

1. Windows 機で https://gpsnmeajp.github.io/VirtualMotionTrackerDocument/ から VMT v0.15 を落とす
2. zip 展開後 `vmt_manager.exe` を起動 → 「Install Driver」ボタンを押下 (内部で `vrpathreg.exe register` を実行、SteamVR の `steamvr.vrsettings` にドライバパスを登録)
3. SteamVR を一度終了して再起動 (Driver の自動ロードが起動時のみ走るため)
4. SteamVR + VMT Manager が両方起動した状態で、HMD を装着して被験者の立ち位置 (Jetson カメラの視野内) で「Set Room Matrix」ボタン押下 → Manager が SteamVR HMD pose を基準に Jetson 世界座標との変換行列を計算し `setting.json` に永続化
5. SteamVR Settings → Controllers → Manage Trackers で 10 個の `vmt_10`..`vmt_19` を見つけ、上の対応表に従って role を手動割当

#### Jetson 通電

```bash
./cpp/build/main --enable-3d --keypoint-format=halpe26 \
    --vmt-out --vmt-host=<windows-ip> --vmt-port=39570 \
    --vmt-pos-smooth=0.5 --vmt-degeneracy-mode=hold \
    --cam0 ... --cam1 ... --calib ... \
    --det-engine ... --pose-engine ...
```

#### 通電チェックリスト (3 段)

1. **Wire**: Windows 側 firewall で UDP 39570 inbound を許可しているか? Jetson 側で `nc -u -l 39570 | xxd` で送信 wire を確認できるか?
2. **Manager**: VMT Manager の status バー「Last Recv」が 60 Hz で更新されているか? Room Matrix のステータスバーが緑色になっているか?
3. **VRChat**: VRChat 起動 → FBT calibration → avatar が 10-point で動くか? `/stats3d.vmt.sent_bundles` が ~60/s で増加、`disabled_count` 定常 0 か?

## Risk・残課題

| ID | 内容 | 対応 |
|---|---|---|
| R1 | 60 Hz × 10 trackers × 10 args が UDP MTU 超え | 1 bundle ~780 byte で MTU (1472) 内に収まる。実測で超えたら 5+5 分割パッチで対応 (現状実装不要) |
| R2 | VMT v0.15 のバージョン依存 | `/VMT/Room/Driver` address + 引数 layout は v0.15 不変。docs に「v0.15+ 前提」明記 |
| R3 | Phase 11/13 Firmware UDP path と VMT path の extractor state 競合 | TrackerExtractor が唯一の producer、両 publisher は read-only consumer → 構造的に競合なし。M2 で `--slimevr-out --vmt-out` 同時起動を validate 通過確認 |
| R5 | `--vmt-degeneracy-mode=disable` の挙動 (SteamVR 側 IK の "lost" 扱い) は実機未確認 | M5 実機検証で `hold`/`disable` 両方試して docs 追記。default は `hold` (Phase 11 実績ベース) |
| R6 | Jetson カメラ再キャリブで world frame が動くと VR room との整合崩れる | 「VR セッション開始時に Manager で Room Matrix を取り直す」運用を docs 明記。CLI から `/VMT/SetRoomMatrix` 自動化は Phase 15 候補 |
| R7 | VMT Manager 未起動 / port 不一致 / firewall で送信が黙って捨てられる | 通電チェックリスト 3 段で診断 |
| R8 | UDP packet loss 時に SteamVR 側で trackers が遅延する可能性 | 60 Hz × 10 messages bundle なら 16 ms の latency 不感帯。LAN packet loss は 0.01% 以下が期待値で実用上問題なし |

## Out of scope (Phase 15 候補)

- HMD なし運用 (Null HMD driver / VMT の HMD-less mode の探索)
- Room Matrix を CLI から OSC で打つ自動化 (`/VMT/SetRoomMatrix`)
- VMT serial / TrackerHints の OSC 自動化 (起動時に SteamVR 側 role 自動割当を狙う)
- 位置の confidence-modulated smoothing (`pos_confidence` 追加)
- 位置の One-Euro filter (EMA が物足りなければ)

## 関連ドキュメント

- [`phase11-slimevr-integration.md`](phase11-slimevr-integration.md) — Firmware UDP 経路 (回転のみ、本流維持)
- [`phase12-slimevr-bridge-relay.md`](phase12-slimevr-bridge-relay.md) — Bridge relay 経路 (没、archive 凍結)
- [`phase13-quality-refinement.md`](phase13-quality-refinement.md) — degeneracy gate + per-tracker stats (Phase 14 はこの上に積む)
- [`cpp-migration-plan.md`](cpp-migration-plan.md) — 段階実装 + 検証戦略表
