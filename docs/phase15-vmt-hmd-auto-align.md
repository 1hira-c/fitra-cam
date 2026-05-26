# Phase 15 — SteamVR HMD pose 駆動の自動 VMT alignment

> Phase 14 で VMT publisher (UDP `/VMT/Room/Driver` 60 Hz) + 手動 alignment UI (yaw + xyz offset, 永続化なし) が入り、SteamVR Driver への直結経路が定着した。残課題は **アバター位置を被験者の HMD 位置に毎回合わせる** こと。手動 slider では yaw 5° の誤差で足元が大きくずれ、被験者ごとに数十秒の調整時間が発生する。
>
> Phase 15 では SteamVR 側にいる **HMD の現在 pose を OpenVR で取り出し**、Jetson 側で chest tracker との対応から **自動で VmtAlignment を逆算** する。手動 UI は残し、自動算出した値を初期値として手動微調整できる連続的なフローにする。

## Context

VMT driver 単体では HMD の現在 pose を SteamVR から取り出す経路が公式 API として整っていない (`vmt_driver/ServerTrackedDeviceProvider.cpp` に `/VMT/Subscribe/Device` の仕組みは存在するが、HMD 自身の serial が `LHR-XXXXXXXX` 形式で運用前に確定しないなど運用が重い)。そこで Phase 15 では **VMT を変更せず、Windows 側に独立した OpenVR overlay app `vmt_hmd_pose_sender` を新設**して HMD pose を取得し、OSC で Jetson に流す。Jetson 側で HMD と chest tracker の xz delta から `VmtAlignment` を 2D Procrustes で解いて `VmtPublisher::set_alignment()` に流し込む。

確定方針 (2026-05-26):

- **Windows 側**: 独立 overlay app を新規作成 (`windows/vmt_hmd_pose_sender/`、C++ + openvr-1.23.7 + oscpack)。VMT 本体は変更しない
- **スコープ**: HMD 座標取得 + 自動 alignment 算出まで。Room Matrix 自動化、alignment YAML 永続化、HMD-chest の Y 差自動補正は **Phase 16 候補**
- **キャリブ操作**:
  - **基本**: T ポーズで「Tポーズで合わせる」ボタン 1 押し → yaw + xz を即時算出
  - **拡張**: 「移動キャリブ開始」で 3 秒間 HMD と chest の軌跡を貯め、2D Procrustes で yaw を精度よく推定
- **手動 UI との関係**: 自動算出は手動と **同じ VmtAlignment channel** に書き込む (`vmt_publisher_->set_alignment()`)。自動で粗く合わせ → 手動 slider で 1cm 単位微調整、というフロー

## ゴール / 完了条件

1. Windows 上で `vmt_hmd_pose_sender.exe --jetson <ip>` を起動 → SteamVR + HMD 装着で `nc -u -l 39571` に 60 Hz で `/fitra/hmd_pose` が届く
2. Jetson 側 `--hmd-listen-enabled` で受信、`/stats3d.hmd` ブロックに `valid=true / age_ms < 200 / pos / yaw_deg` が出る
3. Web UI の「Tポーズで合わせる」押下 → 1 サンプルで `VmtAlignment` (yaw + xz) を更新し SteamVR avatar が HMD と一致
4. Web UI の「移動キャリブ開始」押下 → 3 秒間歩行 → `residual_m < 0.02m` で alignment 更新
5. `ctest -R 'hmd_pose|auto_alignment|vmt'` が全 pass (新 12 ケース + Phase 14 既存退行なし)
6. `docs/cpp-migration-plan.md` 段階実装 + 検証戦略表に Phase 15 行追加

## アーキテクチャ

```
[Windows]                                          [Jetson]
 SteamVR runtime                                    cpp/build/main
   └─ HMD (Quest/Index)                              ├─ TrackerExtractor
   └─ VMT Driver ◀── /VMT/Room/Driver 60Hz ──────    │   (chest=role 2)
                                                     ├─ VmtPublisher
 vmt_hmd_pose_sender.exe (NEW, M1)                   │   └─ apply_vmt_alignment
   ├─ openvr-1.23.7 (VRApplication_Background)       │       (auto + 手動 同 channel)
   ├─ GetDeviceToAbsoluteTrackingPose                ├─ HmdPoseReceiver (M2)
   │   (TrackingUniverseStanding)                    │   └─ HmdPoseBus
   └─ oscpack → /fitra/hmd_pose 60Hz ────────────▶   ├─ AutoAlignmentSolver (M3)
       (Jetson:39571)                                │   ├─ solve_tpose (1サンプル)
                                                     │   └─ solve_motion (2D Procrustes)
                                                     └─ CrowServer (M4)
                                                         /api/vmt/alignment/auto/*
                                                         /stats3d → vmt.hmd
                                                         Web UI: 新 form (HMD ステータス + 3 ボタン)
```

設計の肝:
- **HMD pose は VMT Driver frame と一致** — `TrackingUniverseStanding` (Y-up RH) は `world_pos_to_vmt` の出力と同じ frame。Jetson 側で世界→VMT 変換した chest を **同じ座標系で 2D 平面回帰** できる
- **TrackerExtractor 唯一 producer 原則は維持** — Solver は SlimeTrackerBus と HmdPoseBus を read-only consume のみ
- **自動と手動は同 channel** — VmtAlignment は 1 つしかなく、`set_alignment` mutex で守られる。「最後に書いた人が勝つ」モデル

## ファイル構成

### 新規

| ファイル | 役割 |
|---|---|
| `windows/vmt_hmd_pose_sender/CMakeLists.txt`         | MSVC 専用、ルート CMake からは外す |
| `windows/vmt_hmd_pose_sender/src/main.cpp`           | `VR_Init(VRApplication_Background)` + 60 Hz `GetDeviceToAbsoluteTrackingPose` + OSC 送信 |
| `windows/vmt_hmd_pose_sender/src/osc_send.{hpp,cpp}` | oscpack ラッパー (`OscSender`) |
| `windows/vmt_hmd_pose_sender/cmake/Findopenvr.cmake` | `refs/VirtualMotionTracker/openvr-1.23.7` を expose、`openvr_api.dll` を exe 横にコピー |
| `windows/vmt_hmd_pose_sender/README.md`              | ビルド / Firewall / 通電チェックリスト |
| `cpp/src/vmt/hmd_pose_receiver.{hpp,cpp}`            | `HmdPose` / `HmdPoseBus` (latest-wins, stale 判定) / `HmdPoseReceiver` (UDP listen, 手書き OSC parser) |
| `cpp/src/vmt/auto_alignment.{hpp,cpp}`               | `solve_tpose` / `solve_motion` (2D Procrustes via `cv::SVD`) / `yaw_from_vmt_quat` |
| `cpp/tools/test_hmd_pose_receiver.cpp`               | 4 ケース (parse 往復 / address mismatch / typetag mismatch / truncated + bus stale) |
| `cpp/tools/test_auto_alignment.cpp`                  | 8 ケース (tpose 恒等 / 純平行 / 純 yaw / NoHmd / motion 恒等 / motion yaw+平行 / Gaussian noise / NotEnoughSamples & Degenerate) |

### 修正

| ファイル | 変更 |
|---|---|
| `cpp/src/config/main_config.{hpp,cpp}` | `hmd_listen_*` 4 フィールド + YAML `vmt.hmd_listen_*` + `--hmd-listen-*` CLI + `validate_options` |
| `cpp/src/main.cpp` | `--hmd-listen-*` help + `HmdPoseBus` + `HmdPoseReceiver` ライフサイクル + `SlimeStop` への組み込み + `set_hmd_pose_bus` |
| `cpp/src/web/crow_server.{hpp,cpp}` | `set_hmd_pose_bus()` + 4 ルート + `/stats3d` に `hmd` ブロック splice + `chest_in_vmt` helper + `AutoAlignSession` (`Impl` 構造体内) |
| `web/dual_rtmpose/index.html` | 新 `<form id="vmt-auto-form">` (HMD ステータス + 3 ボタン + result) |
| `web/dual_rtmpose/app.js` | `postAutoTpose / startMotionCalib / stopMotionCalib` + HMD ステータス更新 (`update3DStats`) + 自動結果を `writeVmtAlignmentForm` 経由で手動 form にも反映 |

## OSC プロトコル (`/fitra/hmd_pose`)

```
address  = "/fitra/hmd_pose"
typetag  = ",iffffffff"
args     = valid(i32) timestamp_s(f32)
           x(f32) y(f32) z(f32)
           qx(f32) qy(f32) qz(f32) qw(f32)
```

| フィールド | 意味 |
|---|---|
| `valid`        | 1 = HMD tracking OK / 0 = tracking lost or device disconnected |
| `timestamp_s`  | sender 起動からの monotonic 秒 (latency 計測用) |
| `x, y, z`      | SteamVR Standing universe (Y-up RH, X-right, Z-back, m) |
| `qx, qy, qz, qw` | xyzw 順 (VMT publisher が `/VMT/Room/Driver` で使う順序と同じ) |

単発メッセージ (bundle 無し)。1 datagram = 64 byte。

## CLI

```bash
./cpp/build/main \
  --enable-3d --keypoint-format=halpe26 \
  --vmt-out --vmt-host=<windows-ip> --vmt-port=39570 \
  --hmd-listen-enabled --hmd-listen-port=39571 --hmd-stale-ms=200 \
  --cam0 ... --cam1 ...
```

| Flag | Default | 用途 |
|---|---|---|
| `--hmd-listen-enabled`    | (off) | UDP socket を bind して `/fitra/hmd_pose` を受信 |
| `--hmd-listen-port N`     | 39571 | listen port |
| `--hmd-listen-bind ADDR`  | 0.0.0.0 | bind address |
| `--hmd-stale-ms F`        | 200   | 受信途絶判定 (`HmdPoseSnapshot.stale = age_ms > F`) |

YAML (`configs/*.yaml`):

```yaml
vmt:
  hmd_listen_enabled: true
  hmd_listen_port:    39571
  hmd_listen_bind:    "0.0.0.0"
  hmd_stale_ms:       200
```

## 自動 alignment アルゴリズム

### T-pose (n=1, `solve_tpose`)

```
入力: hmd = (p_h:VmtPos, q_h:VmtQuat), chest = (p_c:VmtPos, q_c:VmtQuat)
       (どちらも VMT Driver frame; chest は world_pos_to_vmt / world_quat_to_vmt 適用後)

# Y-up quaternion から yaw 抽出
yaw(q) = atan2(2(qw·qy + qx·qz), 1 − 2(qy² + qz²))
yaw_rad = wrap_pi(yaw(q_h) − yaw(q_c))

# yaw 適用後の chest を hmd に合わせる平行移動 (xz のみ)
# R_y は apply_vmt_alignment と同じ [[c, s], [-s, c]] 規約
(x', z') = R_y(yaw_rad) · (chest.x, chest.z)
alignment.x       = hmd.x - x'
alignment.z       = hmd.z - z'
alignment.y       = 0          # 自動では触らない (head/chest の Y 差は個人差大)
alignment.yaw_deg = yaw_rad * 180/π
```

Y を触らない理由: HMD は頭頂、chest は胴体中心。被験者差で 0.35〜0.55m あり、自動算出に頼ると床がずれる。手動 UI の Y slider で被験者ごとに合わせる運用。

### Motion (n≥4, `solve_motion`)

```
入力: {(hmd_x_i, hmd_z_i), (chest_x_i, chest_z_i)} の N サンプル (VMT frame xz only)

μ_h = mean({hmd_x_i, hmd_z_i})
μ_c = mean({chest_x_i, chest_z_i})
H   = Σ (p_c_i - μ_c)(p_h_i - μ_h)^T      ∈ R^{2×2}
SVD H = U Σ V^T
d   = sign(det(V · U^T))                  # ±1, 反射防止
R   = V · diag(1, d) · U^T                # proper rotation
yaw = atan2(R[0,1], R[0,0])               # VMT 規約 [[c, s], [-s, c]]
t   = μ_h − R · μ_c
residual = (1/N) Σ ‖R · p_c_i + t − p_h_i‖
```

退化検出: `σ_1 / σ_0 < 1e-3` で `Degenerate` を返す (軌跡が直線状で yaw 不能)。

実装は `cv::SVD::compute` のみ (Eigen を引かない、`cpp/src/lift/triangulator.cpp:196` と同じ流儀)。

## REST API

| Method | Path | 用途 |
|---|---|---|
| `GET`  | `/api/vmt/alignment`                           | 既存 (Phase 14): 現在の VmtAlignment |
| `POST` | `/api/vmt/alignment`                           | 既存 (Phase 14): 手動で `{x,y,z,yaw_deg}` を書き込み |
| `POST` | `/api/vmt/alignment/auto/tpose`                | 単発 T-pose キャリブ。HMD + chest を 1 サンプル取って `solve_tpose` → `set_alignment` |
| `POST` | `/api/vmt/alignment/auto/motion/start`         | `{duration_s, sample_hz}` で bg thread サンプル収集開始 |
| `POST` | `/api/vmt/alignment/auto/motion/stop`          | 収集中なら停止 → `solve_motion` → `set_alignment` |
| `GET`  | `/api/vmt/alignment/auto/status`               | state (`idle`/`collecting`/`ok`/`err`) + 最新 result |

`/stats3d.hmd` ブロック (always present iff `--hmd-listen-enabled`):

```json
"hmd": {
  "enabled":     true,
  "have_any":    true,
  "stale":       false,
  "valid":       true,
  "age_ms":      18.7,
  "timestamp_s": 142.31,
  "pos":         [0.13, 1.65, -0.42],
  "quat_xyzw":   [0.0, 0.342, 0.0, 0.94],
  "yaw_deg":     40.0
}
```

## Web UI

VMT alignment 既存 form の直下に新 `<form id="vmt-auto-form">` を配置:

```
┌─ VMT alignment (既存) ─────────────────────────────┐
│ X / Y / Z / yaw  slider + base + total              │
│ [Apply] [Reset]                                     │
└────────────────────────────────────────────────────┘
┌─ VMT auto-alignment (新規) ────────────────────────┐
│  HMD status: tracking (18ms) | stale | lost | …     │
│  [Tポーズで合わせる] [移動キャリブ開始] [停止]      │
│  result: yaw=12.3° tx=0.31 tz=-0.04 residual=0.018m │
└────────────────────────────────────────────────────┘
```

- 自動結果は `writeVmtAlignmentForm()` で上の手動 form にも反映 → 「自動 → 手動微調整」が一貫した操作になる
- 「移動キャリブ開始」は `duration_s = 3.0`, `sample_hz = 30.0` で start し、3.4 秒後にクライアント側タイマーで自動 stop。手動停止も可
- HMD ステータスは `/stats3d.hmd` を `update3DStats()` で読み、`vmt-hmd-status` バッジに反映

## 検証戦略

### Unit (ctest)

```bash
cmake --build cpp/build -j
ctest --test-dir cpp/build --output-on-failure -R 'hmd_pose|auto_alignment|vmt'
```

合格基準:
- `test_hmd_pose_receiver` 4 ケース pass: golden bytes 往復 / address mismatch reject / typetag mismatch reject / truncated reject + `HmdPoseBus` stale 判定境界
- `test_auto_alignment` 8 ケース pass: T-pose 恒等 / 純平行 / 純 yaw / NoHmd / motion 恒等 / motion yaw+平行 / Gaussian noise (σ=0.01m, N=100, residual < 0.02m) / NotEnoughSamples & Degenerate

### M1 スモークテスト (Windows app)

- Windows: `vmt_hmd_pose_sender.exe --jetson <jetson-ip> --port 39571`
- Jetson: `nc -u -l 39571 | xxd` または Wireshark `udp.port == 39571`
- 60 Hz で 64 byte の `/fitra/hmd_pose` メッセージが流れること
- HMD を頭から外す → 約 100 ms 以内に `valid=0` メッセージに切り替わること

### M2 受信テスト (Jetson, host-only)

```bash
# fixture を作って手で送る
python3 -c "
import socket, struct
# OSC /fitra/hmd_pose ,iffffffff valid=1 ts=1.0 pos=(0.1,1.6,-0.3) quat=(0,0,0,1)
addr = b'/fitra/hmd_pose\x00'
tt   = b',iffffffff\x00\x00'
args = struct.pack('>i9f', 1, 1.0, 0.1, 1.6, -0.3, 0.0, 0.0, 0.0, 1.0)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.sendto(addr+tt+args, ('127.0.0.1', 39571))"
curl http://localhost:8000/stats3d | jq '.hmd'
```

合格基準: `valid=true`、`age_ms < 50`、`pos`/`quat_xyzw` が送った値と一致。`hmd_stale_ms=200` 経過後に `stale=true`。

### M4 E2E オフライン (実機)

```bash
./cpp/build/main --enable-3d --keypoint-format=halpe26 \
   --vmt-out --vmt-host=127.0.0.1 --vmt-port=39570 \
   --hmd-listen-enabled --hmd-listen-port=39571 \
   --cam0 ... --cam1 ...
```

1. ブラウザで `http://<jetson>:8000/` → HMD status が "tracking (XXms)" になる
2. 被験者が T ポーズ → 「Tポーズで合わせる」押下 → 自動 result に yaw/tx/tz 反映、手動 form に同期、SteamVR 側で chest tracker が HMD と一致
3. 「移動キャリブ開始」→ 前後左右 3 秒歩行 → タイマーで自動停止 → residual 表示 (歩行半径 ~0.5m で `< 0.02m` 目安)
4. `curl /stats3d | jq '{hmd: .hmd, vmt_align: .vmt.alignment}'` で整合確認

### M4 E2E オンライン VR

- VRChat 起動 → FBT calibration → chest avatar が HMD と一致
- 自動キャリブ前 / T-pose 後 / motion 後 の avatar 位置を screen-record で比較

## リスク

| ID | 内容 | 対応 |
|---|---|---|
| R1 | T-pose 瞬間の UDP jitter (±16ms) で chest 値が ±1cm 揺れる | 単発 default。気になれば motion mode 3 秒で代替 (Phase 15 内で完結) |
| R2 | SteamVR が seated universe で動いていると Y 軸基準が違う | M1 で `SetTrackingSpace(TrackingUniverseStanding)` を起動時強制 |
| R3 | HMD (頭頂) と chest (胴体中心) の Y 差 (0.35〜0.55m) を自動補正しない | docs (本ドキュメント) と Web UI の Y slider で被験者ごとに調整、明示 |
| R4 | SteamVR 未起動 / HMD 未装着 | M1 で `VR_Init` 失敗時 30s リトライ (最大 60 回)。M2 で `hmd_stale_ms` 超過時 `valid=false`。`/api/.../auto/tpose` は 409 |
| R5 | VMT Manager の Room Matrix を取り直すと HMD 座標も動く | M1 README に「Room Matrix を取り直したら `vmt_hmd_pose_sender.exe` を再起動」明記 |
| R6 | Motion 中に被験者がほぼ動かず直線退化 | `Degenerate` 検出 → Web UI に `degenerate` 文字 + 前回 alignment 維持 |
| R7 | UDP port 39571 が Windows firewall でブロック | M1 README に `netsh advfirewall firewall add rule` 例外手順 |
| R8 | 自動と手動の同時更新 race | 既存 `set_alignment` mutex で守られている、最後の write が勝つ。WebUI の連続 slider の最後に自動結果が上書きされる順序は許容 |
| R9 | OpenVR が SteamVR shutdown 通知を送ったタイミングでクラッシュ | M1 で `VREvent_Quit` を poll し `VR_Shutdown` → 再 init ループに戻る |

## Out of scope (Phase 16 候補)

- alignment の YAML 永続化 (`subjects/<id>/vmt_alignment.yaml`)
- Room Matrix を `/VMT/SetRoomMatrix` で CLI から自動化
- HMD 以外 (Vive Tracker / Controller) を reference にする選択
- Y 軸の自動補正 (HMD-chest の身長依存差を chest profile から推定)
- ICP ベース継続キャリブ (motion 中の常時微調整)
- C# WPF ベースの GUI 版 Windows app
- `vmt_hmd_pose_sender` を VMT 本体に PR

## 関連ドキュメント

- [`phase14-vmt-steamvr.md`](phase14-vmt-steamvr.md) — VMT 経路の確立 (前提)
- [`phase12-slimevr-bridge-relay.md`](phase12-slimevr-bridge-relay.md) — Bridge relay 没経緯
- [`cpp-migration-plan.md`](cpp-migration-plan.md) — 段階実装 + 検証戦略表
