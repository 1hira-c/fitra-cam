# Backlog — SlimeVR Bridge protocol over network (B 案)

> 2026-05-21 に Phase 11 で却下した「位置+回転を SlimeVR に届ける」案を、将来必要になった場合の参照用として残す。Phase 11 本体は Firmware UDP (回転のみ、port 6969) で本流に乗せた。

## なぜ積み課題なのか

SlimeVR には wire 上 **位置を運べる経路が 2 つだけ** ある:

1. **Bridge protocol** — Protobuf over Unix domain socket (`${XDG_RUNTIME_DIR}/SlimeVRInput` on Linux) または Named pipe (`\\.\pipe\SlimeVRInput` on Windows)。SlimeVR-Feeder-App が OpenVR pose を吸って SlimeVR に流すために使うのと **同じ経路**。`Position` message が `tracker_id / x / y / z / qx / qy / qz / qw` を持つので、位置 + 回転 + body-part role 全部いける。
2. **VMC over OSC** — 位置を運べるが、SlimeVR 側で連番扱いになるため body-part assign が手動。Phase 11 で却下した経路。

Bridge protocol は **IPC 専用**。Unix socket / Named pipe は同一マシン内のみ。我々のセットアップは Jetson (Linux) ⇄ Windows PC (SlimeVR Server) のネットワーク越しなので、Bridge を直接は使えない。

Phase 11 では Firmware UDP に倒して回転だけ運ぶことにした。代償:
- カメラ駆動の絶対位置情報が VR 側で活かせない (SlimeVR の IK が HMD と骨格から再構築する)
- 床位置のアンカー / 部屋を歩いている感じが HMD 依存になる

これが許容できない要件 (例: 録画解析のためにアバター絶対位置を SteamVR 上で見たい / カメラ駆動の「室内位置」をベースにマルチユーザを並べたい) が出てきたら、この B 案を起動する。

## 実装構想

```
   Jetson (Linux)                          Windows PC (SlimeVR Server)
   ┌─────────────────────┐                 ┌──────────────────────────────┐
   │ fitra-cam main      │                 │ slimevr-bridge-relay.exe     │
   │  └ NativePublisher  │                 │  (新規 / 別リポジトリ)        │
   │     ─ extra mode ─→ │ ── UDP/TCP ──→  │  ─ Protobuf framing ──→ named│
   │     position+rot    │  custom 自前    │  pipe \\.\pipe\SlimeVRInput  │
   │     binary stream   │  binary frame   │     │                         │
   └─────────────────────┘                 │     ▼                         │
                                            │  SlimeVR-Server (既存)        │
                                            │     ── Bridge protocol ──     │
                                            │     位置+回転を受信、         │
                                            │     名前付きトラッカーで表示  │
                                            └──────────────────────────────┘
```

### Jetson 側 (fitra-cam の追加モード)

- `--slimevr-out=bridge --slimevr-host=<windows-ip> --slimevr-port=<custom>` で起動可能に
- Firmware UDP の publisher と並列 (排他) で動かす独立 publisher を追加
- 送信内容: 10 tracker 分の (sensor_id, world_pos, world_quat, valid) を毎周期 60 Hz でカスタムバイナリで TCP/UDP に流す
- 既存の `slimevr::extract_trackers()` 出力をそのまま流せる (世界フレームのまま)

### Windows 側 (新規 relay app)

候補実装:

1. **SlimeVR-Feeder-App fork**: 既存の OpenVR 入力部を削って、JSON/Protobuf/カスタムバイナリの TCP/UDP 受信に置き換える。bridge protocol への変換ロジックは既にある (`bridge.cpp:218-248` の send 部分)。実装コスト中。
2. **新規 C# / Rust app**: `\\.\pipe\SlimeVRInput` への書き込み + Protobuf シリアライズだけ実装。コードは ~500 行で済む見込み。コードを軽くしたい場合はこちら。
3. **既存 owoTrack-style ブリッジ**: もし誰かが既に書いていれば調査して採用。

座標系:
- Jetson 側: world Z-up, X-right, Y-forward, meters, 床原点
- Bridge protocol: SlimeVR 内部は Y-up Unity LH (`firmware_protocol::world_quat_to_slime` と同等の変換を適用)
- 変換は Jetson 側で済ませる (relay は素通し) のが単純

トラッカー命名 (`TrackerAdded.tracker_role`):
- ID 1=WAIST, 2=LEFT_FOOT, 3=RIGHT_FOOT, 4=CHEST, 5=LEFT_KNEE, 6=RIGHT_KNEE, 7=LEFT_ELBOW, 8=RIGHT_ELBOW, 9=LEFT_SHOULDER, 10=RIGHT_SHOULDER
- 注: Bridge protocol の `tracker_role` は `TrackerRole.kt` で、Firmware UDP の `TrackerPosition.kt` とは別 enum。10 本構成のマッピングをやり直す必要あり。`LEFT_UPPER_LEG` / `LEFT_LOWER_LEG` / `LEFT_UPPER_ARM` 等の細分化が tracker_role にあるか確認すること。

### 関連リポジトリ (clone 済み)

- `~/Documents/refs/slimevr/SlimeVR-Feeder-App` — Protobuf + 名前付きパイプ実装の正本
- `~/Documents/refs/slimevr/SlimeVR-Server` — Bridge handler (`server/desktop/src/main/java/dev/slimevr/desktop/platform/ProtobufBridge.kt`)
- Protobuf schema: `SlimeVR-Feeder-App/ProtobufMessages.proto`

## トリガ条件

このプロジェクトを起動するのは以下のいずれかの要件が立った時:

- カメラ駆動の絶対床位置を SteamVR 上で活用したい (= SlimeVR の IK 推定では満たせない用途)
- 複数被験者を同じ SteamVR 空間に並べたい (= 各人の絶対位置が独立に要る)
- 床に対する歩行軌跡を VR で記録 / 再生したい

逆に「VRChat / SteamVR でアバターを動かす」だけが目的なら、Firmware UDP のままで十分。

## 評価 (Phase 11 実装着手前)

| 項目 | Firmware UDP (現状) | Bridge over relay (B 案) |
|---|---|---|
| 位置情報 | ❌ IK で再構築 | ✅ 直接届く |
| 回転情報 | ✅ | ✅ |
| 名前付き表示 | ✅ SensorInfo packet 15 | ✅ TrackerAdded.tracker_role |
| ネットワーク越し | ✅ | ✅ (relay 経由) |
| Windows 側追加配備 | 不要 | **要 relay app** |
| 実装コスト | 中 (済) | 高 (relay 新規 + Jetson 側 mode 追加) |
| 保守 | SlimeVR firmware 互換性に追従 | SlimeVR Bridge protocol 互換性に追従 |

## 関連リンク

- [`phase11-slimevr-integration.md`](phase11-slimevr-integration.md) — Phase 11 で採用した Firmware UDP の設計
- [`cpp-migration-plan.md`](cpp-migration-plan.md) — 全体ロードマップ
