# Phase 12 — SlimeVR Bridge relay + 回転品質改善

> **方針 (2026-05-22 起票)**: Phase 11 の SlimeVR Firmware UDP (回転のみ, port 6969) で本流に乗せた経路を残したまま、**Windows PC 側に新規 relay バイナリ (`slimevr-bridge-relay.exe`) を常駐させて SlimeVR Bridge protocol (Protobuf over Named pipe `\\.\pipe\SlimeVRInput`) に変換する経路**を新設する。これにより SlimeVR の IK に **位置情報を効かせる** ことが可能になり (`UDPPacket27Position` ルートでは `hasPosition=false` 固定で IK 無視されていた問題を回避)、足の床滑り症状の解消を狙う。
>
> あわせて、`cpp/src/slimevr/tracker_extract.cpp` の **二の腕 / 大腿 / 足 の up ベクトル選択** を物理的に正しい候補に差し替えて、回転品質そのものを底上げする (Bridge 経由でも UDP 経由でも効く改良)。大腿改修は Phase 11 Firmware UDP 経路 (10 本構成) の LEFT/RIGHT_UPPER_LEG にのみ効く (Bridge 経路の 8 本構成は thigh を送らない設計)。
>
> - **Firmware UDP は撤去しない**: Phase 11 の経路はそのまま残す。Bridge 経由 publisher は `--slimevr-bridge` で独立起動。`--slimevr-out` (Firmware UDP) との同時指定は二重登録防止のため early-fail で reject。
> - **Bridge は IPC 専用なので relay 必須**: Bridge protocol の Named pipe / Unix socket は同一マシン内のみ。Jetson (Linux) ⇄ Windows PC (SlimeVR Server) のネットワーク越し構成のため、Windows 側に小さな relay app を立ててカスタムバイナリ TCP を受信→Protobuf に変換→pipe に書き込む。
> - **TrackerRole の制約で 8 本構成へ縮小**: SlimeVR Bridge の `TrackerRole.kt` enum には thigh / shin の細分化が無く `LEFT_KNEE / RIGHT_KNEE` 各 1 個しか持たない。Phase 11 の 10 本構成 (LEFT/RIGHT UPPER_LEG + LOWER_LEG) は **8 本** (LEFT/RIGHT_SHOULDER, CHEST, WAIST, LEFT/RIGHT_KNEE=shin, LEFT/RIGHT_FOOT) に縮める。SlimeVR コミュニティの一般的 6 本 FBT より腕 2 本多い構成。
> - **roll 改良は単体マージ可能**: M1 は Bridge 実装なしで `tracker_extract.cpp` 改修だけで完結する。Phase 11 の Firmware UDP 経路にも即時効く。M1 単独で実機評価して効果を見てから M2 以降に進む。

## Context

Phase 11 で SlimeVR Firmware UDP 経由の 10 本トラッカー (quaternion のみ) 経路が本流に乗ったが、実機運用で次の 2 つの症状が観測された:

1. **二の腕 (Upper Arm) が変に回転する**: `cpp/src/slimevr/tracker_extract.cpp:163-178` の上腕 quat は `forward = elbow − shoulder`, `up = neck − shoulder`。腕を前方挙上 / 頭上挙上すると forward と up が並行に近づき roll の決定権が失われる。境界付近では `quat_from_forward_up()` の degenerate 判定で `valid=false` → identity fallback → 前周期 smoothing 保持となり、roll が振動する。
2. **足 (Foot) が不安定**: `tracker_extract.cpp:241-252` の足 quat は `up = world Z` 固定で pitch / roll を強制ゼロ。つま先立ち / かかと立ち / 横傾きが全部潰れる。SlimeVR の IK 上、足の roll 誤りは足首接続点の滑り (foot slide) を引き起こす。

`docs/backlog-slimevr-bridge-relay.md` (Phase 11 着手時に積み課題として残したもの) には「位置を SlimeVR の IK に効かせるには Bridge protocol over Windows relay しかない」と既に整理済み。Phase 12 でこれを起動する。

ユーザー確認済の方針 (2026-05-22):
- 主症状は **両方** 解決したい (二の腕 + 足)
- Windows 側に新規 relay バイナリ常駐は **OK**
- SlimeVR の AutoBone (身長 / 骨比例の自動校正) は **走らせ済み** (= 設定起因の足滑り因子は除外)

ゴール:

**(A) 送信側 roll ロジック改良** — `tracker_extract.cpp` の up ベクトル選択を物理的に正しい候補に差し替える (二の腕 + 大腿 + 足)。Jetson 内完結。Firmware UDP / Bridge 両経路に効く (ただし大腿は Phase 11 Firmware UDP 経路にのみ効く)。

**(B) Bridge relay 経路新設** — Jetson 側に `bridge_publisher` を追加して位置 + 回転をカスタムバイナリ TCP で Windows へ送出。Windows 側に `slimevr-bridge-relay.exe` (C# .NET 8) を新設して受信 → SlimeVR Bridge Protobuf に変換 → Named pipe `\\.\pipe\SlimeVRInput` に流す。

完了条件は本文末 "## 完了条件" 節。

## アーキテクチャ

```
   Jetson (Linux)                                Windows PC (SlimeVR Server)
   ┌──────────────────────────────────┐         ┌──────────────────────────────────────┐
   │ pipeline::Skeleton3DBus (既存)    │         │ slimevr-bridge-relay.exe (新規)      │
   │   └ Halpe26 3D in world frame    │         │   ├─ TCP listen :6970                │
   │                                  │         │   ├─ frame parse                     │
   │   ─── extract_trackers() ───→    │         │   ├─ NamedPipeClientStream           │
   │     8× SlimeTracker              │         │   │     \\.\pipe\SlimeVRInput        │
   │     (world pos + wxyz quat)      │         │   ├─ Version + 8× TrackerAdded       │
   │                                  │         │   │     起動時 1 回                   │
   │   ─── BridgePublisher ───────→   │ ── TCP ─→  ├─ Position × 8 / frame (60 Hz)   │
   │     custom binary frame (60 Hz)  │  カスタム │   ├─ TrackerStatus(OK) keepalive     │
   │     Y-up Unity 系に変換済        │  binary  │   └─ Jetson 切断時 → DISCONNECTED   │
   └──────────────────────────────────┘         │                  ▼                   │
                                                │ SlimeVR-Server (既存、Bridge listen) │
                                                │   IK に位置 + 回転を反映            │
                                                │   SteamVR vive_tracker_* として       │
                                                │   8 本表示 (名前付き、AutoBone 継続) │
                                                └──────────────────────────────────────┘

  並列で残す経路 (Phase 11、排他):
   --slimevr-out --slimevr-port 6969 ──UDP──→ SlimeVR Firmware UDP receiver
                                                (10 本、quaternion のみ)
```

`BridgePublisher` の send スレッドは `NativePublisher` (`cpp/src/slimevr/native_publisher.cpp` の `send_thread`) の `steady_clock + sleep_until` pacing 構造を踏襲。データパスへの介入なし → スループット影響ゼロ。

## トラッカー定義 (8 本)

| # | TrackerRole (Bridge) | sensor_id | id | Pos joint (world) | Forward | Up |
|---|---|---|---|---|---|---|
| 0 | LEFT_SHOULDER  | 0 | 9  | midpoint(l_shoulder, l_elbow) | elbow − shoulder | **多段** (M1 参照) |
| 1 | RIGHT_SHOULDER | 1 | 10 | midpoint(r_shoulder, r_elbow) | elbow − shoulder | **多段** (M1 参照) |
| 2 | CHEST          | 2 | 4  | midpoint(neck, hip_center)    | ⊥(shoulder_axis × spine) | neck − hip_center |
| 3 | WAIST          | 3 | 1  | hip_center                    | ⊥(hip_axis × spine)      | neck − hip_center |
| 4 | LEFT_KNEE      | 4 | 5  | midpoint(l_knee, l_ankle)     | ankle − knee     | hip − knee (脛 = shin を送る) |
| 5 | RIGHT_KNEE     | 5 | 6  | midpoint(r_knee, r_ankle)     | ankle − knee     | hip − knee |
| 6 | LEFT_FOOT      | 6 | 2  | midpoint(l_heel, l_big_toe)   | big_toe − heel   | **ankle − foot_mid** (M1 改良) |
| 7 | RIGHT_FOOT     | 7 | 3  | midpoint(r_heel, r_big_toe)   | big_toe − heel   | **ankle − foot_mid** (M1 改良) |

- **LEFT/RIGHT_KNEE** は SlimeVR の vive_tracker_left/right_knee として認識される。中身は **脛 (lower_leg = shin)** の quat を入れる。理由: 足首から逆算する IK 上、shin の向きを与えたほうが膝下のリーチが正確に出る。thigh は IK が waist + knee 位置から復元できる。
- **HEAD** は HMD が供給するので送らない。**Forearms / Hands** は SteamVR コントローラ前提で送らない。
- Halpe26 必須。COCO17 では `neck(18)`, `hip_center(19)`, `heel(24,25)`, `big_toe(20,21)` が無いので `extract_trackers_bridge()` (新名) が `runtime_error` を投げる。

### M1: up ベクトル多段選択

**Upper Arm (二の腕)** — `upper_arm` ラムダの up は **1 段 (primary 単独、fallback なし)**:

1. `up1 = (wrist − elbow)` を `forward = (elbow − shoulder)` の直交成分に projection した残差。Halpe26 の手首 (`kLWrist = 9, kRWrist = 10`) を `tracker_extract.cpp:36` 付近の symbol テーブルに追加。前腕は肘 hinge により肩-肘軸と直交し続けるため、上腕 roll の最強の物理的手掛かり。
2. `sin θ < kRollSinLow` で primary degenerate なら **fallback なしで freeze**。`upper_arm` 内では `pick_up_multistage` の secondary/tertiary 引数に **零ベクトル `Vec3f{0,0,0}` を渡す** ことで関数末尾の `confidence = 0; return zero` 経路に落とし、`quat_from_forward_up` の sin θ-based degeneracy gate で `valid = false` を返させ、`apply_quat_smoothing` が前周期 quat を保持する経路に倒す。

**なぜ secondary / tertiary fallback を使わないか (Phase 13 修正 / 2026-05-25)**: 当初実装 (M1 単独 commit `a28f03c` 時点) は secondary に `(neck − shoulder)` (chest lateral pin)、tertiary に `world Z` を渡していたが、これは大腿で撤去した lateral pin 問題と同型の anti-pattern:

- **secondary (neck - shoulder)** は胸郭の lateral 軸そのもの。primary (wrist - elbow) が degenerate (= 腕を伸展して肘がほぼ伸びた瞬間) になると secondary が full confidence で支配し、**上腕 roll が胸郭 yaw に rigid 共有**される (体を捻ると腕も一緒に回って見える)。水平に伸ばした腕で symptomatic。
- **tertiary (world Z)** は水平腕 (`fwd ⊥ Z`) では `sin(worldZ, fwd) = 1` で confidence = 1.0 が割り当てられ、「肘が天井向き」の捏造 roll が確信度満点で書き込まれる。`world Z` は上腕 roll に対しては物理的に意味のある handle を持たない。
- primary 不在時の正しい挙動は **freeze (前周期保持)**、これは `upper_leg` で Phase 12 M1 / 2026-05-23 修正に確立済の方針。`upper_arm` も同じパターンに揃える。

既存の `detail::quat_from_forward_up()` は Phase 13 で sin θ-based degeneracy gate に拡張済 (旧 `norm(cross) < 1e-6` 絶対しきい → `sin θ < kRollSinLow` 相対しきい)。`apply_quat_smoothing()` の前周期保持挙動は維持。

**Foot (足)** — `foot_tracker` ラムダの up を:

- `up_raw = ankle − ((heel + big_toe) * 0.5)`。足甲方向 (足底平面の上向き法線寄り) になり、つま先立ち時は前傾、かかと立ち時は後傾、平らな時はほぼ world Z になる物理的に正しい up。
- `ankle` invalid のときのみ `world Z` にフォールバック。

**Upper Leg (大腿)** — `upper_leg` ラムダの up は **1 段 (primary 単独、world-Z fallback なし)**:

1. `up1 = (ankle − knee)`。膝 hinge により脛は thigh 軸と直交し続けるため、大腿 roll の物理的手掛かり (上腕の `wrist − elbow` と同型)。
2. `up1` が degenerate (sin θ < `kRollSinLow`) なら **fallback なしで freeze**。`upper_leg` 内では `pick_up_multistage` の tertiary 引数に **零ベクトル `Vec3f{0,0,0}` を渡す** ことで、関数末尾の `confidence = 0; return zero` 経路に落とし、`quat_from_forward_up` の `cross(zero, fwd) = zero` で `valid = false` を返させ、`apply_quat_smoothing` が前周期 quat を保持する経路に倒す。

**なぜ world Z fallback を使わないか (2026-05-25 修正)**: 当初実装 (M1 単独 commit `d6d2632` 時点) は tertiary に `world Z` を渡していたが、`pick_up_multistage` の tertiary 採用ロジックは「`i == 2` なら無条件採用 + confidence = smoothstep(sin θ, low, high)」であり、立位完全伸展 (`fwd ∥ worldZ`) では sin θ = 0 → confidence = 0 で freeze が偶然成立する一方、**非垂直な thigh で primary が degenerate になった瞬間に confidence = 1.0 で world-Z 由来の "knee が天井向き" な thigh roll が確信度満点で書き込まれる**バグになっていた (Codex review 2026-05-23 指摘)。

特に **日本式の直座り / 長座 / あぐらからの脚伸ばし** (床に座って脚を前方に伸ばし、膝を完全伸展させる) は屋内ポーズ計測では超頻出。このとき thigh fwd ≈ +Y / shin もほぼ +Y で primary が thigh 軸に並行 → degenerate、しかも fwd が world Z と直交 (`sin(worldZ, fwd) = 1`) なので confidence 1.0。avatar の thigh が "膝裏が床向き" の捏造 roll に張り付く症状を生む。`world Z` は thigh roll に対しては物理的に意味のある handle を持たない (= "knee が天井向き" の幾何的に valid だが任意の姿勢を返すだけ) ので、fallback として使ってはならない。

**設計改訂 (Phase 12 修正 — lateral pin 撤去)**: 当初は中間段に `up2 = (hip − hip_center)` (= Phase 11 現状の lateral pin) を置いていたが、これは骨盤フレームの lateral 軸そのものであり、primary が浅角度域 (sin θ がノイズで `kRollSinLow` 付近を行き来する状況) で degenerate 扱いになるたびに大腿 up が骨盤の lateral 軸へ rigid に張り付き、大腿 roll が骨盤 yaw に追従して両者が一緒に動いて見える現象が実機 (歩行 / 軽い屈伸) で発生した。secondary lateral pin を撤去し、primary が活きる間だけ大腿 roll を更新、活きない間は前周期保持に倒すことで骨盤との rigid 共有を断ち切った。`upper_leg` 内では `kHipCenter` を必須ジョイントから外し、`pick_up_multistage` 呼び出しは secondary 引数に primary を再渡しすることで 2 段選択 (実質 1 段) を表現する (`pick_up_multistage` シグネチャは upper_arm 側のために 3 段のまま維持)。

大腿の改修は **Phase 11 Firmware UDP 経路の 10 本構成** (LEFT/RIGHT_UPPER_LEG を送る) で効く。Phase 12 Bridge 経路の 8 本構成は thigh を送らない設計 (上記 R2 参照) のため無関係だが、Phase 11 経路を残す方針のため M1 に含める。

歩行 / 着座 (膝屈曲) / しゃがみで primary が活きて femur 軸回転を捉える。立位完全伸展、直座り、長座など primary が thigh 軸並行になるケースでは primary が degenerate するため confidence = 0 で前周期保持。立位での hip 外旋 (toes-out 動作) は 3D KP だけでは観測不能という原理的限界はそのままだが、その不可観測性は「骨盤に rigid に張り付く」形でも「world Z に張り付く」形でもなく「最後に観測した roll を持続する」形で表現される (`test_tracker_extract` の `test_thigh_standing_knee_straight` / `test_thigh_seated_extended_straight_knee` が `valid=false` + `roll_confidence=0` を期待する)。

### M1 の追加レイヤ: Confidence-modulated smoothing

ハード閾値 (`kUpperArmRollSwitch` / `kThighRollSwitch` = 0.2) によるソース切替は、閾値跨ぎで roll の discrete jump、浅角度でのノイズ増幅、固定 alpha smoothing で隠せない振動という 3 つの不安定性を残す (`a28f03c` 後のユーザー指摘)。これを **角度に応じた重み付け** で解消する:

- **ソース選択は緩い floor で**: `sin θ ≥ kRollSinLow = 0.05` (≈ sin 2.9°、ほぼ degenerate 検出だけ) を満たす最初のソースを採用する。primary が浅角度でも採用され続け (物理的に最も情報量が多い)、完全に死んだ時のみ secondary に降りる。
- **採用ソースの sin θ から信頼度を導出**: `roll_confidence = smoothstep(sin θ, kRollSinLow, kRollSinHigh)` で `[0, 1]` に写像。`kRollSinHigh = 0.20 (≈ sin 11.5°)` 以上は完全信頼 = 1.0。
- **smoothing alpha を per-tracker に**: `apply_quat_smoothing` 内で `effective_alpha = base_alpha · roll_confidence`。`confidence = 0` のとき前周期 quat をホールド、中間値では遅れて反映、`confidence = 1` で現状 (Phase 11 / M1 hard) と同速。
- **副次修正**: 旧 `apply_quat_smoothing` は `!valid` で `prev_quat` を identity にリセットしていた (一瞬の検出落ちで avatar が identity に snap する原因)。新版は `prev_quat` を保持し、`curr.quat_wxyz` を `prev_quat` に置き換えて publisher にも継続性を見せる。

胸 / 腰 / 脛 / 足 (= rigid pin or 常時非 degenerate) は `roll_confidence = 1.0` 固定 = 現状と同挙動。上腕 / 大腿のみ動的に減衰。

期待効果: 腕を T-pose で水平静止すると `sin θ ≈ 0.03` (KP ノイズ範囲) → confidence = 0 → 前周期保持で twist 振動消失。腕を 12° 屈曲以上 → confidence = 1 → 通常追従。中間屈曲では緩やかな反映で離散ジャンプなし。

## Bridge wire format (Jetson ↔ Windows relay)

依存追加なしの方針 (Phase 11 の `firmware_protocol.cpp` 同様、手書きバイナリ) に揃え、Protobuf を Jetson 側にリンクしない。relay 側で Protobuf に変換するため、Windows 側のみ `Google.Protobuf` NuGet 依存。

### 1 frame (ヘッダ + body + フッタ)

```
[ 4 BE u32 ] magic           = 0xF17AC0DE
[ 2 BE u16 ] version         = 1
[ 2 BE u16 ] tracker_count   = 8

  ×8 SlimeTracker {
    [ 1   u8  ] sensor_id     = 0..7
    [ 1   u8  ] valid_flags   bit0: valid, bit1-7: reserved
    [ 4 BE f32 ] pos_x         | already in SlimeVR Y-up Unity LH (meters)
    [ 4 BE f32 ] pos_y         |  (see firmware_protocol::world_pos_to_slime,
    [ 4 BE f32 ] pos_z         |   new helper added in M3)
    [ 4 BE f32 ] qw            | SlimeVR wire-form quaternion
    [ 4 BE f32 ] qx            |  (world_quat_to_slime applied)
    [ 4 BE f32 ] qy
    [ 4 BE f32 ] qz
  }

[ 8 BE u64 ] sequence        publisher 内で単調増加
[ 8 BE u64 ] ts_unix_ms      Jetson 側 system_clock now
```

合計サイズ = 4 + 2 + 2 + 8*(1+1+4*7) + 8 + 8 = **256 bytes / frame**。60 Hz × 8 tracker でも 15 KB/s 程度、TCP で全く問題なし。

座標変換は Jetson 側で済ませる方針 (relay は素通し):
- 既存 `firmware_protocol::world_quat_to_slime()` で `(qx, qz, -qy, -qw)` 入れ替え
- 新規 `firmware_protocol::world_pos_to_slime()` を追加: world (Z-up, X-right, Y-forward) → SlimeVR (Y-up, X-right, Z-forward) は `(x, z, -y)` の入れ替え。これで relay は frame をほぼそのまま Protobuf `Position` に詰め替えるだけになる。

### TrackerAdded / TrackerStatus (relay → SlimeVR)

接続成立時に Version + 8× TrackerAdded を 1 回だけ送る (TCP frame 自体には乗せず、relay が初回接続時に自動生成):

```protobuf
TrackerAdded {
  tracker_id     = sensor_id (0..7)
  tracker_serial = "fitra-cam-{role}"      // 例 "fitra-cam-LEFT_SHOULDER"
  tracker_name   = "fitra-cam {Role Name}" // 例 "fitra-cam Left Shoulder"
  tracker_role   = TrackerRole.<role>      // TrackerRole.kt の id 値
  manufacturer   = "fitra-cam"
}
```

毎 frame:
```protobuf
Position {
  tracker_id = sensor_id
  x/y/z      = wire frame の pos_xyz (valid_flags bit0 = 1 のときのみ)
  qx/qy/qz/qw = wire frame の quat
  data_source = FULL
}
TrackerStatus {
  tracker_id = sensor_id
  status     = OK (valid) または OCCLUDED (invalid)
  confidence = HIGH
}
```

`Position` の `x/y/z` は `optional`。invalid trackerは rotation のみ送って SlimeVR 側で前周期保持に任せる。

## ファイル構成

| ファイル | 役割 |
|---|---|
| `cpp/src/slimevr/tracker_extract.{hpp,cpp}` | **M1 改修**: upper_arm / foot の up 多段選択。**M2 拡張**: `extract_trackers_bridge()` で 8 本構成版を提供 (10 本版は Phase 11 用に残す) |
| `cpp/src/slimevr/firmware_protocol.{hpp,cpp}` | **M3 拡張**: `world_pos_to_slime(x,y,z)` ヘルパを追加 |
| `cpp/src/slimevr/bridge_publisher.{hpp,cpp}` | **M3 新規**: TCP socket + send スレッド + frame シリアライザ + reconnect ループ |
| `cpp/tools/test_tracker_extract.cpp` | **M1 拡張**: 6 ケース golden (T-pose / 前方挙上 / 頭上挙上 / つま先立ち / かかと立ち / 横傾き) |
| `cpp/tools/test_bridge_publisher.cpp` | **M3 新規**: frame format (magic / version / count / pacing / sequence) byte-level golden |
| `cpp/tools/dump_bridge_frames.py` | **M3 新規** (helper): Python asyncio TCP recv + 統計集計 (60Hz pacing 誤差, drop rate) |
| `windows/slimevr-bridge-relay/` | **M4 新規**: C# .NET 8 single-file exe。Bridge protocol への変換 |
| `windows/slimevr-bridge-relay/README.md` | **M4 新規**: ビルド手順 (`dotnet publish -c Release -r win-x64 --self-contained`)、運用手順 |

CMake:
- `fitra_slimevr` lib に `bridge_publisher.cpp` を追加
- `cpp/tools/CMakeLists.txt` に `test_bridge_publisher` を追加
- Windows 側は CMake 配下に置かず `dotnet` toolchain で独立ビルド

## CLI

```
--slimevr-bridge              Bridge relay 経由 publisher 起動 (bool, default off)
--slimevr-bridge-host ADDR    Windows relay の IP (default 127.0.0.1; 別 PC Windows なら IP 指定)
--slimevr-bridge-port N       TCP port (default 6970; Firmware UDP の 6969 と被らないように)
--slimevr-bridge-rate-hz F    送出周期 (default 60.0、最大 240)
--slimevr-bridge-quat-smooth F  per-tracker slerp alpha 0..1 (default 0.5; Phase 11 と同じ意味)
```

Gating (`main.cpp` で early-fail):

1. `--slimevr-bridge` ⇒ `--enable-3d` 必須
2. `--slimevr-bridge` ⇒ `--keypoint-format=halpe26` 必須
3. `--slimevr-bridge` ⇒ `--calibrate` と排他
4. **`--slimevr-bridge` と `--slimevr-out` の同時指定は reject** (二重登録防止)
5. port ∈ [1, 65535], rate ∈ (0, 240], smooth ∈ [0, 1]

## /stats3d スプライス

`crow_server.cpp` の `/stats3d` ハンドラに `"slimevr_bridge": {...}` を追加 (Phase 11 の `"slimevr": {...}` と並列):

```json
"slimevr_bridge": {
  "connect_count":      3,
  "disconnect_count":   2,
  "sent_frames":        12345,
  "skipped_invalid":    18,
  "tx_bytes":           3160320,
  "last_send_ms":       1715310123456.789,
  "tcp_state":          "connected"
}
```

## マイルストーン

| M | 内容 | 完了基準 |
|---|---|---|
| M1 | `tracker_extract.cpp` の upper_arm / upper_leg / foot up を多段選択 + confidence-modulated smoothing に書き換え + `test_tracker_extract.cpp` に 9 ケース pose golden + 4 ケース confidence (confidence@90° / 全 degenerate / smoothstep midrange / 凍結 freeze) + 1 ケース挙動変更 (invalid 時の prev 保持) | ctest pass, Phase 11 Firmware UDP 経路で実機評価して 二の腕ひねり症状の解消 + 歩行 / 着座で大腿 roll が SlimeVR GUI 上で追従 + **腕完全伸展時の roll twist 振動がゼロに収束** |
| M2 | `docs/phase12-slimevr-bridge-relay.md` 新規 + `docs/backlog-slimevr-bridge-relay.md` に「Phase 12 で実装中、こちらを参照」のリダイレクト注記 + `docs/cpp-migration-plan.md` の段階実装 / 検証戦略行追加 | docs review |
| M3 | `firmware_protocol::world_pos_to_slime()` 追加 + `extract_trackers_bridge()` (8 本版) 追加 + `bridge_publisher.{hpp,cpp}` 新規 + `test_bridge_publisher` で wire format golden 通過 + `main.cpp` CLI 配線 + 二重登録ガード + `/stats3d` splice | ctest pass, `tools/dump_bridge_frames.py` で Jetson loopback frame 60Hz pacing 誤差 < 5ms / 抜け率 < 0.1% |
| M4 | `windows/slimevr-bridge-relay/` 新規 (C# .NET 8): TCP recv + Protobuf 変換 + Named pipe write + reconnect ループ + `--no-pipe` dry-run mode | Windows 機で relay 単体起動 → SlimeVR GUI に 8 本表示 (Jetson 接続せずダミー frame 送信で確認) |
| M5 | Jetson 実機 → Windows relay → SlimeVR Server の E2E 通電 | SlimeVR GUI に 8 本名前付き表示、AutoBone 既存設定が持続、T-pose 静止 30s で誤差 < 5° |
| M6 | E2E 実機評価 (歩行 / 腕回旋 / つま先立ち) | 足 slide < 2cm/step、二の腕 roll 肉眼追従、`recent_pose_fps` 回帰 < 2% |
| M7 | `docs/cpp-migration-plan.md` の段階実装 / 検証戦略を Phase 12 完了行で確定 + backlog 削除可否判断 | doc final, branch merge |

## 検証

### 単体 (実機不要)

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build -j
ctest --test-dir cpp/build --output-on-failure
# test_tracker_extract (M1 拡張), test_bridge_publisher (M3 新規) が pass
# test_firmware_protocol / test_triangulator も pass 維持
```

### 統合 (Jetson 単体、Windows 機不要)

```bash
# Terminal A — dump receiver
python3 cpp/tools/dump_bridge_frames.py --port 6970

# Terminal B — Jetson 実機
./cpp/build/main --enable-3d --keypoint-format=halpe26 \
    --slimevr-bridge --slimevr-bridge-host=127.0.0.1 --slimevr-bridge-port=6970 \
    --cam0 /dev/v4l/by-path/... --cam1 /dev/v4l/by-path/... \
    --calib calibrations/<id>/calibration.yaml \
    --det-engine engines/yolox.engine --pose-engine engines/rtmpose.engine
```

`dump_bridge_frames.py` 側で magic / version / tracker_count / sequence 単調性 / 60Hz pacing 誤差 (< 5ms p95) / drop 率 (< 0.1%) を集計。

### Windows relay 単体テスト (Jetson 不要)

```bash
# Windows 機
slimevr-bridge-relay.exe --port 6970 --no-pipe --self-test
# → 内部生成のダミー frame を Protobuf 化して stdout に hex dump
# → SlimeVR Server 不要

# 続いて SlimeVR Server 起動 + relay 通常モード
slimevr-bridge-relay.exe --port 6970
# → 同じ自己テスト frame で SlimeVR GUI に 8 本表示確認
```

### E2E (Windows SlimeVR 機が必要 — M5/M6)

1. Windows 機の IP を確認 (例 `192.168.1.50`)。SlimeVR Server 起動 → `slimevr-bridge-relay.exe --port 6970` を起動
2. Jetson 側:
   ```bash
   ./cpp/build/main --enable-3d --keypoint-format=halpe26 \
       --slimevr-bridge --slimevr-bridge-host=192.168.1.50 \
       --cam0 ... --cam1 ... --calib ... --det-engine ... --pose-engine ...
   ```
3. SlimeVR GUI で **8 個のトラッカー** が `fitra-cam Left Shoulder` / `fitra-cam Right Shoulder` / `fitra-cam Chest` / `fitra-cam Waist` / `fitra-cam Left Knee` / `fitra-cam Right Knee` / `fitra-cam Left Foot` / `fitra-cam Right Foot` で **名前付き自動表示** (連番ならない、AutoBone 既存設定が持続)
4. 被験者の腕上げ / しゃがみ / 歩行 / つま先立ち で 8 トラッカーが破綻なく追従し、SteamVR Avatar の FBT が成立
5. `curl http://<jetson-ip>:8000/stats3d` の `"slimevr_bridge"` ブロックで:
   - `connect_count` = 1, `disconnect_count` = 0 (steady state)
   - `sent_frames` が steady-state で 60/s 程度
   - `tcp_state` = `"connected"`
   - `aggregate.recent_pose_fps` 低下 < 2%
6. Jetson を再起動して再接続 → relay が自動で再 TrackerAdded → SlimeVR 側のトラッカー配置が serial で一致 (AutoBone 維持)

## 完了条件

1. M1: 二の腕ひねりが Phase 11 経路 (Firmware UDP) でも肉眼で改善
2. M3: Jetson 単体で Bridge frame が 60 Hz で steady に出る (Windows 不要)
3. M5: Windows relay 経由で 8 本トラッカーが名前付き表示される
4. M6: 足 slide が肉眼で減少、`recent_pose_fps` 回帰 < 2%
5. `docs/cpp-migration-plan.md` の段階実装行 / 検証戦略行に Phase 12 が反映される

## リスク・残課題

| ID | 内容 | 対応 |
|---|---|---|
| R1 | M1 の confidence smoothstep 範囲 (`kRollSinLow = 0.05`, `kRollSinHigh = 0.20`) が実機で妥当か | M6 で実機 1 週間運用後に確定。両定数は `tracker_extract.cpp` 内 named constant として宣言済みでチューニング容易。Low を上げすぎると追従が鈍くなる; High を下げすぎると伸展付近で update が早すぎて振動が戻る |
| R2 | LEFT_KNEE に shin を入れる選択が SlimeVR IK で逆効果になるケース | thigh 版に切替できるよう `tracker_extract.cpp` で role→joints マップを変数化。M6 評価時に shin vs thigh を比較 |
| R3 | Bridge `TrackerRole.NONE(0)` で 10 本送る案 (案 Y) が必要になった場合 | `~/Documents/refs/slimevr/SlimeVR-Server` の `ProtobufBridge.kt` を読んで NONE role の表示挙動を確認。GUI 上で連番表示に戻るなら不採用 |
| R4 | TCP 接続切断時の挙動 | `bridge_publisher` 側は exponential backoff (100ms → 5s) で reconnect。relay 側は pipe 切断時も Jetson TCP を維持。`tcp_state` を stats で監視可能に |
| R5 | Windows .NET ランタイム前提が運用上の負担になる | `dotnet publish --self-contained -r win-x64` で single-file exe を出す。ユーザは exe ファイル 1 個だけを置けばよい |
| R6 | Firmware UDP 経路 (Phase 11) を残しているため、運用上どちらを使うか迷う | `docs/cpp-migration-plan.md` に「位置を VR 上で活かしたいなら Bridge、ローカル PC 完結なら Firmware UDP」の指針を明記 |

## 関連ドキュメント

- [`phase11-slimevr-integration.md`](phase11-slimevr-integration.md) — Phase 11 で本流に乗せた Firmware UDP (回転のみ) 経路
- [`backlog-slimevr-bridge-relay.md`](../backlog-slimevr-bridge-relay.md) — 本 phase の前身となる積み課題ドキュメント (Phase 12 完了時に削除可否を判断)
- [`cpp-migration-plan.md`](../cpp-migration-plan.md) — 全体ロードマップと検証戦略
- [`phase9-halpe26-migration.md`](phase9-halpe26-migration.md) — 前提となる Halpe26 移行

## 参照リソース (clone 済)

- `~/Documents/refs/slimevr/SlimeVR-Feeder-App/ProtobufMessages.proto` — Bridge protocol Protobuf スキーマ正本
- `~/Documents/refs/slimevr/SlimeVR-Server/server/core/src/main/java/dev/slimevr/tracking/trackers/TrackerRole.kt` — TrackerRole enum 定義 (id 値の根拠)
- `~/Documents/refs/slimevr/SlimeVR-Server/server/desktop/src/main/java/dev/slimevr/desktop/platform/ProtobufBridge.kt` — Server 側 Bridge handler (relay 動作確認用)
