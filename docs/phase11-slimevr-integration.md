# Phase 11 — SlimeVR (VMC/OSC) 姿勢情報連携

> **実装メモ (2026-05-20)**: 本ドキュメントは設計時点の正本。実装時点で以下の差分が判明したので、行間で参照する際は注意:
>
> 1. **Phase 10 はスキップ**。`--mode {pose,calib-*}` フラグは存在しないため、Phase 11 の launch gate は `--slimevr-out` && `--enable-3d` && `--keypoint-format=halpe26` && !`--calibrate` の 4 条件で判定 (main.cpp で early-fail)。
> 2. **`CrowServer::publisher_loop` の行範囲**は `cpp/src/web/crow_server.cpp:301-334` (本文中の 179-212 から drift)。pacing 構造の参照は実装ファイルで確認すること。
> 3. **VMC OSC quaternion order は xyzw on wire** ([protocol.vmc.info](https://protocol.vmc.info/english.html))。`VmcTracker::quat_wxyz` で wxyz 格納、publisher は `quat_wxyz[1..3, 0]` で xyzw に並べ替えて送信。
> 4. **`/stats3d` 露出方法**: 設計案は CrowServer の `/stats` に追加だったが、3D 関連 stats が `/stats3d` 側にまとまっているので publisher の stats も `/stats3d` JSON 末尾に `"slimevr":{...}` として spliceさせている。`/stats` は変更なし。

## Context

Phase 7 で多視点三角測量 + Kalman + IK による 3D skeleton 出力が安定し、Phase 8 で被験者プロファイルによるボーン長固定、Phase 9 で Halpe26 移行、Phase 10 で 3 カメラ + 完全 C++ キャリブが揃った。Phase 11 のゴールは **この 3D skeleton を SlimeVR Server に流し込み、SteamVR / VRChat の Full-Body Tracking (FBT) 用 tracker として使えるようにする** こと。これにより、IMU ベースの SlimeVR/owoTrack を補完する「カメラ駆動 FBT ソース」が成立する。

調査で確定した方針:

- **プロトコル: VMC over OSC** (SlimeVR-Server の `osc/VMCHandler.kt` が `/VMC/Ext/Tra/Pos` と `/VMC/Ext/Root/Pos` を ingest)。Native SlimeVR UDP (packet 17) は handshake / MAC / seq 管理が重く、実装コスト 2-3 日対 VMC 1 日。SlimeVR solver の挙動が悪ければ後で native へ昇格できる余地は残す。
- **Tracker 構成: 8-point full** = WAIST, CHEST, LEFT_FOOT, RIGHT_FOOT, LEFT_KNEE, RIGHT_KNEE, LEFT_ELBOW, RIGHT_ELBOW (HEAD は HMD が出す前提で除外)。VRChat OSC trackers の上限 8 と SteamVR FBT のフル構成を満たす。
- **前提: Phase 9 (Halpe26) 完了後に着手**。Halpe26 の neck(18) / hip_center(19) を直接 WAIST/CHEST 位置に使い、midpoint 補間誤差を回避する。heel(24/25) と big_toe(20/21) で **足の yaw を初めて出せる** ため、SlimeVR の foot tracker 品質が COCO17 時点と比較して大幅に上がる。
- **依存追加なし**: OSC 1.0 wire format は trivial (32-bit length-prefixed buffer + `,fff...` typetag) なので、Phase 8/10 の「JSON は std::ostringstream で手書き」方針に合わせて **raw UDP socket + 手書き OSC シリアライザ** で実装。liblo / oscpp の apt/FetchContent 追加は不要。
- **Coordinate system**: 我々の world frame (Z-up, X-right, meters, 床原点) → VMC = Unity 左手座標 (Y-up, Z-forward, meters)。位置: `(x, z, -y)`。Quaternion: `(qx, qz, -qy, -qw)` (左右系反転)。詳細は M3 の Quat 合成ルールに記載。

## ゴール / 完了条件

1. `--slimevr-out` フラグで VMC OSC publisher が立ち上がり、3D skeleton が利用可能な間 60 Hz で `/VMC/Ext/Tra/Pos` を 8 tracker 分送出。
2. SlimeVR Server の OSC Receiver UI に 8 trackers が "VMC receiver" デバイスとして自動認識される。
3. 各 tracker の position が世界座標 (床基準) で表示され、被験者の歩行・しゃがみ・腕の上げ下げに対応して動く。
4. Quaternion 合成により、肘・膝・足首が SlimeVR avatar 上で破綻なく曲がる (静止 T ポーズで identity rotation の二乗誤差 < 5°)。
5. `--enable-3d` が無効 / IK locked = false の間は publisher が静止する (異常な tracker 位置を送らない)。
6. 既存 Phase 6b の 170 fps ベンチに対し、aggregate `recent_pose_fps` 低下 < 2% (sender は別スレッドで polling、データ生成パスに介入しない)。

## アーキテクチャ

```
        ┌──── pipeline::Skeleton3DBus (既存) ────┐
        │  ::update(Skeleton3DSnapshot)          │
        │  ::make_bundle_json()                   │
        └─────────────┬────────────────────┬──────┘
                      │                    │
                      ▼                    ▼
              CrowServer publisher    SlimeVrPublisher (新規)
              30 Hz → /ws3d           60 Hz → UDP/OSC → SlimeVR Server
```

新しい publisher は `CrowServer::publisher_loop` (`cpp/src/web/crow_server.cpp:179-212`) の **構造をそのまま真似る**: 独立スレッドで `sleep_until` の periodic pacing、毎周期 `Skeleton3DBus` の最新スナップショットを lock 越しに取得 (今回は `make_bundle_json()` ではなく **新規 `Skeleton3DBus::snapshot()` getter** を生やして生の `Skeleton3DSnapshot` を取る)、変換、UDP `sendto`。

データパスへの介入なし → スループット影響ゼロ。

## マイルストーン

### M1 — `Skeleton3DBus::snapshot()` 公開 (土台)

**修正ファイル**: `cpp/src/pipeline/snapshot.{hpp,cpp}`

`Skeleton3DBus` は現状 `make_bundle_json()` だけが出口で、生の `Skeleton3DSnapshot` を外部から取り出せない。VMC publisher は JSON ではなく **構造体のまま** quaternion 合成等を行うので、lock-protected な getter を追加する:

```cpp
// snapshot.hpp
class Skeleton3DBus {
 public:
    ...
    Skeleton3DSnapshot snapshot() const;   // 新規。lock 内で値コピーして返す
};
```

```cpp
// snapshot.cpp
Skeleton3DSnapshot Skeleton3DBus::snapshot() const {
    std::lock_guard<std::mutex> lk{mu_};
    return snapshot_;   // value copy, ~17 Joint3D + stats ≈ 350 bytes
}
```

`make_bundle_json()` は内部で `snapshot()` を呼ばずに従来通り直接 `snapshot_` を見る (lock 取得を 1 回に抑えるため)。

**検証**: 既存 `/ws3d` 経路が回帰なし。新 getter は `dump_keypoints_3d` 等のオフラインツールから将来便利。

---

### M2 — OSC 1.0 wire format シリアライザ

**新規ファイル**: `cpp/src/slimevr/osc_writer.{hpp,cpp}`

OSC 1.0 の wire format は単純なので 100 行で完結する:

```cpp
namespace fitra::slimevr {
class OscWriter {
 public:
    void clear();
    void begin_message(std::string_view address);    // OSC string + typetag prep
    void add_float(float v);                         // big-endian 32-bit
    void add_int(int32_t v);
    void add_string(std::string_view s);
    void end_message();                              // pad addr/typetag to 4-byte
    // OSC Bundle (#bundle + 64-bit ts + element-prefixed messages)
    void begin_bundle(uint64_t osc_timetag_ntp);
    void end_bundle();
    std::span<const uint8_t> data() const;
 private:
    std::vector<uint8_t> buf_;
    std::vector<std::size_t> bundle_size_pos_;
};
}
```

仕様:
- すべての要素は 4-byte 境界に zero-padded。
- OSC string: `\0` terminator 必須、その後 4-byte 境界まで `\0` パディング。
- Typetag は `,` で始まり、message body と同様にパディング。
- Bundle 要素サイズは 32-bit big-endian で要素先頭に書く (end_bundle で back-patch)。

**検証**:
- `cpp/tools/test_osc_roundtrip.cpp` (新規) — hand-crafted バイト列との一致テスト。
- `python3 -c "from pythonosc.dispatcher import Dispatcher; ..."` でローカルにダンプしたバイナリを読み戻して値一致を確認 (テスト用、Python 依存はテストだけ)。
- 既知 OSC ライブラリのパケット (例: `osc_pkt::OutboundPacketStream` の出力) とバイト単位で一致。

依存追加なし: `std::vector<uint8_t>` + `std::byteswap`(C++23 未使用なら手書き bswap)。

---

### M3 — Tracker 抽出と quaternion 合成

**新規ファイル**: `cpp/src/slimevr/tracker_extract.{hpp,cpp}`

入力: `infer::Skeleton3D` (Halpe26 索引前提、`active_keypoint_format() == Halpe26` をアサート)。
出力: `std::array<VmcTracker, 8>`、各 `VmcTracker { std::string role; cv::Vec3f pos; cv::Vec4f quat_wxyz; bool valid; }`。

**Position の決定** (Halpe26):

| Role | Position |
|---|---|
| WAIST | `joints[19]` (hip-center) |
| CHEST | `joints[18]` (neck) |
| LEFT_KNEE | `joints[13]` |
| RIGHT_KNEE | `joints[14]` |
| LEFT_ELBOW | `joints[7]` |
| RIGHT_ELBOW | `joints[8]` |
| LEFT_FOOT | `0.5 * (joints[24] + joints[20])` (heel + big_toe で足底中心) |
| RIGHT_FOOT | `0.5 * (joints[25] + joints[21])` |

(COCO17 fallback: 仕様外。Phase 9 完了前提。`assert(kp_format == Halpe26)` で起動時拒否。)

**Quaternion 合成** (orthonormal frame → wxyz quaternion):

各 tracker について「forward (z-axis)」「up (y-axis)」のヒントベクトルを定義し、`x = cross(up, forward)`、`y = cross(forward, x)` で正規直交基底を作って 3×3 行列を quaternion に変換 (`cv::Rodrigues` ではなく Shoemake's matrix-to-quat 直接実装)。

| Role | Forward (z+) | Up (y+) |
|---|---|---|
| WAIST | `joints[18] - joints[19]` (hip→neck, spine 上向き) を **y** に、`joints[12] - joints[11]` (r_hip→l_hip 反転で **x**) → **z = x×y** | 上記 spine 方向 |
| CHEST | `joints[18] - joints[19]` を **y**、肩線 `joints[5] - joints[6]` (l→r 反転) を **x** | spine |
| LEFT_KNEE | `joints[15] - joints[13]` (knee→ankle, 下腿方向) | `joints[11] - joints[13]` (knee→hip, 大腿軸) を up に投影 |
| RIGHT_KNEE | 同 (右側) | 同 |
| LEFT_ELBOW | `joints[9] - joints[7]` (elbow→wrist) | `joints[5] - joints[7]` (elbow→shoulder) |
| RIGHT_ELBOW | 同 (右側) | 同 |
| LEFT_FOOT | `joints[20] - joints[24]` (heel→big_toe, つま先方向) | world up `(0,0,1)` (足は yaw のみ意味あり) |
| RIGHT_FOOT | `joints[21] - joints[25]` | 同 |

縮退時 (forward ≈ up 平行、または `valid=false` 関節を含む) は `quat = identity (1,0,0,0)` + `valid = false`。VMC publisher 側で valid=false の tracker はその周期スキップ (前回値保持を SlimeVR 側に任せる)。

**Quaternion exponential smoothing**: 連続フレーム間で `slerp(prev, curr, alpha)` (alpha = 0.5 既定、`--slimevr-quat-smooth` で調整)。位置側は既に Kalman 済みなので追加平滑化しない。

**Coordinate transform (world → Unity 左手 Y-up)**:

我々の world は `(X=right, Y=forward, Z=up, meters, 床=Z0)`。VMC = Unity 左手 = `(X=right, Y=up, Z=forward)`。Quat も座標系反転。

```
pos_vmc  = ( px, pz, -py )
quat_vmc = ( qx_unity, qy_unity, qz_unity, qw_unity )
         = ( qx, qz, -qy, -qw )    // Z/W flip 規約は VMCHandler.kt と一致
```

`VMCHandler.kt` の "Z/W flip" 規約と完全一致させるテストを M5 に含める。

**検証**: `cpp/tools/test_tracker_extract.cpp` — 合成 Halpe26 skeleton (T ポーズ、A ポーズ、片足上げ) を入力し、出力 quat を Unity 慣性軸で 3×3 行列に戻して visual inspection (期待値テーブル付き unit test)。

---

### M4 — VMC Publisher スレッド

**新規ファイル**: `cpp/src/slimevr/vmc_publisher.{hpp,cpp}`

```cpp
namespace fitra::slimevr {

struct VmcPublisherOptions {
    std::string host           = "127.0.0.1";
    uint16_t    port           = 39539;          // VMC default
    double      send_rate_hz   = 60.0;
    double      quat_smooth    = 0.5;
    bool        send_root_pos  = true;           // /VMC/Ext/Root/Pos
    bool        send_time      = true;           // /VMC/Ext/T (loop time)
};

class VmcPublisher {
 public:
    VmcPublisher(pipeline::Skeleton3DBus& bus, VmcPublisherOptions opts);
    ~VmcPublisher();
    void start();
    void stop();
    struct Stats { uint64_t sent_bundles=0; uint64_t skipped_invalid=0;
                   double last_send_ms=0; double last_rt_quat_max_dev_deg=0; };
    Stats stats() const;
 private:
    void loop();
    pipeline::Skeleton3DBus& bus_;
    VmcPublisherOptions opts_;
    int                 sock_fd_ = -1;
    std::thread         thread_;
    std::atomic<bool>   stop_{false};
    std::array<cv::Vec4f, 8> prev_quat_{};   // for smoothing
    OscWriter           writer_;
    mutable std::mutex  stats_mu_;
    Stats               stats_;
};

}
```

**Loop の構造** (`CrowServer::publisher_loop` 完コピ + 修正):

```cpp
void VmcPublisher::loop() {
    using clock = std::chrono::steady_clock;
    auto period = std::chrono::duration<double>(1.0 / std::max(opts_.send_rate_hz, 1.0));
    auto next = clock::now();
    while (!stop_.load()) {
        next += std::chrono::duration_cast<clock::duration>(period);
        std::this_thread::sleep_until(next);
        if (stop_.load()) break;

        auto snap = bus_.snapshot();
        if (!snap.stats.enabled || snap.persons.empty()) { ++stats_.skipped_invalid; continue; }
        if (!snap.stats.ik_locked) { ++stats_.skipped_invalid; continue; }  // ボーン長未確定なら送らない

        auto trackers = extract_vmc_trackers(snap.persons.front());
        apply_quat_smoothing(trackers, prev_quat_, opts_.quat_smooth);

        writer_.clear();
        writer_.begin_bundle(/*ntp ts of now*/);
        if (opts_.send_time)     { writer_.begin_message("/VMC/Ext/T"); writer_.add_float(loop_time_sec); writer_.end_message(); }
        if (opts_.send_root_pos) { writer_.begin_message("/VMC/Ext/Root/Pos"); /* 0,0,0, identity quat */ writer_.end_message(); }
        for (auto& t : trackers) {
            if (!t.valid) continue;
            writer_.begin_message("/VMC/Ext/Tra/Pos");
            writer_.add_string(t.role);              // "waist", "leftFoot", ...
            writer_.add_float(t.pos[0]); writer_.add_float(t.pos[1]); writer_.add_float(t.pos[2]);
            writer_.add_float(t.quat[1]); writer_.add_float(t.quat[2]); writer_.add_float(t.quat[3]); writer_.add_float(t.quat[0]);  // qx qy qz qw order
            writer_.end_message();
        }
        writer_.end_bundle();

        ::sendto(sock_fd_, writer_.data().data(), writer_.data().size(), 0,
                 (sockaddr*)&sa_, sizeof sa_);
        ++stats_.sent_bundles;
    }
}
```

**Socket**: `socket(AF_INET, SOCK_DGRAM, 0)` + `sendto`。`SO_BROADCAST` 不要、`connect()` でデフォルト宛先固定オプションあり。`opts_.host` を `inet_pton` で `sa_in`、ctor で 1 回だけ。

**起動失敗時**: socket() / inet_pton() が失敗したら publisher は起動しない (CLI で warn ログ + 続行)。pose pipeline は影響を受けない。

**スレッド寿命**: `main.cpp` で `MultiCameraDriver` 起動の **後** に start、停止の **前** に stop。`CrowServer` と並列存在。

---

### M5 — CLI 追加 + ライフタイム

**修正ファイル**: `cpp/src/main.cpp`

新規フラグ:
- `--slimevr-out` (bool) — VMC publisher を起動 (既定 OFF)
- `--slimevr-host <ip>` (既定 `127.0.0.1`)
- `--slimevr-port <port>` (既定 `39539`)
- `--slimevr-rate-hz <float>` (既定 `60.0`)
- `--slimevr-quat-smooth <0..1>` (既定 `0.5`)

主要結線:
```cpp
std::unique_ptr<slimevr::VmcPublisher> vmc_pub;
if (args.slimevr_out) {
    if (!args.enable_3d) FITRA_LOG_WARN("--slimevr-out requires --enable-3d, skipping");
    else if (args.keypoint_format != KeypointFormat::Halpe26)
        FITRA_LOG_FATAL("--slimevr-out requires --keypoint-format=halpe26");
    else {
        vmc_pub = std::make_unique<slimevr::VmcPublisher>(*bus3d, opts);
        vmc_pub->start();
    }
}
// ... driver/server.run() ...
// stop order: server -> vmc_pub -> driver
```

`--slimevr-out` を `--mode pose` のときだけ有効化。`--mode calib-*` 時は CLI でエラー。

---

### M6 — Stats / 診断

**修正ファイル**: `cpp/src/pipeline/snapshot.{hpp,cpp}` (or 別ルート)

Skeleton3D stats に **VMC 送信側の指標を直接書かない** (双方向結合になる)。代わりに `VmcPublisher::stats()` を `crow_server` の `/stats` JSON に **任意で** 露出する経路を作る:

修正: `cpp/src/web/crow_server.{hpp,cpp}`
- `CrowServer` に optional `slimevr::VmcPublisher* vmc_pub` ポインタを set できるようにする
- `/stats` JSON 末尾に `"slimevr": {"sent_bundles":N, "skipped_invalid":M, "last_send_ms":F}` を追加 (publisher が nullptr なら省略)

これにより `web/dual_rtmpose/app.js` 側で SlimeVR 送信状況を表示する余地が残る (本フェーズ範囲外、Phase 12 候補)。

**ログ**: 5 秒ごとに `[slimevr] sent=NNN skipped=MMM rate=XX.XHz` を spdlog で吐く。

---

### M7 — 検証ツール: ローカル OSC ループバック

**新規ファイル**: `cpp/tools/slimevr_loopback.cpp`

スタンドアロン CLI:
- UDP `127.0.0.1:39539` を listen
- 受信した OSC bundle をパースして `address, types, args` で stdout に dump
- `--seconds 10` で自動終了

これで SlimeVR Server を起動していない状態でも `vmc_publisher` の wire 出力を目視確認できる。期待出力例:

```
[bundle ts=...]
  /VMC/Ext/T  ,f  3.412
  /VMC/Ext/Root/Pos  ,sfffffff  Root 0 0 0 0 0 0 1
  /VMC/Ext/Tra/Pos  ,sfffffff  waist     0.02 0.94 -0.10 0.01 0.03 -0.02 -0.999
  /VMC/Ext/Tra/Pos  ,sfffffff  leftFoot  0.12 0.05  0.30 ...
  ...
```

---

### M8 — 実機検証

1. **ループバック単体**:
   ```bash
   ./cpp/build/main --mode pose --keypoint-format halpe26 \
     --cam0 ... --cam1 ... --cam2 ... --enable-3d --calib calibrations/.../cam_params.yaml \
     --subject-id subj01 --slimevr-out --slimevr-port 39539 &
   ./cpp/build/tools/slimevr_loopback --port 39539 --seconds 30
   ```
   → 8 trackers × 60 Hz ≈ 480 messages/sec が届くこと、位置値が動的に変動。

2. **SlimeVR Server 連携** (Jetson 外 PC の SlimeVR Server を使用):
   - SlimeVR Server の OSC Receiver 設定で port=39539, "VMC Protocol" を ON。
   - Jetson 側で `--slimevr-host <pc-ip>` 起動。
   - SlimeVR UI で 8 trackers が "VMC receiver" デバイスとして自動認識されること。
   - T ポーズで Autobone → SteamVR で full-body avatar が動くこと (主観評価)。

3. **回帰 (パフォーマンス)**:
   - `--slimevr-out` ON/OFF で aggregate `recent_pose_fps` 差 < 2%。
   - `top -H -p <pid>` で `vmc_publisher` スレッドの CPU < 3%。

4. **Quaternion 妥当性**:
   - 静止 T ポーズ 5 秒の subject 録画で quat の標準偏差 < 1° (Kalman 後 + smoothing 後)。
   - WAIST quat × CHEST quat^-1 が概ね identity (脊柱が回転していない)。

5. **Correctness 回帰** (`outputs/recorded_rtmpose/20260515_064342/raw_cam{0,1}.mp4` 30 frames):
   - Phase 1 / Phase 5 / Phase 9 と同様、`dump_keypoints` で kpt L2 < 1.5 px (Halpe26 で再測定)。Phase 11 は infer 層に触れないので不変。

---

## 既存資産の再利用

- `cpp/src/web/crow_server.cpp:179-212` の `publisher_loop` 構造 — VMC publisher の periodic pacing をそのまま流用。
- `cpp/src/pipeline/snapshot.cpp:107-111` の `Skeleton3DBus::update` lock パターン — M1 の `snapshot()` getter で同じ mutex を共有。
- `cpp/src/lift/skeleton_def.hpp` の Halpe26 索引定数 (Phase 9 で配置済) — tracker extract で直接参照。
- `cpp/src/infer/types.hpp::Joint3D{x,y,z,score,valid}` — 既存定義のまま消費。
- `cpp/src/pipeline/calibration_session.cpp` の `tmp + rename` パターンは不要 (publisher はファイル書かない)。
- OpenCV `cv::Vec3f` / `cv::Vec4f` — 既存依存内、quat 合成と matrix→quat 変換に利用。

## 変更ファイル一覧

**新規**:

| ファイル | 役割 |
|---|---|
| `cpp/src/slimevr/osc_writer.{hpp,cpp}` | OSC 1.0 wire format 手書きシリアライザ |
| `cpp/src/slimevr/tracker_extract.{hpp,cpp}` | Halpe26 → 8 VMC trackers (位置 + 合成 quaternion) |
| `cpp/src/slimevr/vmc_publisher.{hpp,cpp}` | UDP 60Hz publisher スレッド |
| `cpp/tools/slimevr_loopback.cpp` | ローカル受信ダンプ CLI (検証用) |
| `cpp/tools/test_osc_roundtrip.cpp` | OSC wire format 単体テスト |
| `cpp/tools/test_tracker_extract.cpp` | quat 合成 + 座標変換単体テスト |

**修正**:

| ファイル | 変更 |
|---|---|
| `cpp/src/pipeline/snapshot.{hpp,cpp}` | `Skeleton3DBus::snapshot()` getter 追加 (M1) |
| `cpp/src/main.cpp` | `--slimevr-*` フラグ群、`VmcPublisher` の起動・停止結線 (M5) |
| `cpp/src/CMakeLists.txt` | 新規 `fitra_slimevr` ライブラリ追加 (OpenCV のみリンク)、`fitra_pipeline` に追加 |
| `cpp/tools/CMakeLists.txt` | `slimevr_loopback`, `test_osc_roundtrip`, `test_tracker_extract` の add_executable |
| `cpp/src/web/crow_server.{hpp,cpp}` | optional `VmcPublisher*` 受け取り、`/stats` JSON で stats 露出 (M6) |
| `docs/cpp-migration-plan.md` | 「段階実装」に Phase 11 追記 (完了条件 = 8 trackers が SlimeVR で動く) |
| `docs/phase11-slimevr-vmc-integration.md` (新規 docs) | Phase 11 設計の正本 (本プラン file から派生) |

**不変** (依存追加なし):
- `python/` 配下 — 参照実装としてのまま (SlimeVR 連携は C++ 専用)
- `web/dual_rtmpose/` — frontend は無改修
- `cpp/src/infer/*`, `cpp/src/lift/*` — infer / lift 層に介入しない

## リスクと緩和

| ID | リスク | 緩和 |
|---|---|---|
| R1 | VMC Z/W flip 規約の解釈ミス | `VMCHandler.kt` の inverse 変換と完全一致する unit test を M3 に追加 |
| R2 | 合成 quaternion が SlimeVR avatar 上で破綻 | M3 で smoothing alpha を可変、T/A/lunge ポーズの目視ベンチを M8 に明記。失敗時は native UDP packet 17 + identity rotation + 位置のみ送る fallback を Phase 11b で検討 |
| R3 | UDP 60 Hz が SlimeVR Server に過負荷 | `--slimevr-rate-hz` で可変、既定 60。SlimeVR は 100-400 Hz の IMU を受け付けているので余裕がある |
| R4 | Tracker role 文字列の SlimeVR ↔ VMC 命名差 | VMC は `"waist"/"leftFoot"/"rightFoot"/"chest"/"leftKnee"/"rightKnee"/"leftElbow"/"rightElbow"`。SlimeVR の `TrackerRole.kt` enum と OSC 文字列のマップを M3 で固定 |
| R5 | Halpe26 未完了で機能不能 | M5 で起動時 fatal、CLI ヘルプに前提を明記 |
| R6 | Quat 合成の縮退 (横方向ベクトルが zero) | M3 で `|cross| < ε` チェック、縮退時は identity + valid=false で当該フレーム送信スキップ |
| R7 | Skeleton3DBus mutex 競合 | M1 の `snapshot()` は値コピー only (~350 B)、`make_bundle_json()` と排他しても 60 Hz 周期内に十分収まる |
| R8 | foot tracker yaw 不安定 (heel/big_toe 検出失敗時) | M3 で `valid=false` → スキップ。SlimeVR 側で前回値保持 |
| R9 | Halpe26 + VMC で hip-center が床にめり込む傾向 | キャリブの世界座標と SlimeVR の "Floor Y = 0" 規約の整合性を M8 の SlimeVR UI ステップで確認。必要なら `--slimevr-floor-offset-m` で z オフセット |

## 段階実行順序 (依存順)

1. **M1** — `Skeleton3DBus::snapshot()` getter (小、回帰なし)
2. **M2** — OSC writer + roundtrip テスト (M3 以降の土台)
3. **M3** — Tracker 抽出 + quat 合成 + 座標変換 + テスト
4. **M4** — VMC Publisher スレッド (M1+M2+M3 結合)
5. **M5** — CLI + ライフタイム結線
6. **M7** — slimevr_loopback CLI (M4 と並行可、M8 で必須)
7. **M6** — Stats 露出 (任意、Phase 12 候補に倒してもよい)
8. **M8** — 実機検証 (loopback → SlimeVR Server → perf 回帰)

## Phase 12+ への含み (本フェーズ範囲外)

- **Native SlimeVR UDP (packet 17)** への昇格 — VMC で solver の挙動が悪い場合のフォールバック実装。
- **HEAD tracker / HMD anchoring** — SlimeVR HMD 不在環境で head_top(17) を HEAD として送る選択肢。
- **多人数対応** — 現状 `persons.front()` のみ。複数人 SlimeVR は 1 SlimeVR Server / instance 制約があるので、複数被験者は別ポート/別 instance で送る設計が必要。
- **Frontend 表示** — `/stats` の SlimeVR 状況を `web/dual_rtmpose/app.js` のヘッダに badge 表示。
- **手首・指トラッキング** — VRChat OSC trackers / VMC は基本的に大関節のみ。Halpe26 でも手指は無いので別モデル必須。
