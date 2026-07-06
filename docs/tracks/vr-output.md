# Track: vr-output

カメラ由来の 3D pose / tracker を **VR (SlimeVR Server / SteamVR) に流す経路**。
現状最もアクティブなトラック。

## 現状 (2026-05-29)

2 つの出力経路が **同時 enable 可能**で、いずれも pose-3d トラックの `TrackerExtractor`
(単一 producer) を read-only consumer として共有する:

1. **SlimeVR Firmware UDP** (`--slimevr-out`, port 6969) — 回転のみ。位置は SlimeVR 側 IK が
   骨格 + HMD から再構築。10 trackers を named display。
2. **VMT (Virtual Motion Tracker) → SteamVR 直結** (`--vmt-out`) — 位置 + 回転。SlimeVR Server を
   飛ばして SteamVR Driver に直結。VMT_10..VMT_19 を `/VMT/Room/Driver` OSC 60Hz。

### 設計原則 / live な制約

- **10 trackers の TrackerRole 順は固定**: LeftUpperArm / RightUpperArm / Chest / Hip /
  LeftUpperLeg / RightUpperLeg / LeftLowerLeg / RightLowerLeg / LeftFoot / RightFoot。
  SlimeVR `TrackerPosition` enum に完全一致 (骨盤は `HIP(6)`、`WAIST(5)` は auto-assign されない)。
- **VMT 送信トラッカーは preset で選択** (`--vmt-preset {p3,p6,p8,full}`, 既定 `p8`)。VRChat FBT は
  最大 8 点 (hip/chest/両足/両膝/両肘) で脛 (LowerLeg) に対応 role が無いため、既定 `p8` は脛 2 本を落とす。
  `p3`=腰+両足 / `p6`=+胸+両膝 / `full`=全 10 (SlimeVR 互換)。**間引きは VMT publisher のみ**で行い
  extractor は 10 点維持 (SlimeVR 路は不変)。**VMT index は role 固定**なので preset を変えても
  SteamVR「Manage Trackers」の role 割当 (VMT_10=Left Elbow … 18=Left Foot / 19=Right Foot) は安定。
  ランタイム切替は `POST /api/vmt/preset` / Web UI `VmtPresetForm`。設計: [`../design/vr-output-vrchat-tracker-presets.md`](../design/vr-output-vrchat-tracker-presets.md)。
- **足トラッカー位置は `--foot-tracker-pos {ankle,midpoint}`** (既定 `ankle`)。`ankle` は足首位置、
  `midpoint` は足首/足先中点。回転は両モードとも不変 (`fwd = ankle→toe` で足先方向を保持)。位置を
  消費するのは VMT 送信と WebUI viz のみ (SlimeVR は回転のみ)。VRChat 実機で A/B して既定を確定する。
- **Bridge relay (Jetson → Windows .NET relay → Named Pipe → SlimeVR Server) は没**。理由は
  SteamVR 起動中の `\\.\pipe\SlimeVRInput` 排他 + 座標系整合の不安定。位置を VR に流す要求は
  VMT 経路で解決済み。実装一式は `archive/botsu-phase12-bridge-relay` ブランチに凍結。
- **座標変換**: `world_*_to_vmt` は SteamVR Y-up RH frame target。archive Bridge と完全同型。
- **VMT alignment**: HMD pose (SteamVR) を取り込み 2D Procrustes で yaw+xz を自動算出。
  単発(T ポーズ / 3 秒歩行、chest 対応)に加え、**常時バックグラウンドの `ContinuousAligner`**
  が頭部優先・chest フォールバックで継続サンプリング → reservoir → clamped EMA で半継続追従。
  サンプル品質は脊椎/首ボーンの垂直性で重み付け。Y (HMD 頭頂 vs chest 中心の個人差 0.35–0.55m)
  のみ手動 slider 運用。
- **VMT 登録タイミング**: Quest 接続前に VMT が登録されると SteamVR コントローラを奪う。Driver の
  `WaitForHmd=true` で HMD+両コントローラ検知まで `RegisterToVRSystem` を arm 保留。
- **VMT フォーク側の実装**: Driver ゲート / Manager / HMD pose sender は VMT フォーク
  (Windows, `vmt_driver.sln` / `vmt_manager.sln`) に存在。fitra-cam 側はスキーマ受信のみで無改修。

### 検証

`ctest -R 'vmt|firmware_protocol|tracker_extract|hmd_pose|auto_alignment|continuous_aligner'` +
Windows 実機 (SlimeVR Server GUI / SteamVR + VMT Manager + VRChat FBT)。
詳細な合格基準は [`cpp-migration-plan.md` 検証戦略表](../cpp-migration-plan.md) の旧 Phase 11/14/15/15.5 行。

## Changelog (新しい順)

### 2026-06-27 — discovery を初期設定 UI に露出 (自動検出/手動の選択)
zeroconf discovery が出力先 VMT を実行時に自動解決できるのに、Web-UI は手動 IP 入力欄しか
持たず自動検出を露出していなかった（`/api/config` 往復も `vmt_out/host/port/hmd_listen` の
4 つだけで `vmt.discovery` が抜けていた。YAML 永続層は対応済み）。**M1**: `merge_config` /
`draft_to_json` (`crow_routes_setup_mode.cpp`) に `vmt.discovery` を追加し、`ConfigVmt` 型 +
`SetupPage` VMT カードを「自動検出（ランタイム）/ 手動でIP指定」ラジオに再構成。自動 =
`discovery=true`+`host=""`（IP を設定時に焼き込まず実行時再解決を維持）、手動 =
`discovery=false`+`host`。ユーザー要望「設定時固定でなくランタイム自動かを選択」に対応。**M2**:
`statsText.discoveryStatus()` を起こしビューアのヘッダに「出力先 <name> <ip>:<port> / 検索中… /
手動」チップを表示（既存 stats `<pre>` の discovery 行は残置）。setup 中のライブ検出は setup
モードにビーコンが無いため見送り（没案として記録）。検証: `--setup` で `/api/config` の
discovery 往復を curl 確認、`pnpm build` / `cmake --build` パス。
**レビュー反映 (PR #46, 2026-06-27)**: 二重フラグ(`discovery`+`host`)の不整合を是正。`discovery`
を host 空否から導出する single source of truth に統一（`merge_config` で
`vmt_discovery = vmt_host.empty()`、UI は `normalizeCalibPaths` で読み込み時正規化）。dead 状態
`{discovery:false,host:""}`(= `validate_options` の fail) と manual override
`{discovery:true,host:"x"}` の取り違え/入力欄消失 footgun を解消。`discoveryStatus` は全分岐を
`bundle.vmt` でゲートし VMT 出力 OFF + HMD-listen discovery 時の `出力先` 誤表示を修正。
→ [design/vr-output-discovery-ui.md](../design/vr-output-discovery-ui.md)

### 2026-06-26 — HMD 顔オフセットを除いた頭軸アライメント
HMD-driven アライメント（連続 + ワンショット T-pose/motion）が **HMD の生 xz** を体ランドマーク
（head_top / 胸中点 / chest）に直接対応付けていた問題を修正。HMD は顔に乗っていて頭/首の鉛直軸から
注視方向に ~10cm 前方へずれており、このレバーアームが頭の向きとともに回るため解が頭向き分布に
引っ張られ、アバター頭が実頭の前にずれていた。新ヘルパー `hmd_head_axis_xz`（`auto_alignment.hpp`）が
HMD 自身の向きから後方ベクトル `R(q)·(0,0,1)` を取り、`head_axis.xz = hmd.xz + d·back.xz` で頭軸へ
射影し直す。**HMD 向きだけで決まり解の回転に非依存**（数学的にきれい）、かつ水平成分を**正規化しない**
ので見上げ/見下ろしで補正が自然に縮む（ピッチ自動対応）。連続(`make_sample`)・ワンショット
(`solve_tpose` / crow motion 収集)の3経路に同一補正を適用。二段デフォルト（純ヘルパー/関数は
無補正=0 で既存テスト保護、製品デフォルト `vmt.align_hmd_forward_m=0.10`）。`--vmt-align-hmd-forward`
(`[0,0.5]`、0=OFF) で VR 内チューニング可。Y(高さ)は従来どおり手動スライダ（xz のみ補正）。
→ [design/vr-output-hmd-head-offset-alignment.md](../design/vr-output-hmd-head-offset-alignment.md)

### 2026-06-26 — 胸 / 腰トラッカーの高さを脊椎沿いに調整可能化
Chest = `midpoint(neck, hip_center)`・Waist(=Hip) = `hip_center` 固定だった胸/腰トラッカーの
**位置**を、脊椎方向 (neck − hip_center) に沿って引き上げ可能にした
(`pos = hip_center + frac·spine`, `frac ∈ [0,1]`)。実機で両者がやや低く感じられたため。
製品デフォルトを引き上げ済み (Chest `0.65` / Waist `0.15`) にしつつ
`--chest-height-frac` / `--waist-height-frac` (YAML `three_d.vr_{chest,waist}_height_frac`、
validate `[0,1]`) で微調整可能。**位置のみ**で回転 (forward/up) は不変 — 影響は VMT 送信 +
WebUI viz のみ、回転だけの SlimeVR Firmware UDP 路は同一。ワールド上方向ではなく脊椎沿いに
動かすため前傾時もトラッカーが胴体に乗る。`extract_trackers` の**関数デフォルトは歴史的配置
(0.5/0.0)** で既存 golden test を保護、製品デフォルト (`TrackerExtractorOptions`) のみ引き上げ
(foot_pos_mode と同じ二段デフォルト)。設計はプリセットの design doc に追記。
→ [design/vr-output-vrchat-tracker-presets.md](../design/vr-output-vrchat-tracker-presets.md)

### 2026-06-25 — zeroconf ディスカバリのレビュー指摘修正 (バグ修正)
M1/M2 実装に対する Codex / code-review の指摘を反映。**ライフサイクル**: `PoseRelay` で
`beacon` を `receiver` より前に宣言し、デフォルト破棄順 (逆順) で receiver を先に停止 —
受信スレッドは `beacon->endpoint_bus()` を読むため、`stop()` を経ない破棄 (例外巻き戻し)
でも bus 所有者が受信スレッドより長生きするようにした。**ネットワーク入力検証**:
`announce_admissible` で `osc_recv_port` の `[1,65535]` 外を reject (壊れた/古いピアの
0/負/65535超が `uint16_t` 切り詰めで誤送信先になるのを防止)。**設定検証**: `hmd_listen_port`
の範囲検証を `hmd_listen_enabled` ブロック外へ出して無条件化 — ビーコンは `vmt_out` 経路
(listener 無効時) でも本ポートを `self_osc_recv_port` として広告/キャストするため。
**重複解決の集約**: VMT publisher (送信先) と TrackedPoseReceiver (punch) で重複していた
「bus からの宛先解決」を新ヘッダ `vmt/discovery_endpoint.hpp` の `DiscoveryEndpointLatch`
へ集約。`inet_pton` 成功時のみ latch を進めるよう統一 (publisher は失敗時も applied を
更新していた乖離を解消)。`VmtPublisher::set_destination` を `bool` 返りに変更。**hot path**:
`DiscoveryEndpointBus` に世代カウンタを追加し `have/ip/port` 変化時のみ bump (毎 tick の
age 更新では bump しない) — consumer は世代未変化なら lock-free に即 return し、毎 tick の
`ResolvedPeer` (string×4) コピー + ロックを回避。`test_discovery` (世代+latch) /
`test_main_config` (hmd_listen_port 検証) にケース追加。pose wire は不変。
→ [design/vr-output-zeroconf-discovery.md](../design/vr-output-zeroconf-discovery.md)

### 2026-06-25 — zeroconf ディスカバリ M3/M4 完了 (Windows 実装 + 両機 IP 無指定の実機確認)
issue #36 の残り M3/M4 を完了し、zeroconf ディスカバリをクローズ。**M3 (Windows `vmt_manager`)**:
VMT フォーク (`refs/VirtualMotionTracker`) に `DiscoveryAnnounce.cs` (OSC 1.0 codec、golden と byte 一致) +
`ZeroconfDiscovery.cs` (39580 を `ReuseAddress`+`Broadcast`+`MulticastTTL=1` で bind / group join、1 Hz で
multicast+broadcast 二段送信、`role="vmt"`/`osc_recv_port=39570` announce + `role="jetson"` browse、
admission/最小 id 選択/pin/token/`peer_timeout` 5 s) を実装。採用 jetson の `src_ip:osc_recv_port` を
pose-relay 送信先へ自動設定 (`UpdateJetsonSender(...,"discovery")`、driver-learned fallback より優先)。
Manager UI に検出ピア一覧 + pin/token。instance_id は GUID 由来 16hex を Settings に永続。**M4 (実機)**:
Jetson / Windows を**両方 IP 無指定**で起動 → 自動接続を確認。`/stats3d` で Jetson 側
`discovery.mode=discovery` / `resolved.have=true` (→ `VMT-SH_MAIN @ 172.34.1.9:39570`) / `vmt.host=""` /
`sent_bundles` 増加 / HMD pose `valid=true` `age_ms≈3.8` (基準 <100 をクリア)、VR 内トラッカー表示まで
正常。診断メモ: 逆経路 (`/fitra/tracked_pose`) は Windows で SteamVR 未起動だと `HmdPoseTick` の
`util==null` ガードで一切送られない (起動で復帰) — discovery 不具合ではない。golden 一致の独立 Python
プローブ (`peer`/`sniff`/`self-test`) でワイヤ照合に使用。fitra-cam 側は M1/M2 のまま無改修。
→ [design/vr-output-zeroconf-discovery.md](../design/vr-output-zeroconf-discovery.md) /
[design/vr-output-zeroconf-discovery-vmt-spec.md](../design/vr-output-zeroconf-discovery-vmt-spec.md)

### 2026-06-24 — VRChat 向けトラッカープリセット + 足位置モード
VMT 送信を VRChat 標準 8 点に既定で一致させ、本数を `--vmt-preset {p3,p6,p8,full}` (既定 `p8`) で
選択可能にした。従来は 10 点固定送信だったが、VRChat FBT は最大 8 点で脛 (LowerLeg) に対応 role が
無く浮いていた。既定 `p8` は脛 2 本を落とし、残り 8 本 (両肘=上腕 / 胸 / 腰 / 両膝=腿 / 両足) が
VRChat 仕様と body part・装着位置まで一致する。間引きは **VMT publisher の role マスク**のみで行い、
extractor は 10 点維持 (SlimeVR Firmware UDP 路は不変)。VMT index は role 固定なので SteamVR の role
割当が preset 間で安定。ランタイム切替は `GET/POST /api/vmt/preset` + Web UI `VmtPresetForm`。
あわせて足トラッカー位置を `--foot-tracker-pos {ankle,midpoint}` (既定 `ankle`) で切替可能にした
(回転は不変、位置のみ・VMT/viz だけに影響)。設計:
[`../design/vr-output-vrchat-tracker-presets.md`](../design/vr-output-vrchat-tracker-presets.md)。

### 2026-06-23 — VMT ⇔ Jetson zeroconf ディスカバリ M1/M2 実装 (Jetson 側)
6/18 に起票した zeroconf ディスカバリ (issue #36) の Jetson 側を実装。**M1 (純ロジック)**:
`discovery_announce` (`/fitra/announce`・typetag `,sssiiss` の encode/parse + ホスト名 FNV-1a の
`stable_instance_id`) と `peer_registry` (`announce_admissible` / `select_peer` / `PeerRegistry` +
`HmdPoseBus` 同形の `DiscoveryEndpointBus`) を socket/thread なしの純関数で新設。時刻は引数で受け
決定的。`test_discovery.cpp` の 10 ケース (golden バイト列 / round-trip / 不正 reject / id 決定性 /
単一・最小 id・pin・token・stale・proto) で固定。**M2 (配線)**: `DiscoveryBeacon` (39580 を INADDR_ANY
で bind + group join + SO_BROADCAST、1 Hz で multicast 239.255.42.99 と 255.255.255.255 へ二段送信、
TTL=1) を新設し `PoseRelay` に所有 (run / calib-extrinsic 両方で共有)。`VmtPublisher` を
`connect()`+`send()` から `sendto()`+mutex 保護の差し替え可能宛先へ変更し、空 host で落ちず discovery
bus から宛先を解決 (last-known latch、未解決は `skipped_no_endpoint` でスキップ)。`TrackedPoseReceiver`
の punch も recv スレッドでランタイム解決。`vmt.host` 既定を空に変更 (空+discovery on で自動 /
非空で従来どおり手動・discovery 完全バイパス)。`vmt.{discovery,pair_id,pairing_token,discovery_group,
discovery_port,instance_name,peer_timeout_s}` を config/CLI/emit/validate に追加。`/stats3d`+`/ws3d`+
WebUI (`bundle.ts`/`statsText.ts`) に検出ピア (resolved / peers / age) を表示。**pose wire
(`/VMT/Room/Driver` / `/fitra/tracked_pose` / `/fitra/punch`) は不変** — 既存 `test_vmt_osc_writer` /
`test_tracked_pose_receiver` が回帰ガード。loopback 2 ビーコンの相互発見 smoke で multicast/broadcast/
src-IP 学習/role 選択/self 除外/冪等 dedup を確認。**M3 (Windows `vmt_manager`)** は受け渡し仕様
[design/vr-output-zeroconf-discovery-vmt-spec.md](../design/vr-output-zeroconf-discovery-vmt-spec.md)
を別途実装。**M4 (両機 IP 無指定の実機確認)** は未実施。
→ [design/vr-output-zeroconf-discovery.md](../design/vr-output-zeroconf-discovery.md)

### 2026-06-19 — 3D カメラ/HMD マーカーの PR#40 レビュー指摘修正 (バグ修正)
PR #40 の Codex / Gemini レビュー指摘を反映。(1) **HMD 向きバグ**: HMD マーカーの姿勢を
`B·R` から `B·R·B⁻¹` (トラッカーと同じ共役) へ修正。HMD `quat_wxyz` は `vmt_pose_to_world()`
で fitra world フレームへ両側 rebase された姿勢のため、生 extrinsic 由来の camera (`B·R` が正)
とは扱いが異なる。修正前は視線が常に 90°(真下) ずれて HMD 向き確認が機能しなかった。
(2) `make_hmd_status_fragment` の world 変換を `snap.pose.valid` でガード (非有限値の JSON 混入
→ frontend `JSON.parse` クラッシュ防止)。(3) `Triangulator::camera_poses()` をコンストラクタで
事前計算しキャッシュ (毎フレームの行列演算/`cv::Mat` アロケーション排除)。(4) `updateCameras`/
`updateHmd` の毎フレーム quaternion アロケーションを scratch + `multiplyQuaternions` で除去。
軽微なため design doc なし (changelog のみ)。

### 2026-06-18 — 3D プレビューに HMD 位置を表示 (VMT 接続時)
VMT 接続時、3D ビューアに HMD をワイヤーヘッドセット箱＋前方視線で描画。HMD pose は SteamVR から
VMT Driver フレーム (Y-up・alignment 適用後) で届くため、新設の `vmt::vmt_pose_to_world()` で
`apply_vmt_alignment ∘ world_*_to_vmt` の逆 (並進戻し→-yaw→基底逆) を取り、fitra world (Z-up) の
`pos_world`/`quat_wxyz` を `hmd` fragment に追加 (publisher の現 alignment を使用)。frontend は
トラッカーと同じ basis 変換でマーカーを配置し、`ThreeDView` に `show hmd` トグルを追加。スケルトンと
同一空間に重なるため alignment 品質の QA に使える (alignment が効いていないと見当違いの位置に出る)。
往復変換は `test_vmt_protocol` の round-trip テストで固定。コントローラーは今回スコープ外
(controller_bus は受信済みだが live で crow 未配線)。軽微なため design doc なし (changelog のみ)。

### 2026-06-18 — 3D プレビューに校正カメラ位置を表示
3D ビューア (`/ws3d`) に、校正済み各カメラの設置位置と視野方向をワイヤー視錐台 (向き付き四角錐)
で描画。`Triangulator::camera_poses()` を新設し world 上のカメラ中心 (`-Rᵀ·t`) と camera→world
回転を quaternion (w,x,y,z) で公開、`Skeleton3DSnapshot.cameras` 経由で `/ws3d` バンドルへ毎フレーム
同梱 (extrinsics は静的・2〜3 台で数百バイトのため専用 REST は設けず既存ストリームに相乗り)。frontend
は `SkeletonViewer.updateCameras()` で id ごとに視錐台を遅延生成し、トラッカーと同じ basis 変換
(pos `[x,z,-y]` / quat 共役) で配置。`ThreeDView` に `show cameras` トグルを追加。校正の妥当性
(カメラがどこを向くか) を直感確認するためのデバッグ可視化。軽微なため design doc なし (changelog のみ)。

### 2026-06-18 — VMT ⇔ Jetson zeroconf ディスカバリ仕様起票 (仕様のみ)
他ユーザー配布時に唯一手で埋める設定 (`vmt.host` = Windows PC の実 IP) を消すため、Jetson と
VMT Manager が同一 LAN 上で相互に相手を見つける制御プレーンを設計。現状は「Jetson が先に PC IP を
知る」鶏卵問題なので、双方が `/fitra/announce` (OSC) を `239.255.42.99:39580` マルチキャスト
(+broadcast 二段構え, TTL=1) に 1 Hz で投げ合い、src IP + 広告 `osc_recv_port` で相互学習する方式を
採用 (mDNS は外部依存増で没、cloud/QR も没)。pose wire (`/VMT/Room/Driver` / `/fitra/tracked_pose` /
`/fitra/punch`) は不変、純粋な追加。複数ピアは単一→自動 / 複数→`instance_id` 最小 + `pair_id` pin +
`pairing_token` でリグ間遮断。`vmt.host` 明示指定は最優先で discovery を上書き (後方互換・退路)。
**実装は未着手** — M1/M2 が fitra-cam (Jetson) 側、M3 が Windows `vmt_manager` 側、M4 が実機で両機
IP 無指定起動の確認。Windows フォーク開発と歩調を合わせるため wire/挙動契約を先行確定。
→ [design/vr-output-zeroconf-discovery.md](../design/vr-output-zeroconf-discovery.md)

### 2026-06-15 — React WebUI の Gemini レビュー堅牢化 (バグ修正)
PR #33 の Gemini Code Assist 指摘対応。(1) `SkeletonViewer.dispose()` が geometry/material
を解放しておらず、route 往来 (`/` ↔ `/subject-calib`) の度に GPU リソースが蓄積していたのを、
`scene.traverse` で全 geometry/material を一度ずつ dispose するよう修正 (Jetson の共有メモリでは
OOM 要因)。(2) `draw2d` で黒背景塗り後に `canvas.width/height` を代入していたため寸法変更フレームで
背景がクリアされていたのを、リサイズを塗り前へ移動。(3) `useWebSocketJson` のクリーンアップで
`onclose` のみ null 化していたのを全ハンドラ (onopen/onerror/onmessage) を null 化し、unmount 後の
遅延発火を防止。(4) `SubjectCalibPage` / `VmtAutoForm` / `ViewerPage` の各 API 呼び出しに try-catch
を追加し、`fetch` reject 時の未ハンドル例外と `switchPending` のスタックを解消。preflight に hold/frames
の NaN バリデーションを追加。なお commitBase の race 指摘は React 18 の discrete event flush 順
(state flush → macrotask) で実害なく、既存実装を維持。

### 2026-06-15 — Vite dev proxy / VMT 数値欄のレビュー修正 (バグ修正)
Codex レビュー対応。(1) HMR dev で `/extrinsic-calib` を開いた際、legacy SPA が叩く
`/api/excal/*` が Vite proxy 未登録で Crow に届かず start/stop/solve が失敗していたのを、
`web-ui/vite.config.ts` に `/api/excal` を追加して解消。(2) `VmtAlignForm` の base 数値欄が
React `onChange`(毎キーストローク発火) で `Number()` 即時送信していたため、入力途中の空文字 / `-`
が `0` / `NaN`(→JSON null) として live alignment に流れていた。編集中テキストを `baseDraft` で
保持し、blur/Enter の commit 時のみ有限値を検証して送信する旧 UI 挙動 (DOM `change` 相当) へ戻した。

### 2026-06-14 — React WebUI を flow daemon に追従
`Develop` の pose-3d flow daemon 取り込みに合わせ、旧 `web/dual_rtmpose` / `web/subject_calibration`
へ入っていた mode 追従 UI を Vite/React SPA へ移植。`GET /api/state` / `POST /api/flow/switch`
を `web-ui` の typed API + `useFlowWatch` で扱い、viewer は自動 redirect せず banner と
再キャリブ切替ボタンを表示、subject wizard は approve 後の `next_step` と run への自動遷移を追従する。
legacy の `/extrinsic-calib` は未移植のため、`web-ui/public/flow.js` を root static asset として残し
`/flow.js` 互換を維持。Crow の既定 static path は `app/paths` 側で `web-ui/dist` に統一。
→ [design/vr-output-webui-vite-react.md](../design/vr-output-webui-vite-react.md)

### 2026-06-08 — VMT pose relay wire spec (HMD + 左右 controller の統合受信)
controller-marker extrinsic calibration で controller pose が必要になり、旧 PoC の
`/fitra/hmd_pose` + `/fitra/controller_pose` 別ポート構成は運用に乗らないと判断。VMT Manager 側から
1 UDP port (`39571`) に `/fitra/tracked_pose` を role 付きで流す canonical wire spec を起票。
HMD / left controller / right controller を同じ SteamVR Standing universe の absolute pose として送り、
fitra-cam 側は `TrackedPoseReceiver` で role ごとの latest-wins bus に分配する。旧 message は
移行期間だけ互換受信。`extrinsic_calib.controller_role` (`left|right`, 既定 `right`) を追加し、
旧 controller 専用 port は deprecated。
→ [design/vr-output-vmt-pose-relay-wire-spec.md](../design/vr-output-vmt-pose-relay-wire-spec.md)

### 2026-06-08 — One Euro の GitHub レビュー修正 (バグ修正)
PR #25 の gemini / Copilot レビュー指摘を反映。design doc なし(changelog のみ)。
- **(gemini HIGH / 実バグ)** 位置 One Euro の速度推定 `pos_dx_hat` 更新に外れ値ゲートが
  効いておらず、三角測量グリッチの巨大 `dx` で速度状態が汚染 → 直後の数フレームで
  カットオフが開き静止ジッタが素通りしていた。速度更新にも `(1-gate)` を適用し、回帰テスト
  (`test_one_euro_outlier_gate_does_not_pollute_speed`)を追加。
- `TrackerExtractorOptions` の One Euro 既定係数が `MainConfig` のチューニング値と不一致
  だった点を同値化(位置 1.0/4.0、回転 1.5/1.5)+「main 側で上書きされる」旨をコメント明記。
- design doc の「しきい値の根拠」に現行既定値(初期値ではない)の注記を追加、`one_euro_alpha`
  のエッジケースコメントの優先順位明確化、`test_main_config` のコメント実態合わせ。
- 完了の定義に従い `docs/cpp-migration-plan.md` 検証戦略表に One Euro 行を追加。

### 2026-06-08 — One Euro 既定値を実測チューニング値に更新 (閾値調整)
`configs/medium_3d.yaml` で詰めた One Euro 係数を `MainConfig` の既定値へ昇格。位置は
`mincutoff 0.8→1.0` / `beta 0.4→4.0`、回転は `mincutoff 1.0→1.5` / `beta 0.3→1.5`
(`dcutoff` は両軸 1.0 据え置き)。初期既定の `beta` は m/s・rad/s スケールに対し小さすぎ、
動作時もカットオフが開ききらず遅延が残っていたため引き上げ。`main_config.hpp` の既定値と
`main.cpp --help` の表記を同値に更新。design doc なし(閾値調整のため changelog のみ)。

### 2026-06-03 — One Euro フィルタによる動静適応スムージング
座位静止時のトラッカー揺れに対処。固定 α EMA(α=0.5 ≈ カットオフ 9.5Hz)は静止の滑らかさと
動作追従を両立できないため、位置(per-axis)・回転(測地角速度ベース)とも **One Euro
(速度適応カットオフ)** に置換。静止時は低カットオフで強くスムージング、動作時は `beta·速度`
でカットオフを開いて遅延なく追従。既存の swing/twist 分離・parent-yaw transport・hip-relative
hold・外れ値ゲート(8–16 m/s freeze)は温存(swing/twist 本体を per-tracker alpha の `impl` に
抽出、固定 α 版は bit-identical で既存 ctest 無傷)。既定 ON、`--vr-no-one-euro` で旧 EMA に
フォールバック、`beta=0` で固定カットオフ EMA に縮退。`three_d.vr_*` YAML / `--vr-{pos,quat}-*`
CLI を追加。新規 ctest(位置 6 / 回転 3 / config 1)。Phase 14 で見送った One Euro の昇格。
→ [design/vr-output-one-euro-filter.md](../design/vr-output-one-euro-filter.md)

### 2026-06-03 — 継続キャリブ「自動追従」トグルを React UI へ移植 (バグ修正)
Develop マージで判明した移行漏れの解消。WebUI 移行 (2026-06-01) 時点では新 Vite/React UI の
「自動追従」チェックボックスが `disabled` プレースホルダ (`未接続`) のままで、レガシー JS
(`web/dual_rtmpose/app.js`) にあった継続キャリブ操作が未配線だった。design doc なし(既存の
[design/vr-output-continuous-hmd-calibration.md](../design/vr-output-continuous-hmd-calibration.md) /
[design/vr-output-webui-vite-react.md](../design/vr-output-webui-vite-react.md) の範囲、changelog のみ)。
- `api.ts`: `postContinuousAlign(enabled)` を追加(`/api/vmt/alignment/auto/continuous/{start,stop}`)。
- `VmtAutoForm.tsx`: チェックボックスを `continuous_align` ブロックで制御。null=disabled (`未接続`)、
  present=有効でラベルに `ON (cells n/m)` / `OFF` を表示、`onChange` で start/stop POST、失敗は `<output>` に表示。
- `statsText.ts`: 3D stats に `cont_align` / `cont_cells` / `cont_resid_m` / `cont_updates` 行を復元(レガシー版と同レイアウト)。
- `ViewerPage.tsx`: ws3d バンドルの `continuous_align` を ~6Hz スロットル state 経由で `VmtAutoForm` へ供給。
- バックエンド側 (`publisher_loop` の ws3d fragment) は 2026-06-03 レビュー修正で対応済み、フロント無改修で接続。

### 2026-06-03 — 継続キャリブのレビュー修正 (バグ修正)
Codex + GitHub (gemini / Copilot) レビューで顕在化した点を修正。design doc なし(changelog のみ)。
- `SampleReservoir::key_of`: 負座標で符号付き左シフト UB(VMT x/z は通常移動で負になる)→ uint32 経由 pack。負4象限が別セルになる回帰テスト追加。
- `continuous_align`(と既存の `hmd`)ステータスが `/stats3d` にしか載らず、WebUI は `/ws3d` バンドル(`state.bundle3d`)しか読まないため「自動追従」トグルが恒久 disabled だった → `publisher_loop` の ws3d ブロードキャストにも fragment を載せた。
- `make_sample`: 非有限入力(NaN/Inf)を reject。reservoir 汚染と `key_of` の float→int キャスト UB を防止。
- `ramp`: `zero_at == full_at` の退化帯を step 関数化(「full_at で 1」契約を満たす)。
- 自動追従 OFF 時に reservoir を `clear()`(OFF→ON で古セルを使った solve を防止)。HMD 速度計算の dt に下限(`>1e-4`)。
- gemini の「`joints[19]` で範囲外アクセス」指摘は誤検知(`joints` は固定長 `std::array<,26>`、coco17 でも index 19 は valid=false の zero-init)。

### 2026-06-01 — WebUI を Vite/React (TypeScript) へ移行
旧バニラ JS フロント (`web/dual_rtmpose` 1450 行 + `web/subject_calibration`) を Vite/React/TS の単一 SPA
(`web-ui/`) に移植。`BrowserRouter` で `/`→viewer・`/subject-calib`→wizard を出し分け、Crow は両ルートで
同一 `dist/index.html` を返す (SPA fallback)。接続先は `lib/config.ts` の `httpUrl()`/`wsUrl()` に集約し、
同一オリジン (Crow 配信/dev proxy) と絶対 URL (別ホスト/将来の Tauri/Wails デスクトップ) を 1 コードで賄う。
HMR 開発は Vite dev server の proxy (`/ws`・`/ws3d` は `ws:true`) 経由で実データ表示。30Hz バンドルは ref +
`requestAnimationFrame` で命令的描画、stats のみ ~6Hz スロットルで React state 更新。Three.js は依存パッケージ化
(vendored 破棄)。WS/REST スキーマは不変。Crow は `guess_static_dir`/`guess_subject_calib_static_dir` を
`web-ui/dist` に向けるのみ (ルート無改修)。Python キャリブ系 (`web/calibration`, :8010/:8020) は将来 C++ 化
予定のため対象外。最終目標の VMT Manager 統合 (Tauri/Wails 単一 Windows アプリ) を見据えた設計。
→ [design/vr-output-webui-vite-react.md](../design/vr-output-webui-vite-react.md)

### 2026-05-30 — 継続キャリブの cold-start ブースト
初期収束が遅すぎる(実機で 1 分以上歩かないと位置が合わない)問題に対処。原因は fine の
step clamp(`max_pos_step 0.05m` / `max_yaw_step 2°` per 2s resolve)が起動時の大きな初期
ズレまで律速していたこと。純関数 `update_lock_state` でロック状態を導入し、未収束の間は
coarse クランプ(`coarse_max_pos_step 0.50m` / `coarse_max_yaw_step 30°` / `blend 0.6`)で
速く粗収束 → 近接した solve が連続(`lock_streak 3`)したら fine クランプに latch(ジャンプ
防止は維持)。VMT 再センタリング等の大乖離・runtime トグルで coarse へ復帰。`/stats3d` と
Web UI に `locked` を追加。新規 ctest `test_lock_state`。
→ [design/vr-output-continuous-hmd-calibration.md](../design/vr-output-continuous-hmd-calibration.md)(cold-start 追補)

### 2026-05-29 — 自動・半継続 HMD キャリブレーション
Phase 15 の単発 alignment を常時バックグラウンド化。起動時から HMD と「信頼性高く
報告された頭部(不安定時は chest 中点にフォールバック)」を継続サンプリングし、空間
reservoir に代表値を蓄積 → 定期 `solve_motion` → clamped EMA で alignment を自動収束・
追従(Y は手動 slider 維持)。サンプル品質の主要因に**脊椎/首ボーンの垂直性**(直立ほど
高得点)を採用。新規 `ContinuousAligner`(`fitra_vmt`)、`--vmt-continuous-align`(既定 ON)、
`/api/vmt/alignment/auto/continuous/*` + `/stats3d` ブロック。
→ [design/vr-output-continuous-hmd-calibration.md](../design/vr-output-continuous-hmd-calibration.md)

### 2026-05-29 — OSC パディングの単一 insert 化 + gate 定数の static_assert (挙動不変)
(1) `OscWriter::emit_osc_string` の 4-byte 境界パディングを `push_back` ループから単一
`insert(end, 1 + pad4(...), '\0')` に置換。出力バイト列は同一 (`test_vmt_osc_writer` golden 通過)。
(2) `tracker_extract.cpp` の smoothstep gate 定数 (`kRollSin*` / `kPosVelGate*` / `kPelvisYawGate*`)
に `low < high` を固定する `static_assert` を追加 — 将来の境界反転がコンパイル時に弾かれる。
値は不変。微最適化のため design doc なし (changelog のみ)。

### 2026-05-29 — 出力レイテンシ M1: frame-rate 非依存 smoothing (キーストーン)
GPU フロントエンドでパイプラインが詰まった後、E2E の支配項は VR 出力の 60Hz×2 ホップ
(avg +16.7ms / worst ~33ms)。e2e-latency M4 で hop1 をイベント駆動 (opt-in) にしたが、
smoothing が **dt 非依存の固定 alpha** のままで、ソースレート同期だと高 fps で過平滑になる潜在バグが
あった。`apply_quat_smoothing`/`apply_pos_smoothing` を `alpha_eff = 1-(1-base_alpha)^(dt/nominal)` の
frame-rate 非依存形に一般化 (`run_loop` の実測 dt / nominal dt を配線)。固定レート (`dt==nominal`) は
従来と完全一致 (既定ゼロリスク)、イベント駆動は過平滑解消。これがレート引き上げ・イベント駆動を
安全にするキーストーン。`test_tracker_extract_pos` に rate-independence テスト追加 (dt/2 の 2 ステップ ==
dt の 1 ステップ 他)、ctest 9/9。実機 judder / e2e 数値検証 + publisher hop2 は
被写体 (`ik_locked`)+SteamVR 要のため M2 送り。イベント駆動既定化は後にレイテンシ目的ではなく
stale snapshot 再フィルタ防止の freshness 修正として 2026-07-07 に実施済み。
→ [design/vr-output-latency.md](../design/vr-output-latency.md)

### 2026-05-29 — 出力レイテンシ M2: 被写体実測 — VR ペーシングは lever でない (負の結果)
被写体 in view + calib + subject02 で `e2e_capture_to_send_ms` を A/B 実測。**extractor を event-driven に
しても publisher を 60→120Hz にしても e2e は不動 (~34-35ms)** — 理論「60Hz×2 = +16-33ms」は実機では
非該当 (extractor は三角測量にほぼ同期、hop2 も支配項でない)。一方 **nvjpeg 全 GPU フロントエンドで
cap→pub 21→13ms、e2e 34→26ms (−8ms)**。photon→send を削るのはパイプラインのみと確定。残 VR 側 ~13ms は
`sync_window=15ms` + 処理で rate 非依存。よって VR ペーシングのレイテンシ目的変更は見送り (M1 smoothing は
過平滑バグ correctness 修正として維持)。VR レイテンシを下げる手は 3D 設定の `cameras.pixel_format: nvjpeg`
(per-machine config は gitignored、雛形 `configs/live_2cam_3d.yaml.example` に既定記載 / CLI `--pixel-format nvjpeg`)。
judder の体感比較は HMD 主観評価として残課題。
→ [design/vr-output-latency.md](../design/vr-output-latency.md)

### 2026-05-27 — VMT 登録ゲート + sender の Manager 統合
Driver `WaitForHmd` ハードゲートで Quest 接続前の登録レースを解消 (コントローラ奪取回避)。
`vmt_hmd_pose_sender` を廃止し `vmt_manager` に吸収 (HMD pose 中継 + 登録 arm + auto-launch)。
Jetson IP は Driver が OSC `remoteEndpoint` から自動学習。fitra-cam は無改修。
→ [archive/phase15.5-vmt-registration-gate.md](../archive/phase15.5-vmt-registration-gate.md)

### 2026-05-26 — HMD pose 駆動の自動 VMT alignment
SteamVR HMD pose を `/fitra/hmd_pose` UDP で受信 (`HmdPoseReceiver` → `HmdPoseBus`)、
chest tracker との対応から `AutoAlignmentSolver` (cv::SVD 2D Procrustes) で yaw+xyz を自動算出。
T ポーズ瞬時キャリブ + 3 秒歩行精度モードの 2 操作。Web UI `/api/vmt/alignment/auto/*`。
→ [archive/phase15-vmt-hmd-auto-align.md](../archive/phase15-vmt-hmd-auto-align.md)

### 2026-05-25 — VMT 経由 SteamVR 直結
位置 + 回転を VMT 経由で SteamVR Driver に直結 (SlimeVR Server を飛ばす)。Bridge relay 没の
代替経路。`VmtPublisher` を `TrackerExtractor` の read-only consumer として並列接続、
Firmware UDP と同時 enable 可。OSC 1.0 wire writer を旧実装から `fitra::vmt` に復元。
→ [archive/phase14-vmt-steamvr.md](../archive/phase14-vmt-steamvr.md)

### 2026-05-22 — Bridge relay 経路を没
位置を VR に流す Bridge relay (Named Pipe) は SteamVR との排他 + 座標系問題で不採用。
`archive/botsu-phase12-bridge-relay` に凍結。位置経路は後の VMT で復活。
(roll 品質改善 M1 は pose-3d トラックへ。)
→ [archive/phase12-slimevr-bridge-relay.md](../archive/phase12-slimevr-bridge-relay.md)

### 2026-05-21 — SlimeVR ネイティブ Firmware UDP 連携
初版 VMC over OSC が SlimeVR で連番表示になり body-part assign 不能 → Firmware UDP (port 6969)
へ移行。10 trackers を named display。Handshake → SensorInfo×10 → 60Hz RotationData + Heartbeat。
MAC は hostname SHA-1 で安定化 (再起動後も persistence)。
→ [archive/phase11-slimevr-integration.md](../archive/phase11-slimevr-integration.md)
