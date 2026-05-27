# Phase 15.5 — VMT 登録ゲートによるコントローラ奪取の自動回避 + sender の manager 統合

> Phase 15 で HMD pose 駆動の自動 alignment が入ったが、Windows 側に独立 overlay app `vmt_hmd_pose_sender` を常駐させる運用コストが残った。並行して **VMT が SteamVR のコントローラを奪う** 問題 (Quest 接続より先に VMT デバイスが登録され、入力フォーカス/ダッシュボード制御が VMT に張り付く) が判明。
>
> Phase 15.5 では両者を **VMT フォーク側の改修** で同時に解決する。`vmt_hmd_pose_sender` を廃止して **VMT Manager (`vmt_manager`) に吸収**し、Manager が HMD/コントローラの接続を検知して **VMT Driver の登録を「Quest が揃うまで」遅延** させる。Jetson 側 fitra-cam は **無改修** (Phase 15 の `HmdPoseReceiver` がそのまま `/fitra/hmd_pose` を受ける)。
>
> **実装は VMT フォーク側に存在し、fitra-cam リポジトリには本ドキュメント (= 設計の source of truth) のみを残す。**

## Context

### 問題: VMT がコントローラを奪う

- 症状: VRChat 等のアプリで手がおかしくなる。SteamVR 上もアクティブコントローラ表記は Oculus 判定になるが、**ダッシュボードを含む全アプリで入力が VMT に吸われる**
- 切り分け結果 (実機):
  - **VMT Manager の Priority を下げても無効**
  - **互換モードを切っても無効**
  - **VMT のコードを改変・ビルドしても無効** (下記「試して効かなかったこと」)
  - **fitra-cam を「Quest が完全に接続された後」に手動起動すると奪われない** ← 決定的

### 診断: ロール/優先度ではなく「登録タイミング」

SteamVR は「最初に登録されたコントローラ的デバイスに入力フォーカス/ダッシュボード制御を張り付ける」挙動で、これは `Prop_ControllerRoleHint_Int32` / `Prop_ControllerHandSelectionPriority_Int32` より上位の判断。VMT は最初の `enable≠0` パケットを受けた瞬間に `TrackedDeviceAdded` を呼ぶ (`vmt_driver/TrackedDeviceServerDriver.cpp:477 RegisterToVRSystem`、`m_alreadyRegistered` ガードは :479)。Quest のコントローラがまだ無いタイミングで登録が走ると奪取が発生する。

→ **`TrackedDeviceAdded` を呼ぶ時刻そのものを Quest 接続後まで遅らせる以外に手はない。**

### 試して効かなかったこと (記録)

`vmt_driver/TrackedDeviceServerDriver.cpp` の以下の改変はいずれも奪取を止めなかった。プロパティ系は無力という診断を裏付ける:

- `Prop_InputProfilePath_String` をデバイスクラス別 (tracker / controller) に分離
- `Prop_ControllerHandSelectionPriority_Int32` を `isController` のときだけ設定するよう限定

### なぜ Manager 統合か (sender を VMT に寄せる根拠)

- **Driver は HMD pose を読めない**: `IVRServerDriverHost` / `IVRDriverInput` しか持たず、他 driver 管理下の HMD を列挙できない (Phase 15 が外部 sender を立てた理由)
- **Manager はクライアント (`IVRSystem`) なので読める**: `vmt_manager/openvr_api.cs` に `GetDeviceToAbsoluteTrackingPose` / `GetTrackedDeviceClass` があり、`OSC.cs` (Rx 39571 / Tx 39570) と 100ms 周期の `DispatcherTimer` (`MainWindow.xaml.cs:162`) も既存。**sender がやること (HMD/controller pose 読み取り + OSC 送信) は Manager に全部揃っている**
- `SetApplicationAutoLaunch` / `AddApplicationManifest` もリンク済 (`openvr_api.cs`) → **SteamVR 起動と同時に自動起動でき、常駐の手間ゼロ**

### Jetson IP は設定不要 (自動学習)

VMT Driver の OSC 受信は送信元エンドポイントを取得済み:

```cpp
// vmt_driver/CommunicationManager.cpp:184
void OSCReceiver::ProcessMessage(const osc::ReceivedMessage& m, const IpEndpointName& remoteEndpoint)
```

fitra-cam が `/VMT/Room/Driver` を投げてくる `remoteEndpoint` に **Jetson の IP がそのまま入る**。Driver→Manager の Tx 経路 (既定 `127.0.0.1:39571`、`DirectOSC.cpp:70`) で Manager に中継すれば、Manager は学習した IP に HMD pose を送り返せる。**Jetson IP の手動設定が全経路から消える。**

注意: `remoteEndpoint` のポートは fitra-cam の送信側エフェメラルポートなので、送り返しは「学習した **IP** + 既知ポート `--hmd-listen-port` (既定 39571)」を使う。

## 確定方針 (2026-05-27)

- **案A (Driver 側ハードゲート)** を採用。登録が物理的に起きる場所 (Driver) で止めるため最も堅牢で、fitra-cam の挙動に依存しない
- **`vmt_hmd_pose_sender` は廃止**し `vmt_manager` に統合 (HMD pose 中継 + 登録 arm + Jetson IP 中継受信)
- **fitra-cam は無改修**: Phase 15 の `HmdPoseReceiver` / `AutoAlignmentSolver` / Web UI はそのまま。`/fitra/hmd_pose` の送信元が sender から Manager に変わるだけ (スキーマ不変)
- **VMT フォーク必須**: `refs/VirtualMotionTracker` は upstream `gpsnmeajp/VirtualMotionTracker` を指す。改修前に自分の fork へ remote を張り替え、専用ブランチを切る
- **ビルド/検証は Windows のみ** (`vmt_driver.sln` MSVC C++ / `vmt_manager.sln` C#)。Jetson 側からの ctest 対象外 → 検証は Windows 手動スモーク

## ゴール / 完了条件

1. SteamVR 起動 → `vmt_manager` が auto-launch (overlay) で上がる
2. HMD 未装着 / コントローラ未接続の間は、fitra-cam が `/VMT/Room/Driver` を送っても **VMT デバイスが SteamVR に登録されない** (Manage Trackers に出ない)
3. Quest 装着 + 両コントローラ起動 → Manager が検知して Driver を arm → **そのタイミングで初めて** VMT デバイスが登録される
4. VRChat 起動 + FBT → **コントローラが Quest に正しく割り当たり、VMT に奪われない**
5. Jetson IP を Manager に手動設定せずに、Manager から `<学習IP>:39571` に `/fitra/hmd_pose` が 60Hz で届く (`curl http://<jetson>:8000/stats3d | jq .hmd` が Phase 15 同様に valid)
6. `vmt_hmd_pose_sender` を起動しなくても Phase 15 の自動 alignment フローが成立する
7. 本 doc + `docs/cpp-migration-plan.md` の段階実装 + 検証戦略表に Phase 15.5 行

## アーキテクチャ

```
[Windows]  SteamVR 起動 → vmt_manager auto-launch (overlay, 100ms loop)
   │
   │  fitra-cam ──(/VMT/Room/Driver 60Hz)──▶ Driver:39570
   │                                          └─ remoteEndpoint で Jetson IP 学習
   │  Driver ──(/VMT/Report/JetsonAddr <ip>)──▶ Manager:39571   [NEW, 既存Tx経路]
   │
   │  Manager:  IVRSystem で HMD valid && L/R controller connected を poll
   │    ├─ 揃ったら ──(/VMT/Set/RegistrationEnable 1)──▶ Driver:39570  [NEW, 既存Rx経路]
   │    └─ HMD pose ──(/fitra/hmd_pose 60Hz)──▶ <学習IP>:39571      [sender 吸収]
   │
   │  Driver:  WaitForHmd=true の間 RegisterToVRSystem を保留
   │           arm 受信後の最初のパケットで登録 → 奪取が起きない
   │
[Jetson]  fitra-cam: HmdPoseReceiver(39571) → HmdPoseBus → AutoAlignmentSolver  (無改修)
          Jetson 側に残る設定は --vmt-host=<windows-ip> のみ (fitra-cam が起点なので不可避)
```

ポートトポロジ (既存):
- Driver: Rx 39570 (fitra-cam の `/VMT/Room/Driver` + Manager のコマンド), Tx 39571 (Manager へステータス)
- Manager: Rx 39571, Tx 39570
- fitra-cam: Driver:39570 へ送信、`/fitra/hmd_pose` を 39571 で listen

## VMT フォーク側 実装仕様

### Driver (`vmt_driver/`, MSVC C++)

| 変更 | 内容 |
|---|---|
| `Config.{h,cpp}` | `bool m_WaitForHmd{false}` + `GetWaitForHmd()` + JSON `WaitForHmd` (既定 false = 現状互換) |
| 登録ゲート | グローバル/静的 `s_registrationEnabled` (既定: `!WaitForHmd`)。`ServerTrackedDeviceProvider` の OSC ハンドラ (現状 `RegisterToVRSystem(pose.enable)` を呼ぶ :257-275 付近) で、`!s_registrationEnabled` なら **登録をスキップ** (pose 適用は登録後のみなので捨ててよい。fitra-cam は 60Hz 継続送信するので arm 後の次パケットで登録される) |
| 制御コマンド | `/VMT/Set/RegistrationEnable i:enable` を OSC dispatch に追加 → `s_registrationEnabled = (enable != 0)` |
| 送信元 IP 通報 | `OSCReceiver::ProcessMessage` の `remoteEndpoint` から、`/VMT/Room/Driver` 受信時に送信元 IP を取り出し、ループバック以外なら `/VMT/Report/JetsonAddr s:<ip>` を Tx (Manager 向け) に投げる。連投を避けるため IP 変化時のみ |

安全策: `WaitForHmd` は **opt-in (既定 false)**。true かつ Manager 不在だと永久に登録されないので、auto-launch とセット運用を README に明記。

### Manager (`vmt_manager/`, C# WPF, OpenVR client)

| 変更 | 内容 |
|---|---|
| presence poll | `GenericTimer` (100ms) 内で `IVRSystem` を読み、HMD (index 0) が `bPoseIsValid` かつ L/R コントローラ (`GetControllerRoleForTrackedDeviceIndex` or `GetTrackedDeviceClass`==Controller) が `bDeviceIsConnected` を判定。一度「揃った」ら latch (フラッピング防止、以後 disarm しない) |
| arm 送信 | latch 成立で 1 回 `/VMT/Set/RegistrationEnable 1` を Driver:39570 へ |
| HMD pose 中継 | `GetDeviceToAbsoluteTrackingPose(TrackingUniverseStanding)` で HMD pose を取り、**Phase 15 と同一スキーマ** `/fitra/hmd_pose ,iffffffff (valid ts x y z qx qy qz qw)` を `<JetsonAddr>:39571` へ 60Hz 送信 (Tracking lost 時 `valid=0`) |
| Jetson IP 受信 | Driver から `/VMT/Report/JetsonAddr s:<ip>` を受け、HMD pose 中継先に設定。未受信の間は送信しない (or 設定 UI で手動上書き可) |
| auto-launch | `AddApplicationManifest` + `SetApplicationAutoLaunch(appkey, true)` を起動時に実行 → `.vrmanifest` を同梱し SteamVR 起動と同時に上がるよう登録 |

### OSC プロトコル (新規)

| 方向 | アドレス | 引数 | 用途 |
|---|---|---|---|
| Manager → Driver | `/VMT/Set/RegistrationEnable` | `i:enable` | 登録ゲート arm (1=登録許可) |
| Driver → Manager | `/VMT/Report/JetsonAddr` | `s:ip` | 学習した fitra-cam 送信元 IP の通報 |
| Manager → Jetson | `/fitra/hmd_pose` | `,iffffffff` | Phase 15 スキーマ (sender 吸収、不変) |

## 検証戦略

### Windows 手動スモーク (ctest 対象外)

**M1 (Driver ゲート)**:
1. `WaitForHmd=true` でビルド・インストール、SteamVR 再起動
2. HMD/コントローラ未接続のまま fitra-cam から `/VMT/Room/Driver` 送信 → SteamVR Manage Trackers に **VMT デバイスが出ない**こと
3. デバッグ用に OSC で手動 `/VMT/Set/RegistrationEnable 1` を送る → そこで初めて登録されること

**M2 (Manager presence + arm + 中継)**:
1. `vmt_manager` 起動、HMD 装着 + 両コントローラ起動
2. Manager が arm を送る → VMT デバイス登録、**コントローラは Quest に割り当たったまま**
3. Jetson: `nc -u -l 39571 | xxd` または `curl http://<jetson>:8000/stats3d | jq .hmd` で 60Hz の HMD pose が IP 手動設定なしで届くこと
4. HMD を外す → `valid=0` に切り替わること

**M3 (auto-launch)**:
1. `vmt_manager` を一度起動して auto-launch 登録 → 終了
2. SteamVR を起動 → Manager が自動で上がること
3. fitra-cam を **先に** 起動した状態で SteamVR + Quest を立ち上げる順序でも、コントローラが奪われないこと (ゲートが fail-safe に働く)

**M4 (E2E オンライン VR)**:
1. VRChat 起動 → FBT calibration → コントローラ正常 + chest avatar が HMD に一致
2. `vmt_hmd_pose_sender.exe` を **起動せず** Phase 15 の T-pose / motion キャリブが成立すること

### Jetson 側 (回帰確認のみ、fitra-cam 無改修)

```bash
# Phase 15 と同じ。送信元が Manager に変わっても /stats3d.hmd は不変であること
./cpp/build/main --enable-3d --keypoint-format=halpe26 \
  --vmt-out --vmt-host=<windows-ip> --vmt-port=39570 \
  --hmd-listen-enabled --hmd-listen-port=39571 --cam0 ... --cam1 ...
curl http://localhost:8000/stats3d | jq '.hmd'
```

合格基準: `hmd.valid=true` / `age_ms < 200`、`vmt_hmd_pose_sender` 不起動で自動 alignment が動く。

## リスク

| ID | 内容 | 対応 |
|---|---|---|
| R1 | `WaitForHmd=true` かつ Manager 不在で永久未登録 | opt-in 既定 false + auto-launch とセット運用を README 明記。Manager 落ちても次回 SteamVR 起動で復帰 |
| R2 | 過去セッションで VMT がコントローラ枠に紐付いた状態が SteamVR に永続化されている | 初回のみ SteamVR のデバイス/バインディング設定をクリア (手順を README に)。以後はゲートで再発しない |
| R3 | Quest コントローラが「置くと寝る」ため presence が落ちる | latch 方式 (一度揃ったら disarm しない)。arm 後はコントローラが寝ても登録は維持 |
| R4 | Manager 起動が fitra-cam の初回パケットより遅れる | Driver ゲートが fail-safe (arm まで登録保留) なので順序非依存。登録が待つだけ |
| R5 | `remoteEndpoint` がルータ/NAT 経由で書き換わる | 同一 LAN 前提 (Phase 14/15 と同じ)。NAT 跨ぎは out of scope |
| R6 | fork が upstream から乖離し再マージが重い | 差分を Driver (ゲート + コマンド + IP 通報) / Manager (poll + arm + 中継 + auto-launch) に局所化。upstream 追従時の衝突面を最小化 |

## Out of scope (将来候補)

- `/fitra/hmd_pose` に L/R コントローラ pose を載せ、Jetson 側で手の可視化に使う
- Manager 設定 UI で WaitForHmd / Jetson IP を GUI 操作
- HMD だけでなく特定 Vive Tracker を reference に選ぶ (Phase 15 から継続の Phase 16 候補)
- upstream VMT への PR (本家に登録ゲートを提案)

## 関連ドキュメント

- [`phase15-vmt-hmd-auto-align.md`](phase15-vmt-hmd-auto-align.md) — HMD pose 駆動の自動 alignment (前提、sender の元実装)
- [`phase14-vmt-steamvr.md`](phase14-vmt-steamvr.md) — VMT 経路の確立
- [`cpp-migration-plan.md`](../cpp-migration-plan.md) — 段階実装 + 検証戦略表
