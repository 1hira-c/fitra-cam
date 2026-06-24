# vr-output: VMT ⇔ Jetson zeroconf ディスカバリ

(着手日 2026-06-18 / 関連: [`vr-output-vmt-pose-relay-wire-spec.md`](vr-output-vmt-pose-relay-wire-spec.md),
[`vr-output-zeroconf-discovery-vmt-spec.md`](vr-output-zeroconf-discovery-vmt-spec.md) (VMT 側受け渡し仕様),
[`archive/phase15.5-vmt-registration-gate.md`](../archive/phase15.5-vmt-registration-gate.md) /
memory `project-vmt-ip-learning-punch`)

> **ステータス: 完了 (M1-M4、2026-06-25、`vr-output/zeroconf-discovery`)。** Jetson 側の純ロジック
> (M1) + ランタイム配線・config/CLI・`/stats3d`/WebUI (M2) に続き、Windows VMT フォーク側 (M3、
> [`vr-output-zeroconf-discovery-vmt-spec.md`](vr-output-zeroconf-discovery-vmt-spec.md) 準拠) を実装し、
> 両機 IP 無指定の実機確認 (M4) を完了。`/stats3d` で `discovery.resolved.have=true` /
> HMD pose `valid` `age_ms<100` / VR 内トラッカー表示まで成立。

## 背景 / 動機

VMT ⇔ Jetson を繋ぐとき、現状ユーザーが手で埋める唯一の面倒な設定が
**Jetson 側の `vmt.host` (= Windows PC の実 IP)** である。これが起点になって全経路が
成立している:

- **Jetson → PC (pose)**: `VmtPublisher` が `vmt.host:39570` へ `/VMT/Room/Driver` を送る。
  VMT Driver は OSC の src IP から Jetson IP を学習する (返信不要)。
- **PC → Jetson (HMD/controller)**: VMT Manager が Jetson IP:39571 へ `/fitra/tracked_pose`
  を送る。Jetson 側は `vmt.host:39570` へ `/fitra/punch` を撃って自 IP を VMT に学習させる
  (`TrackedPoseReceiver`、memory `project-vmt-ip-learning-punch`)。

つまり**両経路とも「Jetson が PC IP を先に知っている」ことが前提**で、ここが鶏卵問題になっている。
開発者本人は IP を直書きで困らないが、他ユーザーに配る時にこの 1 手が運用の壁になる。

**完了条件**: Jetson と Windows のどちらも IP を一切入力せず起動するだけで、同一 LAN 上で相互に
相手を見つけて pose 経路と HMD pose 経路が自動的に成立する。明示指定 (`vmt.host` 直書き) は
引き続き最優先で効き、ディスカバリを上書きできる (パワーユーザー向け退路)。

## 検討した案

| 案 | 採否 | 理由 |
|---|---|---|
| **A. OSC マルチキャストビーコン (採用)** | ✅ | 既存の hand-rolled UDP/OSC スタック (`OscWriter` / OSC parser / 既存 punch) をそのまま流用でき、外部依存ゼロ。Windows 側も WinSock + 既存 OSC writer だけで実装できる。pose wire を一切変えない純粋な制御プレーン追加。 |
| B. mDNS / DNS-SD (Bonjour/Avahi) | ❌ | 標準 zeroconf で `avahi-browse` 等の外部ツールから可視という利点はあるが、(1) Windows 側 publish に Bonjour SDK / mdns ライブラリ依存が増える、(2) TXT レコードのサイズ/エスケープ制約、(3) どのみち「どのピアと組むか」の意味論は自前で上乗せが必要 ── でリポジトリの「UDP/OSC は手書き」方針から外れる。将来 *外部ツール連携* が欲しくなったら別レイヤとして追加可能 (排他ではない)。 |
| C. クラウド/ランデブーサーバ | ❌ | インターネット必須・外部依存。LAN 内 1:1 の VR リグには過剰。 |
| D. QR / ペアリングコード手入力 | ❌ | zeroconf ではない。「他ユーザーの手間を消す」目的に逆行。 |

### ペアリング方針 (複数ピア時)

- **採用**: 単一ピアなら自動接続。複数検出時は `instance_id` 最小を採用 (決定的)。
  config の `vmt.pair_id` / Windows UI の検出リストで明示 pin 可能。`pairing_token` でリグ間
  クロストークを遮断。
- 没: 「常に明示選択」 ── 誤接続ゼロだが単一リグでも毎回選択操作が要り、目的に反する。

### transport の細目 (multicast vs broadcast)

リンクローカル multicast (`239.255.x.x`, TTL=1) を既定とする。ただし民生 Wi-Fi AP の IGMP
snooping / multicast-to-unicast 変換で取りこぼす環境があるため、**同一データグラムを subnet
broadcast (`255.255.255.255`) にも送る二段構え**にする (受信側は重複を `instance_id` で冪等吸収)。
有線 LAN では multicast で十分。

## 採用設計

### Transport / アドレス

- ディスカバリ group: `239.255.42.99` (admin-scoped, 設定可)、port `39580` (既存 3957x ファミリ隣接)。
- 送信: 1 Hz で group + `255.255.255.255` の両方へ announce (TTL=1, LAN 外へ漏らさない)。
- 受信: `39580` を `SO_REUSEADDR` で bind し group に join。
- pose 本体の wire (`/VMT/Room/Driver` 39570 / `/fitra/tracked_pose` 39571 / `/fitra/punch`) は
  **完全に不変**。ディスカバリは「IP:port を解決する制御プレーン」のみを担う。

### Announce メッセージ (OSC 1.0)

双方が同一フォーマットを *投げ合う*。役割は `role` で区別する。

```text
address = "/fitra/announce"
typetag = ",sssiiss"
args    = role(s)            "jetson" | "vmt"
          instance_id(s)     安定 ID (hostname-SHA1。SlimeVR MAC 安定化と同方式)
          instance_name(s)   人間可読名 (UI 表示用, 例 "Living Rig")
          proto_version(i)   1
          osc_recv_port(i)   このホストが pose プレーンを受ける port
                             jetson=39571 (/fitra/tracked_pose 待受)
                             vmt   =39570 (/VMT/Room/Driver 待受)
          capabilities(s)    csv "pose,hmd,controller"
          pairing_token(s)   "" = 任意 / 非空 = 同 token のピアのみ採用
```

1 つの announce で相手は「相手の送信先 = src_ip : osc_recv_port」を確定できる。

### 解決フロー (Jetson 側、Windows も対称)

1. 起動時に `39580` を bind / group join し、`role=jetson` を 1 Hz で announce 開始。
2. `role=vmt` の announce 受信 (proto_version 互換 & token 一致) → `PeerRegistry` に
   `{src_ip, osc_recv_port, instance_id, last_seen}` を upsert。
3. 採用ピア決定: `vmt.pair_id` 指定があればそれに一致するもの。無ければ生存ピアのうち
   `instance_id` 最小。採用ピアが変わったら **ランタイムで** 反映:
   - `VmtPublisher` の送信先 → `src_ip:osc_recv_port`
   - `TrackedPoseReceiver` の punch 先 → 同上
4. **Liveness**: announce 自体がハートビート。`peer_timeout` (既定 5 s) 無受信で peer を lost
   とし、`/stats3d` に反映 (pose は best-effort で last-known 宛に流し続ける。再 announce で自動復帰)。

### 所有権 / データフロー

- 新規 `DiscoveryBeacon` (名前空間 `fitra::vmt`) が discovery socket・announce 送信スレッド・
  `PeerRegistry` を所有。解決済みエンドポイントを軽量 bus で公開。
- `VmtPublisher` / `TrackedPoseReceiver` はその bus を読み、宛先を atomically 差し替える
  (現状ビルド時固定の `host`/`punch_host` を「未指定なら discovery 由来」に一般化)。
- announce の encode/parse は既存 `OscWriter` / OSC parser を再利用。

### 設定 / 上書き (退路)

| key | 既定 | 意味 |
|---|---|---|
| `vmt.discovery` | `true` | ディスカバリ有効。`--no-vmt-discovery` で無効 |
| `vmt.host` | (空) | **明示指定が最優先**。非空ならその経路は discovery を使わず直書き宛先 |
| `vmt.pair_id` | (空) | 採用ピアを `instance_id` で pin |
| `vmt.pairing_token` | (空) | リグ間クロストーク遮断 (同 token のみ採用) |
| `vmt.discovery_group` / `vmt.discovery_port` | `239.255.42.99` / `39580` | group/port 上書き |

`vmt.host` を入れれば現行と完全に同じ挙動 (後方互換・パワーユーザー pin)。

## Milestone

各 M = コミット境界。M1/M2 は fitra-cam (Jetson) 側、M3 は Windows VMT フォーク側。

- **M1 (Jetson, fitra-cam)** — ✅ 実装済 (2026-06-23): `discovery_announce` (announce encode/parse +
  `stable_instance_id`) + `peer_registry` (`announce_admissible` / `select_peer` / `PeerRegistry` /
  `DiscoveryEndpointBus`) を純関数で新設。pose 経路へは未配線。`cpp/tools/test_discovery.cpp` の
  10 ケース (golden / round-trip / reject / id 決定性 / 単一・最小 id・pin・token・stale・proto) で固定。
- **M2 (Jetson, fitra-cam)** — ✅ 実装済 (2026-06-23): `DiscoveryBeacon` (39580 multicast+broadcast,
  1 Hz) を新設し `PoseRelay` に所有。`VmtPublisher` を `sendto()`+差し替え可能宛先化、
  `TrackedPoseReceiver` の punch をランタイム解決。`vmt.host` 空 → discovery / 非空 → 手動
  (既定を空に変更)。`vmt.{discovery,pair_id,pairing_token,discovery_group,discovery_port,
  instance_name,peer_timeout_s}` を config/CLI/emit/validate に追加。`/stats3d`+`/ws3d`+WebUI に
  検出ピア表示。pose wire は不変 (既存 golden が回帰ガード)。
- **M3 (Windows, vmt_manager)** — ✅ 実装済 (2026-06-25): VMT フォークに `DiscoveryAnnounce.cs`
  (OSC 1.0 codec、golden と byte 一致) + `ZeroconfDiscovery.cs` (39580 bind/join、1 Hz multicast+broadcast、
  `role=vmt` announce + `role=jetson` browse、admission/最小 id/pin/token/timeout 5 s) を実装。採用 jetson の
  `src_ip:osc_recv_port` を pose-relay 送信先へ自動設定 (driver-learned fallback より優先)。Manager UI に
  検出リスト + pin/token。fitra-cam 側は M1/M2 のままで無改修。
  → 受け渡し仕様: [`vr-output-zeroconf-discovery-vmt-spec.md`](vr-output-zeroconf-discovery-vmt-spec.md)。
- **M4 (実機)** — ✅ 確認済 (2026-06-25): Jetson / Windows を**両方 IP 無指定**で起動 → 自動接続。
  `/stats3d` で `discovery.mode=discovery` / `resolved.have=true` (→ `VMT-SH_MAIN @ 172.34.1.9:39570`) /
  `vmt.host=""` / HMD pose `valid=true` `age_ms≈3.8`、VR 内トラッカー表示まで成立。逆経路 pose は
  Windows で SteamVR 起動が前提 (`HmdPoseTick` の `util==null` ガード) という診断のみ別途記録。

## 検証

fitra-cam (M1/M2):

```bash
ctest --test-dir cpp/build --output-on-failure -R 'discovery|announce|peer_registry|tracked_pose'
```

- announce の golden バイト列 (typetag/順序固定)。
- parse の round-trip と不正パケット reject (短い/型違い/非 finite なし=文字列のみ)。
- ピア選択: 単一→自動 / 複数→最小 id / `pair_id` pin / token 不一致は無視 / `peer_timeout` で lost。

実機 (M4): 両機 IP 無指定起動 → `/stats3d` で VMT pose `valid=true` & HMD pose `age_ms<100`。
Wi-Fi / 有線の両方で接続成立を確認。

## 残課題

- **Wi-Fi multicast 信頼性**: AP 依存。broadcast 二段構えで緩和するが、最悪時は `vmt.host`
  手動指定で確実に回避できる旨をユーザー文書に明記。
- **複数 NIC / NAT**: Jetson が複数 IF を持つ場合、announce の出口 IF と pose 送信元 IF が
  一致する必要 (VMT の src IP 学習が壊れる)。bind IF を選べるようにするかは backlog。
- **IPv6**: 当面 IPv4 のみ。
- **セキュリティ**: `pairing_token` は平文。LAN 前提なので可。共有ラボで秘匿が要るなら HMAC 化は別途。
- mDNS 連携 (外部ツール可視化) を将来欲しくなったら、本ビーコンと排他せず追加レイヤとして検討。
