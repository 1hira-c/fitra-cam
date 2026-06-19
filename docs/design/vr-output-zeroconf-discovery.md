# vr-output: VMT ⇔ Jetson zeroconf ディスカバリ

(着手日 2026-06-18 / 関連: [`vr-output-vmt-pose-relay-wire-spec.md`](vr-output-vmt-pose-relay-wire-spec.md),
[`archive/phase15.5-vmt-registration-gate.md`](../archive/phase15.5-vmt-registration-gate.md) /
memory `project-vmt-ip-learning-punch`)

> **ステータス: 仕様起票のみ (2026-06-18)。実装は未着手。** Windows VMT フォーク側
> (`vmt_manager`) の開発と歩調を合わせるため、wire / 挙動の契約を先に固定する。

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

- **M1 (Jetson, fitra-cam)**: `DiscoveryBeacon` + `PeerRegistry` 新設。announce encode/parse、
  ピア選択 (最小 id / token フィルタ / stale timeout) を純関数で実装。まだ pose 経路へは未配線。
  ── ctest で固定。
- **M2 (Jetson, fitra-cam)**: 解決エンドポイントを `VmtPublisher` 送信先 + `TrackedPoseReceiver`
  punch 先へランタイム配線。`vmt.host` 空 → discovery / 非空 → 手動。`/stats3d` + WebUI に
  検出ピア (name / id / age) を表示。CLI `--vmt-discovery` 系を追加。
- **M3 (Windows, vmt_manager)**: `role=vmt` announce + `role=jetson` browse を実装し、pose-relay
  送信先を自動設定。Manager UI に検出リスト + pin。fitra-cam 側は M1/M2 のままで無改修。
- **M4 (実機)**: Jetson / Windows を**両方 IP 無指定**で起動 → 自動接続 → VMT pose +
  HMD continuous align が成立することを確認。複数ピア時の pin / token も確認。

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
