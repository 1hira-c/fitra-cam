# vr-output: VMT 側 zeroconf ディスカバリ仕様 (VMT フォーク受け渡し用)

(作成日 2026-06-23 / fitra-cam 側設計: [`vr-output-zeroconf-discovery.md`](vr-output-zeroconf-discovery.md) /
pose wire: [`vr-output-vmt-pose-relay-wire-spec.md`](vr-output-vmt-pose-relay-wire-spec.md))

> **この文書の位置づけ**: Windows VMT フォーク (`vmt_manager`) 側 (issue #36 の **M3**) を
> fitra-cam リポジトリを読まずに実装できるよう、ディスカバリの wire 契約と挙動を自己完結で
> まとめたもの。fitra-cam (Jetson) 側の M1/M2 は実装済みで、本仕様どおりに announce を投げ合えば
> 相互接続が成立する。**pose 本体の wire は一切変更しない** — discovery は IP:port を解決する
> 制御プレーンの追加のみ。

## 1. 背景 / 目的

VMT ⇔ Jetson 接続で、ユーザーが手で埋める唯一の面倒な設定が Jetson 側の
`vmt.host` (= Windows PC の実 IP) だった。両経路とも「Jetson が PC IP を先に知っている」前提で
成立する鶏卵問題になっている:

- **Jetson → PC (pose)**: `VmtPublisher` が `vmt.host:39570` へ `/VMT/Room/Driver`。VMT Driver は
  OSC の src IP から Jetson IP を学習。
- **PC → Jetson (HMD/controller)**: VMT Manager が `Jetson:39571` へ `/fitra/tracked_pose`。Jetson は
  `vmt.host:39570` へ `/fitra/punch` を撃って自 IP を学習させる。

**完了条件**: Jetson と Windows のどちらも IP を入力せず起動するだけで、同一 LAN 上で相互に相手を
見つけ、pose 経路と HMD pose 経路が自動で成立する。明示指定 (Jetson 側 `vmt.host` 直書き / VMT 側
の手動ピン) は引き続き最優先で効く。

## 2. Transport / アドレス

- プロトコル: UDP / OSC 1.0。外部依存なし (WinSock + 自前 OSC writer で実装可)。
- ディスカバリ group: **`239.255.42.99`** (admin-scoped multicast、設定可)。
- ディスカバリ port: **`39580`** (既存 3957x ファミリ隣接)。
- 送信: **1 Hz** で、同一データグラムを **multicast group と `255.255.255.255` (subnet broadcast) の
  両方**へ投げる二段構え。`IP_MULTICAST_TTL=1` (リンクローカル、サブネット外へ漏らさない)。
  broadcast 送信には `SO_BROADCAST` が必要。
- 受信: `39580` を **`SO_REUSEADDR` + `INADDR_ANY` で bind** し (group アドレスに bind しない)、
  `IP_ADD_MEMBERSHIP` で group に join。`INADDR_ANY` bind なら multicast と broadcast の両 leg を受けられる。
- pose 本体の wire (`/VMT/Room/Driver` 39570 / `/fitra/tracked_pose` 39571 / `/fitra/punch`) は
  **完全に不変**。

二段送信のため同じ announce が multicast + broadcast で 2 回 (環境により loopback でさらに) 届く。
**`instance_id` をキーにした冪等 upsert** で吸収する (下記)。

## 3. announce メッセージ (OSC 1.0)

双方が **同一フォーマット**を投げ合い、`role` で送信者を区別する。1 つの announce で受信側は
「相手の pose 送信先 = `src_ip : osc_recv_port`」を確定できる。

```text
address = "/fitra/announce"
typetag = ",sssiiss"
args    = role(s)            "jetson" | "vmt"
          instance_id(s)     安定 ID (ホスト由来。後述、相手は文字列比較するだけ)
          instance_name(s)   人間可読名 (UI 表示用, 例 "VMT-PC")
          proto_version(i)   1   (= 本仕様)
          osc_recv_port(i)   この送信者が pose プレーンを受ける port
                             vmt    = 39570 (/VMT/Room/Driver 待受)
                             jetson = 39571 (/fitra/tracked_pose 待受)
          capabilities(s)    csv "pose,hmd,controller" 等 (情報用、選択には未使用)
          pairing_token(s)   "" = 任意 / 非空 = 同 token のピアのみ採用
```

OSC 1.0 エンコード規則 (再掲): 文字列は NUL 終端 + 4 byte 境界へ 0 padding、int32 は 4 byte big-endian、
typetag は先頭カンマ。standalone メッセージ (bundle 不要)。

### Golden バイト列 (照合用)

VMT 側 (`role="vmt"`, `instance_id="vmt-pc"`, `instance_name="VMT-PC"`, `proto_version=1`,
`osc_recv_port=39570`, `capabilities="pose"`, `pairing_token=""`) の **68 byte** エンコード結果:

```text
2f 66 69 74 72 61 2f 61 6e 6e 6f 75 6e 63 65 00   "/fitra/announce\0"            (16)
2c 73 73 73 69 69 73 73 00 00 00 00               ",sssiiss\0" + pad             (12)
76 6d 74 00                                       "vmt\0"            role        (4)
76 6d 74 2d 70 63 00 00                           "vmt-pc\0" + pad   instance_id (8)
56 4d 54 2d 50 43 00 00                           "VMT-PC\0" + pad   instance_name(8)
00 00 00 01                                       int32 = 1          proto       (4)
00 00 9a 92                                       int32 = 39570      osc_recv_port(4)
70 6f 73 65 00 00 00 00                           "pose\0" + pad     capabilities(8)
00 00 00 00                                       "" (NUL + pad)     token       (4)
```

(`0x9A92 = 39570`。空文字列引数は NUL 1 byte を 4 byte に padding した 4 個の 0。)
fitra-cam 側のパーサ/エンコーダ実装は `cpp/src/vmt/discovery_announce.cpp`、golden 回帰は
`cpp/tools/test_discovery.cpp`。

### パース時の reject 条件 (fitra-cam 側と合わせること)

address/typetag 不一致、バッファ長不足、文字列の NUL 終端欠落、未知 role 文字列 → 破棄。
proto_version はパースのみ行い、採用判定 (下記 admission) で `!= 1` を弾く。引数は文字列と int のみ
なので NaN/Inf の懸念はない。

## 4. VMT Manager 側の手順 (role=vmt)

1. 起動時に `39580` を `SO_REUSEADDR`+`INADDR_ANY` で bind、group `239.255.42.99` に join、
   `SO_BROADCAST` 有効化、`IP_MULTICAST_TTL=1`。
2. **1 Hz** で `role="vmt"`, `osc_recv_port=39570` の announce を group + `255.255.255.255` の両方へ送信。
3. 受信した `role="jetson"` の announce を採用判定 (下記) に通し、`PeerRegistry` 相当へ
   `{src_ip, osc_recv_port, instance_id, instance_name, last_seen}` を **`instance_id` キーで upsert**。
4. 採用ピア (下記の選択ロジック) の `src_ip:osc_recv_port` を **pose-relay の送信先**に設定する:
   VMT Manager が Jetson へ `/fitra/tracked_pose` を送る宛先 (= 採用 jetson の `src_ip` : その announce の
   `osc_recv_port` = 通常 39571)。
5. pose Driver 側の Jetson IP 学習 (`/VMT/Room/Driver` / `/fitra/punch` の src IP) は従来どおり。
   discovery は「どの Jetson か」を決めて relay 宛先を埋めるだけで、pose 受信ロジックは不変。

WinSock の要点 (擬似コード):

```cpp
SOCKET s = socket(AF_INET, SOCK_DGRAM, 0);
BOOL yes = TRUE;
setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (char*)&yes, sizeof yes);
setsockopt(s, SOL_SOCKET, SO_BROADCAST, (char*)&yes, sizeof yes);
sockaddr_in a{}; a.sin_family=AF_INET; a.sin_addr.s_addr=htonl(INADDR_ANY);
a.sin_port=htons(39580);
bind(s, (sockaddr*)&a, sizeof a);
ip_mreq mreq{}; inet_pton(AF_INET,"239.255.42.99",&mreq.imr_multiaddr);
mreq.imr_interface.s_addr=htonl(INADDR_ANY);
setsockopt(s, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq, sizeof mreq);  // 失敗は警告に留め broadcast leg で継続
DWORD ttl=1; setsockopt(s, IPPROTO_IP, IP_MULTICAST_TTL, (char*)&ttl, sizeof ttl);
// send: sendto(group:39580) と sendto(255.255.255.255:39580) の二発を 1 Hz
// recv: recvfrom で src_ip を取得 → parse → admission → upsert → 採用ピア更新
```

### 自己除外 / 重複吸収

multicast loopback が既定有効だと自分の announce も届く。`instance_id == 自分の instance_id` の
announce は破棄する (二段送信・loopback の重複も `instance_id` upsert で 1 ピアに収束)。

## 5. ピア選択 / ペアリング (fitra-cam 側と同一意味論)

採用判定 (admission) — 次を全て満たす announce のみ registry に入れる:

- `proto_version == 1`
- `role` が「相手役割」(VMT 側は `jetson`、Jetson 側は `vmt`)
- 自分の announce でない (`instance_id` 不一致)
- 自分の `pairing_token` が非空なら、announce の token がそれと一致

採用ピア選択 (生存ピアのうち):

- `peer_timeout` (既定 **5 s**) 以内に announce を受けたピアのみ「生存」とみなす (stale は除外)。
- 手動ピン (`pair_id` / UI 選択) があればその `instance_id` のピア。無ければ **生存ピアのうち
  `instance_id` が辞書順最小** (決定的)。これにより単一ピアは自動採用、複数でも安定。
- `pairing_token` は同 token のピアのみ採用してリグ間クロストークを遮断 (平文、LAN 前提)。

Liveness: announce 自体がハートビート。`peer_timeout` 無受信で peer を lost とし UI に反映するが、
**pose は best-effort で last-known 宛に流し続ける** (再 announce で自動復帰)。

### instance_id について

各ホストが自前生成する安定 ID。fitra-cam 側はホスト名の FNV-1a 64bit hex (16 桁) を使うが、
**ID は相手にとって不透明で文字列比較されるだけ**なので、VMT 側はハッシュ方式を合わせる必要はない。
再起動間で安定し、同一 LAN 内で衝突しなければよい (例: マシン GUID / MAC 由来 hex)。`instance_name` は
UI 表示用の任意ラベル。

## 6. pose 経路との関係 (不変条件)

- `/VMT/Room/Driver` (39570 受信), `/fitra/tracked_pose` (39571 受信), `/fitra/punch` は **不変**。
  discovery は「IP:port 解決」だけを担う。
- 退路: fitra-cam 側で `vmt.host` を明示指定すると、その経路は discovery を使わず直書き宛先になる
  (Jetson 側はその場合 beacon を起動しない)。VMT 側も手動ピンを最優先にできる。
- Manager UI: 検出ピア一覧 + 明示ピン (pair_id 相当) を出すのが望ましい。fitra-cam 側は `/stats3d` の
  `discovery` ブロック (resolved peer / peers / age) で同等情報を出している。

## 7. 既知リスク / 制約

- **Wi-Fi multicast 取りこぼし**: AP の IGMP snooping 依存。broadcast 二段構えで緩和、最悪時は
  手動 IP 指定で確実に回避できる旨をユーザー文書に明記する。
- **複数 NIC / NAT**: announce の出口 IF と pose 送信元 IF が一致する必要がある (VMT の src IP 学習が
  壊れないように)。複数 IF 環境では bind IF 選択 or 手動指定。
- **IPv4 のみ** (当面)。
- `pairing_token` は **平文** (LAN 前提)。秘匿が要る共有ラボでは HMAC 化は別途。
- 送信値は finite (announce は文字列/int のみなので該当せず)。

## 8. 検証

fitra-cam 側 (M1/M2、実装済):

```bash
ctest --test-dir cpp/build --output-on-failure -R 'discovery|announce|peer_registry|tracked_pose'
```

実機 (M4):

- Jetson / Windows を**両方 IP 無指定**で起動 → 自動接続。
- fitra-cam の `/stats3d` で VMT pose `valid=true`、HMD pose `age_ms < 100`、`discovery.resolved.have=true`。
- 複数ピアでの `pair_id` / `pairing_token` ピンの確認。Wi-Fi / 有線の両方で接続成立を確認。

## 9. VMT フォーク側の実装タスク (M3 チェックリスト)

- [ ] `39580` の multicast/broadcast ソケット (上記 §4 の WinSock 設定)。
- [ ] `/fitra/announce` の encode (§3 golden と byte 一致) / parse (§3 reject 条件)。
- [ ] 1 Hz announce (`role="vmt"`, `osc_recv_port=39570`) + `role="jetson"` の受信・upsert。
- [ ] admission + 選択ロジック (§5) と `peer_timeout` 5 s の stale 判定。
- [ ] 採用 jetson の `src_ip:osc_recv_port` を pose-relay 送信先へ自動設定。
- [ ] Manager UI に検出ピア一覧 + 手動ピン。
- [ ] fitra-cam 側は無改修で接続成立することを実機確認 (§8 M4)。
