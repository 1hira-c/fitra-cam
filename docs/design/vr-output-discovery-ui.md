# vr-output: discovery を UI に露出する（出力ターゲット自動検出のUI対応）

(着手日 2026-06-27 / 関連: [`vr-output-zeroconf-discovery.md`](vr-output-zeroconf-discovery.md) のバックエンド実装が前提)

## 背景 / 動機

zeroconf discovery（[`vr-output-zeroconf-discovery.md`](vr-output-zeroconf-discovery.md)）で
VMT 出力先の `ip:port` は同一 LAN 上で実行時に自動解決できるようになった。既定動作も
discovery（`vmt_discovery=true` かつ `vmt_host=""` で起動時に解決し続ける）。
にもかかわらず Web-UI 側は **手動 IP 入力欄しか持たず**、自動検出を一切露出していない:

- 初期設定 (`/setup`) の「3. 出力ターゲット」VMT カードは `host` テキスト欄のみ。
  「自動検出を使う/使わない」の選択肢がない。
- `POST/GET /api/config` (`crow_routes_setup_mode.cpp` の `merge_config` / `draft_to_json`)
  が `vmt_out / host / port / hmd_listen_enabled` の 4 つしか往復しておらず、
  UI から `vmt.discovery` を設定する経路が欠落している（YAML 永続層
  `main_config.cpp` の `load_vmt` / `emit_main_config` は対応済み）。
- 検出状況は `statsText.ts` でビューアの stats テキストに 1 行埋もれているだけ。

完了条件: 初期設定で「IP を打たずに自動検出に任せる」が UI から選べ、起動後ビューアで
検出済みターゲットが一目で分かること。スコープは **VMT のみ**（SlimeVR は zeroconf
の対象外）。

## 検討した案

- **没A: 「検出」ボタンで host 欄に IP を焼き込む** — 検出された IP を設定時に固定欄へ
  書き込む案。ユーザー要望で却下: 「設定時に固定するのではなくランタイム自動かどうかを
  選択したい（固定 IP の人はいいが全員ではない）」。IP を焼き込むと DHCP で VMT 機の
  アドレスが変わったとき追従できなくなり、discovery の利点（実行時再解決）を殺す。
- **没B: setup 画面で検出ピアをライブ表示** — 理想的 UX だが、サーバは Setup モード
  (GPU なし軽量 Crow) と Run モードが排他で、`DiscoveryBeacon` は Run モードでしか
  起動しない (`pose_relay_builder.cpp:21`, setup は `bus3d=nullptr`)。setup 中に
  ライブ表示するには setup モードへビーコン生存を足す中規模改修が必要。今回は見送り、
  検出状況は Run モードのビューアで見せる。
- **採用: モードセレクタ + ビューア状況表示** — setup は「自動検出（ランタイム）/
  手動でIP指定」のラジオのみ（プレビューなし）。自動 = `discovery=true` + `host=""`、
  手動 = `discovery=false` + `host`。検出済みターゲットはビューアのヘッダにチップ表示。
  バックエンドは config 往復に `vmt.discovery` を足すだけ（既存 YAML 層を使う）。

## 採用設計

### モード ⇔ バックエンド config の対応

| UI モード | `vmt.discovery` | `vmt.host` | 実行時挙動 |
|---|---|---|---|
| 自動検出（ランタイム） | `true` | `""`（空） | ビーコン起動・毎 tick 再解決し IP 変化に追従 |
| 手動でIP指定 | `false` | `"192.168.x.x"` | ビーコン起動せず固定先へ送信 |

ランタイムの真の判定は「`host` が空かどうか」（非空 host は manual override で `discovery`
より優先）。UI 上のモード判定は `vmtAuto = vmt.discovery && vmt.host.trim() === ""`。
ラジオ選択時に両フィールドを同時に整合させる（自動選択 → `{discovery:true, host:""}`、
手動選択 → `{discovery:false}` で host はユーザー入力に委ねる）。

### M1: config 往復 + setup モードセレクタ

- `crow_routes_setup_mode.cpp`: `draft_to_json` の vmt 節に `"discovery":<bool>` を追加（JSON
  キーは YAML と同じ `discovery`）。`merge_config` は inbound の `discovery` を読まず
  `d.vmt_discovery = d.vmt_host.empty()` と **host 空否から導出**する（single source of truth。
  下記レビュー反映を参照）。
- `web-ui/src/types/bundle.ts`: `ConfigVmt` に `discovery: boolean` を追加。
- `web-ui/src/routes/SetupPage.tsx`: VMT カードを「enable / hmd_listen」行 +
  モードラジオ + （自動なら説明文 / 手動なら host・port 欄）に再構成。

### M2: ビューアに検出状況チップ

- `web-ui/src/lib/statsText.ts`: `discoveryStatus(bundle): HmdStatus | null` を追加
  （`resolved.have` → `出力先 <name> <ip>:<port>` cls=live / 未解決 → `出力先 検索中… (N)` /
  discovery ブロック無しだが vmt 有効 → `出力先 手動` / vmt off → `null`）。**全分岐を
  `bundle.vmt` の有無でゲート**する: backend は VMT 出力有効時のみ `vmt` ブロックを出すが、
  beacon は HMD-listen punch 経路（`vmt_out=false`）でも起動するため、ゲート無しだと送信して
  いないのに `出力先` チップが出てしまう（下記レビュー反映）。
- `web-ui/src/routes/ViewerPage.tsx`: stats throttle で `discoveryStatus` を state 化し、
  ヘッダ `conn-group` に既存 conn チップと並べて表示。既存の stats `<pre>` の
  `discovery` 行はそのまま残す（チップ=一目、pre=詳細）。

## Milestone

- **M1**（コミット1）: backend config 往復 + `ConfigVmt.discovery` 型 + setup モードセレクタ。
  これだけで「IP を打たずに自動検出に任せる」が UI から選べる。
- **M2**（コミット2）: ビューアの検出状況チップ。起動後に解決先/検索中が一目で分かる。

## 検証

- C++: `cmake --build cpp/build -j` がパス（config 往復の追加フィールドはコンパイルのみ）。
- web-ui: `pnpm -C web-ui build`（tsc + vite）がパス。
- 手動: setup で自動選択 → 保存 → 生成 YAML に `vmt.discovery` 反映 & `host` 空、を確認。
  手動選択で host/port 欄が出ること。Run モードで VMT 検出時にビューアヘッダへ
  `出力先 …` チップが出ること、未検出時 `検索中…` になること。

## レビュー反映 (2026-06-27, PR #46)

自動レビュー (Gemini / Codex) と社内 code-review で二重フラグ (`discovery` bool + `host`
string) の不整合が指摘された。runtime の真の判定は host 空否のみ（`pose_relay_builder.cpp` /
`validate_options`）なので、**`discovery` を host 空否から導出する single source of truth** に
寄せて以下を是正:

- **dead 状態 `{discovery:false, host:""}`**（手動ラジオを選び host 未入力で保存）: 当初認識の
  「無音 idle」は誤りで、実際は `validate_options` (`main_config.cpp` の
  `!vmt_discovery && vmt_host.empty()`) が `--vmt-out needs a destination` で **fail** し
  proceed/保存が弾かれる（Gemini 指摘）。`merge_config` で `vmt_discovery = vmt_host.empty()`
  と導出し、空 host は auto に正規化（既存の「空欄→自動検出」案内が真になる）。
- **manual override `{discovery:true, host:"x.x"}` の取り違え / 入力欄消失 footgun**（Codex 指摘）:
  読み込み時に `normalizeCalibPaths` で `vmt.discovery = host.trim()===""` に正規化。legacy/手編集
  config でも正しいラジオが選ばれ、manual で host を一旦消しても auto に飛ばない（discovery は
  false のまま）。server 側も `merge_config` の導出で同じ invariant を担保。
- **`discoveryStatus` の誤チップ**（code-review 指摘）: 全分岐を `bundle.vmt` でゲートし、
  VMT 出力 OFF + HMD-listen discovery 時に `出力先` を誤表示しないよう修正。

## 残課題

- **M3（保留）**: 複数ピア時に「このVMTを選ぶ」→ `vmt_pair_id` 指定 UI。実行時変更 API が
  無く（設計上 CLI/config 限定・再起動要）大きめ。要望次第で別トピック化。
- setup 中ライブ検出（没B）は setup モードへのビーコン生存追加が必要。backlog。
