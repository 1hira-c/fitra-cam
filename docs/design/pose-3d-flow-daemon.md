# pose-3d: flow daemon — main の常駐 daemon 化とモードのモジュール起動

(着手 2026-06-12 / 前提: [pose-3d-calib-mode-separation.md](pose-3d-calib-mode-separation.md)
実装済み。実装ブランチ `pose-3d/flow-daemon`、M1–M4 実装済み)

## 背景 / 動機

専念モード化 (calib-mode-separation) で 3 モードは排他プロセス + YAML 受け渡しに
なったが、**次段の起動は SSH で手動**だった (auto-exit + `next_step` 案内まで)。
ユーザー要望は「main は daemon 的な存在にして、適宜別モジュールを起動するだけにし、
起動後のキャリブ → 実使用の流れを WebUI 上で潤滑にする」。

完了条件:

- `./main --daemon --config session.yaml` 一発で、ブラウザのタブ 1 枚から
  excal solve → subject approve → run 着地まで SSH なしで通る。
- run 中のビューワから再キャリブへ切り替えられる。
- モード排他の不変条件 (mode 越境は YAML のみ、各モードは必要物だけ構築) を壊さない。

## 検討した案

| 案 | 内容 | 判定 |
|---|---|---|
| 外部 supervisor スクリプト | bash が成果物 mtime を見て次段を起動 | 没。daemon を「main そのもの」にしたいというユーザー方針。bash に分岐ロジックが漏れ、web からの任意切替 (run→calib) の経路がない |
| main の self-exec (execv) | solve/approve 後に自分を次モード argv で exec | 没。「次モードの起動引数を知る」コードがモードプロセス自身に入り、専念モード化で消した越境知識が戻る。クラッシュ時の再起動も担えない |
| daemon が :8000 を常持ちし reverse-proxy | 接続断ゼロの最上 UX | 没。Crow に proxy がなく HTTP+WS の自前 proxy は工数と障害点が大きい。nginx 依存も増やしたくない。再起動の数秒は web の再接続表示で十分 |
| daemon に制御用の別ポート | モジュール死亡中も外から叩ける | 没。daemon にも HTTP 実装が入る + ページからクロスポート fetch (CORS)。切替要求はモジュール経由で足りる |
| **採用: portless daemon + exit code 契約** | daemon はソケットなし。モジュールが :8000 を持ち、切替は `POST /api/flow/switch` → モジュールが exit code で次モードを報告 → daemon が spawn | モード排他をそのまま保ち、daemon は fork/exec/waitpid だけの最小実装になる |

モジュール実体も分岐があった: **単一バイナリ self-spawn を採用** (daemon が
`./main --config X <モードフラグ>` を fork/exec)。モード別バイナリ分割
(fitra-run / fitra-calib-*) は配布物・docs・Docker の更新範囲が広く、利点
(プロセス名で見える / calib コードが run バイナリにリンクされない) が見合わない。

## 採用設計

```
ブラウザ ── :8000 ──▶ モジュール (./main --config X --flow-managed <モードフラグ>)
                          ▲ fork/exec + waitpid (exit code = 次モード)
                      daemon (./main --daemon --config X、ソケット/CUDA/TRT なし)
```

### exit code 契約 (`cpp/src/app/flow.hpp`)

| exit | 意味 | daemon の動作 |
|---|---|---|
| 0 | clean stop (Ctrl-C 含む) | daemon も終了 |
| 80 | 次は run | spawn |
| 81 | 次は calib-subject | spawn |
| 82 | 次は calib-extrinsic | spawn |
| その他 / シグナル死 | クラッシュ | backoff 2s → **run を spawn** (安全側のデフォルト)。正常 exit を挟まず 3 連続で give-up (`kMaxConsecutiveFailures`) |

`FlowControl` (stop 参照 + `managed` + `next_mode`) を main → runner → Crow ハンドラで
共有し、`request_switch()` が next_mode 記録 + stop。runner が EXIT_SUCCESS で戻ったとき
だけ main が flow exit code に変換する (異常終了に化けない)。

### managed フラグ

daemon は spawn argv に `--flow-managed` を付ける。モジュールはこれが立つときだけ:

- `POST /api/flow/switch {"mode": ...}` を登録 (非 managed は未登録 → GET 404 / POST 405)
- `GET /api/state` に `"managed":true`
- calib 完了の自動連鎖: excal `set_on_solved` → CalibSubject、subject
  `set_on_exit_requested` (approve + `--calib-auto-exit`) → Run
- `next_step` 文言が「Flow daemon switches to …」になる

非 managed (手動起動) は専念モード化時点の挙動そのまま (exit 0 + 再起動コマンド案内)。

### argv 合成 (`app/daemon.cpp` `module_argv()`、純関数)

全設定は **union YAML** (`--config`) に集約し、daemon はモードフラグだけ足す。
daemon の他の CLI override は転送しない (help / runbook に明記、起動時に警告):

- run: `--config X --flow-managed` (+ profile YAML 存在時のみ
  `--subject-id <calibration.calib_subject_id>`)
- calib-subject: `+ --calibrate --calib-auto-exit --no-vmt-out --no-slimevr-out`
- calib-extrinsic: `+ --extrinsic-calib --no-vmt-out --no-slimevr-out`

このために **負方向 CLI フラグ `--no-vmt-out` / `--no-slimevr-out` を新設**
(union YAML の `vmt.vmt_out: true` 等が calib spawn の publisher 排他 validate に
当たるのを argv で打ち消す。前例: `--no-3d-kalman`)。

**union YAML の運用規約** (コードでは強制しない):

- `subject.subject_id` を書かない — 書くと初回 calib-subject が存在しない profile を
  load しようとして落ちる。run への受け渡しは daemon が `calib_subject_id` から付与。
- `calibration.calibrate` / `extrinsic_calib.enabled` / `extrinsic_calib.replay_dir` を
  書かない — モードフラグは daemon の専権 (書くと `--daemon` validate が拒否)。
- `extrinsic_calib.out` と `three_d.calib` は同一パスにする — 違うと excal の成果物を
  次段が読まない (daemon が起動時に警告)。

### 初期モードとクラッシュポリシー

- `--daemon-initial {auto,run,calib-subject,calib-extrinsic}` (default auto)。
  auto = 成果物の欠けている最初の段: extrinsics YAML 不在 → calib-extrinsic、
  profile 不在 → calib-subject、両方あり → run。**初回セットアップが「daemon を起動
  するだけ」でキャリブから始まり run に着地する**。
- クラッシュ → run へ fallback (ユーザーはビューワから再切替できる)。run 自体が
  壊れているケースの無限ループは 3 連続 give-up で止める。`--daemon` の validate は
  union opts を run 形で検査するので、engine パス誤り等は spawn 前に落ちる。

### シグナル / ポート引き継ぎ

- daemon は **SIGINT と SIGTERM を同一ハンドラ**で受け、(1) stop フラグを立てる +
  (2) 現在の子へ SIGINT を転送する、の両方を必ず行う (`on_daemon_signal`)。
  どちらか片方だけだと止まらない:
  - stop だけ立てて子に転送しないと、daemon は `waitpid` でブロックし続け子も終わらない
    (端末 Ctrl-C はプロセスグループ全体に届くので子も SIGINT を受け「偶然」動くが、
    `kill -INT <daemon>` 単独や systemd 停止では止まらない)。
  - 子に転送するだけで stop を立てないと、子終了後に next_action が crash 扱いして
    run を spawn し続ける。
  共通ハンドラなので SIGTERM (systemd / docker stop) と SIGINT は同じ挙動になり、
  子が clean に終わろうが異常終了しようが `stop` で必ず 1 サイクルで rc 0 終了する。
  waitpid は EINTR リトライ。`test_flow_daemon` が ready ファイル同期で SIGTERM →
  rc 0 を決定的に固定。
  > 実装中の検証で当初 SIGTERM ハンドラが stop を立てず・SIGINT ハンドラが子へ
  > 転送しない実装だったのを上記に統一 (2026-06-12)。
- ポート引き継ぎはプロセス跨ぎでは fd がプロセス終了で閉じるので問題なし。
  **同一プロセス内では Crow の App が stop() 後も Server (listening fd) を保持する**
  ため再 bind 不可 — CrowServer オブジェクトの破棄が必要 (test_crow_excal で実証・
  コメント化済み。daemon 設計には影響しない)。

### web (`web/dual_rtmpose/flow.js`)

ルート静的 dir は全モードで配信されるので、calib ページも `/flow.js` で共有ロード。

- `FitraFlow.watch({page, redirect, onState, onDown})`: `/api/state` 1s ポーリング。
  接続断 = モジュール入れ替え中 (onDown 表示)。復帰時 mode がページと不一致なら
  該当ページへ遷移 — **calib ページのみ自動 redirect**。ビューワは redirect せず
  バナー (calib-subject 中もカメラ監視に使う用途が現にあるため)。
- ビューワ: managed + run のときだけ「↺ extrinsic / subject calib」切替ボタン
  (confirm 付き、`FitraFlow.requestSwitch`)。
- TRT 構築で再起動ギャップが数十秒になる段 (→ calib-subject / run) もポーリングが
  吸収する。

## Milestone (各 = 1 コミット)

- **M1** `feat(pose-3d)`: flow 基盤 — FlowControl / `/api/flow/switch` /
  `/api/state.managed` / 自動連鎖 / `--flow-managed` / 負方向フラグ /
  approve 応答 next_step。
- **M2** `feat(pose-3d)`: daemon 本体 — `app/daemon.{hpp,cpp}` (module_argv /
  next_action / initial_mode + fork/exec/waitpid ループ)、`--daemon` /
  `--daemon-initial`、main dispatch。
- **M3** `feat(pose-3d)`: web — flow.js + 3 ページの追従/導線。
- **M4** `docs(pose-3d)`: 本 doc + track changelog + runbook 改訂 +
  cpp-migration-plan 注記。

## 検証

- ctest: `test_flow_daemon` (純関数 3 種 + stub スクリプトでの run_daemon 実 spawn
  連鎖 / crash fallback / give-up)、`test_main_config` (フラグ・validate 排他・
  負方向フラグ)、`test_crow_excal` (managed/非 managed の /api/flow/switch 契約)。
  全 21 テスト pass。
- 実バイナリ smoke: `--daemon` で fork/exec → 子 validate 失敗 → crash fallback →
  backoff の経路を確認済み。
- **実機 (未・ユーザー実施)**: カメラ + SteamVR で
  `./main --daemon --config session.yaml` → ブラウザ 1 タブで 3 段通し、
  ビューワからの再キャリブ切替、モジュール kill → run 自動復帰。
  手順は [runbook-pose-3d-calibration.md](../runbook-pose-3d-calibration.md)。

## 実装で確定した判断

- daemon/flow 系フラグ (`--daemon` / `--daemon-initial` / `--flow-managed`) は
  **CLI 専権で YAML キーを作らない** — 起動形態は呼び出し側の責務で、union YAML を
  どのモードにも食わせられる性質を保つ。
- `next_action` のクラッシュ fallback 先は常に run (直前モードの再試行ではない)。
  「ユーザーが使える状態に戻す」を優先し、再試行はビューワの切替ボタンに委ねる。
- subject calib spawn に `--subject-id` を渡さない (再キャリブ時に旧 profile を
  load しない)。run spawn は profile 存在チェック付きで付与。
- daemon の crash backoff はテスト用に引数化 (`run_daemon(..., crash_backoff_ms)`)。

## 残課題

- **calib-extrinsic 起動途中 SIGINT の稀な SIGABRT** (別軸・低優先): モジュールが
  起動シーケンス途中 (Crow / nvjpeg 立ち上げ中) に SIGINT を受けると、稀に
  `signal 6` で異常終了することを 1 度観測 (3 回の追試では再現せず)。安定起動後の
  SIGINT は clean (solve 試行 → サンプル不足で exit 1)。daemon は子の終了コード/
  シグナルに関わらず stop で break するので**停止保証には無影響**。calib-extrinsic の
  shutdown レース (calib-mode-separation トラック) として要調査。
- 実機 3 段通し検証 (上記)。
- systemd unit / docker-compose に `--daemon` を主経路として載せる (vr-output 側の
  運用整備と合わせて)。
- ビューワの switch ボタンに「現在の calib 成果物の鮮度」(YAML mtime) を出すと
  再キャリブ判断がしやすい — backlog。
