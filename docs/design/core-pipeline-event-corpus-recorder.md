# core-pipeline: 常時稼働向けイベント・コーパス収録

(着手日 2026-09-07 / 関連: `../research/pose-model-audition-plan.md`)

## 目的

姿勢推定モデルの比較と、その後段に置く時系列 AI の学習に再利用できる原本を、日常運用中に
自動収集する。推論は常時フル稼働を維持し、ストレージだけを有限資源として扱う。

MVP の完了条件は次のとおり。

- いずれかのカメラで人物姿勢が得られた区間を、全カメラの V4L2 原ペイロードで自動収録する。
- 検出前後を残し、フレームごとの sequence と monotonic timestamp を失わない。
- SSD spool が詰まっても capture / YOLOX / RTMPose を待たせない。
- 人物不在が一定時間続いた時だけ、マウント済み NAS 等へコピーする。
- コピー先のサイズと SHA-256 を照合して commit した後に限り、SSD 側を削除する。

## MVP の境界

採用するトリガは `any camera has inferred persons` の 1 種類だけとする。WebUI 手動トリガ、
低信頼度・遮蔽・再投影誤差などの難例トリガ、平常区間の reservoir sampling、S3 等の cloud SDK、
収録済み動画への pose 推論は後続 milestone とする。

転送先はローカルファイルシステムとして見えるディレクトリに限定する。NAS は mount、cloud は
rclone mount 等を運用側で提供できるが、本体には新しい実行時依存を追加しない。

## データ経路

```text
V4L2 DQBUF -> QBUF -> raw tap -> bounded queue -> recorder worker -> SSD open/ -> ready/
                  \
                   -> latest raw slot -> FrameSource decode/inference -> MultiCameraDriver
                                                                        |
                                                                        | presence tap
                                                                        v
                                                        event state / upload absence gate
                                                                        |
                                                         remote .partial/ -> ready/
```

raw tap は `V4l2Capture` が DQBUF payload をコピーし、kernel buffer を QBUF へ戻した直後に呼ぶ。
decode/inference 側の latest slot 消費とは独立なので、推論 worker が遅れた時も DQBUF 済み sequence を
取り込める。既存の校正用 frame tap は
デコード済み BGR、clone 必須、収録中に推論を止める契約なので流用しない。raw tap が行うのは、
上限予約、ペイロード copy、キューへの move だけであり、ファイル I/O は recorder worker に閉じる。
キューが満杯なら recorder 側だけがそのフレームを drop し、リアルタイムパイプラインは継続する。

**2026-10-27: 人物通知も有界化。** raw payload の byte 上限とは別に、人物有無の変化は最大256件まで
FIFOへ入れる。超過時は追加のedgeを積まず、固定サイズの最新全camera状態1件へ集約する。
writerは古いFIFOを一括で退避し、既存sessionを`presence_queue_overflow`で閉じ、pre-roll履歴を破棄する。
人物ありなら`start_reason: presence_queue_resync`の新sessionを開始し、人物なしなら待機へ戻す。
退避payloadの解放はwriter側・queue mutexの外で行い、解放までbyte予約を維持する。
境界以前にcaptureされたコピーが遅れてenqueueされても、新sessionへ再投入しない。
空payloadも、byte上限に計上されない要素を増やさないようqueue dropとして拒否する。

通知の単純dropでは、upload用の最新presenceとwriterの収録状態が食い違い、入場を逃す／退出しても
収録を続ける問題が残る。通常時から最新状態だけへ畳む案も短い人物出現を失うため採らず、通常はFIFO、
過負荷時だけ明示的なsession境界を作って再同期する。通知欠落は`presence_dropped_events`と
manifestの`recorder_drops.presence`、適用した再同期は`presence_resyncs`で確認できる。
復旧時に退避したrawも`queue_dropped_frames`へ加算し、収録欠落を成功扱いにしない。

presence tap は RTMPose 結果を `CameraSnapshot` に割り当てた直後に、カメラ別の人物有無を通知する。
1 台でも人物ありなら 1 つの session を開き、全台分を収録する。全台不在になっても post-roll の間は
同じ session を維持する。短い再出現は session を分断しない。writer の I/O が遅れても既に queue に
ある post-roll frame を落とさないよう、区間判定には writer の処理時刻でなく元 frame の capture 時刻を使う。

corpus 収録を有効にした Run mode では idle/standby を強制無効化する。原本収集時だけ推論を止める
挙動は、人物進入のトリガそのものを失うためである。calibration mode では corpus を起動しない。
合成人物を常時生成する `--bench-fake-bbox` との併用は、無限イベントになるため起動時に拒否する。

## SSD spool とフォーマット

spool は `open/` と `ready/` に分ける。session は `open/<session-id>` で作り、close 時に manifest を
確定して同一ファイルシステム上で `ready/<session-id>` へ rename する。クラッシュで残った `open/`
は次回起動時に `recovered-incomplete` として隔離し、削除せず転送対象にする。

session は短い immutable segment に分割する。各 segment は次を持つ。

- `cam<N>.mjpeg` / `cam<N>.yuyv`: そのカメラの原ペイロードを連結したファイル。MJPEG / NVJPEG
  設定なら ffmpeg 等でも読める JPEG elementary stream、YUYV 設定なら packed YUYV のまま保存する。
- `frames.jsonl`: `camera`, process `seq`, kernel `v4l2_sequence`, `offset`, `size`,
  `captured_mono_ns`, kernel V4L2 timestamp と
  SOE/EOF 意味論をフレーム単位で記録する。64 bit nanoseconds は JSON number の丸めを避けるため
  decimal string とする。
- `checksums.sha256`: payload と sidecar の SHA-256。

session root の `manifest.json` は schema `fitra_event_corpus_v1`、trigger、pre/post/segment 秒、
カメラ device、V4L2 が実際に受理した capture 解像度・stride・sizeimage・fourcc・frame interval、
要求 fps、推論側の output 解像度、開始/終了理由、フレーム数を持つ。MP4 の平均 fps には畳み込まず、
missing frame と実時刻を後段が復元できることを優先する。

`spool_max_gib` は既存 spool と新規 payload の合計に対する hard stop とする。上限到達後は未転送
データを勝手に消さず、収録フレームだけを drop して counter と警告を出す。metadata の小さな固定
オーバーヘッドは上限外となる。YUYV は原本サイズが大きいため、常時収録では MJPEG/NVJPEG を推奨する。

## remote commit protocol

uploader は recorder writer と別スレッドにし、次の条件をすべて満たす時だけ 1 session ずつ処理する。

1. 全カメラで人物不在。
2. 不在が `upload_absent_s` 継続。
3. active session が close 済み。

起動直後の初期値を「無人」の確定値としては扱わない。全 camera の推論結果を少なくとも一度観測するか、
`presence_stale_s` の起動猶予を過ぎ、かつ通常の `upload_absent_s` も経過するまでは既存 session を送らない。

転送は `<remote>/.partial/<session-id>` へ行う。各ファイルを一時名で copy し、source / destination の
size と SHA-256 を照合してから確定名へ rename する。人物が戻った場合は chunk 境界で中断し、
`.partial` を残して次回再試行する。tree 全体を再照合した後、remote `ready/` へ rename する。
SSD 側の session を削除できるのはこの commit 後だけである。失敗・mount 切断・容量不足では SSD 側を
残し、推論処理へ例外を伝播させない。

NAS が外れた時にローカルの空 mount point へ誤書き込みしないよう、remote root に
`.fitra-corpus-target` が存在する場合だけ uploader を動かす。この marker は NAS を mount した状態で
一度作成する。`remove_after_upload: false` の場合は削除の代わりに SSD の `uploaded/` へ移し、同じ
session を繰り返し転送しない。

## 設定

```yaml
corpus:
  enabled: true
  spool_dir: outputs/pose-corpus
  remote_dir: /mnt/nas/fitra-pose-corpus  # 空なら転送しない
  pre_roll_s: 5
  post_roll_s: 10
  segment_s: 5
  upload_absent_s: 30
  presence_stale_s: 2
  queue_max_mib: 128
  ring_max_mib: 512
  spool_max_gib: 256
  min_free_gib: 10
  remove_after_upload: true
```

既定は `enabled: false` で既存運用に影響しない。CLI は日常運用でよく変更する enabled / path / 時間 /
spool 上限だけを公開し、queue/ring memory 上限は YAML で設定する。

## 不変条件

- capture / inference の latest-frame-wins 契約を変更しない。corpus の欠落は sidecar の sequence gap と
  drop counter で観測し、パイプラインへ backpressure を返さない。
- 原本、将来生成する pose、ラベルは別レイヤにする。この MVP は原本と capture provenance だけを書く。
- V4L2 kernel timestamp が unavailable の時に DQBUF 時刻から捏造しない。
- 人物ありを最後に通知したカメラが切断しても session を無限継続しない。
  `presence_stale_s` 無更新でそのカメラの presence を失効させ、post-roll へ進む。
- `.partial` または checksum 未照合の remote session を commit 済みとして扱わない。
- 未転送ローカル session を容量確保のため自動削除しない。
- 人物通知のoverflow後に旧FIFOや古いin-flight captureで収録状態を再seedしない。

## Milestone

- **M1 (本 MVP)**: raw/presence tap、pre/post-roll、segment + sidecar、SSD 上限、mounted-directory uploader、
  config/CLI、合成フレームによる回帰テスト。
- **M2**: WebUI 状態・手動 keep、難例トリガ、平常 reservoir、収録品質 dashboard。
- **M3**: pose-model audition の派生データ生成。原本 session id + frame key を保持する。
- **M4**: 採用モデルの時系列出力と教師ラベルを追加し、補正・補完 AI の学習 corpus にする。

## 検証

- 単体: pre-roll / post-roll、複数カメラ session 統合、segment rotate、raw offset、64 bit timestamp、
  queue/spool drop、SHA-256、remote commit 前後のローカル保持を一時ディレクトリで確認する。
- config: YAML/CLI/emit-load、範囲、spool と remote の入れ子拒否を確認する。
- 過負荷: productionと共通のqueueで10万通知の上限、最新全camera状態への集約、退避byteの計数、
  capture watermark・restartを固定する。実writerへの通知burst後に新規captureを保存し、退出で閉じること、
  空payloadがbyte上限を迂回しないことも回帰対象にする。
- build/CTest: Release build、focused corpus/config test、全 CTest、`main --help`。
- 実機: 3 camera full inference の fps/latency、人物の進入・退出と pre/post-roll、SSD 実効 bytes/s、
  NAS 切断・再接続、転送中の再進入による pause、spool full 時も pose が継続することを別途確認する。

2026-10-27の統合前確認: Release configure／全target build、`main --help`、全CTest 38/38を通過。
recorder内は既存8ケース＋過負荷・空payload・再同期復帰の3ケースを実行した。Crowはsandboxの
loopback制限を切り分け、修正後の全CTestはhost contextで実行した。合成の10万通知で未修正時に
raw byte上限とは独立したRSS増加を確認し、修正後は同じ通知数でその増加傾向を再現しなかった。
これは合成過負荷の確認であり、3camera収録ONの負荷・NAS障害復帰・実機acceptanceは未検証。
設定・WebUI・推論数値・公開REST/WSは変更していないためpnpmやモデル数値照合は再実行していない。
