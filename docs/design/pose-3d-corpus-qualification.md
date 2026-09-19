# pose-3d: 時刻付き原本の収録確認と3視点グループ化

更新: 2026-09-19。収録確認・captureグループ生成まで実装。
**この原本からの3D再推論と後段評価への接続はまだ未実装**。
関連: [後段評価器](pose-3d-temporal-evaluation.md)、
[収録手順](../research/pose-3d-temporal-capture-protocol.md)。

## 背景と境界

legacy AVIではフレームごとの実測時刻を失っており、同frame番号ペアを同期と呼べない。
通常workspaceの`core-pipeline/event-corpus-recorder` (`c7bb5a9`)に既存のraw recorderがあるため、
リアルタイム処理・wireを変更せず、これを実機で確認して評価用の入口を作る。
解析コードは`pose-3d/temporal-evaluation`に積み、両branchはまだ統合しない。

原本の契約は既存`fitra_event_corpus_v1`。
payload、frame offset/size、camera index/config slot、process/kernel sequence、host DQBUF時刻、
kernel時刻とSOE/EOF意味論、accepted capture formatを保持する。推論消費フレームとは別である。

## 構造

- `scripts/temporal_eval/capture_local.py`: 既存main/configを90秒（10〜300秒で指定可）だけ起動。
  CLIでcorpusを有効化し、remoteを空に、VMT出力/発見をOFF。Webも起動しない。
  config自体は書き換えず、HMD receiverなど他の設定は元configに従う。
  spool上限4GiB、空き10GiB保持。モデル・校正・profileは`--asset`で明示してhashを記録する。
  検証時に長時間起動しっぱなしにしないwrapperであり、製品のイベント駆動常駐方針は変更しない。
- `capture.json`にargv・binary/config/明示assetのSHA-256を保存。
  時間終了時にSIGINT→30秒以内の正常終了を確認し、入力hashを再照合して`stopped.json`を書く。
  強制終了/異常終了ではstoppedを生成しない。正常停止しても人物検出なしならsessionは0。
  `stopped.json`は稼働停止記録で、収録品質の合格マーカーではない。
- `scripts/temporal_eval/inspect_corpus.py`: 閉じた`ready` sessionだけを読み取り検査。
  `.partial`・recovered incomplete、symlink、不正なファイル名、checksum未掲載ファイル、
  digest不一致、frame/segment/session count不一致、payload範囲の重なり・穴・余剰bytesを拒否する。
  session/segment index順序、非負整数型/64bit範囲、sequence・時刻の単調性も検証する。
  32bit kernel sequenceの正常wrapは保持し、逆行/重複とは区別する。
  読み取り中のsource変更も終了時hash照合で検出する。
- `report.json`はcamera別frame数・連番穴・時刻欠損・周期分布・採用率、3視点skew分布、
  recorder dropと原本hashを保存する。`groups.jsonl`はpayload path/offsetと原sequenceを保つ索引。
  64bit時刻・sequence・offsetは出力でdecimal stringにする。

## 時刻グループ化の不変条件

- host DQBUF時刻でkernel時刻の欠損を補わない。
- kernel SOE/EOFの意味が混在すれば組み合わせない。全欠損ならgroup=0/time_basis=unavailable。
- 同じ意味のkernel時刻だけを使い、各camera先頭フレームのmax-minがwindow以内なら採用。
  超えた場合は最古の先頭だけを除外するgreedy方式。各フレームは最大1回使う。
- windowは明示値（既定15ms）。この値以内なら幾何誤差が許容できる、という合格値ではない。
- **全raw録画からのoffline groupingであって、productionの推論後bounded queueを再現しない**。
  採用率は収録frame数に対する割合。除外数とtimestamp missingも別に出す。
- groupの`capture_max_ns`は選択画像の最大capture時刻で、ライブ3D report受信時刻ではない。
  次の3D adapterでも`measured_report`へ偽装せずcapture基準の別契約を用いる。
- kernel SOEは露光中央の実測値でも、USBカメラのハードウェア同期の証明でもない。
- 原画像に人物切替があれば後段で明示境界が必要。capture sessionの区切りだけで人物IDを保証しない。

不採用: 同frame番号による同期、missing kernel時刻のhost時刻への暗黙置換、校正/モデルを
manifestに固定しない収録、本番filter/wireへの評価専用データ追加。

## 実行

```bash
python3 scripts/temporal_eval/capture_local.py /absolute/path/to/main CONFIG_YAML NEW_RUN_DIR \
  --seconds 90 --asset DET_ENGINE --asset POSE_ENGINE --asset EXTRINSICS \
  --asset INTRINSICS --asset SUBJECT_PROFILE
python3 scripts/temporal_eval/inspect_corpus.py NEW_RUN_DIR/spool/ready/SESSION NEW_REPORT_DIR \
  --window-ms 15
```

既存録画や失敗artifactは上書き・削除しない。captureの開始前にカメラ占有と保存先容量を確認し、
ユーザーへ録画開始を知らせる。取得画像の外部送信は行わない。

## 2026-09-19 実機確認

ユーザー準備確認後、HMDなしで「10秒程度立位→画角外へ退出」を依頼。
90秒上限で起動、人物検出によりeventが開き、退出後`post_roll_complete`で閉じた。
capture processもSIGINTでexit 0。NAS uploaded=0、VMT出力OFF、既存config/hash不変。

- 保存: `outputs/temporal-eval/20260919-capture-smoke-01/`（通常workspace、約1.1GiB）。
- session: `20260919T062221948Z-0`、9 segment、合計7688 frames。
- raw MJPEGの索引から各camera中央付近1枚をdecodeし、1280×960と立位全身の写りを目視確認。
  これは全フレームdecode、校正精度、すべての姿勢での見切れの合格証拠ではない。
- 全frameの時刻意味論は`monotonic_soe`。時刻欠損0、process/kernel sequence穴0。
  recorder queue/storage dropは各0。capture開始前/終了後の未収録をゼロと主張しない。

| camera | frames | kernel間隔p50 / p95 / max ms | 15ms window採用率 |
|---|---:|---:|---:|
| cam0 | 2590 | 16.002 / 20.016 / 100.037 | 96.216% |
| cam1 | 2582 | 16.000 / 20.032 / 100.007 | 96.514% |
| cam2 | 2516 | 16.003 / 19.996 / 20.093 | 99.046% |

2492組。skew p50=8.856ms、p95=12.1905ms、max=12.83ms。
sequence穴がなくても間隔は一定ではなく、cam0/1の最大約100ms間隔を記録する。原因は未確定。
再投影誤差/幾何精度、リアルタイム採用率、HMD追従はこの数字から判定しない。

## 検証と残作業

回帰: checksum破損/不足、範囲外/重複offset、segment/manifest count、未完了、時刻逆行、
32bit wrap、64bit保持、欠測時刻非補完、SOE/EOF混在拒否、15ms窓/非再利用、remote/VMT無効argv。
関連CTestと既存recorder/configテストを実行する。新規Python依存なし、runtime C++変更なし。

今回の検証結果: 評価器branch focused CTest 4/4、内部unittest 42/42（skipなし）pass。
通常workspaceの既存`test_event_corpus_recorder` / `test_main_config`も2/2 pass。
Python offline工具だけの追加のため全target再build/全CTestは行わず、今回取得した実機証拠を
上記の収録・停止・timestamp/payload検査に限定する。NAS転送、容量枯渇、全姿勢やVR出力の
受入れ確認は今回の対象外。

残作業は、group索引からの**3視点raw 3D再推論adapter**、capture基準の評価時刻契約、
校正対応/再投影確認、区間ラベル、大型モデル＋未来参照教師、実機体感との対応付け。
raw映像を確保できたため、これらの実装中にユーザーへ同じ動作の再収録を繰り返し求めずに済む。
