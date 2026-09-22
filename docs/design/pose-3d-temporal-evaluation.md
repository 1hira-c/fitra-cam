# pose-3d: 後段補正の固定リプレイ評価器

更新日: 2026-09-22。状態: M1実装、M2の時刻付き3視点raw 3D再生と暫定擬似参照へ進行。
体感との合否校正は未完了。[収録確認・3視点adapter](pose-3d-corpus-qualification.md)を参照。
capture基準の3視点再生と、従来の時刻を失った2視点legacy pilotを区別する。
追補: 回転診断を強化（metrics revision 2、参照可用性と完全回転誤差の扱いはrevision 3）。[観察ケース台帳](../research/pose-3d-evaluation-cases.md)に
前屈時の腿twist不安定を、原因未確定の評価課題として記録した。本番処理は変更しない。
関連: [時系列予測計画](../research/pose-3d-temporal-prediction-plan.md)、
[Issue #65](https://github.com/1hira-c/fitra-cam/issues/65)。

## 背景・今回の実機アンカー

RTMPose-Mを続行し、DWPose-Mは将来の手トラッキング候補に留める。
2026-09-08のライブ比較でユーザーが確認した結果:

| 設定 | 体感（定量合格値ではない） |
|---|---|
| Kalman + One Euro | 滑らかだが追従が遅い |
| Kalmanのみ、代替EMAの位置/回転係数1 | 揺れが増えるが速い。遅延は一応許容 |
| Kalman OFF + One Euro OFF、代替EMA係数1 | さらに揺れるが速く、遅延は十分許容 |

3設定ともIK・床補正・信頼度ゲート・伸展補正を維持。「両方OFF」は無加工raw 3Dではない。
約150msという回想は実測値ではなく、capture→VMT送信統計にもfilter追従遅れや表示待ちは入らない。
目標はKalmanのみの許容できる追従を悪化させず、両方OFFに近い応答を目指しながら揺れを減らすこと。
数値しきい値と体感の対応はまだ校正していないので、現段階では自動採用しない。

## 構成と境界

- `cpp/tools/temporal_replay.cpp`: 共通の後処理前Halpe26 JSONLを再生。productionの
  `SkeletonKalman`、`IkSolver`、`FloorContactStabilizer`、`extract_trackers_with_floor_corrections`、
  位置/回転smoothingを直接呼ぶ。filterの数式をPythonで近似再実装しない。
- `scripts/temporal_eval/evaluate.py`: Python標準ライブラリで固定出力を採点。
  JSON詳細とMarkdown一覧、原本/出力/任意のbinary・profileのSHA-256を残す。
  C++出力の入力byte fingerprintも照合し、同じseq/時刻でも違う入力との取り違えを拒否する。
  fingerprintは事故検出用FNV-1aで、改ざん耐性の主張ではない。
- `synthetic.py`: 既知正解の人工Halpe26系列。静止、並進、腕振り、開始停止反転、人物消失と足欠測。
  モデル精度や実機合格を示すデータではない。
- `import_dump.py`: **全後処理OFF**の既存`dump_keypoints_3d` JSONLを明示的に変換。
  出自はoperator確認が必要。後処理済み`/ws3d`や`/stats3d`をrawと偽装しない。

リアルタイムコード、wire、既定設定、GPU context/stream、queue、サービス起動は変更しない。
`python/` fallbackへの機能追加なし。評価用の既存Python以外に新規依存はない。
上位CMakeは従来通りJetson依存を検出するが、リプレイ処理自体はCPUのみ。

### 比較候補

| 名前 | Kalman | One Euro | 固定EMA係数 |
|---|---|---|---|
| minimal | OFF | OFF | 位置1 / 回転1 |
| kalman | ON | OFF | 位置1 / 回転1 |
| current | ON | ON | 使用しない |

共通でIK・床補正・ankle足位置・chest/waist高さ・伸展補正は製品既定。
One Euroは`TrackerExtractorOptions`の位置1/4/1、回転1.5/1.5/1。
subject profileをCLIで指定でき、省略時はheaderの身長priorを使う。
身長priorと実機profileは同一ではないので実データ比較では同じprofileを指定する。
異なる人物/profileは別ファイル・別実行に分割する。

### 再生スケジュールの制約

**1個の3Dレポートにつき1回**の処理。`t_ns`差分を共通のdtに使う。
liveの60Hz paced extractorによる同一skeleton再利用、publish位相、capture、送信、
ネットワーク、SteamVR表示は再現しない。production関数の再利用はlive出力とのbit一致や
体感順位の再現保証ではない。M2で検証する。先頭/境界直後はnominal dtでseed。
gap > 0.5秒と非Fresh境界でKalman・床・抽出/FK・位置/回転filter履歴を破棄する。
固定profileのIKは再学習させない。Fresh等のwire判定自体は再実装せず入力の区切りを使う。

## 入力契約 `fitra_temporal_input_v1`

JSONL先頭はheader（1行）:

```json
{"schema":"fitra_temporal_input_v1","input_stage":"raw_triangulated","kp_format":"halpe26","coordinate_frame":"fitra_world_z_up_m","nominal_hz":60,"subject_height_m":1.7,"reference_kind":"none","time_basis":"measured_report","provenance":{"session_id":"example","model":"RTMPose-M"}}
```

以降は`seq`（source番号）、`t_ns`（厳密増加する評価時計）、`segment`（不透明なcontinuity区切り）、
`state`（`fresh` / `boundary`）、`phase`（`static` / `motion` / `unlabelled`）、
`joints`、任意の`reference_joints`。関節はHalpe順26要素で各要素が`[x_m,y_m,z_m,score]`または`null`。
欠損をゼロ座標にしない。
`score`は有限の非負値で、確率ではない。RTMPoseのSimCC最大値およびそのtriangulation平均には
1超があり、productionと同様にそのまま渡す。0〜1へ丸めたり高スコアを欠測へ変換したりしない。

- `reference_kind`: `none` / `synthetic_known` / `pseudo`。
- `time_basis`: `measured_report` / `measured_capture_group` / `synthetic_nominal` / `nominal_from_legacy_frames`。
  nominalのmsは宣言周期への換算値で実測msではない。モデルの厳密dt入力を要求するものではない。
- `measured_report`は評価対象が受け取ったレポート時刻。capture時刻があれば別のprovenance/
  sidecarに保存し区別する。原本/model/calibration/profileの対応・hashもprovenanceに残す。
- `measured_capture_group`は3視点groupの最大kernel時刻。原点を引いた整数nsを使い、
  原点は`capture_origin_ns`にdecimal stringで保持。不等間隔・seq穴を維持し、live report時刻と呼ばない。
- 再送は取り込み側で除去。重複/逆行は失敗。seq欠落を残し、穴を埋めて入力にしない。
- 消失・人物切替・再取得・座標系変更は`boundary`または新`segment`。boundaryは全候補unavailable。
  時計リセット、人物/profile変更は別ファイル。
- 教師には大型モデル＋未来を使うオフライン補正を許すが、candidateの入力/状態へ渡さない。
  教師の信頼度で使用可否を判断した結果を`null`maskにする。M1はconfidence重みではなくmask。
  参照trackerはIK/床/時間filterなしの幾何抽出で、欠測を過去FKで埋めない。
- `phase`は映像/収録手順から付ける区間ラベル。候補出力が止まることを根拠に静止としない。
  unlabelledでは静止jitter/動作lagを採点しない。全フレーム人手正解は要求しない。
- 任意の`evaluate:false`は全候補共通の採点除外（履歴更新は実施）。人工系列は各scene先頭1秒を
  startup warmupとし、identityからの初期収束を定常jitterへ混ぜない。動作開始を自動的に除外しない。

## 暫定擬似参照（2026-09-22）

`scripts/temporal_eval/pseudo_reference.py`は、完了済みのM候補runと別pose engineの教師runを読む。
同一plan hash・calibration/profile・detector/inference binary・capture出自を要求し、
completed artifactの改変/不完全なrunを拒否する。教師候補は手元のRTMPose-L Halpe26 FP16。
同系統モデルのため誤り相関があり、Lが必ず正確とは主張しない。モデル選定をやり直すものでもない。

- 品質maskは教師側だけで決める。score≥0.3、inlier views≥2、再投影≤6px、ray angle≥10°。
  暫定の診断値であって校正済み合格値ではない。合成方向端点Halpe 0は参照にしない。
- 各関節の±50ms窓で実時刻を使った局所一次回帰を中心時刻へ評価する。
  前後各2点以上と有効な中心観測を必須にし、等速運動を保持する。
  これは未来を使うoffline教師であり、runtimeに50ms待ちを入れる仕様ではない。
- seq穴・42ms超の隣接gap・品質不足・曖昧人物検出を跨がない。
  不足はnull。エッジの外挿・未観測中心の補間・教師の過去状態による穴埋めはしない。
- 信頼できる参照が減った場合はcoverageを記録する。採点を成立させるために閾値を緩めない。
  再投影は内部整合性であり、低くても実測正解ではない。急な動作の過平滑や教師固有biasが残り得る。
- 参照付与後の3候補を再生し、元runのcandidate出力/resetと完全一致を検証する
  （計測用processing_msは比較しない）。教師は候補状態へ入れない。

```bash
python3 scripts/temporal_eval/pseudo_reference.py M_RUN L_RUN NEW_REFERENCE_RUN --labels LABELS_JSON
```

`reference-quality.json`に品質通過数と未来窓を確保できた数を関節別に保存する。
同時刻誤差は「擬似参照との一致度」であり、精度改善の証明にはしない。
静止ラベルは原画像を根拠に付け、開始・停止・前屈などの動作評価と実機体感校正は別途必要。

## 採点と不採用案

同時刻の3D距離RMSE/p95、quat符号二重被覆を考慮した測地角p95。
静止位置jitterは区間重心からの距離RMS、静止回転は符号を揃えた平均quatからの角度RMS。
動作振幅比は同じ有効ペア上の位置spread比（参照の動き1mm以下なら未評価）。
欠測はreference coverageとoutput availabilityを別記し、誤差0へ置換しない。
revision 3では`reference_availability`（参照数/採点対象部位×report）と
`output_availability`をsummaryに追加。従来の`reference_coverage`は対応ペア/参照数であり、
参照自体が十分にあるという意味ではない。Markdownでも参照有効率と対応率を分ける。
完全quaternionの角誤差は参照roll confidence>0のペアだけを使い、supportを明記する。
方向だけが既知の参照はswing診断のみ有効で、任意rollを正解として採点しない。
境界・gap・phase遷移を越えて違う静止姿勢を混ぜない。

位置lagは±15レポートの中心化3D位置曲線相関。正が遅れ、負が先行。
全シフト共通supportを使い、参照欠測と2.5レポート超の穴を越えて補間しない。
最低12点、低相関/凍結出力/等速や周期の曖昧一致は未評価。探索端はcensoredとして一覧の中央値から
除きJSONに残す。サブレポート精度は主張しない。主誤差は**時間シフトせず**同時刻で計算する。

方向のlagはtrackerのlocal +Zをworldへ回した単位ベクトルの相関で、位置lagと独立に測る。
これはswingの曲線診断であり、twistの遅れはまだ測らない。開始/停止/反転のイベント遅れ、
接地ラベルを使う足滑り、位置jump/overshoot・復帰時間の独立指標はM2。
人工start_stop/dropoutはあるが専用合否は未実装。
`processing_ms`はCPU候補呼出しwall timeのみ（I/O/queue/GPU/送信除外）。逐次実行で他プロセスの
負荷もあるため、live追加latencyの代替ではない。

### 回転診断（metrics revision 2）

- 同時刻の`conjugate(reference) * output`を参照local +Zまわりのswing/twistに分解する。
  swingは骨軸のなす角、twistは符号付き最短残差（±180°）で、Euler角ではない。
  p95はtwist絶対値。qと−qは同じ姿勢として扱う。
- forwardがほぼ正反対（`hypot(delta.w, delta.z) < 1e-6`）ならtwistは非一意なので未定義。
  また参照の`roll_confidence`が0/未収録なら参照不明。両者を別countにし、誤差0で埋めない。
  C++ toolはreference/candidateのroll/swing confidenceを出力するが、live wireは変えない。
  confidence>0も絶対正解の保証ではない。教師生成時のmaskと出自が別途必要。
- 連続seq・有効quat・gap≤2.5 nominal reportの隣接ペアだけからtwist step/rateを計算する。
  欠測や復帰の穴を跨がず、supportを併記。±180°wrapを358°ジャンプと誤認しない。
- `--twist-jump-deg`（既定45°）以上のstepを診断countにする。これは合否閾値でなく、
  実動作も数える。参照があるときは`max(0, abs(output_step)-abs(reference_step))`も出す。
- 同じ有効隣接ペア上のtwist総移動量比（絶対step和）を出す。参照総量が5°未満なら未評価。
  出力固定の0と、ノイズ増幅の大きな比を区別する補助指標。振幅そのものではなく、単独の合否にしない。
- 部位・区間別JSONに加えてcase別の最悪部位twist誤差p95、急変count/support、未評価数を表にする。
  全身中央値だけで局所破綻を隠さない。参照なしでもstep/rateは出せるが精度の良否は断定しない。
- 採点器自身も入力の時刻/seq型、境界reset宣言、参照の有無を検証する。
  candidate出力が境界を隠した場合に、前後の系列を黙って連結しない。

人工系列に`forward_bend_fixed_feet`を追加。上体は前屈し観測足点は固定するが、
これは実機症状の再現保証ではない。別の採点テストで腿だけの既知twist振動を注入して検出力を確認する。

採らない案:

- 滑らかさだけの単一スコア: 停止しっぱなし・微小動作抹消を優遇する。
- 擬似教師一致だけの合否: 教師の癖や過平滑を正解として固定する。
- nominal時刻を実測同期とする: 既存動画のskewを復元できない。
- live APIへ評価用raw全身を即追加: 契約を広げるためM1は既存offline dumpが入口。
- 校正前の数値合格値: レポートは常に`not_calibrated_against_human_AB`、自動優勝なし。

## 実行

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build --target fitra_temporal_replay -j2
ctest --test-dir cpp/build -N
ctest --test-dir cpp/build -R temporal_eval --output-on-failure

# 既存ファイルは上書きしない。毎回新しいpathを選ぶ。
python3 scripts/temporal_eval/synthetic.py /tmp/eval-input.jsonl
cpp/build/tools/temporal_replay /tmp/eval-input.jsonl /tmp/eval-output.jsonl
python3 scripts/temporal_eval/evaluate.py /tmp/eval-input.jsonl /tmp/eval-output.jsonl /tmp/eval-report.json \
  --binary cpp/build/tools/temporal_replay
```

既存動画は`dump_keypoints_3d`に`--no-kalman --no-ik --no-floor-contact-stability`を渡しraw dumpを作る。
元toolのカメラ数・校正条件は維持し、filenameだけで同期済みとしない。

```bash
python3 scripts/temporal_eval/import_dump.py RAW.jsonl INPUT.jsonl \
  --confirm-unfiltered --nominal-hz 60 --subject-height-m 1.7 --segment session_clip --phase static
cpp/build/tools/temporal_replay INPUT.jsonl OUTPUT.jsonl calibrations/subjects/subject01/latest_profile.yaml
python3 scripts/temporal_eval/evaluate.py INPUT.jsonl OUTPUT.jsonl REPORT.json \
  --binary cpp/build/tools/temporal_replay --subject-profile calibrations/subjects/subject01/latest_profile.yaml
```

参照なし実録画では追従誤差/lagを空欄にし、静止区間の揺れ等だけを出す。
C++失敗時の部分outputは採点せず新規pathで再実行する（自動削除/上書きしない）。

### Legacy録画の再現可能なpilot（2026-09-19）

`scripts/temporal_eval/run_recording.py`は`record_3cam`形式の`meta.json`と元映像から
**cam0/1だけ**を既存dump toolで処理し、全後段OFFのraw→3候補replay→reportを一括生成する。
既存toolは2視点専用であり、3カメラliveの再現と呼ばない。cam2は使用しない。
ファイル名cam0/1をmetadata内で照合し、配列の並びからcameraを推測しない。

```bash
python3 scripts/temporal_eval/run_recording.py RECORDING_DIR NEW_OUTPUT_DIR \
  --dump-binary /absolute/path/to/dump_keypoints_3d \
  --replay-binary cpp/build/tools/temporal_replay \
  --calib /absolute/path/to/frozen-extrinsics.yaml \
  --det-engine /absolute/path/to/yolox_s.fp16.engine \
  --pose-engine /absolute/path/to/rtmpose_m_halpe26.fp16.engine \
  --subject-profile /absolute/path/to/subject-profile.yaml --subject-height-m 1.7
```

- outputは新規directoryのみ。開始前に原動画・metadata・calibration・profile・engine・binary・
  解析scriptsのSHA-256と実行argvを`manifest.json`へ保存する。終了時にも入力hashを検査。
  `complete.json`がない実行は未完了。失敗時のraw/log/manifestは残す。
- 期待したframe数/連番が出なければ採点へ進まず、decode途中終了を成功扱いしない。
- 時刻はcam0のmetadata平均FPSに基づく`nominal_from_legacy_frames`。
  同じframe番号をペアにするだけであり、実測のcapture間隔/skew/同期やlive latencyは不明。
  `frames.csv`がある録画はこのadapterで処理せず、時刻を利用できるadapterへ回す。
- calibration/profileの対応はoperator選択。hashは選択の再現性を保証するだけで、収録時の
  正しい校正だったことを証明しない。`latest`でも参照先の絶対pathとhashを残す。
- phaseは既定`unlabelled`。ファイル名から静止を推定しない。
  `--labels FILE`は`fitra_temporal_labels_v1`、`evidence`、`intervals`を持つJSON。
  intervalは`start_frame`以上`end_frame`未満、`phase`と任意のboolean `evaluate`。
  順序逆転/重複/範囲外を拒否し、未ラベル部分はunlabelled。ラベル変更でfilter履歴はresetしない。
- 試験用の静止ラベルは映像1Hzサンプルの確認に基づく暫定値。真の身体動揺を含む出力RMSであり、
  厳密な無動作区間・純粋な推定ノイズ・GT一致とはしない。

実データでscore>1の拒否を発見し、importとC++ readerをproductionの非負score契約へ合わせた。
前屈pilot初回は1186フレームの生成後、この検証で停止。失敗artifactは保存し、新規v2で再実行した。
ライブコード・既定設定・wireは変更しない。
追加収録の依頼手順は[短時間収録プロトコル](../research/pose-3d-temporal-capture-protocol.md)。

## Milestoneと検証

1. **M1（今回）**: 同一入力3設定、時刻契約、人工系列、raw dump adapter、レポート、回帰テスト。
2. **M2**: 固定実録画をraw 3Dへ変換、sessionで開発/検証を分割。大型モデルの擬似教師と区間ラベル、
   イベント遅れ/回転/足滑り指標を追加。今回の体感との順位・トレードオフを校正し、矛盾すれば評価器も修正。
3. **M3**: 軽い補正＋予測／学習候補を同じ入力と出力schemaで評価。将来位置はtarget時刻/horizonを
   明示した別指標とし、現在値へ混ぜない。採用候補のみHMDで最終確認、Jetson追加コストも測る。

回帰: 150ms既知遅延、2レポート先行、凍結、振幅半減、欠測、quat符号反転、重複/逆行/NaN/wrong-stage拒否、
legacyフレーム穴、phase区切り、determinism、prefix causality、teacher差し替えでcandidate不変、境界/gap reset。
人工系列やCTest成功を実録画品質・体感適合の合格証拠にしない。
人工骨格の骨長と身長priorは厳密には一致しないため、同時刻位置誤差には共通IKによる形状差も含む。
人工データの絶対誤差で実機profileや補正方式の品質を選定しない。

### M1検証結果（2026-09-08）

- Release構成と`fitra_temporal_replay`、既存のKalman/位置・回転trackerテストtargetをビルド。
- focused CTest 4/4（`tracker_extract` / `tracker_extract_pos` / `kalman_chain` / `temporal_eval`）pass。
  `temporal_eval`内の標準ライブラリunittestは15/15、skipなし。
- 5 scene × 360 = 1,800 reportsを全3候補で再生し、JSON/Markdownを生成。
  artifact: `outputs/temporal-eval/20260908-m1-final/{input,replay,report}.json*` と `report.md`。
- runtimeソース変更なしのoffline tool追加のため、全target build/全CTest/新binaryでの実機受入れは
  今回実施せず、呼び出す既存filterと新評価器に検証を絞った。実録画/擬似教師の評価は未実施。

### 回転診断追補の検証結果（2026-09-08）

- offline toolをRelease再ビルド。focused CTest 4/4、評価器内unittest 27/27（skipなし）pass。
- 位置150ms/方向100ms遅延、片腿のみ±80°twist振動、正しいtwistを消す固定出力、
  world軸とreference-local軸の区別、±180°wrap、qの符号反転、参照不明・分解未定義、欠測を検証。
- 6 scene × 360 = 2,160 reportsの人工系列を全3候補で再生し、
  `outputs/temporal-eval/20260908-rotation-v2/report.{json,md}`を生成。
- 実機のBEND-TWIST-001を再現・解決したという判定はしていない。runtimeコード/設定は変更せず、
  実録画の擬似教師と体感校正、イベント指標は引き続き未完了。

### 実録画pilotの検証結果（2026-09-19）

- Release `fitra_temporal_replay` build、focused CTest 4/4、unittest 34/34（skipなし）pass。
  C++ reader/importで1超scoreを保持し、負値/非有限値拒否を確認。
- `outputs/records/still`全1185、`forward_bend`全1186フレームをGPUで再推論し3候補で比較。
  原本/成果物の完了hashを照合。出力は通常workspaceの
  `outputs/temporal-eval/20260919-real-{still,forward-bend}-v2/`。
  入口レポート: `outputs/temporal-eval/20260919-real-pilot-summary.md`。
- 静止の暫定ラベル範囲で位置RMSの部位中央値はminimal 6.602mm / kalman 5.401mm /
  current 4.807mm。身体動揺を含む。遅延・参照誤差は未評価なので優勝を決めない。
- 前屈録画では左腿の45°以上twist stepが8/836・7/1153・5/1153、左足はいずれも0。
  分母は有効隣接step。左側出力利用可能数はminimal 877/1186、Kalmanあり1155/1186と異なる。
  欠測/補完・真の動作・startupが混在し、症状再現/原因特定や精度順位の証拠ではない。
- 全target/full CTestはoffline変更に範囲を絞るため未実施。新規実機収録・HMD受入れ・
  3視点時刻付き再生・教師生成/体感校正も未実施。本番処理や稼働設定は変更していない。
