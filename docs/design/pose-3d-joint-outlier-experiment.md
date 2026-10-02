# pose-3d: 関節の外れ値棄却と短期補完の実験

2026-10-02。RTMPose-Mのまま、距離filterで残る「跳ねて戻る」をアルゴリズムで抑える。
前段は[距離filterの体感比較](pose-3d-vmt-live-trial.md)。試験用CLIと評価器に実装する。

## 背景と採用位置

2026-09-23の体感ではdistance5が改善候補だったが、姿勢依存の大きな揺れが残った。
追加の約8秒の2D記録ではcam0の膝が高scoreのまま約112–115px変化し、左右交換で
対応させると変化が数pxになる6イベントを確認した。他2台の膝は同区間で比較的安定。
右big toeもcam0だけscore中央値0.406、0.5未満91.4%。RGBと確定した姿勢名は未取得で、
左右取り違え・つま先立ちへの弱さをGTで確定した結果ではない。

既存の位置smoothingには8–16m/sの速度gateがあるが、回転を作った後のtracker位置に作用する。
距離filterは大きい入力変位ほど通しやすく、外れ値の除去はしない。
今回のgateはraw3DとKalman/IK/骨長補正/床/回転抽出の間に置く。回転抽出の数式は変更しない。

raw観測には視点別の誤りを確定する情報が不足しているため、初版は関節ごとの時間整合性を判断する。
共通Triangulator、カメラの推論、raw PoseGate/FusionPose観測契約へ採否policyを追加しない。

## 構造と不変条件

`cpp/tools/temporal_outlier_gate.hpp`をshared Candidateから呼ぶ。
offlineは`--outlier-experiments`で`guarded_distance_5mm`を追加、live trialはstdinの`guard5`で選択する。
既存distance5のsoft_bones_head_leaf・床・抽出・位置filterを共通にする。

- 連続する2つの**採用raw関節**だけから速度を作り、予測速度を3m/sで制限する。
  棄却/欠測を跨いだ復帰では速度を0へ戻し、次の採用rawで再seedする。
  観測穴の前後を結んだ変位を、そのまま次の補完速度には使わない。
- 現観測が因果予測または最後の採用位置から`0.08m + 0.5 * 30m/s² * min(age, 0.05s)²`以内なら即採用。
  現在位置を通し、正常観測を確認待ちにしない。scoreだけで採否を決めない。
  最近のノイズから大きな速度を作った場合でも、小さな実観測の切り返しを棄却しない。
- 棄却または欠測時は最後の採用時刻から最大50msだけ有界等速予測。速度ゼロなら保持に相当する。
  期限は棄却・補完・再送で延長しない。補完関節のscoreは0、由来はsidecarで明示する。
- 50ms超はinvalid。履歴は自動seedへ戻さず、元軌道への回復または独立pending列による再取得を待つ。
- pending列は連続raw観測が3report、隣接変位が`0.08m + 3m/s * dt + 0.5 * 30m/s² * dt²`
  以内なら現在観測で再取得する。古い速度は捨てる。pending間隔50ms超・欠測で確認数をリセットする。
- 持続する誤推定も再取得条件を満たし得る。このgateは人体正解・左右対応の証明ではない。
  8cm等の閾値は初期実験値で、個人/姿勢全般に校正済みの値ではない。
- dtはsource report/captureの実差分を使う。未来観測を待たず、最新reportを1回だけ処理する。
  gap/候補切替/人物・座標・continuity変更/非Fresh境界で履歴を破棄する。

### 補完の期限を後段へ伝える

`Joint3D.valid`だけでは採用観測と合成値を区別できない。新候補の診断へ`post_gate`と26個の
`joint_gate`を追加する。stateはaccepted/predicted/unavailable、棄却・再取得flagと採用観測からのageを残す。
raw入力ファイルは不変。旧10候補のdiagnosticsは従来の3キーを維持する。

FloorContactには採用関節だけのコピーを渡して接地evidenceを更新し、報告された補正を表示骨格へ適用する。
補完足から新しい接地観測を作らない。膝/足首/big toeに合成値がある間はFootAnchorを再学習させない。
該当点がunavailableになると足anchorを破棄してfoot出力をinvalidにし、既存FKの期限なし復活を防ぐ。
伸展hysteresisも合成chainから更新しない。正常側の関節・trackerは独立して継続する。
伸展regimeは維持するが、連続確認の前回角度/countは切り、観測穴を跨いで確認を成立させない。
FloorContact自身が棄却した採用soleのvalidity/scoreも出力へ反映し、足向きやanchorへ流さない。

liveはprivate表示/TrackerBusのみに適用し、raw busやD50 wireを上書きしない。
VMT wireには補完由来の区別がないため、trialのログとoffline sidecarで診断する。
5候補の同時計算コストは、本番の単一候補コストや計測E2Eとは別である。
入力停止時はlive adapterも最後の採用関節を処理したsteady時刻から50msで補完を失効させ、
直接依存するtrackerを無効化してbusへ再通知する。重複sourceでfilter履歴は進めない。
これは補完の継続時間で、capture/inference/送信/表示のE2E上限ではない。
無効化は次のpoll/送信周期で反映される。source全体の100ms鮮度期限も従来どおり適用する。
live失効時の`age_s`は最後のsource更新時点の診断で、receipt経過時間へ書き換えない。
現状liveログは状態件数だけを出し、2つの時計のageを混同した表示をしない。

### 採らない案

score閾値だけの除外は高scoreの左右取り違えを通すため採らない。
tracker抽出後だけの位置平滑化は既に出た回転の飛びを直せず、棄却値まで履歴へ入れるため代替にしない。
左右の自動再ラベル、全身hard骨長拘束、視点別の再三角測量policyは本実験へ混ぜず、
まずraw3Dの時間整合性だけでどこまで抑えられるかを独立比較する。

## 比較と検証方法

`scripts/temporal_eval/outlier_controls.py`が12人工対照を生成し、旧10候補と新候補を同じ入力で再生する。
人工対照: 単発/2reportの膝・足首・toe飛び、左右膝交換、持続する真の移動、開始停止、3m/s移動、
交互に矛盾する継続飛び、片足欠測。固定templateから骨長priorを作り、正解に未来を使ってcandidateを修正しない。

位置/回転の隣接変化、同時刻人工参照誤差、関節の採用/補完/欠測、tracker availabilityを記録する。
invalidにした区間の変化を0として数えず、有効pairと候補共通のeligible分母を残す。
processing_msはCPUの候補copyコストであり、filter位相遅れ/Jetson負荷/表示E2Eの証拠ではない。
同一入力の旧10候補は処理時間を除き完全一致を要求する。

```bash
python3 scripts/temporal_eval/outlier_controls.py cpp/build/tools/temporal_replay NEW_OUTPUT_DIR
# 固定raw3Dの追加比較
python3 scripts/temporal_eval/outlier_controls.py cpp/build/tools/temporal_replay NEW_OUTPUT_DIR \
  --input RAW_INPUT.jsonl --profile SUBJECT_PROFILE.yaml
```

### HTTP 2D記録からの参考再構成

`temporal_snapshot_import`はproduction TriangulatorをCPUで再利用する。元のHTTP記録は約30Hzで、
カメラ別counter・wall由来整数ms capture推定しかなく、kernel capture時刻/同期groupは失われている。
HTTP取得時刻の整数差を`measured_report`として保持し、元のlive3D再現と呼ばない。
校正のcamera ID/解像度を照合し、原本source/calibration bindingと各camera counter/capture時刻/skewを残す。

全counter不変のfresh snapshotは重複除外。人物なし/複数/欠camera/counter逆行/skew超過/3D不成立は
counter不変でもboundaryとして出す。既存出力は排他的createで拒否する。
既定15msはsnapshot推定skewの診断条件であり、ハード同期の保証ではない。
40ms診断と`--omit-camera 0`の反実仮想は感度比較専用。正常2視点が真値という仮定は置かない。

```bash
cpp/build/tools/temporal_snapshot_import TRACE.jsonl CALIB.yaml NEW_INPUT.jsonl
# 非同期snapshotへの感度確認専用
cpp/build/tools/temporal_snapshot_import TRACE.jsonl CALIB.yaml NEW_INPUT.jsonl --max-skew-ms 40 --omit-camera 0
```

## 結果と残件

初版の「予測残差だけ」判定は、前屈録画seq768付近の小さな切り返しを棄却し、
補完位置が離れてから再取得するため、共有有効798pairでも左足の最大stepが37.293→76.233mmへ悪化した。
採用位置への近さも判断に加えて、ノイズ速度による誤棄却を防ぐ回帰を追加した。
一方、この版ではHTTP参考再構成の右足が116.143→132.360mmへ悪化。
棄却/欠測後の復帰で離れた採用観測を結ぶ速度が次の補完へ出る例を確認し、
連続raw観測だけで速度をseedする追加回帰を入れた。改善例だけを採用根拠にしない。
最終v3では同じ右足の最大stepは105.115mmへ戻ったが、極端な飛びを全部除去したわけではない。

人工対照2160reportでは、2reportの膝飛びによる左腿の最大位置変化126.577mm/回転31.050°が
新候補で0mm/0°、左右膝交換の回転85.781°も0°だった。これは固定人工姿勢の注入対照の結果である。
60Hzの開始停止・3m/s移動ではgate前後の採用が変わらず、出力の最大変化もdistance5と一致した。
50ms gapを跨ぐ3m/sの移動開始/欠測からの復帰は150mm変化し、速度0からの初期許容量を超え得る。
20Hzの連続高速移動では再取得後も再棄却を繰り返し得るため、高速動作一般の合格とはしない。
継続する矛盾/足欠測は補完期限後に該当trackerをinvalidにし、有効reportは120→105。
このavailability減少を品質改善として隠さない。

### 最終v3の固定入力比較

通常動作は既存kernel capture grouping由来のraw、静止/前屈はlegacy frame名目時刻由来のrawを使う。
HTTP弱姿勢は非同期snapshotの参考再構成で、いずれも人体GTなし。各比較の分母は区間ごとに保持する。
下表は両候補が同じ隣接reportで有効なpairだけを揃えて計算した**位置step最大値**。
増減だけで精度や実機体感を判定しない。

| 入力 / tracker | 共通有効pair / 対象pair | distance5 mm | guard5 mm |
|---|---:|---:|---:|
| HTTP弱姿勢・15ms診断 / 左腿 | 104 / 106 | 62.125 | 29.122 |
| 通常動作 / 左腿 | 1668 / 1670 | 60.028 | 37.054 |
| 通常動作 / 左足 | 1666 / 1670 | 139.304 | 94.056 |
| 通常動作 / 右足 | 1665 / 1670 | 125.226 | 126.214 |
| 前屈 / 左足 | 808 / 1185 | 37.293 | 38.362 |
| 前屈 / 右足 | 1084 / 1185 | 20.371 | 22.381 |

HTTP15msの左腿report有効率は165/165→164/165、通常動作の左足は1705/1707→1706/1707。
前屈の足は左877/1186→872/1186、右1186/1186→1103/1186。
右足の有効率低下は、toe欠測時に従来の期限なしFKを許さなくした差も含む。欠損が増えたまま採用しない。
前屈の腿回転stepは共通pairでも左約91.5°・右約76.8°のままで、腿twistは未解決。
静止の評価対象1067reportでは下肢関節を全て採用し、位置/回転step最大値はdistance5と同じだった。
40ms診断でcam0を外す反実仮想では膝/足首/toe全245reportを採用し、gateは介入しなかった。
これはcam0観測の寄与を示唆する感度結果で、2視点を正解と認定したものではない。

ローカル成果物は`outputs/temporal-eval/20261002-outlier-v3-{synthetic,weak15,weak40,omit0,still,bend,motion}/`。
`report.json`は全11候補/26関節、`report.md`は抜粋、`manifest.json`と`complete.json`は
入力/profile/binary/実装hashと実行commandを束縛する。旧版v2・初版の悪化結果も上書きせず保持した。

### ソフト検証

Release全target build、`main --help`、全CTest 42/42（Python 158件を含む）pass。
localhostを使う既存testはsandbox外で実行。WebUI/設定の変更がないためpnpm検証は未実施。
凍結`577ebf9` binaryと同一7入力計7758reportの旧10候補を照合し、
`processing_ms`以外の全JSON数値/構造を保持する。比較記録は
`outputs/temporal-eval/20261002-outlier-checkpoint-comparison/`。

回帰対象: 単発の高score飛び、accepted速度の無汚染、50ms丁度/超過、不規則間隔、再取得、
補完中の足anchor非学習、期限切れFK無効化、teacher/prefix因果性、boundary・重複・switchのreset、
旧候補不変、HTTPのID/整数時刻/間引き/boundary/原本保持。

実機の再体感、閾値の調整、誤棄却率、跳ねが複数視点で持続する場合の採否は残る。
T-pose前後逆転の恒久修正と、浅い屈曲時の腿twistロジックそのものは今回の対象外。
チェックポイントbinaryは`outputs/temporal-eval/20261002-outlier-checkpoint/temporal_replay`へ保存。
通常設定への採用・カメラ/VMTサーバーの起動・Windows送信は行わない。

## Milestone

1. gate・補完の期限/由来・後段履歴の分離と回帰を実装する。
2. 人工対照と固定raw録画、HTTP参考再構成を分けて比較し、凍結旧binaryで既存候補不変を検証する。
3. `distance5`対`guard5`のVMT再体感で飛び/後追い/欠損を別評価する。閾値と本番採用は未決定。
