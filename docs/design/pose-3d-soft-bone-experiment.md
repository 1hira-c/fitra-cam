# pose-3d: 観測重み付き骨長補正のoffline比較

着手: 2026-09-22。前段: [股関節感度評価](pose-3d-hip-observation-sensitivity.md)。
本番Kalman/IK/tracker処理、モデル、設定、校正、wireには適用しない。

## 比較する仮説

股関節を固定して膝・足を押し出す代わりに、両端の観測重みで補正を分配すると、
良好な膝・足観測への誤差伝播を減らせるか。現時点では正しい解剖学的GTを持たないため、
人工対照で効果と副作用を測り、保存済み録画では参照なし診断を行う。

`temporal_replay --ik-experiments`に限り、既存3候補へ以下を追加する。

| 候補 | 空間補正 | Kalman / One Euro |
|---|---|---|
| no_ik | IKなしの切り分け用。採用候補の意味ではない | なし |
| soft_bones | scoreに応じた観測tether + softな骨長拘束 | なし |
| soft_hips | 同じsolverで11/12/19の観測重みだけ0.1倍 | なし |

床、tracker専用伸展補正、出力gate/holdは既存のまま共通。
tracker confidence由来の補間も残るため、パイプライン全体が無状態/遅延ゼロという意味ではない。
soft_bones対soft_hipsが観測重みだけの対照であり、production IKとの差を全て
股関節重みの効果と読み替えない。hard hinge補正は実験solverには含めず、その逸脱を報告する。

## solverと不変条件

目的関数: `sum_i w_i ||x_i-y_i||² + 10 sum_(p,c) (||x_c-x_p||-L_pc)²`。
現在の辺方向を固定したSPD系と辺方向の更新を12回交互に解く。履歴・未来入力・待機・dtを使わない。
世界座標で特定点を固定せず、欠損点の合成もしない。score/validityを改善値に書き換えない。
基礎重みは`clamp(score, 0.05, 1)^2`。SimCCの元scoreはそのまま保存する。
scoreは解剖学的正確さ/校正品質の確率ではなく、hip重み0.1も未校正の仮説値。
これを再投影品質で較正した信頼度と呼ばない。3D幾何品質による重みは未導入。

骨長はsubject profileまたは既存Halpe26身長priorを一致させる。
現在の身長priorがindex12だけにhip-center距離を持つ非対称性も、今回黙って修正しない。
offline helperに写したpriorは全targetをheaderへ出し、production IKとの単独辺テストで差を検出する。
欠損/観測時に一致した端点の辺は方向を捏造せず拘束対象外。正の観測tetherにより系を正定値にする。
顔0–4はbody最適化対象外、head directionは既存合成を使う。

`--ik-experiments`なしでは候補・出力header契約・数値は不変。
追加候補は独立stateを持ち、既存候補へ入力を戻さない。boundary/gap/segmentで既存履歴をreset。
headerの`ik_experiment.schema=fitra_soft_bones_experiment_v1`と候補一覧で実験を識別し、
IKが全候補共通ではなくなるので実験時は`common.ik`を記載しない。
`constraint_metrics`はpost-IKの骨長残差RMS/max・supportと5–175°外のhinge数を保存する。
これは拘束充足であって人体精度ではなく、missingは分母から消さずsupportを併記する。

## 採用・不採用の判断を分ける

- hip-only対照: 入力不変の膝/足の位置差、腿のswing/twist差、可用数、twist支持を併記する。
- 正常移動/開始停止/欠測対照: 動作消失、lag、境界と復帰を調べる。静止だけで重みを決めない。
- 骨長残差/hinge逸脱: soft化やIK削除で拘束を捨てた結果を「精度向上」にしない。
- 保存済み録画: 未知の人体GTやtwistを作らず、候補間の診断差として扱う。
- CPU処理時間: replay内の演算時間であり、実機capture-to-display latencyではない。

独立Kalmanへの全面回帰、股関節の無効化、足固定、新規依存追加、live適用は今回行わない。
この試験で改善しても、追加収録の全密度品質と動作確認、HMDでの最終受入れは別途必要。

## 実行

```bash
cmake --build cpp/build --target fitra_temporal_replay -j2
python3 scripts/temporal_eval/hip_sensitivity.py NEW_OUTPUT \
  --binary cpp/build/tools/temporal_replay --amplitude-mm 20 --ik-experiments
```

検証: 既存候補不変、誤差0、平行移動/回転の同変性、現在入力のみ、欠損非補完、
境界reset、score/非対象点保持、prior一致、同一solverでの重み差、骨長残差分母/nullを含む。

保存済み録画には`run_postprocess_experiment.py NEW_OUTPUT --binary REPLAY --source-run COMPLETED_RUN`。
sourceを省略すると既存6動作の人工系列。完了hashとsubject profileを確認し、入力をbyte単位で維持、
既存3候補の数値を保存済みbaselineと照合する。実録画へ人工GTや推測したphaseを加えない。

## 初回結果（2026-09-22）

Release build、focused CTest 8/8（Python 99件、追加13件）pass。
股関節5/20mm各15ケース、人工6動作2,160 reports、保存済み実録画3 runを完了した。
後者は校正候補の3視点M 2,492、legacy前屈1,186、legacy静止1,185 reports。
既存3候補は保存済み出力と不変。原本・runtime・wireは変更していない。

- 股関節20mmの人工前屈jitter: 最終足位置の対照との差RMS（左右の悪い方）はminimal 19.86、
  soft_bones 5.31、soft_hips 1.41mm。これは物理的な精度ではなく、同じ補正の対入力差。
- 人工静止の左右逆向き20mm固定bias: 右腿twist差43.4679°→soft_hips 0.0030°。
  両方240/240支持pairだが、骨長残差RMSは1.525→5.208mm、5–175°外hingeは0→240/960。
  hard hingeを外した効果と観測重みの効果を混同しない。
- 人工移動の部位/区間中央値: soft_hipsの振幅比1.001、位置lag 0ms。
  これは全tracker/全場面の無遅延ではない。足単体では既存出力補間等によりlagがあり、
  短い欠測前窓のlag推定も不安定。CPU演算p95は約0.08ms台だがlive latencyではない。
- **legacy静止では悪化**: 暫定静止940 reportsの位置揺れ部位中央値はminimal 6.602、
  soft_bones 6.936、soft_hips 11.978mm。waistは5.392→25.380mm、
  左腿6.888→12.369mm。左右足は5.132/6.323→2.398/4.370mmであり、
  足を守る一方で腰/腿へ揺れを移した可能性を示す（人体GTによる誤差ではない）。
- legacy前屈の左腿45°以上twist stepはminimal 8/836、soft_hips 7/836。
  同じ出力利用可能数877/1,186で、劇的な改善とはいえない。Kalman系は1,155/1,186で分母が異なる。

結論: 一律hip重み0.1を本番へ採用しない。誤差の上流固定を緩める効果はあるが、
逆に他関節のノイズを腰へ戻す代償とhinge制約の扱いが残る。次は膝/足首側に誤差を入れる
逆条件と観測scoreの対応を確認し、股関節だけが常に悪い前提へ過適合しないよう評価器を補う。
全target build/full CTest・live/HMD受入れはoffline変更に範囲を絞るため未実施。

成果物: `outputs/temporal-eval/20260922-soft-bones-{hip5,hip20,motion,rescued-m,forward-bend,still}/`。
各manifest/completeが入力・binary/source/scripts・結果hashを固定する。

## 膝・足首側の逆条件（2026-09-22）

`joint_fault_controls.py NEW_OUTPUT --binary REPLAY`は静止/前屈×膝/足首×score 0.95/0.2の
8ケース（clean/fault各2,400 reports）を追加する。股関節入力は全く変更しない。
high/low-score間も座標と乱数は同一で、宣言scoreだけの影響を測る。
scoreは入力が本当に正しい確率ではなく、誤差をscoreが正しく知らせた場合/知らせない場合の対照。
共通runnerは既存hip-only CLI/候補を保ち、対象indexをmanifestへ固定する。

- 静止・膝20mm・高score: 最終腰位置の対照との差RMSはminimal 0、soft_bones 1.052、
  soft_hips 4.192mm。最終足（左右の悪い方）もsoft_bones 3.801→soft_hips 5.352mm。
- 同じ座標で膝scoreだけ0.2へ下げると、腰差はsoft_bones 0.134、soft_hips 1.109mm。
  scoreが誤差を正しく示せれば抑制できるが、hip固定弱重みの副作用自体は残る。
- 静止・足首20mm・高score: 腰差0→0.902→3.840mm、最終足差15.409→18.106→19.244mm
  （minimal→soft_bones→soft_hips）。足首誤差ではsoft化自体が不利なケースもある。
- 前屈でも同傾向。入力不変のhipが動くため、下流の良い観測を守る効果と、
  下流の悪い観測を上流へ戻す効果を両方評価できた。実録画悪化の原因をこれだけで確定しない。

結果: `outputs/temporal-eval/20260922-soft-bones-joint-controls/`。
全48 case/candidateで段階差・欠損/回転support・拘束残差を保存。
対象外点不変、high/low-score対照同一、seed再現、誤差0、全段採点を回帰対象へ追加した。
focused CTest 6/6（Python 103件、追加4件）pass。共通runnerへ移した既存CLIも、
誤差0/clean・fault各4,500 reportsで全支持関節の対照差0と完了hashを確認した。
一律hip重み0.1は引き続き不採用。次は保存済み静止rawの部位別異常と拘束伝播を切り分ける。

## 頭部leafの一方向化による切り分け

legacy静止のraw頭頂17は940/940点が存在するが、位置のばらつきRMSは216.8mm、
neck→head_topの保存prior 273.9mmに対する距離残差RMSは551.4mmだった。
score中央値0.736でもこの異常を示しておらず、全身の双方向拘束に入れると腰まで波及し得る。
人体の真の動き/校正/推論のどれが原因かは未確定だが、offline候補の副作用は対照で分離できる。

`--ik-experiments --isolate-head`に限り`soft_bones_head_leaf` / `soft_hips_head_leaf`を追加。
頭頂17→首18からbodyへのフィードバックだけをなくし、bodyを解いた後に17を首からの保存骨長へ
一方向投影する。頭部を固定したり欠損を合成したりはしない。既存6候補の計算は変更しない。
同一hip重みの候補同士で比較し、head分離とhip重み変更を混同しない。
頭priorがない場合は元候補と完全同一。顔方向の合成は既存処理を維持する。

8候補時はoffline markerを`fitra_soft_bones_experiment_v2`へ分け、既存3/6候補も読める。
`run_postprocess_experiment.py --isolate-head --diagnostics`で同じ完了録画の段階コピーも保存する。
これは品質に応じた外れ値処理ではなく、特定leafの方向性を変える原因切り分け用の候補。
頭以外の外れ値、移動/欠測、骨長不整合、ひねりの観測可能性を解決したとは扱わない。

### 切り分け結果と候補の扱い（2026-09-22）

同じ実録画3 runで、既存6候補の全出力数値がhead分離追加前と一致することを確認した。
legacy静止の940 reportsでは以下。位置揺れは実際の微動も含み、GT誤差ではない。

| 候補 | 腰の揺れRMS mm | 左/右腿 mm | 左/右足 mm | 部位中央値 mm |
|---|---:|---:|---:|---:|
| minimal | 5.392 | 6.888 / 6.378 | 5.132 / 6.323 | 6.602 |
| soft_bones | 17.185 | 7.120 / 6.752 | 2.223 / 4.303 | 6.936 |
| soft_hips | 25.380 | 12.369 / 11.587 | 2.398 / 4.370 | 11.978 |
| soft_bones_head_leaf | 4.960 | 5.242 / 5.156 | 2.263 / 4.287 | 5.058 |
| soft_hips_head_leaf | 4.911 | 5.200 / 5.054 | 2.298 / 4.343 | 4.983 |

全欄940/940出力。post-IK hip-center自身もsoft_hips 22.613→分離後4.943mmへ減る。
頭部からの拘束伝播が、この試作で増えた腰の揺れに寄与したと切り分けられる。
本番で以前体感した遅延・腿twistの原因を認定した結果ではない。
拘束残差RMSはminimal 0.698、head分離2候補1.007/0.585mm（19,740骨pair）。
5–175°外hingeは0対1,777/1,736（各3,760pair）で、hard hingeの相違は依然残る。
この範囲外というだけで人体として誤りと断定もしない。

legacy前屈の左腿twist急変はminimal 8/836、分離後2候補とも7/836。
左腿出力はいずれも877/1,186。新規3視点Mでも、head分離前後で全trackerの急変数は
soft_bones 1/16,675、soft_hips 0/16,675のまま。未観測window85件は急変0扱いにしない。
head分離で位置の悪化を解消できても、前屈の腿ひねり問題の解消には至っていない。

### 通常動作・欠測の対照

`run_postprocess_experiment.py NEW_OUTPUT --binary REPLAY --isolate-head --diagnostics --synthetic-bone-prior`
は既存6動作2,160 reportsを、固定人工templateのhead/torso/feet込みpriorで再生する。
test runや未来から骨長を学習せず、templateから作った人工profileを成果物へhash付きで保存する。
`--source-run`との併用は禁止し、実録画のprofileを上書きしない。
arm_swing等のfixture自体は完全な剛体骨運動ではなく、物理精度ランキングとは区別する。

- head分離2候補の動作振幅比中央値1.0003/1.0004、位置lag中央値0ms。
  ただし既存の足処理で、並進の左足16.7ms、start_stopの左足50msはminimalと同じく残る。
  静止脚の前屈等、動きがないためlag未定義の項目は0扱いにしない。
- minimal/分離2候補とも出力17,935/17,940 tracker-reports。欠測を隠して数値を改善していない。
- stage solverは現在入力のみ。prefix、境界reset、欠損/score保持、同変性、頭のみの外れ値の
  body非伝播、頭priorなしの完全同一性、診断ON/OFF同値、CLI誤用拒否を検証した。
- Release replay build、focused CTest 8/8（Python 114件、head関連11件）pass。
  full CTest/VR runtime起動/HMD受入れは実施していない。

結論: `soft_bones_head_leaf`を次の検証用候補として保持する。hip重み0.1との差はこの静止録画では
小さく、膝/足首逆条件の代償もあるため、hipを一律0.1にする根拠にはしない。
学習型補正の導入前でも、観測の誤りを全身へ伝播させる構造を評価器で検出できるようになった。
実機への採用は保留。未確定のカメラ/観測品質と腿twistを、さらなる平滑化で隠さない。

成果物: `outputs/temporal-eval/20260922-head-leaf-{still,forward-bend,rescued-m,motion-matched}/`。
利用者向け入口: `outputs/temporal-eval/20260922-postprocess-comparison-summary.md`（ローカル成果物）。
