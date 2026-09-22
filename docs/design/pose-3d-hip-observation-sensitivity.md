# pose-3d: 股関節観測の誤差伝播を測る固定対照

着手: 2026-09-22。対象はoffline評価器。RTMPose-M・runtime補正・校正・wireは変更しない。
関連: [後段評価器](pose-3d-temporal-evaluation.md)、[校正・四方向診断](pose-3d-corpus-calibration-rescue.md)、
[観察ケース](../research/pose-3d-evaluation-cases.md)、[既存chain Kalman](pose-3d-locomotion-stability.md)。

## 背景

追加四方向原本では、同じ校正候補でLのraw hip品質通過が左0/24・右2/24、ankleは左右24/24だった。
これは再投影等の入口品質であり、股関節の時間的jitterや真の3D誤差を測った値ではない。
ユーザーは、画像ラベル上のhipが解剖学的な回転中心とは一致せず、向きでずれる可能性を指摘した。
校正残差、衣服・姿勢依存の推定bias、ラベル定義の違いの寄与はまだ分離できていない。

既往研究も2Dラベル/MoCapの関節定義差を論じているが、今回のRTMPoseや実録画の原因認定にはしない:

- [Needham et al., 2021](https://pmc.ncbi.nlm.nih.gov/articles/PMC8526586/):
  OpenPose/AlphaPose/DeepLabCutとmarker-based基準の比較でhipとkneeの系統差を報告。
  marker-based基準も完全な解剖学的GTではない。
- [PoseRN, 2021](https://arxiv.org/abs/2107.03000):
  人間が画像上で付けた関節位置とMoCap定義との差による多視点推定のbiasを扱う。

現行経路はhip-center (19)をrootとするparent-relative Kalman、親を保持して子を動かす骨長補正、
hip→knee等からのtracker抽出を含む。「骨格のroot」と「絶対に正しい観測anchor」を区別して見直す前に、
股関節だけの誤差がどの段で膝・足・腿回転へ伝わるかを計測する。
過去の足置き去り防止には親移動の伝播が必要だったため、単純に全関節独立へ戻す判断はしない。

## 採用した実験

`scripts/temporal_eval/hip_sensitivity.py`で誤差なし/ありの入力を生成し、同じproduction replayへ通す。
同一candidateの両出力の差を測る。raw poseとIK後を直接比べて、骨長priorの食い違いまで
注入誤差の効果と数えることを避ける。これは人体精度や実機症状の再現証明ではない。

### 15ケース

| 動作 | 誤差 | 対象 |
|---|---|---|
| 静止 | Gaussian jitter / 固定bias | centerのみ / 左右hip共通 / 左右hip逆向き |
| 前屈、入力足点は固定 | Gaussian jitter / 固定bias | 同上 |
| 身体yaw回転 | 向き依存bias | 同上 |

- centerはHalpe 19、左右hipは11/12。左右を動かしても19を再合成せず、独立観測経路として分離する。
- 既定の誤差量20mmは**仮定したストレス量**。実録画のpx残差や文献の誤差を換算したものではない。
  jitterは各world軸独立Gaussian、vector RMSの期待値を20mmにする。実現したRMSをcase別に報告。
- 固定biasはworld +Yへ20mm。左右逆向きでは右だけ符号反転し、左右midpointを維持する。
- 向き依存biasはworld +Yへ`amplitude * sin(2 * heading)`。
  **3Dでの代理入力**であり、カメラの2D annotationや三角測量を再現していない。
- score・有効性・股関節以外の観測・時刻・seq・phase・境界は対の間で同一。
  高いscoreでもずれた観測が入る場合を調べる。低品質入力の除外・教師mask緩和はしない。
- 各caseは60Hz・5秒、先頭1秒を採点外にして更新は行う。case間はsegmentを変えて履歴reset。
  先頭から誤差を含め、固定biasを後からstep投入した応答試験とは区別する。
- clean fixtureも身長1.7m priorに完全適合する解剖モデルではない。同じpriorを使う差分評価であって、
  clean出力を実物の正解へ格上げしない。実録画/subject profileを黙って混ぜない。

### 段階別の観測

`temporal_replay INPUT OUTPUT [PROFILE] [--diagnostics]`のoptional診断を追加する。
`--diagnostics`なしの既存出力契約・候補計算は不変。live REST/WSには何も追加しない。

headerに`diagnostics_schema: fitra_temporal_diagnostics_v1`、fresh rowにcandidate別の以下を加える。

- `post_kalman`: Halpe26 xyz/score/null。minimalではKalmanを通らないため入力そのもの。
- `post_ik`: 骨長/hinge補正後、床補正とtracker専用伸展補正より前のjoint。
- `pre_smoothing`: 床補正・伸展補正を使った抽出後、位置/回転smoothing前のtracker。
- 最終trackerは既存`candidates`を使う。boundaryの診断はnullで古いsnapshotを出さない。

診断はコピーだけで、候補へフィードバックしない。CPU時間にはコピーコストも入るため
instrumentation有無の`processing_ms`を性能比較しない。通常数値出力の不変性を回帰で検査する。

## 採点契約

schemaは`fitra_hip_sensitivity_{run,report,complete}_v1`の専用系列。
生成入力・binary・replay source・Python scripts・実行argvを保存。出力FNVと元入力を照合し、
行数・seq/時刻・reset・stage topology・対の共通設定を検査する。
完了前に入力/code/binaryを再hash照合し、完了マーカーに全artifactを記録する。
途中結果/既存出力を上書きしない。通常の実録画・擬似教師runとしては取り込めない。

- jointはraw/post-Kalman/post-IK、trackerは抽出直後/最終出力を対比較する。
  位置差のRMS/p95、符号付き平均差、平均差を除いた変動RMS、注入RMSに対する比を出す。
- 固定biasは変動RMS=0でも同時刻差RMS>0。jitterだけを見て「改善」としない。
- 注入0の対照では全位置差0、gainは割れないためnull。観測不能の差も0でなくnull。
  両系列の可用数・有効pair数・採点対象数を別記し、欠損で誤差が小さく見えるのを防ぐ。
- 回転は同時刻の出力差。quat符号同値を保ち、local +Zのswing/twistを使う。
  腿/下腿以外への解剖学的twist解釈はしない。両出力のroll confidence>0だけをsupported twistに使い、
  confidence不足と180°近傍の分解不能を別countにする。
- 全quatの差はアルゴリズム出力の変化量であって、物理回転の正解誤差ではない。
  支持付きtwistにも出力のconfidenceを使っているだけで、解剖学的正解の保証はない。
- 一覧は左右の悪い方を記載。全身平均で局所破綻を隠さず、JSONに全部位・支持数を残す。
  遅延、実測GT、採用候補の順位、自動合否はこの実験で決めない。

## 不採用案・次段

- 閾値を緩めてhip教師を有効化: 推定点の意味やbiasは直らない。
- hipを全面削除/無効化: 胴体移動や足欠測時の連動まで失う可能性がある。
- 足が今回通過したので固定anchorにする: 再投影一致は正確さ/接地の証明ではない。
- 大型モデル＋未来平滑化を解剖学的GTとする: 共有された定義biasは残り得る。
- 今回すぐ本番Kalman/IKを変更: 実験で誤差伝播の経路と代償を分けてから行う。

M0はこの対照実験と段階別採点。M1候補は品質に応じた観測重みと、観測点を一方的に動かさない
制約解法のoffline比較。固定骨長・姿勢prior・位置/向き/twistの扱いを別に検討する。
M2で保存済み実録画・動作ラベルと照合し、許容応答を維持できる候補だけ実機へ進める。
GT未確定の股関節を別の未確定な点へ置き換えるだけの評価にはしない。

## 実行・検証

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build --target fitra_temporal_replay -j2
ctest --test-dir cpp/build -N -R 'temporal_eval|tracker_extract|kalman'
ctest --test-dir cpp/build --output-on-failure -R 'temporal_eval|tracker_extract|kalman'
python3 scripts/temporal_eval/hip_sensitivity.py NEW_OUTPUT \
  --binary cpp/build/tools/temporal_replay --amplitude-mm 20
```

回帰は対象外関節不変、center/左右hip分離、固定biasとjitterの区別、seed再現、誤差0の負対照、
欠損分母/null、quat符号、twist観測不能、入力改変/lineage/設定不一致拒否、診断ON/OFFで候補不変、
prefix causality、boundary後に古いstageを出さないことを含む。

### M0実行結果（2026-09-22）

- Release構成とreplayを再build。focused CTest 8/8、`temporal_eval`内Python 86件pass（追加15件、skipなし）。
  対象はtriangulator、floor_contact_stabilizer、floor_tag_map、floor_extrinsic_solver、
  tracker_extract、tracker_extract_pos、kalman_chain、temporal_eval。
- 5mmと20mmで各15ケース、各clean/fault 4,500 reports、3候補を完了。
  case/candidate当たり240 reportsを採点し、各runの9 artifactを完了hashと再照合。
- `static/fixed_bias/hips_differential`のminimalで、左膝の対照との差はraw/post-Kalmanとも0、
  post-IKで5mm入力時0.598mm、20mm時2.255mm。股関節以外の入力を変えずに伝播を検出した。
- 同条件の右腿supported twist差p95は5mm時0.0458°、20mm時43.4679°（各240/240支持pair）。
  20mm時の差は抽出直後から存在し、minimal/Kalman/currentいずれの最終出力でも約43.47°。
  **一定の向きのずれであり、43°の時間的jitterを観測したのではない。**
  このfixtureの非線形な感度を示すが、伸展判定・参照方向・骨長補正のどれが支配するかは未分離。
- `forward_bend/jitter/hips_common`の20mm設定（実現入力RMS 19.49mm）で、左右の悪い方の
  最終足位置差RMSはminimal 19.86 / Kalman 9.10 / current 6.03mm。
  平滑化で対照との差が減っても、真の動作を保持したか・遅延が許容かは別の評価を要する。
- 合成骨格は伸展に近い立位を使う。20mmを実際のhip誤差と推定したり、
  全姿勢で20mmが43°を生むと一般化したりしない。観察した腿症状の再現/原因認定でもない。
- 本番C++/設定/モデル/校正/VR wireは不変。offline変更に検証を限定し、
  全target build/full CTest・live VR品質受入れは未実施。

ローカル成果物（Git対象外）:

- `outputs/temporal-eval/20260922-hip-sensitivity-{5,20}mm/`: manifest、対入力/出力、report、complete。
- `outputs/temporal-eval/20260922-hip-sensitivity-summary.md`: 人が読む比較要約と各runへのリンク。

次はこの固定対照と既存の移動/欠測対照を使い、品質に応じた観測重み・拘束の分配をoffline比較する。
良好な膝/足の観測を守ることと、実際の胴体移動や足欠測時の連動を失わないことを同時に検証する。
