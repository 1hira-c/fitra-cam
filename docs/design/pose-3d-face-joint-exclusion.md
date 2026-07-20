# pose-3d: Halpe26 顔ランドマークの 3D body-lift 除外と頭部方向endpoint

(着手日 2026-07-20 / 関連: [pose-3d track](../tracks/pose-3d.md))

## 背景 / 動機

VR 出力の頭部位置・姿勢は HMD が提供し、fitra-cam の tracker 抽出も HEAD を生成しない。一方、
Halpe26 の顔5点（nose / left-eye / right-eye / left-ear / right-ear、index 0–4）は従来、他の
21点と同様に三角測量、kinematic-tree Kalman、骨長 IK、subject profile の骨長観測へ流れていた。

顔点は小さな領域へ密集し、カメラ間の検出対応がずれやすい。subject calibration では顔周辺だけ
再投影誤差が大きくなるフレームがあり、全 valid joint の中央値である `reproj_err_med_px` と
`valid_joints` に、VR body tracker に不要な観測が混入していた。さらに per-joint の SVD / Kalman
補正を消費する一方、tracker pose には一切寄与しない。

目標は、2D 推論と26点の公開 index 契約を維持しながら、顔5点を3D body-liftとsubject calibration
品質から外すこと。HMD alignment に使う `head_top(17)` と、胸・腰・肩チェーンに使う
`neck(18)` は維持する。追補として、3D viewerで身体の前後だけは判別できるよう、nose観測を
品質指標や骨として扱わず、`head_top` 起点の固定長direction endpointへ変換する。

## 検討した案

- **subject解析だけで顔骨長を無視する** — 校正成果物への混入は減るが、三角測量の再投影中央値、
  live Kalman、旧profile由来のIK拘束には顔点が残る。処理系ごとに契約が分かれるため不採用。
- **RTMPose decodeで顔出力を捨てる** — argmax CPU は少し減るが、2D `/ws` の26点観測を失い、
  Python数値参照との一致範囲も変わる。TensorRT engine は固定26出力なので、backboneと出力headの
  GPU計算やD2H転送はほぼ減らない。別の専用モデル評価まで保留。
- **COCO17を使う** — neck、hip-center、toe / heel が無く、安定したVR tracker抽出と床接地に
  必要な点まで失うため不採用。
- **raw noseを通常jointとして残す** — 前後は見えるが、問題となった再投影値、可変骨長、6D joint
  Kalman stateが復活するため不採用。
- **3D body-lift共通ポリシーでHalpe26の0–4を除外し、noseは方向だけ合成（採用）** —
  2D互換を維持し、三角測量を入口としてKalman / IK / profileへ同じ不変条件を適用する。
  raw noseは方向抽出の一時入力に限定し、公開3D endpointは固定長にする。

## 採用設計

`lift/keypoint_format.hpp` に純粋関数
`participates_in_3d_lift(KeypointFormat, joint)` を置く。Halpe26 index 0–4だけ `false`、
それ以外は `true` とする。COCO17は既存の17点契約と `median valid joints >= 13/17` 品質基準を
変えないため、全点を従来どおり処理する。

データフローと不変条件:

1. RTMPose は Halpe26 26点を従来どおり decodeし、2D `/ws` JSONも不変。
2. `Triangulator::triangulate` はeyes / ears (1–4)を処理しない。nose (0)だけは
   **position-only DLT**で一時的な向きを取得し、reprojection計算・outlier pass・`view_count`・
   `valid_joints`・中央値集計へ入れない。
3. `head_direction.hpp` がraw `head_top→nose` から、処理後の `neck→head_top` 軸と平行な成分を
   除く。残ったfront/back成分を正規化し、`head_top` 起点の **0.15m固定長endpoint**として
   nose index 0へ格納する。退化・欠損時は `valid=false`。eyes / earsは常にinvalid。
4. `SkeletonKalman` は顔の6D stateのpredict / correctを実行しない。nose方向だけは3成分EMA
   （時定数0.10秒）で平滑化して単発のfront/back反転を抑え、body state更新後のfiltered
   `head_top`へ固定長で付け直す。これは行列逆演算を行わず、欠損がbody品質へ波及しない。
5. `IkSolver` は顔骨長のobserve / enforce / drift集計をskipし、IK後の `head_top`へdirectionを
   再度付け直す。既存v2 subject profileに
   顔骨長が保存済みでも無視するため、schema更新・再校正を必須にしない。
6. `dump_keypoints_3d` のprofile accumulatorにも同じgateを置き、将来triangulator以外の入力を
   受けても顔骨長を成果物へ書かない。

`Skeleton3D.kp_count` は互換性のため26のまま、noseは表示用endpointとしてvalidになり得るが、
`view_count[0]` / `reproj_error_px[0]` は0、`valid_joints` の最大は21のまま。したがって公開配列indexを
変えず、subject calibration品質と表示用directionを分離できる。

`head_top(17)` は連続HMD alignmentのcamera側対応点、`neck(18)` はspine方向・胸腰tracker・
alignment品質に必要なため、Kalman / IKを含めて従来どおり処理する。

## Milestone

- **M1**: 共通gate、Triangulator / Kalman / IK / offline profile配線、回帰テスト、文書更新。
  公開schemaや設定を増やさない単一の可ビルド変更としてまとめる。
- **M2**: raw noseのposition-only DLT、軽量な方向EMA、固定長head-direction endpointを追加。
  M1の品質分離を維持。

## 検証

- `test_triangulator`: Halpe26のカメラ片側だけ顔点を120px/80pxずらし、body 21点と
  `head_top` / `neck` がvalid、eyes / earsはinvalid、nose endpointは0.15m固定・頭軸直交、
  `view_count[0]` / reprojectionは0で顔誤差が中央値へ入らないこと。
- `test_kalman_chain`: 顔の6D stateを作らず、filtered `head_top`からnose endpointが0.15mを維持し、
  1フレームだけnoseが反転しても公開directionが反転しないこと。
- `test_pose_recognizer`: 顔を数m外してもIK bone driftが増えず、noseは固定長へ再合成される一方、
  `head_top→neck` 長は従来どおり拘束されること。
- focused: `ctest --test-dir cpp/build --output-on-failure -R 'triangulator|kalman_chain|pose_recognizer'`。
- full: `cmake --build cpp/build -j`、`./cpp/build/main --help`、`ctest --test-dir cpp/build`。
- 実機/録画: 同じsubject calibration sessionを変更前後で解析し、四肢・torsoのcoverageを維持したまま
  顔外れフレームによる `reproj_err_med_px` 悪化が消えること。`valid_joints_median` は最大21として読む。

### 保存済みsubject sessionでの確認

`calibrations/subjects/subject01/sessions/20260627_192157`（4 pose × 75 = 300 frames）を、
同じ calibration / YOLOX-S FP16 / RTMPose-M Halpe26 FP16で再解析した。body 21点の座標は保存済み
旧JSONLと一致し、品質値は顔を除外した後続statsだけが変わった。M2追補後もbody品質の契約を維持し、
nose index 0だけが0.15mの表示用endpointとして追加される。FP16推論の再実行ごとに集約値は数百分の
1px程度変動するため、表はM1実行時の比較値を記録する。

| 指標 | 旧26点集計 | body 21点集計 |
|---|---:|---:|
| `reproj_err_med_px` | 2.65774 | 2.54853 |
| `valid_joints_median` | 26 | 21 |
| `major_bone_coverage` | 0.933333 | 0.933333 |
| `profile_bone_drift_pct` | 2.64019 | 2.63978 |
| quality status | pass | pass |

pose別では elbow-flex 2.74314→2.46364、standing 2.67091→2.58535、T-pose
2.37853→2.11796に低下し、knee-flexは2.76280→3.17648に上昇した。顔点の誤差がbodyより
偶然低いposeでは中央値が上がるのが正しい挙動であり、目的は数値を常に改善して見せることではなく、
VR body trackerに対応する観測だけで品質を判定することである。新profileの0–4骨長は全て0だった。

M2実装後の再実行では300/300フレームでdirection endpointがvalidとなり、長さは
0.149991871–0.150004748mだった。eyes / earsのvalidは0、noseの`joint_view_counts[0]`は全フレーム0、
body `valid_joints`は20–21、quality statusはpass（再投影中央値2.55236px）を維持した。pose内の連続
296遷移でdirection変化は平均3.363°、最大15.145°、45°超とfront/back反転はいずれも0だった。
同じ入力をEMA前に測った値（平均18.533°、最大106.139°、反転3回）に対し、6D joint Kalmanを
復活させず表示方向を安定化できた。

## 残課題

- RTMPose GPU計算を減らすには、顔出力channel自体を持たない専用学習/engineが必要。現変更は
  後段CPUと校正品質の整理に限定する。
- 固定長0.15mは表示用でありsubject固有の頭サイズを表さない。viewerで長すぎる/短すぎる場合だけ
  定数を再評価し、subject profileやruntime設定には昇格させない。
- `head_top` を将来外す場合は、連続HMD alignmentをneck対応へ置き換え、実機でyaw/xz収束と
  head lever-arm残差を再評価してから行う。
