# pose-3d: 距離ベースの残差フィルタをoffline評価

着手: 2026-09-22。前段: [soft骨長・頭部伝播分離](pose-3d-soft-bone-experiment.md)。
ユーザー指定のcheckpoint `checkpoint/pose-3d-before-distance-filter-20260922` は`6891845`。
関連CTest 8/8・Python 114件を再実行してpass、旧binaryとログを
`outputs/temporal-eval/20260922-distance-checkpoint/`へ別保存した。localのみでpushはしない。

## 背景・選択

以前の`st_filter`（7月、PR #50のmerge `7bd4bde`、後にrevert）は距離適応EMA＋速度gateだった。
旧記録の位置比較は別推論runの入力差に交絡があり、今回は同一rawへの1回/report replayで比較する。
現在の`soft_bones_head_leaf`で異常な頭部観測のbodyへの伝播を止めた後、残った位置揺れだけを対象とする。
hip重み0.1、回転の距離filter、親FKからの位置再構成、品質gateの変更は今回は含めない。

候補は最大位置差2mm / 5mmの2つ。旧候補8つは数値とスキーマを維持し、
`--ik-experiments --isolate-head --distance-experiments`指定時だけ10候補へ拡張する。
`distance_experiment.schema=fitra_distance_experiment_v1`で別実験と識別する。

## 距離更新式

空間補正・既存floor/伸展/gate/補間を通した位置を`x`、このfilterの前回出力を`y`、
最大許容差を`L`とする。`d=|x-y|`、`r=2L`として

`alpha = d² / (d²+r²)`, `next = y + alpha*(x-y)`。

時間窓、未来、dt、速度を使わず現在位置と前回出力だけで更新する。
`|x-next| = d*r²/(d²+r²) <= r/2 = L`（float丸めの許容差を別に検証）。
小変位では強く抑え、大変位では直接追従に近づく。硬い停止領域はなく、一定targetへ徐々に戻る。
**これは時間遅れゼロではない**。低速や小振幅の動きは減衰/遅延し得る。
更新回数に依存するのでFPS不変とも呼ばない。上限はaccepted targetとの差であって人体GTとの差ではない。
大きな誤検出を追う可能性はあるため、既存gateの後に適用し、gateを緩めることはしない。

採らない案:

- ハードdeadband: 微動を完全に捨てるため、今回の主候補にしない。採点器の凍結対照には使える。
- 旧st_filter全体の復活: 時間適応、部位別速度、Kalman変更、twistまで同時に変わり切り分けにくい。
- 骨盤固定の相対frame: 不確かなrootを再び全身へ伝播させる可能性があり、world位置の最小候補から始める。
- 回転も同時に変更: 腿twistの観測可能性/解釈を別途扱うため今回はposition-only。

## 不変条件・配置

- C++ offline helperのみ。production runtime、REST/WS、設定、モデル、校正、原本は変更しない。
- `soft_bones_head_leaf`と同じ独立stateからtargetを作り、既存位置/回転処理の後に位置だけを更新する。
  quaternion/confidence/validity、post-IK骨格はbase候補と同一。補正位置を上流へ戻さない。
- 初回/復帰は現在targetへseed。tracker欠損で当該履歴を破棄し、欠損を有効出力へ変換しない。
- boundary/segment/gapで全履歴reset。参考正解は一切candidateへ入力しない。
- 位置の独立補正により最終tracker間距離は変わり得る。post-IK拘束残差が同じことを
  最終出力の剛体性の保証と取り違えない。姿勢の向きも位置から再計算しない。

## Milestoneと検証

1. Checkpointと旧binary保存、既存8候補の完全一致テスト。
2. 更新式/誤差上限/連続性/同変性/現在入力のみ/欠損reset/診断一致の回帰。
3. 固定templateの人工静止・低速・1/5mm正弦・開始停止反転・step・欠測を、ノイズなし/ありで比較。
   1mm動作は既存lagの未評価域に入るため、既知周波数の振幅/位相fitとイベントcrossingを別集計する。
   静止の数字だけで微小動作の消失を優勝扱いにしない。
4. 同じ実録画の静止/前屈/3視点Mを比較。人体GT、live遅延、実機受入れとは区別する。

全身中央値だけでなく腰・左右腿・左右足の分母を併記する。最終採用は自動化しない。

## 2026-09-22 初回結果

**2/5mm候補は比較用に保持し、本番への一律採用は保留**。時間窓を持たないことと、
時間的な追従遅れがないことは別だった。今回の式・固定capの結果であり、距離ベース手法全般の否定ではない。

`outputs/temporal-eval/20260922-distance-*`の5 runを完了。各`complete.json`で
`checkpoint_eight_candidates_unchanged=true`を確認。保存した旧binaryに同じraw/profileを通した
全8候補の位置/回転/availability・reference・seq/時刻/resetが完全一致した（処理時間は比較対象外）。
実録画3 runは原runの既存3候補とも照合。原本・校正・runtime・wireは変更していない。

### 実録画の静止

`20260922-distance-still`（入力1,185、採点対象940 reports）の位置RMS spread、単位mm。
表の全候補・全5部位は940/940有効。人体GT誤差ではなく、自然な静止動作も含むばらつき。
「部位中央値」は10 trackerの中央値で、同じ点の全身統合RMSではない。

| 候補 | 部位中央値 | 腰 | 左腿 | 右腿 | 左足 | 右足 |
|---|---:|---:|---:|---:|---:|---:|
| minimal | 6.602 | 5.392 | 6.888 | 6.378 | 5.132 | 6.323 |
| Kalmanのみ | 5.401 | 4.443 | 5.334 | 4.923 | 3.997 | 6.028 |
| current | 4.807 | 4.142 | 4.671 | 4.272 | 3.686 | 5.821 |
| soft_bones_head_leaf (base) | 5.058 | 4.960 | 5.242 | 5.156 | 2.263 | 4.287 |
| distance_2mm | 4.451 | 4.452 | 4.561 | 4.450 | 1.606 | 3.643 |
| distance_5mm | 3.772 | 4.031 | 3.831 | 3.714 | 1.191 | 3.004 |

### 小動作を消していないか

`20260922-distance-controls-clean`は60 reports/s、7動作各7秒、各先頭1秒を採点除外。
ノイズ0、固定template prior、全身のworld X並進。以下は腰の独立な対照結果で各360/360有効。
現実の全身動作やlive end-to-endの遅延を測定した値ではない。

| 対照・指標 | base | Kalmanのみ | current | 2mm | 5mm |
|---|---:|---:|---:|---:|---:|
| 5mm/s移動の10mm到達遅れ ms | 0 | 0 | 150 | 216.7 | 400 |
| 振幅1mm・0.5Hzの振幅比 | 1.000 | 1.047 | 0.945 | 0.501 | 0.137 |
| 同上の位相遅れ ms | 0 | 9.5 | 140.4 | 283.0 | 413.2 |
| 振幅5mm・0.5Hzの振幅比 | 1.000 | 1.047 | 0.987 | 0.902 | 0.744 |
| 同上の位相遅れ ms | 0 | 9.5 | 120.0 | 101.0 | 185.4 |
| 20mm/s開始時の10mm到達遅れ ms | 0 | 16.7 | 116.7 | 83.3 | 150 |
| 20mm/s反転後の原点到達遅れ ms | 0 | 0 | 116.7 | 83.3 | 150 |
| 100mm stepの50mm到達遅れ ms | 0 | 16.7 | 50 | 0 | 0 |

イベントは3連続reportのしきい値通過を確認し、その先頭時刻を採点する。確認処理は評価側のみ。
未到達はcensoredであり0ではない。正弦は定数項＋既知周波数のsin/cosのOLSで振幅/位相を分離し、
ほぼ凍結（振幅比0.1未満）では位相をnullにする。位相は周期内の値で全clipの遅れとは異なる。
100mm stepは不連続な数値ストレス対照で、人体の速度を模したものではない。
低速の停止後2秒窓の同時刻誤差p95は2mm候補0.772mm、5mm候補1.626mm。
この残差も「停止直後から完全追従」という説明を否定する。

同じ真値に各joint/axis独立std 3mmのGaussian noiseを加えた`20260922-distance-controls-noise3`
（seed 20260922、同じ2,940 reports）では、静止の部位中央値はbase 3.217→2.131/1.096mm。
一方、微動の位相が見かけ上先行する例もある。雑音が更新係数を変える非線形filterであり、
短い単一seedの1mm動作fitには雑音成分も混入する。この値を予測能力や負の実機遅延と呼ばない。
noise0の悪化をnoise3の平均的な追従で覆い隠さない。多seed・cadence sweepは未実施。

### 前屈・3視点と不変条件

- `20260922-distance-forward-bend`: 1,186 reports。base/2mm/5mmの左腿利用可能数は877/1,186、
  twist 45°急変はすべて7/836有効step。位置のみのfilterなので回転問題を解決しない。
- `20260922-distance-rescued-m`: 2,492 reports、採点対象1,707。
  base/2mm/5mmの全身twist急変はすべて1/16,675有効step。
  腰・左腿・右腿・左足・右足の位置有効数はそれぞれ1,705/1,705/1,704/1,705/1,703。
  原runは参照なしで、動作中の真値誤差や遅延の順位は付けていない。
- 5 runの全reportでavailability・quaternion・confidence不変、baseとの差がcap＋2µm以内。
  2µmはfloat/JSON往復の検証許容差で、モデルの観測精度ではない。
- Release replay build、focused CTest 8/8（Python 128件、距離関連14件）pass。
  prefix/teacher独立、境界/欠損/時刻gapのreset、式/cap/再収束、world並進/yaw同変性、
  diagnostics ON/OFF、旧候補/段階別拘束一致、tiny motion fit/凍結/未到達を確認。
  full CTest、全target build、live/HMD受入れはoffline-onlyの変更範囲につき未実施。

## 再実行

評価worktreeで以下を実行。`CHECKPOINT`は保存済み旧binaryへの絶対path、出力は毎回未使用directory。
runnerは入力/全Python script/C++実験source/binary/profileをhash固定し、完了時に再照合する。

```bash
python3 scripts/temporal_eval/run_postprocess_experiment.py OUTPUT_CLEAN \
  --binary cpp/build/tools/temporal_replay --distance-controls --noise-mm 0 \
  --baseline-binary CHECKPOINT
python3 scripts/temporal_eval/run_postprocess_experiment.py OUTPUT_NOISY \
  --binary cpp/build/tools/temporal_replay --distance-controls --noise-mm 3 \
  --baseline-binary CHECKPOINT
python3 scripts/temporal_eval/run_postprocess_experiment.py OUTPUT_RECORDING \
  --binary cpp/build/tools/temporal_replay --distance-experiments \
  --source-run COMPLETED_RAW_RECORDING_RUN --baseline-binary CHECKPOINT
```

`--distance-controls`はhead分離/distance実験/template priorを含み、実録画との併用は拒否する。
`report.json/md`は既存指標、`distance.json/md`は小動作/イベントとcap検証、
`constraints.json`はpost-IK拘束診断。10役割の全結果をJSONに保持する。

次は今回の2/5mmをそのまま本番へ足さず、必要ならより小さいcapや更新係数の下限を別候補として
同じnoise0/3の小動作対照へ通す。下限導入は実質的な時間平滑化の性質を持ち、
静止ノイズとの交換条件が消えるわけではない。股関節品質・回転解釈の改善は引き続き別軸。
