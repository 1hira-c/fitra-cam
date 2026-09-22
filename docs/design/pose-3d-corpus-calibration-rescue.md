# pose-3d: 保存済み原本の校正救済と2D観測再利用

更新: 2026-09-22。既存録画の救済実験。live校正の自動更新やIssue #66の実装ではない。
関連: [原本qualification](pose-3d-corpus-qualification.md)、
[後段評価器](pose-3d-temporal-evaluation.md)、[既存床タグ校正](pose-3d-floor-apriltag-extrinsic.md)。

## 背景と境界

保存済みM/Lの3視点raw推論は完了したが、立位のトラッカー参照は品質不足で0%。
cam1の床タグ残差も大きく、ユーザーから微小なカメラずれの可能性が申告された。
原本/2D観測は残っているため、人物を撮り直す前に収録時点の幾何だけを再検証する。

既存校正、latestポインタ、床map、subject profile、runtime設定は変更しない。
元runはimmutable。候補は新しい出力ディレクトリに作り、本番には適用しない。
校正とmaskを人物モデル/後段の評価スコアに合わせて選ばない。

## 構造と不変条件

1. 完了済みrun/原本checksumを再検証。元planから1秒刻みの画像を選択。
   初期設定は推定0〜12秒、検証28〜41秒（先頭group基準）、立位評価16〜24秒とは分離する。
   frame再利用や時間帯重複を拒否し、原seq/kernel時刻/payload索引を保持する。
2. production AprilTagDetector（CLAHE有効）とFloorTagMap/FloorExtrinsicSolverをCPU toolで利用。
   intrinsic・歪み・床mapは固定。2D人物関節、IK/平滑化出力は校正推定に使わない。
3. 推定区間で各tagのcornerを平均し、各tag最低3回・最低2tagを要求する。
   検証区間は推定に入れず、画像上のcorner残差を元校正/候補で比較する。
4. offline診断用の事前固定gate: solver RMS≤3px、検証RMS≤3pxかつ元より改善、
   検証で2tag以上・3画像以上、検証各画像RMS≤4px、移動≤0.1m・回転≤5°。
   点数不足/不適合cameraは元校正を保持する。平面縮退は記録し、合格済みとはしない。
   これらは実証済みの製品受入れ条件ではなく、危険な候補を適用しない実験用制約。
5. 保存済みの同一2D観測をproduction Triangulatorで再計算する。推論モデル、group、
   seq/時刻、複数検出による境界は不変。旧校正での再計算も行い、元3Dとの数値一致を検証する。
6. 校正候補の元校正・床map・推定/検証画像・gate・binary・reportをhashで紐付け、
   derived runに元runと変更理由を残す。capture時点からこの校正だったと偽装しない。
   通常`run_corpus.py`の校正一致チェックを緩めず、明示的なderived経路を別にする。
7. M/L両runへ同一候補を使い、既存の未来参照生成・3候補replay・採点を再実行。
   参照品質条件は変えない。参照有効率と静止揺れを報告し、動作lag/実機合格は主張しない。

## 不採用案と限界

- 今のカメラで校正して過去録画へ無条件適用: 収録後の移動を区別できない。
- subject骨長で幾何ずれを吸収: 身体形状へカメラ誤差を押し付けるため不採用。
- 同じ画像でfit/validation: 過適合を見落とすため分離する。
- 1tagのPnPを全域の正解扱い: 遠方・平面配置の弱さを隠すため、更新条件を満たさなければ保持。
- 教師が0%だから再投影閾値を緩める: 判定を良く見せるだけなので行わない。

床mapの形状精度、人物高さでの外挿、身体profileの元校正への依存は残る。
held-outの同じtagでの整合性は絶対3D精度の証明ではない。
動作セット、独立姿勢基準、実機体感との対応付けは引き続き必要。

## 検証

原本変更/派生出自の不一致/推定検証重複/不足対応点/品質不合格/時刻逆行を拒否する回帰を追加。
旧校正の再三角測量が元のraw 3Dをfloat精度で再現すること、2D観測が変わらないことを確認。
Release tools build、関連CTest、実録画で完了マーカーとartifact hashを確認する。
新規依存・カメラ収録・VR出力・live設定変更は行わない。

## 実装と実行

- `cpp/tools/corpus_geometry.cpp`: CPU専用の`calibrate` / `triangulate`。
  calibrationではproduction detector/solverを呼び、検証結果と採否をcamera別に記録する。
  stridedなT_cw translation列はcloneしてfisheye projectPointsの連続メモリ条件を守る。
  triangulateではraw header、plan FNV、sequence/64bit時刻、観測数と曖昧検出を検査する。
- `scripts/temporal_eval/calibrate_corpus.py`: 元runと原本をhash照合し、同じgroupが再構成できるか確認。
  `--confirm-floor-map-unchanged`はユーザーによる配置一致の申告であり、収録時hashの証明ではない。
  完了時に原本を再検証する。`complete.json`なしの途中結果は採用しない。
- `scripts/temporal_eval/retriangulate_corpus.py`: 旧校正control→候補校正derivedの順で処理。
  座標/score/再投影/ray angle/有効点を照合し、数値再現に失敗すればderivedを完了扱いしない。
  2D観測は元rowから無変換で保持する。manifestに`calibration_derivation`を追加し、
  capture時点の校正と今回の実験候補を区別する。通常runの厳格な校正一致チェックは維持する。
- `pseudo_reference.py`はM/Lの元校正・候補出自・geometry binary一致も要求する。
  capture-boundとderivedの混在は拒否する。未来参照による候補出力不変チェックは従来通り。

```bash
cmake --build cpp/build --target fitra_corpus_geometry fitra_temporal_replay -j2
python3 scripts/temporal_eval/calibrate_corpus.py ORIGINAL_M_RUN NEW_CALIB_RUN \
  --geometry-binary cpp/build/tools/corpus_geometry --floor-map FLOOR_MAP --confirm-floor-map-unchanged
python3 scripts/temporal_eval/retriangulate_corpus.py ORIGINAL_M_RUN NEW_CALIB_RUN NEW_M_RUN \
  --geometry-binary cpp/build/tools/corpus_geometry
python3 scripts/temporal_eval/retriangulate_corpus.py ORIGINAL_L_RUN NEW_CALIB_RUN NEW_L_RUN \
  --geometry-binary cpp/build/tools/corpus_geometry
python3 scripts/temporal_eval/pseudo_reference.py NEW_M_RUN NEW_L_RUN NEW_REFERENCE_RUN --labels LABELS
```

## 2026-09-22 実録画結果

保存先は通常workspaceの`outputs/temporal-eval/`。
`20260922-calibration-rescue-v2`、`20260922-rescued-{m,l}`、`20260922-rescued-reference`を完了。
推定13時刻、検証14時刻で、同じframeを再利用しない。intrinsic/歪みは3台とも完全保持。

| camera | 推定に使えたtag数 | 検証RMS 元→候補 px | カメラ中心差 / 回転差 | 扱い |
|---|---:|---:|---:|---|
| cam0 | 8 | 2.032→1.715 | 1.43mm / 0.077° | offline候補へ更新 |
| cam1 | 5 | 6.954→2.160 | 34.65mm / 0.429° | offline候補へ更新 |
| cam2 | 4 | 3.334→3.772 | 76.23mm / 1.984° | 検証で悪化、元のT_cwを完全保持 |

候補の中心差/回転差は推定値で、カメラをその距離だけ動かしたという実測ではない。
全候補に平面縮退警告があり、人物高さでの精度保証にはしない。
以前の少数画像・Python OpenCV検出より、production CLAHE検出でcam2にも複数tagが得られたが、
点数が増えただけで候補を採用しない。上表は同じ検証cornerでの比較である。

M/L各2,492組について旧校正controlの最大差は座標/score/再投影/ray angleすべて0。
候補校正で再計算し、未来参照の有無によるcandidate/reset不変も確認した。
立位472 reportの部位別位置揺れRMS中央値は以下。全候補のtracker出力は有効。

| 設定 | 元校正 mm | 候補校正 mm |
|---|---:|---:|
| minimal | 8.518 | 3.999 |
| Kalmanのみ | 3.815 | 3.162 |
| Kalman＋One Euro | 3.108 | 2.866 |

**トラッカー参照有効率は依然0%**。Lのshoulder/kneeなどは品質通過が増えたが、
左右hip・hip center・右kneeなどの支持が足りない。誤差/lag/振幅比は未評価のままとし、
閾値・future window・本番filterを変えない。静止揺れの減少を真の位置精度改善と呼ばない。

補助診断として同じ立位L観測の2視点別DLTを確認。右hipの再投影残差p50は
cam0–1=7.54 / cam0–2=19.23 / cam1–2=9.21px（検出score≥0.3）。
単に1視点だけを除けば全身教師になる状態でもなく、残る校正・2D関節bias/見え方を切り分ける必要がある。
この補助診断で視点選別や候補の採用条件を変更してはいない。

Release build、関連CTest 7/7、内部Python 63件（skipなし）pass。
旧run・元校正・captureに紐付く7 assetのhash不変を確認。main/main設定の変更はないため
全アプリtarget/full CTestは未実施。live VRと実測GTによる品質受入れは未確認。
詳細・数値は通常workspaceの`outputs/temporal-eval/20260922-calibration-rescue-summary.md`。
