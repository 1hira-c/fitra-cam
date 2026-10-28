# pose-3d: 人物の2D関節を使うマーカーレス外部校正

(着手日 2026-10-28 / 関連: [Issue #66](https://github.com/1hira-c/fitra-cam/issues/66))

## 背景・目標

intrinsicは既存のChArUco校正を維持し、extrinsicの導入操作を身長入力と短い案内動作に減らす。
目標は外部校正単体で2〜3分。これは未実証の製品目標であり、今回の相対幾何実装で達成した
性能ではない。初版は固定2〜3カメラ、単一人物、Halpe26、平坦な床を対象とする。

2026-10-28の実装範囲は、人物不在で進められる**固定intrinsicからの相対カメラ姿勢推定、
有界な2D観測収集境界、オフラインreplay、合成データ検証**。人物を用いたlive受け入れ、
身長によるmetric scale、床の自動推定、案内UIとflowへの適用は後続である。

## 検討した案

- **採用: 既存intrinsicを固定してraw 2D対応を使う**。既存のYOLOX/RTMPoseを将来の
  観測producerとして再利用でき、追加の印刷物やVR機器を必須にしない。
- **校正済み3DやSimCC posteriorを入力にする案は不採用**。未校正カメラの3Dが必要になる
  循環依存と、時間priorやIKで補った位置を実測扱いする誤りを避ける。顔0〜4は使用しない。
- **全機能を専念モードへ直ちに接続する案は保留**。相対解にはmetricと床の自由度が残る。
  既存CalibrationSetへ書くと、任意単位・camera座標をmetres・fitra Z-upと誤解させる。
- **新規BAライブラリは導入しない**。既存OpenCVのEssential/PnP/小行列演算で始め、
  合成GTと独立holdoutで数値特性を確認する。新しい実行時依存はない。

方式の参考は [Human Pose As Calibration Pattern (CVPRW 2018)](https://openaccess.thecvf.com/content_cvpr_2018_workshops/w34/html/Takahashi_Human_Pose_As_CVPR_2018_paper.html)。
先行研究の成立と、本構成での人体検出誤差・実機精度は別に検証する。
[OpenCVのrecoverPose](https://docs.opencv.org/4.10.0/d9/d0c/group__calib3d.html)から得るtranslationも
方向と任意scaleの情報であり、それだけで実寸にはならない。

## 構造・不変条件

### 入力と収集

`lift/markerless_extrinsic.hpp`が、camera ID、source seq、stream epoch、capture monotonic ns、
timestamp semantics、画像寸法、person count、raw Halpe26のjoint index/x/y/scoreを持つ。
frame setにはproducerが発行する非空のcontinuity tokenとsample IDを付ける。配列indexを
人物identityへ変換しない。sample内のcamera順序は問わないが、設定された全cameraが一度ずつ必要。
関節欠損は許し、不足は幾何solverが診断する。

`MarkerlessCollector`は単一ownerの共通ingest seamで、live captureや推論を起動しない。
既定240 set、設定上限1024 set。上限到達時は最古を捨てるが、source watermarkは保持し、
evictした古いframeの再投入も拒否する。intrinsic行列と採用観測は所有するコピーで保持する。

- 同一source seq、capture time、sample IDは二重採用しない。
- continuity/stream/clock変更、時刻・sequence逆行、不正なframe setで旧収集を破棄する。
  境界のframe自体は採用せず、次の妥当なsetから収集する。reset/reject/evictionを別計数する。
- 既知のkernel monotonic SOEまたはEOFを使い、semanticsを混在させない。合成用の
  `synthetic_monotonic`は明示的に区別し、実露光時刻の証拠にしない。
- 既定sync windowは20ms。既に組まれたsetのcapture max−minを検査する。
  この段階では非同期ストリームの最近傍matchingや静止区間の自動選別は実装していない。
- 画像寸法はintrinsicと一致が必要。crop/resizeを暗黙推定しない。finite値、scoreの範囲、
  画像内座標、joint indexの重複を検査し、単一人物以外は拒否する。
- 通常runtimeのlatest-frame-wins、TRT所有権、公開REST/WS、既存trackerには変更を加えない。

### 相対幾何

固定intrinsicに従ってpinhole/fisheyeを別々にundistortし、sample IDとjoint indexで対応点を作る。
score 0.5未満と顔0〜4は使用しない。毎5番目のsampleをholdoutへ分け、camera姿勢の初期化・
refineには一切使用しない。実録では、隣接frameをこの規則で割るだけでは独立評価にならないため、
前段で静止区間・位置単位にkeyframe化し、調整用と最終評価用のsessionも分ける必要がある。

基準camera[0]と他cameraの対応からEssentialをRANSACで求め、consensus全点の8-point再fitと
Essential特異値制約を適用する。最小サンプル解だけを最終poseにしない。正深度、視差、再構成構造、
homography優勢を検査する。初期解の支持が多いpairをanchorにする。残りのcameraはanchorの
三角測量点とPnPで共通scaleに載せる。**初版の3台接続にはanchor pairと3台目に共通の
joint tracksが必要**。camera0と十分な対応がない、または共有trackが不足する構成は失敗する。
任意の共視グラフを連結する汎用SfMは今回の範囲に含めない。

三角測量は最大3pairからinlier合意を選び、悪い1viewで良い2viewを引かせない。camera0を
固定した有界refinementを行い、anchor cameraのtranslation normを1に正規化する。
holdoutでは、採用された3D点の再投影を除外viewも含めて評価し、各cameraの独立した支持も検査する。
再投影誤差が小さくても正解を保証しないため、合成テストではcamera回転・translation・基線比を
GTと別に比較する。純回転、低視差、平面・直線、共有観測不足では部分poseを成功として返さない。

初期数値既定は6 set以上、pair 24点以上、RANSAC 3px、検証再投影上限8px、最小ray angle 1度。
これらは現行解像度での開発初期値で、人体実録や解像度を跨いだ製品の合格しきい値ではない。

### 出力は適用不能な診断

`MarkerlessRelativeResult`のgeometry `ok`は相対配置の成立だけを示す。座標系はcamera[0]の
OpenCV軸、単位は選択anchor baseline=1の任意単位。

- `T_camera_reference`はreference camera→各camera。既存のfitra Z-up world→camera
  `T_cw`とは型名・JSON key・schemaを分ける。
- `metric_scale_known=false`、`floor_aligned=false`、`runtime_ready=false`はこの段階の定数。
- `write_calibration`、`write_calibration_versioned`、latestの削除/更新、runtime起動は行わない。
- 後続のmetric/floor段を通すまでは、実寸位置・床高さ・fitra Z-upの意味を付与しない。

## オフラインreplay

既存intrinsic YAMLとraw 2D JSONLを入力する。JSONLの先頭行は
`{"schema":"fitra_markerless_observations_v1","keypoint_format":"halpe26"}`。
以降は`MarkerlessFrameSet`に対応する`sample_id / continuity_id / frames`。
各frameは`camera_id / source_seq / stream_epoch / capture_mono_ns / timestamp_kind /
width / height / person_count / keypoints`、keypointsは`[joint_index,x,y,score]`の配列。
整数時刻とseqを浮動小数へ変換せず、64bitのまま読む。1行256KiB・全体32MiBで制限する。
収集reject/resetやparse失敗を無人の校正成功にしない。

```bash
./cpp/build/tools/markerless_extrinsic_replay \
  --intrinsics calibrations/intrinsics/latest.yaml \
  --observations /tmp/markerless-observations.jsonl \
  --output /tmp/markerless-relative.json
```

出力schemaは`fitra_markerless_relative_v1`、既存ファイルへの上書きを拒否する。
`--output`省略時はstdout。exit 0はclean inputで相対幾何が成立した場合だけで、metric校正完了を
示さない。reportには入力量、reject/reset/eviction、cameraごとの相対pose、train/holdout診断を残す。
カメラ、TensorRT、SteamVRなしで使用でき、`.json`の診断成果物を通常校正として適用しない。

## Milestoneと検証

- **今回: M0の幾何コア/合成実証とM1の入力境界/replay**。精度の実機合格、M0全体の人物あり
  実証、M1 live配線が完了したことは意味しない。
- **後続: 人物あり収集**。既存2D producerとの同capture対応、最近傍matching、静止/位置多様性の
  案内、遮蔽・左右誤認、camera移動、live/replay等価性を実機で確認する。
- **後続: 実寸と床**。入力身長、頭頂・足部の観測誤差を評価し、metric scale、床、fitra座標への
  gauge変換を独立した段として実装する。失敗時は有効な旧校正を保持する。
- **後続: UI/flowと実機受け入れ**。2〜3分の所要時間、初見完了率、従来校正との差、再現性、
  基線/床の独立参照を測る。GTに対する合成の成功だけで実用品質としない。

自動テストは`test_markerless_extrinsic`（独立した合成人体投影、2/3camera、歪み、noise/outlier、
退化、holdout）、`test_markerless_collection`（容量、所有権、duplicate、境界）、
`test_markerless_replay`（strict IO、64bit、診断schema、CLI、既存出力保護）を使用する。
Release build、既存を含む全CTest、`main --help`で統合回帰を確認する。

### 2026-10-28の検証実績

- Jetsonのapt OpenCV 4.8.0でRelease configure/buildと`main --help`が成功。
  新規3件を含む全CTestはホスト実行で41/41成功。sandboxではCrowのloopbackだけが
  `open: Operation not permitted`となり、ホスト再実行で環境制約と区別した。
- 独立した合成GT投影のnoiseなし2/3camera・pinhole/fisheye/混在では、最大translation差は
  `9.27e-9` baseline単位以下、holdout再投影p95は`7.70e-7 px`以下。
- 0.3px Gaussian noise、少数の外れ値、関節欠損を含む6 seed×2/3cameraの12条件で、
  最大camera回転差`0.207164度`、translation差`0.004465` baseline単位、
  holdout再投影p95の最大値`0.651565 px`。実寸誤差へ換算しない。
- collectorは別途ASan/UBSan付きで成功。LeakSanitizerはptrace環境制約のため無効化し、
  リーク検査に合格したとは扱わない。replayは64bit整数・NUL末尾破損・reset・新規出力保護も検査した。
- 人物ありlive、記録済み実人物の校正精度、metric scale、床、導入所要時間は未検証。
  WebUIと推論には変更がなく、WebUIのtypecheck/buildやFP16数値再比較は対象外。

## 残る制約

相対camera配置の精度と人体pose modelの系統誤差は未実証。非同期cameraでは静止でも手ぶれや
露光差が残り、kernel timestampの意味も実機で確認が必要。holdoutは独立した真値ではなく
自己整合の補助指標である。最新のSimCC posteriorやIMU融合を校正用実測へ戻さない。
