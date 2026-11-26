# pose-3d: 人物の2D関節を使うマーカーレス外部校正

(着手日 2026-10-28 / 関連: [Issue #66](https://github.com/1hira-c/fitra-cam/issues/66))

## 背景・目標

intrinsicは既存のChArUco校正を維持し、extrinsicの導入操作を身長入力と短い案内動作に減らす。
初版は固定2〜3カメラ、単一人物、Halpe26、平坦な床を対象とする。導入時間2〜3分は体験上の目安。
設置時・移設時にC++で校正を実行し、必要な観測をまとめて全カメラの配置を推定する。
撮影・案内にかかる時間と校正計算にかかる時間は別に記録し、計算には精度を優先して時間を使える。

2026-10-28の実装範囲は、人物不在で進められる**固定intrinsicからの相対カメラ姿勢推定、
有界な2D観測収集境界、オフラインreplay、合成データ検証**。人物を用いたlive受け入れ、
身長によるmetric scale、床の自動推定、案内UIとflowへの適用は後続である。

### 校正方式を選ぶ基準

同じ設置・同じ収録で、まず人体だけのC++校正を基準にする。背景を併用する案も最後まで
同じ全台構成で解き、校正成功率（全試行が分母）と最終的な3Dの誤差を比較する。

- **配置の精度**。現配置の独立参照に対するカメラ間の相対回転差（度）を全組で測る。
  実寸化できた段階で、カメラ中心間の距離誤差（cm）、床の法線の角度差（度）と高さ方向のずれ（cm）も測る。
  実寸精度を測る際は推定結果の尺度を参照に合わせ直さない。
- **撮影空間での3D精度**。推定に使わなかった区間で、手前・奥や異なる高さにある対象を復元し、
  同じ設置で別に測った長さ・位置と照合する。3台なら2台で復元した点を残り1台へ投影し、
  各カメラを順に外して偏りも調べられる。これは画像上の整合性を調べる検査である。
- **再現性**。配置を固定したまま区間AとCを独立に校正し、全台の配置のばらつきを測る。
  同じ結果が繰り返されることと、実測に対する誤差を個別に報告する。

開発時の独立参照には、同じ設置・画角で別撮りした検証用ボードや独立に測った距離を使える。
利用者の操作は身長入力と短い案内動作のままとする。尺度に使った身長、過去の床の既知点、
他の設置の外部校正は精度の採点から外す。参照を取得していない量は未測定とする。
特徴点数・照合数・再投影誤差は原因調査に使う。CPU試走の秒数や毎フレームの処理速度で候補を絞らない。

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
- 画像寸法はintrinsicと一致が必要。crop/resizeを暗黙推定しない。finite値、RTMPoseが返す非負のfloat32範囲のscore、
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
- **後続: UI/flowと実機受け入れ**。現配置でのカメラ間・床・3Dの精度と再現性を測る。
  導入体験として初見完了率、案内の長さと計算の待ち時間も記録する。

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

### 人物がいる機会のまとめ撮り（2026-10-28追加）

[撮影runbook](../runbook-markerless-capture.md)と`tools/markerless_record`で、
校正A・独立検証B・反復C・将来の床/scale用・動作確認を1セット370秒で収録できるようにした。
これは実機で確定した必要長ではなく、別pose/time区間を確保する初期予算である。
`--record`を明示しない限り、cameraを開かず出力も作らない。
設定は`load_camera_config`でcamera節だけを読み、runtimeのconfig loaderの厳格さを変えない。

captureは既存V4L2の最新1slotを単一ownerでpollし、ファイルI/OをDQBUF threadへ入れない。
raw MJPEG/YUYVを再圧縮せず、offset/size、process/kernel seq、kernel timestampと意味論、
host DQBUF timestamp、実際の案内時刻、actual format、intrinsic実体とSHA-256を保存する。
遅い書き込みで捨てたsourceとdriver gapは別に記録する。既存event corpusのperson triggerを
偽装して流用せず、推論/relay/upload/校正適用を起動しない専用toolとした。
raw payload上限32GiBと空き2GiB reserve、3秒camera停止をguardし、途中終了の原本も保持する。

`check_markerless_capture.py`は標準libraryだけのoffline QCで、checksum、index範囲/連続性、
時刻のknown/unknown/mixed/regression、source非再使用の20ms frame set、予定静止区間を検査する。
原本bundleを変更せず、fit/validation/repeat/floor partition付き最大72候補をJSONへ出す。
保存・timing QCと、人が見えている/静止している/幾何的に十分という判定は別で、
`calibration_ready / person_visibility_checked / geometry_checked`はfalseを維持する。
候補へ2D推論を付けて現行replayへ渡すadapterは未実装で、raw frame位置とtimestampを先に確保する。

合成JPEGと模擬kernel時刻でrecorder→QC→候補出力、64bit時刻、途中終了、上書き拒否、
capacity、時刻不明/混在/同期miss、corruption、unsafe path、camera-only preflightを検証する。
2026-10-28にRelease configure/build、`main --help`、新規収録テストを含む全CTest 42/42が成功。
収録テスト内ではPython標準libraryによる12ケースも実行した。既存640出力設定と1280出力設定で
deviceを開かないpreflightのID/寸法照合を確認したが、物理camera割当とFOVの一致を確認した証拠ではない。
撮影時の実fps・ブラー・全身可視性・drop率・所要容量・十分な録画長の実機受け入れは未実施。

相対camera配置の精度と人体pose modelの系統誤差は未実証。非同期cameraでは静止でも手ぶれや
露光差が残り、kernel timestampの意味も実機で確認が必要。holdoutは独立した真値ではなく
自己整合の補助指標である。最新のSimCC posteriorやIMU融合を校正用実測へ戻さない。

## 2026-11-26 実人物録画での入力修正と相対姿勢の検証

RTMPoseのdecodeは `max(0, min(x_max, y_max))` を返す。9/19録画の3台単一人物1,702組のうち
153組、10/25の360組のうち166組で1を超える関節があった。最大はそれぞれ1.1182と1.0850。
旧collectorは1関節でもscoreが1を超えると同期組全体を拒否し、蓄積中なら過去の組も破棄した。
入力検証と `min_score` の指定範囲をRTMPoseの出力に合わせ、元値をそのまま三角測量の重みに渡す。
既定の採用下限は0.5。負値、非有限値、float32の範囲を超える値は拒否する。

カメラごとのRTMPose-Mの `observations_2d` と元の撮影計画をhash・seq・kernel monotonic SOE時刻で照合した。
3台で1人を検出した連続区間を使った。9/19は最長890組から等index間隔で120組を選び、
10/25は既存の立位・移動・着座移行の窓から全360組を採った。着座中は3台で揃う組が0だった。
入力は3台の撮影時とK/歪み係数が一致したChArUco内参、画像寸法1280×960。各試行は1つの区間で完結し、
選択した480組に画像外の関節はなかった。録画当時の外部校正は比較にだけ使った。

| 区間 | 同期組 | 修正前の入力拒否 | 修正前のリセット | 修正後の3台相対校正 |
|---|---:|---:|---:|---|
| 9/19 最長の単一人物区間 | 120 | 5 | 4 | 検証再投影p95 31.40px |
| 10/25 立位 | 80 | 26 | 12 | cam1の対応点支持が不足 |
| 10/25 移動 | 80 | 46 | 14 | 検証再投影p95 25.95px |
| 10/25 着座移行 | 200 | 94 | 12 | 検証再投影p95 25.85px |

修正後は全480組を保持し、入力拒否・リセット・容量による破棄は各0。相対幾何は4区間とも不合格で、
上表の再投影p95を得た3区間は既定上限8pxを超えた。各5組目を留保する現行の検査を使用した。

同じ観測・数値設定で2台ずつ解くと12通り中9通りが区間内の検査を通った。
10/25のcam0・cam2では、同じカメラ配置でも立位と移動で相対回転が115.26度、基線方向が127.16度異なった。
立位の解を固定すると、立位の留保分で支持された関節対応は254/318（79.9%）、未使用の移動区間では
406/1,516（26.8%）。逆に移動の解を固定すると、未使用の立位で1,565/1,587（98.6%）、
着座移行で2,593/3,349（77.4%）だった。分母は両台でscoreが0.5以上の同一関節で、
支持には正深度・視差1度以上・両台の再投影誤差8px以下を求めた。
独立区間の検査に使った計算は、同一区間の留保分でもC++の再投影p95と1e-5px以内で一致した。
短い静止だけで誤った解が通り得る。案内動作で異なる位置・向きを取り、推定したカメラ姿勢を固定して
別区間での支持率と再現性を測る根拠とする。今回の録画は身長・床用の案内収録と実寸評価を含まない。

照合元は `outputs/temporal-eval/20260922-corpus-3d-m` と `20261025-cam0-comparison.ukkQm8`。
入力hashと選択、修正前後の診断、全カメラ対の結果、別動作での検証と再実行スクリプトは
`outputs/markerless-recorded-20261126-Qjcf5ZkA/` に保存した。
Release全targetのbuild、`main --help`、ホストでの全CTest 42/42が成功した。

### 3台を一つの配置として再検証した結果

固定内参と元の2D関節だけをC++の `markerless_extrinsic_replay` に渡すと、上記4区間の
3台校正はいずれも棄却された。追加診断としてカメラ対をそれぞれ独立に解き、cam0→cam2
の回転とcam0→cam1→cam2の合成回転を比べた。ペアが3組とも通った移動区間でも両経路は
2.44度、着座移行区間では14.79度ずれた。これは当時の推定同士の食い違いで、実際の設置
に対する角度誤差は未測定。各区間の全台の成否・理由とペア診断を
`outputs/markerless-recorded-20261126-Qjcf5ZkA/three-camera-check.json` に記録した。

## 旧録画での背景照合（探索記録）

10/25の画像に付けた床5点の注釈は現在の校正評価から除外した。この記録とCPU時間を使った
モデルの採用判断は取り下げ、上記の「校正方式を選ぶ基準」を使う。

<details>
<summary>10/25録画の床マットと学習済みモデルの試験条件・数値</summary>

10/25録画のcam0・cam2で、上記の立位・移動から推定した相対姿勢をそれぞれ固定した。
推定後の別区間から撮影組180・320（約2.35秒間隔）の元JPEGを取り、同じ床マットの露出した
外角を起点に、ベッドの支柱に向かう辺とカーテンに向かう辺を見て5点を手で対応づけた。
各画像で角位置をサブピクセル精密化し、各カメラの2時刻の平均位置を使用した。最大の時刻間差は
2.13px。各点は1回ずつ数えた。人物と床の印刷タグを除外し、既存の外部校正やタイルの寸法は使用していない。

| 固定した人体由来の姿勢 | 既定条件を満たす床点 | 各点で大きい方のカメラ再投影誤差の中央値 |
|---|---:|---:|
| 立位からの推定 | 1/5 | 23.67px |
| 移動からの推定 | 5/5 | 2.72px |

全点が両カメラで正深度となり、既定の支持条件は視差1度以上・両画像の誤差8px以下。
人物姿勢を使わずに床4点から歪み補正後の平面射影を求めると、留保した5点目の転送誤差は
cam0→cam2で3.32px、逆で5.93pxだった。格子には取り違えの余地がある。cam2の5点の
対応を全120通り入れ替えると、移動の解で5点すべてが支持された割当ては正しい並びを含む8通りだった。

同じ画像の背景全体でRootSIFTの双方向比率0.75、両画像で時刻間差4px以下、別の点と16px以上離す
条件を試した。人物・タグに記述子が掛かる点も除くと、2時刻とも対応した候補数は
cam0/1・cam0/2・cam1/2で順に1・0・0。cam0/1の1候補は移動の解で正深度を満たさなかった。
今回の5点による比較は手動対応での検証である。自然な床の角や継ぎ目を自動で使う際は、
反復模様の並びと外周を照合し、人体による推定とは独立に位置関係を確認できた点で候補を検査する。
共通の模様が乏しい場合は背景の判定を保留する。平面上の5点による比較から実寸や床高は確定できない。

入力hash、対応座標、全点の誤差、床の切り出し図と再現手順は
`outputs/markerless-background-20261126-2TfPGZvQ/` に保存した。

### 学習済みモデルで背景の対応を探した結果

上と同じ10/25の2組の元JPEGを使った。マスクした人物・印刷タグを灰色で埋め、マスクと画像端から
元画像上で24pxより遠い点を使った。床の5点の座標と人体から推定したカメラ姿勢は採点にだけ使った。
疎な特徴点方式は896×672・各画像最大1,024点、屋内用LoFTRは640×480のグレースケールで計算した。
表は画角の差が大きいcam0・cam2で、各時刻の候補数と、両画像で4px以内に再現した候補数を示す。
再現した候補は両画像で他の候補から16px以上離れたものを数えた。

| 検出と照合の構成 | 1時刻目 / 2時刻目 | 2時刻で再現 | 移動区間の人体由来の配置と整合 |
|---|---:|---:|---:|
| RootSIFT・相互比率0.75 | 2 / 0 | 0 | 0 |
| DISK・相互比率0.75 | 0 / 0 | 0 | 0 |
| DISK＋LightGlue | 46 / 47 | 1 | 1 |
| XFeat・相互最近傍 | 259 / 244 | 38 | 3 |
| XFeat＋LighterGlue | 13 / 37 | 2 | 2 |
| DeDoDe v2検出器L・記述器B・dual-softmax | 74 / 59 | 11 | 3 |
| LoFTR屋内モデル | 0 / 1 | 0 | 0 |

右端は正深度・視差1度以上・両画像で8px以内の整合数。人体由来の配置自体に誤差があり、
この数から背景全体の正答率は求められない。床マットの既知5点では、どの方式も各時刻・
2時刻の再現分とも正しい対応が0件（両画像で同じラベルの8px以内）。DISK＋LightGlueで
残った1点は固定家具の同じ文字だった。XFeatの相互最近傍ではマットの外角と別物の角が
2時刻とも結ばれた。1時刻目には格子のDをEへ取り違える例もあった。

XFeatはマッチング前には各時刻で5点中2点を両画像に検出できた（1時刻目はD・E、2時刻目はB・E）。
cam2の向きを90度刻みで変えて候補を合わせても、既知5点の再現は0点だった。
両画像の下半分だけを照合してもXFeatの相互最近傍は1時刻目のD→Eを取り違え、既知5点の正答は0点だった。
別途、同一の背景画像を14px・7px移動して明るさを変えた対照では、全構成が既知変位3px以内の
対応を回収した。この実験で調べたのは、当時の録画の2台の画像間で対応が再現するかまでである。

使った構成の一次資料は[LightGlue](https://github.com/cvg/LightGlue)、
[XFeat](https://github.com/verlab/accelerated_features)、[DeDoDe](https://github.com/Parskatt/DeDoDe)、
[LoFTR](https://github.com/zju3dv/LoFTR)。推論はJetson上の一時環境でCPU・FP32、2スレッド。
重みの出典・SHA-256、全対応座標、比較画像、再実行手順は `outputs/markerless-lightglue-20261126-uUWi5EK4/` に保存した。

</details>

## 空室の3台初期値と人体による実寸校正

2026-11-26。固定ChArUco内参と同じ配置の空室1組からRoMa v2で相対姿勢を解く。
平面以外の固定物も使い、fisheyeを逆投影して3組のカメラ対と3視点の点を一緒に調整する。
cam0を基準座標、cam0–1間を1とする。GPUの1回限りのRoMa推論・初期バンドル調整は
現時点では実験用Pythonにある。C++ツールの入力は相対姿勢と元画像上の観測だけ。

```bash
python3 cpp/tools/export_markerless_background.py ROMA_RUN INTRINSICS.yml NEW_BACKGROUND.yml
cpp/build/tools/markerless_hybrid INTRINSICS.yml NEW_BACKGROUND.yml FIT.jsonl CHECK.jsonl 1.63 NEW_DIR
```

`ROMA_RUN` に `roma-geometry.json` の `relative_poses`、`roma-pairs.npz` の
`tracks[N,3,2]` を置く。変換にはその推定に使った内参を指定する。`fitra_markerless_background_v1`
はcam0〜2のID・解像度・K・歪み係数・モデル、4×4の基準カメラから各カメラへの変換、
同じ背景点を観測した6列の画素座標を持つ。C++で内参との一致、剛体回転、基線規約を検査する。
人体の入力は既存の `fitra_markerless_observations_v1`。3台が同じ単一人物を観測したkernel時刻、
source sequence、同じepochと生の2D関節を使う。画面外の点は欠測とする。
fitの終端とcheckの先頭を各カメラで0.5秒以上離す。

- cam0を固定し、他の2視点で三角測量した背景点と肩・肘・股・膝を使って残り2台のPnP補正を試す。
  各カメラの補正を1.5度・cam0–1基線長の3%以内に制限する。人体の採用数は背景支持数の半分まで。
  checkの人体について各カメラを順に外し、外したカメラでの予測と元の2D座標を比較する。
  人体の中央値が0.1 px超改善し、90%点が悪化せず、補正に使わない背景領域でも誤差増加が
  中央値0.25 px・90%点0.5 px以内の場合に補正を採用する。この背景検査は微調整についての検査。
- 2視点以上で支持された頭頂・股・膝・足首・左右足部を使う。脚の伸展を検査し、両足の観測から
  平面をRANSACで推定する。平面上の広がりを2方向で確認し、人物の頭方向を上とする。
  接地候補の頭頂から平面までの中央値を入力身長に合わせる。XZの入れ替えや古い床座標は使わない。
  世界の原点は支持された足位置の重心、Xはcam0画像の横方向を床面に射影、Zは床の上向き法線。
- 背景予測の中央値4 px・90%点8 px、人体予測の中央値12 px・90%点50 pxを超える配置を棄却する。
  別時間の立位候補で足の床からの距離の90%点が8 cm、頭頂高と入力身長の差の90%点が12 cmを
  超える場合も棄却する。これらは試作の出力条件。独立した3Dの正解に対する許容精度ではない。
  出力は新規ディレクトリの `report.json` と、通常のrunで読める `calibration.yaml`。
  `latest.yaml` への反映は行わない。

### 保存映像の実時間試行

9/22の「動作A」は同じ録画に人物の横移動・前屈と退出後の空室を含む。94.995秒の空室を使い、
床のタグをマスクした3枚をOrinで照合した。3視点の背景対応623組、初期バンドル調整のランドマーク714点。
9〜44秒を人体の推定用、46〜56秒を補正選択・尺度と床の検査用、58〜96秒を追跡用とした。
既存のYOLOX-S・RTMPose-M Halpe26を使い、推定用231組・検査用66組を得た。
身長163 cmで推定したcam0–1基線は3.721 m。基線長の独立した実測値はない。

人体による補正候補は検査区間の人体予測の中央値を11.02→9.98 pxに下げたが、
90%点が27.22→28.19 px、背景の90%点も1.80→3.71 pxに増え、元のRoMa姿勢を採用した。
立位候補は推定用88組、検査用42組。検査用の足の床からの距離は90%点1.91 cm、
頭頂高と入力身長の差は90%点10.93 cm。服・足・姿勢・2D頭頂推定を含む再現誤差。
較正から隠した床の3枚のタグの12隅を、同じ空室画像の2台ずつから復元した。
床既知点・設置座標・タグ寸法は使用していない。推定床との高さ差の絶対値は90%点3.16 cm、
12隅の平面法線と上方向の角度は0.73度（評価範囲約1.47×0.50 m）。

JetsonのC++ runの入力だけを元のMJPEGに差し替え、各カメラの実際のcaptureとホスト到着の
時刻差を保って38秒再生した。新規の外部校正、既存のsubject01 profile、通常のTensorRT・
三角測量・IK・TrackerExtractor・ローカルWebSocketを使用した。58〜82秒の24秒で、
実観測の腰・両足首を含むFusionPoseを703回（29.3 Hz）、TrackerAxisを706回配信し
そのうち703回で全6軸がFreshだった。人物付きWebUI 714フレームでは10個全ての位置と
単位quaternionが有効だった。退出後は人物なしに遷移した。HMD・SteamVRへ送信する試験は含まない。

別の9/19録画では人体予測の中央値13.76 pxで画像整合ゲートを超えた。
11/26の現配置はドアを開けた空室で3台の相対解まで。今回の人体・実寸・追跡証拠は9/22の配置に属する。
ローカルの動画、YAML、時刻・ログは `outputs/markerless-hybrid-20261126-QTVWSHGn/` にある。
