# 姿勢推定モデル・オーディション計画（遅延×精度）

## Goal

RTMPose 固定の前提を外し、Jetson Orin Nano Super 上で遅延と精度の両面から
実用的な 2D 姿勢推定モデル（と動作点）を選定し直す。足部（toe/heel）を含む
topology を必須条件とする（Halpe26 に限定しない）。結論は実測のパレート表と
採用・不採用理由の記録として残す。

## 後続作業との順序（2026-09-07 更新）

まず本計画で姿勢推定モデルと運用条件を選定する。3D化した関節系列を入力にする
平滑化・直近未来予測 AI の開発は、選定後の別 backlog
[#65](https://github.com/1hira-c/fitra-cam/issues/65) とする。
検討資料: [3D平滑化・1〜2レポート先予測](pose-3d-temporal-prediction-plan.md)。

教師データは人手による全フレーム注釈を前提にせず、大型モデルと未来情報を使った
オフライン補正で作る方針。これは実測正解とは区別した擬似教師であり、モデル選定時にも
利用する場合は教師への一致度として評価する。後段の時系列予測モデル完成を、本選定の
開始条件にしない。

## Success Criteria

- 同一コーパス・同一手順で測った比較表があること。各 arm について
  遅延（`rtm_ms`、`det_ms`、`cap→pub`、`recent_pose_fps`、集約 fps）と
  数値再現性（同 variant の ORT FP32 基準の max / p95 / mean kpt L2、bbox IoU）と、
  動作品質（静止 jitter、関節欠損、動作中の破綻、同一映像の目視比較）が区別されている。
  異なる family の score は同じ確率尺度とみなさず、しきい値・判定方法を記録する。
- 遅延と動作品質から採用動作点が指名され、「なぜそれか」が一文で説明できる。
  同 variant FP32 との差を、モデル間の絶対的な姿勢精度の順位へ読み替えない。
  共通擬似教師を使う場合も、教師への一致度と教師由来の偏りを明記する。
  足部点（big-toe/heel）の数値差・欠損・jitter は全身点とは別に評価する。
- 採用モデルの既定 engine 切替（または現状維持）の判断と、切り戻し手順が文書化
  されている。
- 再現情報（ONNX ハッシュ、engine ビルドコマンド、TRT/CUDA 版数、電源モード、
  計測コミット）が結果報告に付いている。

## Context And Current Facts

- 現行パイプラインは YOLOX 人物検出＋RTMPose 2D pose の top-down 構成。
  検出は間引き（既定 `--det-frequency 10`）、間は bbox 再利用。最新フレーム優先
  （SPSC size 1、drop-old）、モデルごとに単一 TRT context・単一 CUDA stream
  （`docs/tracks/core-pipeline.md` live な制約、`CLAUDE.md` Architecture）。
- 運用モデルは RTMPose-M FP16（`docs/research/rtmpose-int8-eval-plan.md` 背景）。
  S/M の ONNX・FP16 engine と L 系 ONNX は `outputs/onnx`・`outputs/models` に
  既存。YOLOX は tiny（416）と S（640、TopK surgery 済み）が実績あり
  （`docs/research/yolox-detector-eval-result.md`）。
- 数値照合の基準は Python ORT FP32 で、合格目安は bbox IoU > 0.99、
  高スコア keypoint L2 < 1.0px（`docs/tracks/core-pipeline.md` 検証節、
  `docs/cpp-migration-plan.md` 検証戦略表）。FP32 でも max kpt L2 ≈ 1.15px の
  実測があり、FP16 は低スコア点のドリフト（Y 方向 100–200px）が既知。
- keypoint topology は COCO17 / Halpe26 を `--keypoint-format` で切替、
  配列は `kMaxKeypoints=26` 統一（core-pipeline track）。Halpe26 の顔 5 点は
  3D lift 対象外だが 2D JSON は 26 点維持（pose-3d track 設計原則）。
- 足部は 20–25（big-toe / small-toe / heel）に依存する下流が多い:
  `tracker_extract` の FootAnchor は heel（24/25）と big-toe（20/21）から
  ankle / big-toe を再構成し、伸展脚の toe 方向は ankle→big-toe 観測を使う
  （`cpp/src/lift/skeleton_def.hpp`、`cpp/src/tracking/tracker_extract.hpp`）。
  floor 接触安定化も sole 3 点を接地証拠にする。COCO17（17 点止まり）では
  これらが動かず、floor-contact 安定化は COCO17 no-op が既定の扱い
  （`docs/cpp-migration-plan.md` 検証表）。よって本 audition の必須条件は
  足部点を含む topology とし、COCO17 arm は評価しない。Halpe26 に限定はせず、
  ユーザ指名の MediaPipe Pose（BlazePose）を含む足部持ちの family は
  P2 のゲート付き候補とする（一次情報・実測は実行時に確認）。
- 使える計測資産: `det_bench`（wall-clock 比較の前例あり）、`pose_bench`、
  `dump_keypoints`、`python/scripts/dump_reference_keypoints.py`、
  `python/scripts/compare_keypoints.py`、`cpp/tools/CMakeLists.txt`。
  コーパスは `outputs/recorded_rtmpose/20260515_064342/raw_cam{0,1}.mp4`
 （2 視点×30 秒、correctness 実績あり）に加え、既存 `outputs/records/` に
  9シーン×3カメラのMJPEG AVIがある。各約20秒、1280×960、約58.72–58.91fps。
  静止・Tポーズ・ski姿勢・前屈・屈伸・歩行・dash・kick・椅子遮蔽を一次比較に使える
 （2026-09-07: metadataと各ファイル先頭・中間・末尾の読み出しを確認）。
  この確認は全編の品質保証やカメラ間の厳密な同期証明ではない。
- MoveNet Thunder は過去に評価され、人 skeleton が成立せず採用外
  （`docs/research/movenet-tensorrt.md`）。再開条件は公式 runtime baseline
  での成立確認から、と結論済み。本計画では再評価対象に含めない。
- INT8 評価は別計画（`docs/research/rtmpose-int8-eval-plan.md`、calibrator 未配線）
  があるため、本オーディションは FP32/FP16 に限定し、INT8 は結果報告の将来課題に回す。
- 計画セッション時点で外部ネットワークは不通（web 検索応答不正、
  `curl` で github・docs 取得が両方タイムアウト）。外部モデルの一次情報確認は
  実行フェーズに延期し、本計画の Phase 1 はリポジトリ内で裏付けできる arm のみで組む。

## Constraints And Non-goals

- 対象 HW は Jetson Orin Nano Super 実機のみ。電源モードは MAXN_SUPER
  （`nvpmodel -m 2 && jetson_clocks`、`CLAUDE.md` 訂正済み）に固定して測る。
- TensorRT 10.3 / CUDA 12.6 / apt 版 OpenCV の枠内で動くこと。
  OpenCV・TensorRT を pip 導入しない（`AGENTS.md` 境界）。
- Python 実装は数値参照・フォールバック。新機能を足さず、calib blob ダンプ等の
  再現用スクリプト追加に留める。
- REST / WebSocket JSON スキーマの破壊的変更なし。keypoint 順序・点数の変更は
  `kMaxKeypoints` 経路と WebUI 表示の互換確認を必須にする。
- 評価対象は足部点（最低でも big-toe＋heel）を含むモデルに限定する。
  COCO17 系は候補に入れない。topology の系統は問わない（Halpe26／BlazePose 等）。
- 非目標: INT8 PTQ 本格化（別計画）、検出器の新規学習、3D lift / IK / VMT 側の
  変更、校正 UI の移行。`docs/archive/phaseN-*.md` は編集しない。

## Key Decisions

1. 単一勝者ではなくパレート frontier で選ぶ。遅延と動作品質の重み付けは事前に
   固定せず、同一映像での比較と運用観点（VR 用途の jitter・欠損・動作追従）で
   動作点を指名する。数値再現性と姿勢精度を同じ軸に混ぜない。
2. Phase 1 は足部点を含む arm に限定する（RTMPose Halpe26 系 × FP16/FP32、
   YOLOX tiny/S、検出間引き sweep）。COCO17 系は足がなく下流（FootAnchor・
   floor 接触・toe 方向）が動かないため評価しない。外部未確認のモデル名を
   計画の推奨に含めない。外部一次情報を引けない計画時の制約による。
3. 実装・precision の数値再現性基準は variant ごとの Python ORT CPU FP32 にする。
   各 engine を同 variant の FP32 と照合し、variant 間の優劣は同一コーパスの
   動作品質で読む。大型モデル等の共通擬似教師を使う場合は、この数値再現性試験と
   分けて教師への一致度を集計する。
4. 検出器も audition の対象に含める。top-down の実コストは
   `det_amortized + pose` であり、pose 単体比較では運用判断ができないため。
5. 非 RTMPose 系は Phase 2 のゲート付き survey とする。ユーザ指名により
   MediaPipe Pose（BlazePose）を P2 候補に含める。受入ゲートは足部点
   （最低 big-toe＋heel 相当）の写像可能性＋adapter・schema 対応・
   TRT ビルド性＋ライセンスとし、通過分だけ Phase 1 と同一手順で測る。
   BlazePose の性能・精度に関する主張は計画時に未確認のため、実行時の一次情報
   確認と実測で判断する。その他の family 追加は実行時の一次情報確認後に決める。
6. 計測時は idle/standby を無効化し（`--no-idle`）、GPU クロック・室温・実行条件
   を記録する。yolox 評価の前例（300 iter×3 round、中央値）に倣い round 条件と round 間分散を残す。

## Recommended Approach

P0 でコーパスと手順を凍結し、P1 で in-family パレートを確定させる。
P1 で knee が見つかれば P2（cross-family）は時間箱で打ち切り可とし、
見つからなければ P2 を本格化する。P3 で既定 engine 切替または現状維持を決定し、
track changelog に記録する。トラックは `core-pipeline`（推論・性能・topology が範囲）。

## Work Plan

- P0 — 手順凍結（0.5 日目安）
  - 対象: `cpp/tools/pose_bench.cpp`、`dump_keypoints.cpp`、`det_bench.cpp`、
    `python/scripts/compare_keypoints.py`、`dump_reference_keypoints.py`。
  - コーパス確定: 既存 `outputs/records/` の9シーンとcorrectness用
    `raw_cam{0,1}.mp4` を固定し、追加収録を待たず一次比較を開始する。
    足元・低照度等の不足条件は結果を見て追加する。multi-person は既定の対象外を維持する。
    `outputs/spatial/` の旧MP4はヘッダー59fpsとmetadata約8fpsが一致しないため、
    レポート間隔を揃える時系列比較へそのまま混ぜない。
    2Dはカメラ別に評価し、録画だけで実機の同期精度やcapture→publish遅延を断定しない。
  - 指標凍結: `rtm_ms`、`det_ms`、`cap→pub`、`recent_pose_fps`、集約 fps、
    max/p95/mean L2、IoU、静止時 jitter（ankle/wrist の frame 間 RMS）、
    低スコア（<0.3）欠損率、足部別集計（big-toe/heel の L2・欠損率・jitter）。
    比較スクリプトの追加集計が要る分だけ拡張する。
  - 検証: 手順書どおりに 1 arm を通しで回し、表の 1 行が出ること。
- P1 — in-family パレート（2–3 日目安、engine ビルド待ち含む）
  - 対象: `cpp/src/infer/rtmpose.*`、`trt_builder.*`、`cpp/tools/build_engines.cpp`。
  - arm（精度基準は同 variant ORT FP32、topology は Halpe26 固定）:
    - RTMPose M / L Halpe26 × FP16（256x192。既存 ONNX を使用）
    - 上の shortlist（パレート近傍）× FP32（ドリフト対照）
    - S-Halpe26 の weight 調達（公式配布の有無を実行時に確認。無ければ S は
      足なしを理由に不採用の一行記録で落とす。COCO17 の S は評価しない）
    - YOLOX tiny-416-FP16 × S-640-FP16（`det_bench` で wall-clock 再確認）
    - 検出間引き sweep（例 5 / 10 / 20。`--det-frequency` の既定 10 を含む）
  - 固定条件: 電源 MAXN_SUPER＋`jetson_clocks`、warmup 50＋300 iter×3 round、
    `--no-idle`、engine profile min/opt/max=1/3/3 維持、単一 context/stream 維持。
  - 検証: 結果表＋散布図を `docs/research/pose-audition-result.md`（新規）に記録。
- P2 — cross-family survey（時間箱 2–3 日、ゲート付き）
  - 候補: ユーザ指名の MediaPipe Pose（BlazePose）。その他の family 追加は
    実行時の一次情報確認後に決める。
  - 受入ゲート: 配布形態からの ONNX 化可否、TRT 10.3 ビルド性、足部点
    （最低 big-toe＋heel、Halpe26 の 20/21/24/25 相当）への写像可能性、
    ライセンス、既存 `Person`/`kp_count` 経路への収まり。
    足なしモデルはゲート不通過とする。BlazePose の詳細（配布形態・公称精度・
    公称速度）は計画時に未確認のため、実行時に公式リポジトリ・公式ドキュメントの
    一次情報を確認してから測る。
  - 対象（見込み）: `cpp/src/infer/` への adapter 追加、`snapshot.cpp`・WebUI payload
    の schema 互換確認。Python 側は参照生成のための wrapper に留める。
  - ゲート不通過の family は結果報告に不通過理由の一行を残して落とす。
  - 検証: 通過分だけ P1 と同一コーパス・同一指標で 1 行ずつ追加。
- P3 — 決定と反映（0.5–1 日目安）
  - 採用動作点（pose engine＋det engine＋det-frequency＋topology＋precision）の指名、
    既定 engine 切替（`docker-compose.yml` の build-engines 既定含む）または現状維持。
  - 影響確認: `ctest --test-dir cpp/build` 全通過、correctness 閾値の再確認、
    実機 WebUI 目視（横向き・遮蔽シーン）、`cap→pub` と 3cam 時 pending の確認。
  - 文書: 結果報告の判断記述、非自明な変更なら `docs/design/core-pipeline-*.md`、
    `docs/tracks/core-pipeline.md` に日付入り changelog。検証戦略に触れれば
    `docs/cpp-migration-plan.md` も更新。
  - ブランチ `core-pipeline/pose-audition`、commit prefix `feat(core-pipeline):` /
    `docs(core-pipeline):`。中間ビルド不能を作らない単位で分割する。

## Validation Plan

- ビルド・回帰: `cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release`、
  `cmake --build cpp/build -j`、`ctest --test-dir cpp/build`（全件）。
  存在確認は `ctest --test-dir cpp/build -N` で先に行う。
- 遅延: `det_bench --frame <frame0> --iters 300 --warmup 50 --engine <each>`（3 round 中央値）、
  `pose_bench` の `rtm_ms` / `recent_pose_fps`、`cap→pub` ログ。
  期待証拠: 同一フレーム・同一電源での arm 間差が round 分散より大きいこと。
- 数値再現性: `dump_reference_keypoints.py --pose-model <variant>.onnx` で variant 別 FP32 基準、
  `dump_keypoints --pose-engine <engine> --video <raw> ` で engine 別ダンプ、
  `compare_keypoints.py` で max/p95/mean L2＋IoU（全身＋足部分割）。
  期待証拠: 高スコア点 L2 < 1.0px（FP16 ドリフトは低スコア除外の上で
  平均 0.3px 台が目安、M2 Step B 実績）。heel は 2D→3D で noisy が既知
  （`tracker_extract.cpp` FootAnchor 経緯）のため heel 単独の worst ではなく
  big-toe/heel の p95・欠損率で読む。
- E2E 実機: 3 カメラ 60fps 構成で `recent_pose ≈ recv`、定常 drop なし、
  `pending` 安定、idle 無効時の復帰不要であること。足元シーンで floor 接触表示と
  toe 方向が破綻しないことを WebUI 目視で確認する。`tegrastats`（GPU%/温度/
  throttle）を添える。ローカル計測のみを実機品質の証拠にしない
  （`AGENTS.md` 検証境界）。
- 最高リスクの検証: FP16 低スコアドリフトの再発チェック。新規 precision・新規
  variant は必ず参照動画の新旧シーン両方で目視＋数値の両輪を通す。

- モデル間品質: 同じ映像で静止・歩行・急動作・足部の破綻を比較する。
  共通擬似教師を用意した場合は生成条件・版・信頼度を残し、教師への一致度を別集計する。
  同 variant ORT FP32 との差だけで、異なるモデルの精度順位を決めない。

## Risks / Rollback

- FP16 ドリフト再発 → FP32 対照 arm が検出網。採用は FP32 に倒すか該当 variant を落とす。
  足部（特に heel）のドリフトは FootAnchor・floor 接触に直結するため、全身点とは別に判定する。
- S-Halpe26 weight が公式配布に無い → S は足なしを理由に落とす。COCO17 S で代替しない。
- 新規 ONNX が TRT 10.3 でビルド不可（TopK/NMS 系の前例あり）→ surgery 可否を
  P2 ゲートで判定し、不可なら一行記録して落とす。既存 engine は残置する。
- 計測の交絡（USB 帯域、MJPEG 破損、電源・温度スロットリング、cam1 ジッタの既知課題）
  → 電源・クロック・室温・round 分散を記録し、怪しい arm は撮り直し。破損フレーム guard
  の drop を drop 率と分けて数える。
- calibration 的偏り（室内・同一被写体）→ 新規シーン追加で緩和し、残留バイアスを
  結果報告に明記する。
- 既定切替の後退 → 旧 engine ファイルと旧既定値を残し、`--pose-engine`/`--det-engine`
  指定で即時切り戻し可能にする。WebUI schema 変更を伴う採用はしない。
- P2 発散 → 時間箱で打ち切り、P1 の knee を暫定採用にする。

## Open Questions

- 遅延と精度のどちらを優先する運用か（VR フルボディ用途で jitter・欠損の許容値は）。
  現回答: パレート指名で吸収する前提。暫定の重み付けがあれば P3 の指名が早くなる。
- Halpe26（足部含む 26 点）は必須か、COCO17 で足りるか。解決済み:
  足部必須（FootAnchor・floor 接触・toe 方向が COCO17 では動かない）につき
  COCO17 は評価しない。Halpe26 への限定はせず、BlazePose 等の足部持ちも P2 候補。
  pose-3d の顔除外原則は不変。
- 複数人運用は単一人物（最大面積 bbox）継続か。現回答: 既定 single-person 継続、
  multi-person は計測対象外。変える場合は P0 で宣言し直す。
- 実機計測の実施主体と枠（Jetson 占有時間、engine ビルド約 7 分/本の待ち）。
  現回答: P1 の engine 本数（Halpe26 M/L の FP16＋FP32 対照で 3–4 本見込み）から逆算して確保する。
- 外部 family の shortlist。MediaPipe Pose（BlazePose）はユーザ指名で P2 候補に確定済み。
  それ以外の追加は実行時の一次情報確認後に決める（計画時はネットワーク不通で未確認）。
