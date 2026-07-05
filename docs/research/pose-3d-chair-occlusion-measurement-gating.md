# pose-3d: chair occlusion 向け prediction-aware measurement gating

(作成日 2026-07-05 / 状態: 未実装・別 issue 候補)

## 背景

着座クリップ `outputs/records/chair_occluded/` で、椅子の陰に体が隠れた状態だと、半端に見えている
カメラの 2D keypoint を 3D lift が拾い、WebUI 上の体が揺れる現象を確認した。

これは「画像端で見切れる」問題ではない。被写体はフレーム内にいるが、椅子で体の一部が遮蔽される。
RTMPose は遮蔽された右半身 keypoint を **高 score のまま hallucination** し、triangulator の重み付き DLT に
混ざる。結果として初期 3D 点が悪い view に引かれ、その後の reprojection gate だけでは外せない。

観測メモ:

- clip: `outputs/records/chair_occluded/`、3 cam MJPEG、`fps_written` 58.82、1185 frames。
- RAW triangulation では右側 joint (`r_sho` / `r_elb` / `r_hip` / `r_ank` / `r_toe` / `r_heel`) の
  median reproj が概ね 9-13 px。
- 2D score は 0.62-0.79 程度で、きれいな左側 joint と同程度。score gate では落ちない。
- `--max-reproj-px` を 6 から 3.5 に締めても visible jitter は改善しなかった。

## 失敗した案

### 1. frame-edge / bbox truncation gate

今回の問題は「フレーム端に貼り付いた keypoint」ではなく、椅子による画面内 occlusion。edge margin や
bbox border 接触では主因を拾えない。別 failure mode には効く可能性があるが、この着座問題の本筋ではない。

### 2. score gate

遮蔽 hallucination の score が高い。きれいな view と score 分布が重なるため、score を締めると正しい
joint も落とし、悪い joint だけを狙い撃ちできない。

### 3. `max_reproj_px` の単純 tightening

triangulator はまず全 view の score-weighted DLT で点を作る。高 score の悪い view が混ざると初期解自体が
そちらへ引かれるため、後段 reprojection check は「壊れた点への整合性」を測ってしまう。閾値を締めても
原因 view を安定して外せなかった。

### 4. stateless pair-consensus / pair RANSAC

2-view pair を列挙して低 reproj の pair を選ぶ案を試した。`chair_occluded` では reproj は大きく改善した
一方、pair の切り替わりと 2-view ノイズが出力へ出て、visible jitter は悪化した。

実測例(ON vs OFF):

- `r_sho`: reproj 9.6 -> 0.5 px、adjacent-frame jitter +36%。
- `r_elb`: reproj 13.3 -> 2.7 px、adjacent-frame jitter +79%。
- `r_hip`: reproj 9.6 -> 1.0 px、adjacent-frame jitter +32%。

低 reproj は「見た目の安定」と同義ではない。stateless に view set を選ぶだけでは、時間方向の一貫性を壊す。

### 5. high-reproj joint invalidation

通常 triangulation 後に mean reproj が高い joint を invalid にする案も試したが、右側 joint の valid rate が
大きく崩れた。Kalman/IK は JSON 出力上でそれを自動補完しないため、dropout risk が高い。

例: `r_elb` 17.4%、`r_hip` 16.6%、`r_ank` 20.8%、toe/heel 約 26.8% まで valid が低下。

## 次 issue の方向

triangulator 単体で stateless に直すのではなく、**prediction-aware measurement gating** として扱う。

基本方針:

- triangulator は当面 stateless のままにする。
- Kalman prediction、前回 accepted skeleton、IK / bone length から「今回あり得る測定位置」を持つ。
- 測定 joint が high reproj かつ prediction から大きく外れる場合は、その joint の measurement update を棄却する。
- 棄却時は joint を invalid にして downstream へ落とすのではなく、予測値または last-good を valid な出力として保持する。
- gate 結果、棄却率、prediction distance、reproj を telemetry / dump に出し、実機で原因を見えるようにする。

設計で決めること:

- gate の所有者: Kalman 内部に measurement quality を渡すか、Kalman 前に stateful gate を置くか。
- reset 条件: idle resume / subject profile 切替 / tracking target switch。
- joint ごとの閾値: trunk / arm / leg / foot で許容速度と prediction distance を分けるか。
- IK との順序: 棄却後に IK で補うか、IK 後の整合性で棄却するか。
- `dump_keypoints_3d` で再現可能な offline A/B と live stats の出し方。

## 合格基準

- `chair_occluded` で右半身の adjacent-frame jitter / visible jitter が下がる。
- valid rate を高-reproj invalidation 案のように崩さない。
- clean clip (`still` / `walk_around` / `kick` / `dash`) で jitter・lag・valid rate の有意な退行がない。
- synthetic ctest で「confidently wrong measurement を reject し、last-good/prediction が continuity を保つ」ことを固定する。

## Issue draft

Title:

`feat(pose-3d): chair occlusion に強い prediction-aware measurement gating`

Body:

椅子で体が遮蔽された着座 clip で、半端に見えているカメラの高 score hallucinated keypoint を 3D lift が拾い、
右半身が揺れる。frame edge / bbox truncation / score gate / reproj tightening では原因 view を安定して外せない。
stateless pair-consensus は reproj を改善するが visible jitter を悪化させ、高-reproj invalidation は valid rate を崩す。

triangulator 単体の stateless 対策ではなく、Kalman prediction / last-good skeleton / IK 制約を使った
measurement gate として設計する。bad measurement は downstream dropout にせず、prediction または last-good を
valid 出力として保持する。`chair_occluded` と clean clips で A/B し、jitter 改善・valid rate 維持・clean 非退行を
合格基準にする。
