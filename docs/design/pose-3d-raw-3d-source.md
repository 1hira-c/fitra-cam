# pose-3d: raw 3D source mode

着手日: 2026-08-07
関連: [Issue #59](https://github.com/1hira-c/fitra-cam/issues/59)

## 背景 / 動機

`/ws3d` を下流で独自に平滑化・拘束・融合する consumer には、三角測量直後の
`Skeleton3D` が必要になる。従来の live 経路は次の順に出力を加工していた。

```text
triangulate -> SkeletonKalman -> IkSolver -> FloorContactStabilizer -> Skeleton3DBus /ws3d
```

個別の `--no-3d-kalman`、`--no-3d-ik`、`--no-floor-contact-stability`
だけを組み合わせると、利用側が一つ忘れたときに生の観測と異なる出力になる。また
`ik_locked` は solver/profile の lock 状態であって、このフレームで IK を適用したかを
表す値ではない。

## 検討した案

### A. 個別 kill switch の組み合わせを利用する

不採用。3 つを常に正しく同期させる必要があり、設定の読み書きや実行中の診断で
「どの stage が実際に動いたか」を一意に表せない。

### B. 通常 `/ws3d` を維持し、別 WebSocket に raw skeleton を追加する

不採用。consumer ごとに二重の bus / lifecycle を持ち、同じ入力についてどちらを
読めばよいか曖昧になる。Issue #59 は既存 `/ws3d` を opt-in で raw source に切り替える
契約を要求している。

### C. 個別 stage 値を raw mode 有効時に false へ書き換える

不採用。保存した YAML が通常 mode の調整値を失い、raw mode を解除した後に意図しない
設定になる。設定値と実効値を分ける必要がある。

## 採用設計

### 設定と優先順位

- CLI: `--no-3d-postprocess`
- YAML: `three_d.no_3d_postprocess: true`
- 既定: `false`。従来どおり Kalman / IK / floor stabilization が有効。

`no_3d_postprocess` が true の場合は、個別 stage が true でも raw mode が優先される。
ただし `kalman_3d` / `ik_3d` / `floor_contact_stability` の設定値自体は変更しない。
emit/load round-trip は個別設定と umbrella flag の両方を保存する。raw mode を false に
戻せば、保存していた個別設定が再びそのまま有効になる。

適用範囲は `RunMode::Run` に限る。flow daemon の共有 YAML にこの設定があっても、
`CalibSubject` child は umbrella flag を無効化する。subject calibration の pose-hold 判定は
測定 skeleton と post-IK drift の組を前提にするためであり、個別 stage の既存設定値は変更しない。

### 実行経路と不変条件

`MultiCameraDriver::ThreeDConfig::effective_stages()` を唯一の実効 stage 判定にする。

```text
normal:
  triangulate -> optional Kalman -> optional IK -> optional floor -> /ws3d

raw:
  triangulate -----------------------------------------------> /ws3d
```

raw mode では `SkeletonKalman::update`、`IkSolver::update`、
`FloorContactStabilizer::update` を呼ばない。従って観測から個別 stage の履歴を
更新しない。Triangulator 内の既存の座標系、reprojection gate、Halpe26 の派生 joint は
triangulation stage の契約として残る。

subject profile は任意入力のままであり、profile が無くても raw `/ws3d` は配信する。
`ik_locked` は従来どおり「solver/profile が lock されているか」を表す。raw mode によって
値の意味を変えず、IK 適用の判定には使わない。

VMT の通常 source は従来どおり `ik_locked` を readiness gate にする。一方 raw mode は
IK を意図的に bypass するので、`stats.raw_3d_source=true` を VMT readiness とし、profile /
IK lock が無くても tracker extractor の共有 source を送信できる。これは raw mode を opt-in
した場合だけであり、VMT 側の tracker extraction / smoothing や通常 mode の品質 gate は変えない。

### 診断と互換性

`/ws3d` の `stats` に後方互換な次の field を追加する。

```json
{
  "raw_3d_source": true,
  "kalman_enabled": false,
  "ik_enabled": false,
  "floor_stability_enabled": false
}
```

これらは設定希望ではなく実効 stage を表し、sync miss / idle の空 snapshot にも載せる。
WebUI は `postprocess` 行で同じ状態を表示する。既存 field、joint 配列、VMT の既定経路は
変更しない。raw mode は opt-in であり、VMT を有効にした場合はその共有 bus の raw skeleton を
tracker extractor も読むため、通常の VR 出力品質を期待する用途では既定 mode を使う。

## Milestone

- **M1: config contract** — CLI/YAML/emit-load と help を追加する。
- **M2: stage bypass** — `effective_stages()` を pipeline の唯一の gate にし、raw mode が
  mutable postprocess state を更新しないようにする。
- **M3: telemetry / regression** — `/ws3d.stats` と WebUI 表示、config・stage selection・
  JSON serialization 回帰テストを追加する。

## 検証

- `ctest -N` で `test_main_config`、`test_raw_3d_source`、
  `test_snapshot_floor_stats`、`test_vmt_protocol` を確認し、focused test と full `ctest` を実行する。
- `./cpp/build/main --help` に `--no-3d-postprocess` が出ることを確認する。
- 実機では同一の calibration / input で raw `/ws3d.persons_3d` と
  `dump_keypoints_3d --no-kalman --no-ik --no-floor-contact-stability` を比較する。
  position / valid / score が許容差内で一致し、`stats` の実効 stage がすべて false であることを
  確認する。raw + VMT は profile 未指定でも bundle が送信されることを別に確認し、通常 mode の
  `ik_locked` gate と VRChat 挙動を回帰させない。subject calibration は同じ YAML でも raw flag が
  無効化され、通常の pose-hold drift gate で開始できることを確認する。

## 残課題

- #60 の joint provenance / age 契約が着地したら、raw mode でも observation state と
  supporting view 情報を additive に公開する。
