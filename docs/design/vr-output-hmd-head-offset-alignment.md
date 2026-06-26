# vr-output: HMD 顔オフセットを除いた頭軸アライメント

(着手 2026-06-26 / 関連: `archive/phase15-vmt-hmd-auto-align.md`, `tracks/vr-output.md`,
[[project-steamvr-alignment-not-applied]])

## 背景 / 動機

VMT から受け取った HMD ポーズと、カメラ側 3D スケルトンの体ランドマークを対応付けて
VMT↔world の (yaw, x, z) 変換を解くアライメントが 2 経路ある:

- **連続アライメント** (`ContinuousAligner::make_sample`): 毎 tick、HMD xz と
  head_top（信頼度が高い時）/ 胸中点（フォールバック）の xz をペアにし、空間バケット
  リザーバに溜めて `solve_motion`（2D Procrustes）で解く。常時 ON。
- **ワンショット** (`solve_tpose` / motion 収集): ユーザがボタンで起動。chest トラッカー xz と
  HMD xz をペアにする。

いずれも **HMD の生 xz** をそのまま体ランドマークに対応付けていた。しかし HMD は
**顔（目の前）に乗っている**ため、頭/首の鉛直軸から注視方向に ~10cm 前方へずれている。
このレバーアームは頭の向きとともに回るので:

- 頭を回すとオフセットが VMT フレーム内で回転し、一定並進に吸収されない → 解が頭向き分布に
  引っ張られ、残差が乗る。
- 体をほぼ正面固定でもアバター頭が実頭より HMD 前方ぶんずれる（絶対 xz バイアス）。

達成目標: HMD を**頭軸へ射影し直してから**対応付け、このレバーアームを除く。

## 検討した案

- **(採用) HMD 自身の向きから後方ベクトルで射影**。
  `back = R(q)·(0,0,1)`（注視は local −Z なので +Z が頭の後ろ）。
  `head_axis.xz = hmd.xz + d · back.xz`。
  - 数学的根拠: world フレームの head→HMD オフセットを `d·facing_world` とすると、
    HMD 向き `facing` は steamvr フレームで既知なので、解こうとしている回転 R に**依存せず**
    steamvr 側だけで除去できる（`hmd_corrected = T·head_world` がそのまま出る）。
  - ピッチ自動対応: `back` の水平成分 (x,z) は見上げ/見下ろしで自然に縮むので、
    **正規化しない**（真下を見ると水平補正 0）。これが肝。
- **(没) 水平 facing を単位化して固定 d を引く**。見上げ/見下ろしで過補正する
  （水平オフセットは実際にはピッチで縮むのに、常に d を引いてしまう）。
- **(没) ワールド上 (Y) 方向の固定オフセット**。HMD の向きを使わない近似で、頭を回すと
  ずれる。そもそも除きたいのは向き依存のレバーアーム。
- **(没) Y(高さ) も自動補正**。HMD は head_top の下・胸中点の上にあり高さ差は個人差が大きい。
  従来どおり手動スライダ（`alignment.y` は両経路とも 0 のまま）に委ねる。今回は xz のみ。

## 採用設計

### 純ヘルパー (`auto_alignment.{hpp,cpp}`)

```cpp
struct HmdAxisXZ { float x, z; };
HmdAxisXZ hmd_head_axis_xz(const HmdPose& hmd, float forward_offset_m);
// back_x = 2(qx·qz + qw·qy);  back_z = 1 − 2(qx² + qy²)
// return { hmd.x + d·back_x, hmd.z + d·back_z }
```

`d = 0` は無補正（恒等）。steamvr Y-up フレーム前提。

### 適用箇所（3 つ・同一補正）

- `make_sample`: `cfg.hmd_forward_offset_m` で `s.hmd_x/z` を頭軸に置換。
- `solve_tpose`: 引数 `forward_offset_m`（既定 0）で HMD を頭軸に置換してから並進を導出。
- crow motion 収集ワーカ (`crow_server.cpp`): `MotionSample` 構築時に同ヘルパーを適用。

### 二段デフォルト（既存テスト保護）

foot_pos_mode / 胸腰 frac と同じ方式:
- **純ヘルパー/関数の既定は無補正**: `solve_tpose(..., 0.0f)`、`ContinuousAlignerConfig
  ::hmd_forward_offset_m = 0.0f`。既存の `test_auto_alignment` / `test_continuous_aligner`
  はそのまま通る。
- **製品デフォルトは 0.10m**: `MainOptions::vmt_align_hmd_forward_m = 0.10`。
  `output_builder` → `ContinuousAlignerConfig`、`mode_run` → `CrowServer::
  set_align_hmd_forward_m` で両経路へ。

### 設定 / CLI

`vmt.align_hmd_forward_m`（YAML）/ `--vmt-align-hmd-forward F`（CLI）。validate は
`[0, 0.5]`（>0.5m は HMD↔頭距離として非物理、0 = 補正 OFF）。VR 内で詰める想定。

## Milestone

単一コミット（ヘルパー + 3 経路適用 + config 配線 + crow setter + test + docs）。

## 検証

- `ctest -R 'auto_alignment|continuous_aligner|main_config'`:
  `hmd_head_axis_xz`（yaw0=+z 方向 / yaw90=+x 方向 / ピッチ真下=補正 0 / d=0 恒等）、
  offset 付き `solve_tpose`（恒等 chest/HMD → +z= d）、`make_sample`（offset 適用で hmd_z シフト・
  body 不変）、`vmt.align_hmd_forward_m` の YAML/CLI/default/validate + emit 往復。
- 実機: continuous 既定 ON で起動し、頭を左右に振っても水平位置が安定するか、アバター頭が
  実頭に重なるかを確認。`--vmt-align-hmd-forward` を 0.08〜0.15 で振って詰める。

## 残課題

- 高さ (Y) の自動推定は引き続き手動スライダ。HMD↔頭頂/胸の高さ差を推定する別案は保留。
- `d` の自動推定（被写体の頭サイズ・IPD 由来）は未着手。現状は固定 + 手動チューニング。
