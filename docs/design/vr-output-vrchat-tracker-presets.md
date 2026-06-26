# vr-output: VRChat 向けトラッカープリセット + 足位置モード

(着手 2026-06-24 / 関連: `archive/phase14-vmt-steamvr.md`, `tracks/vr-output.md`)

## 背景 / 動機

VMT publisher は従来 `kTrackerCount = 10` 本すべて (`LeftUpperArm` / `RightUpperArm` /
`Chest` / `Waist` / `LeftUpperLeg` / `RightUpperLeg` / `LeftLowerLeg` / `RightLowerLeg` /
`LeftFoot` / `RightFoot`) を `/VMT/Room/Driver` で送っていた。これは SlimeVR の
`TrackerPosition` enum に合わせた構成だが、**VRChat FBT が扱えるトラッカーは最大 8 点**
(hip / chest / 両 feet / 両 knees / 両 elbows) で、`LeftLowerLeg` / `RightLowerLeg` (脛) には
VRChat 上の対応 role が存在しない (phase14 でも「未割当・送るだけ」と既述)。結果として
VRChat 側のトラッカー数・位置と食い違い、IK の挙動が読みにくかった。

達成目標:
- VMT 送信を **VRChat 標準 8 点に既定で一致**させる (脛 2 本を落とす)。
- VRChat 公式が「トラッカーは少ない方が IK が安定する場合あり」と明記しているため、
  **3 / 6 / 8 / full のプリセットを CLI / Web UI で切替可能**にする。
- 足トラッカー位置の仮説 (「足首位置 + 足先方向の回転」が VRChat 的に正しいかも) を
  **A/B 検証できるトグル**にする。

出力経路は VMT→SteamVR を維持 (VRChat OSC 直送への移行はしない)。

## 検討した案

- **(採用) VMT publisher 側で role マスク**。extractor は 10 点を維持し、VMT 送信時だけ
  preset で role を間引く。SlimeVR Firmware UDP 路 (回転のみ・脛も IK に有用) を壊さず、
  位置を送る VMT/VRChat だけ本数を絞れる。`TrackerExtractor` / `SlimeTrackerBus` は単一
  producer のまま。
- **(没) extractor 側で本数を削る**。SlimeVR 路まで 8 点に巻き込まれ、脛トラッカーの
  IK 寄与を失う。出力ごとに要求が違うのに producer を片方に最適化するのは筋が悪い。
- **(没) VMT index を詰め直す** (preset ごとに 0,1,2… と連番)。SteamVR「Manage Trackers」の
  role 割当が preset 変更のたびに崩れる。→ **index は role 固定** (`vmt_index_for` 不変)、
  間引いた role は index 欠番にするだけ。VMT_10=Left Elbow… の対応が preset 間で安定。
- **足位置**: 既定を `ankle` に変更しつつ `midpoint` (従来) を残すトグル。回転は両モードとも
  不変 (`fwd = ankle→toe`) なので足先方向は常に保持。`extract_trackers` の**関数デフォルトは
  Midpoint** (既存 golden test を保護)、**ランタイム既定は Ankle** (`TrackerExtractorOptions`)。

## 採用設計

### プリセット (送信する TrackerRole 集合)

| preset | role | VMT index (base=10) |
|---|---|---|
| `p3` | Waist, LeftFoot, RightFoot | 13, 18, 19 |
| `p6` | p3 + Chest, LeftUpperLeg, RightUpperLeg | +12, 14, 15 |
| `p8` (既定) | p6 + LeftUpperArm, RightUpperArm | +10, 11 |
| `full` | 全 10 (LeftLowerLeg 16, RightLowerLeg 17 を追加) | 10..19 |

ネストした上位集合 (`full` のみ脛を追加)。`vmt::role_mask_for(preset)` が
`std::array<bool, kTrackerCount>` を返す (`vmt_protocol.cpp`)。

### データフロー / 所有権

- `vmt_protocol.{hpp,cpp}`: `enum VmtTrackerPreset`, `parse_vmt_preset` / `vmt_preset_name`,
  `role_mask_for`。
- `VmtPublisher`: `opts.preset` から `preset_` + `role_enabled_` を構築。`set_preset()` /
  `preset()` は `alignment_` と同じ mutex パターンでランタイム切替可。`send_loop` は毎 tick
  マスクをスナップショットし `if (!mask[i]) continue;` で間引く。
- `MainOptions.vmt_tracker_preset` (既定 `"p8"`): YAML `vmt.preset` / CLI `--vmt-preset` /
  validation (`p3|p6|p8|full`)。`output_builder` が `parse_vmt_preset` → `vopts.preset`。
- Web: `GET/POST /api/vmt/preset` (crow)、`/stats3d` の `vmt.preset`、`VmtPresetForm.tsx`。

### 足位置モード (`FootPosMode`)

- `extract_trackers(skel, ctx, FootPosMode)`: `pos = (Ankle) ? ankle : midpoint(ankle, toe)`。
  `fwd`/`up` は不変。位置を消費するのは VMT 送信と WebUI viz のみ (SlimeVR は回転のみ)。
  FootAnchor 再アンカーは ankle/toe を直接使うため影響なし。
- `TrackerExtractorOptions.foot_pos_mode` (既定 `Ankle`) ← `MainOptions.vr_foot_pos_mode`
  (`three_d.vr_foot_pos_mode` / CLI `--foot-tracker-pos {ankle,midpoint}`)。

### 胸 / 腰トラッカーの高さ (脊椎沿い・追加 2026-06-26)

従来 Chest = `midpoint(neck, hip_center)`、Waist (=SteamVR Hip) = `hip_center` 固定で、実機で
両者がやや低く感じられた。両位置を **脊椎方向に沿って引き上げ可能**にする
(`pos = hip_center + frac · (neck − hip_center)`, `frac ∈ [0, 1]`, 0=hip_center / 1=neck)。

- **方向は脊椎沿い (ワールド上方向ではない)**。前傾時もトラッカーが胴体に乗ったままになる。
  ワールド Y 上だと leaning で体から外れる。
- **位置のみ。回転 (forward/up) は不変** → 影響は VMT 送信 + WebUI viz のみ、回転だけの
  SlimeVR Firmware UDP 路は完全に同一。foot 位置モードと同じ「位置だけの調整」分類。
- **関数デフォルト = 歴史的配置** (`extract_trackers` の chest=0.5 / waist=0.0) で既存 golden
  test (`Chest pos == midpoint`, `Waist pos == hip_center`) を保護。**製品デフォルト = 引き上げ
  済み** (`TrackerExtractorOptions` chest=`0.65` / waist=`0.15`) で胸郭中央〜ベルトライン寄りに。
  foot_pos_mode と同じ二段デフォルト方式。
- `MainOptions.vr_chest_height_frac` / `vr_waist_height_frac` (`three_d.*` / CLI
  `--chest-height-frac` `--waist-height-frac`、validate `[0,1]`)。`output_builder` →
  `TrackerExtractorOptions`。hip-relative 位置ホールドの基準は引き続き生 `hip_center` (joint 19)
  のまま (Waist トラッカーは同 joint から `frac` 分だけ上にオフセット)。
- **検討した単位**: ① 脊椎長に対する割合 (採用) — 被写体の体格 (身長) が変わっても自動追従。
  ② cm 固定オフセット (没) — 直感的だが体格差に追従せず、マルチ被写体運用で破綻。
- ランタイム調整 (Web UI/API) は今回見送り (YAML/CLI で再起動反映)。必要になれば preset/alignment
  と同じ mutex パターンで後付け可能。

### SteamVR「Manage Trackers」role 割当 (運用)

VMT_10→Left Elbow / 11→Right Elbow / 12→Chest / 13→Waist / 14→Left Knee / 15→Right Knee /
16,17→(full のみ・未割当) / 18→Left Foot / 19→Right Foot。preset を変えても index は不変。

## Milestone

単一コミット (publisher マスク + foot トグル + config 配線 + web + test + docs)。中間状態が
ビルド不能になる分割を避けるためまとめる。

## 検証

- `ctest -R 'vmt_protocol|main_config|tracker_extract'`: `role_mask_for` の各 preset 集合
  (count 3/6/8/10, p8 に脛なし, ネスト) と parse 往復、`vmt.preset` / `vr_foot_pos_mode` /
  `vr_chest_height_frac` / `vr_waist_height_frac` の YAML/CLI/validate。`tracker_extract` に
  胸/腰の高さ frac テスト (frac 0.5/0.0=歴史的配置, 0.65/0.15=引き上げ, 回転は frac 非依存)。
- 起動ログ `{N} trackers (preset=…)`、`/stats3d` の `vmt.preset` と `sent_trackers`/tick。
- Windows 実機: Manage Trackers で上表 role 割当 → VRChat FBT calibration を p8/p6/p3 で
  比較。`--foot-tracker-pos ankle|midpoint` を切替え、足の接地・向きが自然な方を採用
  (仮説が良ければ既定維持、ダメなら既定を midpoint に戻す)。
