# pose-3d: 床接地を利用した足部安定化

(着手日 2026-07-13 / 関連: [locomotion-stability](pose-3d-locomotion-stability.md),
[spatial-filtering](pose-3d-spatial-filtering.md))

## 背景 / 動機

Kalman と VR 出力側 One Euro は時間方向の揺れを抑えるが、接地中の足が床面を数 cm 滑る現象と
床貫通は「床」という既知の空間制約を使わない限り残る。一方、脚チェーン全体を解き直す IK は
膝や腰へ誤差を伝播し、接地の誤判定が姿勢全体を壊す危険がある。

完了条件は次のとおり。

- Halpe26 の接地足で足首の床面 XY 揺れを低減し、sole が床を貫通しない。
- 離地・高速移動が短い確認時間だけ継続したら拘束を外し、左右の足を独立に扱う。
- IK、subject calibration、骨長、膝位置には副作用を与えない。
- 補正量を有界にし、設定、`/stats3d`、WebUI、オフライン dump から挙動を確認できる。
- 問題時は `--no-floor-contact-stability` または YAML 1 項目で従来経路へ戻せる。

## 検討した案

### sole の Z だけを個別 clamp — 不採用

床貫通は止まるが、足首ベースの VR foot tracker へ効果が届かず、toe/heel の相対形状も変形する。
接地中の XY skating も残る。

### root / 脚チェーンを接地足へ寄せる IK — 保留

物理的には整合するが、両足接地、膝可動域、骨盤移動を同時に解く必要があり、誤接地の影響範囲が
全身へ広がる。今回の「出力を安定化する」目的には過大である。

### ankle だけを固定 — 不採用

VR tracker は安定するが、sole 点との相対位置が変わり WebUI 上で足が伸縮する。

### ankle + sole の有界剛体平行移動 — 採用

接地判定には sole 3 点を使い、補正は ankle と valid な toe/heel へ同じ XYZ 平行移動として適用する。
回転、骨長、膝、骨盤は変えない。効果範囲と失敗時の最大変位を明示的に制限できる。

## 採用設計

### データフローと所有権

```text
triangulate → Kalman → calibration tap / IK → FloorContactStabilizer
                                                ├→ Skeleton3DBus → WebUI
                                                └→ TrackerExtractor
                                                     ├→ 補正前の脚形状で FootAnchor / 向きを更新
                                                     └→ foot 位置だけ補正を復元 → VMT / SlimeVR
```

`FloorContactStabilizer` は `MultiCameraDriver` が 1 個所有し、3D loop thread だけが更新する。
subject calibration tap は補正前、`bone_len_drift_pct` は IK 出力に対して計算するため、床補正が
校正値や IK state にフィードバックしない。VR 抽出時は足部へ適用済みの平行移動を一度除いて
FootAnchor の tibia 長・方向と lower-leg を計算し、左右 foot tracker の位置だけ平行移動を戻す。
idle 復帰と設定可能な更新 gap（既定 `dt > 0.50 s`）では履歴をリセットする。COCO17 は sole 点を
持たないため自動 no-op となる。

### 接地状態機械

左右それぞれに contact latch、前回の生 ankle、XY anchor、直前補正、欠損フレーム数、
離地候補の継続時間を持つ。

- 観測成立: ankle が valid かつ big-toe / small-toe / heel のうち 2 点以上が valid。
- 支持高さ: valid sole の最小 Z と `floor_z_m` の差。ただし、床より 5 mm 以上低く、次点から
  enter band より孤立した最下点は三角測量外れ値として invalid 化し、残る 2 点以上で判定する。
- 接地開始: 前回から速度を計算でき、支持高さ `<= 0.04 m` かつ足首速度 `< 0.35 m/s`。
- 離地候補: 支持高さ `> 0.08 m`、速度 `> 1.00 m/s`、XY 補正要求 `> 0.04 m` のいずれか。
  候補が2回以上かつ `0.05 s` 継続した場合だけ離地へ移り、低fpsでも単発の3D跳ねでは接地を維持する。候補中は新しい
  外れ値からXY補正を作らず、直前の有界補正を保持する。Zだけは床貫通防止範囲へ制約する。
  床下方向の Z 補正要求 `> 0.08 m` は安全上即時解除する。
  離地後は直前補正を時定数 `0.05 s` で 0 へ減衰し、再接地は減衰完了後に許可するため解除時 snap と
  re-enter chatter を避ける。
- 欠損猶予: 接地中だけ直前の有界補正を最大 4 フレーム再適用し、5 フレーム目で解放する。
- 非接地時: sole が床下に入った場合は最大 Z 補正まで必ず持ち上げ、8 cm を超える深い貫通も
  fail-open にしない。離地減衰中も最終支持点が床下へ入らない範囲へ Z 補正を戻す。

開始/終了に別しきい値を使うため、境界付近の contact chatter を避ける。最初の有効フレームは
速度履歴の seed のみで、いきなり接地させない。

### 補正

接地中の ankle 生座標を `p`、XY anchor を `a` とし、毎フレーム次で更新する。

```text
alpha = 1 - exp(-dt / 0.25 s)
a     = a + alpha * (p.xy - a)
delta_z = clamp(floor_z_m - robust_min(valid sole.z), -max_z, +max_z)
delta   = (a - p.xy, delta_z)
```

`delta` を ankle と valid な 3 sole 点へ同一適用する。これは接地足の XY を時定数 0.25 s で
低域通過しながら、外れ値除外後の最下点をZ補正上限内で床面へ近づける操作である。離地高さを
Z補正上限より広く設定しても、接地中を含む全経路でZ補正の絶対値上限は維持する。左右は state を共有しない。
接地解除時は `delta_release = delta_prev * exp(-dt / release_tau)` として連続的に 0 へ戻す。

### 設定と観測性

`three_d` の既定値は以下。機能は既定 ON とし、実機回帰時の kill switch を常設する。

| key | 既定 | 意味 |
|---|---:|---|
| `floor_contact_stability` | `true` | 機能の有効/無効 |
| `floor_z_m` | `0.0` | fitra Z-up world の床高 |
| `floor_contact_enter_height_m` / `exit_height_m` | `0.04` / `0.08` | 接地高さヒステリシス |
| `floor_contact_enter_speed_mps` / `exit_speed_mps` | `0.35` / `1.00` | 速度ヒステリシス |
| `floor_contact_xy_tau_s` | `0.25` | 接地 XY anchor の時定数 |
| `floor_contact_max_xy_correction_m` | `0.04` | XY 補正要求の上限 |
| `floor_contact_max_z_correction_m` | `0.08` | Z補正の絶対値上限 |
| `floor_contact_missing_grace_frames` | `4` | sole 観測欠損の猶予 |
| `floor_contact_reset_gap_s` | `0.50` | 更新不連続として履歴を捨てる gap |
| `floor_contact_release_tau_s` | `0.05` | 離地時補正減衰の時定数 |
| `floor_contact_exit_grace_s` | `0.05` | 高さ・速度・XY離地候補の継続確認時間 |

`/stats3d.stats` と `/ws3d` bundle は左右の contact、当該フレームの sole evidence、fresh/stale、
実際に適用した補正ノルム、床高を公開する。sync miss / idle snapshot は最後の contact を保持したまま
`floor_contact_fresh=false` とし、「未更新」を「両足 air」と誤報しない。WebUI 3D viewer は短い stale 区間で
接地リングを保持し、stats は `plant / grace / air` を分ける。`dump_keypoints_3d` は同じ段を既定 ON で通し、
`--fps` で入力 cadence を明示上書きできる。接地率の分母は各側の `evidence_valid` フレームだけとし、
grace を分子に含めず 1.0 を超えない。A/B summary には補正 p95、左右および pooled 足首 XY RMS、
床下 sole 割合、実効 input fps、左右の observation frame 数を出す。

## Milestone

- **M1**: 独立 `FloorContactStabilizer` と決定的単体テスト。
- **M2**: MainOptions / YAML / CLI、ライブ pipeline、offline dump への配線。
- **M3**: `/stats3d`、WebUI 接地リング、A/B summary 指標。
- **M4**: track / 検証戦略の更新とローカル回帰検証。

## 検証

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build -j
ctest --test-dir cpp/build --output-on-failure
pnpm -C web-ui build
```

単体テストでは左右独立、足部の剛体平行移動、XY jitter 80% 以上減衰、孤立 sole 外れ値除外、
8 cm 超の床貫通への有界 fail-safe、単発離地候補を吸収するサンプル数+時間 grace、継続離地と補正の単調減衰、
欠損 grace と実適用値の分離、8 fps 相当を維持する既定 reset gap、設定した長い `dt` / idle reset、
VR FootAnchor の補正前脚長、床 offset、COCO17 no-op を固定する。

録画 A/B は静止クリップを同じ engine / calib / frame 数で処理し、
`ankle_xy_rms_m_pooled` が OFF 比 40% 以上低下、ON の `sole_below_floor_fraction == 0`、
`correction_p95_m_* <= hypot(0.04, 0.08) ≈ 0.089 m` であることを目標とする。
歩行クリップでは離地時にリングが消え、
足を引きずらず、補正解除時に snap がないことを確認する。

実機では WebUI と `/stats3d` を見ながら静止、足踏み、歩行、しゃがみ、片足立ちを行い、最後に
VMT / SlimeVR の足 tracker が接地時に安定し離地時に遅延しないことを確認する。ローカル green だけを
既定値確定の根拠にはせず、実機で問題があれば kill switch で即時切り戻す。

### 2026-07-15 接地判定緩和後の録画検証結果

2026-07-14 と同じ engine / calibration / 先頭240フレームを使った。静止録画では pooled ankle XY RMS が
OFF 3.390mm に対して ON 1.903mm（43.9%低下）、sole床下率0%、接地率 L/R = 99.58% / 99.58%、
補正 p95 L/R = 11.4 / 14.3mmで、緩和前の安定性と床制約を維持した。

歩行録画では接地率 L/R = 47.1% / 48.8%（緩和前 29.6% / 22.5%）、状態遷移 L/R = 8 / 6回、
両足 air = 30フレーム、両足 contact = 20フレームだった。補正 p95 L/R = 41.2 / 37.6mmで合成上限内、
sole床下率は0%。接地を拾う範囲を広げながら、保存録画上は離地遷移と床非貫通を維持した。
これは実機フィードバックを反映したオフライン再確認であり、VMT / SlimeVRを含む再目視は別途必要である。

### 2026-07-14 レビュー修正後の録画検証結果

`outputs/records/still` の先頭 240 フレームを同じ calibration / TensorRT engine で A/B した。
録画 metadata の実測値に合わせて `--fps 58.81` を明示した。

| 指標 | OFF | ON | 変化 |
|---|---:|---:|---:|
| ankle XY RMS pooled | 3.390 mm | 1.901 mm | **43.9% 低下** |
| ankle XY RMS left | 4.097 mm | 2.517 mm | 38.6% 低下 |
| ankle XY RMS right | 2.489 mm | 0.946 mm | 62.0% 低下 |
| sole below floor | 17.15% | **0%** | 床貫通解消 |
| contact ratio L / R | — | 99.17% / 99.58% | 静止接地成立 |
| correction p95 L / R | — | 11.5 mm / 14.3 mm | 合成上限内 |

再投影誤差 median は ON/OFF とも 1.90781 px で同一（床段は三角測量へ非帰還）。
`outputs/records/walk_around` は `--fps 58.72` で 240 フレーム処理し、観測成立フレームに対する
contact ratio L/R = 29.6% / 22.5%、状態遷移 L/R = 10 / 8 回、両足 air = 117 フレーム、
両足 contact = 2 フレームとなり、歩行中の離地を確認した。補正 p95 L/R = 30.0 / 30.9 mm、
床下 valid sole は 0%。右足の孤立した床下外れ値は 14 フレームで invalid 化され、足全体の持ち上げへ
波及しなかった。

これは保存録画による数値検証であり、VR出力を含む実機目視は別途必要である。

## 残課題

- 初期しきい値は設計値。異なる床校正・fps・被験者で録画 A/B と実機調整を続ける。
- 接地補正を膝・骨盤まで整合させる脚チェーン IK は、足首/sole だけで不足が確認された場合に別設計する。
- 複数人の接地 state は現行 3D MVP が person 0 のみのため未対応。
