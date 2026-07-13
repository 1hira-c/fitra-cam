# pose-3d: 床接地を利用した足部安定化

(着手日 2026-07-13 / 関連: [locomotion-stability](pose-3d-locomotion-stability.md),
[spatial-filtering](pose-3d-spatial-filtering.md))

## 背景 / 動機

Kalman と VR 出力側 One Euro は時間方向の揺れを抑えるが、接地中の足が床面を数 cm 滑る現象と
床貫通は「床」という既知の空間制約を使わない限り残る。一方、脚チェーン全体を解き直す IK は
膝や腰へ誤差を伝播し、接地の誤判定が姿勢全体を壊す危険がある。

完了条件は次のとおり。

- Halpe26 の接地足で足首の床面 XY 揺れを低減し、sole が床を貫通しない。
- 離地・高速移動時は即座に拘束を外し、左右の足を独立に扱う。
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
                                                └→ TrackerExtractor → VMT / SlimeVR
```

`FloorContactStabilizer` は `MultiCameraDriver` が 1 個所有し、3D loop thread だけが更新する。
subject calibration tap は補正前、`bone_len_drift_pct` は IK 出力に対して計算するため、床補正が
校正値や IK state にフィードバックしない。idle 復帰と `dt > 0.10 s` では履歴をリセットする。
COCO17 は sole 点を持たないため自動 no-op となる。

### 接地状態機械

左右それぞれに contact latch、前回の生 ankle、XY anchor、直前補正、欠損フレーム数を持つ。

- 観測成立: ankle が valid かつ big-toe / small-toe / heel のうち 2 点以上が valid。
- 支持高さ: valid sole の最小 Z と `floor_z_m` の差。
- 接地開始: 前回から速度を計算でき、支持高さ `<= 0.03 m` かつ足首速度 `< 0.25 m/s`。
- 離地: 支持高さ `> 0.06 m`、速度 `> 0.80 m/s`、XY 補正要求 `> 0.03 m`、または
  Z 補正要求 `> 0.08 m` のいずれか。大外れ値は clamp せず fail-open する。
- 欠損猶予: 接地中だけ直前の有界補正を最大 2 フレーム再適用し、3 フレーム目で解放する。
- 非接地時: sole が床下に入った場合だけ、最大 Z 補正内で足部全体を状態レスに持ち上げる。

開始/終了に別しきい値を使うため、境界付近の contact chatter を避ける。最初の有効フレームは
速度履歴の seed のみで、いきなり接地させない。

### 補正

接地中の ankle 生座標を `p`、XY anchor を `a` とし、毎フレーム次で更新する。

```text
alpha = 1 - exp(-dt / 0.25 s)
a     = a + alpha * (p.xy - a)
delta = (a - p.xy, floor_z_m - min(valid sole.z))
```

`delta` を ankle と valid な 3 sole 点へ同一適用する。これは接地足の XY を時定数 0.25 s で
低域通過しながら、最下点を床面へ置く操作である。左右は state を共有しない。

### 設定と観測性

`three_d` の既定値は以下。機能は既定 ON とし、実機回帰時の kill switch を常設する。

| key | 既定 | 意味 |
|---|---:|---|
| `floor_contact_stability` | `true` | 機能の有効/無効 |
| `floor_z_m` | `0.0` | fitra Z-up world の床高 |
| `floor_contact_enter_height_m` / `exit_height_m` | `0.03` / `0.06` | 接地高さヒステリシス |
| `floor_contact_enter_speed_mps` / `exit_speed_mps` | `0.25` / `0.80` | 速度ヒステリシス |
| `floor_contact_xy_tau_s` | `0.25` | 接地 XY anchor の時定数 |
| `floor_contact_max_xy_correction_m` | `0.03` | XY 補正要求の上限 |
| `floor_contact_max_z_correction_m` | `0.08` | Z 補正要求の上限 |
| `floor_contact_missing_grace_frames` | `2` | sole 観測欠損の猶予 |

`/stats3d.stats` と `/ws3d` bundle は左右の contact、補正ノルム、床高を公開する。WebUI 3D viewer は
接地中の ankle 直下へ緑のリングを表示する。`dump_keypoints_3d` は同じ段を既定 ON で通し、
`--no-floor-contact-stability` A/B、接地率、補正 p95、左右および pooled 足首 XY RMS、床下 sole 割合を
summary に出す。

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

単体テストでは左右独立、足部の剛体平行移動、XY jitter 80% 以上減衰、床貫通 clamp、全離地条件、
2 フレーム欠損猶予、長い `dt` / idle 相当 reset、床 offset、COCO17 no-op を固定する。

録画 A/B は静止クリップを同じ engine / calib / frame 数で処理し、
`ankle_xy_rms_m_pooled` が OFF 比 40% 以上低下、ON の `sole_below_floor_fraction == 0`、
`correction_p95_m_* <= hypot(0.03, 0.08) ≈ 0.086 m` であることを目標とする。
歩行クリップでは離地時にリングが消え、
足を引きずらず、補正解除時に snap がないことを確認する。

実機では WebUI と `/stats3d` を見ながら静止、足踏み、歩行、しゃがみ、片足立ちを行い、最後に
VMT / SlimeVR の足 tracker が接地時に安定し離地時に遅延しないことを確認する。ローカル green だけを
既定値確定の根拠にはせず、実機で問題があれば kill switch で即時切り戻す。

### 2026-07-13 録画検証結果

`outputs/records/still` の先頭 240 フレームを同じ calibration / TensorRT engine で A/B した。

| 指標 | OFF | ON | 変化 |
|---|---:|---:|---:|
| ankle XY RMS pooled | 3.410 mm | 1.907 mm | **44.1% 低下** |
| ankle XY RMS left | 4.127 mm | 2.525 mm | 38.8% 低下 |
| ankle XY RMS right | 2.494 mm | 0.946 mm | 62.0% 低下 |
| sole below floor | 17.08% | **0%** | 床貫通解消 |
| contact ratio L / R | — | 99.17% / 99.58% | 静止接地成立 |
| correction p95 L / R | — | 11.5 mm / 14.3 mm | 合成上限内 |

再投影誤差 median は ON/OFF とも 1.90781 px で同一（床段は三角測量へ非帰還）。
`outputs/records/walk_around` 240 フレームでは contact ratio L/R = 37.1% / 31.7%、状態遷移
L/R = 10 / 12 回、両足 air = 80 フレーム、両足 contact = 5 フレームとなり、歩行中の離地を確認した。
補正最大ノルムは L/R = 57.6 / 36.2 mm、床下 sole は 0%。

これは保存録画による数値検証であり、VR出力を含む実機目視は別途必要である。

## 残課題

- 初期しきい値は設計値。異なる床校正・fps・被験者で録画 A/B と実機調整を続ける。
- 接地補正を膝・骨盤まで整合させる脚チェーン IK は、足首/sole だけで不足が確認された場合に別設計する。
- 複数人の接地 state は現行 3D MVP が person 0 のみのため未対応。
