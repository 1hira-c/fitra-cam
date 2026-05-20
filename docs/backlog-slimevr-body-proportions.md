# 番外編積みタスク — SlimeVR Body Proportions 推奨値ページ

> **状態**: 未着手。
>
> **着手条件**: Phase 11 の `--slimevr-out` で SlimeVR Server へ VMC tracker を送れるようになった後、SlimeVR 側 IK の体格設定が既定値 / AutoBone 依存のままで、fitra-cam 側の被験者キャリブレーション値と食い違っていると感じたとき。
>
> **調査基準日**: 2026-05-20。SlimeVR の設定名・説明は公式 docs の [Body Proportions Configuration](https://docs.slimevr.dev/server/body-config.html) を確認した。

## 背景

fitra-cam は Phase 8 の `SubjectProfile` で、被験者ごとの IK 用ボーン長を `bone_lengths_m`, `shoulder_width_m`, `hip_width_m`, `subject_height_m` として保存している。Phase 11 ではこの 3D skeleton を SlimeVR Server に VMC/OSC tracker として送るが、SlimeVR Server 側の IK は別途 Body Proportions を持つため、SlimeVR 側の仮想骨格が実寸からズレると knee / waist / chest / elbow tracker の見え方が悪くなる。

SlimeVR 公式 docs では、Body Proportions の手動設定値は cm 単位で、上から順に調整する前提になっている。一方で fitra-cam の計測値は meter 単位の skeleton parent-child 長なので、単純なキー名対応ではなく「測定値」「身長からの推定」「SlimeVR 既定維持」を分けて出す必要がある。

## 目的 / 対象外

| | 範囲 |
|---|---|
| 目的 | WebUI から `/slimevr-proportions` を開くと、現在または指定 subject の `latest_profile.yaml` を読み、SlimeVR の Body Proportions に入力する推奨値を cm で一覧表示する。各値に、fitra-cam 側の参照フィールド、計算式、信頼度、警告を付ける。 |
| 対象外 | SlimeVR の `vrconfig.yml` 直接編集。SlimeVR GUI / SteamVR dashboard の自動操作。SlimeVR AutoBone の置き換え。fitra-cam 側 IK の挙動変更。VMC tracker 位置・姿勢の送信仕様変更。 |

## SlimeVR 項目と fitra-cam 計測値の対応

凡例:

- `measured`: `SubjectProfile` の実測値をそのまま、または左右平均して使う。
- `estimated`: 実測できないため身長や torso 全長から推定する。
- `default`: SlimeVR 側既定または docs 推奨のままにする。
- `manual`: 物理 HMD / controller / avatar 依存で、fitra-cam からは決めない。

| SlimeVR 項目 | 推奨値の出し方 | 信頼度 | メモ |
|---|---|---|---|
| Head Shift | `10.0 cm` を初期表示。ユーザーが HMD 固有値を上書きできる入力欄を置く。 | default | HMD から頭中心までの距離で、fitra-cam のカメラ計測からは直接取れない。 |
| Neck Length | Halpe26 なら参考値として `0.5 * bone_lengths_m[17] * 100`。それが無い場合は `clamp(0.066 * height_cm, 8, 14)`。 | estimated | SlimeVR の定義は頭中心から肩付近まで。Halpe26 の `neck -> head_top` とは定義がズレるので低信頼扱い。 |
| Upper Chest Length | `upper_chest_cm = clamp(0.24 * torso_cm, 12, 20)` | estimated | SlimeVR の胸部 split は fitra-cam では直接観測していない。 |
| Chest Length | `chest_cm = clamp(0.28 * torso_cm, 12, 20)` | estimated | single chest tracker では Upper Chest + Chest の合計が重要、と docs にあるため合計も表示する。 |
| Hip Length | `hip_length_cm = clamp(0.025 * height_cm, 2, 6)` | estimated | docs 上も実験的調整項目。変える場合は Waist Length と合計が変わらないようにする。 |
| Waist Length | `max(20, torso_cm - upper_chest_cm - chest_cm - hip_length_cm)` | estimated | `torso_cm` は Halpe26 なら `bone_lengths_m[18] * 100` (`hip_center -> neck`)、COCO17 なら `avg(bone_lengths_m[5], bone_lengths_m[6]) * 100`。 |
| Upper Leg Length | `avg(bone_lengths_m[13], bone_lengths_m[14]) * 100` | measured | hip to knee。左右差も併記する。 |
| Lower Leg Length | `avg(bone_lengths_m[15], bone_lengths_m[16]) * 100` | measured | knee to ankle。SlimeVR docs の目安範囲を外れたら警告だけ出し、値は丸めず表示する。 |
| Foot Length | Halpe26 なら `avg(bone_lengths_m[20], bone_lengths_m[21]) * 100`。COCO17 なら `5.0 cm`。 | measured/default | Halpe26 は ankle to big toe を使える。COCO17 は足先が無いので SlimeVR docs の feet extension 手順に寄せる。 |
| Hips Width | `hip_width_m * 100` を測定値として表示。推奨値は、測定値が 26-32 cm なら測定値、外れたら `clamp(0.199 * height_cm, 26, 32)`。 | measured/estimated | COCO/Halpe の hip keypoint は SlimeVR docs の「大腿骨間距離」より内側に出やすい。測定値と採用値を分けて表示する。 |
| Hip Offset | `0.0 cm` | default | avatar / application 補正用。 |
| Chest Offset | `0.0 cm` | default | avatar / application 補正用。 |
| Skeleton Offset | `0.0 cm` | default | 全 tracker の前後補正。 |
| Foot Shift | `-5.0 cm` を初期表示し、foot tracker / avatar を見て手動調整。 | default/manual | docs の feet extension 手順に合わせる。 |
| Shoulders Distance | Halpe26 なら `avg(bone_lengths_m[5], bone_lengths_m[6]) * 100` (`neck -> shoulder`)。COCO17 なら `clamp(0.050 * height_cm, 4, 10)`。 | measured/estimated | COCO17 の `bone_lengths_m[5/6]` は torso side なので使ってはいけない。 |
| Shoulders Width | `shoulder_width_m * 100` | measured | left shoulder to right shoulder。 |
| Upper Arm Length | `avg(bone_lengths_m[7], bone_lengths_m[8]) * 100` | measured | shoulder to elbow。 |
| Lower Arm Length | `avg(bone_lengths_m[9], bone_lengths_m[10]) * 100` | measured | elbow to wrist。 |
| Controller Distance Z / Y | 値を出さず「SlimeVR 既定維持」。任意入力欄だけ置く。 | manual | controller 位置と手首回転の問題で、fitra-cam profile からは決めない。 |
| Elbow Offset | `0.0 cm` | default | arm tracking の補正用。 |

## 推奨値の丸めと警告

表示値は SlimeVR GUI で入力しやすいよう `0.1 cm` 単位に丸める。ただし内部 JSON には丸め前の `raw_cm` も載せる。

警告条件:

- `quality_status != pass`: ページ上部に「キャリブレーション品質が warn/fail」と出す。
- `profile.schema` が起動中の `--keypoint-format` と合わない: API は 409 を返し、ページは再キャリブレーション案内を出す。
- 左右差: arms / legs の左右差が平均値の 10% を超える項目は yellow、15% を超える項目は red。
- SlimeVR docs の目安範囲外: 入力自体は妨げず、`outside SlimeVR guide range` を表示する。
- COCO17 profile: foot / neck / shoulders distance は低信頼表示にする。SlimeVR 連携本番は Phase 11 と同じく Halpe26 profile 推奨。

## API

新規 endpoint:

```text
GET /api/slimevr/proportions?subject_id=subject01
GET /api/slimevr/proportions?profile=calibrations/subjects/subject01/latest_profile.yaml
```

優先順位:

1. `profile` query があればその path を読む。
2. `subject_id` query があれば `<subjects_dir>/<subject_id>/latest_profile.yaml` を読む。
3. query が無ければ、起動時の `--subject-profile` または `--subject-id` から読んだ profile を使う。

JSON 例:

```json
{
  "ok": true,
  "units": "cm",
  "subject": {
    "id": "subject01",
    "height_cm": 163.0,
    "schema": "fitra_subject_profile_v1",
    "quality_status": "pass",
    "profile_path": "calibrations/subjects/subject01/latest_profile.yaml"
  },
  "settings": [
    {
      "slimevr_label": "Upper Leg Length",
      "recommended_cm": 37.9,
      "raw_cm": 37.9232447112,
      "source": "measured",
      "fitra_fields": ["bone_lengths_m[13]", "bone_lengths_m[14]"],
      "note": "avg(left hip-knee, right hip-knee)"
    }
  ],
  "warnings": [
    "COCO17 profile has no foot keypoints; Foot Length uses default 5.0 cm"
  ]
}
```

実装上は `cpp/src/slimevr/body_proportions.{hpp,cpp}` のような小さい変換モジュールにして、Crow route と unit test の両方から呼ぶ。OpenCV `FileStorage` で `SubjectProfile` を読む現行実装を再利用する。

## WebUI

新規静的ページ:

```text
web/slimevr_proportions/
├── index.html
├── app.js
└── styles.css
```

Crow route:

- `GET /slimevr-proportions`
- `GET /slimevr-proportions/<path>`

Live UI の header に `slimevr proportions` link を追加する。`/subject-calib` と同じ独立ページ扱いにし、既存 2D/3D canvas の bundle 配信には触らない。

画面要件:

- Subject ID 入力、または profile path 入力。
- `Load` ボタンで API を呼び、SlimeVR 入力順に table 表示。
- 列: `SlimeVR item`, `Recommended cm`, `Source`, `fitra-cam field`, `Warning/Note`。
- `Copy table` ボタンで TSV を clipboard に入れる。
- `measured / estimated / default / manual` の filter。
- 上部に `quality_status`, `schema`, `height`, `profile_path` を表示。
- `Hips Width` のように測定値と採用値が違う項目は、両方見えるようにする。

## 例: `subject01` の現在値

この workspace の `calibrations/subjects/subject01/latest_profile.yaml` は COCO17 profile (`fitra_subject_profile_v1`, height 163.0 cm, pass)。この profile からページが出す代表値は以下。

| SlimeVR item | 推奨値 |
|---|---:|
| Head Shift | 10.0 cm |
| Neck Length | 10.8 cm |
| Upper Chest Length | 12.0 cm |
| Chest Length | 14.0 cm |
| Hip Length | 4.1 cm |
| Waist Length | 20.0 cm |
| Upper Leg Length | 37.9 cm |
| Lower Leg Length | 37.4 cm |
| Foot Length | 5.0 cm |
| Hips Width | 32.0 cm |
| Shoulders Distance | 8.2 cm |
| Shoulders Width | 32.2 cm |
| Upper Arm Length | 25.5 cm |
| Lower Arm Length | 21.0 cm |
| Hip / Chest / Skeleton Offset | 0.0 cm |
| Foot Shift | -5.0 cm |
| Elbow Offset | 0.0 cm |

併記警告:

- `Hips Width`: profile 測定値は 15.2 cm で SlimeVR docs の目安 26-32 cm から外れるため、採用値は身長 prior の 32.0 cm。
- `Lower Leg Length`: profile 測定値 37.4 cm は docs 目安より短い。日本人体格 prior では自然な値なので、SlimeVR GUI が受け付けるならこの値を優先し、avatar 上の ankle / foot tracker の高さを確認する。
- `Foot Length`, `Neck Length`, `Shoulders Distance`: COCO17 profile 由来のため低信頼。SlimeVR 本番用には Halpe26 で subject calibration を取り直す。

## 修正対象ファイル

新規:

- `cpp/src/slimevr/body_proportions.hpp`
- `cpp/src/slimevr/body_proportions.cpp`
- `cpp/tools/test_slimevr_body_proportions.cpp`
- `web/slimevr_proportions/index.html`
- `web/slimevr_proportions/app.js`
- `web/slimevr_proportions/styles.css`

変更:

- `cpp/src/web/crow_server.hpp/.cpp`: route と profile 読み込み設定を追加。
- `cpp/src/main.cpp`: `ServerOptions` に `subjects_dir`, `subject_id`, `subject_profile_path`, `slimevr_proportions_static_dir` を渡す。
- `cpp/src/CMakeLists.txt`, `cpp/tools/CMakeLists.txt`: 新規変換モジュールとテストを build に追加。
- `web/dual_rtmpose/index.html`: header link を追加。
- `docs/phase11-slimevr-integration.md`: 運用メモとして `/slimevr-proportions` を追記。

## 確認項目

1. `cmake --build cpp/build -j`
2. `./cpp/build/tools/test_slimevr_body_proportions`
3. `./cpp/build/main --enable-3d --subject-id subject01 ...` で起動し、`http://JETSON_IP:8000/slimevr-proportions` が開ける。
4. `GET /api/slimevr/proportions?subject_id=subject01` が cm 値と warnings を返す。
5. COCO17 profile では foot / neck / shoulder distance が低信頼になる。
6. Halpe26 profile では `bone_lengths_m[18]`, `[20]`, `[21]`, `[5]`, `[6]` を使った torso / foot / shoulders distance が measured または higher confidence になる。
7. `quality_status=fail` profile はページ表示可能だが、最上部に red warning が出る。
8. `Copy table` の TSV が SlimeVR 入力順で、数値が `0.1 cm` 丸めになっている。

## リスク / メモ

- SlimeVR Server の GUI 名や分割項目はバージョンで変わる可能性がある。着手時に公式 docs と、実際に使う SlimeVR Server バージョンの Body Proportions タブを再確認する。
- fitra-cam の 2D pose keypoint は人体の実際の骨端ではないため、hip width や foot length は実寸から外れやすい。ページでは「測定値を採用する」よりも「測定値・採用値・理由を見せる」ことを優先する。
- Lower leg のように SlimeVR docs の目安範囲と AIST/HQL prior がズレる項目がある。目安範囲外を自動 clamp すると実寸から遠ざかるため、clamp は hips width のように明確に pose keypoint 定義が違う項目に限定する。
- 将来、SlimeVR 側の native config export/import を扱うなら別タスクにする。今回は manual entry helper に閉じる。
