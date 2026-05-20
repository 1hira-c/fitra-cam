# Phase 9 — COCO17 → Halpe26 キーポイント移行

## Summary

- Phase 7 (3D lift) / Phase 8 (subject profile + pose calibration) は全て RTMPose の **COCO 17 キーポイント**前提でハードコードされている。
- Phase 9 では **Halpe26 (26 キーポイント)** にアップグレードする。Halpe26 は **0–16 が COCO17 と同一順序**で 17–25 に追加 9 点 (head_top, neck, hip-center, big_toe×2, small_toe×2, heel×2) が付く上位互換であり、既存 IK / pose_recognizer のハードコード索引 (5/6/7/8/11/12/13/14/15/16) はそのまま使える。
- `--keypoint-format {coco17,halpe26}` で **CLI 切替**。既定は `coco17` で出荷し、回帰検証後に `halpe26` を既定に昇格する。
- 追加 9 点のうち **hip-center(19)** と **neck(18)** は Phase 8 の pose_recognizer が計算している「肩中点／腰中点」を直接置換でき、midpoint 由来の誤差を除去できる。足部 (heel/toe) は Phase 9 では描画と JSON 出力のみ。歩行解析等は Phase 10+。

## 設計判断

| 項目 | 決定 |
|---|---|
| モデル | Halpe26 RTMPose ONNX (現行 repo の検証済み artifact は `outputs/onnx/rtmpose-m_simcc-body7_pt-body7-halpe26_700e-256x192-4d3e73dd_20230605.onnx`)。 |
| 併存方式 | `--keypoint-format {coco17,halpe26}` で CLI 切替。既定 `coco17`。 |
| 追加 9 点 | **選択的活用**。pose_recognizer / IK で hip-center(19) と neck(18) を直接使用。head_top / 足部はトポロジー拡張 (描画+JSON) のみ。 |
| Subject profile 互換 | schema 文字列を `fitra_subject_profile_v1` → `v2` に bump。v1 を halpe26 モードで読込 → 明示エラーで拒否 (再キャリブ案内)。マイグレーションは行わない。 |

## 設計の肝

`infer::kNumKeypoints = 17` (`cpp/src/infer/types.hpp:24`) がコンパイル時定数で、`std::array<Keypoint, kNumKeypoints>` が `Person` / `Skeleton3D` / `subject_profile::bone_lengths_m` 等の配列サイズに直結している。CLI 切替を成立させる方針:

- 定数を **`constexpr std::size_t kMaxKeypoints = 26;`** に置換。配列容量を 26 に統一する。
- 論理的な有効点数 (17 / 26) は実行時値で扱う。プロセス内で 1 度だけ設定する `keypoint_format::set_active()` のシングルトンと、`Person::kp_count` / `Skeleton3D::kp_count` の 1 バイトフィールドで反復上限を伝搬する。
- COCO17 動作時は kpts[17..25] がゼロ初期化のまま無視される。snapshot/drawer/web のループは `kp_count` で打ち切る (`kpts.size()` ベースの暗黙反復を全て論理点数ベースに修正)。

## トポロジー定義

新規ヘッダ `cpp/src/lift/keypoint_format.hpp` に enum と SkeletonDef を定義:

```cpp
enum class KeypointFormat { Coco17, Halpe26 };

struct SkeletonDef {
    KeypointFormat                       format;
    std::size_t                          kp_count;
    std::span<const std::pair<int,int>>  edges;
    std::span<const int>                 parents;             // -1 = root
    std::span<const int>                 hinge_joints;
    std::span<const int>                 major_bone_children;
    int (*hinge_child)(int);
};
const SkeletonDef& skeleton_def(KeypointFormat fmt);   // static instances
void set_active_keypoint_format(KeypointFormat);       // CLI 起動時のみ
KeypointFormat active_keypoint_format();
```

### Halpe26 parent tree (root = hip-center 19)

| child | parent | child | parent | child | parent |
|---|---|---|---|---|---|
| 0 nose | 18 neck | 9 l_wrist | 7 | 18 neck | 19 hip-center |
| 1 l_eye | 0 | 10 r_wrist | 8 | 19 hip-center | -1 |
| 2 r_eye | 0 | 11 l_hip | 19 | 20 l_big_toe | 15 |
| 3 l_ear | 1 | 12 r_hip | 19 | 21 r_big_toe | 16 |
| 4 r_ear | 2 | 13 l_knee | 11 | 22 l_small_toe | 15 |
| 5 l_shoulder | 18 | 14 r_knee | 12 | 23 r_small_toe | 16 |
| 6 r_shoulder | 18 | 15 l_ankle | 13 | 24 l_heel | 15 |
| 7 l_elbow | 5 | 16 r_ankle | 14 | 25 r_heel | 16 |
| 8 r_elbow | 6 | 17 head_top | 18 | | |

### Halpe26 edges (AlphaPose canonical)

- 頭: (17,18) (0,17) (0,1) (0,2) (1,3) (2,4)
- 体幹: (18,5) (18,6) (18,19) (11,19) (12,19) (5,6) (11,12)
- 腕: (5,7) (7,9) (6,8) (8,10)
- 脚: (11,13) (13,15) (12,14) (14,16)
- 足: (15,20) (15,22) (15,24) (16,21) (16,23) (16,25)

### 不変な要素

- `kHingeJoints = {7,8,13,14}` (肘・膝) は COCO17 と同じ索引で有効。
- pose_recognizer の角度キー (`l_elbow` / `r_elbow` / `l_sh_abd` / `r_sh_abd` / `l_knee` / `r_knee`) は不変。calibration_session の出力 JSON スキーマも不変 → `web/subject_calibration/` フロントは無改修。

## 修正対象ファイル

### Infer 層

| ファイル | 変更内容 |
|---|---|
| `cpp/src/infer/types.hpp:24` | `kNumKeypoints=17` → `kMaxKeypoints=26`。`Person`/`Skeleton3D` の配列容量を 26、`kp_count` フィールドを追加 (既定 17)。 |
| `cpp/src/infer/rtmpose.hpp:13-16` | output spec コメントを「K is runtime」に。 |
| `cpp/src/infer/rtmpose.cpp:151-172` | `prepare_batch_buffers` で engine から K を取得し member に保存。`kNumKeypoints` 参照を runtime K に置換。CLI で指定された format の kp_count と engine の K が一致するかをアサート (不一致なら明示メッセージで終了)。 |
| `cpp/src/infer/rtmpose.cpp:198-235` | `decode_current_outputs` のループ上限を runtime K に。`p.kp_count = K` を設定。 |

### Lift 層

| ファイル | 変更内容 |
|---|---|
| `cpp/src/lift/keypoint_format.hpp/.cpp` (新規) | `KeypointFormat` enum、`SkeletonDef`、`skeleton_def(fmt)`、active format singleton、文字列⇔enum 変換。 |
| `cpp/src/lift/skeleton_def.hpp` | 既存 `kCocoEdges`/`kCocoParent`/`kHingeJoints`/`kMajorBoneChildren`/`hinge_child()` を保持しつつ `kHalpeEdges`/`kHalpeParent`/`kHalpeMajorBoneChildren` を追加。 |
| `cpp/src/lift/ik.hpp:60,62` | `locked_parent_len_` / `samples_` を `kMaxKeypoints` サイズに。 |
| `cpp/src/lift/ik.cpp:83-93` | `apply_subject_height_model_locked()` の比率設定は索引 5–16 のみ (Halpe26 でも意味は同じ)。Halpe26 追加点の bone length は観測フェーズで lock される。 |
| `cpp/src/lift/ik.cpp:146-171,245-246` | ループ上限を `skeleton_def(active).parents.size()` に。 |
| `cpp/src/lift/ik.cpp:188-189,256-257` | Halpe26 時は肩中点 `joints[18]`、腰中点 `joints[19]` を直接使用する分岐を追加。 |
| `cpp/src/lift/pose_recognizer.cpp:112-127` | 同様に neck/hip-center 分岐。角度キー名は不変。 |
| `cpp/src/lift/subject_profile.hpp:14,20` | `schema` 既定値を format で分岐 (`v1`/`v2`)。`bone_lengths_m` を `kMaxKeypoints` サイズに。 |
| `cpp/src/lift/subject_profile.cpp:53` | `kMajorBoneChildren` 参照を SkeletonDef 経由に。`load_subject_profile()` で schema 文字列と active format の整合性チェック、不一致は例外。 |

### Pipeline / Web

| ファイル | 変更内容 |
|---|---|
| `cpp/src/pipeline/calibration_session.cpp:601-607` | angle キー名不変。pose_recognizer 強化を間接的に享受。 |
| `cpp/src/pipeline/snapshot.cpp:74-82,148-160` | 反復上限を `kp_count` ベースに。フレーム最上位に `"kp_format": "coco17" or "halpe26"` を 1 度だけ追加。 |
| `cpp/src/pipeline/multi_pipeline.cpp/.hpp` | `Args` / `Config` に `KeypointFormat keypoint_format` 追加、起動時に singleton 設定。 |
| `cpp/src/web/crow_server.cpp/.hpp` | kp_format が JSON ペイロード経路で透過することを確認。API は無改修。 |

### CLI & Tools

| ファイル | 変更内容 |
|---|---|
| `cpp/src/main.cpp` | `--keypoint-format {coco17,halpe26}` (既定 `coco17`) を追加。起動ログに `[fitra] kp_format=halpe26 (26 keypoints)`。`--pose-engine` 既定値は format ごとに別 (engine cache は ONNX hash 依存なので自然に分離)。 |
| `cpp/src/CMakeLists.txt` | 新規 `keypoint_format.cpp` を追加。 |
| `cpp/tools/dump_keypoints.cpp:90-96` | edge テーブル重複定義を SkeletonDef 経由に。`--keypoint-format` 引数。 |
| `cpp/tools/dump_keypoints_3d.cpp` | 同上。先頭コメントの "COCO-17" を更新。 |

### Web Frontend

| ファイル | 変更内容 |
|---|---|
| `web/dual_rtmpose/app.js:7-12,121,174-188` | JSON の `kp_format` を読み `SKELETON_COCO17` / `SKELETON_HALPE26` を切替。`Array.from({length: 17})` を動的化。 |
| `web/subject_calibration/` | angle キー不変のため無改修 (動作確認のみ)。 |

### Python 参照

| ファイル | 変更内容 |
|---|---|
| `python/scripts/pose_pipeline.py:25-37,47-49,274-275` | `HALPE26_KP_NAMES` / `HALPE26_SKELETON` 定数追加。`--keypoint-format` 引数。`DEFAULT_POSE_MODEL` を format で分岐。`RtmposeOnnx` shape 検証は K から決定。 |
| `python/scripts/dual_rtmpose_web.py:108-112` | フレーム JSON 最上位に `kp_format` を追加。 |

### Docs

| ファイル | 変更内容 |
|---|---|
| `docs/cpp-migration-plan.md` | 「段階実装」に Phase 9 を追記。完了条件 = halpe26 でフル機能 (検出→IK→Phase 8 calibration)。 |
| `docs/phase8-subject-profile-runbook.md` | `--keypoint-format` フラグの記載、v1 profile を halpe26 モードで読めない旨と再キャリブ手順を追記。 |
| `docs/phase9-halpe26-migration.md` (本ファイル) | Phase 9 設計の正本。 |
| `docs/research/rtmpose-int8-eval-plan.md:9,37-38` | body7 → body8/halpe26 仕様変更を追記。 |

## Verification

1. **回帰**: `--keypoint-format=coco17 --pose-engine <既存COCO17.engine>` で 2-cam パイプライン起動 → `outputs/recorded_rtmpose/20260515_064342/overlay_cam{0,1}.mp4` と比較し bbox IoU > 0.99 / keypoint L2 < 1 px を満たすこと。
2. **モデル準備**: `./cpp/build/tools/build_engines --preset rtmpose --onnx outputs/onnx/rtmpose-m_simcc-body7_pt-body7-halpe26_700e-256x192-4d3e73dd_20230605.onnx --output outputs/tensorrt_engines/rtmpose_m_halpe26.fp16.engine --fp16` で engine を生成。
3. **C++ 単体**: `./cpp/build/tools/dump_keypoints --keypoint-format halpe26 --video outputs/recorded_rtmpose/20260515_064342/raw_cam0.mp4 --det-engine outputs/tensorrt_engines/yolox_tiny.fp16.engine --pose-engine outputs/tensorrt_engines/rtmpose_m_halpe26.fp16.engine --output /tmp/halpe26.jsonl` で 26 点を JSONL ダンプ。`jq '.persons[0].kpts | length' /tmp/halpe26.jsonl` で 26 を確認、足部 (20–25) が被写体可視時にもっともらしい位置にあること。
4. **2-cam 統合**: `./cpp/build/main --keypoint-format=halpe26 --cameras 2 ...` で起動。`http://<jetson>:8080/dual_rtmpose/` で 26 関節と新規 edge (foot/head_top/neck) が描画されること、フレームレートが Phase 8 レベル (集約 ≥ 60 fps、目標 90 fps) を維持すること。
5. **Phase 8 calibration**: `web/subject_calibration/` から halpe26 で新規 subject を作成 → `subject_profile.json` の schema が `fitra_subject_profile_v2`。angle 出力 (`l_elbow`/`r_knee` 等) が COCO17 動作との差 < 2°。
6. **互換性チェック**: v1 schema の既存プロファイルを halpe26 モードで読込 → 起動失敗し、明示メッセージで再キャリブを案内すること。
7. **Python 参照**: `python python/scripts/dual_rtmpose_web.py --keypoint-format=halpe26 --device auto` で WebSocket viewer が動作。C++ 出力との kpt[0..16] L2 < 2 px。
8. **コードスメル**: `grep -rn "kNumKeypoints" cpp/` が 0 件 (全廃)。`grep -rn "kCocoEdges\b" cpp/` の残存はテーブル定義 1 箇所のみ。

## Phase 10+ への含み (本フェーズ範囲外)

- 足部 (heel/toe) を活用したバランス指標 / 接地検出 / 歩行解析。
- head_top を使った身長推定の精度向上 (現状は kp[0]=nose 起点)。
- coco17 サポートの廃止判断 (halpe26 既定昇格後 1–2 phase 後)。
- **Pluggable PoseBackend + YAML topology**: infer 層全体 (YOLOX+RTMPose / BlazePose / YOLO-Pose 等) を backend 単位で差し替え可能にする抽象化。設計案は [`backlog-pose-backend-abstraction.md`](backlog-pose-backend-abstraction.md) に温存。
