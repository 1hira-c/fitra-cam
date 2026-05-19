# Backlog — Pluggable PoseBackend + YAML Topology

> **Status**: Phase 10+ 候補 (未着手)。Phase 9 (Halpe26 移行) 設計時に派生した別軸の課題として切り出し、本書に温存。
>
> **Trigger to start**: BlazePose / YOLO-Pose / MoveNet など RTMPose 以外の pose model を追加したい必要性が具体化したとき、または Halpe26 既定昇格後の整理 phase で。

## Context

`docs/phase9-halpe26-migration.md` は「COCO17 → Halpe26 への keypoint 拡張」に絞った設計で、`SkeletonDef` 抽象は導入するものの **YOLOX+RTMPose という 2 段 top-down 構成自体は前提のまま**で、`infer/types.hpp:24` の `kNumKeypoints` 拡張と CLI `--keypoint-format` 切替に閉じている。

中長期的には **infer 層全体 (前処理 + 検出 + Pose + 後処理) を実行環境ごとに丸ごと差し替え可能にしたい** ニーズがある。BlazePose のような単一人物 1 段モデル、YOLO-Pose のような検出統合モデル、将来の MoveNet 再評価などを backend 追加だけで差せる土台が要る。骨格の edges/parents/hinge 等の topology 情報は **YAML で外部化**し、backend が自分の topology を YAML から読んで返す。

ただしこの抽象化は Halpe26 移行とは独立した重い変更 (interface 切出し + lift 層の capability ベース再構成 + YAML loader + frontend 動的化) なので、Phase 9 のスコープから外して本 backlog に温存する。Phase 9 (Halpe26) と本案は将来合流する可能性が高いが、着手判断と着地は別フェーズで行う。

## Goal / Non-goal

| | 範囲 |
|---|---|
| Goal | (1) `PoseBackend` interface 切出し。 (2) 既存 YOLOX+RTMPose を `RtmPoseBackend` として interface 実装に再パッケージ (挙動同一)。 (3) YAML topology + `SkeletonDef` ローダ。 (4) `--backend <id>` CLI。 (5) `BackendCapabilities` 経由で IK / pose_recognizer / subject calibration の有効化を制御。 (6) BlazePose / YOLO-Pose 等を **backend 1 個追加で差せる土台**であることをサンプル実装で担保。 (7) frontend (web) が `backend_id` を見て skeleton を切替。 |
| Non-goal | BlazePose / YOLO-Pose / MoveNet backend の実体実装 (土台確認のサンプル止まり)。マルチ推論バックエンド (ORT / TFLite) 対応。Topology の hot-reload。 |

## アーキテクチャ案

### Layer 図

```
                pipeline (multi_pipeline, frame_source, snapshot)
                        │ holds: infer::PoseBackend&
                        ▼
            ┌──────── infer::PoseBackend (interface) ────────┐
            │   infer(frame) -> vector<Person>               │
            │   skeleton() -> const SkeletonDef&             │
            │   capabilities() -> BackendCapabilities        │
            │   id() -> string                               │
            └────────────────────────────────────────────────┘
                  ▲                              ▲
                  │                              │
  RtmPoseBackend (rtmpose-coco17,        (future) BlazePoseBackend
                 rtmpose-halpe26)                  YoloPoseBackend
    internal: YOLOX + RTMPose                       internal: 自由
              + SkeletonDef from YAML
```

`pipeline` 層は backend 種別を知らない。2 段か 1 段かは backend の内部都合に閉じる。

### `infer::PoseBackend` interface

新規 `cpp/src/infer/pose_backend.hpp`:

```cpp
namespace fitra::infer {

struct BackendCapabilities {
    bool coco17_compatible_indices = false;  // kpts[0..16] が COCO17 順序
    bool supports_3d_lift          = false;  // lift/triangulator 互換
    bool supports_ik               = false;  // lift/ik 互換
    bool supports_pose_recognizer  = false;  // lift/pose_recognizer 互換 (Phase 8 角度計算)
    bool supports_subject_profile  = false;  // lift/subject_profile 互換
};

class PoseBackend {
public:
    virtual ~PoseBackend() = default;
    virtual std::string         id()           const = 0;
    virtual std::size_t         kp_count()     const = 0;
    virtual const lift::SkeletonDef& skeleton() const = 0;
    virtual BackendCapabilities capabilities() const = 0;

    virtual std::vector<Person> infer(const cv::Mat& frame) = 0;

    // 既存 RtmPose の prebaked 経路を活かす低レベル拡張点 (optional)。
    virtual bool supports_prebaked_pose() const { return false; }
};

}  // namespace fitra::infer
```

「prebaked の有無」を capability として表面化することで、現行の per-camera preprocess 並列化 (frame_source 経由) を維持しつつ、それを持たない backend (BlazePose 等) も差せる。

### `lift::SkeletonDef` を YAML 駆動に

現行 `kCocoEdges` 等の const tables は YAML loader の `coco17.yaml` に移植:

```cpp
namespace fitra::lift {

struct SkeletonDef {
    std::string                              name;
    std::size_t                              kp_count;
    std::vector<std::string>                 keypoint_names;
    std::vector<std::pair<int,int>>          edges;
    std::vector<int>                         parents;
    std::vector<int>                         hinge_joints;
    std::vector<int>                         major_bone_children;
    int                                      hip_center_index = -1;
    int                                      neck_index       = -1;
    int                                      hinge_child(int parent) const;
};

SkeletonDef load_skeleton_yaml(const std::filesystem::path& path);

}
```

YAML 例 (`cpp/configs/skeletons/halpe26.yaml`):

```yaml
name: halpe26
kp_count: 26
keypoint_names: [nose, l_eye, r_eye, l_ear, r_ear, l_shoulder, r_shoulder,
                 l_elbow, r_elbow, l_wrist, r_wrist, l_hip, r_hip,
                 l_knee, r_knee, l_ankle, r_ankle, head_top, neck, hip_center,
                 l_big_toe, r_big_toe, l_small_toe, r_small_toe, l_heel, r_heel]
edges:    [...]
parents:  [...]
hinge_joints: [7, 8, 13, 14]
major_bone_children: [...]
hip_center_index: 19
neck_index: 18
```

YAML 探索順: (1) `--skeleton-yaml <path>` 明示、(2) `<install_prefix>/share/fitra/skeletons/<name>.yaml`、(3) リポジトリ相対 `cpp/configs/skeletons/<name>.yaml` (開発時 fallback)。

### Backend factory

```cpp
namespace fitra::infer {
struct BackendOptions {
    std::string id;
    std::filesystem::path det_onnx;
    std::filesystem::path pose_onnx;
    std::filesystem::path det_engine;
    std::filesystem::path pose_engine;
    std::filesystem::path skeleton_yaml;
    // ... TRT 関連 options
};
std::unique_ptr<PoseBackend> create_backend(const BackendOptions& opts);
}
```

実装内部で id を switch、未知 id は即時例外。

### `RtmPoseBackend` (既存実装の再パッケージ)

`cpp/src/infer/backends/rtmpose_backend.hpp/.cpp` (新規)。現行 YOLOX + RtmPose の組合せをそのままラップ:

- コンストラクタで `BackendOptions` を受け、det / pose 用 `TrtEngine` を構築 (`trt_builder.hpp` 経由)
- `infer(frame)` は YOLOX → bbox → RtmPose の流れ。現 `pipeline` がやっている backend 依存部分のみ吸収
- `skeleton()` はコンストラクタで YAML 読込済みの `SkeletonDef` を返す
- `capabilities()`: rtmpose-coco17 / rtmpose-halpe26 ともに 4 機能全部 true
- `supports_prebaked_pose() = true`、追加メソッド `prebaked_pose_request(...)` を expose

**detector 間引き (`--det-frequency`) は backend 内部状態として持つ**。今は frame_source が制御しているがこれを backend に移譲する (BlazePose 等は detector 間引きの概念がそもそも違うので、backend が自分のペーシングを持つのが筋)。

### Capability ベース制御

`lift/ik.cpp`, `lift/pose_recognizer.cpp`, `lift/subject_profile.cpp`, `pipeline/calibration_session.cpp` の入口で:

```cpp
if (!backend.capabilities().supports_ik) {
    spdlog::error("Backend '{}' does not support 3D IK; disable Phase 7-8 features or pick a compatible backend.", backend.id());
    return; // または起動時 fatal
}
```

RTMPose 系 backend は全部 true で通る。BlazePose 追加時にここで自然に分岐できる。

### `kNumKeypoints` 廃止 (Phase 9 で既出予定の作業)

`infer/types.hpp:24` の `kNumKeypoints=17` → `kMaxKeypoints=26`、`Person::kp_count` / `Skeleton3D::kp_count` 追加。snapshot / drawer / web JSON 出力は全て `kp_count` で打ち切り。**Phase 9 が先行してこれを済ませている場合は本 backlog では再利用するだけ**。

## 修正対象ファイル (案)

### 新規

| ファイル | 内容 |
|---|---|
| `cpp/src/infer/pose_backend.hpp` | `PoseBackend` interface + `BackendCapabilities` |
| `cpp/src/infer/backend_factory.hpp/.cpp` | id 文字列 → backend 生成 |
| `cpp/src/infer/backends/rtmpose_backend.hpp/.cpp` | YOLOX + RTMPose を `PoseBackend` 実装に再パッケージ |
| `cpp/src/lift/skeleton_def_yaml.hpp/.cpp` | YAML → `SkeletonDef` ローダ + path 解決 |
| `cpp/configs/skeletons/coco17.yaml` | 既存 const tables の移植 |
| `cpp/configs/skeletons/halpe26.yaml` | Halpe26 topology (Phase 9 docs から移植) |

### 変更

| ファイル | 変更 |
|---|---|
| `cpp/src/lift/skeleton_def.hpp/.cpp` | const tables 削除、`SkeletonDef` を struct に。active singleton は持たず backend 注入で対応 |
| `cpp/src/lift/ik.{hpp,cpp}` | capability チェック追加。`apply_subject_height_model_locked()` は backend.skeleton().hip_center_index / neck_index を参照 (-1 なら midpoint 代用) |
| `cpp/src/lift/pose_recognizer.cpp` | 同 (neck/hip-center を index or midpoint 切替)。angle キー名は不変 |
| `cpp/src/lift/subject_profile.{hpp,cpp}` | `schema` を backend.id() ベース (`fitra_subject_profile_<backend_id>_v1`)、互換性チェック失敗で再キャリブ案内 |
| `cpp/src/pipeline/multi_pipeline.{hpp,cpp}` | `infer::RtmPose&` → `infer::PoseBackend&`。`--det-frequency` を backend に移譲 |
| `cpp/src/pipeline/frame_source.cpp` | RtmPose preprocess は `backend.supports_prebaked_pose()` の時のみ実行。それ以外は backend に frame を直渡し |
| `cpp/src/pipeline/snapshot.cpp` | フレーム JSON 最上位に `"backend_id"` を追加 |
| `cpp/src/pipeline/calibration_session.cpp` | 入口で capability チェック |
| `cpp/src/web/crow_server.{hpp,cpp}` | `/api/info` 等で backend id / skeleton edges を露出 |
| `cpp/src/main.cpp` | `--backend <id>` フラグ追加。既存の `--det-onnx` `--pose-onnx` `--det-engine` `--pose-engine` は backend optional override として残す |
| `cpp/CMakeLists.txt` | yaml-cpp を `FetchContent` で追加 (asio / Crow と同じパターン、tag `yaml-cpp-0.8.0`) |
| `cpp/tools/dump_keypoints*.cpp` | edge テーブル重複定義削除、`--backend` 経由で SkeletonDef 取得 |
| `web/dual_rtmpose/app.js` | JSON の `backend_id` を見て skeleton 切替 |

## CLI / Engine cache

```bash
./cpp/build/main                              # 既定 backend
./cpp/build/main --backend rtmpose-halpe26
./cpp/build/main --backend rtmpose-halpe26 --pose-onnx ~/.cache/.../halpe26.onnx
```

- Engine cache: `outputs/tensorrt_engines/<backend_id>_<onnx_hash>_<precision>.engine` (現在の `<onnx_hash>.engine` から命名変更、初回 re-build ~7 分)
- Skeleton YAML 既定 path: `cpp/configs/skeletons/<topology>.yaml`

## 着手時の Verification

1. **回帰**: `--backend rtmpose-coco17` で 2-cam パイプライン起動 → `outputs/recorded_rtmpose/20260515_064342/overlay_cam{0,1}.mp4` と比較し bbox IoU > 0.99 / keypoint L2 < 1 px
2. **Backend interface 健全性**: `grep -rn "RtmPose\b\|Yolox\b" cpp/src/pipeline/` でゼロ件 (backend factory 配下にのみ残存)
3. **YAML 駆動性**: skeleton YAML を一時的に壊して起動 → 明示エラーで停止すること
4. **Capability gating**: 偽の capability=false backend stub で IK / pose_recognizer / Phase 8 calibration が起動時に明示エラーで停止
5. **将来 backend 追加コスト**: ダミー backend (`rtmpose-coco17` を別 id で複製) を 1 ファイル + 1 YAML 追加で `--backend` から呼べることを実証

## Phase 9 (Halpe26) との関係

- Phase 9 で先行する `kMaxKeypoints=26` / `Person::kp_count` / `Skeleton3D::kp_count` / `--keypoint-format` 切替は本案で **そのまま再利用**。本案着手時に `--keypoint-format` を `--backend` に統合する整理が入る。
- Phase 9 の `lift/keypoint_format.{hpp,cpp}` (もし作られていれば) は本案で `lift/skeleton_def_yaml.{hpp,cpp}` に統合し、active singleton は backend 注入に置き換える。
- Phase 9 で Halpe26 が既定昇格していれば、本案の `RtmPoseBackend` 既定 ID は `rtmpose-halpe26` から開始。

## 着手前提条件

- Phase 9 (Halpe26) 完了、または Phase 9 と並行で進めるなら scope 衝突を整理してから
- BlazePose / YOLO-Pose を「具体的にいつ追加するか」が決まっている (土台だけ作って実体追加が無期限延期になるリスク回避)
- `outputs/recorded_rtmpose/20260515_064342/` 相当の回帰用素材が現役 (backend 切替の動作確認に必要)

## 想定リスク

- **yaml-cpp の Jetson build 時間**: header-only ではないので FetchContent 時の cmake build に数十秒。初回 `cmake --build` で吸収。
- **Backend interface に detector 間引きを移譲する変更が pipeline 側に副作用**: 現 `frame_source` 制御の `--det-frequency` を backend に移動するため、stats / pending 計測の reporter が backend を経由する必要が出る。snapshot.cpp の stats 出力を要見直し。
- **Engine cache 命名の後方互換**: 命名変更で既存 engine の再 build (~7 分) が走る。初回 1 回限りなので許容。
- **CLI 互換**: 既定 `--backend rtmpose-coco17` で従来コマンドが動くようにするが、`--pose-engine` 等の旧フラグは backend が消費する形に整理が必要。`main.cpp` の arg parser で「未指定なら backend 既定を使う」セマンティクスに統一。
- **抽象化のための抽象化リスク**: 追加 backend の具体的需要 (BlazePose / YOLO-Pose を実際に使う必要性) が確定する前に着手すると、interface 設計が実モデルにフィットせず再設計コストが発生する。本 backlog は着手前提条件で「具体的需要の確定」を必須条件にしている。
