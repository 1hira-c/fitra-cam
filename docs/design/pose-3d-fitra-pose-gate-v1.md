# `fitra_pose_gate_v1` position-only 観測契約

作成日: 2026-08-02
状態: M0 実装

## 背景

fitra-fusion の初期入力は、VR tracker の姿勢や Kalman/IK の結果ではなく、
カメラ三角測量がそのフレームで観測した body joint の位置である必要がある。
従来の `/ws3d` は WebUI/VMT 互換のために post-processing 後の
`Skeleton3D` を公開しており、そこから fusion 入力を作ると保持値・予測値・
IK で拘束された値が新しい観測に見える。

M0 では既存の `/ws3d` 契約を変更せず、`triangulator->triangulate()` の戻り値を
直接 position-only 契約へ写像する独立 bus と `WS /ws/pose-gate` を追加する。

## 採用した契約

```json
{
  "protocol_version": "fitra_pose_gate_v1",
  "stream_id": "opaque per-process id",
  "subject_track_id": "opaque lifecycle id, or \"none\"",
  "coordinate_epoch": 1,
  "content_mono_ns": 1234567890123456,
  "provenance": {
    "pipeline": "fitra-cam",
    "stage": "tri.skeleton",
    "position_source": "multi_view_triangulation",
    "postprocess": "none",
    "kalman": false,
    "ik": false,
    "floor_contact": false
  },
  "source_state": "Fresh",
  "position_space": "fitra_world_z_up_m",
  "joints": {
    "hips": {
      "position_m": [0.0, 0.0, 0.9],
      "availability": "Fresh",
      "keypoint_score": 0.95,
      "view_count": 2,
      "reproj_error": 0.7
    }
  }
}
```

`joints` は `hips`, `neck`, `left_hip`, `right_hip`, `left_knee`,
`right_knee`, `left_ankle`, `right_ankle` の8個を常に出す。
未観測 joint は `position_m: null`, `availability: "Unavailable"`,
品質値も `null` とする。`Unavailable` の joint に最後の位置をコピーしない。
`reproj_error` の単位は px、`position_m` の座標は fitra world の meter である。

`content_mono_ns` は各同期入力フレームに記録した Jetson の
`clock_gettime(CLOCK_MONOTONIC)` の値のうち最も古いものを使う。HTTP/WS送信時刻や
wall-clock を content time として使わない。送信の重複を防ぐため JSON には
`sample_seq` も含め、同じ観測を固定周期で再送しない。

## lifecycle と安全境界

- `stream_id` はプロセス起動ごとに生成する opaque 値で、配列 index や camera index
  を意味しない。
- `subject_track_id` は観測の lifecycle ごとに生成する opaque 値である。M0 は
  `--multi-person` を gate の対象外とし、単一人物経路だけを扱う。複数人物の割当を
  配列順で代用しない。
- 初回観測・再接続後の観測・人物切替後の観測は新しい `subject_track_id` になる。
  消失、同期失敗、idle、unsupported topology は全8 joint `Unavailable` を一度公開し、
  前回値を保持しない。
- 人物切替は raw hips/neck の連続性と体幹スケールを使った保守的な jump gate で検出する。
  切替フレームは `source_state: "PersonSwitched"` と新しい ID を持つ。下流は ID または
  state の変化を見て hold state を捨てられる。
- `coordinate_epoch` は calibration/basis/origin が同じ間だけ同じ値とする。現在の
  Triangulator は runtime hot-reload を持たず、校正成果物の変更はプロセス再起動を要求する。
  起動時に実際に読み込んだ calibration artifact の FNV-1a fingerprint を epoch として使うため、
  成果物が変わった再起動では epoch も変わる。プロセス再起動では `stream_id` も変わる。
  将来 hot-reload を追加する場合は `set_coordinate_epoch()` で epoch を先に変え、最初の観測を
  `EpochChanged` + 全 `Unavailable` の境界フレームにする。

`source_state` は `Fresh`, `Reacquired`, `PersonSwitched`, `Unavailable`,
`EpochChanged`, `UnsupportedTopology`, `UnsupportedMultiPerson` のいずれかである。
`Reacquired`/`PersonSwitched` でも、そのフレームで実際に三角測量できた joint だけが
`Fresh` になる。state 名が Fresh でないことを理由に観測値を hold 値へ変換してはならない。

## 配信境界

- `GET /api/pose-gate`: 最新の `fitra_pose_gate_v1` JSON。診断・curl用。3D が無効な場合も
  必須フィールドを揃えた全 `Unavailable` を HTTP 409 で返す。
- `WS /ws/pose-gate`: 新しい `sample_seq` が生成されたときだけ JSON を送る。
- `/ws3d` と `/stats3d` は既存 WebUI/VMT 契約のまま。pose gate は tracker quaternion
  を含まず、既存 tracker bus からも生成しない。
- pose-gate WS の接続は idle の consumer count に含める。fusion だけを接続した状態で
  3D producer が standby に入らないようにする。

## M0 の非スコープ

- 複数人物の cross-camera association / persistent identity
- 関節の予測、保持、補完、平滑化、IK、床接地補正
- runtime calibration hot-reload
- 実機 trace の自動収集。`docs/samples/fitra_pose_gate_v1/` の JSONL は契約と
  lifecycle を確認する deterministic fixture であり、Jetson 実機合格記録ではない。

## 検証

`test_pose_gate` で正常、関節欠損、消失→再取得、人物切替、epoch変更、JSON禁止事項を
固定する。実機では Halpe26 + 2台以上の camera で `/ws/pose-gate` を JSONL 保存し、
I/Ski の8 joint が `Fresh` になること、再接続/再校正で ID/epoch と全Unavailable境界が
記録されることを確認する。
