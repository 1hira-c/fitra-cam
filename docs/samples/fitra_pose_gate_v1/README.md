# `fitra_pose_gate_v1` サンプル

このディレクトリの JSONL は、M0 の schema と lifecycle 境界を確認するための
決定的 fixture である。`normal.jsonl`、`missing.jsonl`、`person-switch.jsonl`、
`epoch-change.jsonl` を個別に読み込める。

値は契約テスト用の合成値であり、Jetson 実機の I/Ski 合格記録ではない。実機検収では
`WS /ws/pose-gate` をそのまま JSONL 保存し、`content_mono_ns`、8 joint の Fresh 状態、
再接続時の track ID、再較正時の coordinate epoch を確認する。
