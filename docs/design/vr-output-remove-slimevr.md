# vr-output: SlimeVR 出力廃止

(実装日 2026-07-14)

## 背景

VR 出力は VMT → SteamVR を実運用としており、回転のみの SlimeVR Firmware UDP 経路、設定、補正 UI を維持する理由がなくなった。共有 tracker 抽出が `slimevr` 名前空間に置かれていたため、不要な出力を消すだけでは VMT の依存が不明瞭だった。

## 採用設計

- Firmware UDP protocol、NativePublisher、`/stats3d.slimevr`、`/api/slimevr/corrections` と対応 WebUI を削除する。
- 共有する 10 点 tracker pose、抽出、平滑化、snapshot bus は `cpp/src/tracking/` と `fitra::tracking` に移す。VMT と WebUI はこの単一 producer を read-only に消費する。
- VMT の TrackerRole 順、index、OSC `/VMT/Room/Driver` wire、`/ws3d.trackers` の JSON shape は維持する。
- `slimevr:` YAML と `--slimevr-*` は互換受理せず、削除して VMT を使うよう案内するエラーで停止する。

## 検証

Release build と `ctest -R 'tracker_extract|vmt|main_config|flow_daemon'`、`pnpm -C web-ui build` を通す。実機では VMT Manager/SteamVR へ送信し、`/stats3d` に `trackers` と `vmt` が残り `slimevr` が出ないことを確認する。

## 履歴

Phase 11–13 の SlimeVR 設計と bridge relay の没記録は `docs/archive/` に残す。未実装の SlimeVR 専用 backlog は削除する。
