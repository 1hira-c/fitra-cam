# pose-3d: キャリブレーション専念モード化 (calib↔runtime の YAML 疎結合化)

(着手日 2026-06-10 / 関連: [pose-3d-controller-marker-extrinsic.md](pose-3d-controller-marker-extrinsic.md),
[vr-output-continuous-hmd-calibration.md](vr-output-continuous-hmd-calibration.md) /
前提合意: **初期設定・キャリブレーション中に他モジュールが動く必要はない** — 専念してよい)

## 背景 / 動機

subject wizard と controller-marker extrinsic は live パイプラインと**同一プロセスに同居**し、
結果を稼働中の driver へ**ライブ再注入**する設計になっている。結合点の列挙:

1. **IK ホットリロード** — `main.cpp:558-562` の `on_approved` →
   `driver->ik().reload_from_profile(p)`。approve 後に走行中 IK を外部から書き換えるための
   変異エントリ (`multi_pipeline.hpp:96` コメント)。呼び出し元はこの 1 箇所のみ。
2. **Triangulator ホットスワップ** — `main.cpp:857-877` の
   `set_extrinsic_calib_solved_callback`。`/api/excal/solve` 成功 → 書き出した YAML を読み直し
   → `driver->set_triangulator(next)` で走行中 driver の 3D を late-binding 差し替え。
3. **frame tap の多重化** — `main.cpp:653-680`。excal が collecting/solving の間は AprilTag
   collector へ、それ以外は subject recorder へ、**毎フレーム状態を見て分岐**。
4. **排他フラグの全 FrameSource 配布** — `calib_recording_flag`
   (`camera/frame_source.hpp:119`) を main が生成し全カメラへ配る。
5. **Crow が両 session を直接知る** — `crow_server.cpp:363/369` の setter +
   `register_calibration_routes_` (`crow_server.cpp:990` 以降) が `/api/calib/*` `/api/excal/*`
   を無条件登録。
6. **~20 個の `--calib-*` / `--excal-*` フラグ**と上記配線がすべて `main.cpp` に同居。

このうち 1〜3 は「キャリブレーションを稼働中ランタイムに同居させ、結果を**再起動なしで**反映する」
ためだけに存在する。冒頭の前提 (キャリブ中に tracker 出力・配信は不要) を置くと同居要件そのものが
落ち、calib↔runtime の接点を **YAML 成果物のみ** — `CalibrationSet` (intrinsics+extrinsics,
`lift/calib_io`) と `SubjectProfile` (v1/v2, `lift/subject_profile`) — に縮退できる。
ファイル経由は到達可能な最も疎な結合であり、スキーマが契約になる (subject profile は v1/v2 の
厳格分離が既にある。track doc 設計原則参照)。

初回分析 (2026-06-10 会話) の誤りを 2 点記録しておく:

- (a) `calib_recording_flag` は「録画窓では推論を止めてディスク I/O に CPU/GPU を譲る」という
  **ウィザード内在の機構**であり、同居の産物ではない。ウィザードは録画窓の合間に live 3D
  (PoseRecognizer の hold 判定) を必要とするため、専念モードでもフラグは残る (配線が
  calib-subject モード限定になるだけ)。
- (b) excal→subject の同一プロセス続行 (track changelog 2026-06-09) は**意図された UX**。
  これを壊すかどうかが本設計の主要トレードオフで、案C/C' の分岐点。

**完了条件**: runtime モードの構築パスに calibration session / tap / ホットスワップが存在しない。
setup 系モードは publisher を構築しない。モード境界を越えるのは YAML のみ。ctest 全 pass +
実機で excal → subject → runtime の 3 段フローが回る。

## 検討した案

### 案A: 現状維持 (同居 + ライブ再注入) (没)
同居が買っているのは「再起動 1 回の節約」だけなのに、calib 機能を足すたびに結合点が増殖してきた
(tap の状態分岐 → solved callback → …)。floor-AprilTag SfM (research) など今後の校正方式追加で
さらに膨らむ。**没: 前提下では同居の利得がコストに見合わない。**

### 案B: 別バイナリ×3 (runtime / subject-calib / excal) (没・将来含み)
リンク依存まで最小化できる (excal は TRT 不要になりうる) が、先に build graph の手術が要る:
`fitra_camera → fitra_infer` の融合 (`cpp/src/CMakeLists.txt:63` — FrameSource が Yolox を所有し
RTMPose 入力を prebake する**意図的な性能設計**) と、`fitra_vmt → fitra_slimevr → fitra_pipeline`
連鎖 (excal が要るのは `controller_pose_receiver` だけなのに全部リンクされる)。さらに main.cpp
構築コードの共有化が前提で、Jetson へのデプロイ品も 3 つに増える。
**没 (今は): 疎結合の本体は配線分離 (案C) だけで達成でき、リンク最小化は後からでも遅くない。**

### 案C: 1 バイナリ + 排他 RunMode + 再起動受け渡し (採用)
mode = `run` / `calib-subject` / `calib-extrinsic`。各モードは**自分に必要なものだけ構築**し、
モード間の受け渡しは YAML + プロセス再起動。パッケージングは現状維持 (1 バイナリ) なので
ビルド・デプロイに変化なし。疎結合の実体は配線にあり、バイナリ分割は二次的、という整理。

### 案C': 案C + 同一プロセス逐次再構築 (没)
excal 完了 → 全 teardown → subject 構築を 1 プロセス内でやれば従来の続行 UX を保てるが、
TRT/CUDA context・V4L2 の mid-process 完全 teardown→再構築という**新しい失敗モード**を持ち込む。
再起動コストは TRT engine deserialize の数秒〜十数秒で、初期設定の頻度 (subject 切替・カメラ移設時
のみ) に対して許容範囲。**没: UX 利得が小さくリスクだけ新規。** web UI の導線ガイダンスで補う。

## 採用設計

### 不変条件

- **calib↔runtime の契約は YAML のみ**。プロセス内状態 (session オブジェクト・driver への参照・
  callback) はモード境界を越えない。
- **runtime (`run`) は calibration を知らない**: `CalibrationSession` / `ExtrinsicCalibSession` /
  frame tap / skeleton3d tap / triangulator 差し替えを構築・登録しない。YAML は boot で読む
  (読めなければ 2D-only — 現状踏襲)。
- **setup 系モードは出力しない**: SlimeVR / VMT **publisher** を構築しない。VMT 側でも
  **受信** (pose relay = `controller_pose_receiver` / `hmd_pose_receiver`) は excal の入力なので
  送信と区別して扱う。
- **IkSolver の外部変異エントリを最小化**: `reload_from_profile` は唯一の呼び出し
  (`main.cpp:561`) ごと削除し、profile 反映は boot 時ロード (`main.cpp:500` 経由) に一本化。
  `apply_subject_height` は calib-subject モード内部 (preflight → recognizer の bone-length lock)
  で引き続き使う — 専念モードでは「自分のパイプラインを自分で設定する」内部処理であり結合ではない。

### モードごとの構築物

| mode | capture | TRT | 3D (triangulator+IK) | session | publisher | web routes |
|---|---|---|---|---|---|---|
| `run` | FrameSource (prebake) | ○ | ○ (YAML 必須でなければ 2D-only) | なし | SlimeVR/VMT | viewer 系のみ |
| `calib-subject` | FrameSource (prebake + recording_flag) | ○ | ○ (extrinsics YAML **必須**) | CalibrationSession | なし | viewer + `/api/calib/*` |
| `calib-extrinsic` | FrameSource (**decode-only**, `Yolox=nullptr` — `frame_source.hpp:127` で既サポート) | **実行時不要** | なし | ExtrinsicCalibSession + ControllerPoseReceiver | なし | `/api/excal/*` |

- `calib-extrinsic` は `MultiCameraDriver` を使わず、FrameSource 群を poll して
  `session->on_frame()` を呼ぶ**軽量 capture ループ** (新規・数十行) で回す。これにより
  driver の frame tap は calib-subject 専用の単一 consumer 機構になり、`main.cpp:653-680` の
  状態分岐 mux が消える。
- CLI は既存フラグを温存して mode を導出する (`--extrinsic-calib` → calib-extrinsic、
  `--calibrate` → calib-subject、どちらもなし → run)。両指定はエラー (排他)。
  invocation 互換を保ち、runbook/スクリプトの書き換えを不要にする。

### excal→subject 導線の置き換え

現状: solve 成功 → triangulator hot-swap → 同一プロセスで subject wizard 続行。
新: solve 成功 → YAML 書き出し → web UI に「subject calib モードで再起動」のガイダンス
(実行コマンド表示) を出して auto-exit。プロセス内 supervisor は持たない
(必要になったら `scripts/` の薄い 2 段ラッパで足りる — 残課題)。

### web の出し分け

Crow のルート登録をモジュール化し、各モードは自分のルート群だけ登録する。run モードでは
`/api/calib/*` `/api/excal/*` が **404** になる (現契約の「session 未 attach 時 503」は
setup 系モード内の一時状態にのみ残る)。トップページの calib 導線はモード情報
(`/api/state` 等に mode フィールド追加) で出し分け。

### 削除されるもの (= 結合点の物理削除)

- `MultiCameraDriver::set_triangulator` (`multi_pipeline.hpp:93`) と solved callback 一式
  (`main.cpp:857-877`, `CrowServer::set_extrinsic_calib_solved_callback`)
- `IkSolver::reload_from_profile` と `on_approved` ホットリロード (`main.cpp:558-562`)
- frame tap の excal/calib 状態分岐 mux (`main.cpp:653-680`)
- run モードでの session 構築・calib ルート登録・`calib_recording_flag` 配布

### composition root の抽出

`main.cpp:274-922` の構築シーケンス (config → cameras → engines → driver → sessions →
publishers → web) を `app/` builder (例 `app/runtime_builder.{hpp,cpp}`) へ抽出し、main.cpp は
mode dispatch + 各モードの組み立て宣言だけにする。calib-subject が「run から publisher を
引いたもの」である構造を、コピーではなく**構築部品の共有**で表現するための土台。

## Milestone (= コミット境界)

- **M0**: 本 doc + `docs/tracks/pose-3d.md` changelog 1 行。`docs(pose-3d):` コミット。
- **M1**: RunMode 導入 + 構築ゲーティング。`--calibrate` / `--extrinsic-calib` を排他モード化し、
  setup 系で publisher 非構築 / run で session・tap 非構築。配線自体は main.cpp 内のまま。
  挙動変更: calib 中の tracker 出力停止、run での `/api/calib/*` 404。
- **M2**: ライブ再注入の削除。`set_triangulator` / `reload_from_profile` / solved callback /
  tap mux を削除。calib-extrinsic を FrameSource 直結軽量ループ + decode-only 化
  (TRT 実行時不要)。solve 後は auto-exit + 再起動ガイダンス。`test_crow_excal` の
  同一プロセス続行を固定しているテストは新契約 (solve → guidance → exit) に書き換え。
- **M3**: composition root 抽出 + Crow ルート登録のモジュール化。挙動不変リファクタ
  (M1/M2 の配線を builder へ移すだけ)。
- **M4**: ドキュメント整備。track doc 現状節・`cpp-migration-plan.md` の該当アーキ記述・
  3 段フロー runbook。

各 M で build + `ctest` 全 pass を green ゲート。M1 と M2 は独立 revert 可能、M3 は M1/M2 後提。

## 検証

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release && cmake --build cpp/build -j
ctest --test-dir cpp/build --output-on-failure
```

- 既存 ctest pass: `test_extrinsic_calib_session` / `test_crow_excal` / `test_pose_recognizer` /
  `test_main_config` ほか。M2 で `test_crow_excal` の続行系テストのみ新契約へ置換 (削除でなく
  「solve 成功 → exit 要求が立つ」を固定し直す)。
- **実機 3 段フロー**: (1) `main --extrinsic-calib ...` → extrinsics YAML、(2) `main --calibrate ...`
  → subject profile YAML、(3) `main --enable-3d ...` → tracker 出力。各段の入力が**前段の YAML
  だけ**であること (プロセスをまたいで他に何も渡らない) を確認。
- 構造検査: run モードで `/api/calib/state` が 404。`set_triangulator` / `reload_from_profile`
  のシンボルが削除済み (コンパイルレベルで再注入経路が存在しない)。
- 性能: run モードの hot path は不変ないし微減 (tap の nullptr チェック分岐が消える)。
  `calib-extrinsic` は TRT 初期化が消えるぶん起動が速くなる。

## 残課題

- **案B (別バイナリ化)**: FrameSource の decode/prebake 分割、pose receiver 群の小ライブラリ化
  (`fitra_vmt → fitra_slimevr` 連鎖切り)。案C 完了後にリンク最小化の価値が残っていれば。
- **continuous HMD calibration は対象外**: ランタイム同時実行が本質
  ([vr-output-continuous-hmd-calibration.md](vr-output-continuous-hmd-calibration.md)) なので
  本設計の「setup 系」には含めず run モード側に残る。
- floor-AprilTag SfM ([research/floor-apriltag-sfm-map.md](../research/floor-apriltag-sfm-map.md))
  を実装する場合は calib-extrinsic の別フェーズ or 新 setup モードとして追加する。
- excal→subject の 1 コマンド運用が実機で欲しくなったら `scripts/` の薄い 2 段ラッパで対応
  (案C' の同一プロセス再構築は再考しない)。
