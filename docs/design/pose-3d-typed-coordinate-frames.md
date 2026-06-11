# pose-3d: 座標フレームの型レベル区別による疎結合化

(着手日 2026-06-10 / plan: `~/.claude/plans/witty-brewing-lemon.md` / 関連: `pose-3d-controller-marker-extrinsic.md` の 2026-06-09 split-brain 修正)

## 背景 / 動機

cpp/ の pose-3d / vr-output トラックは 3D 数学を**素の OpenCV コンテナ**（`cv::Matx44d` / `cv::Vec3d` / `cv::Point3d`）で表現し、座標フレームの意味論は変数名（規約 `T_<to>_<from>` = T_to←from）とコメントだけに宿る。コンパイラは一切検査しない。

実害が出たのが 2026-06-09 の "split-brain" リグレッション（`pipeline/extrinsic_calib_session.cpp`）。検証シーンの cameras を fitra Z-up・live poses を VMT Y-up のまま混在させ、カメラが別位置に描画された。根因は次の 2 つが型で防げないこと:

- **(a) to/from 連鎖の取り違え** — `T_cam_world * T_cam_marker` のような不整合合成が build を通る。
- **(b) world 種別（fitra Z-up / VMT Y-up）の取り違え** — `T_cam_world` という同名フィールドが、solver 出力時点では VMT Y-up、YAML 永続化後は fitra Z-up という非対称を名前で区別できない。

加えて Z-up↔Y-up 変換が **3 箇所に別々の手書きロジック**で散在する:

- `vmt/vmt_protocol.hpp` `world_pos_to_vmt` / `world_quat_to_vmt`（fitra Z-up → VMT Y-up, Rx(-90°)）
- `slimevr/firmware_protocol.cpp` `world_quat_to_slime`（fitra Z-up → Unity, X-flip + Y/Z swap）
- `pipeline/extrinsic_calib_session.cpp` `kVmtWorldToFitra` + `to_fitra_world`（VMT Y-up → fitra Z-up）

これらは互いに逆/関連変換のはずだが別実装で、整合は個別 test が固定するだけ。

**完了条件**: 主要フレームを型で区別し、(a)(b) をコンパイルエラー化。Z-up↔Y-up 変換を単一の型付き関数へ集約。**実機挙動・数値は不変**（既存 ctest 全 pass）。

## 検討した案

### 案A: Phantom-typed transforms + points をフル適用（没）
`Transform<To,From>` / `Point3<Frame>` を全モジュール（Joint3D / kalman / IK / tracker_extract / wire まで）へ。完全なコンパイル時安全だが blast radius が全域。毎フレーム 26 関節を回すホットパスや wire シリアライズまで総称化する実利が薄く、`.raw()` escape hatch が頻発して churn 過大。**没: 払うコストに対し守れるバグが M2 境界に集中しており、全域化は過剰。**

### 案B: 量×フレームごとの個別 wrapper 構造体（没）
`struct FitraWorldPose{cv::Matx44d T;}` 等を量×フレームで列挙。総称代数なし。組合せ爆発し、`compose`/`inverse`/`average` を型ごとに重複実装するか結局 `.T` を剥がして素の演算に戻る → hand-eye のような連鎖合成（split-brain の震源）に弱い。**没: 連鎖の型安全という肝心の利得が出ない。**

### 案C: 混同された world 種別のみ最小型付け（没）
camera/marker は素のまま `FitraWorldPose`/`VmtWorldPose` の 2 型だけ。最小 blast radius だが、solver 内部の `T_cam_marker` ↔ `T_cam_world` 取り違え（(a)）を防げない。**没: (b) は塞ぐが (a) を残す。**

### 案D: ハイブリッド（採用）
SE(3) レイヤ（extrinsic solver / triangulation / calib I/O / calib session）に phantom-typed `Transform<To,From>` 代数を導入し、leaf I/O（`Joint3D`/`Skeleton3D`/kalman/IK/wire）は現状維持。Z-up↔Y-up は単一の型付き basis-change に集約。

**採用理由**: split-brain の本質は (a) to/from 連鎖 と (b) world 種別 の 2 つ。(a) は phantom `Transform` の合成規則で、(b) は単一変換関数 + `FitraWorld`/`VmtWorld` タグで両方塞げる。一方ホットパス・wire は値が 1 つ落ちる leaf で、命名規約 + 境界変換で足りる。SE(3) は「合成の鎖」が命なので型代数の利得が最大、そこに限定して churn を抑える。

## 採用設計

### 不変条件（設計の軸）

**fitra Z-up が内部の唯一の world フレーム。** VMT Y-up は (1) extrinsic 入力（controller pose）, (2) VMT publisher 出力 でしか現れない。`Transform<..., frame::VmtWorld>` が triangulation / kalman / tracker_extract に現れたら設計違反。

### コア型（`cpp/src/geom/frames.hpp` — header-only）

frame タグ（空構造体、インスタンス化しない）:
`frame::FitraWorld`（Z-up RH, X-right, Y-forward）/ `frame::VmtWorld`（Y-up RH, X-right, Z-back）/ `frame::Camera`（X-right, Y-down, Z-forward）/ `frame::Marker`（AprilTag face）/ `frame::Controller`（VR controller local）。

```cpp
template <class To, class From>
class Transform {                       // To <- From の剛体変換。内部 cv::Matx44d
  cv::Matx44d m_;
 public:
  Transform() : m_(cv::Matx44d::eye()) {}
  static Transform from_raw(const cv::Matx44d& m);   // OpenCV 境界 escape hatch
  const cv::Matx44d& raw() const;                    // 同上 (剥がしは呼び出し1行)
  cv::Matx33d rot() const;  cv::Vec3d trans() const;
  Transform<From, To> inverse() const;               // 剛体逆 (R^T, -R^T t)
};
// 中間フレーム Mid 一致時のみ実体化:
template <class To, class Mid, class From>
Transform<To,From> operator*(const Transform<To,Mid>&, const Transform<Mid,From>&);
template <class F> struct Point3 { cv::Vec3d v; };
template <class To, class From>
Point3<To> operator*(const Transform<To,From>&, const Point3<From>&);
```

エイリアス: `T_cam_world = Transform<Camera, FitraWorld>` / `T_cam_vmtworld = Transform<Camera, VmtWorld>` / `T_world_controller = Transform<VmtWorld, Controller>` / `T_cam_marker = Transform<Camera, Marker>` / `T_marker_controller = Transform<Marker, Controller>`。

SE(3) ヘルパ（`rot_of` / `trans_of` / `compose(R,t)` / `invert_rigid` / `pose_from_pos_quat` / `rotation_angle_deg` / `average_poses`）は現状 `extrinsic_solver.cpp` の匿名 namespace と test に重複コピーがある。これを `geom` へ昇格し単一実装に集約（集約自体が疎結合化の一部）。raw `cv::Matx44d` を受け取る低レベル形は残し、Transform は内部でそれを呼ぶ。

### 単一 basis 変換（`cpp/src/geom/world_convention.hpp`）

```cpp
// fitra (Z-up,Y-fwd) -> VMT (Y-up,Z-back): (x,y,z)->(x,z,-y) == Rx(-90°)
Transform<frame::VmtWorld, frame::FitraWorld> fitra_to_vmt_basis();   // 唯一の正
Transform<frame::FitraWorld, frame::VmtWorld> vmt_to_fitra_basis();   // = .inverse()
```

### split-brain を型で塞ぐ要点（extrinsic 経路）

solver が消費する controller pose の world は VMT Y-up なので、**solver 出力 `Z = T_cam←world` は `Transform<Camera, VmtWorld>`（= `T_cam_vmtworld`）で型付けする**。同名フィールドが「変換前=VMT / 変換後=fitra」と非対称な点（(b)）が、ここで型として顕在化する:

基底再表現の数式: ある点の fitra 座標 `p_f` と VMT 座標 `p_v` は `p_v = M·p_f`（M = `world_pos_to_vmt` の回転 = 旧 `kVmtWorldToFitra` の行列）。`T_cam_world·p_f = T_cam_vmtworld·p_v = T_cam_vmtworld·M·p_f` より右乗算行列は M。型では M は「fitra 座標を入れて vmt 座標を出す」= `Transform<VmtWorld, FitraWorld>` = `fitra_to_vmt_basis()`。

```cpp
// solution_ (検証シーン用) は VMT Y-up のまま:  FaceSolution/CameraExtrinsic.T_cam_world : T_cam_vmtworld
// 永続 YAML だけ fitra Z-up へ:
geom::T_cam_world T_cw = ce.T_cam_world * geom::fitra_to_vmt_basis();  // Cam<-Vmt * Vmt<-Fitra = Cam<-Fitra
extr.T_cw = cv::Mat(T_cw.raw()).clone();
```

`ce.T_cam_world * geom::vmt_to_fitra_basis()`（向き違い）は `Cam<-Vmt * Fitra<-Vmt` でフレーム不一致 → コンパイルエラー。現状 `kVmtWorldToFitra` の右乗算と数式的に同一だが、向きを取り違えると通らなくなる。`kVmtWorldToFitra` ローカル定数は削除。

### OpenCV 境界の escape-hatch 規約

OpenCV 関数（`solvePnP` / `calibrateRobotWorldHandEye` / `projectPoints` / `undistortPoints` / DLT SVD）を呼ぶ**直前の 1 行**で `.raw()` 剥がし、戻り値を `from_raw` / `Point3{}` で即再ラップ。剥がした raw の使い回しは型安全の穴 → レビュー観点とする。バースト蓄積バッファ（`BurstState`）等の内部 vector は raw `cv::Matx44d` のまま、`ExtrinsicSample` 構築の 1 行でラップ。

### leaf（型を付けない）

`infer::Joint3D` / `Skeleton3D`（ホットパス・JSON・IK が広域に触る、常に fitra Z-up を不変条件で固定）、wire 型 `VmtPos`/`VmtQuat`/`QuatXyzw`、kalman / IK 状態、receiver 生 float。

## Milestone（= コミット境界）

- **M0**: 本 doc + `docs/tracks/pose-3d.md` changelog 1 行。`docs:` コミット。
- **M1**: `geom/frames.hpp` / `geom/world_convention.hpp` 追加、SE(3) ヘルパ昇格（実装移動、API 不変）、新規 `test_geom_frames`。既存コード未接続で build green 自明。`refactor(pose-3d):`
- **M2**: extrinsic_solver / apriltag_marker / extrinsic_calib_session を typed alias 化 + basis 集約置換（split-brain 震源）。既存 test は数値不変。
- **M3**: `Extrinsics::pose()` 型付きアクセサ（`geom::T_cam_world`、fitra Z-up）を追加し Triangulator ctor がそれ経由で R/t を取得。`Extrinsics.T_cw` 自体は `cv::Mat` のまま据置 — calib_io は**任意入力を検証する直列化境界**であり、`validate_calibration` が T_cw の行列サイズ/有限性/同次行を弾く契約を持つ。固定 `Matx44d` 化すると不正ファイルが wrap 時に例外/誤読してこの検証を迂回するため、フィールド型は変えず**消費側で型付け**する（型安全の利得は triangulation が fitra Z-up extrinsic を受けることの保証、直列化境界の堅牢性は維持）。`test_triangulator` 数値不変。
- **M4**: `test_vmt_protocol` に「`world_pos_to_vmt` が `geom::fitra_to_vmt_basis` と一致」のクロスチェック追加（実装据置、回帰ネット強化）。**firmware (Slime) は対象外** — Slime 変換は Unity 左手系 (X-flip, det -1) で剛体 Transform ではなく、`fitra_to_vmt_basis`（正規直交・det +1）と照合する意味がない（残課題の左手系統合に委ねる）。

各 M で `ctest` 全 pass を green ゲート。M1→M4 は独立 revert 可能。

## 検証

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release && cmake --build cpp/build -j
ctest --test-dir cpp/build --output-on-failure
```

- 既存 ctest が数値不変で pass: `test_extrinsic_solver`, `test_apriltag_marker`, `test_triangulator`, `test_tracker_extract`, `test_kalman_chain`, `test_vmt_protocol`, `test_firmware_protocol`, `test_extrinsic_calib_session` ほか。
- 新規 `test_geom_frames`: 合成結合律 / `inverse` 往復恒等 / `fitra_to_vmt_basis` が `world_pos_to_vmt` の R と一致 かつ 旧 `kVmtWorldToFitra` とビット一致 / `Point3` 変換。
- ネガティブコンパイル確認（手順を本 doc に記載、CI 化は任意）: `T_cam_world{} * T_cam_marker{}` がコンパイルエラーになること（フレーム不一致）。
- 実機: extrinsic calib → YAML 出力 → main 起動で検証シーンと live pose が同一フレームに乗る（2026-06-09 リグレッション再現防止）。

### ネガティブコンパイル確認スニペット

```cpp
// 以下はいずれもコンパイルエラーになるべき（型でフレーム不整合を弾く）:
geom::T_cam_world{} * geom::T_cam_marker{};      // Cam<-Fitra * Cam<-Marker  : Mid 不一致
geom::T_cam_vmtworld{} * geom::fitra_to_vmt_basis();  // Cam<-Vmt * Vmt<-Fitra... は OK。逆向きが NG
```

## 残課題

- `Skeleton3D` / `Joint3D` の `Point3<FitraWorld>` 化（ホットパス影響評価が前提）。
- `Rotation<Frame>` / quaternion wrapper の導入（今回は SE(3) と位置のみ）。
- Slime の X-flip（Unity 左手系 = det -1）は剛体 Transform 枠外。basis 体系への統合可否は別途。
- 将来 floor-AprilTag SfM 経路（research 案D）を入れる際、solver の world タグが VmtWorld 固定では合わない → solver を world タグでパラメタ化する余地。
