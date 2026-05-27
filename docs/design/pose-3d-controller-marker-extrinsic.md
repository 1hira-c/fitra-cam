# pose-3d: コントローラ固定 AprilTag による多カメラ extrinsic キャリブ

(着手日 2026-05-27 / 上流 = pose-3d トラック、原点合わせで vr-output トラックと接続)

> ステータス: **検討中 (PoC 前)**。実装着手前の設計記録。数値根拠の一部は PoC (M0)
> で実測して確定させる。
>
> 主要参考: **Sensors 26(8):2285 (2026)** "Robot-Driven Calibration and Accuracy
> Assessment of Meta Quest 3 Inside-Out Tracking (TECHMAN TM5-900)" — Touch Plus を
> コボット端末に剛体固定し robot FK を真値に **本 doc と同型の AX=ZB** で Quest 3 を実測。
> 方法論・精度の直接の下敷き。

## 背景 / 動機

multi-view で 3D を起こす / 各カメラの単眼結果を共通 frame に載せるには、カメラ間の
extrinsic (相対 6DoF) が要る。現状リポジトリにカメラ extrinsic キャリブの仕組みは無い
(`TrackerExtractor` 系は単眼 lift/IK)。VR 原点合わせは vr-output の HMD-Procrustes
(2D yaw+xyz + 手動 Y) で済ませているが、これはカメラ rig の幾何そのものは与えない。

**狙い**: 各カメラの 6DoF を共通 frame で精度良く確定する。VR 原点ズレ (SLAM ドリフト)
はユーザーに飲んでもらってよいが、**カメラ間相対位置はできるだけ正確に**したい。

完了条件: 全カメラの extrinsic が再現性よく求まり、PoC で測った Quest 静止精度の実力値
から「達成可能な extrinsic 精度」が定量化され、運用手順 (静止多点・インターリーブ取得)
が固まること。

## 検討した案

### 案A: でかでか床 ChArUco を全カメラで共視 (静止一発) — 没 (実環境で不成立)

- 利点: VR 不要・hand-eye 不要・同期不要。PnP 誤差のみ。floor plane と up がタダで付く。
  メンテナ・ユーザーともに概念が単純。
- **没理由 (実測)**: 被写距離 1.2m で **A3 / 5×7 ChArUco は全く認識せず**、**A2 に拡大して
  ようやく 2 カメラ認識がギリギリ**。距離での **px/marker 不足**で ArUco の bit デコードが
  落ち、ChArUco corner 内挿が走らない (4×4 辞書で marker ≥ 約 15px 必要、A3 の marker は
  この距離で閾値下)。さらに **2 カメラ同時認識が事実上不可能** = 共視前提が崩壊。
- スケール面でも不利: board 寸法 = ワールドスケールなので大判印刷の寸法誤差が直行する。
- 個人用の最高精度オプションとしては温存可 (静止・floor 付き) だが、共視不可の環境では
  default にできない。

### 案B: 中サイズ ChArUco を手持ちで振り回す + bundle adjustment — 没 (同上の距離負け)

- 利点: 大判印刷不要、Anipose / multical 等の既存 OSS フォーマットに乗れる、VR を
  extrinsic チェーンに入れない (Quest 誤差ゼロ)。
- **没理由**: 案A と同じ距離 px 問題に加え、BA は**ペアでの (時間をまたいだ) 共視**が前提。
  本環境は「2 カメラ同時認識不可能」なので、共視経由でカメラを繋ぐ経路自体が成立しない。
  ほどほど board を遠方から、という中間案は実データで脱落。

### 案C: コントローラ固定マーカーを各カメラの近くに持って行く + VR world で連結 — **採用**

- 距離問題を**サイズではなく近接**で解く。小さいマーカーでも近ければ px が足りる。
- 2 カメラ同時可視が不要 (後述の連結原理)。これが本環境の決め手。
- 代償: **VR world を連結基準に使う = Quest tracking 誤差が inter-camera extrinsic に乗る**。
  案B の「Quest を外す」逃げ道は共視不可で閉じた。この劣化は飲む前提とし、運用で絞り取る。

## 採用設計

### 連結原理 (同時可視なしでカメラを繋ぐ)

コントローラに剛体固定したマーカーは「VR world で 6DoF 既知の動く標的」。各カメラは別々の
時刻にマーカーを見ればよい。各サンプル i の剛体鎖を forward に書くと (下の hand-eye と同形):

```
T_cam←marker(i) = T_cam←world · T_world←controller(i) · T_controller←marker
```

`X = T_cam←world` と `Y = T_controller←marker` を hand-eye で解けば、各カメラが同一 world frame
に載る (合成の隣接インデックスが消えるよう逆行列を取る):

```
T_cam←world    = T_cam←marker(i) · Y⁻¹ · T_world←controller(i)⁻¹
               = T_cam←marker(i) · T_marker←controller · T_controller←world
extrinsic(A,B) = T_camA←world ∘ (T_camB←world)⁻¹
```

未知は各カメラの `T_cam←world` と、全カメラ共有の剛体オフセット `T_controller←marker`。
**hand-eye / robot-world (AX=ZB) 形**で、`cv::calibrateRobotWorldHandEye` がそのまま使える。

### マウントオフセット `T_controller←marker` の扱い

- **実測しない。hand-eye 解の中で同時推定する。** AX=ZB を
  `T_cam←marker(i) = X · T_world←controller(i) · Y` と置くと、X = `T_cam←world` (各カメラ)、
  Y = `T_controller←marker` (全カメラ共有の定数) が同時に出る。`calibrateRobotWorldHandEye`
  (カメラ固定・標的が動く eye-to-hand 変種)。Y は副産物。
- **CAD/手計測案は没**: OpenVR が返すコントローラ pose の tracking 原点は Meta 内部基準で物理
  形状との関係が正確に非公表 → 手計算 Y は不確実。ジョイント推定が厳密に上。
- **観測性 = 回転多様性**。純並進では Y の回転が立たない → 多面タグが自然に稼ぐ。
- **クロスチェックがタダ**: Y は全カメラで一致すべき。カメラ独立解の Y ばらつき = 校正品質指標。
  あるいは全カメラで Y を共有して一括解 (条件数↑)。
- **多面タグはオフセットが面ごと → 面間ジオメトリは CAD を信じず *データで解く*** (採用)。
  各面を独立 marker として自前の `Y_face = T_controller←face_k` を同時推定 → 面間の相対ジオメトリ
  はデータから出る (タグ剛体の self-calibration、多マーカー rig 校正と同じ筋)。3Dプリントを
  分割組みしても CAD 精度もキー付き接合も要らない。**近接運用で各面 PnP が高品質**なので条件は十分。
  - 代償: 未知が面ごとに増える → **各面を十分な姿勢多様性で観測する収集**が要る (1 面に偏らない)。
  - 面間剛性は当初拘束しない (各 `Y_face` 独立解)。復元した面間ジオメトリが**セッション中一定で
    あるべき** = タダの品質チェック。精度が要れば M3 BA で「1 個の剛体タグ + 未知の面 pose 群」
    として一括拘束に格上げ。
- **剛性が唯一の絶対条件**: 各 `Y_face` が一定でないと全カメラの X が汚れる。

### マーカー: 多面 AprilTag (3面・手の甲側・HMD逆)

- **AprilTag 36h11** (`DICT_APRILTAG_36h11`)。ArUco より低解像・遠距離のデコードが頑健。
  **各面は別 ID** (検出器がどの面か判別 → 正しい face オフセットに対応)。
- **3 面をコントローラの手の甲側・HMD と逆側に配置**。理由(重要):
  - **コントローラ自身のトラッキングを潰さない**: Touch Plus は自前 LED で HMD カメラに追われ
    てる(= 我々の基準 pose の源)。LED 面 / HMD 視線側を覆うと**基準が死ぬ**。手の甲=HMD逆へ逃がす。
  - **同時に部屋カメラへ正対しやすい**: 手の甲側は体の外向き。HMD は前面、部屋カメラは背面のタグ、
    で取り合いが分離する。3 面でどの向きでもデコード可能な面が向き、回転多様性も稼げる。
- **サイズは可能な範囲で最大、8〜12cm 想定**(近接運用なので 1.2m でも余裕、大きいほど PnP 良)。
- **印刷サイズの制約はタグと分離して回避**: タグ面は 3Dプリントせず**マット印刷(反射回避)を薄い
  平板に貼る**。3Dプリンタの仕事は**小物のみ** — コントローラ用クランプ + 3 面をつなぐハブ +
  (必要なら)薄い裏当て。全てビルドボリュームに収まり、**タグの大きさがベッドサイズに縛られない**。
- マウント**剛性が最重要** (オフセットの扱いは上記サブセクション参照)。手持ちなので**軽量・てこ短め**
  (大きく/長くするほど重い + しなりで剛性条件に反する)。サイズ最大化と剛性/疲労の折り合い点。

### 不変条件 / live な制約

- **Quest は extrinsic チェーンに入る** (共視不可ゆえ不可避)。ただし実測 (Sensors 2026) で
  **静止・FOV内・明るい・低速なら並進 3D RMSE ≈ 0.62mm / 回転 ≈ 0.14°** と sub-mm 級。
  **コントローラ pose は律速項ではない**。律速は下記の SLAM ドリフトと我々の PnP / マウント剛性。
- **カメラ間取得は時間的に近接させる** (最重要・見落としやすい・実は最有力の律速)。
  `extrinsic(A,B)` は tA と tB で world が同一という仮定。HMD は VI-SLAM で **原点が緩慢に
  ドリフト + re-localization で離散ジャンプ**し (Quest 3 の厳密なドリフト率は公表値なし)、
  tA→tB のズレがそのまま extrinsic 誤差に直行する。「カメラ0を全部 → カメラ1を全部」は最悪。
  **全カメラをインターリーブで短時間に**取り切る。1 セッション、recenter 厳禁。可能なら
  **静止 HMD の world pose を監視**し、取得中に > ~1mm 動いていないか確認 (動いていれば world
  frame が動いている証拠)。
- **準静止・手持ちで取得 (三脚/完全静止は不要)**: 三脚を各カメラ近傍に置ける保証はないので
  **手持ち前提**。各 pose で一瞬止め、HMD FOV **中央**・適度な一定 **depth**・明るい状態で撮る
  (depth 軸と FOV 周辺が最ノイズ。Quest 2 実測、Quest 3 はコンステレーション拡大で多少緩和)。
  - **モーションゲート**: ①カメラ側 marker corner の移動量 (画像上の静止 = ブラーも同時に弾ける)
    と ②OpenVR velocity が**両方しきい以下**のフレームのみ採用。採用フレームを短バースト平均で
    手ブレ random を潰す。これで **2 クロック跨ぎの時刻同期が不要**になる (動いてる瞬間を捨てる
    ので velocity×lag の誤差が小さい。手ブレ数 mm/s × lag 数 ms = sub-mm)。
  - **手持ちはむしろ有利**: (a) 回転多様性が手で勝手に稼げて hand-eye 条件数↑、(b) 三脚立て直し
    より速くカメラ間をインターリーブでき「時間近接取得」不変条件に追い風。純粋な妥協ではない。
- **世界 frame は既存 HMD 経路と同じ Standing universe + VMT frame を踏襲** (当初案の
  `TrackingUniverseRaw` は不採用)。`hmd_pose_receiver.hpp` の pose は既に `world_*_to_vmt` を
  通した **SteamVR Standing (Guardian 床基準)・Y-up RH・metres**。hand-eye は frame が内部
  整合してれば成立するので Raw に拘る必要はなく、Standing は**床基準 up がタダで付く** (floor
  plane の旨味) ぶん有利。**条件: セッション中に recenter しない** (Standing は再レベリングで
  原点が飛ぶ ← 既出の「時間近接・1 セッション」不変条件と同じ)。
- **OpenVR 取得は送信側 (`vmt_manager`) で `fPredictedSecondsToPhotonsFromNow = 0`** (予測誤差を
  消す)。静止取得なので Link/Air Link の遅延は空間誤差に化けない。
- **取得サンプルのゲート**: **`bPoseIsValid && eTrackingResult == Running_OK`**。現スキーマは
  `valid(i32)` のみ持つので **`eTrackingResult` を wire に 1 フィールド追加が必要** (valid だが
  劣化中=extrapolated/occluded を弾けない)。FOV 外 / ロスト時は 1s 未満で cm 級に飛ぶため、
  ドリフト定数でモデル化せず status で弾く。静止判定 (velocity≈0) は Jetson 側で連続 pose の
  差分でも代替可 (velocity field 送信は任意)。
- **マーカー寸法スケールは VR metric から逆算固定** (1-DoF・多サンプル平均で頑健)。3Dプリント
  / 印刷の寸法誤差に免疫がつく。
- **カメラ intrinsics + 歪みの事前キャリブが前提** (PnP に必須、別工程)。**手法は ChArUco**
  (`calibrateCameraCharuco` 系)。extrinsic で ChArUco が死んだのは「1.2m 遠距離・複数カメラ
  共視」が原因で、intrinsics は **1 カメラずつ・近接・画面いっぱい**の真逆条件なので問題なく通る。
  AprilTag PnP はここで得た camera matrix + distortion を消費する (上下流関係)。
- **役割分担**: extrinsic + (可能なら) floor = 本 doc。VR 原点合わせ (妥協可) は vr-output の
  HMD-Procrustes / コントローラ。Quest 誤差を「精度が要る所」には極力入れない設計思想は
  共視不可で部分的に破れたが、原点側は引き続き分離する。

### 誤差バジェット (差し込み値)

| 項 | レジーム内の値 | 確度 | 備考 |
|---|---|---|---|
| コントローラ静止 jitter (FOV内・明) | 0.3–0.6mm / 0.15° RMS | 高 (Quest 3 直接実測) | 実用マウント/動きで ×2–3 → ~1–2mm を見込む |
| depth 軸 / FOV 周辺ペナルティ | 〜1cm | 中 (Quest 2) | 中央・一定 depth で回避 |
| FOV 外 / ロスト | 1s 未満で cm 級、定数なし | 低 (ギャップ) | status ゲートで弾く・取得しない |
| **SLAM 原点ドリフト** | **mm級/分 + 離散ジャンプ、Quest 3 公表値なし** | **低 (ギャップ)** | **最有力の律速** — 短時間1セッション・recenter禁・HMD pose 監視 |
| Link/Air Link (静止・予測0) | ≈0 空間誤差 (遅延 ~55–80ms のみ) | 高 | `fPredicted=0` / `TrackingUniverseRaw` |
| AprilTag PnP (我々) | 近接で sub-mm〜数mm | — | M0 で実測 |

結論: **律速は「Quest コントローラ精度」ではなく「SLAM 原点ドリフト」と「我々の PnP / マウント
剛性」**。設計努力はこの 2 つ (時間近接取得・剛性・タグ近接) に集中する。

### 本番三角測量への影響なし

「マーカー同時可視不可」≠「人物三角測量不可」。人物は小マーカーより遥かに大きく多くの
カメラに同時に映るので、Quest-linked マーカーで extrinsic を確定 → でかい被写体 (人物) を
普通に三角測量、で分離成立。校正ターゲットの共視失敗は本番を巻き込まない。

### セットアップ UI (収集フィードバック + 結果体感)

既存 WebUI (Crow + three.js AxesHelper + per-camera Canvas + JSON/WS publisher) に**校正モード**を
追加する。手持ち準静止は**ライブフィードバックが収集成功率の一部**(いつ止まってるか・どの面/
カメラが足りないかが見えないと回らない)。

**コア (これが無いと collection が回らない / M2)**

1. **per-camera ライブ + AprilTag 検出オーバーレイ**: 各カメラ画像に tag 四隅 + face ID + デコード✓。
   「今このカメラが tag を見えてるか」が一目。publisher schema に tag 検出を 1 ブロック追加 (互換維持)。
2. **ゲート状態インジケータ (最重要・glanceable)**: `NO TAG` / `FOV外 or !Running_OK` / `MOVING` /
   `GOOD→撮影` を理由付きで大表示。手持ちで「今が撮り時」を operator に伝える唯一の窓。
3. **被覆マトリクス**: 行=カメラ × 列=face のサンプル数 + 姿勢多様性充足を色表示。「次どこを撮るか」。

**セッション健全性 (警告系 / M2)**

4. recenter 検出 or 静止 HMD pose のドリフト超過を警告 (world frame が動いた証拠)。短時間取得を
   促すセッションタイマー。

**3D 検証シーン (完全有料・コア後 / M4)**

5. HMD + コントローラ + **各カメラ (intrinsics から frustum を起こし `inv(T_cam←world)` に配置)** を
   同一 world frame に描画 → カメラが部屋のどこにあるか体感。HMD/コントローラは受信 pose にモデルを当てる。
6. **リアルタイム漸進 solve**: hand-eye 解は ms オーダーで compute 上リアルタイム可。律速は
   observability (回転多様性が溜まるまで解けない) → カメラは**解ける状態になった瞬間 pop-in、以降
   収束**。明示的な `recompute/preview` ボタンも併設 (両取り)。
7. **結果体感 (ハイライト)**: solve 後、各カメラの tag 観測を world に投影
   (`T_world←marker = T_world←cam · T_cam←marker`) し、VR 真値 (`T_world←controller · Y_face`) と
   並べて描画 → **ズレ = ライブ残差を目視**。コントローラを振ると「カメラの意見」が「VR真値」に
   吸い付く。**カメラ毎に色分け**で「どの 1 台が mis-calib か」を即診断 (多カメラ診断がタダ)。
   - コスト: live solver をループに組込 + extrinsics/品質を web publish + schema 拡張 + 全要素が
     単一 world frame (`world_*_to_vmt`) に乗る規約厳守 (ミスると全部それっぽくズレる)。コア校正を
     これでブロックしない。

## Milestone

- **M0 (PoC / 設計確定の前提)**: コントローラ静止精度は Sensors 2026 で sub-mm と既知なので、
  M0 が測るべきは**未計測の 2 項** — (1) **我々の AprilTag PnP 残差**(1 カメラで PnP pose vs
  コントローラ pose、近接・静止)と、(2) **このセッション環境での SLAM 原点ドリフト**(静止 HMD
  pose を数分監視し world frame の動きを実測)。加えて **(3) 手持ち準静止時の残揺れ**(velocity /
  corner 移動量の分布)を測り、モーションゲートのしきい値を裏付ける。この 3 つで案C の現実的な
  精度天井を確定し、本設計のしきい(サンプル数・許容ドリフト時間窓・ゲート閾)を裏付ける。
- **M1**: 多面タグマウントの 3D モデル + intrinsics キャリブ手順 (ChArUco 近接・カメラ毎、
  `calibrateCameraCharuco`)。**コントローラ pose の受信経路**
  — 既存の VMT alignment 用 HMD pose 経路 (`vmt_manager` → UDP/OSC → `HmdPoseReceiver` /
  `HmdPoseBus`, 既存 frame 変換ごと) を流用する。具体:
  - **transport は丸ごと再利用**: `/fitra/controller_pose` を並列にもう 1 本生やし、`HmdPoseBus`
    と同型の latest-wins bus に載せる。Windows 側は別 device index を query するだけ。
  - **スキーマ拡張**: 現 `,iffffffff` (valid + ts + pos + quat) に **`eTrackingResult` を 1 個追加**
    (上記ゲートのため)。座標系は HMD 経路と同じ Standing+VMT frame をそのまま使う。
  - 同期は静止取得ゆえ latest-wins + stale で十分 (timestamp は厳密ペアリング不要)。
- **M2**: 準静止・インターリーブ取得の収集 UI (**全カメラ × 全 face を姿勢多様性込みで被覆**、
  モーションゲート + バースト平均) + 各面 `Y_face` 同時推定の `calibrateRobotWorldHandEye` 一括校正。
  再現性 (繰り返し校正の extrinsic ばらつき)・Y_face 一致・面間ジオメトリ一定性を評価。
- **M3 (任意・精度上限狙い)**: 全サンプル (per-camera PnP + VR world 拘束) を小さな BA に
  放り込み、取得時間ズレ / ドリフト・剛体タグ拘束を重み付きで大域分配。M2 で不足なら着手。
- **M4 (任意・完全有料 / コア後)**: 3D 検証シーン (上記セットアップ UI の 5〜7) — カメラ frustum
  表示・リアルタイム漸進 solve + preview ボタン・カメラの意見 vs VR真値の重畳で校正結果を体感。
  live solver のループ統合 + publisher schema 拡張が要る。コア校正完了後の UX 投資。

## 検証

- M0: PnP pose vs コントローラ pose の残差統計 (静止時)。目標は「現行 HMD-Procrustes より
  良い」ことの定量確認。
- M2: 同一 rig を複数回校正した extrinsic の分散 / 既知配置との突き合わせ。
- 実機: WebUI three.js シーン (既存 AxesHelper×10) にコントローラ pose とカメラ推定を重ねて
  目視残差確認 (vr-output の可視化資産を流用)。

## 残課題

- 案C で extrinsic が Quest 精度に縛られる件、M0 の実測値次第では「案A (でかでか床) を
  個人用最高精度パスとして併設」する tiered 構成を再検討。
- OSS 公開時の依存方針: 校正ソルバを既存 OSS (Anipose / multical) フォーマットに寄せるか
  自前 BA か (M3 と連動)。
- VR 原点合わせ (vr-output) との統合 API の切り分け。
- SLAM ドリフトの Quest 3 厳密値が文献に無い → M0 の自前実測で埋める (上記 M0-(2))。
- **連続運動フォールバック**: 準静止ゲートでサンプルが集まらない場合、カメラフレームに capture
  timestamp 付与 + pose リングバッファ + frame 時刻への SLERP 補間 + Jetson↔Windows クロック
  オフセット推定 (or 同時推定) で緩い連続運動をサンプル化。現 transport は latest-wins 単スロット
  なのでバッファ化が必要 = 工数増。任意・M3 寄りに置く。

## 参考

- Sensors 26(8):2285 (2026) Robot-Driven Calibration and Accuracy Assessment of Meta
  Quest 3 Inside-Out Tracking (TM5-900) — <https://www.mdpi.com/1424-8220/26/8/2285>
  (Quest 3 静止 FOV内: 並進 3D RMSE 0.62mm / 回転 0.14°、AX=ZB 同型手法)
- Oculus Quest 2 locomotion validity vs Vicon (T&F 2023): 静止 abs-max ~1cm/1°、動的 ~19cm/14°
- OpenVR `IVRSystem::GetDeviceToAbsoluteTrackingPose` (予測秒・universe origin・TrackingResult)
