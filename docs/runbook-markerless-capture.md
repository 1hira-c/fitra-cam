# マーカーレス外部校正を後から検証するためのまとめ撮り

人物がいる機会に原本をまとめて収録し、人物不在でも固定intrinsicからの相対校正を再実行する。
intrinsicは従来どおりChArUco。新しい校正方式の実用品質を証明した撮影条件ではなく、
最初の検証に必要な多様性・時刻・独立評価区間を取りこぼさないための収録手順である。
実装と到達点は[設計書](design/pose-3d-markerless-extrinsic.md)を参照。

## 必要な映像と長さ

**標準は6分10秒を1セット**。移動できる共通画角内で、1人が画面または端末の案内に従う。
製品として目標にしている1回2〜3分の校正とは区別し、このまとめ撮りには独立評価と反復も含める。
固定姿勢を長く撮るだけでは位置や奥行きの多様性が増えない。数学的な最小対応点数やsolverの
6 set下限を、必要な録画秒数と読み替えない。十分な長さの実機確認はこれからであり、採用できた
関節対応・独立した静止姿勢・視差・holdout・繰り返し精度から判定する。

| 区間 | 長さ | 撮る内容と目的 |
|---|---:|---|
| 準備 | 10秒 | 全cameraの頭頂〜足元と明るさを確認 |
| 同期確認動作 | 10秒 | 間隔を変えて片腕ずつ上げて止める。後の映像照合用 |
| 校正A | 96秒 | 中央・左・右・手前・奥・斜めの6位置×2姿勢。各8秒の前半3秒が移動猶予、後半5秒が静止 |
| 独立検証B | 64秒 | 別の時刻・少し違う位置と腕配置・向きで8静止。Aの隣接frameを検証用にしない |
| 再現性C | 96秒 | 逆順の6位置×2姿勢。A/Bとは分けてもう一度解き、相対配置の再現性を見る |
| 将来の実寸・床用 | 40秒 | 4位置で直立・両足接地・頭頂/つま先/踵を可視にして各10秒 |
| 動作確認 | 40秒 | ゆっくりした斜め歩行、横移動、向き変更、片腕の動作。位置を止めるだけで良く見せていないか確認 |
| 終了時の同期動作・退出 | 14秒 | 時間をまたぐずれと終了を確認 |

時間が限られる場合は最初の**3分でA+Bまで**撮れる。ただし途中終了として記録し、C・床用区間が
欠けたセットを標準撮影完了とは扱わない。再現性と将来の床・実寸評価まで残すなら最後まで撮る。

### 画角と動き

- 同じ1人を全cameraで同時に撮る。頭頂、肩/腰、肘/手首、膝、足先/踵が見え、人物が十分な大きさで映ること。
  初版の3camera復元にはanchor pairと3台目で共通の関節対応が必要。
- 左右だけでなく手前/奥にも動く。腕を同じ平面に揃え続けず、片腕を前に出す・体を斜めに向ける等で
  奥行きと姿勢の多様性を作る。狭い部屋では半歩でよく、見切れる場所まで移動しない。
- 止まる区間では足と体を止める。camera非ハード同期のため、速い腕振り・連続回転・ジャンプを主な校正入力にしない。
- 途中でcamera、レンズ、ズーム、フォーカス、解像度/FOVを変えない。設置を変えた比較は**別セット**で記録する。
- 頭頂を隠すHMDや帽子、足元を隠す物を避ける。将来の身長・床用は裸足または薄い靴下を推奨し、靴の条件をメモする。
- 人物の入れ替わり、遮蔽、照明変更などの難例は正常なA/B/Cへ混ぜず、追加セットとして撮る。

## 撮影前に残す情報

- 同じレンズ/FOVに対応するintrinsic。解像度が同じだけではFOVの一致を保証しない。
- 分かれば実測身長を`--height-m`へ入力する。身長をscale推定に使った場合、その同じ身長との一致は独立した精度証明にはならない。
- 同じ設置・レンズ・画角で別撮りした参照校正がある場合は、取得日時と対象セットを記録し、
  比較用に`--reference`でコピーする。過去の配置の外部校正や床の既知点は採点から除く。
- 独立した実寸確認を行う場合は、任意でその設置のカメラ間距離や検証物の長さを実測してメモする。
  これは開発評価用で、通常のマーカーレス導入の必須操作にはしない。画像上でどの地点か識別できるようにする。
- [撮影メモ](templates/markerless-capture-notes.md)をセット名と一緒に保存する。身長・履物・移設の有無・照明条件を残す。

## 収録コマンド

既存のカメラ設定をそのまま読む。**`cameras`節だけ**を読み、推論・出力・アップロードは起動しない。
旧設定のSlimeVR等の項目があっても録画の障害にはせず、camera節の未知keyや不正値は拒否する。
相対パスは通常運用と同じ作業ディレクトリ基準。出力解像度に合うintrinsicを明示する。

```bash
# 案内だけ。cameraを開かず、ファイルも変更しない。
./cpp/build/tools/markerless_record

# 手元の設定とintrinsicのID・寸法を確認する（撮影しない）。
./cpp/build/tools/markerless_record \
  --config configs/medium_3d.yaml \
  --intrinsics calibrations/intrinsics_640.yaml

# 撮影できる時に、通常のcamera所有プロセスをユーザーが停止してから実行する。
# 1セットを一度に撮る。既存出力ディレクトリは上書きしない。
./cpp/build/tools/markerless_record \
  --config configs/medium_3d.yaml \
  --intrinsics calibrations/intrinsics_640.yaml \
  --out outputs/markerless-capture/take-01 \
  --record

# 分かる場合だけ追加: --height-m 1.70 --reference calibrations/extrinsics/latest.yaml
```

端末に各動作を日本語で表示し、切替時に端末ベルを送る。ベルが鳴らない環境では画面を見られる位置へ置くか、
補助する人に案内を読んでもらう。自動の人物認識・静止判定はまだないので、身体が見切れた時はメモする。
まず人物不在で10秒ほど試写してCtrl-Cで止め、後述のQCで全cameraの時刻・受信を確認しておくこともできる。
この試写は意図どおり`recording_complete=false`となり、標準セットのQCは通らない。

撮影は同じ設定でも通常runtimeと負荷が異なる。`nvjpeg`設定でも収録は原MJPEGのままで、GPU decodeや推論はしない。
`assist`露出のfeedback loopは走らず、設定された初期露出/gainをmanualで使用する。適用の成否や実際の露出値を
実測したとは扱わない。明るさとブラーは試写または通常viewerで確認する。

### 容量

既定のraw上限は32GiB、最低空き容量は2GiB。上限/空き不足/3秒超のcamera停止で途中終了し、
既に記録した原本は保持する。遅い書き込みはlatest-frame-winsの欠落として計数する。
撮影を優先してlocal SSDへ保存し、撮影中のNAS転送を必須にしない。

例として3camera×60fps×370秒×平均150KiB/JPEGならrawは約9.5GiB。
これは**未測定の容量例**。実サイズは画質・場面・driver設定で変わる。
YUYVはさらに大きく、1280×960×2byte×60fps×370秒だけでも1camera約50.8GiBとなるため、
長いまとめ撮りにはMJPEG系を推奨する。必要なら`--max-gib`と`--min-free-gib`を明示する。

## 保存されるものと撮影後の確認

`manifest.json`、`plan.json`、実際の案内開始時刻を持つ`cues.jsonl`、
原camera payloadの`cam<N>.mjpeg`/`.yuyv`、frame別offset・size・source/driver seq・
kernel capture時刻/semantics・host DQBUF時刻の`frames.jsonl`、intrinsicの実体コピー、
任意のreference、全ファイルのSHA-256を残す。64bit整数はdecimal stringで保存し、
unknown kernel時刻をhost時刻や平均FPSから作り直さない。
異なるraw/output解像度は両方記録する。後処理は記録時のfull-frame resizeを再現し、別FOVを暗黙補正しない。

```bash
# 原本セットの外へ、新規のQC結果を書き出す。
python3 cpp/tools/check_markerless_capture.py \
  outputs/markerless-capture/take-01 \
  --output outputs/markerless-capture/take-01.qc.json
```

- checksumとpayload範囲、camera数、frame数、時刻の欠落/逆行、SOE/EOF混在を検査する。
- 既定20ms以内の組だけをsource再使用なしで組み、skew median/p95/max、採用率、取りこぼし、camera別の欠落/実FPSを報告する。
  採用率の分母は各cameraの最小frame数であり、runtimeの`sync_miss`とは別指標。
- 各静止区間の移動猶予後と末尾に0.5秒の余白を置き、時刻が揃うframe setを探す。
  36静止区間から各2組、最大72組を再推論候補にし、fit/validation/repeat/floorを明示的に分ける。
- `capture_qc_pass=true`は保存・時刻・予定区間の確認に限る。**人物可視性、静止、多様性、幾何、実寸・床の合格ではない**。
  欠落区間があれば撮影者がいる間に確認して補撮する。原本bundleへQCファイルを追加してchecksumを壊さない。
- exit 0は収録QC合格、2は不完全または時刻/区間不足、1は壊れたファイル等のエラー。どの場合も原本を消さない。

`.mjpeg`はJPEG elementary streamとして閲覧できる。MP4等へ閲覧用変換する場合も原本とsidecarを残す。
変換した動画の平均FPSだけを同期・実dt・露光時刻の根拠にしない。

## 後日の再校正・検証

QCの`candidates`はraw frame位置とpartitionを持つ候補一覧で、2D関節はまだ含まない。
**現段階では`markerless_extrinsic_replay`へ直接渡す入力ではない**。後段で同じ原frameをdecode/resizeし、
選択したモデルのhash・前処理・detector cadence等を付けたraw 2D観測を別ファイルへ生成する。
そのadapterと人物ありの静止・coverage判定は後続実装。原映像を残すためモデルを変更しても再撮影が不要になる。

Aから相対校正し、Bの観測を初期化/refineへ混ぜずに検証する。Cでも独立に解いて相対配置を比較する。
現在のsolverの5番目ごとのholdoutだけでは動画内の隣接frame混入を防げないので、区間のpartitionを維持する。
短い部分集合を段階的に増やし、成功率、poseの再現性、独立Bの誤差が安定するかを調べて、必要長を詰める。
同じ設置の独立参照がある量について、カメラ間の回転と距離、床の傾き・高さ、Bで復元した長さや位置の
誤差を記録する。参照のない量は未測定と明記し、AとCから得た配置のばらつきは再現性として記録する。

背景: [Human Pose As Calibration Pattern (2018)](https://openaccess.thecvf.com/content_cvpr_2018_workshops/w34/html/Takahashi_Human_Pose_As_CVPR_2018_paper.html)は人体関節を共通参照として扱う。
[CasCalib (2024)](https://arxiv.org/html/2405.06845v1)は周期運動の時刻合わせの曖昧さや身長仮定の影響を示している。
本手順の370秒はこれらの論文が保証する必要長ではなく、fit/評価/反復を確保する本プロジェクトの初期予算である。
