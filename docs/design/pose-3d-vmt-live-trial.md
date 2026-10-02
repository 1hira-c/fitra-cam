# pose-3d: 距離filterのVMT体感試験

2026-09-22。ユーザー依頼でoffline比較からlive試験へ進む。
前段: [距離残差実験](pose-3d-distance-filter-experiment.md)。既定mainへの採用ではない。

## 目的と基準

手はコントローラで追う前提。腰・脚の通常動作でE2E 100ms以内、Jetson側50ms以内を
暫定予算とし、許容内で位置揺れを抑える。知覚の一般則や達成済み測定値とはしない。
微小低速対照は残すが、数mmの位置差を時間換算した大きなms値だけで候補を落とさない。
実機の開始/停止/切り返し、静止、前屈と腿twistを別々にユーザー評価する。
capture-to-sendの時計差とfilterの追従遅れ、Windows/表示は別測定。

## 実装境界

- 通常`main`、runtime既定、設定schema、モデル、校正原本、D50 wireは変更しない。
- 独立tool `temporal_vmt_trial --config TRIAL.yaml`を明示起動する。通常サービスとの同時起動はしない。
- 既存capture/TRT/triangulatorとVMT publisher/relay/aligner/Crowを再利用。
  このプロセスの上流Kalman/IK/floorをOFFにし、raw専用busを数値候補へ1回/report入力する。
  同じ候補内でIKまたはsoft骨長→床→抽出→gate→距離filterを実行。二重補正しない。
- offlineのCandidate本体を共通headerへ抽出し、保存済みreplayとの数値一致を確認する。
- `minimal / spatial / distance2 / distance5`をstdinで切替。spatialは`soft_bones_head_leaf`。
  2026-10-02に`guard5`を実験候補として追加した（raw関節の棄却/50ms補完→distance5）。
  [関節外れ値実験](pose-3d-joint-outlier-experiment.md)を参照。旧4候補の体感結果と混ぜない。
  切替時は全候補の履歴を捨て、次の新しいreportからseedする。各切替後2秒は体感採点から外す。
- 入力はlatest-only。boundary FIFO・identity/epoch・時刻gapでreset。
  同じsnapshotを再更新せず、受信時刻基準のageが100msを超えた入力は出力無効化する。
- 欠損をVMTが永久holdしないようこのtoolはdegeneracy=disableを強制する。
  invalid poseも明示disableとして送信し、終了時にも短時間disableを送って停止する。
- WebUIとVMTは同じ選択候補のTrackerBusを参照。rawと補正後のbusは別にする。
  実験toolはFusionPose/TrackerAxis公開busを接続しない（実験補正を通常契約の実測証拠にしない）。
- モデル/校正/profileは起動中固定、alignmentは候補切替と独立に維持。
  初回配置合わせ後、比較中はcontinuous alignmentを止めて交絡を減らす。

## 非採用案

通常mainへの既定ON化、独立Python再実装、UDP送信直前だけの二重filter、
再起動ごとの再alignment、無制限rawログI/Oをhot pathへ追加する案は採らない。
通常TrackerExtractorと実験adapterを同時producerにしない。

## 検証・記録

shared Candidateのoffline回帰、live adapterの重複/境界/stale/reset/切替テスト、
VMT/HMD/alignerの既存test、起動/送信/無効化/停止を順に確認する。
通常mainに対する体感同一性やE2E合格はbuild/CTestだけでは主張しない。
ユーザー準備→基準確認→spatial→2mm→5mm→必要なら基準再確認の順。
数字のGTと体感コメントは別欄に残す。初回の試験結果は追記する。

### 2026-09-22: ソフト準備完了、実機は翌日へ延期

- Releaseでtrial/replay/live-state testをビルド。新規state/help 2件、既存VMT/HMD/
  discovery/alignment/config 8件、temporal_eval 1件（Python 128件）のfocused CTestがpass。
  full CTestは未実施。ネットワークtestはloopbackでありWindows受信確認ではない。
- 保存済み静止1185 reportの10候補をshared Candidateで再計算し、`processing_ms`を除く
  全JSONが旧結果と一致。filter演算の抽出による数値変更はない。
- 実機プロセスは未起動。GPU/camera起動、HMD戻り、配置合わせ、VMT受信、欠損時disableと
  停止時disable、体感品質、E2E予算は未検証。翌日ユーザーと確認する。
- ローカルの`outputs/temporal-eval/20260922-vmt-live-trial/`に試験YAML・再計算結果・手順を保存。
  校正は同日rescue-v2の候補を参照し、正式校正へ昇格しない。初回は配置/床の一致から確認する。
- 通常main/binary・設定・校正原本は変更しない。trialのWebUI rolling tracker statsは未計測
  （window=0）、上流floor statsも補正後の品質指標には使わない。出力時刻差のEMAは
  filter遅れ・Windows/表示を含むE2Eではない。4候補同時計算の負荷も本番単一候補と区別する。

### 2026-10-02: 関節棄却/短期補完候補を追加、再体感は未実施

`guard5`は`distance5`の前段にraw3D関節の採否を置く。補完は採用観測から最大50ms、
棄却値や補完値で速度/期限/接地/足anchorを更新しない。無入力時のreceipt-clock失効も別に行う。
旧候補の数式は維持し、既定は従来のminimal。5候補の同時計算負荷を本番単一候補/E2Eと混同しない。
固定rawと人工対照で数値比較するが、今回カメラ/VMTサーバー起動やWindows送信はしていない。
通常設定への採用、前後逆転の恒久対応、腿twistの回転ロジック変更は対象外。
詳細: [アルゴリズム・検証結果・残件](pose-3d-joint-outlier-experiment.md)。

### 2026-09-23: Jetson実機のlocalhost事前確認

- `30294f3`のtrial binaryで3 USBカメラ1280x960 MJPGと既存RTMPose-M FP16/YOLOX-Sを起動。
  カメラ受信は約59.2–60.0fps、4候補の採取末尾の3D更新は約53–56fps。
  rescue-v2校正候補・既存subject profileを正常ロード。校正の実空間一致は未受入れ。
- Web/HMD bind・VMT宛先をすべて`127.0.0.1`へ固定しdiscoveryをOFF。HTTP 200と
  localhost限定listenを確認。Windowsへの送信・HMD擬似入力・録画は行っていない。
- minimal/spatial/distance2/distance5をstdinで切替し、各8秒のUDP受信は481 bundles、
  約60Hz、P8固定index、finite pose、単位quaternion（norm²誤差最大2.8e-7未満）、schema異常0。
  各区間の人物の姿勢/位置は統制していないため、disable率や候補品質の比較には使わない。
- `quit`終了時の最後5 bundlesが全enable=0、正常終了とHTTP/HMDポート解放を確認。
  その後minimalでlocalhost待機用に再起動。停止時無効化はローカル受信までの証拠。
- 採取末尾のcapture-to-send EMAは約37–45ms。分位値でもfilter遅れ込みのE2Eでもなく、
  Jetson50ms/E2E100msの合格判定には使わない。Windows/HMD/配置/体感は引き続き未確認。
- 設定・集約のみのプローブ結果・ログは`outputs/temporal-eval/20260923-vmt-preflight/`。
  通常設定・校正原本・production codeは変更なし。Windows接続時だけ試験YAMLへ切り替える。

### 2026-09-23: Windows接続と最初の体感比較

HMD戻りは届いたがdiscovery広告は受信0で、最初はtracker送信0だった。
ユーザー確認済みのWindows宛先を試験YAMLへ手動指定し、送信とfreshな実HMD戻りを確認。
以下は順番を告知した短時間の主観比較であり、blind比較・定量GT・E2E測定ではない。

| 候補 | ユーザーの体感 |
|---|---|
| minimal | 歩行で足が滑る感じはない。足踏みも極端に悪くないが、常時ガタつく |
| spatial | minimalから体感できる変化はない |
| distance2 | 少し改善。ただし時々カクッと跳ねて戻る挙動が結構ある |
| distance5 | かなり良くなった。遅延もそこまで感じない |

distance5を試験中の暫定維持候補とし、通常既定への昇格はしない。
追加の小さい左右移動→停止でも、ユーザーは「多少の遅延はあるが極端な後追いは感じず、
手足と同じような遅延に感じる」と報告。同時計測ではなく、遅延同値や予算達成の証拠にはしない。
位置filterは連続gainであり閾値外をrejectする処理ではなく、大きい推定スパイクを除去できるとは
主張しない。位置だけを処理し、回転は全候補共通。reportの欠損カウント/有効状態も併記するが、
その瞬間の症状と完全に時刻対応させた解析は未実施。

残件は次の3件として分離する:

- **伸展時の腿twist**: ユーザーは、追従が遅いよりも「勝手にひねる」感覚に近いと明確化。
  さらに軽く膝を曲げた状態で、膝位置の揺れが回転に出ているように感じると補足。
  完全伸展だけでなく浅い屈曲も再現条件へ含める。膝位置ノイズからの伝播は未検証の仮説。
  足滑り・位置ガタつき・遅延と同じ評価項目へ混ぜない。ユーザー合意で今回の回転改修は保留。
  toe参考方向/膝平面の切替、股関節由来の軸誤差、quaternion処理のどれが原因かは未確定。
- **T-pose位置合わせの前後逆転**: API成功後にユーザーが逆転を報告。胸quaternionのbasis変換後yawと、
  胸local +Zを前方ベクトルとして変換したheadingの解釈のずれが疑われる。
  実際に前者の補正は約+178°、後者とHMDの同方向対応は約-3°だった。
  試験runtimeだけ胸の送信位置を保ってyawを180°戻し、ユーザーは「大体合ってそう」と回答。
  continuous alignmentはOFFのまま。API/コードの恒久修正はしておらず、T-pose再実行は避ける。
  胸前方とHMD視線の定義を固定した回帰テストと、実機での移動方向確認を残す。
- **特定姿勢で大きく悪化する可能性**: 停止前にユーザーが「距離filterに極端に弱い姿勢が
  あるかもしれない」と追加報告。姿勢の具体形・再現性・filter ON/OFF差はまだ未確認。
  filter単体に原因帰属せず、当該姿勢の確認と録画replay比較を残す。全姿勢の受入れとはしない。

動作中の試験設定と経過はローカルpreflightディレクトリのREADME/ログに保存。
この段階でも正式な校正受入れ、計測E2E予算の達成、長時間安定性は未確認。
ユーザー了承で試験を終了。`quit`後exit 0、試験プロセス終了とHTTP/HMDポート解放を確認。
終了時disable処理は実行されたが、Windowsでの最終無効化表示は未確認。SteamVR自体は操作していない。
