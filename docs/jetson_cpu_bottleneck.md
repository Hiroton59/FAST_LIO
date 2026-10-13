# Jetson で 1 コアが 100% に張り付く問題の調査と対策

FAST-LIO（ROS 2 Foxy）と Livox MID360 を Jetson で動かしたときに起きた問題の記録です。

## 環境

| 項目 | 内容 |
|---|---|
| ボード | Jetson（L4T R35.3、CPU 6 コア） |
| ROS | ROS 2 Foxy |
| RMW | `rmw_cyclonedds_cpp`（`CYCLONEDDS_URI` で eth1 に固定） |
| LiDAR | Livox MID360（`livox_ros_driver2`、`~/ws_livox`） |
| ドライバ起動 | `launch_ROS2/msg_MID360s_launch.py` |
| SLAM | FAST-LIO（ROS2 ブランチ、`config/mid360.yaml`） |

## 症状

- 1 つの CPU コアだけが 100% に張り付き、他のコアは空いている。
- 処理性能を出し切れていない。

## 調査の経過

### 1. 最初の仮説：FAST-LIO が 1 スレッドで動いている

FAST-LIO のコードを確認すると、次の状態でした。

- 並列化されていたのは最近傍探索（`h_share_model`）の 1 ループだけ。
- `rclcpp::spin` がシングルスレッドで、LiDAR 前処理・IMU 受信・LIO 本体が 1 スレッドで順番に動いていた。

FAST-LIO 側を並列化しました（後述の「対策 1」）。しかし張り付きは解消しませんでした。

### 2. スレッド単位で計測

```bash
ps -eo pid,pcpu,comm --sort=-pcpu | head
top -H -b -n 2 -d 2 -p <driver_pid>,<fastlio_pid>
```

| プロセス / スレッド | CPU |
|---|---|
| livox_ros_driver2 の 1 スレッド | **99.5%** |
| fastlio_mapping（全スレッド合計） | 約 27%（各スレッド 10% 以下） |

張り付いていたのは FAST-LIO ではなく、**Livox ドライバの 1 スレッド**でした。

### 3. どのスレッドかを特定

- スレッドの生成時刻（`/proc/<pid>/task/<tid>/stat` の 22 番目の値）と、ソース上のスレッド生成順を照合しました。結果、点群を publish する `DriverNode::PointCloudDataPollThread` だと判明しました。
- `/proc/<pid>/task/<tid>/status` の `voluntary_ctxt_switches` は約 2 分で 92 回でした。セマフォ待機にほとんど入らず、回り続けていたことになります。つまり publish が 10Hz に追いつかず、キューに常にデータが溜まっている状態でした。

### 4. 原因 1：ドライバが最適化なしでビルドされていた

```bash
grep CXX_FLAGS ~/ws_livox/build/livox_ros_driver2/CMakeFiles/livox_ros_driver2.dir/flags.make
# CXX_FLAGS =   -fPIC -Wall ...      ← -O 指定なし（-O0）
```

`livox_ros_driver2` の CMakeLists.txt は、Release の既定値を **ROS1 用の分岐でしか設定していません**。そのため ROS2 で普通にビルドすると `CMAKE_BUILD_TYPE` が空になり、`-O0` でビルドされます。点ごとのループが多いドライバでは、これが大きく効きます。

### 5. 原因 2：CustomMsg のシリアライズが非常に重い

ドライバは CustomMsg（`xfer_format: 1`）で出力していました。CustomMsg は「構造体の配列」なので、DDS は 1 点ずつ要素単位でシリアライズします。PointCloud2 はバイト列なので、ほぼコピーだけで済みます。

この Jetson で 20,000 点（MID360 の 10Hz 1 フレーム相当）のメッセージを `rclcpp::Serialization` でベンチマークした結果です。

| 形式 | Fast DDS | CycloneDDS（実際に使用） |
|---|---|---|
| CustomMsg | 3.7 ms | **21.4 ms** |
| PointCloud2 | 0.14 ms | **0.15 ms** |

CycloneDDS では、CustomMsg が PointCloud2 の約 145 倍遅いという結果でした。

### 6. 補足：受信バッファが小さい

- `net.core.rmem_max` / `wmem_max` が既定値の 212992（約 200KB）のままでした。
- 1 フレームは CustomMsg で約 400KB、PointCloud2 で約 520KB あり、受信バッファより大きいです。

## 対策

### 対策 1：FAST-LIO の並列化

| ファイル | 変更内容 |
|---|---|
| `CMakeLists.txt` | ARM では「コア数 − 2」を OpenMP スレッド数にする（6 コアなら 4）。`-DFASTLIO_MP_PROC_NUM=N` で上書き可。OpenMP を `REQUIRED` に変更 |
| `src/laserMapping.cpp` | `MultiThreadedExecutor`（3 スレッド）と、LiDAR / IMU / メインの 3 つのコールバックグループを導入 |
| 同上 | LiDAR 前処理をバッファのロック外へ移動。`sync_packages` はロック内で呼ぶ |
| 同上 | ヤコビアン計算、`map_incremental` の追加判定、publish 時の座標変換、ikd-Tree 初期構築を OpenMP で並列化 |
| `src/IMU_Processing.hpp` | 点群の歪み補正を IMU 区間ごとに並列化 |
| `src/laserMapping.cpp` | `map_save` サービスがローカル変数に入っていて、コンストラクタ終了時に破棄されていた不具合を修正 |

### 対策 2：ドライバを Release でビルド

```bash
cd ~/ws_livox
colcon build --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=foxy -DCMAKE_BUILD_TYPE=Release
```

**今後ドライバをビルドするときも、必ず `-DCMAKE_BUILD_TYPE=Release` を付けてください。** 付けないと `-O0` に戻ります。

### 対策 3：点群を PointCloud2 で配信

| ファイル | 変更内容 |
|---|---|
| `~/ws_livox/.../launch_ROS2/msg_MID360s_launch.py` | `xfer_format = 0`（PointCloud2） |
| `~/ws_livox/.../src/lddc.cpp` | PointCloud2 生成時の二重コピーをやめ、出力バッファに直接書き込む |
| `FAST_LIO/config/mid360.yaml` | `lidar_type: 4`（MID360 PointCloud2） |
| `FAST_LIO/src/preprocess.cpp` | `mid360_handler` を書き直し（下記） |
| `FAST_LIO/src/laserMapping.cpp` | MID360 の購読 QoS を reliable（depth 20）に変更 |

`mid360_handler` を書き直した理由：

- 既存版はこのドライバの PointCloud2 形式に合っていませんでした。
  - フィールド名 `reflectivity` を探すが、ドライバは `intensity` で出力している。
  - 点ごとのタイムスタンプを使わず、ヨー角から時刻を推定していた。MID360 は非反復スキャンなので不正確。
- 新しい版は CustomMsg 用の `avia_handler` と同じ処理をします（点ごとの時刻、tag フィルタ、`point_filter_num`、`blind`）。
- `pcl::fromROSMsg` を使わず、PointCloud2 のバイト列を直接読みます。

QoS を reliable にした理由：1 フレームが約 500KB と大きいので、best-effort だと断片が 1 つ欠けただけでフレームごと失われるためです。

## 結果

再起動後、1 コアへの張り付きは解消しました。

## 推奨設定（任意）

CycloneDDS の受信バッファの上限を引き上げます。

```bash
sudo sysctl -w net.core.rmem_max=8388608 net.core.wmem_max=8388608
```

再起動後も残すには、`/etc/sysctl.d/` に設定ファイルを置いてください。

また、Jetson の性能モードも最大にしておくと安定します。

```bash
sudo nvpmodel -m 0
sudo jetson_clocks
```

## 注意点

- `/livox/lidar` のメッセージ型が CustomMsg から PointCloud2 に変わりました。CustomMsg で購読している他のノードは受信できません。
- 元に戻すときは、ドライバの `xfer_format = 1` と FAST-LIO の `lidar_type: 1` を**両方**戻してください。片方だけ戻すと型が合わず、点群を受信できません。

## 今後の調査手順（同様の問題が起きたとき）

1. `top -H` でスレッド単位の CPU 使用率を見て、どのプロセスが張り付いているかを最初に確認する。
2. `/proc/<pid>/task/<tid>/status` の `voluntary_ctxt_switches` を見る。ほとんど増えていなければ、待機せずに回り続けている（処理が追いついていない）。
3. ROS 2 パッケージのビルドフラグ（`flags.make` の `CXX_FLAGS`）に `-O2` / `-O3` が入っているか確認する。
4. 大きなメッセージは、構造体配列の型より PointCloud2 のようなバイト列の型を選ぶ。
