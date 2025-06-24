# Robot Body Filter Plugin System - テスト手順

## 実装したプラグインシステムのテスト方法

### 前提条件

1. **ROS2 Humbleの環境設定**
```bash
source /opt/ros/humble/setup.bash
```

2. **ワークスペースのビルド**
```bash
cd /home/kaya/oedo/livoxrosworkspace
colcon build --packages-select robot_body_filter --cmake-clean-cache
source install/setup.bash
```

### 自動テストスクリプトの実行

最も簡単な方法は自動テストスクリプトを使用することです：

```bash
cd /home/kaya/oedo/livoxrosworkspace/src/robot_body_filter
./scripts/run_plugin_test.sh
```

このスクリプトは以下をテストします：
- パッケージのビルド
- プラグインの登録確認
- サンプルノードの起動
- テストデータでの動作確認

### 手動テスト手順

#### 1. プラグインの発見テスト

```bash
# パッケージが認識されているか確認
ros2 pkg list | grep robot_body_filter

# プラグインが登録されているか確認（pluginlibがインストールされている場合）
ros2 run pluginlib list_plugins robot_body_filter::LidarProcessorBase
```

#### 2. 基本的なプラグインロードテスト

```bash
# 1つのプラグインのみを使用してテスト
ros2 run robot_body_filter lidar_processor_example \
  --ros-args \
  -p active_processors:="[\"robot_body_filter/DistanceFilterProcessor\"]" \
  -p distance_filter.min_distance:=0.1 \
  -p distance_filter.max_distance:=30.0
```

#### 3. 複数プラグインの組み合わせテスト

```bash
# 距離フィルタ + ボクセルグリッドダウンサンプリング
ros2 run robot_body_filter lidar_processor_example \
  --ros-args \
  -p active_processors:="[\"robot_body_filter/DistanceFilterProcessor\", \"robot_body_filter/VoxelGridProcessor\"]" \
  -p distance_filter.min_distance:=0.1 \
  -p distance_filter.max_distance:=25.0 \
  -p voxel_grid.leaf_size_x:=0.1 \
  -p voxel_grid.leaf_size_y:=0.1 \
  -p voxel_grid.leaf_size_z:=0.1
```

#### 4. テストデータの生成と処理

```bash
# ターミナル1: テストデータ生成
ros2 run robot_body_filter test_pointcloud_publisher

# ターミナル2: プラグイン処理
ros2 run robot_body_filter lidar_processor_example

# ターミナル3: 結果確認
ros2 topic echo /processed_cloud
ros2 topic echo /processed_scan
```

#### 5. 設定ファイルを使用したテスト

```bash
# 設定ファイルを使用
ros2 run robot_body_filter lidar_processor_example \
  --ros-args --params-file src/robot_body_filter/examples/processor_params.yaml
```

#### 6. Launch fileを使用したテスト

```bash
ros2 launch robot_body_filter test_plugins.launch.py
```

### プラグインの動作確認方法

#### Distance Filter Processor
- 近距離（0.05m未満）の点が除去される
- 遠距離（30m超）の点が除去される
- 有効範囲内の点のみが残る

#### Voxel Grid Processor
- 点密度が削減される
- ボクセルサイズに応じて点数が減る
- 全体的な形状は保持される

#### Robot Body Filter Processor
- URDFモデルが必要（実際のロボット環境で有効）
- ロボット本体部分の点が除去される

### 期待される出力

正常に動作している場合、以下のようなログが表示されます：

```
[INFO] [lidar_processor_example]: LiDAR Processor Example Node initialized
[INFO] [distance_filter_processor]: Distance filter processor initialized
[INFO] [distance_filter_processor]: Distance filter configured: min=0.10, max=30.00, organized=false
[INFO] [voxel_grid_processor]: Voxel grid processor initialized
[INFO] [voxel_grid_processor]: Voxel grid configured: leaf_size=[0.050, 0.050, 0.050]
[INFO] [lidar_processor_example]: Loaded 2 processors
```

### トラブルシューティング

#### プラグインが見つからない場合
```bash
# 環境変数の確認
echo $ROS_PACKAGE_PATH
echo $LD_LIBRARY_PATH

# 再ビルド
colcon build --packages-select robot_body_filter --cmake-clean-cache
source install/setup.bash
```

#### 依存関係エラーの場合
```bash
# 依存関係のインストール
rosdep install --from-paths src --ignore-src -y

# 特定の依存関係
sudo apt install ros-humble-pluginlib ros-humble-pcl-conversions ros-humble-laser-geometry
```

#### プラグインロードエラーの場合
- `plugins.xml`ファイルが正しく配置されているか確認
- CMakeLists.txtでプラグインライブラリが正しくビルドされているか確認
- package.xmlでプラグインが正しくエクスポートされているか確認

### 検証項目チェックリスト

- [ ] パッケージのビルドが成功する
- [ ] プラグインライブラリが生成される
- [ ] plugins.xmlが正しい場所にインストールされる
- [ ] 例示ノードが起動する
- [ ] プラグインが動的にロードされる
- [ ] 各プラグインが正しく動作する
- [ ] 複数プラグインの組み合わせが動作する
- [ ] パラメータが正しく設定される
- [ ] エラーハンドリングが適切に動作する

### パフォーマンステスト

大きなデータセットでのテスト：
```bash
# 大きな点群データでのテスト
ros2 bag play your_lidar_data.bag

# リアルタイム性能の確認
ros2 topic hz /processed_cloud
```

このテスト手順に従って、pluginlibベースのLiDARプロセッサシステムの機能を包括的に検証できます。