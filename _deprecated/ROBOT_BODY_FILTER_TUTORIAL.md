# Robot Body Filter Tutorial

このチュートリアルでは、`RobotBodyFilterProcessor`を使用してLiDARデータからロボット本体部分を除去する方法を学びます。

## 概要

ロボット本体フィルタは、LiDARスキャンからロボット自身の部分を自動的に検出して除去するツールです。これにより：

- **ナビゲーションの精度向上**: ロボット本体の誤検知を防止
- **マッピングの品質向上**: 環境マップからロボット本体を除外
- **障害物検知の信頼性向上**: 実際の障害物のみを検出

## 使用するロボットモデル

このチュートリアルでは`full_example.urdf`のNIFTiロボットを使用します：

```
NIFTi Robot Structure:
├── base_link (1.8×1.8×1.8m box)
├── antenna (球形、半径1.24m) 
├── laser_base (回転軸)
└── laser (LiDARセンサー、回転可能)
```

## 起動方法

### 1. 基本的な起動

```bash
# パッケージをビルド
cd /home/kaya/oedo/livoxrosworkspace
colcon build --packages-select robot_body_filter

# 環境をセットアップ
source install/setup.bash

# チュートリアルを起動
ros2 launch robot_body_filter robot_body_filter_tutorial.launch.py
```

### 2. RVizなしで起動

```bash
ros2 launch robot_body_filter robot_body_filter_tutorial.launch.py use_rviz:=false
```

### 3. カスタム設定で起動

```bash
ros2 launch robot_body_filter robot_body_filter_tutorial.launch.py \
  config_file:=/path/to/your/config.yaml \
  urdf_file:=/path/to/your/robot.urdf
```

## 動作確認

### RVizでの可視化

チュートリアルを起動すると、RVizに以下が表示されます：

1. **ロボットモデル** (半透明)
   - NIFTiロボットの3Dモデル
   - レーザーが回転している様子

2. **生データ** (`/raw_pointcloud`) - 赤色
   - フィルタリング前のLiDARデータ
   - ロボット本体部分も含む

3. **フィルタ済みデータ** (`/filtered_pointcloud`) - 青色
   - ロボット本体部分が除去されたデータ
   - 環境の点群のみ

4. **デバッグデータ** (`/debug_pointcloud`) - 緑色
   - フィルタリング結果の可視化

### ターミナル出力

```
[INFO] [robot_body_filter_tutorial]: === Robot Body Filter Tutorial ===
[INFO] [robot_body_filter_tutorial]: RobotBodyFilterProcessor loaded and configured successfully!
[INFO] [robot_body_filter_tutorial]: Generated test cloud with 3321 points from laser frame
[INFO] [robot_body_filter_tutorial]: Filtering: 3321 -> 2156 points (35.1% removed)
```

## フィルタリング設定の詳細

### 主要パラメータ

チュートリアル設定 (`robot_body_filter_tutorial.yaml`) の重要な設定：

```yaml
# 座標フレーム
fixedFrame: "base_link"          # 固定参照フレーム
sensorFrame: "laser"             # LiDARセンサーフレーム
filteringFrame: "base_link"      # フィルタリング実行フレーム

# ロボットモデル
body_model:
  robot_description_param: "robot_description"
  inflation:
    scale: 1.1                   # ロボット形状を10%拡大
    padding: 0.05                # 5cm のパディング追加

# フィルタリング設定
filter:
  do_clipping: true              # クリッピングを有効
  do_contains_test: true         # 内包テストを有効
  do_shadow_test: true           # 影テストを有効
  max_shadow_distance: 0.5      # 影の最大距離

# 無視するリンク
ignored_links:
  bounding_sphere: ["antenna"]              # アンテナを境界球から除外
  shadow_test: ["laser", "antenna"]        # レーザーとアンテナを影テストから除外
```

### フィルタリングアルゴリズム

1. **内包テスト (Contains Test)**
   - 点がロボット本体の内部にあるかチェック
   - 内部の点は除去される

2. **影テスト (Shadow Test)**
   - ロボットによって生じる影の領域を検出
   - センサーから見てロボットの後ろにある点を除去

3. **クリッピング (Clipping)**
   - 距離制限による点の除去
   - 最小・最大距離範囲外の点を除去

## トピック構成

### 入力トピック
- `/raw_pointcloud` - 生のLiDARデータ

### 出力トピック
- `/filtered_pointcloud` - フィルタリング済みデータ
- `/debug_pointcloud` - デバッグ用可視化データ

### TFフレーム
- `world` → `base_link` (静的)
- `base_link` → `laser_base` (静的)
- `laser_base` → `laser` (動的、回転)

## カスタマイズ方法

### 1. 異なるロボットモデルの使用

```bash
# 自分のURDFファイルを使用
ros2 launch robot_body_filter robot_body_filter_tutorial.launch.py \
  urdf_file:=/path/to/your_robot.urdf
```

### 2. フィルタリングパラメータの調整

`robot_body_filter_tutorial.yaml`を編集：

```yaml
# より大きなパディング
body_model:
  inflation:
    scale: 1.2      # 20%拡大
    padding: 0.1    # 10cmパディング

# 影テストの無効化
filter:
  do_shadow_test: false
```

### 3. 実際のLiDARデータの使用

```cpp
// tutorial.cppを編集して外部トピックを使用
pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/velodyne_points",  // 実際のLiDARトピック
    10,
    std::bind(&RobotBodyFilterTutorial::processPointCloud, this, std::placeholders::_1));
```

## トラブルシューティング

### よくある問題

1. **フィルタリングが動作しない**
   ```bash
   # robot_descriptionが正しく設定されているかチェック
   ros2 param get /robot_body_filter_tutorial robot_description
   ```

2. **TFエラー**
   ```bash
   # TFツリーを確認
   ros2 run tf2_tools view_frames
   ```

3. **パフォーマンス問題**
   ```yaml
   # 更新頻度を下げる
   filter:
     model_pose_update_interval: 0.1  # 100msに変更
   ```

### デバッグ設定

デバッグ情報を有効にする：

```yaml
debug:
  pcl:
    inside: true     # 内部点をデバッグ
    clip: true       # クリップ点をデバッグ
    shadow: true     # 影点をデバッグ
  marker:
    contains: true   # 内包マーカー表示
    shadow: true     # 影マーカー表示
```

## まとめ

このチュートリアルでは：

✅ RobotBodyFilterProcessorの基本的な使用方法  
✅ URDFモデルとの連携  
✅ パラメータ設定の方法  
✅ RVizでの可視化  
✅ カスタマイズ方法  

を学びました。実際のロボットプロジェクトでこの知識を活用して、高精度なLiDARデータ処理を実現してください。