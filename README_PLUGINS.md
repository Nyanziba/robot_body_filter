# Robot Body Filter - LiDAR Processor Plugins

robot_body_filterパッケージでpluglinlibを使用したLiDARデータ処理システムについて説明します。

## 概要

pluginlibを使用して、LiDARデータ（点群とレーザースキャン）を処理するための拡張可能なプラグインシステムを実装しました。これにより、異なる処理アルゴリズムを組み合わせて、柔軟なデータ処理パイプラインを構築できます。

## Pluginlibとは

Pluginlibは、ROSにおいて実行時に動的にライブラリをロードするためのC++ライブラリです。コンパイル時に決定せずに、実行時に使用するアルゴリズムを選択できます。

### Pluginlibの利点

1. **動的ロード**: 実行時にプラグインを選択・ロード
2. **拡張性**: 新しいアルゴリズムを既存システムに容易に追加
3. **モジュール化**: 個別のプラグインとして機能を分離
4. **パフォーマンス**: 必要なプラグインのみロード

## アーキテクチャ

### 基底クラス: LidarProcessorBase

すべてのプラグインが継承する抽象基底クラス：

```cpp
class LidarProcessorBase
{
public:
  virtual bool initialize(std::shared_ptr<rclcpp::Node> node) = 0;
  virtual bool processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input_cloud,
                                sensor_msgs::msg::PointCloud2& output_cloud) = 0;
  virtual bool processLaserScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& input_scan,
                               sensor_msgs::msg::LaserScan& output_scan) = 0;
  virtual std::string getProcessorName() const = 0;
  virtual bool configure() = 0;
  virtual void reset() = 0;
};
```

### 実装されたプラグイン

#### 1. DistanceFilterProcessor
- **機能**: センサー原点からの距離に基づく点フィルタリング
- **パラメータ**:
  - `min_distance`: 最小距離閾値
  - `max_distance`: 最大距離閾値
  - `keep_organized`: 組織化構造の維持

#### 2. VoxelGridProcessor
- **機能**: ボクセルグリッドダウンサンプリング
- **パラメータ**:
  - `leaf_size_x/y/z`: X/Y/Z方向のボクセルサイズ

#### 3. RobotBodyFilterProcessor
- **機能**: ロボット本体の除去フィルタリング
- **既存のRobotBodyFilterをプラグインインターフェースで包含**

## 使用方法

### 1. プラグインのロード

```cpp
#include <pluginlib/class_loader.hpp>
#include <robot_body_filter/lidar_processor_base.h>

// プラグインローダーの作成
auto plugin_loader = std::make_shared<pluginlib::ClassLoader<robot_body_filter::LidarProcessorBase>>(
    "robot_body_filter", "robot_body_filter::LidarProcessorBase");

// プラグインのロード
auto processor = plugin_loader->createSharedInstance("robot_body_filter/DistanceFilterProcessor");
```

### 2. プラグインの初期化と設定

```cpp
// ノードハンドルで初期化
if (processor->initialize(node)) {
    // パラメータで設定
    if (processor->configure()) {
        RCLCPP_INFO(logger, "Processor loaded: %s", processor->getProcessorName().c_str());
    }
}
```

### 3. データ処理

```cpp
// 点群処理
sensor_msgs::msg::PointCloud2 output_cloud;
if (processor->processPointCloud(input_cloud, output_cloud)) {
    // 処理成功
}

// レーザースキャン処理
sensor_msgs::msg::LaserScan output_scan;
if (processor->processLaserScan(input_scan, output_scan)) {
    // 処理成功
}
```

### 4. 設定ファイル

```yaml
lidar_processor_example:
  ros__parameters:
    # 使用するプラグインリスト（順序が処理順を決定）
    active_processors:
      - "robot_body_filter/DistanceFilterProcessor"
      - "robot_body_filter/VoxelGridProcessor"
      - "robot_body_filter/RobotBodyFilterProcessor"

    # 各プラグインのパラメータ
    distance_filter:
      min_distance: 0.1
      max_distance: 30.0
    
    voxel_grid:
      leaf_size_x: 0.05
      leaf_size_y: 0.05
      leaf_size_z: 0.05
```

## ビルド

### CMakeLists.txt の更新

```cmake
# Pluginlibの依存関係
find_package(pluginlib REQUIRED)

# プラグインライブラリの作成
add_library(${PROJECT_NAME}_plugins ${PLUGIN_SRCS})
target_link_libraries(${PROJECT_NAME}_plugins ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_plugins pluginlib)
```

### package.xml の更新

```xml
<depend>pluginlib</depend>

<export>
  <build_type>ament_cmake</build_type>
  <robot_body_filter plugin="${prefix}/plugins.xml" />
</export>
```

### plugins.xml

```xml
<library path="lib/librobot_body_filter_plugins">
  <class name="robot_body_filter/DistanceFilterProcessor" 
         type="robot_body_filter::DistanceFilterProcessor" 
         base_class_type="robot_body_filter::LidarProcessorBase">
    <description>Distance-based point filtering processor</description>
  </class>
</library>
```

## プラグイン登録

```cpp
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(robot_body_filter::DistanceFilterProcessor, 
                       robot_body_filter::LidarProcessorBase)
```

## 新しいプラグインの作成

### 1. ヘッダーファイル

```cpp
#include <robot_body_filter/lidar_processor_base.h>

class MyCustomProcessor : public robot_body_filter::LidarProcessorBase
{
public:
  bool initialize(std::shared_ptr<rclcpp::Node> node) override;
  bool processPointCloud(...) override;
  bool processLaserScan(...) override;
  std::string getProcessorName() const override { return "my_custom_processor"; }
  bool configure() override;
  void reset() override;
};
```

### 2. 実装ファイル

```cpp
#include "my_custom_processor.h"

bool MyCustomProcessor::processPointCloud(...)
{
  // カスタム処理ロジック
  return true;
}

// プラグイン登録
PLUGINLIB_EXPORT_CLASS(MyCustomProcessor, robot_body_filter::LidarProcessorBase)
```

### 3. plugins.xmlに追加

```xml
<class name="robot_body_filter/MyCustomProcessor" 
       type="MyCustomProcessor" 
       base_class_type="robot_body_filter::LidarProcessorBase">
  <description>My custom LiDAR processor</description>
</class>
```

## 実行例

```bash
# パラメータファイルを使用してノード実行
ros2 run robot_body_filter lidar_processor_example --ros-args --params-file processor_params.yaml

# 入力データを送信
ros2 topic pub /input_cloud sensor_msgs/msg/PointCloud2 "{...}"
```

## 利点

1. **柔軟性**: 実行時にプラグインを選択・組み合わせ
2. **再利用性**: プラグインを他のプロジェクトでも使用可能
3. **保守性**: 各機能が独立したプラグインとして管理
4. **拡張性**: 新しいアルゴリズムを容易に追加

## 注意事項

- プラグインのロード順序が処理パイプラインの順序を決定
- 各プラグインは独立してパラメータを管理
- エラーハンドリングでプラグインの失敗に対応
- メモリ管理に注意（shared_ptrの使用推奨）