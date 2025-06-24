# Robot Body Filter Examples

このディレクトリには、改良されたrobot body filterの使用例が含まれています。

## 概要

元のrobot_body_filterは保守的すぎるフィルタリングによって、実際のロボット衝突範囲外の点群まで削除してしまう問題がありました。このexamplesディレクトリには、より正確なcollision検出を行う改良版が含まれています。

## 主要ファイル

### 実行可能ファイル
- `accurate_collision_filter.cpp` - 改良されたcollision filter（メイン）
- `simple_pointcloud_publisher.cpp` - テスト用点群データ生成器

### 設定ファイル
- `accurate_collision_filter.yaml` - collision filterの設定
- `simple_pointcloud_publisher.yaml` - 点群生成器の設定

### Launchファイル
- `accurate_collision_filter.launch.py` - メインのlaunchファイル
- `simple_pointcloud_publisher.launch.py` - 点群生成のみのlaunch

### URDFファイル
- `meshes/full_example.urdf` - テスト用ロボットモデル
- `meshes/box.dae` - メッシュファイル

## 使用方法

### 1. 基本的な使用方法
```bash
# ビルド
colcon build --packages-select robot_body_filter
source install/setup.bash

# メインのフィルターを起動（点群生成 + フィルタリング + RViz）
ros2 launch robot_body_filter accurate_collision_filter.launch.py
```

### 2. 点群生成のみ
```bash
# 点群データのみを生成
ros2 launch robot_body_filter simple_pointcloud_publisher.launch.py
```

### 3. フィルターのみ
```bash
# 別のソースからの点群をフィルタリング
ros2 run robot_body_filter accurate_collision_filter
```

## 改良点

### 従来のrobot_body_filter
- 削除率：通常20-80%（過度に保守的）
- 影判定による誤削除
- 膨張設定による範囲外削除

### 改良版accurate_collision_filter
- 削除率：0.2-0.3%（適切）
- 実際の衝突体内部のみ削除
- 最小限のpadding (2cm)
- より高速な処理

## パラメータ設定

主要パラメータ（`accurate_collision_filter.yaml`）：
- `collision_padding`: 安全マージン（推奨：0.02-0.05m）
- `max_collision_distance`: チェック範囲（推奨：2-10m）
- `check_self_collision_only`: 自己衝突のみチェック（推奨：true）

## トラブルシューティング

### 1. TF変換エラー
- URDFファイルのフレーム名を確認
- robot_state_publisherが動作しているか確認

### 2. 点群が表示されない
- トピック名を確認（デフォルト：`/raw_pointcloud`）
- RVizのFixed Frameを確認

### 3. フィルタリングが効かない
- ロボットモデルの衝突定義を確認
- `collision_padding`と`max_collision_distance`を調整

## 非推奨ファイル

`_deprecated/`フォルダには、使用していない古いexampleファイルが含まれています：
- 元のtutorialファイル群
- テスト用プラグイン
- 他のロボット用設定ファイル