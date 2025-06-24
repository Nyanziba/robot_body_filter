# Plugin System Troubleshooting

## 現在の問題

`robot_body_filter/DistanceFilterProcessor`など、作成したプラグインがplugينlibによって発見されない。

## 確認済み項目

### ✅ 正常な項目
1. **パッケージビルド**: `colcon build`が成功している
2. **ライブラリ生成**: `librobot_body_filter_plugins.so`が生成されている
3. **プラグインメタデータ**: `plugins.xml`が正しく配置されている
4. **package.xml**: `<pluginlib plugin="${prefix}/plugins.xml" />`でエクスポート設定済み
5. **環境変数**: `AMENT_PREFIX_PATH`にワークスペースが含まれている
6. **シンボル**: プラグインクラスのシンボルがライブラリに存在する
7. **依存関係**: ライブラリの依存関係が正常に解決される

### ❌ 問題のある項目
1. **プラグイン発見**: pluginlibが`getDeclaredClasses()`で空のリストを返す
2. **ランタイムロード**: プラグインのインスタンス化ができない

## 発生しているエラー

```
[ERROR] [lidar_processor_example]: Failed to load processor robot_body_filter/DistanceFilterProcessor: 
According to the loaded plugin descriptions the class robot_body_filter/DistanceFilterProcessor 
with base class type robot_body_filter::LidarProcessorBase does not exist. Declared types are 
```

「Declared types are」の部分が空なので、pluginlibが全くプラグインを発見できていない。

## 可能な原因と解決策

### 1. プラグインライブラリのリンク問題
**現象**: プラグイン登録マクロが実行されていない
**解決策**: 
```bash
# 完全なクリーンビルド
rm -rf ../../build/robot_body_filter ../../install/robot_body_filter
colcon build --packages-select robot_body_filter --cmake-clean-cache
```

### 2. pluginlibキャッシュ問題
**現象**: 古いキャッシュ情報が残っている
**解決策**:
```bash
# ROS2のキャッシュクリア
rm -rf ~/.ros/pluginlib_*
```

### 3. プラグイン登録の静的初期化問題
**現象**: `PLUGINLIB_EXPORT_CLASS`マクロが実行されていない
**解決策**: プラグイン登録ファイルの強制リンク（実装済み）

### 4. ROSディストリビューション非互換
**現象**: ROS2 Humbleとplugινlib間の非互換性
**解決策**: 
```bash
# pluginlibの再インストール
sudo apt update
sudo apt install --reinstall ros-humble-pluginlib
```

## 推奨解決手順

1. **完全なクリーンビルド**:
   ```bash
   cd /home/kaya/oedo/livoxrosworkspace
   rm -rf build/robot_body_filter install/robot_body_filter
   colcon build --packages-select robot_body_filter --cmake-clean-cache
   source install/setup.bash
   ```

2. **キャッシュクリア**:
   ```bash
   rm -rf ~/.ros/pluginlib_* 2>/dev/null || true
   ```

3. **依存関係チェック**:
   ```bash
   rosdep install --from-paths src --ignore-src -y
   ```

4. **テスト実行**:
   ```bash
   ros2 run robot_body_filter lidar_processor_example \
     --ros-args -p active_processors:='["robot_body_filter/DistanceFilterProcessor"]'
   ```

## 代替アプローチ

プラグインシステムが動作しない場合、以下の代替手段があります：

1. **直接リンク**: プラグインを静的にリンクして使用
2. **ファクトリーパターン**: pluginlibを使わずに独自のファクトリーパターンで実装
3. **既存フィルター**: robot_body_filterの既存のフィルターシステムを活用

## 検証方法

プラグインが正常に動作している場合、以下のような出力が期待されます：

```
[INFO] [lidar_processor_example]: Distance filter processor initialized
[INFO] [lidar_processor_example]: Distance filter configured: min=0.10, max=30.00
[INFO] [lidar_processor_example]: Loaded 1 processors
[INFO] [lidar_processor_example]: LiDAR Processor Example Node initialized
```

## 参考情報

- pluginlib documentation: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Pluginlib.html
- ROS2 plugin development: https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html