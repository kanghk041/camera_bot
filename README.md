README
# 人間追従ロボット (Person Following Robot)

ROS2とYOLOv8とNav2を組み合わせた、カメラベースの人間追従ロボットです。
Gazebo シミュレーター上で実装・動作確認を行いました。

## デモ動画

[YouTubeリンク] : https://www.youtube.com/watch?v=vw65ky0wy5c

## システム構成

RGB-Dカメラ(Gazeboシミュレーター)
        ↓
yolo_detection_node
  - YOLOv8nによる人間検出
  - depth情報から3D座標を計算
  - /person_positionトピックへ配信
        ↓
person_follower_node
  - TFを用いてカメラ座標系からマップ座標系へ変換
  - Nav2アクションクライアント経由でゴール送信
  - 一定距離以内では停止
  - 経路計画・自律走行

## 使用環境

- ロボットOS:ROS2 Jazzy 
- シミュレーター:Gazebo(gz_sensor RGB-D カメラ)
- 物体検出モデル:YOLOv8n (Ultralytics)
- 自律走行:Nav2 (NavigateToPose アクション)
- 座標変換:TF2 (camera_link → map)
- 言語:Python3
- ハードウェアモデル:PinkyBotベース 

## ノード構成

### yolo_detection_node
- RGB画像とdepth画像をApproximateTimeSynchronizerで同期して取得
- YOLOv8nでクラス0(person)のみ検出
- バウンディングボックス中心の深度値から3D座標(X, Y, Z)を計算
- 検出結果を/person_position(PointStamped)として配信

### person_follower_node
- /person_positionを購読
- TF2でcamera_linkからマップ座標系へ変換
- 人間との距離が閾値以上の場合のみゴールを送信
- Nav2のフィードバック・結果を非同期で受信

## 実行環境

- Windows 11 + WSL2 (Ubuntu 24.04)
- ROS2 Jazzy
- Python 3.12.3
- Gazebo SIM

## 実行方法
bash
# (既に終わっている作業)
# マッピング
ros2 launch camera_bot_navigation map_building.launch.xml
# teleop_twist_keyboardでロボットを手動操作しながらLiDARデータを収集
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# マップを保存
ros2 run nav2_map_server map_saver_cli -f ~/map

# (実行)
# シミュレーター起動
ros2 launch camera_bot_gazebo launch_sim_empty.launch.xml

# ローカライゼーション起動
ros2 launch camera_bot_navigation localization_launch.xml
(Rviz2上で「2D Pose Estimate」ボタンでロボットの初期位置を指定すること)

# ナビゲーション起動
ros2 launch camera_bot_navigation navigation_launch.xml

# YOLO検出ノード起動
ros2 run my_package yolo_detection_node

# 人間追従ノード起動
ros2 run my_package person_follower_node

## 今後の課題
- 実機ロボットへの移行
- 一回も人間を見つけなかった時の制御(回転、移動)
- ゴール付近での細かい振動・往復動作の抑制

## 参考
-  Robot hardware model based on [PinkyBot](https://github.com/PinkWink/pinky_for_edu) by PinkWink
  - URDFベースモデルを使用
  - 既存モデルにはなかったRGB-Dカメラ(depth camera)を独自に追加・統合
  - カラーのみ変更
- [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics)
- [Nav2](https://navigation.ros.org/)
