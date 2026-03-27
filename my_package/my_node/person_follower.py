import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import tf2_ros
 
 
class PersonFollowerNode(Node):
 
    def __init__(self):
        super().__init__('person_follower_node')
 
        # 人間の位置情報登録
        self.position_sub = self.create_subscription(
            PointStamped,
            '/person_position',
            self.position_callback,
            10
        )
 
        # Nav2へのアクションクライアント
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
 
        # TF変換用のバッファとリスナー
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
 
        # ナビゲーション中かどうかのフラグ
        self.is_navigating = False
 
        # 人間との最小追従距離
        self.min_distance = 1.0
 
        self.get_logger().info("Person follower node started")
 
    def position_callback(self, msg):
 
        # ナビゲーション中は新しいゴールを送らない
        if self.is_navigating:
            return
 
        # カメラ座標系での人間までの距離を計算
        distance = math.sqrt(msg.point.x**2 + msg.point.y**2)
        if distance < self.min_distance:
            self.get_logger().info(f"Person is close enough ({distance:.2f}m), stopping.")
            return
 
        # TFからカメラリンクからマップ座標系への変換を取得
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'camera_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return
 
        # ロボットの現在位置を取得
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
 
        # クォータニオンからヨー角を取り出す
        q = transform.transform.rotation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
 
        # カメラ座標系での人間位置
        # xが前後 yが左右
        person_x = msg.point.x
        person_y = msg.point.y
 
        # カメラ座標系からマップ座標系へ変換
        goal_x = tx + person_x * math.cos(yaw) - person_y * math.sin(yaw)
        goal_y = ty + person_x * math.sin(yaw) + person_y * math.cos(yaw)
 
        self.get_logger().info(f"Person in camera frame: x={person_x:.2f}, y={person_y:.2f}")
        self.get_logger().info(f"Goal in map frame: x={goal_x:.2f}, y={goal_y:.2f}")
 
        # ゴールのPoseを作成してNav2へ送信
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0
 
        self.send_goal(goal_pose)
 
    def send_goal(self, goal_pose):
 
        # アクションサーバーの起動を待つ
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("Nav2 action server not available")
            return
 
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
 
        self.get_logger().info(
            f"Sending goal: x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}"
        )
 
        # ナビゲーション開始フラグを立てる
        self.is_navigating = True
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
 
    def goal_response_callback(self, future):
        goal_handle = future.result()
 
        # ゴールが拒否された場合はフラグをリセット
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            self.is_navigating = False
            return
 
        self.get_logger().info("Goal accepted!")
 
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
 
    def result_callback(self, future):
        result = future.result()
        status = result.status
        self.get_logger().info(f"Navigation status: {status}")
 
        # ステータス4はSUCCEEDED成功時の処理
        if status == 4:
            self.get_logger().info("Goal reached!")
        else:
            self.get_logger().warn(f"Navigation failed with status: {status}")
 
        # ナビゲーション完了でフラグをリセット
        self.is_navigating = False
 
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
 
        # 残り距離を2秒ごとに出力
        remaining = feedback.distance_remaining
        self.get_logger().info(f"Distance remaining: {remaining:.2f}m", throttle_duration_sec=2.0)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()