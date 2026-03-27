import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image as RosImage, CameraInfo
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

import cv2
from ultralytics import YOLO
import numpy as np


class YoloDetectionNode(Node):

    def __init__(self):
        super().__init__('yolo_detection_node')

        self.model = YOLO("yolov8n.pt")
        self.bridge = CvBridge()

        # camera_info (fx, fy, cx, cy)
        self.fx = self.fy = self.cx = self.cy = None
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # RGB + Depth同時登録(時間同期化)
        self.rgb_sub = Subscriber(self, RosImage, '/camera/image_raw/image')
        self.depth_sub = Subscriber(self, RosImage, '/camera/image_raw/depth_image')
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.image_callback)

        # 配信
        self.center_pub = self.create_publisher(Float32, '/object_center', 10)
        self.image_pub = self.create_publisher(RosImage, '/yolo_image', 10)
        self.position_pub = self.create_publisher(PointStamped, '/person_position', 10)

        self.get_logger().info("YOLO detection node started")

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def image_callback(self, rgb_msg, depth_msg):

        if self.fx is None:
            self.get_logger().warn("Waiting for camera_info...")
            return

        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

        results = self.model(frame, classes=[0])  # 人間のみ
        boxes = results[0].boxes

        if boxes is None or len(boxes) == 0:
            ros_img = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(ros_img)
            return

        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            conf = box.conf[0].item()
            cls = int(box.cls[0].item())
            label = self.model.names[cls]

            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            # depthから距離を読み込む (ｍ)
            distance = depth[center_y, center_x]

            if np.isnan(distance) or distance <= 0.0:
                self.get_logger().warn("Invalid depth value, skipping...")
                continue

            # 3D座標変換
            X = (center_x - self.cx) * distance / self.fx  # 左右
            Y = (center_y - self.cy) * distance / self.fy  # 上下
            Z = distance                                     # 前後

            # 3D位置配信
            point = PointStamped()
            point.header = rgb_msg.header
            point.point.x = float(X)    #ロボット基準前後
            point.point.y = float(Z)   #ロボット基準左右
            point.point.z = float(0.0)  #nav2は2D
            self.position_pub.publish(point)

            # object_center
            msg_center = Float32()
            msg_center.data = float(center_x)
            self.center_pub.publish(msg_center)

            # バウンディングボックス、距離表示
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(
                frame,
                f"{label} {conf:.2f} | {distance:.2f}m",
                (int(x1), int(y1) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

        ros_img = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(ros_img)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()