"""ROS 2 node to save a single image frame from a topic to disk and exit."""

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


class SingleImageSaver(Node):
    """Subscribes to image topic, saves one frame, and shuts down."""

    def __init__(self):
        super().__init__("single_image_saver")
        self.bridge = CvBridge()

        # Compatible with both Reliable and Best Effort publishers
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(
            Image, "/chess/debug_image", self.image_callback, qos
        )
        self.get_logger().info("Waiting for one image on /chess/debug_image...")

    def image_callback(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        file_path = "/home/duykhongcay/docker_ws/lekiwi_ros2/log/debug_single.jpg"
        cv2.imwrite(file_path, cv_img)
        self.get_logger().info(f"Saved image to: {file_path}")
        # Terminate node after saving the first frame
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SingleImageSaver()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
