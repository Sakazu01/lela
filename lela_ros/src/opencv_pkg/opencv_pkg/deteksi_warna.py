import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from opencv_pkg.program_warna import tarp
from opencv_pkg.hsv_config import HSVConfig

class opencv_node(Node):
    def __init__(self):
        super().__init__('opencv_node')
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.listener_callback, 10
        )
        self.publisher = self.create_publisher(String, 'color_warning', 10)

        self.bridge = CvBridge()
        self.detector = tarp()
        self.config = HSVConfig()
        self.get_logger().info('Color detection node started.')

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_values = self.config.get_hsv()
        result, _, _, objects = self.detector.process_frame(frame, hsv_values)

        for obj in objects:
            color_name = obj['color'].lower()
            message = String()
            message.data = color_name
            self.publisher.publish(message)
            self.get_logger().info(f"Published warning for: {color_name}")

        # Tampilkan frame hasil deteksi (dengan kotak), bukan frame asli
        cv2.imshow('Detection Result', result)
        cv2.waitKey(1)

def main(args = None):
    rclpy.init(args=args)
    node = opencv_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()