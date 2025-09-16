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
            Image,
            '/image_raw',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(String, 'color_warning', 10)

        self.bridge = CvBridge()
        self.detector = tarp()
        self.config = HSVConfig()
        self.get_logger().info('Hello world')

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        hsv_values = self.config.get_hsv()

        result, red_mask, blue_mask, objects = self.detector.process_frame(frame, hsv_values)

        for obj in objects:
            color = obj['color']
            # self.get_logger().info(f"Detected {obj['color']} object at {obj['center']}")

            if color.lower() == "blue":
                message = String()
                message.data = f'blue'
                self.publisher.publish(message)
                # self.get_logger().info(f"Published: {message.data}")
            elif color.lower() == "red":
                message = String()
                message.data = f'red'
                self.publisher.publish(message)


        cv2.imshow('Camera', frame)
        cv2.waitKey(1)

def main(args = None):
    rclpy.init(args=args)
    node = opencv_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()