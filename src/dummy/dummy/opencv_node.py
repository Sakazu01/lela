#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from dummy.color_detector import ColorDetector
from dummy.hsv_config import HSVConfig

class OpencvNode(Node):
    """
    ROS2 Node untuk deteksi warna
    - Subscribe: /image_raw (camera feed)
    - Publish: /color_warning (detected color)
    """
    
    def __init__(self):
        super().__init__('opencv_node')
        
        # Subscribe ke camera topic
        self.subscription = self.create_subscription(
            Image, 
            '/image_raw', 
            self.image_callback, 
            10
        )
        
        # Publish hasil deteksi warna
        self.publisher = self.create_publisher(String, '/color_warning', 10)
        
        # CV Bridge untuk convert ROS Image ke OpenCV
        self.bridge = CvBridge()
        
        # Color detector
        self.detector = ColorDetector()
        
        # HSV Config
        self.config = HSVConfig()
        
        self.get_logger().info('OpenCV Node started - waiting for images...')
    
    def image_callback(self, msg):
        """
        Callback saat menerima image dari camera
        """
        try:
            # Convert ROS Image ke OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Get HSV values dari trackbar
            hsv_values = self.config.get_hsv()
            
            # Process frame
            result, red_mask, blue_mask, objects = self.detector.process_frame(frame, hsv_values)
            
            # Publish detected colors
            for obj in objects:
                color_msg = String()
                color_msg.data = obj['color']
                self.publisher.publish(color_msg)
                self.get_logger().info(f"Detected: {obj['color']} at {obj['center']}")
            
            # Tampilkan hasil
            cv2.imshow('Detection Result', result)
            cv2.imshow('Red Mask', red_mask)
            cv2.imshow('Blue Mask', blue_mask)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = OpencvNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
