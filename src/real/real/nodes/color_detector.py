#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('red_h_min_1', 0), ('red_s_min_1', 70), ('red_v_min_1', 50),
                ('red_h_max_1', 20), ('red_s_max_1', 230), ('red_v_max_1', 255),
                ('red_h_min_2', 170), ('red_s_min_2', 70), ('red_v_min_2', 50),
                ('red_h_max_2', 179), ('red_s_max_2', 230), ('red_v_max_2', 255),
                ('blue_h_min', 100), ('blue_s_min', 50), ('blue_v_min', 50),
                ('blue_h_max', 130), ('blue_s_max', 255), ('blue_v_max', 255),
                ('min_area', 500),
                ('enable_visualization', False)
            ]
        )
        
        self.bridge = CvBridge()
        
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos)
        
        self.color_pub = self.create_publisher(String, '/detection/color', 10)
        
        self.kernel_small = np.ones((3, 3), np.uint8)
        self.kernel_medium = np.ones((5, 5), np.uint8)
        
        self.get_logger().info('Color Detector ready with BEST_EFFORT QoS')
    
    def get_hsv_ranges(self):
        return {
            'red_low1': np.array([
                self.get_parameter('red_h_min_1').value,
                self.get_parameter('red_s_min_1').value,
                self.get_parameter('red_v_min_1').value
            ]),
            'red_up1': np.array([
                self.get_parameter('red_h_max_1').value,
                self.get_parameter('red_s_max_1').value,
                self.get_parameter('red_v_max_1').value
            ]),
            'red_low2': np.array([
                self.get_parameter('red_h_min_2').value,
                self.get_parameter('red_s_min_2').value,
                self.get_parameter('red_v_min_2').value
            ]),
            'red_up2': np.array([
                self.get_parameter('red_h_max_2').value,
                self.get_parameter('red_s_max_2').value,
                self.get_parameter('red_v_max_2').value
            ]),
            'blue_low': np.array([
                self.get_parameter('blue_h_min').value,
                self.get_parameter('blue_s_min').value,
                self.get_parameter('blue_v_min').value
            ]),
            'blue_up': np.array([
                self.get_parameter('blue_h_max').value,
                self.get_parameter('blue_s_max').value,
                self.get_parameter('blue_v_max').value
            ])
        }
    
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv_ranges = self.get_hsv_ranges()
        
        red_mask1 = cv2.inRange(hsv, hsv_ranges['red_low1'], hsv_ranges['red_up1'])
        red_mask2 = cv2.inRange(hsv, hsv_ranges['red_low2'], hsv_ranges['red_up2'])
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        red_mask = self.clean_mask(red_mask)
        
        blue_mask = cv2.inRange(hsv, hsv_ranges['blue_low'], hsv_ranges['blue_up'])
        blue_mask = self.clean_mask(blue_mask)
        
        min_area = self.get_parameter('min_area').value
        red_detected = self.has_object(red_mask, min_area)
        blue_detected = self.has_object(blue_mask, min_area)
        
        if red_detected:
            msg = String()
            msg.data = 'red'
            self.color_pub.publish(msg)
            self.get_logger().info('RED detected')
        
        if blue_detected:
            msg = String()
            msg.data = 'blue'
            self.color_pub.publish(msg)
            self.get_logger().info('BLUE detected')
        
        if self.get_parameter('enable_visualization').value:
            cv2.imshow('Frame', frame)
            cv2.imshow('Red Mask', red_mask)
            cv2.imshow('Blue Mask', blue_mask)
            cv2.waitKey(1)
    
    def clean_mask(self, mask):
        mask = cv2.medianBlur(mask, 5)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_small, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_medium, iterations=2)
        return mask
    
    def has_object(self, mask, min_area):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) >= min_area:
                return True
        return False

# INI DI LUAR CLASS - PERHATIKAN INDENTASI!
def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()