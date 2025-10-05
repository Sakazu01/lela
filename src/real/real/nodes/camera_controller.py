#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraController(Node):
    def __init__(self):
        super().__init__('camera_controller')
        
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_rate', 10)
        
        cam_idx = self.get_parameter('camera_index').value
        frame_rate = self.get_parameter('frame_rate').value
        
        self.cap = None
        self.bridge = CvBridge()
        self.camera_enabled = False
        
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        
        self.cmd_sub = self.create_subscription(
            Bool, '/camera/enable', self.enable_callback, 10)
        
        self.timer = self.create_timer(1.0 / frame_rate, self.capture_frame)
        
        self.get_logger().info(f'Camera Controller ready (device: {cam_idx})')
    
    def enable_callback(self, msg):
        if msg.data and not self.camera_enabled:
            self.start_camera()
        elif not msg.data and self.camera_enabled:
            self.stop_camera()
    
    def start_camera(self):
        cam_idx = self.get_parameter('camera_index').value
        self.cap = cv2.VideoCapture(cam_idx)
        
        if self.cap.isOpened():
            self.camera_enabled = True
            self.get_logger().info('Camera started')
        else:
            self.get_logger().error('Failed to open camera')
    
    def stop_camera(self):
        if self.cap:
            self.cap.release()
            self.camera_enabled = False
            self.get_logger().info('Camera stopped')
    
    def capture_frame(self):
        if not self.camera_enabled or not self.cap:
            return
        
        ret, frame = self.cap.read()
        if ret:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_pub.publish(img_msg)
    
    def __del__(self):
        if self.cap:
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = CameraController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
