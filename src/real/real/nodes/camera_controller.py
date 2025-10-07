#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Picamera2 untuk native Raspberry Pi Camera support
try:
    from picamera2 import Picamera2
    PICAMERA_AVAILABLE = True
except ImportError:
    PICAMERA_AVAILABLE = False

class CameraController(Node):
    def __init__(self):
        super().__init__('camera_controller')
        
        # Parameters
        self.declare_parameter('frame_rate', 10)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('use_picamera2', True)
        
        frame_rate = self.get_parameter('frame_rate').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        use_picamera2 = self.get_parameter('use_picamera2').value
        
        # Validate parameters
        if frame_rate <= 0 or frame_rate > 60:
            raise ValueError(f'frame_rate must be 1-60, got {frame_rate}')
        
        if width <= 0 or height <= 0:
            raise ValueError(f'Invalid resolution: {width}x{height}')
        
        # State
        self.bridge = CvBridge()
        self.camera_enabled = False
        self.consecutive_failures = 0
        self.max_failures = 10
        self.cam = None
        self.use_picamera2 = use_picamera2 and PICAMERA_AVAILABLE
        self.width = width
        self.height = height
        self.frame_rate = frame_rate
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.status_pub = self.create_publisher(Bool, '/camera/status', 10)
        
        # Subscribers
        self.cmd_sub = self.create_subscription(
            Bool, '/camera/enable', self.enable_callback, 10
        )
        
        # Timer for frame capture
        self.timer = self.create_timer(1.0 / frame_rate, self.capture_frame)
        
        cam_type = "Picamera2" if self.use_picamera2 else "OpenCV VideoCapture"
        self.get_logger().info(
            f'Camera Controller ready ({cam_type}, {width}x{height} @ {frame_rate}Hz)'
        )
        
        if not self.use_picamera2 and use_picamera2:
            self.get_logger().warn(
                'Picamera2 requested but not available, using OpenCV fallback'
            )
    
    def enable_callback(self, msg):
        """Handle camera enable/disable command"""
        if msg.data and not self.camera_enabled:
            self.start_camera()
        elif not msg.data and self.camera_enabled:
            self.stop_camera()
            
    def start_camera(self):
        """Start camera capture with warm-up period"""
        self.get_logger().info('Starting camera...')
        
        try:
            if self.use_picamera2:
                self._start_picamera2()
                
                # Warm-up period untuk auto-exposure stabilization
                self.get_logger().info('Camera warming up (2s)...')
                import time
                time.sleep(2.0)  # Allow AE/AWB to stabilize
                self.get_logger().info('Camera ready')
            else:
                self._start_opencv()
            
            self.camera_enabled = True
            self.consecutive_failures = 0
            
            status = Bool()
            status.data = True
            self.status_pub.publish(status)
            
            self.get_logger().info('✓ Camera started successfully')
            
        except Exception as e:
            self.get_logger().error(f'✗ Failed to start camera: {e}')
            self.cam = None
            self.camera_enabled = False
    
    def _start_picamera2(self):
        """Start Picamera2 (native Raspberry Pi Camera)"""
        self.cam = Picamera2()
        
        # Configure camera
        config = self.cam.create_preview_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"},
            controls={"FrameRate": self.frame_rate}
        )
        self.cam.configure(config)
        
        # Start camera
        self.cam.start()
        
        self.get_logger().info(f'Picamera2 started: {self.width}x{self.height}')
    
    def _start_opencv(self):
        """Start OpenCV VideoCapture (USB camera fallback)"""
        self.cam = cv2.VideoCapture(0)
        
        if not self.cam.isOpened():
            raise RuntimeError('Could not open camera device')
        
        # Set properties
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cam.set(cv2.CAP_PROP_FPS, self.frame_rate)
        
        self.get_logger().info(f'OpenCV camera started: {self.width}x{self.height}')
    
    def stop_camera(self):
        """Stop camera capture"""
        if self.cam:
            try:
                if self.use_picamera2:
                    self.cam.stop()
                    self.cam.close()
                else:
                    self.cam.release()
            except Exception as e:
                self.get_logger().error(f'Error stopping camera: {e}')
            
            self.cam = None
        
        self.camera_enabled = False
        self.consecutive_failures = 0
        
        # Publish status
        status = Bool()
        status.data = False
        self.status_pub.publish(status)
        
        self.get_logger().info('Camera stopped')
    
    def capture_frame(self):
        """Capture and publish frame"""
        if not self.camera_enabled or not self.cam:
            return
        
        try:
            if self.use_picamera2:
                frame = self._capture_picamera2()
            else:
                frame = self._capture_opencv()
            
            if frame is not None:
                # Convert and publish
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = 'camera_frame'
                self.image_pub.publish(img_msg)
                
                # Reset failure counter
                self.consecutive_failures = 0
            else:
                self._handle_capture_failure()
        
        except Exception as e:
            self.get_logger().error(f'Error capturing frame: {e}')
            self._handle_capture_failure()
    
    def _capture_picamera2(self):
        """Capture frame from Picamera2"""
        try:
            # Capture array (RGB888)
            frame = self.cam.capture_array()
            
            # Convert RGB to BGR for OpenCV compatibility
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            return frame
        
        except Exception as e:
            self.get_logger().error(f'Picamera2 capture error: {e}')
            return None
    
    def _capture_opencv(self):
        """Capture frame from OpenCV VideoCapture"""
        ret, frame = self.cam.read()
        
        if ret:
            return frame
        else:
            return None
    
    def _handle_capture_failure(self):
        """Handle camera capture failure"""
        self.consecutive_failures += 1
        
        if self.consecutive_failures >= self.max_failures:
            self.get_logger().error(
                f'Camera capture failed {self.max_failures} times - stopping camera'
            )
            self.stop_camera()
        else:
            self.get_logger().warn(
                f'Frame capture failed ({self.consecutive_failures}/{self.max_failures})'
            )
    
    def destroy_node(self):
        """Cleanup before node destruction"""
        self.get_logger().info('Cleaning up camera resources...')
        self.stop_camera()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Camera Controller...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()