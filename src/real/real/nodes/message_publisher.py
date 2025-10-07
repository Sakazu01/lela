#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

class MessagePublisher(Node):
    def __init__(self):
        super().__init__('message_publisher')
        
        # Subscribers - hanya detection result
        self.detection_sub = self.create_subscription(
            String, '/mission/detection_result', self.detection_callback, 10)
        
        # Publishers - ke GCS (Mission Planner bisa baca topic ini)
        self.gcs_pub = self.create_publisher(String, '/gcs/detection', 10)
        
        self.get_logger().info('Message Publisher ready (detection only)')
    
    def detection_callback(self, msg):
        """Forward detection result ke GCS dengan format sederhana"""
        detection_data = msg.data  # Format: "WP1:red:dropped"
        
        # Parse detection data
        parts = detection_data.split(':')
        if len(parts) >= 2:
            waypoint = parts[0]  # WP1, WP2, dst
            color = parts[1]     # red, blue, none
            
            # Format pesan untuk GCS
            if color == 'none':
                message = f"{waypoint}: Not detected"
            else:
                message = f"{waypoint}: {color.capitalize()} detected"
            
            # Publish ke GCS
            gcs_msg = String()
            gcs_msg.data = message
            self.gcs_pub.publish(gcs_msg)
            
            self.get_logger().info(f'📡 Sent to GCS: {message}')

def main(args=None):
    rclpy.init(args=args)
    node = MessagePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Message Publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()