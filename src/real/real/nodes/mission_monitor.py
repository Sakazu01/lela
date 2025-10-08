#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import WaypointReached
from std_msgs.msg import Int16  # MODIFIED: Changed from Bool to Int16

class MissionMonitor(Node):
    def __init__(self):
        super().__init__('mission_monitor')
        
        # Parameters
        self.declare_parameter('target_waypoint', -1)
        self.target_wp = self.get_parameter('target_waypoint').value
        
        # Subscribers
        self.wp_sub = self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.waypoint_callback,
            10
        )
        
        # Publishers
        # MODIFIED: Publisher now sends Int16 (the waypoint number)
        self.wp_reached_pub = self.create_publisher(Int16, '/mission/waypoint_reached', 10)
        
        # State
        self.last_waypoint = -1
        
        if self.target_wp == -1:
            self.get_logger().info('Mission Monitor ready (triggering on ANY waypoint)')
        else:
            self.get_logger().info(f'Mission Monitor ready (target WP: {self.target_wp})')
    
    def waypoint_callback(self, msg):
        """Handle waypoint reached event"""
        wp_seq = msg.wp_seq
        
        # Prevent duplicate triggers
        if wp_seq == self.last_waypoint:
            self.get_logger().debug(f'Waypoint {wp_seq} already processed, ignoring')
            return
        
        self.last_waypoint = wp_seq
        self.get_logger().info(f'Waypoint {wp_seq} reached')
        
        # Check if this is target waypoint
        if self.target_wp == -1 or wp_seq == self.target_wp:
            self.get_logger().info(f'✓ Target waypoint {wp_seq} reached - triggering system')
            
            # MODIFIED: Publish the waypoint sequence number
            trigger_msg = Int16()
            trigger_msg.data = wp_seq
            self.wp_reached_pub.publish(trigger_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MissionMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Mission Monitor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()