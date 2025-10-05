#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import WaypointReached
from std_msgs.msg import Bool

class MissionMonitor(Node):
    def __init__(self):
        super().__init__('mission_monitor')
        
        self.declare_parameter('target_waypoint', -1)
        self.target_wp = self.get_parameter('target_waypoint').value
        
        self.wp_sub = self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.waypoint_callback,
            10
        )
        
        self.wp_reached_pub = self.create_publisher(Bool, '/mission/waypoint_reached', 10)
        
        self.get_logger().info(f'Mission Monitor ready (target WP: {self.target_wp})')
    
    def waypoint_callback(self, msg):
        wp_seq = msg.wp_seq
        self.get_logger().info(f'Waypoint {wp_seq} reached')
        
        if self.target_wp == -1 or wp_seq == self.target_wp:
            self.get_logger().info(f'Target waypoint reached - triggering system')
            
            trigger = Bool()
            trigger.data = True
            self.wp_reached_pub.publish(trigger)

def main(args=None):
    rclpy.init(args=args)
    node = MissionMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
