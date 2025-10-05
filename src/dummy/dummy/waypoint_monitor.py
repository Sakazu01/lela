#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import WaypointReached
from std_msgs.msg import String

class WaypointMonitor(Node):
    """
    Monitor waypoint reached, publish color_warning saat sampai waypoint tertentu
    """
    
    def __init__(self):
        super().__init__('waypoint_monitor')
        
        # Subscribe waypoint reached
        self.waypoint_sub = self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.waypoint_callback,
            10
        )
        
        # Publish color warning
        self.color_pub = self.create_publisher(String, '/color_warning', 10)
        
        # Waypoint number untuk trigger drop
        self.drop_waypoint = 1  # Drop di waypoint ke-3
        
        self.get_logger().info(f'Waypoint Monitor started - will drop at waypoint {self.drop_waypoint}')
    
    def waypoint_callback(self, msg):
        """Callback saat pesawat sampai waypoint"""
        waypoint_num = msg.wp_seq
        self.get_logger().info(f'Reached waypoint {waypoint_num}')
        
        if waypoint_num == self.drop_waypoint:
            self.get_logger().info(f'Waypoint {waypoint_num} reached - Publishing RED!')
            
            # Publish red untuk trigger dropping
            color_msg = String()
            color_msg.data = 'red'
            self.color_pub.publish(color_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
