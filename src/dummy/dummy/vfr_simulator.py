#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import VfrHud

class VfrSimulator(Node):
    """
    Node untuk simulasi VFR HUD data
    Mengirim fake data untuk testing tanpa ArduPlane SITL
    """
    
    def __init__(self):
        super().__init__('vfr_simulator')
        
        # Publisher ke topic MAVROS VFR HUD
        self.publisher = self.create_publisher(VfrHud, '/mavros/vfr_hud', 10)
        
        # Timer untuk publish data setiap 1 detik
        self.timer = self.create_timer(1.0, self.publish_vfr_data)
        
        # Dummy data
        self.airspeed = 15.0  # m/s
        self.altitude = 50.0  # meter
        self.groundspeed = 15.0
        self.heading = 90
        self.throttle = 70.0
        self.climb = 0.0
        
        self.get_logger().info('VFR Simulator started - publishing fake data')
    
    def publish_vfr_data(self):
        """
        Publish fake VFR HUD data
        """
        msg = VfrHud()
        msg.airspeed = self.airspeed
        msg.groundspeed = self.groundspeed
        msg.heading = self.heading
        msg.throttle = self.throttle
        msg.altitude = self.altitude
        msg.climb = self.climb
        
        self.publisher.publish(msg)
        self.get_logger().info(
            f'Published VFR: airspeed={self.airspeed}, altitude={self.altitude}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = VfrSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
