import rclpy
from rclpy.node import Node
from mavros_msgs.msg import VfrHud

class VfrHudMock(Node):
    def __init__(self):
        super().__init__('vfr_hud_mock')
        self.publisher = self.create_publisher(VfrHud, '/mavros/vfr_hud', 10)
        self.timer = self.create_timer(1.0, self.publish_fake_data)
        self.get_logger().info("Simulating VFR HUD data...")

    def publish_fake_data(self):
        msg = VfrHud()
        msg.airspeed = 20.0     # m/s
        msg.groundspeed = 20.0  # m/s
        msg.heading = 90      # degrees
        msg.throttle = 70.0     # %
        msg.altitude = 50.0     # meters
        msg.climb = 0.0         # m/s
        self.publisher.publish(msg)
        self.get_logger().info("Published fake VFR HUD data")

def main(args=None):
    rclpy.init(args=args)
    node = VfrHudMock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
