# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# from mavros_msgs.msg import VfrHud
# import math

# class dropping(Node):
#     def __init__(self):
#         super().__init__('dropping_node')
#         self.color_sub = self.create_subscription(
#             String,
#             '/color_warning',
#             self.color_callback,
#             10
#         )
#         self.vfrhub_sub = self.create_subscription(
#             VfrHud,
#             '/mavros/vfr_hud',
#             self.vfr_callback,
#             10
#         )
#         self.drop_pub = self.create_publisher(String, '/drop_command', 10)

#         self.detected_color = None
#         self.altitude = 0.0
#         self.airspeed = 0.0
#         self.target_distance = 30.0
#         self.get_logger().info("dropping node start ")


#     def color_callback(self, msg):
#         self.detected_color = msg.data.lower()
#         if "blue" in self.detected_color:
#             self.get_logger().info("BLUE DETECTED")
    
#     def vfr_callback(self, msg):
#         self.airspeed = msg.airspeed
#         self.altitude = msg.altitude

#         if self.detected_color == "blue":
#             self.rumus_dropping()

#     def rumus_dropping(self):
#         gravitasi = 9.81
#         t_fall = math.sqrt((2 * self.altitude) / gravitasi)
#         d_drop = self.airspeed * t_fall

#         self.get_logger().info(
#             f"Airspeed={self.airspeed:.2f} m/s | "
#             f"Altitude={self.altitude:.2f} m | "
#             f"Drop distance={d_drop:.2f} m"
#         )

#         if self.target_distance <= d_drop:
#             self.pub_drop()

#     def pub_drop(self):
#         msg = String()
#         msg.data = 'DROP'
#         self.drop_pub.publish(msg)
#         self.get_logger().info(f"Published: {msg.data}")






# def main(args=None):
#     rclpy.init(args=args)
#     node = dropping()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.msg import VfrHud
import math

class Dropping(Node):
    def __init__(self):
        super().__init__('dropping_node')
        self.color_sub = self.create_subscription(
            String,
            '/color_warning',
            self.color_callback,
            10
        )
        self.vfrhub_sub = self.create_subscription(
            VfrHud,
            '/mavros/vfr_hud',
            self.vfr_callback,
            10
        )
        self.drop_pub = self.create_publisher(String, '/drop_command', 10)

        # State variables
        self.detected_color = None
        self.altitude = 0.0
        self.airspeed = 0.0
        self.target_distance = 1.0
        self.countdown_timer = None
        self.time_left = 0.0
        self.drop_started = False

        self.get_logger().info("Dropping node started.")

    def color_callback(self, msg):
        self.detected_color = msg.data.lower()

        if "red" in self.detected_color:
            self.get_logger().info("RED DETECTED")
        elif "blue" in self.detected_color:
            self.get_logger().info("BLUE DETECTED --> SKIPPED")

    def vfr_callback(self, msg):
        # self.airspeed = msg.airspeed if msg.airspeed > 0 else (msg.groundspeed if msg.groundspeed > 0 else 1.0)
        self.airspeed = 1.0
        # self.altitude = msg.altitude if msg.altitude > 0 else (15)
        self.altitude = 15

        if self.detected_color == "red" and not self.drop_started:
            self.calculate_drop_timing()


    def calculate_drop_timing(self):
        gravitasi = 9.81
        t_fall = math.sqrt((2 * self.altitude) / gravitasi)
        d_drop = self.airspeed * t_fall

        # Time before needing to drop
        if d_drop > self.target_distance:
            self.time_left = (d_drop - self.target_distance) / self.airspeed
            self.get_logger().info(
                f"Time until drop: {self.time_left:.1f} seconds "
                f"(airspeed={self.airspeed:.2f}, altitude={self.altitude:.2f})"
            )

            # Start countdown timer
            self.countdown_timer = self.create_timer(1.0, self.countdown_callback)
            self.drop_started = True

    def countdown_callback(self):
        if self.time_left > 0:
            self.get_logger().info(f"Countdown: {self.time_left:.0f} seconds left")
            self.time_left -= 1.0
        else:
            self.publish_drop()
            self.destroy_timer(self.countdown_timer)

    def publish_drop(self):
        msg = String()
        msg.data = 'DROP'
        self.drop_pub.publish(msg)
        self.get_logger().info("DROP executed!")
        self.drop_started = False
        self.detected_color = None
        self.time_left = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = Dropping()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
