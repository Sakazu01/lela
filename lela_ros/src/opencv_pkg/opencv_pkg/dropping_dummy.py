import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.msg import VfrHud
import math

class DroppingDummy(Node):
    def __init__(self):
        super().__init__('dropping_dummy_node')
        self.color_sub = self.create_subscription(
            String, '/color_warning', self.color_callback, 10
        )
        # We still subscribe to VFR to trigger the callback, but we ignore its data
        self.vfrhub_sub = self.create_subscription(
            VfrHud, '/mavros/vfr_hud', self.vfr_callback, 10
        )
        self.drop_pub = self.create_publisher(String, '/drop_command', 10)
        self.servo_pub = self.create_publisher(String, '/servo_command', 10)

        # State variables
        self.detected_color = None
        self.target_distance = 1.0
        self.countdown_timer = None
        self.time_left = 0.0
        self.drop_started = False
        self.servo_moves = 0
        self.servo_phase = False

        self.get_logger().info("Dropping node (DUMMY DATA) started.")

    def color_callback(self, msg):
        self.detected_color = msg.data.lower()
        if "red" in self.detected_color:
            self.get_logger().info("RED DETECTED")

    def vfr_callback(self, msg):
        # --- MENGGUNAKAN DATA DUMMY/HARDCODED ---
        self.airspeed = 15.0 # m/s
        self.altitude = 50.0 # meter
        # ----------------------------------------

        if self.detected_color == "red" and not self.drop_started:
            self.calculate_drop_timing()

    def calculate_drop_timing(self):
        gravitasi = 9.81
        t_fall = math.sqrt((2 * self.altitude) / gravitasi)
        d_drop = self.airspeed * t_fall

        if d_drop > self.target_distance:
            self.time_left = (d_drop - self.target_distance) / self.airspeed
            self.get_logger().info(
                f"Time until drop: {self.time_left:.1f}s "
                f"(DUMMY airspeed={self.airspeed:.2f}, altitude={self.altitude:.2f})"
            )
            self.countdown_timer = self.create_timer(1.0, self.countdown_callback)
            self.drop_started = True

    def countdown_callback(self):
        if self.time_left > 1.0:
            self.get_logger().info(f"Countdown: {self.time_left:.0f}s left")
            self.time_left -= 1.0
        else:
            self.trigger_servo_sequence()
            if self.countdown_timer:
                self.destroy_timer(self.countdown_timer)

    def trigger_servo_sequence(self):
        self.get_logger().info("Starting servo pre-drop sequence...")
        self.servo_timer = self.create_timer(1.0, self.servo_callback)

    def servo_callback(self):
        if self.servo_moves < 3:
            pos = "ON" if not self.servo_phase else "OFF"
            msg = String()
            msg.data = f"SERVO {pos}"
            self.servo_pub.publish(msg)
            self.get_logger().info(f"Servo {pos} ({self.servo_moves+1}/3)")
            self.servo_phase = not self.servo_phase
            if self.servo_phase:
                self.servo_moves += 1
        else:
            self.publish_drop()
            if self.servo_timer:
                self.destroy_timer(self.servo_timer)

    def publish_drop(self):
        msg = String()
        msg.data = 'DROP'
        self.drop_pub.publish(msg)
        self.get_logger().info("DROP executed!")
        # Reset state
        self.drop_started = False
        self.detected_color = None
        self.time_left = 0.0
        self.servo_moves = 0
        self.servo_phase = False

def main(args=None):
    rclpy.init(args=args)
    node = DroppingDummy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()