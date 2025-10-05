#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.msg import VfrHud
import math

class DroppingNode(Node):
    """
    Node untuk logika dropping payload
    - Subscribe: /color_warning (deteksi warna)
    - Subscribe: /mavros/vfr_hud (altitude, airspeed)
    - Publish: /drop_command (trigger dropping)
    - Publish: /servo_command (servo control)
    """
    
    def __init__(self):
        super().__init__('dropping_node')
        
        # Subscribers
        self.color_sub = self.create_subscription(
            String, '/color_warning', self.color_callback, 10
        )
        self.vfr_sub = self.create_subscription(
            VfrHud, '/mavros/vfr_hud', self.vfr_callback, 10
        )
        
        # Publishers
        self.drop_pub = self.create_publisher(String, '/drop_command', 10)
        self.servo_pub = self.create_publisher(String, '/servo_command', 10)
        
        # State variables
        self.detected_color = None
        self.altitude = 0.0
        self.airspeed = 0.0
        self.target_distance = 1.0  # meter
        self.countdown_timer = None
        self.servo_timer = None
        self.time_left = 0.0
        self.drop_started = False
        self.servo_moves = 0
        self.servo_phase = False
        
        self.get_logger().info('Dropping Node started')
    
    def color_callback(self, msg):
        """Callback saat deteksi warna"""
        self.detected_color = msg.data.lower()
        if "red" in self.detected_color:
            self.get_logger().info('RED DETECTED!')
    
    def vfr_callback(self, msg):
        """Callback untuk VFR HUD data"""
        self.altitude = msg.altitude
        self.airspeed = msg.airspeed
        
        # Jika detect red dan belum mulai drop sequence
        if self.detected_color == "red" and not self.drop_started:
            self.calculate_drop_timing()
    
    def calculate_drop_timing(self):
        """Hitung kapan harus drop berdasarkan fisika"""
        if self.airspeed < 0.1 or self.altitude < 1.0:
            self.get_logger().warn('Invalid VFR data, skipping drop calculation')
            return
        
        gravitasi = 9.81  # m/s^2
        
        # Waktu jatuh bebas
        t_fall = math.sqrt((2 * self.altitude) / gravitasi)
        
        # Jarak horizontal saat payload jatuh
        d_drop = self.airspeed * t_fall
        
        if d_drop > self.target_distance:
            # Hitung waktu tunggu sebelum drop
            self.time_left = (d_drop - self.target_distance) / self.airspeed
            
            self.get_logger().info(
                f'Time until drop: {self.time_left:.1f}s '
                f'(airspeed={self.airspeed:.2f} m/s, altitude={self.altitude:.2f} m)'
            )
            
            # Start countdown
            self.countdown_timer = self.create_timer(1.0, self.countdown_callback)
            self.drop_started = True
    
    def countdown_callback(self):
        """Countdown sebelum drop"""
        if self.time_left > 1.0:
            self.get_logger().info(f'Countdown: {self.time_left:.0f}s left')
            self.time_left -= 1.0
        else:
            self.trigger_servo_sequence()
            if self.countdown_timer:
                self.countdown_timer.cancel()
                self.countdown_timer = None
    
    def trigger_servo_sequence(self):
        """Mulai sequence servo pre-drop"""
        self.get_logger().info('Starting servo pre-drop sequence...')
        self.servo_timer = self.create_timer(1.0, self.servo_callback)
    
    def servo_callback(self):
        """Servo sequence: ON-OFF-ON-OFF-ON-OFF lalu DROP"""
        if self.servo_moves < 3:
            pos = "ON" if not self.servo_phase else "OFF"
            msg = String()
            msg.data = f"SERVO {pos}"
            self.servo_pub.publish(msg)
            self.get_logger().info(f'Servo {pos} ({self.servo_moves + 1}/3)')
            
            self.servo_phase = not self.servo_phase
            if self.servo_phase:
                self.servo_moves += 1
        else:
            self.publish_drop()
            if self.servo_timer:
                self.servo_timer.cancel()
                self.servo_timer = None
    
    def publish_drop(self):
        """Publish drop command"""
        msg = String()
        msg.data = 'DROP'
        self.drop_pub.publish(msg)
        self.get_logger().info('DROP EXECUTED!')
        
        # Reset state
        self.drop_started = False
        self.detected_color = None
        self.time_left = 0.0
        self.servo_moves = 0
        self.servo_phase = False

def main(args=None):
    rclpy.init(args=args)
    node = DroppingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
