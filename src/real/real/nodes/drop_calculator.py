#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from mavros_msgs.msg import VfrHud
from geometry_msgs.msg import Vector3
import math

class DropCalculator(Node):
    def __init__(self):
        super().__init__('drop_calculator')
        
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('target_distance', 2.0)
        self.declare_parameter('min_altitude', 10.0)
        self.declare_parameter('max_altitude', 100.0)
        self.declare_parameter('min_airspeed', 5.0)
        
        self.altitude = 0.0
        self.airspeed = 0.0
        self.groundspeed = 0.0
        self.drop_enabled = False
        self.drop_executed = False
        self.countdown_timer = None
        
        self.vfr_sub = self.create_subscription(
            VfrHud, '/mavros/vfr_hud', self.vfr_callback, 10)
        self.state_sub = self.create_subscription(
            String, '/system/state', self.state_callback, 10)
        
        self.drop_cmd_pub = self.create_publisher(Bool, '/drop/execute', 10)
        self.drop_info_pub = self.create_publisher(Vector3, '/drop/info', 10)
        
        self.get_logger().info('Drop Calculator ready')
    
    def state_callback(self, msg):
        if msg.data == 'TARGET_DETECTED':
            self.drop_enabled = True
            self.drop_executed = False
            self.get_logger().info('Drop calculation enabled')
        else:
            self.drop_enabled = False
    
    def vfr_callback(self, msg):
        self.altitude = msg.altitude
        self.airspeed = msg.airspeed
        self.groundspeed = msg.groundspeed
        
        if self.drop_enabled and not self.drop_executed:
            self.calculate_drop()
    
    def calculate_drop(self):
        g = self.get_parameter('gravity').value
        target_dist = self.get_parameter('target_distance').value
        min_alt = self.get_parameter('min_altitude').value
        max_alt = self.get_parameter('max_altitude').value
        min_speed = self.get_parameter('min_airspeed').value
        
        if self.altitude < min_alt or self.altitude > max_alt:
            self.get_logger().warn(f'Altitude {self.altitude:.1f}m out of range')
            return
        
        speed = self.airspeed if self.airspeed >= min_speed else self.groundspeed
        
        if speed < min_speed:
            self.get_logger().warn(f'Speed {speed:.1f} m/s too low')
            return
        
        t_fall = math.sqrt((2 * self.altitude) / g)
        d_drop = speed * t_fall
        
        if d_drop > target_dist:
            time_to_drop = (d_drop - target_dist) / speed
        else:
            self.get_logger().warn(f'Drop distance {d_drop:.1f}m < target {target_dist}m')
            return
        
        info = Vector3()
        info.x = time_to_drop
        info.y = d_drop
        info.z = self.altitude
        self.drop_info_pub.publish(info)
        
        self.get_logger().info(
            f'Drop calc: t_fall={t_fall:.2f}s, d_drop={d_drop:.1f}m, time_to_drop={time_to_drop:.1f}s'
        )
        
        if time_to_drop > 0 and not self.countdown_timer:
            self.start_countdown(time_to_drop)
    
    def start_countdown(self, delay):
        """Start countdown timer"""
        self.get_logger().info(f'Countdown started: {delay:.1f} seconds')
        self.countdown_timer = self.create_timer(delay, self.execute_drop)  # HAPUS one_shot=True
    
    def execute_drop(self):
        self.get_logger().info('EXECUTING DROP!')
        
        cmd = Bool()
        cmd.data = True
        self.drop_cmd_pub.publish(cmd)
        
        self.drop_executed = True
        self.drop_enabled = False
        
        if self.countdown_timer:
            self.countdown_timer.cancel()
            self.countdown_timer = None

def main(args=None):
    rclpy.init(args=args)
    node = DropCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
