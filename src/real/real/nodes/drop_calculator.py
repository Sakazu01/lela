#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Bool
from mavros_msgs.msg import VfrHud
from geometry_msgs.msg import Vector3
import math

class DropCalculator(Node):
    def __init__(self):
        super().__init__('drop_calculator')
        
        # Parameters
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('target_distance', 2.0)
        self.declare_parameter('min_altitude', 10.0)
        self.declare_parameter('max_altitude', 100.0)
        self.declare_parameter('min_airspeed', 5.0)
        
        # Validate parameters
        self._validate_parameters()
        
        # State variables
        self.altitude = 0.0
        self.airspeed = 0.0
        self.groundspeed = 0.0
        self.drop_active = False
        self.calculation_done = False
        self.countdown_timer = None
        
        # Subscribers (use sensor QoS for VFR data)
        self.vfr_sub = self.create_subscription(
            VfrHud, '/mavros/vfr_hud', self.vfr_callback, qos_profile_sensor_data
        )
        self.state_sub = self.create_subscription(
            String, '/system/state', self.state_callback, 10
        )
        
        # Publishers
        self.drop_cmd_pub = self.create_publisher(Bool, '/drop/execute', 10)
        self.drop_info_pub = self.create_publisher(Vector3, '/drop/info', 10)
        
        self.get_logger().info('Drop Calculator ready')
    
    def _validate_parameters(self):
        """Validate parameter ranges"""
        min_alt = self.get_parameter('min_altitude').value
        max_alt = self.get_parameter('max_altitude').value
        min_speed = self.get_parameter('min_airspeed').value
        
        if min_alt >= max_alt:
            raise ValueError(
                f'min_altitude ({min_alt}) must be < max_altitude ({max_alt})'
            )
        
        if min_speed <= 0:
            raise ValueError(f'min_airspeed must be > 0')
        
        self.get_logger().info(
            f'Altitude range: {min_alt}-{max_alt}m, Min speed: {min_speed}m/s'
        )
    
    def state_callback(self, msg):
        """Handle state changes"""
        state = msg.data
        
        if state == 'DROPPING':
            # Enable drop calculation
            self.drop_active = True
            self.calculation_done = False
            self.get_logger().info('Drop calculation ENABLED')
        
        else:
            # Disable drop calculation
            if self.drop_active:
                self.get_logger().info('Drop calculation DISABLED')
            
            self.drop_active = False
            self.calculation_done = False
            
            # Cancel any active countdown
            if self.countdown_timer:
                self.get_logger().warn('Cancelling active countdown timer')
                self.countdown_timer.cancel()
                self.countdown_timer = None
    
    def vfr_callback(self, msg):
        """Update vehicle flight data"""
        self.altitude = msg.altitude
        self.airspeed = msg.airspeed
        self.groundspeed = msg.groundspeed
        
        # Only calculate if drop is active AND not already calculated
        if self.drop_active and not self.calculation_done:
            self.calculate_drop()
    
    def calculate_drop(self):
        """Calculate drop timing and execute countdown"""
        
        # Get parameters
        g = self.get_parameter('gravity').value
        target_dist = self.get_parameter('target_distance').value
        min_alt = self.get_parameter('min_altitude').value
        max_alt = self.get_parameter('max_altitude').value
        min_speed = self.get_parameter('min_airspeed').value
        
        # Validate altitude
        if self.altitude < min_alt:
            self.get_logger().warn(
                f'Altitude too low: {self.altitude:.1f}m < {min_alt}m'
            )
            return
        
        if self.altitude > max_alt:
            self.get_logger().warn(
                f'Altitude too high: {self.altitude:.1f}m > {max_alt}m'
            )
            return
        
        # Select speed (prefer airspeed)
        speed = self.airspeed if self.airspeed >= min_speed else self.groundspeed
        
        if speed < min_speed:
            self.get_logger().warn(
                f'Speed too low: {speed:.1f}m/s < {min_speed}m/s'
            )
            return
        
        # Calculate free fall time
        t_fall = math.sqrt((2 * self.altitude) / g)
        
        # Calculate horizontal distance during fall
        d_drop = speed * t_fall
        
        # Calculate time until drop point
        if d_drop <= target_dist:
            self.get_logger().warn(
                f'Drop distance {d_drop:.1f}m ≤ target {target_dist}m - '
                f'payload will overshoot!'
            )
            return
        
        time_to_drop = (d_drop - target_dist) / speed
        
        # Sanity check
        if time_to_drop <= 0:
            self.get_logger().error('Calculated time_to_drop ≤ 0, skipping')
            return
        
        # Publish drop info
        info = Vector3()
        info.x = time_to_drop
        info.y = d_drop
        info.z = self.altitude
        self.drop_info_pub.publish(info)
        
        self.get_logger().info(
            f'Drop calculation: '
            f't_fall={t_fall:.2f}s, '
            f'd_drop={d_drop:.1f}m, '
            f'countdown={time_to_drop:.2f}s'
        )
        
        # Mark calculation as done to prevent re-calculation
        self.calculation_done = True
        
        # Start countdown timer
        self.start_countdown(time_to_drop)
    
    def start_countdown(self, delay):
        """Start countdown timer"""
        
        # Double-check no active timer (safety)
        if self.countdown_timer:
            self.get_logger().error('Timer already active! This should not happen.')
            return
        
        self.get_logger().info(f'🚀 Countdown started: {delay:.2f} seconds')
        self.countdown_timer = self.create_timer(delay, self.execute_drop)
    
    def execute_drop(self):
        """Execute drop command"""
        self.get_logger().info('💥 EXECUTING DROP!')
        
        # Send drop command
        cmd = Bool()
        cmd.data = True
        self.drop_cmd_pub.publish(cmd)
        
        # Cleanup
        if self.countdown_timer:
            self.countdown_timer.cancel()
            self.countdown_timer = None
        
        self.drop_active = False
        self.calculation_done = False

def main(args=None):
    rclpy.init(args=args)
    node = DropCalculator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Drop Calculator...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()