#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandLong
from std_msgs.msg import Bool

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        # Parameters
        self.declare_parameter('servo_channel', 9)
        self.declare_parameter('pwm_drop', 1900)
        self.declare_parameter('pwm_hold', 1100)
        self.declare_parameter('reset_delay', 1.0)
        
        # Validate parameters
        self._validate_parameters()
        
        # MAVROS service client
        self.client = self.create_client(CommandLong, '/mavros/cmd/command')
        
        # Wait for service
        self.get_logger().info('Waiting for MAVROS command service...')
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                'MAVROS command service not available after 10s!'
            )
        else:
            self.get_logger().info('MAVROS command service connected')
        
        # Subscribers
        self.drop_sub = self.create_subscription(
            Bool, '/drop/execute', self.drop_callback, 10
        )
        
        # Publishers
        self.complete_pub = self.create_publisher(Bool, '/drop/completed', 10)
        
        # State
        self.busy = False
        self.reset_timer = None
        self.drop_command_success = False
        
        self.get_logger().info('Servo Controller ready')
    
    def _validate_parameters(self):
        """Validate servo parameters"""
        channel = self.get_parameter('servo_channel').value
        pwm_drop = self.get_parameter('pwm_drop').value
        pwm_hold = self.get_parameter('pwm_hold').value
        
        if not (1 <= channel <= 16):
            raise ValueError(f'Servo channel must be 1-16, got {channel}')
        
        if not (900 <= pwm_drop <= 2100):
            raise ValueError(f'PWM drop must be 900-2100, got {pwm_drop}')
        
        if not (900 <= pwm_hold <= 2100):
            raise ValueError(f'PWM hold must be 900-2100, got {pwm_hold}')
        
        self.get_logger().info(
            f'Servo config: CH{channel}, Hold={pwm_hold}, Drop={pwm_drop}'
        )
    
    def drop_callback(self, msg):
        """Handle drop execute command"""
        if not msg.data:
            return
        
        if self.busy:
            self.get_logger().warn('Servo busy, ignoring drop command')
            return
        
        self.busy = True
        self.drop_command_success = False
        self.get_logger().info('Drop command received - executing servo')
        
        servo_ch = self.get_parameter('servo_channel').value
        pwm_drop = self.get_parameter('pwm_drop').value
        
        # Send drop command
        self.send_servo_command(servo_ch, pwm_drop, is_drop=True)
    
    def send_servo_command(self, channel, pwm, is_drop=False):
        """Send MAV_CMD_DO_SET_SERVO command"""
        req = CommandLong.Request()
        req.command = 183  # MAV_CMD_DO_SET_SERVO
        req.param1 = float(channel)
        req.param2 = float(pwm)
        
        future = self.client.call_async(req)
        
        # Attach callback dengan context
        future.add_done_callback(
            lambda f: self.command_callback(f, channel, pwm, is_drop)
        )
    
    def command_callback(self, future, channel, pwm, is_drop):
        """Handle servo command response"""
        try:
            response = future.result()
            
            if response.success:
                self.get_logger().info(f'✓ Servo CH{channel} → {pwm} PWM: SUCCESS')
                
                # If this was a drop command, start reset timer
                if is_drop:
                    self.drop_command_success = True
                    reset_delay = self.get_parameter('reset_delay').value
                    
                    self.get_logger().info(
                        f'Drop servo activated, reset in {reset_delay}s'
                    )
                    
                    # Cancel any existing timer (safety)
                    if self.reset_timer:
                        self.reset_timer.cancel()
                    
                    # Start reset timer
                    self.reset_timer = self.create_timer(reset_delay, self.reset_servo)
            
            else:
                self.get_logger().error(
                    f'✗ Servo CH{channel} → {pwm} PWM: FAILED'
                )
                self.busy = False  # Reset busy flag on failure
                
        except Exception as e:
            self.get_logger().error(f'Service call exception: {e}')
            self.busy = False  # Reset busy flag on exception
    
    def reset_servo(self):
        """Reset servo to hold position"""
        self.get_logger().info('Resetting servo to hold position')
        
        # Cancel timer
        if self.reset_timer:
            self.reset_timer.cancel()
            self.reset_timer = None
        
        servo_ch = self.get_parameter('servo_channel').value
        pwm_hold = self.get_parameter('pwm_hold').value
        
        # Send hold command
        self.send_servo_command(servo_ch, pwm_hold, is_drop=False)
        
        # Publish completion (only if drop command was successful)
        if self.drop_command_success:
            complete = Bool()
            complete.data = True
            self.complete_pub.publish(complete)
            self.get_logger().info('✓ Drop sequence completed')
        
        # Reset state
        self.busy = False
        self.drop_command_success = False

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Servo Controller...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()