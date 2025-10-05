#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandLong
from std_msgs.msg import String

class ServoNode(Node):
    """
    Node untuk kontrol servo via MAVROS
    - Subscribe: /drop_command (trigger drop)
    - Service call: /mavros/cmd/command (MAV_CMD_DO_SET_SERVO)
    """
    
    def __init__(self):
        super().__init__('servo_node')
        
        # Declare parameter
        self.declare_parameter('dummy_mode', False)
        self.dummy_mode = self.get_parameter('dummy_mode').value
        
        if not self.dummy_mode:
            # Real mode - tunggu MAVROS
            self.client = self.create_client(CommandLong, '/mavros/cmd/command')
            while not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for MAVROS command service...')
        else:
            # Dummy mode - skip MAVROS
            self.get_logger().info('DUMMY MODE - No MAVROS connection')
        
        # Subscribe ke drop command
        self.drop_sub = self.create_subscription(
            String,
            '/drop_command',
            self.drop_callback,
            10
        )
        
        # Servo parameters
        self.servo_number = 1
        self.pwm_open = 1900
        self.pwm_close = 1100
        self.busy = False
        self.timer = None
        
        self.get_logger().info('Servo Node ready')
    
    def drop_callback(self, msg):
        """Callback saat menerima DROP command"""
        if msg.data.upper() == "DROP":
            if self.busy:
                self.get_logger().warn('DROP command ignored: already in progress')
                return
            
            self.busy = True
            self.get_logger().info(f'DROP command received - Moving servo to {self.pwm_open}')
            self.send_servo_command(self.servo_number, self.pwm_open)
            
            # Schedule reset setelah 1 detik
            self.timer = self.create_timer(1.0, self.reset_servo)
    
    def reset_servo(self):
        """Reset servo ke posisi awal"""
        self.get_logger().info(f'Resetting servo to {self.pwm_close}')
        self.send_servo_command(self.servo_number, self.pwm_close)
        
        # Cancel timer
        if self.timer:
            self.timer.cancel()
            self.timer = None
        
        # Allow new DROP commands
        self.busy = False
    
    def send_servo_command(self, servo_num, pwm_value):
        """
        Kirim command ke MAVROS untuk set servo PWM
        Args:
            servo_num: Nomor servo (1-16)
            pwm_value: PWM value (1000-2000)
        """
        if self.dummy_mode:
            # Dummy mode - just log
            self.get_logger().info(f'[DUMMY] Servo {servo_num} set to {pwm_value} PWM')
            return
        
        # Real mode - kirim ke MAVROS
        req = CommandLong.Request()
        req.broadcast = False
        req.command = 183  # MAV_CMD_DO_SET_SERVO
        req.confirmation = 0
        req.param1 = float(servo_num)
        req.param2 = float(pwm_value)
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0
        
        future = self.client.call_async(req)
        future.add_done_callback(self.command_response_callback)
    
    def command_response_callback(self, future):
        """Callback untuk response dari MAVROS"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Servo command successful')
            else:
                self.get_logger().error('Servo command failed')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()