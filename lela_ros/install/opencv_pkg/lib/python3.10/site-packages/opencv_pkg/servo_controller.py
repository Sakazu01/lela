# import rclpy
# from rclpy.node import Node
# from mavros_msgs.srv import CommandLong
# from std_msgs.msg import String
    
# class servo_controller(Node):
#     def __init__(self):
#         super().__init__('servo_control_node')
#         self.client = self.create_client(CommandLong, '/mavros/cmd/command')
#         self.drop_sub = self.create_subscription(
#             String,
#             '/drop_command',
#             self.drop_callback,
#             10
#         )

#         self.drop_executed = False

#         while not self.client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('waiting for mavros cmd command')

#         self.get_logger().info('Servo controller ready')


#     def drop_callback(self, msg):
#         if not self.drop_executed and msg.data.upper() == "DROP":
#             self.get_logger().info("DROP command received, starting drop")
#             self.send_command(servo_number=9, pwm_value=1900)
#             self.drop_executed = True
    
#     def send_command(self, servo_number, pwm_value):
#         req = CommandLong.Request()
#         req.broadcast = False
#         req.command = 183  # MAV_CMD_DO_SET_SERVO
#         req.confirmation = 0
#         req.param1 = float(servo_number)
#         req.param2 = float(pwm_value)
#         req.param3 = 0.0
#         req.param4 = 0.0
#         req.param5 = 0.0
#         req.param6 = 0.0
#         req.param7 = 0.0
#         future = self.client.call_async(req)
#         future.add_done_callback(self.callback)


#     def callback(self, future):
#         try:
#             response = future.result()
#             if response.success:
#                 self.get_logger().info("Servo moved successfully.")
#             else:
#                 self.get_logger().error("Failed to move servo.")
#         except Exception as e:
#             self.get_logger().error(f"Service call failed: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = servo_controller()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()







# import rclpy
# from rclpy.node import Node
# from mavros_msgs.srv import CommandLong

# class ServoController(Node):
#     def __init__(self):
#         super().__init__('servo_control_node')
#         self.client = self.create_client(CommandLong, '/mavros/cmd/command')

#         while not self.client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('waiting for mavros cmd command')

#         self.get_logger().info('Servo controller ready')

#         # Send PWM every 1 second (you can make this faster if you want, e.g. 0.1s)
#         self.timer = self.create_timer(1.0, self.send_servo_command)

#     def send_servo_command(self):
#         req = CommandLong.Request()
#         req.broadcast = False
#         req.command = 183  # MAV_CMD_DO_SET_SERVO
#         req.confirmation = 0
#         req.param1 = float(1)      # Servo number
#         req.param2 = float(1100)   # PWM value
#         req.param3 = 0.0
#         req.param4 = 0.0
#         req.param5 = 0.0
#         req.param6 = 0.0
#         req.param7 = 0.0

#         future = self.client.call_async(req)
#         future.add_done_callback(self.callback)

#     def callback(self, future):
#         try:
#             response = future.result()
#             if response.success:
#                 self.get_logger().info("Servo command sent successfully.")
#             else:
#                 self.get_logger().error("Failed to send servo command.")
#         except Exception as e:
#             self.get_logger().error(f"Service call failed: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = ServoController()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()










# import rclpy
# from rclpy.node import Node
# from mavros_msgs.srv import CommandLong

# class ServoController(Node):
#     def __init__(self):
#         super().__init__('servo_control_node')
#         self.client = self.create_client(CommandLong, '/mavros/cmd/command')

#         while not self.client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('waiting for mavros cmd command')

#         self.get_logger().info('Servo controller ready')

#         # Start with high PWM
#         self.current_pwm = 1900  

#         # Send PWM every 1 second (you can make this faster, e.g. 0.5s or 0.1s)
#         self.timer = self.create_timer(1.0, self.send_servo_command)

#     def send_servo_command(self):
#         req = CommandLong.Request()
#         req.broadcast = False
#         req.command = 183  # MAV_CMD_DO_SET_SERVO
#         req.confirmation = 0
#         req.param1 = float(1)        # Servo number (adjust to your servo channel)
#         req.param2 = float(self.current_pwm)  # Current PWM
#         req.param3 = 0.0
#         req.param4 = 0.0
#         req.param5 = 0.0
#         req.param6 = 0.0
#         req.param7 = 0.0

#         self.get_logger().info(f"Sending PWM: {self.current_pwm}")

#         future = self.client.call_async(req)
#         future.add_done_callback(self.callback)

#         # Toggle PWM for next cycle
#         if self.current_pwm == 1900:
#             self.current_pwm = 1100
#         else:
#             self.current_pwm = 1900

#     def callback(self, future):
#         try:
#             response = future.result()
#             if response.success:
#                 self.get_logger().info("Servo command sent successfully.")
#             else:
#                 self.get_logger().error("Failed to send servo command.")
#         except Exception as e:
#             self.get_logger().error(f"Service call failed: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = ServoController()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#_________________________________________________________________________________________________



# import rclpy
# from rclpy.node import Node
# from mavros_msgs.srv import CommandLong
# from std_msgs.msg import String

# class ServoController(Node):
#     def __init__(self):
#         super().__init__('servo_control_node')
#         self.client = self.create_client(CommandLong, '/mavros/cmd/command')
#         self.drop_sub = self.create_subscription(
#             String,
#             '/drop_command',
#             self.drop_callback,
#             10
#         )

#         self.servo_number = 1
#         self.current_pwm = 1100  # Start at 1100

#         while not self.client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('waiting for mavros cmd command')

#         self.get_logger().info('Servo controller ready')

#     def drop_callback(self, msg):
#         if msg.data.upper() == "DROP":
#             self.get_logger().info("DROP command received, starting servo toggle")
#             self.send_command(self.servo_number, 1900)
#             # Start a timer that toggles servo every second
#             self.timer = self.create_timer(1.0, self.toggle_servo)

#     def toggle_servo(self):
#         # Flip between 1100 and 1900
#         self.get_logger().info("Resetting servo")
#         self.send_command(self.servo_number, 1100)
#         self.timer.cancel()

#     def send_command(self, servo_number, pwm_value):
#         req = CommandLong.Request()
#         req.broadcast = False
#         req.command = 183  # MAV_CMD_DO_SET_SERVO
#         req.confirmation = 0
#         req.param1 = float(servo_number)
#         req.param2 = float(pwm_value)
#         req.param3 = 0.0
#         req.param4 = 0.0
#         req.param5 = 0.0
#         req.param6 = 0.0
#         req.param7 = 0.0
#         future = self.client.call_async(req)
#         future.add_done_callback(self.callback)

#     def callback(self, future):
#         try:
#             response = future.result()
#             if response.success:
#                 self.get_logger().info("Servo moved successfully.")
#             else:
#                 self.get_logger().error("Failed to move servo.")
#         except Exception as e:
#             self.get_logger().error(f"Service call failed: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = ServoController()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#---------------------------------------


import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandLong
from std_msgs.msg import String

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        self.client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.drop_sub = self.create_subscription(
            String,
            '/drop_command',
            self.drop_callback,
            10
        )

        self.servo_number = 1
        self.current_pwm = 1100  # Start at 1100
        self.busy = False        # Prevent overlapping DROP cycles
        self.timer = None

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for mavros cmd command')

        self.get_logger().info('Servo controller ready')

    def drop_callback(self, msg):
        if msg.data.upper() == "DROP":
            if self.busy:
                self.get_logger().warn("DROP command ignored: already in progress")
                return

            self.busy = True
            self.get_logger().info("DROP command received, moving servo to 1900")
            self.send_command(self.servo_number, 1900)

            # Schedule reset after 1 second
            self.timer = self.create_timer(1.0, self.reset_servo)

    def reset_servo(self):
        self.get_logger().info("Resetting servo to 1100")
        self.send_command(self.servo_number, 1100)

        # Cancel timer so it doesn't repeat
        if self.timer:
            self.timer.cancel()
            self.timer = None

        # Allow new DROP commands
        self.busy = False

    def send_command(self, servo_number, pwm_value):
        req = CommandLong.Request()
        req.broadcast = False
        req.command = 183  # MAV_CMD_DO_SET_SERVO
        req.confirmation = 0
        req.param1 = float(servo_number)
        req.param2 = float(pwm_value)
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0
        future = self.client.call_async(req)
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Servo moved successfully.")
            else:
                self.get_logger().error("Failed to move servo.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
