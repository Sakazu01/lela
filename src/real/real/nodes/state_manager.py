#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from enum import Enum

class SystemState(Enum):
    IDLE = 0
    WAITING_WAYPOINT = 1
    CAMERA_ACTIVE = 2
    TARGET_DETECTED = 3
    DROPPING = 4
    DROP_EXECUTED = 5

class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')
        
        # Publishers
        self.camera_cmd_pub = self.create_publisher(Bool, '/camera/enable', 10)
        self.system_state_pub = self.create_publisher(String, '/system/state', 10)
        
        # Subscribers
        self.waypoint_sub = self.create_subscription(
            Bool, '/mission/waypoint_reached', self.waypoint_callback, 10)
        self.color_sub = self.create_subscription(
            String, '/detection/color', self.color_callback, 10)
        self.drop_complete_sub = self.create_subscription(
            Bool, '/drop/completed', self.drop_complete_callback, 10)
        
        # State
        self.current_state = SystemState.IDLE
        self.target_color = 'red'
        self.return_timer = None  # TAMBAHKAN INI
        
        # Timer for state monitoring
        self.create_timer(1.0, self.state_monitor)
        
        self.get_logger().info('State Manager initialized - waiting for waypoint')
        self.transition_to(SystemState.WAITING_WAYPOINT)
    
    def transition_to(self, new_state):
        old_state = self.current_state
        self.current_state = new_state
        self.get_logger().info(f'State: {old_state.name} → {new_state.name}')
        
        msg = String()
        msg.data = new_state.name
        self.system_state_pub.publish(msg)
        
        self.on_state_entry(new_state)
    
    def on_state_entry(self, state):
        if state == SystemState.CAMERA_ACTIVE:
            cmd = Bool()
            cmd.data = True
            self.camera_cmd_pub.publish(cmd)
            self.get_logger().info('Camera activated')
        
        elif state == SystemState.DROP_EXECUTED:
            cmd = Bool()
            cmd.data = False
            self.camera_cmd_pub.publish(cmd)
            self.get_logger().info('Camera deactivated')
            # HAPUS one_shot=True
            self.return_timer = self.create_timer(2.0, self.return_to_waiting)
    
    def return_to_waiting(self):
        """Return to WAITING_WAYPOINT state after drop"""
        self.transition_to(SystemState.WAITING_WAYPOINT)
        if self.return_timer:
            self.return_timer.cancel()
            self.return_timer = None

    def waypoint_callback(self, msg):
        if msg.data and self.current_state == SystemState.WAITING_WAYPOINT:
            self.get_logger().info('Waypoint reached - activating camera')
            self.transition_to(SystemState.CAMERA_ACTIVE)
    
    def color_callback(self, msg):
        detected_color = msg.data.lower()
        
        if self.current_state == SystemState.CAMERA_ACTIVE:
            if detected_color == self.target_color:
                self.get_logger().info(f'Target color {self.target_color} detected!')
                self.transition_to(SystemState.TARGET_DETECTED)
            elif detected_color == 'blue':
                self.get_logger().info('Blue detected - skipping')
    
    def drop_complete_callback(self, msg):
        if msg.data and self.current_state == SystemState.DROPPING:
            self.get_logger().info('Drop completed successfully')
            self.transition_to(SystemState.DROP_EXECUTED)
    
    def state_monitor(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = StateManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
