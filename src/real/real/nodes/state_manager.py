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
        self.return_timer = None
        self.state_timeout_timer = None
        
        # Timeout configuration (seconds)
        self.STATE_TIMEOUTS = {
            SystemState.CAMERA_ACTIVE: 30.0,      # Max 30s untuk deteksi
            SystemState.TARGET_DETECTED: 5.0,     # Max 5s transition ke DROPPING
            SystemState.DROPPING: 10.0,           # Max 10s untuk drop execution
            SystemState.DROP_EXECUTED: 5.0        # Max 5s untuk cleanup
        }
        
        # State entry timestamp untuk monitoring
        self.state_entry_time = self.get_clock().now()
        
        # Timer for state monitoring
        self.create_timer(1.0, self.state_monitor)
        
        self.get_logger().info('State Manager initialized')
        self.transition_to(SystemState.WAITING_WAYPOINT)
    
    def transition_to(self, new_state):
        """State transition dengan timeout management"""
        old_state = self.current_state
        
        # Cancel existing timeout timer
        if self.state_timeout_timer:
            self.state_timeout_timer.cancel()
            self.state_timeout_timer = None
        
        # Update state
        self.current_state = new_state
        self.state_entry_time = self.get_clock().now()
        
        self.get_logger().info(f'State transition: {old_state.name} → {new_state.name}')
        
        # Publish state
        msg = String()
        msg.data = new_state.name
        self.system_state_pub.publish(msg)
        
        # Execute state entry actions
        self.on_state_entry(new_state)
        
        # Start timeout timer if configured
        if new_state in self.STATE_TIMEOUTS:
            timeout = self.STATE_TIMEOUTS[new_state]
            self.state_timeout_timer = self.create_timer(
                timeout, 
                lambda: self.on_state_timeout(new_state)
            )
            self.get_logger().debug(f'State timeout set: {timeout}s')
    
    def on_state_entry(self, state):
        """Actions to perform when entering a state"""
        
        if state == SystemState.WAITING_WAYPOINT:
            # Ensure camera is off
            self._send_camera_command(False)
            self.get_logger().info('Waiting for waypoint...')
        
        elif state == SystemState.CAMERA_ACTIVE:
            # Turn on camera
            self._send_camera_command(True)
            self.get_logger().info('Camera activated - searching for target')
        
        elif state == SystemState.TARGET_DETECTED:
            # Auto-transition to DROPPING state
            self.get_logger().info('Target locked! Initiating drop sequence...')
            # Small delay untuk stability
            self.create_timer(0.5, lambda: self.transition_to(SystemState.DROPPING))
        
        elif state == SystemState.DROPPING:
            # Drop calculator akan handle ini via /system/state subscription
            self.get_logger().info('Drop sequence active - calculating trajectory')
        
        elif state == SystemState.DROP_EXECUTED:
            # Turn off camera
            self._send_camera_command(False)
            self.get_logger().info('Drop completed - preparing for next waypoint')
            
            # Return to WAITING_WAYPOINT setelah 2 detik
            if self.return_timer:
                self.return_timer.cancel()
            self.return_timer = self.create_timer(2.0, self._return_to_waiting)
    
    def _send_camera_command(self, enable):
        """Helper untuk publish camera command"""
        cmd = Bool()
        cmd.data = enable
        self.camera_cmd_pub.publish(cmd)
        self.get_logger().debug(f'Camera command: {"ON" if enable else "OFF"}')
    
    def _return_to_waiting(self):
        """Return to WAITING_WAYPOINT state"""
        if self.return_timer:
            self.return_timer.cancel()
            self.return_timer = None
        self.transition_to(SystemState.WAITING_WAYPOINT)
    
    def on_state_timeout(self, timed_out_state):
        """Handle state timeout - emergency fallback"""
        if self.current_state == timed_out_state:
            self.get_logger().error(
                f'TIMEOUT in state {timed_out_state.name}! '
                f'Forcing transition to IDLE.'
            )
            
            # Emergency cleanup
            self._send_camera_command(False)
            
            # Transition to IDLE then back to WAITING
            self.current_state = SystemState.IDLE
            self.create_timer(1.0, self._return_to_waiting)
    
    def waypoint_callback(self, msg):
        """Handle waypoint reached event"""
        if msg.data and self.current_state == SystemState.WAITING_WAYPOINT:
            self.get_logger().info('✓ Waypoint reached - starting detection')
            self.transition_to(SystemState.CAMERA_ACTIVE)
    
    def color_callback(self, msg):
        """Handle color detection event"""
        detected_color = msg.data.lower()
        
        if self.current_state == SystemState.CAMERA_ACTIVE:
            if detected_color == self.target_color:
                self.get_logger().info(
                    f'✓ Target color "{self.target_color}" detected!'
                )
                self.transition_to(SystemState.TARGET_DETECTED)
            else:
                self.get_logger().debug(
                    f'Non-target color "{detected_color}" detected - ignoring'
                )
    
    def drop_complete_callback(self, msg):
        """Handle drop completion event"""
        if msg.data and self.current_state == SystemState.DROPPING:
            self.get_logger().info('✓ Drop sequence completed successfully')
            self.transition_to(SystemState.DROP_EXECUTED)
    
    def state_monitor(self):
        """Periodic state monitoring for debugging"""
        if self.current_state != SystemState.WAITING_WAYPOINT:
            elapsed = (self.get_clock().now() - self.state_entry_time).nanoseconds / 1e9
            self.get_logger().debug(
                f'Current state: {self.current_state.name} ({elapsed:.1f}s)'
            )

def main(args=None):
    rclpy.init(args=args)
    node = StateManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down State Manager...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()