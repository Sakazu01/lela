#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int16
from mavros_msgs.msg import StatusText # <--- TAMBAHKAN INI
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
        
        # Parameters
        self.declare_parameter('target_color', 'red')  # NEW: Target color is now a parameter
        
        # Publishers
        self.camera_cmd_pub = self.create_publisher(Bool, '/camera/enable', 10)
        self.system_state_pub = self.create_publisher(String, '/system/state', 10)
        self.gcs_pub = self.create_publisher(String, '/mavlink/statustext/send', 10) # NEW: GCS publisher
        
        # Subscribers
        self.waypoint_sub = self.create_subscription(
            Int16, '/mission/waypoint_reached', self.waypoint_callback, 10) # MODIFIED: Subscribes to Int16
        self.color_sub = self.create_subscription(
            String, '/detection/color', self.color_callback, 10)
        self.drop_complete_sub = self.create_subscription(
            Bool, '/drop/completed', self.drop_complete_callback, 10)
        
        # State
        self.current_state = SystemState.IDLE
        self.target_color = self.get_parameter('target_color').value.lower()
        self.current_waypoint = -1 # NEW: Track the active waypoint
        self.return_timer = None
        self.state_timeout_timer = None
        
        # Timeout configuration (seconds)
        self.STATE_TIMEOUTS = {
            SystemState.CAMERA_ACTIVE: 30.0,
            SystemState.TARGET_DETECTED: 5.0,
            SystemState.DROPPING: 10.0,
            SystemState.DROP_EXECUTED: 5.0
        }
        
        self.state_entry_time = self.get_clock().now()
        self.create_timer(1.0, self.state_monitor)
        
        self.get_logger().info(f'State Manager initialized, target color: {self.target_color.upper()}')
        self.transition_to(SystemState.WAITING_WAYPOINT)
    
    def _publish_to_gcs(self, message):
        """Helper to publish a formatted MAVLink STATUSTEXT message to GCS."""
        gcs_msg = StatusText()
        # Severity level untuk MAVLink. 6 = MAV_SEVERITY_INFO
        gcs_msg.severity = 6
        # Pesan teks yang akan ditampilkan di GCS
        gcs_msg.text = f"LELA: WP{self.current_waypoint}: {message}"
        self.gcs_pub.publish(gcs_msg)
        self.get_logger().info(f'📡 Sent to GCS (MAVLink): "{gcs_msg.text}"')

    def transition_to(self, new_state):
        if self.state_timeout_timer:
            self.state_timeout_timer.cancel()
        
        old_state = self.current_state
        self.current_state = new_state
        self.state_entry_time = self.get_clock().now()
        
        self.get_logger().info(f'State transition: {old_state.name} → {new_state.name}')
        
        msg = String()
        msg.data = new_state.name
        self.system_state_pub.publish(msg)
        
        self.on_state_entry(new_state)
        
        if new_state in self.STATE_TIMEOUTS:
            timeout = self.STATE_TIMEOUTS[new_state]
            self.state_timeout_timer = self.create_timer(
                timeout, lambda: self.on_state_timeout(new_state))
    
    def on_state_entry(self, state):
        if state == SystemState.WAITING_WAYPOINT:
            self._send_camera_command(False)
            self.get_logger().info('Waiting for waypoint...')
        
        elif state == SystemState.CAMERA_ACTIVE:
            self._send_camera_command(True)
            self.get_logger().info('Camera activated - searching for target')
        
        elif state == SystemState.TARGET_DETECTED:
            self.get_logger().info('Target locked! Initiating drop sequence...')
            self.create_timer(0.5, lambda: self.transition_to(SystemState.DROPPING))
        
        elif state == SystemState.DROP_EXECUTED:
            self._send_camera_command(False)
            self.get_logger().info('Drop completed - preparing for next waypoint')
            
            if self.return_timer: self.return_timer.cancel()
            self.return_timer = self.create_timer(2.0, self._return_to_waiting)
    
    def _send_camera_command(self, enable):
        cmd = Bool()
        cmd.data = enable
        self.camera_cmd_pub.publish(cmd)
    
    def _return_to_waiting(self):
        if self.return_timer: self.return_timer.cancel()
        self.transition_to(SystemState.WAITING_WAYPOINT)
    
    def on_state_timeout(self, timed_out_state):
        if self.current_state == timed_out_state:
            self.get_logger().error(f'TIMEOUT in state {timed_out_state.name}! Forcing transition to IDLE.')
            
            # NEW: Report timeout to GCS
            if timed_out_state == SystemState.CAMERA_ACTIVE:
                self._publish_to_gcs("Target not detected (Timeout)")
            
            self._send_camera_command(False)
            self.current_state = SystemState.IDLE
            self.create_timer(1.0, self._return_to_waiting)
    
    def waypoint_callback(self, msg):
        """MODIFIED: Handle waypoint reached event with waypoint number"""
        if self.current_state == SystemState.WAITING_WAYPOINT:
            self.current_waypoint = msg.data  # Store current waypoint number
            self.get_logger().info(f'✓ Waypoint {self.current_waypoint} reached - starting detection')
            self.transition_to(SystemState.CAMERA_ACTIVE)
    
    def color_callback(self, msg):
        detected_color = msg.data.lower()
        
        if self.current_state == SystemState.CAMERA_ACTIVE:
            if detected_color == self.target_color:
                self.get_logger().info(f'✓ Target color "{self.target_color}" detected!')
                self._publish_to_gcs(f"{self.target_color.capitalize()} detected") # NEW: Report to GCS
                self.transition_to(SystemState.TARGET_DETECTED)
    
    def drop_complete_callback(self, msg):
        if msg.data and self.current_state == SystemState.DROPPING:
            self.get_logger().info('✓ Drop sequence completed successfully')
            self._publish_to_gcs("Drop successful") # NEW: Report to GCS
            self.transition_to(SystemState.DROP_EXECUTED)
    
    def state_monitor(self):
        if self.current_state != SystemState.WAITING_WAYPOINT:
            elapsed = (self.get_clock().now() - self.state_entry_time).nanoseconds / 1e9
            self.get_logger().debug(f'Current state: {self.current_state.name} ({elapsed:.1f}s)')

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