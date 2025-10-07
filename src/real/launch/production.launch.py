from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Declare launch arguments untuk easy override
    return LaunchDescription([
        
        # Launch arguments
        DeclareLaunchArgument(
            'target_waypoint',
            default_value='-1',
            description='Target waypoint index (-1 = any waypoint)'
        ),
        
        DeclareLaunchArgument(
            'enable_visualization',
            default_value='false',
            description='Enable OpenCV visualization windows'
        ),
        
        # State Manager - orchestrator
        Node(
            package='real',
            executable='state_manager',
            name='state_manager',
            output='screen',
            emulate_tty=True
        ),
        
        # Mission Monitor - subscribe to MAVROS waypoint reached
        Node(
            package='real',
            executable='mission_monitor',
            name='mission_monitor',
            parameters=[{
                'target_waypoint': LaunchConfiguration('target_waypoint')
            }],
            output='screen',
            emulate_tty=True
        ),
        
        # Camera Controller - real camera hardware
        Node(
            package='real',
            executable='camera_controller',
            name='camera_controller',
            parameters=[{
                'camera_index': 0,
                'frame_rate': 10
            }],
            output='screen',
            emulate_tty=True
        ),
        
        # Color Detector - HSV-based detection
        Node(
            package='real',
            executable='color_detector',
            name='color_detector',
            parameters=[{
                'min_area': 5000,
                'enable_visualization': LaunchConfiguration('enable_visualization'),
                # Red HSV range 1 (0-10 degrees)
                'red_h_min_1': 0,
                'red_s_min_1': 70,
                'red_v_min_1': 50,
                'red_h_max_1': 10,
                'red_s_max_1': 255,
                'red_v_max_1': 255,
                # Red HSV range 2 (170-180 degrees)
                'red_h_min_2': 170,
                'red_s_min_2': 70,
                'red_v_min_2': 50,
                'red_h_max_2': 180,
                'red_s_max_2': 255,
                'red_v_max_2': 255,
                # Blue HSV range
                'blue_h_min': 100,
                'blue_s_min': 50,
                'blue_v_min': 50,
                'blue_h_max': 130,
                'blue_s_max': 255,
                'blue_v_max': 255
            }],
            output='screen',
            emulate_tty=True
        ),
        
        # Drop Calculator - physics-based calculation
        Node(
            package='real',
            executable='drop_calculator',
            name='drop_calculator',
            parameters=[{
                'gravity': 9.81,
                'target_distance': 2.0,
                'min_altitude': 10.0,
                'max_altitude': 100.0,
                'min_airspeed': 5.0
            }],
            output='screen',
            emulate_tty=True
        ),
        
        # Servo Controller - MAVROS servo command
        Node(
            package='real',
            executable='servo_controller',
            name='servo_controller',
            parameters=[{
                'servo_channel': 9,
                'pwm_drop': 1900,
                'pwm_hold': 1100,
                'reset_delay': 1.0
            }],
            output='screen',
            emulate_tty=True
        )
    ])