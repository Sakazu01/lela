from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    LELA PRODUCTION LAUNCH FILE
    ============================
    Starts all 7 nodes with optimized parameters
    """
    
    return LaunchDescription([
        
        # ==========================================
        # LAUNCH ARGUMENTS
        # ==========================================
        
        DeclareLaunchArgument(
            'target_waypoint',
            default_value='-1',
            description='Target waypoint (-1 = all waypoints)'
        ),
        
        DeclareLaunchArgument(
            'enable_visualization',
            default_value='false',
            description='Enable OpenCV windows for HSV tuning'
        ),
        
        # ==========================================
        # NODE 1: STATE MANAGER (Orchestrator)
        # ==========================================
        
        Node(
            package='real',
            executable='state_manager',
            name='state_manager',
            output='screen',
            emulate_tty=True
        ),
        
        # ==========================================
        # NODE 2: MISSION MONITOR (Waypoint Tracker)
        # ==========================================
        
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
        
        # ==========================================
        # NODE 3: CAMERA CONTROLLER (Picamera2)
        # ==========================================
        
        Node(
            package='real',
            executable='camera_controller',
            name='camera_controller',
            parameters=[{
                'frame_rate': 10,
                'width': 640,
                'height': 480,
                'use_picamera2': True
            }],
            output='screen',
            emulate_tty=True
        ),
        
        # ==========================================
        # NODE 4: COLOR DETECTOR (HSV + Center)
        # ==========================================
        
        Node(
            package='real',
            executable='color_detector',
            name='color_detector',
            parameters=[{
                # Detection parameters - OPTIMIZED FOR 30-50m ALTITUDE
                'min_area': 2000,              # ← Support up to 50m altitude
                'center_tolerance': 0.4,       # 40% = wider acceptance zone
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
        
        # ==========================================
        # NODE 5: DROP CALCULATOR (Physics)
        # ==========================================
        
        Node(
            package='real',
            executable='drop_calculator',
            name='drop_calculator',
            parameters=[{
                'gravity': 9.81,
                'target_distance': 0.0,     # ← 0.0 for center landing!
                'min_altitude': 15.0,
                'max_altitude': 50.0,
                'min_airspeed': 8.0
            }],
            output='screen',
            emulate_tty=True
        ),
        
        # ==========================================
        # NODE 6: SERVO CONTROLLER (MAVROS)
        # ==========================================
        
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
        ),
        
        # ==========================================
        # NODE 7: MESSAGE PUBLISHER (GCS Telemetry)
        # ==========================================
        
        Node(
            package='real',
            executable='message_publisher',
            name='message_publisher',
            output='screen',
            emulate_tty=True
        )
    ])