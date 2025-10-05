from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # State Manager
        Node(
            package='real',
            executable='state_manager',
            name='state_manager',
            output='screen'
        ),
        
        # Mission Monitor (subscribe to real MAVROS)
        Node(
            package='real',
            executable='mission_monitor',
            name='mission_monitor',
            parameters=[{'target_waypoint': -1}],  # -1 = any waypoint
            output='screen'
        ),
        
        # Camera Controller (real camera)
        Node(
            package='real',
            executable='camera_controller',
            name='camera_controller',
            parameters=[
                {'camera_index': 0},  # Adjust jika /dev/video1 atau lainnya
                {'frame_rate': 10}
            ],
            output='screen'
        ),
        
        # Color Detector
        Node(
            package='real',
            executable='color_detector',
            name='color_detector',
            parameters=[
                {'min_area': 5000},
                {'enable_visualization': False},  # Set True untuk tuning HSV
                # HSV values - sesuaikan dengan lighting real
                {'red_h_min_1': 0}, {'red_h_max_1': 10},
                {'red_s_min_1': 70}, {'red_v_min_1': 50},
                {'red_s_max_1': 255}, {'red_v_max_1': 255},
                {'red_h_min_2': 170}, {'red_h_max_2': 180},
                {'red_s_min_2': 70}, {'red_v_min_2': 50},
                {'red_s_max_2': 255}, {'red_v_max_2': 255}
            ],
            output='screen'
        ),
        
        # Drop Calculator
        Node(
            package='real',
            executable='drop_calculator',
            name='drop_calculator',
            parameters=[
                {'target_distance': 2.0},  # meters before target
                {'min_altitude': 10.0},
                {'max_altitude': 100.0},
                {'min_airspeed': 5.0}
            ],
            output='screen'
        ),
        
        # Servo Controller
        Node(
            package='real',
            executable='servo_controller',
            name='servo_controller',
            parameters=[
                {'servo_channel': 9},  # Sesuaikan dengan Matek setup
                {'pwm_drop': 1900},
                {'pwm_hold': 1100},
                {'reset_delay': 1.0}
            ],
            output='screen'
        )
    ])
