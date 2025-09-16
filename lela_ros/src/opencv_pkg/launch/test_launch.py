from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Simple test launch for basic functionality testing"""
    
    return LaunchDescription([
        # Color detection node with HSV tuning
        Node(
            package='opencv_pkg',
            executable='hsv_config',
            name='hsv_tuning_test',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        # Dummy VFR data for testing drop calculations
        Node(
            package='opencv_pkg',
            executable='dummy_vfr',
            name='dummy_vfr_test', 
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        # Dropping node for testing logic
        Node(
            package='opencv_pkg',
            executable='dropping',
            name='dropping_test',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'target_distance': 1.0
            }]
        ),
        
        # Servo controller for testing drop mechanism
        Node(
            package='opencv_pkg',
            executable='servo_controller', 
            name='servo_test',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        )
    ])

# Usage:
# ros2 launch opencv_pkg test_launch.py