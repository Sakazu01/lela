from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_dummy_vfr_arg = DeclareLaunchArgument(
        'use_dummy_vfr',
        default_value='true',
        description='Use dummy VFR HUD data instead of real MAVROS data'
    )
    
    enable_gui_arg = DeclareLaunchArgument(
        'enable_gui',
        default_value='true', 
        description='Enable OpenCV GUI windows for debugging'
    )
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/image_raw',
        description='Camera image topic name'
    )
    
    servo_channel_arg = DeclareLaunchArgument(
        'servo_channel',
        default_value='1',
        description='Servo channel number for dropping mechanism'
    )

    # Get launch configurations
    use_dummy_vfr = LaunchConfiguration('use_dummy_vfr')
    enable_gui = LaunchConfiguration('enable_gui')
    camera_topic = LaunchConfiguration('camera_topic')
    servo_channel = LaunchConfiguration('servo_channel')

    # Define nodes
    
    # 1. Color Detection Node - Main computer vision node
    color_detection_node = Node(
        package='opencv_pkg',
        executable='deteksi_warna',
        name='color_detection_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'camera_topic': camera_topic
        }],
        remappings=[
            ('/image_raw', camera_topic),
        ]
    )
    
    # 2. Dropping Logic Node - Handles drop timing calculations
    dropping_node = Node(
        package='opencv_pkg',
        executable='dropping',
        name='dropping_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'target_distance': 1.0,  # meters before target to drop
        }]
    )
    
    # 3. Servo Controller Node - Controls the dropping mechanism
    servo_controller_node = Node(
        package='opencv_pkg',
        executable='servo_controller',
        name='servo_controller_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'servo_channel': servo_channel
        }]
    )
    
    # 4. Status Text Sender Node - Sends status messages to ground station
    status_sender_node = Node(
        package='opencv_pkg',
        executable='status_text_sender',
        name='status_text_sender_node',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    # 5. HSV Configuration Node - For real-time color tuning (optional)
    hsv_config_node = Node(
        package='opencv_pkg',
        executable='hsv_config',
        name='hsv_config_node',
        output='screen',
        condition=IfCondition(enable_gui),
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    # 6. Dummy VFR Node - Simulates flight data when not connected to real drone
    dummy_vfr_node = Node(
        package='opencv_pkg', 
        executable='dummy_vfr',
        name='dummy_vfr_node',
        output='screen',
        condition=IfCondition(use_dummy_vfr),
        parameters=[{
            'use_sim_time': False,
            'altitude': 50.0,      # meters
            'airspeed': 20.0,      # m/s
            'groundspeed': 20.0,   # m/s
            'heading': 90,         # degrees
            'throttle': 70.0       # %
        }]
    )

    # Group nodes for better organization
    vision_group = GroupAction([
        color_detection_node,
        hsv_config_node
    ])
    
    flight_control_group = GroupAction([
        dropping_node,
        servo_controller_node,
        status_sender_node
    ])
    
    simulation_group = GroupAction([
        dummy_vfr_node
    ], condition=IfCondition(use_dummy_vfr))

    return LaunchDescription([
        # Launch arguments
        use_dummy_vfr_arg,
        enable_gui_arg, 
        camera_topic_arg,
        servo_channel_arg,
        
        # Node groups
        vision_group,
        flight_control_group,
        simulation_group,
    ])


# Alternative simple launch for testing individual components
def generate_test_launch_description():
    """Simplified launch for testing specific components"""
    return LaunchDescription([
        # Only color detection and dummy data
        Node(
            package='opencv_pkg',
            executable='deteksi_warna',
            name='color_detection_test',
            output='screen'
        ),
        Node(
            package='opencv_pkg',
            executable='dummy_vfr', 
            name='dummy_vfr_test',
            output='screen'
        ),
        Node(
            package='opencv_pkg',
            executable='hsv_config',
            name='hsv_config_test',
            output='screen'
        )
    ])

# Usage examples:
"""
# Launch all nodes with real MAVROS data:
ros2 launch opencv_pkg main_launch.py use_dummy_vfr:=false

# Launch with dummy VFR data (for testing):
ros2 launch opencv_pkg main_launch.py use_dummy_vfr:=true

# Launch without GUI windows:
ros2 launch opencv_pkg main_launch.py enable_gui:=false

# Launch with custom camera topic:
ros2 launch opencv_pkg main_launch.py camera_topic:=/camera/image_raw

# Launch with custom servo channel:
ros2 launch opencv_pkg main_launch.py servo_channel:=9

# Launch test configuration:
ros2 launch opencv_pkg test_launch.py
"""