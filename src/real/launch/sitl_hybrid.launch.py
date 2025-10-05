from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='real', executable='state_manager', name='state_manager', output='screen'),
        Node(package='real', executable='mission_monitor', name='mission_monitor', parameters=[{'target_waypoint': -1}], output='screen'),
        Node(package='real', executable='dummy_camera', name='dummy_camera', parameters=[{'color_to_show': 'red'}], output='screen'),
        Node(package='real', executable='color_detector', name='color_detector', parameters=[{'min_area': 500}, {'red_s_min_1': 0}, {'red_v_min_1': 0}, {'red_s_min_2': 0}, {'red_v_min_2': 0}], output='screen'),
        Node(package='real', executable='drop_calculator', name='drop_calculator', parameters=[{'target_distance': 2.0}], output='screen'),
        Node(package='real', executable='servo_controller', name='servo_controller', parameters=[{'servo_channel': 9}], output='screen')
    ])
