from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # VFR Simulator - generate fake VFR data
        Node(
            package='dummy',
            executable='vfr_simulator',
            name='vfr_simulator',
            output='screen'
        ),
        
        # Waypoint Monitor - monitor waypoint reached dari MAVROS
        Node(
            package='dummy',
            executable='waypoint_monitor',
            name='waypoint_monitor',
            output='screen'
        ),
        
        # Dropping Node - trigger drop based on color
        Node(
            package='dummy',
            executable='dropping_node',
            name='dropping_node',
            output='screen'
        ),
        
        # Servo Node - dengan dummy mode
        Node(
            package='dummy',
            executable='servo_node',
            name='servo_node',
            output='screen',
            parameters=[{'dummy_mode': True}]
        ),
    ])