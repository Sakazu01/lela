from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Deklarasi argumen saat launch
    use_dummy_vfr_arg = DeclareLaunchArgument(
        'use_dummy_vfr', default_value='true',
        description='Set true for dummy data, false for real MAVROS data'
    )
    enable_gui_arg = DeclareLaunchArgument(
        'enable_gui', default_value='true',
        description='Enable OpenCV GUI windows for debugging'
    )

    # Menangkap nilai argumen
    use_dummy_vfr = LaunchConfiguration('use_dummy_vfr')
    enable_gui = LaunchConfiguration('enable_gui')

    # Definisi Node
    color_detection_node = Node(
        package='opencv_pkg', executable='deteksi_warna', name='color_detection_node'
    )
    servo_controller_node = Node(
        package='opencv_pkg', executable='servo_controller', name='servo_controller_node'
    )
    hsv_config_node = Node(
        package='opencv_pkg', executable='hsv_config', name='hsv_config_node',
        condition=IfCondition(enable_gui) # Hanya jalan jika enable_gui=true
    )
    dummy_vfr_node = Node(
        package='opencv_pkg', executable='dummy_vfr', name='dummy_vfr_node',
        condition=IfCondition(use_dummy_vfr) # Hanya jalan jika use_dummy_vfr=true
    )

    # --- Logika Pilihan Dropping Node ---
    # Jalankan dropping_dummy HANYA JIKA use_dummy_vfr adalah 'true'
    dropping_dummy_node = Node(
        package='opencv_pkg', executable='dropping_dummy', name='dropping_node',
        condition=IfCondition(use_dummy_vfr)
    )
    # Jalankan dropping_real HANYA JIKA use_dummy_vfr adalah 'false'
    dropping_real_node = Node(
        package='opencv_pkg', executable='dropping_real', name='dropping_node',
        condition=UnlessCondition(use_dummy_vfr)
    )
    # ------------------------------------

    return LaunchDescription([
        use_dummy_vfr_arg,
        enable_gui_arg,

        color_detection_node,
        servo_controller_node,
        hsv_config_node,
        dummy_vfr_node,
        dropping_dummy_node,
        dropping_real_node,
    ])