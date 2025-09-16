OpenCV Package - Autonomous Color Detection and Dropping System

A ROS2 package for computer vision-based color detection and autonomous dropping system for drones using OpenCV and MAVROS.
Features

    Real-time color detection (red and blue objects) using HSV color space
    Autonomous dropping mechanism with physics-based timing calculations
    Servo control for releasing payloads
    MAVROS integration for drone communication
    Interactive HSV tuning with trackbars for color calibration
    Status messaging to ground control station

Package Structure

lela/
├── src/
│   └── opencv_pkg/
│       ├── package.xml
│       ├── setup.py
│       ├── resource/opencv_pkg
│       └── opencv_pkg/
│           ├── __init__.py
│           ├── deteksi_warna.py      # Main color detection node
│           ├── program_warna.py      # Color processing algorithms
│           ├── hsv_config.py         # HSV configuration class
│           ├── dropping.py           # Drop timing calculations
│           ├── servo_controller.py   # Servo control for dropping
│           ├── dummy_vfr.py         # VFR data simulator
│           └── status_text_sender.py # Status messages

Dependencies
ROS2 Dependencies

    rclpy - ROS2 Python client library
    std_msgs - Standard message types
    sensor_msgs - Sensor message types (Image)
    cv_bridge - OpenCV-ROS2 bridge
    mavros_msgs - MAVROS message types

Python Dependencies

    opencv-python - Computer vision library
    numpy - Numerical computing

Installation

    Clone or create your workspace:
    bash

    mkdir -p ~/lela/src
    cd ~/lela/src

    Place your package in the src directory
    Install Python dependencies:
    bash

    pip3 install opencv-python numpy

    Build the package:
    bash

    cd ~/lela
    colcon build --packages-select opencv_pkg

    Source the workspace:
    bash

    source install/setup.bash

Usage
Individual Nodes

Run individual nodes for testing:
bash

# Color detection node
ros2 run opencv_pkg deteksi_warna

# HSV configuration tool
ros2 run opencv_pkg hsv_config

# Dropping logic node
ros2 run opencv_pkg dropping

# Servo controller
ros2 run opencv_pkg servo_controller

# Dummy VFR data (for testing without drone)
ros2 run opencv_pkg dummy_vfr

# Status text sender
ros2 run opencv_pkg status_text_sender

Launch Files

Full system with dummy VFR data (for testing):
bash

ros2 launch opencv_pkg main_launch.py use_dummy_vfr:=true

Full system with real MAVROS data:
bash

ros2 launch opencv_pkg main_launch.py use_dummy_vfr:=false

Without GUI windows (headless mode):
bash

ros2 launch opencv_pkg main_launch.py enable_gui:=false

Custom camera topic:
bash

ros2 launch opencv_pkg main_launch.py camera_topic:=/camera/image_raw

Custom servo channel:
bash

ros2 launch opencv_pkg main_launch.py servo_channel:=9

System Architecture
Node Communication

/image_raw ──→ [deteksi_warna] ──→ /color_warning ──→ [dropping]
                                                        │
/mavros/vfr_hud ──────────────────────────────────────→ │
                                                        │
                                    /drop_command ←──── │
                                           │
                                           ↓
                                   [servo_controller] ──→ MAVROS servo commands

Topics

    /image_raw - Camera input (sensor_msgs/Image)
    /color_warning - Detected colors (std_msgs/String)
    /mavros/vfr_hud - Flight data (mavros_msgs/VfrHud)
    /drop_command - Drop trigger (std_msgs/String)
    /mavros/cmd/command - MAVROS commands (mavros_msgs/srv/CommandLong)

Configuration
HSV Color Tuning

Use the HSV configuration tool to calibrate color detection:

    Run the HSV config node: ros2 run opencv_pkg hsv_config
    Adjust trackbars in the "HSV Controls" window
    Press 's' to save configuration
    Press 'l' to load saved configuration
    Press 'q' to quit

Dropping Parameters

Edit dropping.py to adjust:

    target_distance - Distance before target to initiate drop (meters)
    Physics calculations based on altitude and airspeed

Servo Configuration

Edit servo_controller.py to adjust:

    servo_number - Servo channel number
    PWM values (1100 = closed, 1900 = open)

Troubleshooting
Camera Issues
bash

# Check available cameras
ls /dev/video*

# Test camera with OpenCV
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera failed')"

MAVROS Connection
bash

# Check MAVROS topics
ros2 topic list | grep mavros

# Monitor VFR HUD data
ros2 topic echo /mavros/vfr_hud

Color Detection Issues

    Use HSV configuration tool to tune color ranges
    Check lighting conditions
    Adjust morphological operations in program_warna.py
    Modify min_area and max_area parameters

Build Issues
bash

# Clean build
cd ~/lela
rm -rf build install log
colcon build --packages-select opencv_pkg

# Check dependencies
rosdep check opencv_pkg

Development
Adding New Colors

    Add HSV ranges in HSVConfig class
    Update create_mask() method in tarp class
    Add detection logic in color processing nodes

Modifying Drop Logic

Edit the calculate_drop_timing() method in dropping.py to implement custom physics calculations.
Testing
bash

# Run individual components
ros2 run opencv_pkg dummy_vfr  # Start dummy data
ros2 run opencv_pkg hsv_config  # Start HSV tuning
ros2 run opencv_pkg deteksi_warna  # Start detection

# Monitor topics
ros2 topic echo /color_warning
ros2 topic echo /drop_command

License

MIT License - See LICENSE file for details
Contributing

    Fork the repository
    Create a feature branch
    Make changes and test thoroughly
    Submit a pull request

Support

For issues and questions:

    Check the troubleshooting section
    Review ROS2 and OpenCV documentation
    Test components individually before running full system

