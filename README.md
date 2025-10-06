# LELA - Autonomous Payload Dropping System


## Spesifikasi

- **Platform**: Raspberry Pi 5 (Ubuntu Server 24.04)
- **ROS2**: Jazzy Jalisco
- **Flight Controller**: Matek H743 (ArduPilot/PX4)
- **Camera**: Raspberry Pi Camera (720p)
- **Payload**: 1.5 kg
- **Altitude**: 30 meters (Approx)
- **Mission**: 12 hotspot waypoints

## Misi

1. **Waypoint 1 (HS1)**: Deteksi warna + Validasi + Kirim message + DROP payload
2. **Waypoint 2-12 (HS2-HS12)**: Deteksi warna + Validasi + Kirim message (NO DROP)
3. **RTL**: Return to launch

### Deteksi Warna
- **Merah/Orange**: Titik api → Message: `"Titik Api - {Warna}"`
- **Biru**: Bukan titik api → Message: `"Bukan Titik Api - Biru"`

---

## Setup di Raspberry Pi 5

### 1. Install ROS2 Jazzy

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS2 Jazzy
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-ros-base
```

### 2. Install Dependencies

```bash
sudo apt install -y \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-opencv \
    python3-opencv \
    python3-numpy \
    python3-pip \
    git

# Install GeographicLib datasets for MAVROS
cd ~
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```

### 3. Clone Repository

```bash
cd ~
git clone https://github.com/Sakazu01/lela.git
cd lela
```

### 4. Build Package

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select real --symlink-install
source install/setup.bash
```

### 5. Setup Auto-Source (Optional)

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/lela/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Konfigurasi Hardware

### 1. Connect Matek H743

**Via USB:**
```bash
# Check port
ls /dev/ttyACM*

# Launch MAVROS
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200
```

**Via UART:**
```bash
# For serial connection
ros2 launch mavros apm.launch fcu_url:=/dev/ttyAMA0:921600
```

### 2. Camera Setup

**Pi Camera via CSI:**
```bash
# Enable camera
sudo raspi-config
# Interface Options → Camera → Enable

# Test camera
libcamera-hello
```

**Camera index di params.yaml:**
```yaml
camera_index: 0  # Default untuk Pi Camera
```

### 3. Servo Channel Configuration

Edit `src/real/real/config/params.yaml`:
```yaml
servo_channel: 9      # Sesuaikan dengan output channel di Matek
pwm_drop: 1900        # PWM untuk posisi drop
pwm_hold: 1100        # PWM untuk posisi hold
```

---

## Running the System

### 1. Launch MAVROS (Terminal 1)

```bash
source ~/lela/install/setup.bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200
```

### 2. Launch LELA System (Terminal 2)

```bash
source ~/lela/install/setup.bash
ros2 launch real production.launch.py
```

### 3. Monitor System (Terminal 3)

```bash
# Monitor detection messages
ros2 topic echo /gcs/detection

# Monitor system state
ros2 topic echo /system/state

# Monitor drop info
ros2 topic echo /drop/info
```

---

## Parameter Tuning

Edit `src/real/real/config/params.yaml`:

### Detection Parameters
```yaml
# HSV Color Ranges (tune based on lighting condition)
red_h_min: 0
red_h_max: 10
red_s_min: 100
red_v_min: 100

# Detection threshold
min_contour_area: 5000        # Adjust based on altitude test
validation_frames: 5           # Multi-frame validation count
```

### Drop Parameters
```yaml
fixed_altitude: 30.0          # Competition altitude
payload_mass: 1.5             # kg
target_distance: 2.5          # meters before center
```

### Camera Parameters
```yaml
camera_index: 0
frame_rate: 10                # Hz (10 fps recommended for 720p)
```

---

## Testing Procedures

### 1. Ground Test - Color Detection

```bash
# Enable visualization for tuning
ros2 launch real production.launch.py

# In another terminal, enable camera
ros2 topic pub --once /camera/enable std_msgs/Bool "data: true"

# Place colored tarp in front of camera
# Red/Orange → should detect as "Titik Api"
# Blue → should detect as "Bukan Titik Api"
```

### 2. Servo Test

```bash
# Test servo without flight
ros2 topic pub --once /drop/execute std_msgs/Bool "data: true"

# Check servo moves to drop position then returns to hold
```

### 3. Flight Test at 30m

1. Manual flight to 30m altitude
2. System should activate camera at waypoint
3. Detect tarp color (5x5m tarp on ground)
4. Validate detection (5 consecutive frames)
5. Send GCS message
6. If WP1: Execute drop

---

## Troubleshooting

### Camera tidak terdeteksi
```bash
# Check camera
vcgencmd get_camera
v4l2-ctl --list-devices

# Set camera permissions
sudo usermod -a -G video $USER
```

### MAVROS connection failed
```bash
# Check port
ls -l /dev/ttyACM*

# Check permissions
sudo chmod 666 /dev/ttyACM0

# Verify Matek baudrate (QGroundControl/Mission Planner)
```

### Detection tidak akurat
```bash
# Test dari ketinggian sebenarnya (30m)
# Adjust min_contour_area berdasarkan hasil test
# Tune HSV range dengan enable_visualization: true
```

### Drop tidak akurat
```bash
# Log drop data: altitude, speed, calculated distance
# Adjust target_distance parameter based on test results
# Consider wind conditions
```

---

## System Architecture

```
Mission Monitor → Waypoint Reached
       ↓
State Manager → Enable Camera
       ↓
Camera Controller → Image Stream
       ↓
Color Detector → Multi-frame Validation
       ↓
Message Publisher → GCS Message
       ↓
[If WP1] Drop Calculator → Physics Calculation
       ↓
Servo Controller → Execute Drop
```

---

## Topics

### Subscribed
- `/mavros/mission/reached` - Waypoint reached event
- `/mavros/vfr_hud` - Altitude, speed data
- `/camera/image_raw` - Camera feed

### Published
- `/gcs/detection` - Detection message to GCS
- `/system/state` - Current system state
- `/drop/execute` - Drop command
- `/drop/completed` - Drop completion status

---

## Safety Notes

1. Always test servo mechanism on ground first
2. Verify altitude before enabling drop
3. Check wind conditions (affects drop accuracy)
4. Monitor battery level
5. Have manual override ready
6. Test detection at actual altitude (30m) before competition

---

## License

MIT License - Competition Version 2.0.0

---

## Support

For issues or questions:
- GitHub: https://github.com/Sakazu01/lela
- Check logs: `~/lela/log/latest_build/`
