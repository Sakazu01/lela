# REAL - Autonomous Payload Dropping System

Sistem autonomous untuk dropping payload berbasis waypoint dan deteksi warna.

## Build
```bash
cd ~/lela_ws
colcon build --packages-select real
source install/setup.bash

##Structure
lela_ws/src/real/
├── launch/
│   ├── simulation.launch.py   ← Untuk testing tanpa hardware
│   ├── hardware.launch.py     ← Untuk hardware real (BARU)
│   └── full_system.launch.py  ← Original (deprecated)
├── real/nodes/
│   ├── state_manager.py
│   ├── mission_monitor.py
│   ├── camera_controller.py
│   ├── color_detector.py
│   ├── drop_calculator.py
│   ├── servo_controller.py
│   ├── dummy_waypoint.py      ← Hanya untuk simulasi
│   ├── dummy_vfr.py           ← Hanya untuk simulasi
│   └── dummy_camera.py        ← Hanya untuk simulasi
