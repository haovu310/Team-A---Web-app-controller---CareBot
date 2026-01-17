# CareBot Terminal Commands

> **Note:** Each terminal is self-contained, building only its required package and dependencies using `--packages-up-to`.
> **PC VS ROBOT:** Terminals 1-6 are for the **Robot/RPI**. Terminals 7-9 are for the **PC/Workstation**.

---

# === ROBOT (Raspberry Pi) ===

## Terminal 1: Motor Controller
```bash
echo "=== Sourcing ROS2 Jazzy environment ==="
source /opt/ros/jazzy/setup.bash

echo "=== Cleaning previous build artifacts ==="
rm -rf build install log

echo "=== Building rmitbot_controller, rmitbot_description and dependencies ==="
colcon build --packages-select rmitbot_description --packages-up-to rmitbot_controller

echo "=== Sourcing workspace setup ==="
source install/setup.bash

echo "=== Launching Motor Controller ==="
ros2 launch rmitbot_controller controller.launch.py
```

---

## Terminal 2: Web Controller (RPI)
```bash
echo "=== Sourcing ROS2 Jazzy environment ==="
source /opt/ros/jazzy/setup.bash

echo "=== Cleaning previous build artifacts ==="
rm -rf build install log

echo "=== Building rmitbot_web_controller and dependencies ==="
colcon build --packages-up-to rmitbot_web_controller

echo "=== Sourcing workspace setup ==="
source install/setup.bash

echo "=== Launching Web Controller (RPI) ==="
ros2 launch rmitbot_web_controller web_rpi.launch.py
```

---

## Terminal 3: Twist Mux (Navigation)
```bash
echo "=== Sourcing ROS2 Jazzy environment ==="
source /opt/ros/jazzy/setup.bash

echo "=== Cleaning previous build artifacts ==="
rm -rf build install log

echo "=== Building rmitbot_navigation and dependencies ==="
colcon build --packages-up-to rmitbot_navigation

echo "=== Sourcing workspace setup ==="
source install/setup.bash

echo "=== Launching Twist Multiplexer ==="
ros2 launch rmitbot_navigation twistmux.launch.py
```

---

## Terminal 4: LiDAR Sensor
```bash
echo "=== Sourcing ROS2 Jazzy environment ==="
source /opt/ros/jazzy/setup.bash

echo "=== Cleaning previous build artifacts ==="
rm -rf build install log

echo "=== Building rmitbot_mapping and dependencies ==="
colcon build --packages-up-to rmitbot_mapping

echo "=== Sourcing workspace setup ==="
source install/setup.bash

echo "=== Launching RPLidar Sensor ==="
ros2 launch rmitbot_mapping rplidar.launch.py
```

---

## Terminal 5: Localization (EKF)
```bash
echo "=== Sourcing ROS2 Jazzy environment ==="
source /opt/ros/jazzy/setup.bash

echo "=== Cleaning previous build artifacts ==="
rm -rf build install log

echo "=== Building rmitbot_localization and dependencies ==="
colcon build --packages-up-to rmitbot_localization

echo "=== Sourcing workspace setup ==="
source install/setup.bash

echo "=== Launching Localization (Extended Kalman Filter) ==="
ros2 launch rmitbot_localization localization.launch.py
```

---

## Terminal 6: Vision
```bash
echo "=== Sourcing ROS2 Jazzy environment ==="
source /opt/ros/jazzy/setup.bash

echo "=== Cleaning previous build artifacts ==="
rm -rf build install log

echo "=== Building rmitbot_vision and dependencies ==="
colcon build --packages-up-to rmitbot_vision

echo "=== Sourcing workspace setup ==="
source install/setup.bash

echo "=== Launching Vision System ==="
ros2 launch rmitbot_vision vision.launch.py
```

---
---

# === PC / WORKSTATION ===

## Terminal 7: Web Controller (PC)
```bash
echo "=== Sourcing ROS2 Jazzy environment ==="
source /opt/ros/jazzy/setup.bash

echo "=== Cleaning previous build artifacts ==="
rm -rf build install log

echo "=== Building rmitbot_web_controller and dependencies ==="
colcon build --packages-up-to rmitbot_web_controller

echo "=== Sourcing workspace setup ==="
source install/setup.bash

echo "=== Launching Web Controller (PC) ==="
ros2 launch rmitbot_web_controller web_pc.launch.py
```

---

## Terminal 8: Mapping (SLAM Toolbox)
```bash
echo "=== Sourcing ROS2 Jazzy environment ==="
source /opt/ros/jazzy/setup.bash

echo "=== Cleaning previous build artifacts ==="
rm -rf build install log

echo "=== Building rmitbot_mapping and dependencies ==="
colcon build --packages-up-to rmitbot_mapping

echo "=== Sourcing workspace setup ==="
source install/setup.bash

echo "=== Launching SLAM (Mapping) ==="
ros2 launch rmitbot_mapping mapping.launch.py
```

---

## Terminal 9: Navigation (Nav2)
```bash
echo "=== Sourcing ROS2 Jazzy environment ==="
source /opt/ros/jazzy/setup.bash

echo "=== Cleaning previous build artifacts ==="
rm -rf build install log

echo "=== Building rmitbot_navigation and dependencies ==="
colcon build --packages-up-to rmitbot_navigation

echo "=== Sourcing workspace setup ==="
source install/setup.bash

echo "=== Launching Nav2 (Navigation) ==="
ros2 launch rmitbot_navigation nav.launch.py
```
