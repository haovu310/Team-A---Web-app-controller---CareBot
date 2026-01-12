# CareBot - ROS 2 Workspace

**Complete ROS 2 workspace for the CareBot Autonomous Mobile Robot with Mecanum Drive.**

This workspace provides everything from low-level motor control to high-level SLAM and autonomous navigation using the Nav2 stack.

---

## 🏗️ Architecture & Features

- **Hardware Control:** ESP32-based motor controller with PID feedback control
- **Mapping (SLAM):** Real-time 2D mapping using **SLAM Toolbox**
- **Localization:** Sensor fusion via **Extended Kalman Filter (EKF)** (`robot_localization`)
- **Navigation:** Path planning and obstacle avoidance using **Nav2**
- **Web Interface:** Real-time browser-based teleoperation and monitoring
- **Hybrid Deployment:** Seamless integration across Raspberry Pi + PC via **CycloneDDS**

---

## 🚀 Quick Start (Split Deployment - Recommended)

### On Raspberry Pi (Robot Hardware):
```bash
./start_robot.sh
```

### On Your PC/Laptop (Compute Stack):
```bash
./start_pc.sh
```

### Access Web Interface:
Open browser to: **http://localhost:8000**

That's it! No more opening 9 terminals! 🎉

---

## 📋 Prerequisites

### System Requirements
- **OS:** Ubuntu 24.04 (or ROS 2 Jazzy compatible)
- **ROS 2:** Jazzy Jalisco
- **Hardware:**
  - Raspberry Pi 5 (4GB RAM)
  - ESP32 DevKit
  - RPLidar A1/A2
  - USB Camera (optional)

### Software Dependencies

```bash
# Update package list
sudo apt update

# Install ROS 2 Jazzy packages
sudo apt install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-rosbridge-suite \
    ros-jazzy-rplidar-ros

# Install system tools
sudo apt install -y git python3-pip python3-colcon-common-extensions
```

---

## 🛠️ Installation & Setup

### 1. Clone the Repository
```bash
cd ~/Documents
git clone <your-repository-url> team_A_web_app
cd team_A_web_app
```

### 2. Build the Workspace
```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### 3. Configure Network (For Split Deployment)

#### On Raspberry Pi:
1. Find your network interface:
   ```bash
   ip a
   ```

2. Edit `config/cyclonedds_pi.xml`:
   ```xml
   <NetworkInterface name="wlan0" />  <!-- or eth0 for ethernet -->
   <Peer address="YOUR_PC_IP"/>  <!-- e.g., 192.168.1.20 -->
   ```

#### On PC/Laptop:
1. Find your network interface:
   ```bash
   ip a
   ```

2. Edit `config/cyclonedds_pc.xml`:
   ```xml
   <NetworkInterface name="wlan0" />  <!-- or eth0 for ethernet -->
   <Peer address="YOUR_PI_IP"/>  <!-- e.g., 192.168.1.100 -->
   ```

**⚠️ Important:** Both machines must be on the same network!

---

## 🕹️ Usage Modes

### Mode 1: Split Deployment (Production)
**Best for:** Real robot operation with optimal performance

**On Raspberry Pi:**
```bash
./start_robot.sh
```
Launches: Motors, LiDAR, Localization, Camera

**On PC/Laptop:**
```bash
./start_pc.sh
```
Launches: SLAM, Nav2, RViz, Web Server, Rosbridge

**Access Web UI:** `http://localhost:8000?rpi_host=<PI_IP>`

---

### Mode 2: All-in-One (Development/Testing)
**Best for:** Testing on a single powerful machine

```bash
./start_all.sh
```
Launches: Everything on one machine

**Access Web UI:** `http://localhost:8000`

---

### Mode 3: Custom Configuration

**Robot with custom serial ports:**
```bash
./start_robot.sh --esp32=/dev/ttyUSB0 --lidar=/dev/ttyUSB1
```

**Robot with vision system:**
```bash
./start_robot.sh --vision
```

**PC without RViz (headless):**
```bash
./start_pc.sh --no-rviz
```

**PC with only mapping (no navigation):**
```bash
./start_pc.sh --no-nav
```

---

## 📦 Package Overview

### Core Packages

| Package | Description |
|---------|-------------|
| `rmitbot_bringup` | Launch files for complete system startup |
| `rmitbot_controller` | Motor control via ros2_control + ESP32 interface |
| `rmitbot_description` | URDF robot model and visualization |
| `rmitbot_firmware` | ESP32 Arduino firmware (PID control) |
| `rmitbot_localization` | EKF sensor fusion for odometry |
| `rmitbot_mapping` | SLAM Toolbox configuration |
| `rmitbot_navigation` | Nav2 configuration and twist multiplexer |
| `rmitbot_vision` | Camera and AprilTag detection |
| `rmitbot_web_controller` | Web interface and rosbridge server |

---

## 🎮 Web Interface Features

- **Virtual D-Pad:** Touch/click controls for omnidirectional movement
- **Keyboard Controls:** WASD for movement, Q/E for strafing
- **Speed Control:** Real-time linear and angular speed adjustment
- **Live Camera Feed:** Real-time video streaming
- **Map Visualization:** Interactive SLAM map with robot position
- **Mode Switching:** IDLE / MANUAL / AUTO modes
- **Navigation:** Click-to-navigate autonomous path planning

**Access:** http://localhost:8000

---

## 🔧 Troubleshooting

### Serial Ports Not Found
```bash
# List available ports
ls -la /dev/ttyUSB*

# Give permissions (if needed)
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect

# Specify correct ports
./start_robot.sh --esp32=/dev/ttyUSB0 --lidar=/dev/ttyUSB1
```

### Can't See Robot Topics on PC
1. **Check network configuration:**
   ```bash
   # On both machines, verify same network
   ip a
   ping <OTHER_MACHINE_IP>
   ```

2. **Verify CycloneDDS config:**
   ```bash
   echo $CYCLONEDDS_URI
   cat $CYCLONEDDS_URI  # Should show correct config
   ```

3. **Check RMW implementation:**
   ```bash
   echo $RMW_IMPLEMENTATION  # Should be: rmw_cyclonedds_cpp
   ```

4. **Test topic visibility:**
   ```bash
   ros2 topic list
   ros2 topic echo /scan  # Should see LiDAR data
   ```

### Web Interface Not Loading
1. **Check if rosbridge is running:**
   ```bash
   ros2 node list | grep rosbridge
   ```

2. **Check web server:**
   ```bash
   curl http://localhost:8000
   ```

3. **For split deployment, add Pi IP:**
   ```
   http://localhost:8000?rpi_host=192.168.1.100
   ```

### ESP32 Firmware Upload Failed
```bash
# Install PlatformIO
pip3 install platformio

# Navigate to firmware directory
cd src/rmitbot_firmware/arduino_firmware/rmitbot_arduino_firmware

# Upload firmware
pio run --target upload

# Monitor serial output
pio device monitor
```

---

## 📊 Performance Metrics

| Component | Frequency | Notes |
|-----------|-----------|-------|
| ESP32 PID Control | 100 Hz (10ms) | Closed-loop motor control |
| ESP32 Telemetry | 100 Hz (10ms) | Velocity feedback |
| Frontend Publish Rate | ~60 Hz (16ms) | Smooth command streaming |
| LiDAR Scan Rate | 5-10 Hz | RPLidar A1/A2 |
| Camera Stream | 15 FPS | Configurable |

---

## 🌐 Network Architecture (Split Deployment)

```
┌─────────────────────┐         ┌──────────────────────┐
│   Raspberry Pi      │         │    PC/Laptop         │
│                     │         │                      │
│  • Motor Control    │◄──DDS──►│  • SLAM Toolbox      │
│  • LiDAR Driver     │         │  • Nav2 Stack        │
│  • Localization     │         │  • RViz              │
│  • Camera Stream    │         │  • Web Server        │
│  • Twist Mux        │         │  • Rosbridge         │
└─────────────────────┘         └──────────────────────┘
         ▲                               │
         │                               │
         └───────────ESP32───────────────┘
         (USB Serial: /dev/ttyUSB0)

User Browser ──► http://localhost:8000 ──► Web Server (PC)
                                            │
                                            └──► Rosbridge ──► ROS 2 Topics
```

---

## 🔄 Development Workflow

### Making Changes to Code

```bash
# After modifying source files:
colcon build --packages-select <package_name>
source install/setup.bash

# For web interface changes:
colcon build --packages-select rmitbot_web_controller
./start_pc.sh  # Restart to see changes
```

### Updating ESP32 Firmware

```bash
cd src/rmitbot_firmware/arduino_firmware/rmitbot_arduino_firmware
pio run --target upload
```

---

## 📝 Manual Launch (Advanced Users)

If you prefer manual control or need to debug individual components:

### On Raspberry Pi:
```bash
# Terminal 1: Motor Controller
ros2 launch rmitbot_controller controller.launch.py

# Terminal 2: LiDAR
ros2 launch rmitbot_mapping rplidar.launch.py

# Terminal 3: Localization
ros2 launch rmitbot_localization localization.launch.py

# Terminal 4: Twist Mux
ros2 launch rmitbot_navigation twistmux.launch.py
```

### On PC:
```bash
# Terminal 1: SLAM
ros2 launch rmitbot_mapping mapping.launch.py

# Terminal 2: Navigation
ros2 launch rmitbot_navigation nav.launch.py

# Terminal 3: Web Interface
ros2 launch rmitbot_web_controller web_pc.launch.py
```

---

## 🎯 Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | Twist | Final velocity commands to robot |
| `/cmd_vel_keyboard` | TwistStamped | Manual control from web UI |
| `/scan` | LaserScan | LiDAR measurements |
| `/odom` | Odometry | Wheel odometry |
| `/map` | OccupancyGrid | SLAM-generated map |
| `/camera/image_raw` | Image | Camera feed |
| `/joint_states` | JointState | Motor positions |

---

## 📚 Additional Resources

- **ROS 2 Jazzy Docs:** https://docs.ros.org/en/jazzy/
- **Nav2 Documentation:** https://navigation.ros.org/
- **SLAM Toolbox:** https://github.com/SteveMacenski/slam_toolbox
- **ros2_control:** https://control.ros.org/

---

## 🆘 Getting Help

1. **Check logs:**
   ```bash
   ros2 topic echo /rosout
   ```

2. **View active nodes:**
   ```bash
   ros2 node list
   ```

3. **Check node info:**
   ```bash
   ros2 node info /controller_manager
   ```

4. **Monitor topics:**
   ```bash
   ros2 topic list
   ros2 topic hz /scan
   ```

---

**Enjoy building with CareBot! 🤖✨**

For issues or contributions, please open an issue on the repository.
