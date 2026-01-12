/re# CareBot - ROS 2 Workspace

**Complete ROS 2 workspace for the CareBot Autonomous Mobile Robot with Mecanum Drive.**

This workspace provides everything from low-level motor control to high-level SLAM and autonomous navigation using the Nav2 stack.

---

## 🏗️ Architecture & Features

- **Hardware Control:** ESP32-based motor controller with PID feedback control (100Hz)
- **Sensor Fusion:** IMU (MPU6050) + Wheel Encoders fused via EKF at 20Hz
- **Mapping (SLAM):** Real-time 2D mapping using **SLAM Toolbox**
- **Localization:** Extended Kalman Filter (EKF) with odometry + IMU fusion
- **Navigation:** Path planning and obstacle avoidance using **Nav2**
- **Web Interface:** Real-time browser-based teleoperation and monitoring
- **Hybrid Deployment:** Raspberry Pi 5 (hardware) + PC (compute) via ROS 2 DDS

### System Architecture

**Critical Components Distribution:**

| Component | Location | Why | Configurable? |
|-----------|----------|-----|---------------|
| **Motor Controller** | Pi | Direct USB serial to ESP32 | No - must be on Pi |
| **RPLidar Driver** | Pi | Direct USB to LiDAR sensor | No - must be on Pi |
| **EKF Localization** | Pi | Low-latency sensor fusion | Recommended on Pi |
| **Twist Mux** | Pi | Safety-critical control arbitration | Recommended on Pi |
| **Camera Stream** | Pi | Camera hardware connected here | No - must be on Pi |
| **Web Server + Rosbridge** | Pi or PC | Serves web UI | Yes - either works |
| **SLAM Toolbox** | PC | CPU-intensive mapping | Recommended on PC |
| **Nav2 Stack** | PC | CPU-intensive path planning | Recommended on PC |
| **RViz** | PC | 3D visualization (requires display) | PC only |

**Current Restructure Branch Configuration:**
- Pi runs: Hardware interfaces + Control + Web services (all-in-one on robot)
- PC runs: SLAM + Navigation (currently commented out - enable for autonomous features)

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

### 3. Configure ROS 2 Domain ID (Critical!)

**⚠️ IMPORTANT:** Both Raspberry Pi and PC must use the same ROS_DOMAIN_ID for communication.

```bash
# On BOTH machines (Pi and PC):
export ROS_DOMAIN_ID=42
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
source ~/.bashrc

# Verify it's set:
echo $ROS_DOMAIN_ID
# Should output: 42
```

### 4. Configure Network (For Split Deployment)

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

## ⚠️ Pre-Deployment Checklist (CRITICAL!)

**Before connecting to real hardware, verify these critical configurations:**

### 1. Hardware Safety Checks

```bash
# A. Verify ESP32 is connected
ls -l /dev/ttyUSB*
# Should show: /dev/ttyUSB0 (or similar)

# B. Set USB permissions
sudo chmod 666 /dev/ttyUSB0

# C. Verify serial communication (optional test)
sudo apt install screen
screen /dev/ttyUSB0 115200
# Should see streaming data: <vel1 vel2 vel3 vel4 quat... >
# Press Ctrl+A then K to exit
```

### 2. Configuration Verification

**Critical:** These parameters MUST be set correctly or robot won't work!

| Parameter | File | Line | Required Value | Why |
|-----------|------|------|----------------|-----|
| `enable_odom_tf` | `rmitbot_controller.yaml` | 28 | `true` | **CRITICAL:** Publishes odom→base_footprint transform |
| `reference_timeout` | `rmitbot_controller.yaml` | 17 | `0.3` | Safety: stops motors if connection lost |
| `use_sim_time` | Launch files override | N/A | `false` | Real robot uses wall clock, not sim time |
| `ROS_DOMAIN_ID` | Environment | N/A | `42` | Both Pi & PC must match |

**Verify with:**
```bash
# Check odom TF enabled
grep "enable_odom_tf" src/rmitbot_controller/config/rmitbot_controller.yaml
# Should show: enable_odom_tf: true

# Check timeout is safe
grep "reference_timeout" src/rmitbot_controller/config/rmitbot_controller.yaml
# Should show: reference_timeout: 0.3

# Check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID
# Should show: 42
```

### 3. First Power-On Procedure

**⚠️ SAFETY FIRST:** Have physical access to power switch before testing!

**Step 1: Verify Systems (Without Motors Moving)**
```bash
# On Raspberry Pi
ros2 launch rmitbot_bringup rmitbot.launch.py

# In another terminal, check topics
ros2 topic list | grep -E "odom|cmd_vel|scan|imu"
# Should see:
#   /cmd_vel
#   /cmd_vel_keyboard
#   /odom
#   /scan
#   /imu (if IMU connected)
```

**Step 2: Verify TF Tree**
```bash
# Install tf2_tools if not installed
sudo apt install ros-jazzy-tf2-tools

# Check TF tree
ros2 run tf2_ros tf2_echo odom base_footprint
# Should show: Transform from odom to base_footprint
# If it says "Timeout" - STOP! enable_odom_tf is still false!

# Generate full TF tree visualization
ros2 run tf2_tools view_frames
# Open frames.pdf - should show complete tree:
# map → odom → base_footprint → base_link → [wheel_joints, imu_link, laser_frame]
```

**Step 3: Test Manual Control (SUPERVISED!)**
```bash
# Open web UI
firefox http://<Pi_IP>:8000

# 1. Check status indicator - should be "Online" (green dot)
# 2. Switch to MANUAL mode
# 3. Tap W key BRIEFLY - robot should move forward slightly
# 4. Press SPACE immediately - robot should stop
# 5. If robot doesn't respond or moves erratically - KILL POWER!
```

### 4. Known Hardware Limitations

| Limitation | Impact | Mitigation |
|------------|--------|------------|
| **No physical E-stop** | Can't immediately cut power | Keep hand on power switch during testing |
| **IMU requires calibration** | Yaw drift over time | Run magnetometer calibration routine |
| **Mecanum wheel slip** | Odometry errors on carpet | Test on smooth, hard floors first |
| **WiFi latency** | 20-50ms control delay | Acceptable for manual control, not critical tasks |
| **No battery monitoring** | Robot may behave erratically when low | Monitor ESP32 terminal for voltage warnings |

### 5. Post-Deployment Validation Tests

After successful first power-on, run these validation tests:

```bash
# Test 1: Odometry Accuracy
# Command robot to drive forward 1 meter
# Measure actual distance traveled
# Should be within ±10% (0.9m - 1.1m)

# Test 2: IMU Sanity Check
ros2 topic echo /imu --once
# Quaternion should be approximately [0, 0, 0, 1] when robot is level

# Test 3: Wheel Encoder Verification
ros2 topic echo /joint_states
# Positions should increase when you manually spin wheels

# Test 4: Control Latency
# Press W key in web UI, measure time until robot moves
# Should be < 200ms

# Test 5: Emergency Stop Response
# Drive robot forward, press SPACE
# Robot should stop within 300ms (reference_timeout)
```

### 6. Common First-Time Issues

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| Robot doesn't move | ESP32 not connected | Check `/dev/ttyUSB0` exists |
| TF timeout errors | `enable_odom_tf: false` | Set to `true` and rebuild |
| Web UI won't connect | Rosbridge not running | Check port 9090 is open |
| Robot moves erratically | Low battery voltage | Charge/replace battery |
| All zeros in odometry | Serial parsing error | Check ESP32 firmware is uploaded |
| IMU data frozen | MPU6050 not initialized | Check I2C connections, power cycle |

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
| **ESP32 PID Control** | 100 Hz (10ms) | Closed-loop motor control |
| **ESP32 Telemetry** | 100 Hz (10ms) | 14-value serial protocol: 4 wheels + 10 IMU |
| **Controller Manager** | 100 Hz (10ms) | ros2_control update rate |
| **EKF Localization** | 20 Hz (50ms) | Reduced from 30Hz for Pi 5 performance |
| **Frontend Publish Rate** | ~60 Hz (16ms) | Smooth command streaming with RAF batching |
| **LiDAR Scan Rate** | 5-10 Hz | RPLidar A1/A2 |
| **Camera Stream** | 15 FPS | Limited for Pi 5 CPU efficiency |
| **Rosbridge Queue** | 10 messages | Optimized for low memory usage |
| **Control Timeout** | 300ms | Safety: stops motors if no commands received |

**Serial Communication Protocol (ESP32 ↔ Pi):**
```
TX (ESP32 → Pi): <w1\tw2\tw3\tw4\tq0\tq1\tq2\tq3\tgx\tgy\tgz\tax\tay\taz>
RX (Pi → ESP32): <w1_ref\tw2_ref\tw3_ref\tw4_ref>
Baud Rate: 115200
Format: Tab-separated values wrapped in < >
```

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
| `/cmd_vel` | Twist | Final velocity commands to robot (from twist_mux) |
| `/cmd_vel_keyboard` | TwistStamped | Manual control from web UI (priority: 20) |
| `/cmd_vel_navigation` | TwistStamped | Autonomous navigation commands (priority: 5) |
| `/scan` | LaserScan | LiDAR measurements (RPLidar) |
| `/odom` | Odometry | Wheel odometry from mecanum controller |
| `/imu` | Imu | IMU data (MPU6050): orientation, gyro, accel |
| `/map` | OccupancyGrid | SLAM-generated map |
| `/camera/image_raw/compressed` | CompressedImage | Camera feed (MJPEG) |
| `/joint_states` | JointState | Motor positions and velocities |

---

## 🌳 Transform (TF) Tree

**Critical:** The TF tree must be complete for the robot to function!

```
map                          (Published by: SLAM Toolbox)
  └─ odom                    (Published by: EKF)
      └─ base_footprint      (Published by: mecanum_drive_controller)
          └─ base_link       (Published by: robot_state_publisher - static)
              ├─ imu_link    (Published by: robot_state_publisher - static)
              ├─ laser_frame (Published by: robot_state_publisher - static)
              ├─ camera_link (Published by: robot_state_publisher - static)
              └─ [4 wheel joints] (Published by: robot_state_publisher - dynamic)
```

**Who Publishes What:**
- `map → odom`: EKF filter node (based on SLAM corrections)
- `odom → base_footprint`: **mecanum_drive_controller** (requires `enable_odom_tf: true`)
- `base_footprint → base_link`: robot_state_publisher (URDF static transform)
- All sensor links: robot_state_publisher (URDF static transforms)

**Verification:**
```bash
# Check critical transform
ros2 run tf2_ros tf2_echo odom base_footprint
# Should show transform, NOT timeout!

# Visualize full tree
ros2 run tf2_tools view_frames
# Opens frames.pdf with complete tree diagram
```

---

## 📝 Recent Updates & Fixes (Restructure Branch)

### ✅ Critical Fixes Applied (2026-01-12)

1. **Fixed TF Tree Publishing** (`enable_odom_tf: true`)
   - **Issue:** Robot TF tree was broken - odom→base_footprint transform missing
   - **Impact:** SLAM, navigation, and sensor fusion would fail
   - **Fix:** Set `enable_odom_tf: true` in `rmitbot_controller.yaml`
   - **Status:** ✅ FIXED

2. **Improved Safety Timeout** (`reference_timeout: 0.3`)
   - **Issue:** 1-second timeout too long for safety
   - **Impact:** Robot would continue moving for 1s if connection lost
   - **Fix:** Reduced to 300ms for faster emergency stop
   - **Status:** ✅ FIXED

3. **Fixed Web Server Regression** (directory parameter)
   - **Issue:** Using `os.chdir()` changed process working directory
   - **Impact:** Could break ROS operations expecting workspace root
   - **Fix:** Use `directory=` parameter instead
   - **Status:** ✅ FIXED

4. **Disabled Crashing teleop_twist_keyboard**
   - **Issue:** Node crashed due to no terminal stdin
   - **Impact:** Error logs on every launch
   - **Fix:** Disabled node (web UI provides keyboard control)
   - **Status:** ✅ FIXED

5. **Reduced EKF Frequency** (30Hz → 20Hz)
   - **Issue:** Pi 5 couldn't meet 30Hz update rate with IMU
   - **Impact:** "Failed to meet update rate" errors
   - **Fix:** Reduced to 20Hz for stable performance
   - **Status:** ✅ FIXED

### 🆕 Improvements in Restructure Branch

- ✅ **Configurable Serial Port:** Can specify ESP32 port via launch argument
- ✅ **IMU Support:** Full MPU6050 integration with quaternion, gyro, accel
- ✅ **Rosbridge Optimizations:** Reduced queue size, compression enabled
- ✅ **Web UI Enhancements:** Connection retry logic, custom dialog system
- ✅ **Performance Tuning:** Optimized for Raspberry Pi 5 limitations

### ⚠️ Known Differences from Archive Branch

| Aspect | Archive | Restructure | Notes |
|--------|---------|-------------|-------|
| **SLAM/Nav2** | Enabled on PC | Commented out | Enable lines 84-97 in rmitbot.launch.py for autonomous features |
| **IMU** | Not connected | Connected & configured | Better localization accuracy |
| **Teleop keyboard** | Crashes with xterm | Disabled | Web UI provides same functionality |
| **Web server** | Uses directory param | Fixed to use directory param | Was temporarily regressed, now fixed |
| **EKF rate** | 30 Hz | 20 Hz | Better stability on Pi 5 |

---

## 📚 Additional Resources

- **ROS 2 Jazzy Docs:** https://docs.ros.org/en/jazzy/
- **Nav2 Documentation:** https://navigation.ros.org/
- **SLAM Toolbox:** https://github.com/SteveMacenski/slam_toolbox
- **ros2_control:** https://control.ros.org/
- **Mecanum Drive Controller:** https://github.com/ros-controls/ros2_controllers/tree/master/mecanum_drive_controller

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
