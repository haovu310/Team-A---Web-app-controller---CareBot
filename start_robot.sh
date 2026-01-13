#!/bin/bash
# =============================================================================
# CareBot Robot Startup Script - ONE COMMAND LAUNCH (RASPBERRY PI)
# =============================================================================
# This script launches ALL robot hardware components in a single command.
# Run this on the Raspberry Pi.
#
# Usage:
#   ./start_robot.sh
#   ./start_robot.sh --no-vision    # Skip camera/vision
#   ./start_robot.sh --esp32=/dev/ttyUSB0 --lidar=/dev/ttyUSB1
#
# What it launches:
#   - Motor Controller (ESP32)
#   - LiDAR Sensor
#   - Localization (EKF)
#   - Twist Multiplexer
#   - Camera Stream
#   - Vision System (optional)
# =============================================================================

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default parameters
SERIAL_PORT_ESP32="/dev/ttyUSB0"
SERIAL_PORT_LIDAR="/dev/ttyUSB1"
LIDAR_BAUD="115200"
ENABLE_VISION="false"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --esp32=*)
            SERIAL_PORT_ESP32="${1#*=}"
            shift
            ;;
        --lidar=*)
            SERIAL_PORT_LIDAR="${1#*=}"
            shift
            ;;
        --lidar-baud=*)
            LIDAR_BAUD="${1#*=}"
            shift
            ;;
        --vision)
            ENABLE_VISION="true"
            shift
            ;;
        --no-vision)
            ENABLE_VISION="false"
            shift
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

echo -e "${BLUE}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║           CareBot Robot Startup - Raspberry Pi                ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Check if we're on the Pi (optional - can remove if not needed)
if [[ ! -f /proc/device-tree/model ]] || ! grep -q "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
    echo -e "${YELLOW}⚠️  Warning: This doesn't appear to be a Raspberry Pi${NC}"
    echo -e "${YELLOW}   Continuing anyway...${NC}"
    echo ""
fi

# Source ROS2
echo -e "${GREEN}[1/6]${NC} Sourcing ROS2 Jazzy environment..."
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo -e "${RED}Error: ROS2 Jazzy not found!${NC}"
    exit 1
fi

# Source workspace
echo -e "${GREEN}[2/6]${NC} Sourcing workspace..."
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo -e "${YELLOW}⚠️  Workspace not built. Building now...${NC}"
    colcon build --symlink-install
    source install/setup.bash
fi



# Kill existing ROS processes and free serial ports
echo -e "${GREEN}[4/8]${NC} Cleaning up existing processes..."
pkill -9 -f ros2 2>/dev/null || true
pkill -9 -f spawner 2>/dev/null || true
pkill -9 -f controller_manager 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f rplidar 2>/dev/null || true
pkill -9 -f rosbridge 2>/dev/null || true
pkill -9 -f slam_toolbox 2>/dev/null || true
pkill -9 -f nav2 2>/dev/null || true
pkill -9 -f bt_navigator 2>/dev/null || true
pkill -9 -f planner_server 2>/dev/null || true
pkill -9 -f controller_server 2>/dev/null || true
pkill -9 -f behavior_server 2>/dev/null || true
pkill -9 -f camera_stream 2>/dev/null || true
pkill -9 -f web_server 2>/dev/null || true
sleep 1
echo -e "    ${BLUE}✓${NC} Killed existing ROS processes"

# Free serial ports
echo -e "${GREEN}[5/8]${NC} Releasing serial ports..."
if sudo lsof /dev/ttyUSB0 2>/dev/null | grep -v COMMAND | grep -q .; then
    sudo lsof /dev/ttyUSB0 2>/dev/null | grep -v COMMAND | awk '{print $2}' | xargs -r sudo kill -9
    echo -e "    ${BLUE}✓${NC} Released /dev/ttyUSB0 (ESP32)"
else
    echo -e "    ${BLUE}✓${NC} /dev/ttyUSB0 already free"
fi

if sudo lsof /dev/ttyUSB1 2>/dev/null | grep -v COMMAND | grep -q .; then
    sudo lsof /dev/ttyUSB1 2>/dev/null | grep -v COMMAND | awk '{print $2}' | xargs -r sudo kill -9
    echo -e "    ${BLUE}✓${NC} Released /dev/ttyUSB1 (RPLidar)"
else
    echo -e "    ${BLUE}✓${NC} /dev/ttyUSB1 already free"
fi
sleep 2

# Check serial ports and set permissions
echo -e "${GREEN}[6/8]${NC} Checking serial ports..."
if [ -e "$SERIAL_PORT_ESP32" ]; then
    echo -e "    ${BLUE}✓${NC} ESP32 port found: $SERIAL_PORT_ESP32"
    sudo chmod 666 "$SERIAL_PORT_ESP32"
else
    echo -e "${YELLOW}    ⚠️  ESP32 port not found: $SERIAL_PORT_ESP32${NC}"
    echo -e "    Available ports:"
    ls -la /dev/ttyUSB* 2>/dev/null || echo "      No USB serial devices found"
fi

if [ -e "$SERIAL_PORT_LIDAR" ]; then
    echo -e "    ${BLUE}✓${NC} LiDAR port found: $SERIAL_PORT_LIDAR"
    sudo chmod 666 "$SERIAL_PORT_LIDAR"
else
    echo -e "${YELLOW}    ⚠️  LiDAR port not found: $SERIAL_PORT_LIDAR${NC}"
fi

# Display configuration
echo -e "${GREEN}[7/8]${NC} Launch configuration:"
echo -e "    ${BLUE}•${NC} ESP32 Port:     $SERIAL_PORT_ESP32"
echo -e "    ${BLUE}•${NC} LiDAR Port:     $SERIAL_PORT_LIDAR"
echo -e "    ${BLUE}•${NC} LiDAR Baud:     $LIDAR_BAUD"
echo -e "    ${BLUE}•${NC} Vision System:  $ENABLE_VISION"
echo ""

# Launch robot
echo -e "${GREEN}[8/8]${NC} Launching robot hardware..."
echo -e "${YELLOW}════════════════════════════════════════════════════════════════${NC}"
echo ""

ros2 launch rmitbot_bringup rmitbot.launch.py serial_port_lidar:=$SERIAL_PORT_LIDAR serial_baudrate:=$LIDAR_BAUD

# This line is only reached if launch is stopped (Ctrl+C)
echo ""
echo -e "${YELLOW}════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}Robot shutdown complete.${NC}"
