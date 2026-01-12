#!/bin/bash
# =============================================================================
# CareBot Full Stack Startup - ALL-IN-ONE LAUNCH (Single Machine)
# =============================================================================
# This script launches EVERYTHING on a single machine for testing/development.
# Useful when running simulation or all components on a powerful laptop.
#
# Usage:
#   ./start_all.sh
#   ./start_all.sh --no-rviz     # Skip RViz (saves resources)
#
# What it launches:
#   - Robot hardware drivers (ESP32, LiDAR, etc.)
#   - SLAM Toolbox
#   - Nav2 Navigation
#   - RViz Visualization
#   - Rosbridge + Web Server
#
# After startup:
#   - Web UI: http://localhost:8000
# =============================================================================

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color

# Default parameters
SERIAL_PORT_ESP32="/dev/ttyUSB0"
SERIAL_PORT_LIDAR="/dev/ttyUSB1"
ENABLE_VISION="false"
ENABLE_RVIZ="true"

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
        --vision)
            ENABLE_VISION="true"
            shift
            ;;
        --no-rviz)
            ENABLE_RVIZ="false"
            shift
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

echo -e "${MAGENTA}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${MAGENTA}║          CareBot Full Stack - All-in-One Launch                ║${NC}"
echo -e "${MAGENTA}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Source ROS2
echo -e "${GREEN}[1/4]${NC} Sourcing ROS2 Jazzy environment..."
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo -e "${RED}Error: ROS2 Jazzy not found!${NC}"
    exit 1
fi

# Source workspace
echo -e "${GREEN}[2/4]${NC} Sourcing workspace..."
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo -e "${YELLOW}⚠️  Workspace not built. Building now...${NC}"
    colcon build --symlink-install
    source install/setup.bash
fi

# Set CycloneDDS (local machine - simpler config)
echo -e "${GREEN}[3/4]${NC} Configuring network..."
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo -e "    ${BLUE}✓${NC} Using CycloneDDS (local configuration)"

# Display configuration
echo -e "${GREEN}[4/4]${NC} Launch configuration:"
echo -e "    ${BLUE}•${NC} ESP32 Port:     $SERIAL_PORT_ESP32"
echo -e "    ${BLUE}•${NC} LiDAR Port:     $SERIAL_PORT_LIDAR"
echo -e "    ${BLUE}•${NC} Vision:         $ENABLE_VISION"
echo -e "    ${BLUE}•${NC} RViz:           $ENABLE_RVIZ"
echo -e "    ${BLUE}•${NC} Web Server:     http://localhost:8000"
echo ""

# Launch everything using the unified launch file
echo -e "${MAGENTA}════════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}✓ Starting full stack...${NC}"
echo ""
echo -e "${BLUE}Web Interface:${NC} http://localhost:8000"
echo -e "${BLUE}Press Ctrl+C to stop all nodes${NC}"
echo ""
echo -e "${MAGENTA}════════════════════════════════════════════════════════════════${NC}"
echo ""

ros2 launch rmitbot_bringup rmitbot.launch.py \
    serial_port_esp32:=$SERIAL_PORT_ESP32 \
    serial_port_lidar:=$SERIAL_PORT_LIDAR \
    enable_vision:=$ENABLE_VISION \
    enable_rviz:=$ENABLE_RVIZ

# This line is only reached if launch is stopped (Ctrl+C)
echo ""
echo -e "${MAGENTA}════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}Full stack shutdown complete.${NC}"
