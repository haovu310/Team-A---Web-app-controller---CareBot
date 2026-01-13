#!/bin/bash
# =============================================================================
# CareBot PC Startup Script - ONE COMMAND LAUNCH (WORKSTATION/LAPTOP)
# =============================================================================
# This script launches ALL PC-side components in a single command.
# Run this on your PC/laptop.
#
# Usage:
#   ./start_pc.sh
#   ./start_pc.sh --no-slam       # Skip mapping
#   ./start_pc.sh --no-nav        # Skip navigation
#   ./start_pc.sh --no-rviz       # Skip visualization
#
# What it launches:
#   - SLAM Toolbox (Mapping)
#   - Nav2 Navigation Stack
#   - RViz Visualization
#   - Rosbridge WebSocket Server
#   - Web Server (port 8000)
#
# After startup:
#   - Web UI: http://localhost:8000
#   - Add ?rpi_host=<PI_IP> to connect to robot on different machine
#   - Example: http://localhost:8000?rpi_host=192.168.1.100
# =============================================================================

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Default parameters
ENABLE_SLAM="true"
ENABLE_NAV="true"
ENABLE_RVIZ="true"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-slam)
            ENABLE_SLAM="false"
            shift
            ;;
        --no-nav)
            ENABLE_NAV="false"
            shift
            ;;
        --no-rviz)
            ENABLE_RVIZ="false"
            shift
            ;;
        --slam)
            ENABLE_SLAM="true"
            shift
            ;;
        --nav)
            ENABLE_NAV="true"
            shift
            ;;
        --rviz)
            ENABLE_RVIZ="true"
            shift
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

echo -e "${CYAN}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║              CareBot PC Startup - Workstation                  ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""

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

# Set CycloneDDS configuration for PC
echo -e "${GREEN}[3/6]${NC} Configuring network (CycloneDDS)..."
if [ -f "$SCRIPT_DIR/config/cyclonedds_pc.xml" ]; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=file://$SCRIPT_DIR/config/cyclonedds_pc.xml
    echo -e "    ${BLUE}✓${NC} Using CycloneDDS config: $SCRIPT_DIR/config/cyclonedds_pc.xml"
else
    echo -e "${YELLOW}    ⚠️  CycloneDDS config not found, using default${NC}"
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
fi

# Check network connectivity to robot (optional)
echo -e "${GREEN}[4/6]${NC} Checking robot connectivity..."
if command -v ros2 &> /dev/null; then
    # Give a moment for topics to appear
    sleep 1

    # Check if we can see robot topics
    ROBOT_TOPICS=$(ros2 topic list 2>/dev/null | grep -E "odom|scan|joint_states" | wc -l)

    if [ "$ROBOT_TOPICS" -gt 0 ]; then
        echo -e "    ${BLUE}✓${NC} Robot topics detected (found $ROBOT_TOPICS topics)"
    else
        echo -e "${YELLOW}    ⚠️  No robot topics detected yet${NC}"
        echo -e "    Make sure the robot is running (start_robot.sh on Pi)"
    fi
else
    echo -e "${YELLOW}    ⚠️  Cannot check - ros2 command not available${NC}"
fi

# Display configuration
echo -e "${GREEN}[5/6]${NC} Launch configuration:"
echo -e "    ${BLUE}•${NC} SLAM Toolbox:   $ENABLE_SLAM"
echo -e "    ${BLUE}•${NC} Navigation:     $ENABLE_NAV"
echo -e "    ${BLUE}•${NC} RViz:           $ENABLE_RVIZ"
echo -e "    ${BLUE}•${NC} Web Server:     http://localhost:8000"
echo -e "    ${BLUE}•${NC} Rosbridge:      ws://localhost:9091"
echo ""

# Launch PC nodes
echo -e "${GREEN}[6/6]${NC} Launching PC nodes..."
echo -e "${CYAN}════════════════════════════════════════════════════════════════${NC}"
echo ""

# Display helpful information
echo -e "${GREEN}✓ Starting...${NC}"
echo ""
echo -e "${CYAN}Web Interface:${NC}"
echo -e "  ${BLUE}→${NC} http://localhost:8000"
echo -e "  ${BLUE}→${NC} Add ?rpi_host=<IP> if robot is on different machine"
echo ""
echo -e "${CYAN}Press Ctrl+C to stop all nodes${NC}"
echo ""
echo -e "${CYAN}════════════════════════════════════════════════════════════════${NC}"
echo ""

ros2 launch rmitbot_bringup pc.launch.py \
    enable_slam:=$ENABLE_SLAM \
    enable_nav:=$ENABLE_NAV \
    enable_rviz:=$ENABLE_RVIZ

# This line is only reached if launch is stopped (Ctrl+C)
echo ""
echo -e "${CYAN}════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}PC nodes shutdown complete.${NC}"
