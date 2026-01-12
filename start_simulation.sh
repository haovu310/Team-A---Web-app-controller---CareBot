#!/bin/bash
# =============================================================================
# CareBot Simulation Startup - ONE COMMAND LAUNCH (TESTING)
# =============================================================================
# This script launches the complete simulation environment for testing.
# Everything runs in Gazebo with virtual hardware.
#
# Usage:
#   ./start_simulation.sh
#   ./start_simulation.sh --no-nav    # Skip navigation
#   ./start_simulation.sh --no-rviz   # Skip visualization (headless)
#
# What it launches:
#   - Gazebo Simulation (Virtual robot)
#   - Motor Controllers (Simulated)
#   - SLAM Toolbox (Mapping)
#   - Nav2 Navigation Stack
#   - Localization (EKF)
#   - Twist Multiplexer
#   - RViz Visualization
#   - Rosbridge WebSocket Server
#   - Web Server (port 8000)
#
# After startup:
#   - Web UI: http://localhost:8000
#   - Test smooth control with RAF + velocity smoothing!
# =============================================================================

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${CYAN}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║            CareBot Simulation - Testing Environment            ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Source ROS 2
echo -e "${GREEN}[1/5]${NC} Sourcing ROS2 Jazzy environment..."
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo -e "${RED}Error: ROS2 Jazzy not found!${NC}"
    exit 1
fi

# Source workspace
echo -e "${GREEN}[2/5]${NC} Sourcing workspace..."
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo -e "${YELLOW}⚠️  Workspace not built. Building now...${NC}"
    echo -e "    This may take a few minutes..."
    colcon build --symlink-install
    source install/setup.bash
fi

# Check for Gazebo
echo -e "${GREEN}[3/5]${NC} Checking Gazebo installation..."
if command -v gz &> /dev/null || command -v gazebo &> /dev/null; then
    echo -e "    ${BLUE}✓${NC} Gazebo found"
else
    echo -e "${RED}    ✗ Gazebo not found!${NC}"
    echo -e "    Install with: sudo apt install ros-jazzy-ros-gz"
    exit 1
fi

# Set use_sim_time
echo -e "${GREEN}[4/5]${NC} Configuring simulation parameters..."
# Use default DDS for simulation (FastDDS)
if [ -z "$RMW_IMPLEMENTATION" ]; then
    echo -e "    ${BLUE}✓${NC} Using default DDS (FastDDS)"
else
    echo -e "    ${BLUE}✓${NC} Using $RMW_IMPLEMENTATION"
fi
echo -e "    ${BLUE}✓${NC} use_sim_time will be set to true"

# Display configuration
echo -e "${GREEN}[5/5]${NC} Launch configuration:"
echo -e "    ${BLUE}•${NC} Simulation:     Gazebo (virtual robot)"
echo -e "    ${BLUE}•${NC} Controllers:    Mecanum drive + joint states"
echo -e "    ${BLUE}•${NC} SLAM:           Active (mapping enabled)"
echo -e "    ${BLUE}•${NC} Navigation:     Nav2 stack (delayed 10s)"
echo -e "    ${BLUE}•${NC} Localization:   EKF sensor fusion"
echo -e "    ${BLUE}•${NC} Web Server:     http://localhost:8000"
echo -e "    ${BLUE}•${NC} Rosbridge:      ws://localhost:9090"
echo ""

# Launch simulation
echo -e "${CYAN}════════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}✓ Starting simulation...${NC}"
echo ""
echo -e "${YELLOW}⏳ Note: Simulation takes ~15 seconds to fully initialize${NC}"
echo -e "   - Gazebo world: ~5 seconds"
echo -e "   - Controllers: ~3 seconds"
echo -e "   - Navigation: ~10 seconds"
echo ""
echo -e "${CYAN}Testing the NEW smooth control features:${NC}"
echo -e "  ${BLUE}→${NC} requestAnimationFrame (60fps)"
echo -e "  ${BLUE}→${NC} Velocity smoothing/ramping"
echo -e "  ${BLUE}→${NC} Keyboard debouncing"
echo ""
echo -e "${CYAN}Web Interface:${NC} http://localhost:8000"
echo -e "${CYAN}Press Ctrl+C to stop simulation${NC}"
echo ""
echo -e "${CYAN}════════════════════════════════════════════════════════════════${NC}"
echo ""

ros2 launch rmitbot_bringup simulation.launch.py

# This line is only reached if launch is stopped (Ctrl+C)
echo ""
echo -e "${CYAN}════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}Simulation shutdown complete.${NC}"
