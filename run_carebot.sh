#!/bin/bash

# CareBot Launch Helper
# Run from: ~/Documents/App Control/team_A_web_app

echo "🏥 Preparing CareBot System..."

# 1. Clean (Optional, safer)
# rm -rf build install log

# 2. Build
echo "🔧 Building Workspace..."
colcon build --symlink-install

# 3. Path Fix (Crucial)
echo "🔗 Linking Web Server..."
mkdir -p install/rmitbot_web_controller/lib/rmitbot_web_controller
ln -sf "$(pwd)/install/rmitbot_web_controller/bin/web_server" \
       "install/rmitbot_web_controller/lib/rmitbot_web_controller/web_server"

echo "✅ Ready to Launch!"
echo ""
echo "Open TWO terminals:"
echo "1. Robot:  ros2 launch rmitbot_controller controller.launch.py"
echo "2. Web:    ros2 launch rmitbot_web_controller web.launch.py"
echo ""
echo "Then go to: http://localhost:8000"
