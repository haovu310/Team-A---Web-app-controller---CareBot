# CareBot Deployment Guide - Real Robot (Pi 5 + PC)

## Quick Reference

**Pi IP:** `172.20.10.4`  
**Pi User:** `alpha-pi`  
**ROS_DOMAIN_ID:** `42` (set on both machines)

---

## Pre-Deployment Checklist

### On Raspberry Pi 5

1. **Install camera_ws** (if not already done):
   - Follow instructions in VISION_SETUP.md
   - Workspace location: `~/camera_ws`

2. **Set ROS_DOMAIN_ID:**
   ```bash
   echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Verify network:**
   ```bash
   ping 172.20.10.4  # Should respond
   ifconfig          # Verify IP is correct
   ```

### On PC

1. **Set ROS_DOMAIN_ID:**
   ```bash
   echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
   source ~/.bashrc
   ```

2. **Verify network:**
   ```bash
   ping 172.20.10.4  # Should reach Pi
   ```

---

## Deployment Steps

### Step 1: Launch on Raspberry Pi

1. **SSH into Pi:**
   ```bash
   ssh alpha-pi@172.20.10.4
   ```

2. **Source Workspaces:**
   ```bash
   source ~/camera_ws/install/setup.bash
   source ~/your_workspace/install/setup.bash  # Your main workspace
   ```

3. **Edit Launch File:**
   ```bash
   nano ~/your_workspace/src/rmitbot_bringup/launch/rmitbot.launch.py
   ```
   
   In the `return LaunchDescription([...])` section, **uncomment**:
   ```python
   controller,
   rplidar,
   localization,
   vision,
   twistmux,
   ```
   
   **Comment out:**
   ```python
   # display,
   # mapping,
   # navigation_delayed,
   ```

4. **Launch Robot:**
   ```bash
   ros2 launch rmitbot_bringup rmitbot.launch.py
   ```

5. **Verify Topics (in new SSH terminal):**
   ```bash
   ros2 topic list
   # Should see: /cmd_vel, /odom, /scan, /camera/image_raw, etc.
   ```

### Step 2: Launch on PC

1. **Edit Launch File:**
   ```bash
   nano src/rmitbot_bringup/launch/rmitbot.launch.py
   ```
   
   In the `return LaunchDescription([...])` section, **uncomment**:
   ```python
   display,
   mapping,
   navigation_delayed,
   ```
   
   **Comment out:**
   ```python
   # controller,
   # rplidar,
   # localization,
   # vision,
   # twistmux,
   ```

2. **Launch PC Components:**
   ```bash
   source ~/your_workspace/install/setup.bash
   ros2 launch rmitbot_bringup rmitbot.launch.py
   ```

3. **Verify Cross-Machine Communication:**
   ```bash
   ros2 topic list
   # Should see topics from BOTH Pi and PC
   ros2 topic echo /scan --no-arr  # Should see lidar data from Pi
   ```

4. **Launch Web Interface:**
   ```bash
   ros2 launch rmitbot_web_controller web.launch.py
   ```

5. **Open Browser:**
   ```
   http://localhost:8000
   ```
   The page will connect to Pi's rosbridge and camera stream automatically.

---

## Testing Procedure

### Test 1: Manual Control

1. Open web UI
2. Switch to **MANUAL** mode
3. Use WASD keys or D-pad buttons
4. **Expected:** Robot moves smoothly

### Test 2: Camera Feed

1. Check left panel of web UI
2. **Expected:** Live video feed visible
3. If not, check Pi terminal for `camera_stream` errors

### Test 3: Localization

1. In PC terminal:
   ```bash
   ros2 run tf2_ros tf2_echo odom base_link
   ```
2. Move robot manually
3. **Expected:** Position updates in real-time

### Test 4: Mapping

1. Drive robot around slowly in **MANUAL** mode
2. Watch RViz on PC
3. **Expected:** Map builds as robot explores

### Test 5: Autonomous Navigation

1. Build a map first (Test 4)
2. Switch to **AUTO** mode in web UI
3. Click a waypoint button (e.g., "Table A")
4. **Expected:** Robot navigates autonomously

---

## Troubleshooting

### Problem: Web UI shows "Disconnected"

**Cause:** Can't reach rosbridge on Pi  
**Fix:**
1. Verify Pi is running: `ssh alpha-pi@172.20.10.4`
2. Check rosbridge running on Pi: `ros2 node list | grep rosbridge`
3. Check firewall on Pi: `sudo ufw status` (should be inactive or allow 9090)

### Problem: Camera shows "Camera Offline"

**Cause:** Can't reach camera stream  
**Fix:**
1. Verify camera_stream running on Pi: `ros2 node list | grep camera_stream`
2. Check camera topic: `ros2 topic hz /camera/image_raw/compressed`
3. Check camera_ws is sourced on Pi

### Problem: Motors don't respond

**Cause:** Topic mismatch or controller not running  
**Fix:**
1. Check controller on Pi: `ros2 node list | grep controller`
2. Listen to /cmd_vel topic: `ros2 topic echo /cmd_vel`
3. Verify twist_mux is running: `ros2 node list | grep twist_mux`

### Problem: No topics from Pi visible on PC

**Cause:** ROS_DOMAIN_ID mismatch  
**Fix:**
1. On both machines: `echo $ROS_DOMAIN_ID` (should be 42)
2. Re-source bashrc or set manually: `export ROS_DOMAIN_ID=42`

### Problem: Navigation won't start

**Cause:** No map or Nav2 not ready  
**Fix:**
1. Build map first in MANUAL mode
2. Save map: Use "Save Map" button in web UI
3. Wait 5 seconds after SLAM starts (navigation_delayed timer)

---

## Performance Notes (Pi 5)

- **CPU Usage:** ~60-70% under full load (all components)
- **Network:** ~10-15 Mbps (5GHz WiFi recommended)
- **Latency:** <20ms typical on good WiFi

**If experiencing lag:**
- Move closer to WiFi router
- Disable AprilTag detection: Comment out `vision` in launch file
- Lower camera resolution in `camera.launch.py`

---

## Files Modified for Distributed Setup

All fixes have been committed separately:

1. **Navigation:** `src/rmitbot_navigation/launch/nav.launch.py`
   - `use_sim_time: False`

2. **Controller:** `src/rmitbot_controller/launch/controller.launch.py`
   - Topic remapped to `/cmd_vel`

3. **Web Controller:** `src/rmitbot_web_controller/content/js/index.js`
   - rosHost = '172.20.10.4'

4. **Camera Stream:** `src/rmitbot_web_controller/content/index.html`
   - Camera URL: `http://172.20.10.4:8001/camera/stream`

5. **Localization:** `src/rmitbot_localization/config/ekf.yaml`
   - `base_link_frame: base_link`

---

## Next Steps

1. Test each component individually
2. Test full integrated system
3. Calibrate navigation parameters if needed
4. Create maps for your environment
5. Define waypoint positions for common locations
