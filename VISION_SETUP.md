# RMITBot Vision System Setup Guide

## Quick Start

The vision system integration is complete! Follow these steps to use camera and AprilTag detection on your robot.

## Prerequisites

**On the Raspberry Pi**, you must have:
- `~/camera_ws` installed (following the provided camera installation guide)
- Camera Module v3 connected to the Pi

**System packages** (install if missing):
```bash
sudo apt install ros-jazzy-apriltag-ros
sudo apt install ros-jazzy-image-proc
sudo apt install ros-jazzy-cv-bridge
```

## Building the Workspace

```bash
cd ~/your_workspace
colcon build --packages-select rmitbot_description rmitbot_vision rmitbot_web_controller
source install/setup.bash
```

## Running the System

### Option 1: Full System with Vision (Recommended)

**On PC:**
```bash
source ~/your_workspace/install/setup.bash
ros2 launch rmitbot_bringup rmitbot.launch.py
```

This launches:
- RViz for visualization
- Twist multiplexer for control
- Navigation stack
- **Vision system** (camera + AprilTag detection)

### Option 2: Vision Only (Testing)

**On Raspberry Pi:**
```bash
# Terminal 1: Source camera workspace
cd ~/camera_ws
source install/setup.bash
source /opt/ros/jazzy/setup.bash

# Launch vision system
ros2 launch rmitbot_vision vision.launch.py
```

**On PC:**
```bash
# View camera feed
ros2 run rviz2 rviz2
# In RViz: Add -> Image -> Topic: /camera/image_raw
```

### Option 3: Web Interface with Camera Stream

**On PC:**
```bash
ros2 launch rmitbot_web_controller web.launch.py
```

Open browser: `http://localhost:8000`

The camera stream will appear in the left panel.

## Camera Configuration

### Adjust Resolution

Lower resolution for better performance:
```bash
ros2 launch rmitbot_vision camera.launch.py camera_width:=320 camera_height:=240
```

Higher resolution for better quality:
```bash
ros2 launch rmitbot_vision camera.launch.py camera_width:=1280 camera_height:=720
```

### Camera Position

The camera is defined in `rmitbot_camera.xacro`:
- **Position:** Front of robot, 14cm forward, 10cm up from base_link
- **Orientation:** Tilted down 15° for better ground view

To adjust position, edit:
```xml
<origin xyz="0.14 0 0.10" rpy="0 0.26 0"/>
```

## Testing AprilTag Detection

### 1. Print AprilTag Markers

Print tags from the 36h11 family:
- https://github.com/AprilRobotics/apriltag-imgs/tree/master/tag36h11
- Print at 121.5mm size (or adjust `apriltag_params.yaml`)

### 2. Launch Vision System

```bash
ros2 launch rmitbot_vision vision.launch.py
```

### 3. View Detections

```bash
# See detected tags
ros2 topic echo /apriltag/detections

# View TF frames
ros2 run tf2_tools view_frames
# Check for frames like: tag36_11_<id>
```

### 4. Visualize in RViz

In RViz:
- Add -> TF -> enable visualization
- Add -> Image -> `/camera/image_raw`
- Add -> Image -> `/apriltag/detection_image` (shows detected tags)

## Topics Published

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | sensor_msgs/Image | Raw camera feed |
| `/camera/image_raw/compressed` | sensor_msgs/CompressedImage | Compressed JPEG feed |
| `/camera/camera_info` | sensor_msgs/CameraInfo | Camera calibration |
| `/apriltag/detections` | apriltag_msgs/AprilTagDetectionArray | Detected tags |
| `/tf` | tf2_msgs/TFMessage | Tag pose transforms |

## Web Interface

Camera stream endpoint: `http://localhost:8001/camera/stream`

The web UI automatically connects to this stream when running the web controller.

## Troubleshooting

### Camera not appearing

**Check camera is detected:**
```bash
libcamera-hello --list-cameras
```

**Check ROS topics:**
```bash
ros2 topic list | grep camera
```

**Check camera_ws is sourced:**
```bash
# Must be sourced before launching
cd ~/camera_ws
source install/setup.bash
```

### AprilTag not detecting

**Lighting:** Ensure good lighting conditions
**Distance:** Keep tags 0.5m - 3m from camera
**Size:** Verify tag size matches config (121.5mm)

**Check subscription:**
```bash
ros2 node info /apriltag_node
# Verify subscriptions show /camera/image_raw
```

### Web stream not working

**Check camera_stream node:**
```bash
ros2 node list | grep camera_stream
```

**Check stream URL:**
```bash
curl http://localhost:8001/camera/stream
# Should return JPEG stream data
```

**Port conflict:** Change port in `web.launch.py` if 8001 is in use

## Use Cases

### 1. Visual Navigation

Place AprilTags at waypoints:
- Tag ID 0 → "Home"
- Tag ID 1 → "Table A"  
- Tag ID 2 → "Table B"
- Tag ID 3 → "Station"

Robot can detect and navigate to these locations precisely.

### 2. Object Detection

Attach AprilTags to objects for:
- Delivery tasks
- Object tracking
- Interaction points

### 3. Localization Correction

AprilTags provide absolute position reference:
- Corrects odometry drift
- Improves SLAM accuracy
- Enables relocalization

## Next Steps

1. **Calibrate camera** for better accuracy:
   ```bash
   ros2 run camera_calibration cameracalibrator
   ```

2. **Add specific tags** to config:
   Edit `apriltag_params.yaml` to define tag IDs and sizes

3. **Integrate with navigation:**
   Create navigation goals based on detected tag poses

4. **Add visual features:**
   Implement object detection, line following, etc.

## File Reference

| File | Purpose |
|------|---------|
| `rmitbot_camera.xacro` | Camera hardware definition |
| `camera.launch.py` | Pi Camera driver |
| `apriltag.launch.py` | AprilTag detection |
| `vision.launch.py` | Combined camera + vision |
| `camera_stream.py` | Web UI MJPEG streamer |
| `apriltag_params.yaml` | AprilTag configuration |
