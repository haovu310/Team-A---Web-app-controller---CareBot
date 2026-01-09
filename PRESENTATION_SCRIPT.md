# CareBot Autonomous System - Presentation Script
## Work Package 4: Autonomous Implementation with ROS2
### Duration: 3-5 minutes

---

## SLIDE 1: TITLE SLIDE (0:00 - 0:15)
**Visual cues: Make eye contact with camera, smile confidently**

"Good [morning/afternoon], everyone. My name is [Your Name], and today I'm excited to present Work Package 4—Autonomous Implementation with ROS2—for the CareBot healthcare robot project. This work package represents the intelligence layer of our robot, encompassing sensor integration, navigation systems, and the user interface that brings autonomous capabilities to healthcare workers. Over the next few minutes, I'll demonstrate how we transformed a mechanical platform into a fully autonomous system."

---

## SLIDE 2: WP4 OVERVIEW & DELIVERABLES (0:15 - 0:50)

"Work Package 4 is the most technically complex component of the CareBot project, focusing on autonomous behavior through ROS2 integration. According to our project specification, WP4 consists of three major deliverables:

**Deliverable 4.1: LiDAR Implementation** – Integration of the RPLidar A1M8 sensor for environmental perception and obstacle detection. This provides the robot with 360-degree awareness of its surroundings.

**Deliverable 4.2: Camera Implementation** – Integration of the Raspberry Pi Camera Module V3 for visual navigation, AprilTag detection, and future AI applications like object recognition.

**Deliverable 4.3: Navigation and Mapping** – Implementation of SLAM (Simultaneous Localization and Mapping) using SLAM Toolbox, and autonomous navigation using Nav2 stack. This enables the robot to build maps, localize itself, and navigate autonomously.

Additionally, we developed a **Web Application Controller** to provide an accessible user interface for healthcare staff to interact with these autonomous capabilities without requiring robotics expertise.

This work package required two team members working collaboratively due to its technical depth and integration complexity."

---

## SLIDE 3: SYSTEM ARCHITECTURE - ROS2 INTEGRATION (0:50 - 1:35)

"Let me explain the complete system architecture. At the core, everything runs on ROS2 Jazzy, which is the Robot Operating System framework providing communication infrastructure between all components.

**The Sensor Layer** consists of three critical inputs:
- **RPLidar A1M8** publishing laser scan data at 10Hz on the `/scan` topic, providing 360-degree range measurements
- **Raspberry Pi Camera Module V3** publishing compressed images on `/camera/image_raw/compressed` for visual feedback
- **Motor Encoders** through the Arduino firmware, publishing odometry data on `/odom` for position tracking

**The Processing Layer** includes:
- **SLAM Toolbox** consuming LiDAR and odometry data to build occupancy grid maps and maintain robot localization
- **Robot Localization (EKF)** fusing multiple sensor sources for accurate pose estimation
- **Nav2 Stack** handling path planning, obstacle avoidance, and goal execution
- **AprilTag Detection** processing camera images to detect fiducial markers for visual navigation

**The Control Layer** consists of:
- **twist_mux** prioritizing velocity commands from multiple sources (keyboard, autonomous nav, emergency stop)
- **Hardware Interface** sending finalized commands to motor controllers

**The User Interface Layer** provides:
- **Web Application** for browser-based control and monitoring
- **RViz2** for advanced visualization during development

This layered architecture demonstrates proper software engineering principles—separation of concerns, modularity, and standardized interfaces through ROS2 topics and services."

---

## SLIDE 4: D4.1 - LIDAR IMPLEMENTATION (1:35 - 2:10)

"Let me discuss Deliverable 4.1—LiDAR implementation. We integrated the RPLidar A1M8, which is a 360-degree laser ranging scanner that provides distance measurements to surrounding objects.

**Technical Implementation**: We used the `rplidar_ros` package, which interfaces with the sensor over USB serial communication. The sensor rotates at 10Hz, capturing distance measurements at 8000 samples per second. This data is published on the `/scan` topic as LaserScan messages.

**Key Configuration**: We configured the sensor in the `rplidar.launch.py` file with parameters including frame_id for coordinate transformation, inverted orientation to match our robot's mounting, and scan angle filtering to remove readings blocked by the robot chassis.

**Integration Challenges**: The primary challenge was coordinate frame alignment. The LiDAR sensor frame must be properly defined in the robot's URDF with accurate position and orientation relative to the base_link frame. We achieved millimeter-level accuracy in mounting position specification, which is critical for accurate SLAM.

**Performance Metrics**: The sensor provides 0.15 to 12-meter range with 1-degree angular resolution. In practical testing, we achieved consistent obstacle detection at 6 meters in typical indoor environments, which exceeds our requirements for hospital corridor navigation.

This sensor forms the foundation of our perception system, enabling both mapping and obstacle avoidance capabilities."

---

## SLIDE 5: D4.2 - CAMERA & VISION SYSTEM (2:10 - 2:50)

"Deliverable 4.2 focuses on camera implementation and computer vision capabilities. We integrated the Raspberry Pi Camera Module V3, which is a 12-megapixel sensor with autofocus capabilities.

**ROS2 Integration**: We set up a dedicated `camera_ws` workspace using the `camera_ros` package. This node interfaces with the Pi's camera hardware through the libcamera API and publishes images on multiple topics: raw images, compressed images for bandwidth efficiency, and camera_info for calibration parameters.

**AprilTag Detection**: Beyond basic image capture, we implemented AprilTag detection using the `apriltag_ros` package. AprilTags are fiducial markers—like QR codes but optimized for robotic vision. The system detects tags in the camera frame, estimates their 3D pose, and publishes transformations to the TF tree. This enables visual localization and navigation to specific landmarks.

**Configuration Details**: In our `apriltag_params.yaml`, we configured the system for the 36h11 tag family, which provides robust detection under varying lighting conditions. We set the detection thread count to 4 to leverage the Pi 5's multi-core processor, achieving real-time detection at 15 frames per second.

**Practical Applications**: AprilTags can mark rooms, medical equipment stations, or patient areas. The robot can navigate to "Room 302" by detecting and approaching the associated tag, providing semantic understanding beyond geometric mapping.

**Integration with SLAM**: Camera data complements LiDAR—while LiDAR provides 2D geometric mapping, the camera enables visual features and landmark recognition, improving overall localization accuracy."

---

## SLIDE 6: D4.3 - SLAM & NAVIGATION SYSTEMS (2:50 - 3:40)

"Deliverable 4.3 is Navigation and Mapping—the culmination of sensor integration. This deliverable has two components: SLAM for map building and Nav2 for autonomous navigation.

**SLAM Toolbox Implementation**: SLAM stands for Simultaneous Localization and Mapping. The SLAM Toolbox package consumes LiDAR scans and odometry to simultaneously build a map of the environment while tracking the robot's position within that map. We configured it in `slam.yaml` with parameters including:
- Resolution: 0.05 meters per pixel for detailed indoor mapping
- Scan matching tolerance for loop closure detection
- Pose graph optimization for correcting accumulated drift

The output is an occupancy grid—a 2D representation where each cell is marked as occupied, free, or unknown.

**Map Persistence**: We implemented map serialization through SLAM Toolbox's `/serialize_map` service. Maps are saved as `.posegraph` files containing the pose graph structure and `.data` files with the occupancy grid. This allows reuse of previously built maps.

**Nav2 Stack**: For autonomous navigation, we configured Nav2 with custom parameters in `nav2_params.yaml`. Key components include:
- **Global Planner**: Uses Dijkstra's algorithm to find optimal paths on the map
- **Local Planner (DWB)**: Generates velocity commands while avoiding dynamic obstacles
- **Costmap Layers**: Integrates static map, LiDAR-detected obstacles, and inflation zones
- **Recovery Behaviors**: Handles stuck situations through rotation and backing up

**Navigation Workflow**: When given a goal pose, Nav2 plans a global path, then continuously replans locally while following that path, adjusting for obstacles in real-time. The system publishes progress on the `/navigate_to_pose` action server.

This implementation enables fully autonomous navigation—the robot can navigate from Point A to Point B while dynamically avoiding obstacles, which is essential for healthcare delivery tasks."

---

## SLIDE 7: WEB APPLICATION INTERFACE (3:40 - 4:25)

"To make these autonomous capabilities accessible to non-technical users, we developed a web-based control interface. This represents the user-facing component of Work Package 4.

**Architecture**: The web application uses a client-server architecture. On the server side, we have two Python nodes:
- `web_server_node`: Serves HTML/CSS/JS files and provides REST API endpoints for map management
- `camera_stream_node`: Converts ROS2 compressed images to MJPEG HTTP streams for browser compatibility

On the client side, JavaScript connects to ROS2 through `rosbridge_suite`, which provides WebSocket-to-ROS translation. This allows the browser to subscribe to topics, publish messages, and call services using JSON.

**Three-Mode Operation System**:
- **IDLE Mode**: Safe state with built-in documentation and system status
- **MANUAL Mode**: Direct teleoperation with keyboard (WASD) or on-screen D-pad controls. Supports 8-directional movement including diagonals for mecanum wheels. Enables real-time SLAM mapping.
- **AUTO Mode**: Loads saved maps and accepts navigation goals via coordinate entry or map clicking. Displays navigation progress and robot position in real-time.

**Real-Time Visualization**: The interface displays live camera feed using MJPEG streaming with <200ms latency, and real-time map rendering using ROS2D with HTML5 Canvas showing occupancy grid, robot position, and navigation paths.

**Map Management**: Users can save maps during Manual mode mapping sessions and load existing maps for Auto mode navigation. The system implements a RESTful API that scans for `.posegraph` files and provides search/filter functionality.

**Significance**: This interface democratizes robotics—healthcare staff can operate an autonomous robot without ROS2 knowledge. Any device with a browser becomes a control terminal. This is critical in hospital settings where specialized robotics workstations are impractical."

---

## SLIDE 8: SYSTEM INTEGRATION & TECHNICAL CHALLENGES (4:25 - 5:05)

"Integrating all WP4 components required solving several significant technical challenges:

**Challenge 1: Multi-Sensor Calibration and Synchronization**. Each sensor operates at different rates—LiDAR at 10Hz, camera at 15fps, odometry at 50Hz. The robot_localization EKF filter fuses these asynchronously while handling different latencies. We configured covariance matrices to weight each sensor appropriately based on its reliability.

**Challenge 2: Coordinate Frame Management**. ROS2 requires precise transformation trees. We defined frames for base_link, laser_frame, camera_link, and odom in the URDF with millimeter accuracy. The `static_transform_publisher` maintains these relationships, enabling SLAM Toolbox and Nav2 to correlate sensor data correctly.

**Challenge 3: Network Architecture for Distributed Computing**. The system spans Raspberry Pi (sensors, low-level control) and PC (navigation, visualization). We configured ROS_DOMAIN_ID=42 for network isolation and verified topic discovery across machines. The web interface needed to handle connections to services running on different hosts.

**Challenge 4: Real-Time Performance on Embedded Hardware**. Running SLAM, navigation, and vision on a Raspberry Pi 5 required optimization: reducing camera resolution to 640x480, configuring SLAM Toolbox for asynchronous processing, and using compressed image transport to reduce bandwidth.

**Challenge 5: twist_mux Priority Management**. Multiple sources can send velocity commands (keyboard, web app, autonomous nav, emergency stop). The twist_mux prioritizes these with autonomous navigation having lowest priority, manual control medium, and emergency stop highest. This ensures safe operation.

These challenges taught me that robotics is systems engineering—success requires integration expertise beyond individual component knowledge."

---

## SLIDE 9: REAL-WORLD IMPACT & HEALTHCARE APPLICATIONS (5:05 - 5:35)

"Let me connect this technical work to real-world healthcare impact, which aligns with Sustainable Development Goal 3: Good Health and Well-Being.

**Medication Delivery Scenario**: A nurse prepares medications at the pharmacy station. Using the web interface, she selects 'Ward 3, Bed 12' from preset destinations. The robot autonomously navigates through corridors, avoiding staff and equipment, delivering medications while the nurse attends to other patients. This reduces nurse walking time by up to 30%, allowing more direct patient care.

**Contact-Free Sample Transport**: During infectious disease outbreaks, the robot can transport lab samples or supplies between isolation wards and labs, minimizing healthcare worker exposure. The autonomous navigation system ensures reliable delivery without human escort.

**Elderly Care Facilities**: In aged care homes, residents often require regular medication rounds or meal delivery. The robot can perform these routine tasks, with staff monitoring through the web interface from a central station. The AprilTag system enables room-specific navigation: 'Deliver to Mrs. Johnson's room'.

**Scalability**: The system architecture supports fleet management. Multiple robots could share maps and coordinate tasks. The web interface's multi-viewer capability means supervisors can monitor entire fleets from a dashboard.

**Accessibility**: The no-installation web interface means temporary staff or visiting healthcare providers can immediately utilize the robot without training, crucial in high-turnover healthcare environments.

This demonstrates that WP4 delivers not just technical achievement, but meaningful human impact through thoughtful engineering."

---

## SLIDE 10: SYSTEM DEMONSTRATION (5:35 - 6:00)

**[Show screen recording or sequential screenshots demonstrating full workflow]**

"Let me demonstrate the complete Work Package 4 system in operation:

**Sensor Verification** (0:00-0:10): In RViz, we see live LiDAR scans showing surrounding walls and obstacles, camera feed displaying the robot's forward view, and odometry data tracking wheel rotations.

**Manual Mapping** (0:10-0:25): Switching to Manual mode in the web interface, I drive the robot using the D-pad. Watch the SLAM Toolbox building the occupancy grid in real-time as the LiDAR explores new areas. Black cells represent walls, white represents free space, and gray is unexplored.

**Map Saving** (0:25-0:30): Once the area is mapped, I save it as 'hospital_corridor_A'. The system calls SLAM Toolbox's serialize service, storing the map to disk.

**Autonomous Navigation** (0:30-0:50): Switching to Auto mode, I load the saved map. The robot's position appears on the map. I click a destination point across the room. Nav2 immediately plans a path (shown in green), then the robot begins autonomous navigation, smoothly avoiding obstacles detected by LiDAR while following the global plan.

**AprilTag Detection** (0:50-1:00): As the robot approaches a marked station with an AprilTag, the vision system detects it, publishes the tag's pose to TF, enabling precise docking or confirmation of arrival location.

This 60-second demonstration shows the complete autonomous cycle: sense, plan, act—the fundamental loop of mobile robotics."

---

## SLIDE 11: LESSONS LEARNED & FUTURE ENHANCEMENTS (6:00 - 6:25)

"This work package provided invaluable learning across multiple engineering domains:

**Technical Skills Gained**:
- Deep understanding of ROS2 architecture: topics, services, actions, and the TF transform system
- Sensor integration and calibration for multi-modal perception systems
- SLAM algorithms and probabilistic robotics concepts like particle filters and pose graph optimization
- Full-stack development bridging embedded systems and web technologies
- Distributed systems with network communication between Raspberry Pi and PC

**Engineering Lessons**:
- The importance of systematic debugging with `ros2 topic echo`, `ros2 bag record`, and RViz visualization
- Documentation is critical—our DEPLOYMENT_GUIDE.md and VISION_SETUP.md enabled reproducible setups
- Modularity pays off—separating concerns into rmitbot_mapping, rmitbot_navigation, and rmitbot_web_controller packages made parallel development and testing manageable

**Future Enhancements**:
- **3D Navigation**: Integrate depth cameras for multi-floor mapping and obstacle classification
- **AI Vision**: Implement YOLO object detection to recognize medical equipment, people, or hazards
- **Fleet Coordination**: Multi-robot systems with task allocation and collision avoidance
- **Voice Interface**: Web Speech API for hands-free operation: 'Robot, go to Pharmacy'
- **Predictive Maintenance**: Monitor motor currents and vibration for failure prediction
- **Hospital System Integration**: Connect to Electronic Health Records for delivery task automation

These enhancements would transform CareBot from a research platform to a production-ready healthcare solution."

---

## SLIDE 12: CONCLUSION & ACKNOWLEDGMENTS (6:25 - 6:45)

"To conclude, Work Package 4 represents the complete implementation of autonomous capabilities for the CareBot platform:

**Technical Achievements**:
✅ **D4.1 Complete**: RPLidar A1M8 integrated, providing 360-degree environmental perception at 10Hz
✅ **D4.2 Complete**: Pi Camera Module V3 with AprilTag detection, enabling visual navigation and future AI applications  
✅ **D4.3 Complete**: SLAM Toolbox mapping and Nav2 autonomous navigation functional with obstacle avoidance
✅ **Bonus**: Accessible web interface enabling non-technical healthcare staff operation

**Engineering Excellence Demonstrated**:
- Multi-sensor fusion and calibration
- Real-time embedded systems programming  
- Distributed robotics architecture
- Human-centered interface design
- Systems integration across hardware, firmware, middleware, and application layers

**Impact**: We've created a system where a healthcare worker can say 'deliver this to Room 305,' click a button, and the robot autonomously navigates there—transforming autonomous robotics from research concept to practical healthcare tool.

I want to acknowledge my WP4 team member for collaborative work on sensor integration and navigation tuning, team members from other work packages for mechanical and electrical integration, our instructors for guidance through complex challenges, and the open-source ROS2 community whose powerful tools made this possible.

Thank you for your attention. I'm ready to answer questions about any aspect of the autonomous system—from low-level sensor drivers to high-level navigation algorithms to user interface design."

---

## DELIVERY TIPS

### Voice and Pace:
- Speak clearly at 130-150 words per minute
- Pause after complex technical terms to let them sink in
- Vary your tone to maintain engagement
- Use rising intonation for questions, falling for statements

### Body Language:
- Maintain eye contact with camera 80% of the time
- Smile naturally, especially during introduction and conclusion
- Use hand gestures when describing architecture and data flow
- Sit up straight with shoulders back—conveys confidence

### Transition Phrases:
- "Moving on to..." / "Let's now examine..."
- "This is significant because..."
- "What makes this interesting is..."
- "Building on this concept..."

### If Running Short on Time:
- Condense slides 7 and 11 (challenges and lessons)
- Keep slides 3-6 (architecture and features) in full detail
- Never rush the conclusion

### If Running Over Time:
- You have good content coverage through slide 10
- Slide 11 can be condensed to 15 seconds
- Slide 12 can be 10 seconds if needed

### Handling Technical Questions:
- Pause 2 seconds before answering—shows you're thinking
- If you don't know: "That's an excellent question. I'd need to research [X] to give you a precise answer, but my initial thought is..."
- Relate answers back to project goals and real-world impact

---

## TECHNICAL TERMS GLOSSARY (Know These Cold)

**Core ROS2 Concepts:**
- **ROS2 (Robot Operating System 2)**: Middleware framework for robotics software development with distributed communication
- **Topic**: Named bus for publishing/subscribing to typed messages (e.g., `/scan` for LiDAR data)
- **Service**: Synchronous request-response communication pattern
- **Action**: Asynchronous goal-based communication with feedback (e.g., navigation goals)
- **TF (Transform Framework)**: System for tracking coordinate frames and transformations between them

**Sensor Technologies:**
- **LiDAR (Light Detection and Ranging)**: Laser-based ranging sensor providing distance measurements
- **RPLidar A1M8**: 360-degree 2D laser scanner with 12m range and 10Hz scan rate
- **LaserScan Message**: ROS message containing array of range measurements at angular intervals
- **Odometry**: Measurement of robot position change based on wheel encoder data
- **IMU (Inertial Measurement Unit)**: Sensor measuring acceleration and angular velocity

**SLAM & Navigation:**
- **SLAM (Simultaneous Localization and Mapping)**: Building a map while tracking position within it
- **SLAM Toolbox**: ROS2 package implementing graph-based SLAM with pose optimization
- **Occupancy Grid**: 2D grid representation of environment (occupied/free/unknown cells)
- **Pose Graph**: Network of robot poses connected by relative transform constraints
- **Loop Closure**: Recognizing previously visited locations to correct accumulated drift

**Navigation Stack (Nav2):**
- **Nav2**: ROS2 navigation framework for autonomous mobile robot path planning and control
- **Global Planner**: Finds optimal path on static map using algorithms like Dijkstra or A*
- **Local Planner (DWB)**: Generates velocity commands following global path while avoiding obstacles
- **Costmap**: 2D grid assigning traversal cost to each cell based on obstacles and inflation
- **Recovery Behaviors**: Actions like rotation or backing up when robot gets stuck
- **Behavior Tree**: Hierarchical control structure coordinating navigation behaviors

**Computer Vision:**
- **AprilTag**: Fiducial marker system for robust pose estimation in camera images
- **Fiducial Marker**: Visual pattern with known geometry for camera localization
- **Camera Calibration**: Determining intrinsic (focal length, distortion) and extrinsic (position/orientation) parameters
- **Compressed Images**: JPEG-compressed image transport reducing bandwidth by ~90%

**Sensor Fusion:**
- **EKF (Extended Kalman Filter)**: Algorithm fusing multiple sensor sources for state estimation
- **robot_localization**: ROS2 package implementing EKF/UKF for pose estimation
- **Covariance Matrix**: Statistical measure of sensor uncertainty used for fusion weighting
- **Multi-Modal Fusion**: Combining different sensor types (LiDAR, odometry, IMU, vision)

**Web & Networking:**
- **rosbridge_suite**: Provides WebSocket interface to ROS2, converting messages to JSON
- **WebSocket**: Protocol for bidirectional real-time communication over TCP
- **MJPEG (Motion JPEG)**: Video streaming format sending individual JPEG frames
- **REST API**: Architectural style for web services using HTTP methods (GET, POST, etc.)
- **CORS (Cross-Origin Resource Sharing)**: Security mechanism controlling cross-domain web requests

**Control Systems:**
- **twist_mux**: ROS package multiplexing velocity commands from sources with priority levels
- **TwistStamped**: ROS message type for velocity commands (linear/angular) with timestamp
- **Mecanum Wheels**: Omni-directional wheels with rollers enabling lateral movement
- **Differential Drive**: Two independently driven wheels for steering by speed difference

**System Architecture:**
- **Distributed System**: Software across multiple computers communicating over network
- **Middleware**: Software layer abstracting communication between application components
- **Launch File**: ROS2 configuration file starting multiple nodes with parameters
- **URDF (Unified Robot Description Format)**: XML format describing robot kinematic/dynamic model
- **Static Transform**: Fixed transformation between coordinate frames (e.g., sensor mounting)

---

## BACKUP SLIDES (If Questions Arise)

### Backup Slide A: Detailed System Specifications

**Hardware Configuration:**
- **Main Computer**: Raspberry Pi 5 (8GB RAM, quad-core ARM Cortex-A76)
- **Sensors**:
  - RPLidar A1M8: 360° scan, 8000 samples/sec, 0.15-12m range, USB interface
  - Pi Camera Module V3: 12MP, autofocus, 1080p@50fps, MIPI CSI interface
  - Wheel encoders: 600 PPR via Arduino Mega
- **Actuators**: 4× mecanum wheels with DC motors, H-bridge control
- **Network**: WiFi (802.11ac) or Ethernet for ROS2 DDS communication
- **Power**: 12V LiPo battery with 5V/3A regulators

**Software Stack:**
- **OS**: Ubuntu 24.04 LTS (ARM64)
- **ROS2**: Jazzy Jalisco distribution
- **Key Packages**: 
  - slam_toolbox (2.8.1)
  - nav2_bringup (1.3.2)  
  - robot_localization (3.7.0)
  - apriltag_ros (3.4.1)
  - rosbridge_suite (1.3.2)

**Performance Metrics:**
- LiDAR update rate: 10 Hz
- Camera frame rate: 15 FPS (configurable)
- Odometry rate: 50 Hz
- SLAM processing: 0.05s per scan (~20 Hz capability)
- Nav2 control loop: 20 Hz
- Web interface latency: <200ms
- Maximum velocity: 0.5 m/s (linear), 1.0 rad/s (angular)
- Navigation accuracy: ±0.1m position, ±5° orientation

### Backup Slide B: ROS2 Topic & Service Architecture

**Published Topics:**
```
/scan (sensor_msgs/LaserScan) - LiDAR measurements
/camera/image_raw/compressed (sensor_msgs/CompressedImage) - Camera feed
/odom (nav_msgs/Odometry) - Wheel odometry
/tf (tf2_msgs/TFMessage) - Coordinate transformations
/tf_static (tf2_msgs/TFMessage) - Fixed transformations
/map (nav_msgs/OccupancyGrid) - SLAM-generated map
/cmd_vel (geometry_msgs/Twist) - Final velocity commands to motors
/diagnostics (diagnostic_msgs/DiagnosticArray) - System health
```

**Subscribed Topics:**
```
/cmd_vel_keyboard (geometry_msgs/TwistStamped) - Manual control input
/goal_pose (geometry_msgs/PoseStamped) - Navigation goal
/initialpose (geometry_msgs/PoseWithCovarianceStamped) - Localization reset
```

**Services:**
```
/slam_toolbox/serialize_map - Save current map to disk
/slam_toolbox/deserialize_map - Load map from disk  
/slam_toolbox/toggle_interactive_mode - Enable/disable pose adjustment
/navigate_to_pose - Send navigation goal (action server)
/clear_costmap - Reset Nav2 costmaps
```

**Parameter Namespaces:**
```
/slam_toolbox/* - SLAM configuration (resolution, scan matching, optimization)
/controller_server/* - Local planner (DWB) parameters  
/planner_server/* - Global planner parameters
/robot_localization/* - EKF fusion parameters
/apriltag/* - Tag detection and pose estimation
```

### Backup Slide C: Configuration Files Overview

**Key Configuration Files:**

1. **rmitbot_mapping/config/slam.yaml**
   - Map resolution: 0.05m
   - Scan matcher: correlation-based with 0.5 search space
   - Loop closure: enabled with 3.0 tolerance
   - Mode: asynchronous (non-blocking)

2. **rmitbot_navigation/config/nav2_params.yaml**
   - Global planner: Smac Planner Hybrid-A* for non-holonomic constraints
   - Local planner: DWB with TEB optimization
   - Costmap resolution: 0.05m, inflation radius: 0.55m
   - Max velocities: 0.5 m/s linear, 1.0 rad/s angular
   - Obstacle layer: 12m range, decay model
   
3. **rmitbot_localization/config/ekf.yaml**
   - Fused sources: odometry (6-DOF), IMU (orientation)
   - Update frequency: 50 Hz
   - Covariances: odometry (0.01), IMU (0.05)

4. **rmitbot_vision/config/apriltag_params.yaml**
   - Tag family: 36h11
   - Max hamming distance: 2 (error tolerance)
   - Detection threads: 4 (multi-core)
   - Minimum decision margin: 50 (quality threshold)

5. **rmitbot_controller/config/twistmux.yaml**
   - Priority levels: emergency=100, manual=50, auto=10
   - Timeout: 0.5s (commands expire if not refreshed)

### Backup Slide D: Coordinate Frame Tree

```
odom ← [EKF fusion]
  └── base_link (robot center, XY plane)
      ├── laser_frame (+0.15m forward, +0.10m up)
      ├── camera_link (+0.20m forward, +0.15m up, pitched 10° down)
      ├── imu_link (centered, +0.05m up)
      ├── wheel_front_left  
      ├── wheel_front_right
      ├── wheel_rear_left
      └── wheel_rear_right

map ← [SLAM Toolbox]
  └── odom (corrected for drift via loop closure)
```

**Frame Conventions (REP-103):**
- X: forward, Y: left, Z: up
- Right-hand coordinate system
- All sensors report in their local frame, TF transforms to base_link
- SLAM provides map→odom transform (drift correction)
- EKF provides odom→base_link transform (fused pose estimate)

### Backup Slide E: Network Configuration

**Multi-Machine Setup:**

**Raspberry Pi (Robot Onboard):**
- IP: 172.20.10.4 (static)
- ROS_DOMAIN_ID: 42
- Runs: sensors, low-level control, SLAM, camera, rosbridge
- CPU load: ~60% during SLAM, ~40% during navigation

**Development PC:**
- IP: 172.20.10.10
- ROS_DOMAIN_ID: 42 (must match!)
- Runs: RViz, Nav2 computation, web server
- Connection: SSH for terminal, ROS2 DDS for topics

**Network Requirements:**
- Bandwidth: ~5 Mbps (compressed camera + topics)
- Latency: <50ms recommended for responsive control
- Ports: 9090 (rosbridge WebSocket), 8000 (web server), 8001 (camera stream)

**Firewall Configuration:**
```bash
sudo ufw allow 9090/tcp  # rosbridge
sudo ufw allow 8000/tcp  # web server
sudo ufw allow 8001/tcp  # camera stream  
sudo ufw allow 7400:7450/udp  # ROS2 DDS discovery
```

### Backup Slide F: Debugging & Testing Methodology

**Common Debug Commands:**
```bash
# Check if LiDAR is publishing
ros2 topic hz /scan
ros2 topic echo /scan --once

# Verify camera feed
ros2 topic list | grep camera
ros2 run image_view image_view --ros-args --remap image:=/camera/image_raw

# Monitor TF tree
ros2 run tf2_tools view_frames  # generates PDF
ros2 run tf2_ros tf2_echo map base_link

# Test navigation
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "..."
ros2 action send_goal /navigate_to_pose ...

# Record data for offline analysis
ros2 bag record /scan /camera/image_raw/compressed /odom /tf
```

**Testing Procedure:**
1. **Sensor verification**: Confirm data on all sensor topics
2. **SLAM validation**: Drive manually, verify map builds correctly
3. **Localization test**: Reset robot pose, verify re-localization
4. **Navigation test**: Send waypoints, confirm path planning and execution
5. **Obstacle avoidance**: Place dynamic obstacles, verify replanning
6. **Integration test**: Full autonomous mission with web interface

### Backup Slide G: Safety & Error Handling

**Safety Features Implemented:**
- **Emergency stop**: Highest priority in twist_mux, instantly halts robot
- **Velocity limits**: Hard-coded maximum speeds prevent dangerous accelerations
- **Timeout protection**: Commands expire after 0.5s, preventing runaway
- **Obstacle inflation**: 0.55m safety buffer around obstacles in costmaps
- **Recovery behaviors**: Controlled responses to stuck situations (no violent maneuvers)
- **Connection monitoring**: Web UI displays real-time ROS connection status

**Error Handling:**
- **Sensor failure**: Navigation continues with degraded capability (e.g., odometry-only if LiDAR fails)
- **Network loss**: Robot stops if no commands received for timeout period
- **SLAM failure**: Falls back to odometry-based dead reckoning
- **Navigation failure**: Recovery behaviors attempt to unstick, then cancel goal if unsuccessful
- **Map-world mismatch**: Large particle spread triggers localization warning

**Future Safety Enhancements:**
- Proximity sensors for close-range (<0.5m) obstacle detection
- Audible/visual warnings during movement
- Remote emergency stop via web interface
- Automatic return-to-dock on low battery
- Collision detection via motor current monitoring

---

## FINAL CHECKLIST

**24 Hours Before:**
- [ ] Practice presentation 3-5 times
- [ ] Record yourself—check pace and clarity
- [ ] Test all demo videos/screenshots
- [ ] Prepare backup laptop/connection

**1 Hour Before:**
- [ ] Close unnecessary applications
- [ ] Check camera angle and lighting
- [ ] Test microphone levels
- [ ] Have water nearby
- [ ] Do vocal warm-up exercises

**Recording Starts:**
- [ ] Deep breath—smile—begin
- [ ] Maintain energy throughout
- [ ] Stick to time limits
- [ ] End strong with confidence

**After Recording:**
- [ ] Review before submitting
- [ ] Check audio levels
- [ ] Verify all slides are visible
- [ ] Ensure file format is correct

---

**Remember**: You built something impressive. Show it with confidence, explain it with clarity, and demonstrate your expertise with enthusiasm. Good luck!
