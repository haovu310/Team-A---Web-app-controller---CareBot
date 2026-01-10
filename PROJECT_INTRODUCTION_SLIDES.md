# CareBot Project Introduction - Slide Structure

## 📊 RECOMMENDED SLIDE DECK (5-7 Slides for Introduction)

---

### SLIDE 1: TITLE SLIDE
**Project Overview & Identity**

**Content:**
- Title: **"CareBot: Autonomous Healthcare Assistance Robot"**
- Subtitle: Work Package 4 - Autonomous Implementation with ROS2
- Your name, student ID, date
- Visual: Professional photo of CareBot showing LiDAR and camera

**Design Notes:**
- Gradient background: Deep Blue (#1A3A6B) to Medical Teal (#00A8A8)
- Include ROS2 logo and RMIT branding
- Keep it clean and professional

**Speaker Notes:**
"Today I'm presenting CareBot, an autonomous healthcare assistance robot designed to support hospital staff with navigation, delivery, and patient monitoring tasks."

---

### SLIDE 2: PROBLEM STATEMENT
**Why Healthcare Needs Autonomous Robots**

**Content - Three Key Challenges:**

1. **Staffing Shortage Crisis**
   - Healthcare worker burnout at all-time high
   - Time spent on non-clinical tasks: 30-40%
   - Hospital staff-to-patient ratios declining globally

2. **Infection Control Challenges**
   - Human-mediated cross-contamination risks
   - Manual delivery of supplies increases exposure
   - Need for contactless logistics solutions

3. **Operational Inefficiency**
   - Staff walking 4-5 miles per shift for supply retrieval
   - Equipment search time: avg. 15-20 minutes per shift
   - Poor resource utilization

**Visual Elements:**
- Split screen: Left side shows stressed healthcare workers, right side shows inefficient manual processes
- Statistics displayed with icons (chart icons, clock icons, walking person)
- Color scheme: Use Alert Red (#F44336) for problems, transitioning to Success Green for your solution

**Speaker Notes:**
"The healthcare industry faces three critical challenges that autonomous robotics can address. First, there's a global staffing shortage with workers spending up to 40% of their time on non-clinical tasks like retrieving supplies and equipment. Second, infection control is paramount, yet human-mediated logistics create unnecessary exposure risks. Finally, operational inefficiency means nurses walk miles per shift just finding equipment. CareBot directly addresses these pain points."

---

### SLIDE 3: CURRENT TECHNOLOGY LANDSCAPE
**State of Healthcare Robotics**

**Content - Three Categories:**

**1. Existing Commercial Solutions:**
   - **TUG Robot** (Aethon) - Autonomous delivery, $100K+
   - **Moxi** (Diligent Robotics) - Supply retrieval, cloud-based
   - **Relay** (Savioke) - Item delivery in hospitals
   
   **Limitations:**
   - High cost (>$50K-$100K per unit)
   - Proprietary closed systems
   - Require specialized infrastructure (beacons, building modifications)
   - Limited customization for specific hospital workflows

**2. Research & Open-Source Projects:**
   - ROS-based mobile robots (TurtleBot, similar platforms)
   - Academic research prototypes
   
   **Limitations:**
   - Lab-only demonstrations, not clinical-ready
   - Lack user-friendly interfaces for non-technical staff
   - No healthcare-specific features

**3. Our Opportunity - The Gap:**
   - ✅ Affordable (materials cost <$2,000)
   - ✅ Open-source and customizable
   - ✅ Web-based interface (no app installation)
   - ✅ Healthcare-focused design (pink branding, gentle interaction)

**Visual Layout:**
- Left column: Commercial robots (show logos/photos of TUG, Moxi)
- Center column: Research platforms (TurtleBot, academic robots)
- Right column: CareBot advantage (your robot photo with checkmarks)
- Use comparison table format

**Color Coding:**
- Commercial: Blue (#1A3A6B)
- Research: Purple (#9C27B0)  
- CareBot: Medical Teal + Pink (#00A8A8 + #FF69B4)

**Speaker Notes:**
"Let's examine the current technology landscape. Commercial solutions like TUG and Moxi exist, but they cost $50,000 to $100,000 per unit and use proprietary closed systems. Many require building modifications like beacon installation. Research projects demonstrate ROS capabilities but remain lab-bound, lacking clinical readiness. CareBot fills this gap—we've built an affordable, open-source, healthcare-focused autonomous robot with a web interface that requires no technical expertise to operate."

---

### SLIDE 4: OUR APPROACH - SYSTEM OVERVIEW
**How CareBot Achieves Autonomy**

**Content - Three-Layer Architecture:**

**Layer 1: PERCEPTION (Sensing the Environment)**
- 🎯 RPLidar A1M8: 360° laser scanning at 10Hz
- 📷 Pi Camera Module V3: 12MP visual navigation
- 🔄 Motor Encoders: Odometry tracking
- **Result:** Complete environmental awareness

**Layer 2: COGNITION (Understanding & Planning)**
- 🗺️ SLAM Toolbox: Real-time mapping
- 🧭 Nav2 Stack: Path planning & obstacle avoidance
- 🤖 Robot Localization: Sensor fusion (EKF)
- 🏷️ AprilTag Detection: Visual landmarks
- **Result:** Intelligent decision-making

**Layer 3: INTERACTION (Human Interface)**
- 🌐 Web Application: Browser-based control
- 📱 3-Mode Operation: IDLE → MANUAL → AUTO
- 📊 Real-time Visualization: Maps & camera feed
- ☁️ RESTful API: Map management
- **Result:** Accessible to non-technical users

**Unified by ROS2 Jazzy:**
- Standard communication framework
- Modular architecture
- Industry-proven reliability

**Visual Design:**
- Three horizontal layers (perception at bottom, interaction at top)
- Each layer uses different color:
  - Perception: Purple (#9C27B0) with sensor icons
  - Cognition: Deep Blue (#1A3A6B) with brain/algorithm icons
  - Interaction: Pink (#FF69B4) with user interface icons
- Center: Large ROS2 logo connecting all layers
- Arrows showing data flow between layers

**Speaker Notes:**
"Our approach is built on three layers. The Perception layer uses LiDAR for 360-degree environmental scanning, a camera for visual navigation, and wheel encoders for position tracking. The Cognition layer implements SLAM for mapping, Nav2 for autonomous path planning, and AprilTag detection for landmark recognition. The Interaction layer provides a web-based interface with three operating modes accessible to any hospital staff member with a browser. Everything is unified through ROS2, the industry-standard Robot Operating System, ensuring modularity and reliability."

---

### SLIDE 5: OUR APPROACH - KEY INNOVATIONS
**What Makes CareBot Different**

**Content - Four Innovation Pillars:**

**1. Mode-Aware Architecture**
   - 🛑 IDLE: Safe monitoring state
   - 🕹️ MANUAL: Human-guided mapping
   - 🤖 AUTO: Autonomous navigation
   - **Innovation:** Prevents unsafe mode transitions, validates map loading

**2. Map Management Workflow**
   - Build maps in MANUAL mode with real-time SLAM
   - Save to persistent storage (.posegraph format)
   - Load maps in AUTO mode for navigation
   - **Innovation:** Prevents unsaved work loss, clear workflow separation

**3. Dual-Launch System**
   - `mapping.launch.py`: Live SLAM for map building
   - `localization.launch.py`: Static map for navigation
   - **Innovation:** Optimized for each task (mapping vs navigating)

**4. Healthcare-Centric Design**
   - Gentle pink color scheme (non-threatening)
   - Intuitive web interface (no robotics training needed)
   - Failsafe confirmations for safety
   - **Innovation:** Designed for clinical environment, not lab

**Visual Layout:**
- Four quadrants (2x2 grid)
- Each quadrant has icon + title + 3 bullet points
- Use different accent colors for each pillar:
  - Mode Architecture: Success Green (#4CAF50)
  - Map Management: Warning Orange (#FF9800)
  - Dual-Launch: Purple (#9C27B0)
  - Healthcare Design: Pink (#FF69B4)

**Speaker Notes:**
"CareBot introduces several key innovations. First, our mode-aware architecture prevents common errors—you can't navigate without loading a map, and the system warns you before losing unsaved mapping work. Second, we've implemented a clear map management workflow where maps are built in MANUAL mode and loaded explicitly for AUTO navigation. Third, we use a dual-launch system—separate configurations optimized for mapping versus navigating tasks. Finally, everything is designed for healthcare environments with gentle branding, intuitive interfaces, and safety confirmations."

---

### SLIDE 6: TECHNICAL IMPLEMENTATION HIGHLIGHTS
**ROS2 Integration Stack**

**Content - Component Overview:**

**Sensor Integration:**
- RPLidar: `/scan` topic @ 10Hz, LaserScan messages
- Camera: `/camera/image_raw/compressed` for bandwidth efficiency
- Odometry: `/odom` from Arduino-based motor controllers

**Navigation Stack:**
- SLAM Toolbox: Occupancy grid mapping, 0.05m resolution
- Nav2: DWB local planner, recoveries, behavior trees
- Robot Localization: EKF fusing scan matching + odometry

**Control Architecture:**
- Twist Mux: Priority-based velocity command arbitration
- Three control sources: Keyboard, Nav2, Emergency stop
- Hardware abstraction via ROS2 Control

**User Interface:**
- Rosbridge WebSocket: Browser ↔ ROS2 bridge (port 9090)
- Python HTTP server: Serves web app (port 8000)
- MJPEG stream: Camera feed (port 8001)

**Visual Design:**
- System architecture diagram showing:
  - Left: Hardware layer (sensors, motors, Pi5)
  - Center: ROS2 middleware (topics, services, TF)
  - Right: Application layer (SLAM, Nav2, Web UI)
- Use boxes and arrows showing data flow
- Color code by function:
  - Hardware: Dark gray
  - ROS2: Blue gradient
  - Applications: Mixed colors
- Include topic names in small font on arrows

**Speaker Notes:**
"Let me highlight the technical implementation. Sensors publish on standard ROS2 topics—LiDAR at 10Hz, camera with compression for efficiency, and odometry from motor controllers. The navigation stack combines SLAM Toolbox for real-time mapping with Nav2 for path planning. Robot Localization fuses multiple sensor sources using an Extended Kalman Filter. The control architecture uses Twist Mux to arbitrate between manual keyboard control, autonomous navigation, and emergency stops. Finally, the web interface connects through Rosbridge, creating a WebSocket bridge between browsers and ROS2."

---

### SLIDE 7: PROJECT IMPACT & NEXT STEPS
**Real-World Applications & Future Development**

**Content - Two Sections:**

**Healthcare Impact (Left Side):**
- 🏥 **Current Capabilities:**
  - Autonomous navigation in hospital corridors
  - Map building for new environments
  - Waypoint-based task execution
  - Real-time obstacle avoidance

- 💡 **Use Cases:**
  - Supply delivery between stations
  - Equipment location and retrieval
  - Medication cart autonomous routing
  - Patient room navigation with AprilTags

- 🌍 **SDG Alignment:**
  - SDG 3: Good Health and Well-being
  - SDG 9: Industry, Innovation, Infrastructure
  - Reduces healthcare worker burden

**Future Enhancements (Right Side):**
- 🚀 **Phase 2 Features:**
  - Object detection and recognition
  - Voice command integration
  - Multi-robot fleet coordination
  - Electronic health record integration

- 🔬 **Research Opportunities:**
  - Patient monitoring sensors
  - UV disinfection attachment
  - Telepresence capabilities
  - AI-powered anomaly detection

- 📈 **Scalability:**
  - Cloud-based map sharing
  - Hospital-wide deployment
  - Cross-institutional learning

**Visual Layout:**
- Split screen: Left (Impact) vs Right (Future)
- Current state shown with solid icons
- Future features shown with outlined/dashed icons
- Timeline arrow at bottom showing progression
- Use gradient from current (teal) to future (purple)

**Color Scheme:**
- Current capabilities: Success Green (#4CAF50) checkmarks
- Future features: Purple (#9C27B0) with "Coming Soon" badges
- SDG logos in official UN colors

**Speaker Notes:**
"CareBot is already capable of autonomous hospital navigation, real-time mapping, and obstacle avoidance. It aligns with UN Sustainable Development Goals 3 and 9—improving healthcare delivery and fostering innovation. Current use cases include supply delivery, equipment retrieval, and room navigation using AprilTags. Looking ahead, Phase 2 will add object recognition, voice commands, and multi-robot coordination. Research opportunities include patient monitoring sensors, UV disinfection, and AI-powered anomaly detection. The system is designed for scalability—we envision cloud-based map sharing enabling hospital-wide or even cross-institutional deployment."

---

## 🎯 PRESENTATION FLOW (Total: 7-10 minutes)

**Slide 1 (1 min):** Project identity and scope
**Slide 2 (1.5 min):** Problem statement - hook the audience
**Slide 3 (2 min):** Current tech landscape - establish context
**Slide 4 (1.5 min):** Your approach overview - architecture
**Slide 5 (1.5 min):** Key innovations - what makes you unique
**Slide 6 (1.5 min):** Technical details - prove capability
**Slide 7 (1 min):** Impact and future - inspire action

---

## 💡 PRESENTATION TIPS

### Opening (Strong Hook):
*"Nurses walk an average of 5 miles per shift. 40% of their time is spent on tasks that don't involve patient care. CareBot changes that."*

### Transitions:
- Slide 1→2: "But why do we need this?"
- Slide 2→3: "Others have tried to solve this..."
- Slide 3→4: "Here's how we're different..."
- Slide 4→5: "Let me highlight what makes CareBot unique..."
- Slide 5→6: "Now, the technical implementation..."
- Slide 6→7: "So what's the real-world impact?"

### Key Messages to Emphasize:
1. **Affordable**: <$2K vs $50K+ commercial solutions
2. **Accessible**: Web interface, no training required
3. **Open-source**: Customizable for specific hospital needs
4. **Proven tech**: ROS2, Nav2, SLAM Toolbox—industry standards
5. **Healthcare-focused**: Not just a robot, but a care assistant

### Questions to Anticipate:
- **"How does it compare to commercial solutions?"** → Cost, customization, accessibility
- **"What about safety?"** → Obstacle avoidance, emergency stop, mode validation
- **"Can it handle real hospital chaos?"** → Real-time obstacle avoidance, dynamic replanning
- **"How long to deploy in a new hospital?"** → ~2-3 hours to map facility

---

## 📸 ASSETS YOU'LL NEED

### Photos:
- [ ] CareBot full body showing LiDAR and camera
- [ ] RPLidar A1M8 sensor closeup
- [ ] Pi Camera Module V3
- [ ] Hospital corridor (stock photo OK)
- [ ] Healthcare workers (stock photo OK)

### Screenshots:
- [ ] Web interface in IDLE mode
- [ ] Web interface in MANUAL mode (showing map building)
- [ ] Web interface in AUTO mode (showing navigation)
- [ ] RViz showing LiDAR scan data
- [ ] RViz showing SLAM map building
- [ ] RViz showing Nav2 path planning

### Diagrams:
- [ ] System architecture (3-layer diagram)
- [ ] ROS2 node graph (showing topic connections)
- [ ] Mode transition flow diagram

### Logos:
- [ ] ROS2 logo (download from ros.org)
- [ ] RMIT logo
- [ ] UN SDG logos (SDG 3 and 9)

---

## 🎨 DESIGN CONSISTENCY

**Color Palette (use throughout):**
- Deep Blue `#1A3A6B` - Technology, primary
- Medical Teal `#00A8A8` - Healthcare theme
- Pink `#FF69B4` - CareBot branding
- Success Green `#4CAF50` - Working systems
- Purple `#9C27B0` - Sensors/special features
- Warning Orange `#FF9800` - Future/enhancements
- White `#FFFFFF` - Clean backgrounds

**Typography:**
- Titles: Calibri Bold, 40-44pt
- Subtitles: Calibri, 28-32pt
- Body: Calibri, 20-24pt
- Captions: Calibri, 16-18pt

**Icons:**
- Use consistent icon style (e.g., Font Awesome or Material Icons)
- Keep icon color scheme aligned with content

---

## 📝 SPEAKING NOTES CHECKLIST

Before each slide, remember:
- [ ] Make eye contact (look at camera if virtual)
- [ ] Speak slower than normal (nervous tendency to speed up)
- [ ] Use hand gestures to emphasize key points
- [ ] Pause after important statements
- [ ] Show enthusiasm (you built something cool!)
- [ ] Reference the visual ("As you can see here...")

Practice timing:
- [ ] Record yourself and check timing
- [ ] Aim for 8-10 minutes (leaves room for Q&A)
- [ ] Have 1-2 sentences you can cut if running long
- [ ] Have 1-2 extra details you can add if running short

---

## 🚀 READY TO BUILD YOUR SLIDES?

Use this structure to create your PowerPoint/Google Slides deck. Focus on:
1. **Visual clarity** - Less text, more images and diagrams
2. **Storytelling** - Problem → Context → Solution → Impact
3. **Technical credibility** - Show you understand the tech deeply
4. **Practical value** - Emphasize real-world healthcare benefits

Good luck with your presentation! 🎤
