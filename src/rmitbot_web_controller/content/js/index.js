// CareBot Controller Logic

// ROS Connection - Connect to Pi's rosbridge
var rosHost = '172.20.10.4'; // Pi IP address
var ros = new ROSLIB.Ros({
    url: 'ws://' + rosHost + ':9090'
});

// UI Elements
const statusDot = document.getElementById('status-dot');
const statusText = document.getElementById('connection-text');
const speedSlider = document.getElementById('speed-slider');
const speedDisplay = document.getElementById('speed-display');

// Connection Handlers
ros.on('connection', function () {
    console.log('Connected to websocket server.');
    statusDot.className = 'status-dot connected';
    statusText.innerText = 'Online';
});

ros.on('error', function (error) {
    console.log('Error connecting: ', error);
    statusDot.className = 'status-dot disconnected';
    statusText.innerText = 'Error';
});

ros.on('close', function () {
    console.log('Connection closed.');
    statusDot.className = 'status-dot disconnected';
    statusText.innerText = 'Offline';
});

// 2. Publisher Setup
var cmdVel = new ROSLIB.Topic({
    ros: ros,
    name: 'cmd_vel_keyboard',
    messageType: 'geometry_msgs/TwistStamped'
});

// State
var maxLinearSpeed = 1.0; // Max speed at 100% slider
var currentLinearScale = 0.5; // Default 50%
var currentAngularSpeed = 1.0;

// 3. Speed Slider Logic
speedSlider.addEventListener('input', function (e) {
    var pct = parseInt(e.target.value);
    // Map 0-100 to 0.0 - 1.0 (or whatever max speed)
    currentLinearScale = (pct / 100.0) * maxLinearSpeed;
    speedDisplay.innerText = currentLinearScale.toFixed(2);
});

// 4. Movement Logic
function publishTwist(lx, ly, az) {
    // timestamp could be 0, or current time
    var twist = new ROSLIB.Message({
        header: {
            frame_id: "base_link",
            stamp: { sec: 0, nanosec: 0 }
        },
        twist: {
            linear: {
                x: lx * currentLinearScale,
                y: ly * currentLinearScale,
                z: 0.0
            },
            angular: {
                x: 0.0,
                y: 0.0,
                z: az * currentAngularSpeed
            }
        }
    });
    cmdVel.publish(twist);
    console.log("Cmd:", lx, ly, az);
}

function stop() {
    publishTwist(0, 0, 0);
}

// 5. Button Bindings
var PUBLISH_RATE_MS = 100; // Send command every 100ms while held

function bindBtn(id, lx, ly, az) {
    const btn = document.getElementById(id);
    if (!btn) return;

    let intervalId = null;

    // Mouse/Touch Down - Start continuous publishing
    const start = (e) => {
        e.preventDefault(); // Prevent scrolling on mobile
        btn.classList.add('active');

        // Send immediately
        publishTwist(lx, ly, az);

        // Then send continuously
        intervalId = setInterval(() => {
            publishTwist(lx, ly, az);
        }, PUBLISH_RATE_MS);
    };

    // Release - Stop publishing
    const end = (e) => {
        e.preventDefault();
        btn.classList.remove('active');

        // Clear the interval
        if (intervalId) {
            clearInterval(intervalId);
            intervalId = null;
        }
        stop();
    };

    btn.addEventListener('mousedown', start);
    btn.addEventListener('mouseup', end);
    btn.addEventListener('mouseleave', end); // If mouse leaves button

    btn.addEventListener('touchstart', start);
    btn.addEventListener('touchend', end);
}

// Bind D-Pad (Cardinal)
bindBtn('forward', 1, 0, 0);
bindBtn('backward', -1, 0, 0);
bindBtn('left', 0, 1, 0);
bindBtn('right', 0, -1, 0);

// Bind D-Pad (Diagonal)
bindBtn('diag_for_left', 1, 1, 0);    // Forward-Left
bindBtn('diag_for_right', 1, -1, 0);  // Forward-Right
bindBtn('diag_back_left', -1, 1, 0);  // Backward-Left
bindBtn('diag_back_right', -1, -1, 0); // Backward-Right

// Rotate (Spin)
bindBtn('rotateL', 0, 0, 1);
bindBtn('rotateR', 0, 0, -1);

// Stop Button (Action on click)
const stopBtn = document.getElementById('stop');
if (stopBtn) {
    // E-Stop should probably be a latch or just send stop?
    // For now, standard click = stop.
    stopBtn.addEventListener('click', stop);
}

// 6. Keyboard Control
const keyMap = {
    'w': [1, 0, 0],
    's': [-1, 0, 0],
    'a': [0, 1, 0],
    'd': [0, -1, 0],
    'q': [0, 0, 1],
    'e': [0, 0, -1],
    ' ': [0, 0, 0] // Spacebar = Stop
};

const keysPressed = {};

document.addEventListener('keydown', (e) => {
    const key = e.key.toLowerCase();
    if (keyMap[key] && !keysPressed[key]) {
        keysPressed[key] = true;
        updateKeyDrive();
        // Visual feedback
        // Could enable .active class on buttons here
    }
});

document.addEventListener('keyup', (e) => {
    const key = e.key.toLowerCase();
    if (keysPressed[key]) {
        keysPressed[key] = false;
        updateKeyDrive();
    }
});

function updateKeyDrive() {
    let lx = 0, ly = 0, az = 0;

    if (keysPressed[' ']) { stop(); return; } // E-Stop priority

    if (keysPressed['w']) lx += 1;
    if (keysPressed['s']) lx -= 1;
    if (keysPressed['a']) ly += 1;
    if (keysPressed['d']) ly -= 1;
    if (keysPressed['q']) az += 1;
    if (keysPressed['e']) az -= 1;

    if (lx === 0 && ly === 0 && az === 0) {
        stop();
    } else {
        publishTwist(lx, ly, az);
    }
}

// === AUTOMATION ADDITIONS ===

// 7. Mode State Management
var currentMode = 'IDLE'; // 'IDLE', 'MANUAL', 'AUTO'

// Nav2 Action Client for navigate_to_pose
var navigateClient = new ROSLIB.ActionClient({
    ros: ros,
    serverName: '/navigate_to_pose',
    actionName: 'nav2_msgs/action/NavigateToPose',
    timeout: 10000
});

// Current navigation goal handle
var currentGoal = null;

// Mode switching functions
function setMode(mode) {
    currentMode = mode;
    updateModeUI();

    // Show/hide relevant sections
    if (mode === 'AUTO') {
        document.getElementById('waypoints-section').style.display = 'block';
        document.getElementById('nav-status').style.display = 'block';
        document.getElementById('mapping-section').style.display = 'none';

        // Disable manual movement controls visually
        document.querySelector('.movement-section').style.opacity = '0.5';
        document.querySelector('.movement-section').style.pointerEvents = 'none';

        // Also disable speed slider
        document.querySelector('.speed-section').style.opacity = '0.5';
        document.querySelector('.speed-section').style.pointerEvents = 'none';

    } else {
        document.getElementById('waypoints-section').style.display = 'none';
        document.getElementById('nav-status').style.display = 'none';

        // Enable manual movement in MANUAL mode
        if (mode === 'MANUAL') {
            document.querySelector('.movement-section').style.opacity = '1';
            document.querySelector('.movement-section').style.pointerEvents = 'auto';
            document.querySelector('.speed-section').style.opacity = '1';
            document.querySelector('.speed-section').style.pointerEvents = 'auto';

            // Show mapping tools in MANUAL mode
            document.getElementById('mapping-section').style.display = 'block';
        } else {
            // IDLE: Disable everything
            document.querySelector('.movement-section').style.opacity = '0.5';
            document.querySelector('.movement-section').style.pointerEvents = 'none';
            document.querySelector('.speed-section').style.opacity = '0.5';
            document.querySelector('.speed-section').style.pointerEvents = 'none';
            document.getElementById('mapping-section').style.display = 'none';
        }

        // Cancel any ongoing navigation when leaving AUTO mode
        if (currentGoal) {
            cancelNavigation();
        }
    }
}

// Send navigation goal to Nav2
function sendNavGoal(x, y, yaw) {
    if (currentMode !== 'AUTO') {
        console.warn('Not in AUTO mode, ignoring navigation goal');
        return;
    }

    // Cancel any existing goal first
    if (currentGoal) {
        currentGoal.cancel();
    }

    // Convert yaw to quaternion (simple 2D rotation around Z)
    var qz = Math.sin(yaw / 2);
    var qw = Math.cos(yaw / 2);

    var goal = new ROSLIB.Goal({
        actionClient: navigateClient,
        goalMessage: {
            pose: {
                header: {
                    frame_id: 'map',
                    stamp: { sec: 0, nanosec: 0 } // Use 0 for "now" in sim/real usually works if transform is available
                },
                pose: {
                    position: { x: x, y: y, z: 0.0 },
                    orientation: { x: 0.0, y: 0.0, z: qz, w: qw }
                }
            }
        }
    });

    goal.on('feedback', function (feedback) {
        updateNavProgress(feedback);
    });

    goal.on('result', function (result) {
        navComplete(result);
    });

    goal.send();
    currentGoal = goal;

    updateNavStatusText('Navigating...');
    document.getElementById('cancel-nav').style.display = 'block';
    console.log(`Sent goal: x=${x}, y=${y}, yaw=${yaw}`);
}

// Cancel ongoing navigation
function cancelNavigation() {
    if (currentGoal) {
        currentGoal.cancel();
        currentGoal = null;
        updateNavStatusText('Navigation cancelled');
        document.getElementById('cancel-nav').style.display = 'none';
        document.getElementById('nav-progress').style.width = '0%';
    }
}

// Navigation complete callback
function navComplete(result) {
    currentGoal = null;
    document.getElementById('cancel-nav').style.display = 'none';
    document.getElementById('nav-progress').style.width = '100%';
    updateNavStatusText('Goal reached!');

    setTimeout(() => {
        // Reset only if we haven't started a new goal
        if (!currentGoal) {
            document.getElementById('nav-progress').style.width = '0%';
            updateNavStatusText('Ready');
        }
    }, 3000);
}

// Update progress display
function updateNavProgress(feedback) {
    // NavigateToPose feedback contains distance_remaining
    if (feedback && feedback.distance_remaining !== undefined) {
        // Just a visual approximation: assume starting roughly < 5m away for 0-100%
        // A better way is to capture start distance, but this suffices for simple UI
        var dist = feedback.distance_remaining;
        var pct = Math.max(0, Math.min(100, (1 - (dist / 3.0)) * 100)); // Assume 3m max trip for progress bar

        // If really close, snap to 95%
        if (dist < 0.2) pct = 95;

        document.getElementById('nav-progress').style.width = pct + '%';
        updateNavStatusText(`Dist: ${dist.toFixed(2)}m`);
    }
}

// UI update helpers
function updateModeUI() {
    var modeVal = document.getElementById('mode-value');
    modeVal.innerText = currentMode;

    // Classes for badge
    modeVal.className = 'mode-value mode-' + currentMode.toLowerCase();

    // Toggle active buttons
    document.querySelectorAll('.btn-mode').forEach(btn => btn.classList.remove('active'));
    document.getElementById('mode-' + currentMode.toLowerCase()).classList.add('active');
}

function updateNavStatusText(text) {
    document.getElementById('nav-status-text').innerText = text;
}

// Event bindings for mode buttons
document.getElementById('mode-idle').addEventListener('click', () => setMode('IDLE'));
document.getElementById('mode-manual').addEventListener('click', () => setMode('MANUAL'));
document.getElementById('mode-auto').addEventListener('click', () => setMode('AUTO'));

// Event binding for cancel button
document.getElementById('cancel-nav').addEventListener('click', cancelNavigation);

// Bind waypoint buttons
document.querySelectorAll('.btn-waypoint').forEach(btn => {
    btn.addEventListener('click', function () {
        var x = parseFloat(this.dataset.x);
        var y = parseFloat(this.dataset.y);
        var yaw = parseFloat(this.dataset.yaw);
        sendNavGoal(x, y, yaw);
    });
});

// Initialize in IDLE mode
setMode('IDLE');

// === MAP VISUALIZATION ===
var viewer = null;
var gridClient = null;

function initMap() {
    // Create the viewer
    // We assume the map-canvas div is present
    // We get dimensions from the wrapper
    const mapDiv = document.getElementById('map-canvas');
    const width = mapDiv.clientWidth;
    const height = mapDiv.clientHeight;

    if (!viewer) {
        viewer = new ROS2D.Viewer({
            divID: 'map-canvas',
            width: width,
            height: height,
            background: '#efefef' // matches css background roughly
        });
    }

    // Setup the map client
    if (!gridClient) {
        gridClient = new ROS2D.OccupancyGridClient({
            ros: ros,
            rootObject: viewer.scene,
            continuous: true, // track map updates
            topic: '/map'
        });

        // Scale the viewer to fit map when it loads
        gridClient.on('change', function () {
            viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
            viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
            // Optionally zoom out a bit or center better
        });
    }
}

// Hook into connection
// We already have ros.on('connection', ...) at the top. 
// We can add another listener or modify the existing one. 
// Since we can have multiple listeners, adding one here is cleaner.
ros.on('connection', function () {
    setTimeout(initMap, 1000); // Small delay to ensure DOM is ready/stable
});


// === MAPPING TOOLS ===
var saveMapClient = new ROSLIB.Service({
    ros: ros,
    name: '/slam_toolbox/save_map',
    serviceType: 'slam_toolbox/srv/SaveMap'
});

document.getElementById('btn-save-map').addEventListener('click', function () {
    var name = document.getElementById('map-name').value;
    if (!name) {
        alert('Please enter a map name');
        return;
    }

    var request = new ROSLIB.ServiceRequest({
        name: { data: name }
    });

    var statusSpan = document.getElementById('save-map-status');
    statusSpan.innerText = "Saving...";
    statusSpan.style.color = "#636E72";

    saveMapClient.callService(request, function (result) {
        console.log('Result for service call on ' + saveMapClient.name + ': ' + result);
        statusSpan.innerText = "Map Saved Successfully!";
        statusSpan.style.color = "var(--success)";

        setTimeout(() => {
            statusSpan.innerText = "";
        }, 3000);
    }, function (error) {
        console.error(error);
        statusSpan.innerText = "Error Saving Map";
        statusSpan.style.color = "var(--danger)";
    });
});


