// CareBot Controller Logic

// 1. ROS Connection
var ros = new ROSLIB.Ros({
    url: 'ws://' + window.location.hostname + ':9090'
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
    name: '/cmd_vel',
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

// ==========================
// 7. Navigation & Map Logic
// ==========================
var viewer = null;
var gridClient = null;

function initMap() {
    if (viewer) return; // Already init

    // Create the main viewer.
    viewer = new ROS2D.Viewer({
        divID: 'nav-map',
        width: 400,
        height: 400
    });

    // Setup the map client.
    gridClient = new ROS2D.OccupancyGridClient({
        ros: ros,
        rootObject: viewer.scene,
        continuous: true // Track map updates
    });

    // Scale the viewer to fit the map
    gridClient.on('change', function () {
        viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
        // Shift map so it's centered
        viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
    });

    // Setup Goal Publisher
    var goalTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/goal_pose',
        messageType: 'geometry_msgs/PoseStamped'
    });

    // Add Click Handler
    viewer.scene.addEventListener('stagemousedown', function (event) {
        if (!document.getElementById('mode-switch').checked) return;

        var coords = viewer.scene.globalToLocal(event.stageX, event.stageY);
        // coords are in meters in the map frame

        var pose = new ROSLIB.Message({
            header: { frame_id: "map", stamp: { sec: 0, nanosec: 0 } },
            pose: {
                position: { x: coords.x, y: coords.y, z: 0.0 },
                orientation: { x: 0, y: 0, z: 0, w: 1.0 }
            }
        });

        console.log("Creating Navigation Goal:", coords.x, coords.y);
        goalTopic.publish(pose);
    });
}

// Mode Switching
const modeSwitch = document.getElementById('mode-switch');
const manualPanel = document.getElementById('manual-panel');
const mapPanel = document.getElementById('map-panel');

if (modeSwitch) {
    modeSwitch.addEventListener('change', (e) => {
        if (e.target.checked) {
            // Auto Mode
            console.log("Switched to Auto Mode");
            manualPanel.classList.add('hidden');
            mapPanel.classList.remove('hidden');
            initMap(); // Init map only when needed to save resources
        } else {
            // Manual Mode
            console.log("Switched to Manual Mode");
            manualPanel.classList.remove('hidden');
            mapPanel.classList.add('hidden');
        }
    });
}
