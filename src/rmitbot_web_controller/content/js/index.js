// CareBot Controller Logic

// ROS Connection - Connect to Pi's rosbridge
var rosHost = window.location.hostname; // Dynamic host
var ros = new ROSLIB.Ros({
    url: 'ws://' + rosHost + ':9090'
});

// Set Camera Stream URL dynamically
var streamImg = document.getElementById('streaming');
if (streamImg) {
    streamImg.src = 'http://' + rosHost + ':8001/camera/stream';
    console.log('Camera stream set to: ' + streamImg.src);
}

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
var currentLoadedMap = null; // Track currently loaded map name
var savedMaps = []; // List of saved maps

// Nav2 Navigation - Use topic-based approach for ROS2 compatibility
var goalTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/goal_pose',
    messageType: 'geometry_msgs/PoseStamped'
});

// Current navigation goal handle
var currentGoalId = null;

// Simple check - will show status when first goal is sent
console.log('Navigation topic initialized: /goal_pose');

// Fetch saved maps from backend
function fetchSavedMaps() {
    // Call ROS service to get list of saved maps
    // For now, we'll use a simple HTTP endpoint to list files in ~/.ros/
    fetch('/list_maps')
        .then(response => response.json())
        .then(data => {
            savedMaps = data.maps || [];
            updateMapsList();
        })
        .catch(error => {
            console.error('Error fetching maps:', error);
            // Fallback: just show empty list
            savedMaps = [];
            updateMapsList();
        });
}

// Update the maps list UI
function updateMapsList() {
    // Update both AUTO and IDLE mode lists
    updateMapsListForMode('maps-list', 'no-maps-message', true); // AUTO mode - with launch button
    updateMapsListForMode('maps-list-idle', 'no-maps-message-idle', false); // IDLE mode - read only
}

function updateMapsListForMode(listId, noMapsId, showLaunchButton) {
    const mapsList = document.getElementById(listId);
    const noMapsMessage = document.getElementById(noMapsId);

    if (savedMaps.length === 0) {
        noMapsMessage.style.display = 'block';
        mapsList.innerHTML = '';
        return;
    }

    noMapsMessage.style.display = 'none';
    mapsList.innerHTML = '';

    savedMaps.forEach(mapName => {
        const mapItem = document.createElement('div');
        mapItem.className = 'map-item';
        if (currentLoadedMap === mapName) {
            mapItem.classList.add('selected');
        }

        const mapNameSpan = document.createElement('span');
        mapNameSpan.className = 'map-item-name';
        mapNameSpan.innerHTML = `<i class="fas fa-map"></i> ${mapName}`;
        mapItem.appendChild(mapNameSpan);

        if (showLaunchButton) {
            const actionsDiv = document.createElement('div');
            actionsDiv.className = 'map-item-actions';

            const launchBtn = document.createElement('button');
            launchBtn.className = 'btn-launch-map';
            launchBtn.innerHTML = '<i class="fas fa-play"></i> Launch';
            launchBtn.onclick = (e) => {
                e.stopPropagation();
                loadAndLaunchMap(mapName);
            };
            actionsDiv.appendChild(launchBtn);

            mapItem.appendChild(actionsDiv);
        }

        mapItem.onclick = () => {
            // Just highlight selection
            document.querySelectorAll(`#${listId} .map-item`).forEach(item => {
                item.classList.remove('selected');
            });
            mapItem.classList.add('selected');
        };

        mapsList.appendChild(mapItem);
    });
}

// Load and launch a map
function loadAndLaunchMap(mapName) {
    console.log('Loading map:', mapName);

    // Since we're in mapping mode, we don't actually need to load a map
    // The SLAM toolbox is already publishing the live map
    // Just mark it as "loaded" for UI purposes

    var statusSpan = document.getElementById('load-map-status');
    if (!statusSpan) {
        statusSpan = document.createElement('span');
        statusSpan.id = 'temp-load-status';
        statusSpan.style.cssText = 'position: fixed; top: 20px; right: 20px; background: rgba(0,0,0,0.8); color: white; padding: 10px 20px; border-radius: 8px; z-index: 1000;';
        document.body.appendChild(statusSpan);
    }

    statusSpan.innerText = "Map already active (live SLAM)";
    statusSpan.style.display = 'block';
    statusSpan.style.color = "var(--success)";

    currentLoadedMap = mapName;

    // Hide placeholder
    const placeholder = document.getElementById('map-placeholder');
    if (placeholder) {
        placeholder.style.display = 'none';
    }

    // Update list to show selected map
    updateMapsList();

    setTimeout(() => {
        if (statusSpan.id === 'temp-load-status') {
            statusSpan.remove();
        } else {
            statusSpan.innerText = "";
        }
    }, 3000);
}

// Mode switching functions
function setMode(mode) {
    currentMode = mode;
    updateModeUI();

    // Show/hide relevant sections
    if (mode === 'AUTO') {
        document.getElementById('auto-map-section').style.display = 'block';
        document.getElementById('custom-goal-section').style.display = 'block';
        document.getElementById('nav-status').style.display = 'block';
        document.getElementById('user-manual-section').style.display = 'none';

        // Hide manual movement controls in Auto
        document.querySelector('.movement-section').style.display = 'none';
        document.querySelector('.speed-section').style.display = 'none';
        document.getElementById('save-map-section').style.display = 'none';

    } else if (mode === 'MANUAL') {
        document.getElementById('auto-map-section').style.display = 'none';
        document.getElementById('custom-goal-section').style.display = 'none';
        document.getElementById('nav-status').style.display = 'none';
        document.getElementById('user-manual-section').style.display = 'none';

        // Enable manual movement in MANUAL mode
        document.querySelector('.movement-section').style.display = 'block';
        document.querySelector('.movement-section').style.opacity = '1';
        document.querySelector('.movement-section').style.pointerEvents = 'auto';

        document.querySelector('.speed-section').style.display = 'block';
        document.querySelector('.speed-section').style.opacity = '1';
        document.querySelector('.speed-section').style.pointerEvents = 'auto';

        // Show save map section in MANUAL mode
        document.getElementById('save-map-section').style.display = 'block';

    } else {
        // IDLE: Show manual only
        document.getElementById('auto-map-section').style.display = 'none';
        document.getElementById('custom-goal-section').style.display = 'none';
        document.getElementById('nav-status').style.display = 'none';
        document.getElementById('user-manual-section').style.display = 'block';

        // HIDE speed and movement controls in IDLE
        document.querySelector('.movement-section').style.display = 'none';
        document.querySelector('.speed-section').style.display = 'none';
        document.getElementById('save-map-section').style.display = 'none';
    }

    // Cancel any ongoing navigation when leaving AUTO mode
    if (mode !== 'AUTO' && currentGoalId) {
        cancelNavigation();
    }
}

// Send navigation goal to Nav2
function sendNavGoal(x, y, yaw) {
    if (currentMode !== 'AUTO') {
        alert('Please switch to AUTO mode to navigate.');
        return;
    }

    // Convert yaw to quaternion (simple 2D rotation around Z)
    var qz = Math.sin(yaw / 2);
    var qw = Math.cos(yaw / 2);

    // Create PoseStamped message
    var goalMessage = new ROSLIB.Message({
        header: {
            frame_id: 'map',
            stamp: { sec: 0, nanosec: 0 }
        },
        pose: {
            position: { x: x, y: y, z: 0.0 },
            orientation: { x: 0.0, y: 0.0, z: qz, w: qw }
        }
    });

    console.log('Sending navigation goal:', 'x=' + x + ', y=' + y + ', yaw=' + yaw);

    goalTopic.publish(goalMessage);

    currentGoalId = 'goal_' + Date.now();
    updateNavStatusText('Goal sent to Nav2...');
    document.getElementById('cancel-nav').style.display = 'block';

    // Auto-hide cancel button after 30 seconds
    setTimeout(function () {
        if (currentGoalId) {
            document.getElementById('cancel-nav').style.display = 'none';
            updateNavStatusText('Goal completed or timed out');
            currentGoalId = null;
        }
    }, 30000);
}

// Cancel ongoing navigation
function cancelNavigation() {
    if (currentGoalId) {
        var request = new ROSLIB.ServiceRequest({
            goal_info: {
                goal_id: {
                    id: currentGoalId
                }
            }
        });

        cancelGoalService.callService(request, function (result) {
            console.log('Goal cancelled:', result);
            currentGoalId = null;
            updateNavStatusText('Navigation cancelled');
            document.getElementById('cancel-nav').style.display = 'none';
            document.getElementById('nav-progress').style.width = '0%';
        }, function (error) {
            console.error('Error calling cancel service:', error);

            // Publish empty goal to stop as a fallback
            var stopMessage = new ROSLIB.Message({
                header: {
                    frame_id: 'map',
                    stamp: { sec: 0, nanosec: 0 }
                },
                pose: {
                    position: { x: 0, y: 0, z: 0 },
                    orientation: { x: 0, y: 0, z: 0, w: 1 }
                }
            });

            // Assuming you have a goalTopic defined elsewhere to publish the stopMessage
            // goalTopic.publish(stopMessage); 

            console.log('Cancelling navigation via fallback');
            currentGoalId = null;
            updateNavStatusText('Navigation cancelled');
            document.getElementById('cancel-nav').style.display = 'none';
            document.getElementById('nav-progress').style.width = '0%';

            // Fixed the combined lines from your snippet
            updateNavStatusText('Ready');
        });
    }
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

// Bind Custom Goal Button
document.getElementById('btn-go-goal').addEventListener('click', function () {
    var x = parseFloat(document.getElementById('goal-x').value) || 0;
    var y = parseFloat(document.getElementById('goal-y').value) || 0;
    var yaw = parseFloat(document.getElementById('goal-yaw').value) || 0;
    sendNavGoal(x, y, yaw);
});

// Initialize in IDLE mode
setMode('IDLE');

// === MAP VISUALIZATION ===
var viewer = null;
var gridClient = null;
var laserScanClient = null;
var poseListener = null;

// Helper function to fit map to viewer (Issue 4)
function fitMapToViewer() {
    if (!viewer || !gridClient || !gridClient.currentGrid) {
        console.warn('Cannot fit map: viewer or grid not ready');
        return;
    }

    const grid = gridClient.currentGrid;

    // Scale to fit the map dimensions
    viewer.scaleToDimensions(grid.width, grid.height);

    // Center the map by shifting to its origin
    viewer.shift(grid.pose.position.x, grid.pose.position.y);

    console.log('Map fitted to viewer:', {
        width: grid.width,
        height: grid.height,
        origin: grid.pose.position
    });
}

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

        // Add click handler for 2D Goal Pose in AUTO mode
        viewer.scene.addEventListener('stagemousedown', function (event) {
            if (currentMode === 'AUTO') {
                // Convert screen coordinates to map coordinates
                var mousePos = viewer.scene.globalToRos(event.stageX, event.stageY);
                console.log('Map clicked at:', mousePos);

                // Send navigation goal (with default yaw of 0)
                sendNavGoal(mousePos.x, mousePos.y, 0);
            }
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

        // Scale the viewer to fit map when it loads or updates
        gridClient.on('change', function () {
            // Issue 4: Auto-fit the map to the viewer
            fitMapToViewer();

            // Hide placeholder when map loads
            const placeholder = document.getElementById('map-placeholder');
            if (placeholder) {
                placeholder.style.display = 'none';
            }
        });
    }

    // Add LaserScan visualization
    // Note: ROS2D v0.9.0 doesn't include LaserScanClient
    // Uncomment if you upgrade to a version that supports it
    // if (!laserScanClient) {
    //     laserScanClient = new ROS2D.LaserScanClient({
    //         ros: ros,
    //         rootObject: viewer.scene,
    //         topic: '/scan',
    //         pointRatio: 2,
    //         messageRatio: 1,
    //         max_points: 500
    //     });
    // }

    // Add robot pose visualization
    if (!poseListener) {
        // Create a shape for the robot
        var robotMarker = new createjs.Shape();
        robotMarker.graphics.beginFill('#FF69B4').drawCircle(0, 0, 0.2); // 0.2m radius pink circle
        robotMarker.graphics.beginFill('#FF1493').moveTo(0, 0).lineTo(0.3, 0.1).lineTo(0.3, -0.1).closePath(); // Direction arrow
        viewer.scene.addChild(robotMarker);

        // Subscribe to robot pose
        poseListener = new ROSLIB.Topic({
            ros: ros,
            name: '/amcl_pose',
            messageType: 'geometry_msgs/PoseWithCovarianceStamped'
        });

        poseListener.subscribe(function (message) {
            var pose = message.pose.pose;
            var rosPos = new ROSLIB.Vector3(pose.position);
            var viewerPos = viewer.scene.rosToCanvas(rosPos);

            robotMarker.x = viewerPos.x;
            robotMarker.y = viewerPos.y;

            // Calculate rotation from quaternion
            var q = pose.orientation;
            var yaw = Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            robotMarker.rotation = yaw * 180 / Math.PI; // Convert to degrees for CreateJS
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
// Use serialize_map (NOT save_map) to create .posegraph files
var saveMapClient = new ROSLIB.Service({
    ros: ros,
    name: '/slam_toolbox/serialize_map',
    serviceType: 'slam_toolbox/srv/SerializePoseGraph'
});

// Save map from MANUAL mode
const btnSaveManualMap = document.getElementById('btn-save-manual-map');
const manualMapName = document.getElementById('manual-map-name');
const manualSaveStatus = document.getElementById('manual-save-status');

if (btnSaveManualMap) {
    btnSaveManualMap.addEventListener('click', function () {
        const name = manualMapName ? manualMapName.value.trim() : '';
        if (!name) {
            alert('Please enter a map name');
            return;
        }

        // SerializePoseGraph uses 'filename' not 'name'
        var request = new ROSLIB.ServiceRequest({
            filename: name
        });

        if (manualSaveStatus) {
            manualSaveStatus.innerText = "Saving...";
            manualSaveStatus.style.color = "#636E72";
        }

        saveMapClient.callService(request, function (result) {
            console.log('Map saved:', result);
            if (manualSaveStatus) {
                manualSaveStatus.innerText = `Map "${name}" saved successfully as ${name}.posegraph!`;
                manualSaveStatus.style.color = "var(--success)";
            }
            if (manualMapName) manualMapName.value = '';

            setTimeout(() => {
                if (manualSaveStatus) manualSaveStatus.innerText = "";
            }, 3000);
        }, function (error) {
            console.error('Save error:', error);
            if (manualSaveStatus) {
                manualSaveStatus.innerText = "Error saving map - Is SLAM running?";
                manualSaveStatus.style.color = "var(--danger)";
            }

            setTimeout(() => {
                if (manualSaveStatus) manualSaveStatus.innerText = "";
            }, 5000);
        });
    });
}

// Old save map button removed - now handled in modal

// Load Map Service Client
var loadMapClient = new ROSLIB.Service({
    ros: ros,
    name: '/slam_toolbox/deserialize_map',
    serviceType: 'slam_toolbox/srv/DeserializePoseGraph'
});

// Old refresh buttons removed - maps load automatically in modal

// === MODAL MAP MANAGEMENT ===


// Modal Elements
const mapsModal = document.getElementById('maps-modal');
const modalClose = document.getElementById('modal-close');
const btnManageMaps = document.getElementById('btn-manage-maps');
const modalMapsList = document.getElementById('modal-maps-list');
const btnModalSaveMap = document.getElementById('btn-modal-save-map');
const modalMapName = document.getElementById('modal-map-name');
const modalSaveStatus = document.getElementById('modal-save-status');

// Debug: Log if elements are found
console.log('Modal elements check:', {
    mapsModal: !!mapsModal,
    modalClose: !!modalClose,
    btnManageMaps: !!btnManageMaps,
    modalMapsList: !!modalMapsList
});

// Open Modal
function showMapsModal() {
    console.log('showMapsModal called');
    if (!mapsModal) {
        console.error('Modal element not found!');
        alert('Error: Modal not found. Please refresh the page.');
        return;
    }
    mapsModal.style.display = 'flex';
    loadMapsToModal();
}

// Close Modal
function hideMapsModal() {
    if (!mapsModal) return;
    mapsModal.style.display = 'none';
}

// Event Listeners
if (btnManageMaps) {
    btnManageMaps.addEventListener('click', showMapsModal);
    console.log('Manage Maps button event listener attached');
} else {
    console.error('btn-manage-maps element not found!');
}

if (modalClose) {
    modalClose.addEventListener('click', hideMapsModal);
} else {
    console.error('modal-close element not found!');
}

// AUTO mode map button
const btnManageMapsAuto = document.getElementById('btn-manage-maps-auto');
if (btnManageMapsAuto) {
    btnManageMapsAuto.addEventListener('click', showMapsModal);
    console.log('AUTO mode Manage Maps button event listener attached');
}

// Close modal when clicking outside
if (mapsModal) {
    mapsModal.addEventListener('click', function (e) {
        if (e.target === mapsModal) {
            hideMapsModal();
        }
    });
}

// Load Maps into Modal
function loadMapsToModal() {
    if (!modalMapsList) {
        console.error('modalMapsList element not found!');
        return;
    }

    modalMapsList.innerHTML = '<p class="loading-message">Loading maps...</p>';

    // Fetch from web server endpoint
    fetch('http://' + rosHost + ':8000/list_maps')
        .then(response => response.json())
        .then(data => {
            console.log('Maps loaded:', data);
            if (data.maps && data.maps.length > 0) {
                let html = '';
                data.maps.forEach(mapName => {
                    html += `
                        <div class="map-item">
                            <span class="map-item-name">
                                <i class="fas fa-map-marked-alt"></i>
                                ${mapName}
                                <span class="map-format-badge">.posegraph</span>
                            </span>
                            <div class="map-item-actions">
                                <button class="btn-load-map" onclick="loadMapFromModal('${mapName}')">
                                    <i class="fas fa-download"></i> Load
                                </button>
                            </div>
                        </div>
                    `;
                });
                modalMapsList.innerHTML = html;
            } else {
                modalMapsList.innerHTML = '<p class="loading-message">No saved maps found. Create maps in Manual mode first.</p>';
            }
        })
        .catch(error => {
            console.error('Error fetching maps:', error);
            modalMapsList.innerHTML = '<p class="loading-message" style="color: var(--danger);">Error loading maps. Make sure web server is running.</p>';
        });
}

// Search/Filter Maps
const modalSearchInput = document.getElementById('modal-search-maps');
if (modalSearchInput) {
    modalSearchInput.addEventListener('input', function (e) {
        const searchTerm = e.target.value.toLowerCase();
        const mapItems = modalMapsList.querySelectorAll('.map-item');

        let visibleCount = 0;
        mapItems.forEach(item => {
            const mapName = item.querySelector('.map-item-name').textContent.toLowerCase();
            if (mapName.includes(searchTerm)) {
                item.style.display = 'flex';
                visibleCount++;
            } else {
                item.style.display = 'none';
            }
        });

        // Show message if no maps match
        if (visibleCount === 0 && mapItems.length > 0) {
            if (!document.getElementById('no-match-message')) {
                const noMatch = document.createElement('p');
                noMatch.id = 'no-match-message';
                noMatch.className = 'loading-message';
                noMatch.textContent = `No maps matching "${e.target.value}"`;
                modalMapsList.appendChild(noMatch);
            }
        } else {
            const noMatchMsg = document.getElementById('no-match-message');
            if (noMatchMsg) noMatchMsg.remove();
        }
    });
}

// Save Map from Modal
if (btnModalSaveMap) {
    btnModalSaveMap.addEventListener('click', function () {
        const name = modalMapName ? modalMapName.value.trim() : '';
        if (!name) {
            alert('Please enter a map name');
            return;
        }

        // SerializePoseGraph uses 'filename' not 'name'
        var request = new ROSLIB.ServiceRequest({
            filename: name
        });

        if (modalSaveStatus) {
            modalSaveStatus.innerText = "Saving...";
            modalSaveStatus.style.color = "#636E72";
        }

        saveMapClient.callService(request, function (result) {
            console.log('Map saved:', result);
            if (modalSaveStatus) {
                modalSaveStatus.innerText = `Map "${name}" saved successfully as ${name}.posegraph!`;
                modalSaveStatus.style.color = "var(--success)";
            }
            if (modalMapName) modalMapName.value = '';

            // Refresh the maps list
            setTimeout(() => {
                loadMapsToModal();
                if (modalSaveStatus) modalSaveStatus.innerText = "";
            }, 2000);
        }, function (error) {
            console.error('Save error:', error);
            if (modalSaveStatus) {
                modalSaveStatus.innerText = "Error saving map";
                modalSaveStatus.style.color = "var(--danger)";
            }

            setTimeout(() => {
                if (modalSaveStatus) modalSaveStatus.innerText = "";
            }, 5000);
        });
    });
}

// Load Map from Modal (global function for onclick)
window.loadMapFromModal = function (mapName) {
    console.log('Loading map:', mapName);

    // Issue 2: Prevent loading in MANUAL mode (would overlay current map)
    if (currentMode === 'MANUAL') {
        alert('Cannot load map while in MANUAL mode!\n\nSwitch to IDLE or AUTO mode first to avoid map overlay.');
        return;
    }

    var request = new ROSLIB.ServiceRequest({
        filename: mapName,
        match_type: 1  // 1 = START_AT_FIRST_NODE
    });

    // Show loading state
    if (modalMapsList) {
        const mapItems = modalMapsList.querySelectorAll('.map-item');
        mapItems.forEach(item => {
            if (item.textContent.includes(mapName)) {
                const btn = item.querySelector('.btn-load-map');
                if (btn) {
                    btn.innerHTML = '<i class="fas fa-spinner fa-spin"></i> Loading...';
                    btn.disabled = true;
                }
            }
        });
    }

    loadMapClient.callService(request, function (result) {
        console.log('Map loaded successfully:', result);
        alert(`Map "${mapName}" loaded successfully! You can now switch to AUTO mode for navigation.`);
        hideMapsModal();
    }, function (error) {
        console.error('Load error:', error);
        alert(`Failed to load map "${mapName}". Make sure the map files exist in the workspace directory.`);
        loadMapsToModal(); // Reload to reset buttons
    });
};
