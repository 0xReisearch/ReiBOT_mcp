// Robot visualization JavaScript - View Only Mode
document.addEventListener('DOMContentLoaded', function() {
    // Socket.io connection
    const socket = io();
    const connectionStatus = document.getElementById('connection-status');
    const modeIndicator = document.getElementById('mode-indicator');
    const robotStatus = document.getElementById('robot-status');
    const batteryLevel = document.getElementById('battery-level');
    const batteryFill = document.getElementById('battery-fill');
    const positionValue = document.getElementById('position-value');
    const orientationValue = document.getElementById('orientation-value');
    const eventLog = document.getElementById('event-log');
    const commandsLog = document.getElementById('commands-log');
    
    // Three.js visualization
    let scene, camera, renderer, robot;
    
    // Initialize 3D visualization
    function initVisualization() {
        console.log("Initializing 3D visualization...");
        
        // Create scene
        scene = new THREE.Scene();
        scene.background = new THREE.Color(0x222222);
        
        // Create camera
        camera = new THREE.PerspectiveCamera(75, 800 / 400, 0.1, 1000);
        camera.position.set(8, 8, 8);
        camera.lookAt(0, 0, 0);
        
        // Create renderer
        renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(800, 400);
        renderer.shadowMap.enabled = true;
        renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        
        const canvas = document.getElementById('robot-canvas');
        canvas.innerHTML = ''; // Clear any existing content
        canvas.appendChild(renderer.domElement);
        
        // Add lighting
        const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
        scene.add(ambientLight);
        
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(10, 10, 5);
        directionalLight.castShadow = true;
        scene.add(directionalLight);
        
        // Add grid
        const gridHelper = new THREE.GridHelper(20, 20, 0x888888, 0x444444);
        scene.add(gridHelper);
        
        // Add axes
        const axesHelper = new THREE.AxesHelper(3);
        scene.add(axesHelper);
        
        // Create robot representation - make it bigger and more visible
        const robotGeometry = new THREE.BoxGeometry(2, 1, 3);
        const robotMaterial = new THREE.MeshLambertMaterial({ color: 0x3498db });
        robot = new THREE.Mesh(robotGeometry, robotMaterial);
        robot.position.set(0, 0.5, 0); // Lift it slightly off the ground
        robot.castShadow = true;
        scene.add(robot);
        
        // Add direction indicator (front of robot) - make it more visible
        const directionGeometry = new THREE.ConeGeometry(0.5, 1, 8);
        const directionMaterial = new THREE.MeshLambertMaterial({ color: 0xe74c3c });
        const direction = new THREE.Mesh(directionGeometry, directionMaterial);
        direction.position.set(0, 0.5, 1.8); // Position at front of robot
        direction.rotation.x = Math.PI / 2;
        robot.add(direction);
        
        console.log("3D visualization initialized successfully");
        
        // Start animation loop
        animate();
    }
    
    // Animation loop
    function animate() {
        requestAnimationFrame(animate);
        
        // Rotate camera around the scene for better view
        const time = Date.now() * 0.0005;
        camera.position.x = Math.cos(time) * 12;
        camera.position.z = Math.sin(time) * 12;
        camera.lookAt(robot.position);
        
        renderer.render(scene, camera);
    }
    
    // Update robot position and orientation in 3D visualization
    function updateRobotVisualization(position, orientation) {
        if (!robot) {
            console.log("Robot object not initialized yet");
            return;
        }
        
        console.log("Updating robot position:", position);
        console.log("Updating robot orientation:", orientation);
        
        // Update position - scale it up for better visibility
        robot.position.set(position.x * 2, 0.5, position.y * 2);
        
        // Update orientation (convert to radians)
        robot.rotation.set(
            orientation.roll * Math.PI / 180,
            orientation.yaw * Math.PI / 180,
            orientation.pitch * Math.PI / 180
        );
        
        console.log("Robot updated to position:", robot.position);
    }
    
    // Add log entry
    function addLogEntry(message) {
        const entry = document.createElement('div');
        entry.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
        eventLog.appendChild(entry);
        eventLog.scrollTop = eventLog.scrollHeight;
        
        // Also log to console for debugging
        console.log(message);
    }
    
    // Add command entry
    function addCommandEntry(command, details) {
        const entry = document.createElement('div');
        entry.className = 'command-entry';
        entry.innerHTML = `
            <span class="command-time">[${new Date().toLocaleTimeString()}]</span>
            <span class="command-type">${command}</span>
            <span class="command-details">${details}</span>
        `;
        commandsLog.appendChild(entry);
        commandsLog.scrollTop = commandsLog.scrollHeight;
        
        // Keep only last 50 commands
        while (commandsLog.children.length > 50) {
            commandsLog.removeChild(commandsLog.firstChild);
        }
    }
    
    // Socket.io event handlers
    socket.on('connect', function() {
        connectionStatus.textContent = 'Connected';
        connectionStatus.classList.add('connected');
        addLogEntry('Connected to server');
    });
    
    socket.on('disconnect', function() {
        connectionStatus.textContent = 'Disconnected';
        connectionStatus.classList.remove('connected');
        addLogEntry('Disconnected from server');
    });
    
    socket.on('robot_state_update', function(state) {
        console.log("Received robot state update:", state);
        
        // Update UI with robot state
        robotStatus.textContent = state.status.charAt(0).toUpperCase() + state.status.slice(1);
        batteryLevel.textContent = `${state.battery.toFixed(1)}%`;
        batteryFill.style.width = `${state.battery}%`;
        
        // Update position display
        positionValue.textContent = `X: ${state.position.x.toFixed(2)}, Y: ${state.position.y.toFixed(2)}, Z: ${state.position.z.toFixed(2)}`;
        
        // Update orientation display
        orientationValue.textContent = `Roll: ${state.orientation.roll.toFixed(2)}, Pitch: ${state.orientation.pitch.toFixed(2)}, Yaw: ${state.orientation.yaw.toFixed(2)}`;
        
        // Update mode indicator
        modeIndicator.textContent = `Mode: ${state.mode.charAt(0).toUpperCase() + state.mode.slice(1)}`;
        
        // Update 3D visualization
        updateRobotVisualization(state.position, state.orientation);
        
        // Log state update
        addLogEntry(`State updated: ${state.status} at (${state.position.x.toFixed(2)}, ${state.position.y.toFixed(2)})`);
        
        // Log command received if status indicates movement
        if (state.status === 'moving') {
            addCommandEntry('MOVE', `Position: (${state.position.x.toFixed(2)}, ${state.position.y.toFixed(2)}, ${state.position.z.toFixed(2)})`);
        } else if (state.status === 'rotating') {
            addCommandEntry('ROTATE', `Orientation: (${state.orientation.roll.toFixed(2)}, ${state.orientation.pitch.toFixed(2)}, ${state.orientation.yaw.toFixed(2)})`);
        }
    });
    
    socket.on('command_sent', function(data) {
        addLogEntry(`Command sent: ${data.command}`);
        
        // Add to commands log with details
        if (data.command === 'move' && data.position) {
            addCommandEntry('MOVE COMMAND', `X: ${data.position.x}, Y: ${data.position.y}, Z: ${data.position.z}`);
        } else if (data.command === 'rotate' && data.orientation) {
            addCommandEntry('ROTATE COMMAND', `Roll: ${data.orientation.roll}, Pitch: ${data.orientation.pitch}, Yaw: ${data.orientation.yaw}`);
        } else if (data.command === 'stop') {
            addCommandEntry('STOP COMMAND', 'All movement stopped');
        }
    });
    
    socket.on('error', function(data) {
        addLogEntry(`Error: ${data.message}`);
    });
    
    // Initialize visualization
    console.log("Starting visualization initialization...");
    initVisualization();
});
