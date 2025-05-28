// Robot visualization JavaScript - 6-DOF Robot Arm
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
    let scene, camera, renderer, robotArm;
    let joints = []; // Array to store robot arm joints
    
    // Robot arm configuration (6-DOF)
    const armConfig = {
        base: { height: 0.5, radius: 0.8 },
        joint1: { height: 1.5, radius: 0.3 },
        joint2: { height: 1.2, radius: 0.25 },
        joint3: { height: 0.8, radius: 0.2 },
        joint4: { height: 0.6, radius: 0.15 },
        joint5: { height: 0.4, radius: 0.12 },
        endEffector: { height: 0.3, radius: 0.1 }
    };
    
    // Current joint angles (in degrees) and target angles for smooth animation
    let jointAngles = [0, 0, 0, 0, 0, 0];
    let targetJointAngles = [0, 0, 0, 0, 0, 0];
    let animationSpeed = 0.1; // How fast joints move to target (0.1 = 10% per frame)
    
    // Initialize 3D visualization
    function initVisualization() {
        console.log("Initializing 6-DOF robot arm visualization...");
        
        // Create scene
        scene = new THREE.Scene();
        scene.background = new THREE.Color(0x222222);
        
        // Create camera - ZOOMED IN CLOSER
        camera = new THREE.PerspectiveCamera(75, 800 / 400, 0.1, 1000);
        camera.position.set(6, 5, 6); // Moved closer from (10, 8, 10)
        camera.lookAt(0, 2, 0); // Look at center of robot arm
        
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
        directionalLight.shadow.mapSize.width = 2048;
        directionalLight.shadow.mapSize.height = 2048;
        scene.add(directionalLight);
        
        // Add grid - smaller grid for closer view
        const gridHelper = new THREE.GridHelper(10, 10, 0x888888, 0x444444);
        scene.add(gridHelper);
        
        // Add axes
        const axesHelper = new THREE.AxesHelper(2);
        scene.add(axesHelper);
        
        // Create robot arm
        createRobotArm();
        
        console.log("6-DOF robot arm visualization initialized successfully");
        
        // Start animation loop
        animate();
    }
    
    // Create 6-DOF robot arm
    function createRobotArm() {
        robotArm = new THREE.Group();
        joints = [];
        
        // Base (fixed)
        const baseGeometry = new THREE.CylinderGeometry(
            armConfig.base.radius, 
            armConfig.base.radius, 
            armConfig.base.height
        );
        const baseMaterial = new THREE.MeshLambertMaterial({ color: 0x444444 });
        const base = new THREE.Mesh(baseGeometry, baseMaterial);
        base.position.y = armConfig.base.height / 2;
        base.castShadow = true;
        robotArm.add(base);
        
        // Joint 1 (Base rotation - around Y axis)
        const joint1Group = new THREE.Group();
        const joint1Geometry = new THREE.CylinderGeometry(
            armConfig.joint1.radius, 
            armConfig.joint1.radius, 
            armConfig.joint1.height
        );
        const joint1Material = new THREE.MeshLambertMaterial({ color: 0x3498db });
        const joint1 = new THREE.Mesh(joint1Geometry, joint1Material);
        joint1.position.y = armConfig.joint1.height / 2;
        joint1.castShadow = true;
        joint1Group.add(joint1);
        joint1Group.position.y = armConfig.base.height;
        joints.push(joint1Group);
        
        // Joint 2 (Shoulder - around Z axis)
        const joint2Group = new THREE.Group();
        const joint2Geometry = new THREE.CylinderGeometry(
            armConfig.joint2.radius, 
            armConfig.joint2.radius, 
            armConfig.joint2.height
        );
        const joint2Material = new THREE.MeshLambertMaterial({ color: 0xe74c3c });
        const joint2 = new THREE.Mesh(joint2Geometry, joint2Material);
        joint2.rotation.z = Math.PI / 2; // Horizontal orientation
        joint2.position.x = armConfig.joint2.height / 2;
        joint2.castShadow = true;
        joint2Group.add(joint2);
        joint2Group.position.y = armConfig.joint1.height;
        joints.push(joint2Group);
        
        // Joint 3 (Elbow - around Z axis)
        const joint3Group = new THREE.Group();
        const joint3Geometry = new THREE.CylinderGeometry(
            armConfig.joint3.radius, 
            armConfig.joint3.radius, 
            armConfig.joint3.height
        );
        const joint3Material = new THREE.MeshLambertMaterial({ color: 0x2ecc71 });
        const joint3 = new THREE.Mesh(joint3Geometry, joint3Material);
        joint3.rotation.z = Math.PI / 2;
        joint3.position.x = armConfig.joint3.height / 2;
        joint3.castShadow = true;
        joint3Group.add(joint3);
        joint3Group.position.x = armConfig.joint2.height;
        joints.push(joint3Group);
        
        // Joint 4 (Wrist roll - around X axis)
        const joint4Group = new THREE.Group();
        const joint4Geometry = new THREE.CylinderGeometry(
            armConfig.joint4.radius, 
            armConfig.joint4.radius, 
            armConfig.joint4.height
        );
        const joint4Material = new THREE.MeshLambertMaterial({ color: 0xf39c12 });
        const joint4 = new THREE.Mesh(joint4Geometry, joint4Material);
        joint4.position.x = armConfig.joint4.height / 2;
        joint4.castShadow = true;
        joint4Group.add(joint4);
        joint4Group.position.x = armConfig.joint3.height;
        joints.push(joint4Group);
        
        // Joint 5 (Wrist pitch - around Z axis)
        const joint5Group = new THREE.Group();
        const joint5Geometry = new THREE.CylinderGeometry(
            armConfig.joint5.radius, 
            armConfig.joint5.radius, 
            armConfig.joint5.height
        );
        const joint5Material = new THREE.MeshLambertMaterial({ color: 0x9b59b6 });
        const joint5 = new THREE.Mesh(joint5Geometry, joint5Material);
        joint5.rotation.z = Math.PI / 2;
        joint5.position.x = armConfig.joint5.height / 2;
        joint5.castShadow = true;
        joint5Group.add(joint5);
        joint5Group.position.x = armConfig.joint4.height;
        joints.push(joint5Group);
        
        // Joint 6 (Wrist yaw - around X axis)
        const joint6Group = new THREE.Group();
        const joint6Geometry = new THREE.CylinderGeometry(
            armConfig.endEffector.radius, 
            armConfig.endEffector.radius, 
            armConfig.endEffector.height
        );
        const joint6Material = new THREE.MeshLambertMaterial({ color: 0x1abc9c });
        const joint6 = new THREE.Mesh(joint6Geometry, joint6Material);
        joint6.position.x = armConfig.endEffector.height / 2;
        joint6.castShadow = true;
        joint6Group.add(joint6);
        joint6Group.position.x = armConfig.joint5.height;
        joints.push(joint6Group);
        
        // Add end effector indicator
        const endEffectorGeometry = new THREE.SphereGeometry(0.15);
        const endEffectorMaterial = new THREE.MeshLambertMaterial({ color: 0xff0000 });
        const endEffector = new THREE.Mesh(endEffectorGeometry, endEffectorMaterial);
        endEffector.position.x = armConfig.endEffector.height;
        endEffector.castShadow = true;
        joint6Group.add(endEffector);
        
        // Build hierarchy: each joint is child of previous
        robotArm.add(joints[0]); // Joint 1 to base
        joints[0].add(joints[1]); // Joint 2 to Joint 1
        joints[1].add(joints[2]); // Joint 3 to Joint 2
        joints[2].add(joints[3]); // Joint 4 to Joint 3
        joints[3].add(joints[4]); // Joint 5 to Joint 4
        joints[4].add(joints[5]); // Joint 6 to Joint 5
        
        scene.add(robotArm);
    }
    
    // Update robot arm joint angles with SMOOTH ANIMATION
    function updateRobotArm(newJointAngles) {
        if (!joints || joints.length !== 6) {
            console.log("Robot arm not initialized yet");
            return;
        }
        
        console.log("Setting target joint angles:", newJointAngles);
        
        // Set target angles for smooth animation
        targetJointAngles = [...newJointAngles];
    }
    
    // Smooth animation function called every frame
    function animateJoints() {
        if (!joints || joints.length !== 6) return;
        
        let needsUpdate = false;
        
        // Smoothly interpolate current angles toward target angles
        for (let i = 0; i < 6; i++) {
            const diff = targetJointAngles[i] - jointAngles[i];
            if (Math.abs(diff) > 0.1) { // Only animate if difference is significant
                jointAngles[i] += diff * animationSpeed;
                needsUpdate = true;
            } else {
                jointAngles[i] = targetJointAngles[i]; // Snap to target when close
            }
        }
        
        if (needsUpdate) {
            // Apply smooth rotations to each joint
            joints[0].rotation.y = THREE.MathUtils.degToRad(jointAngles[0]); // Base rotation
            joints[1].rotation.z = THREE.MathUtils.degToRad(jointAngles[1]); // Shoulder
            joints[2].rotation.z = THREE.MathUtils.degToRad(jointAngles[2]); // Elbow
            joints[3].rotation.x = THREE.MathUtils.degToRad(jointAngles[3]); // Wrist roll
            joints[4].rotation.z = THREE.MathUtils.degToRad(jointAngles[4]); // Wrist pitch
            joints[5].rotation.x = THREE.MathUtils.degToRad(jointAngles[5]); // Wrist yaw
        }
    }
    
    // Animation loop
    function animate() {
        requestAnimationFrame(animate);
        
        // Animate joints smoothly
        animateJoints();
        
        // Rotate camera around the scene for better view - CLOSER ORBIT
        const time = Date.now() * 0.0003;
        camera.position.x = Math.cos(time) * 8; // Reduced from 15 to 8
        camera.position.z = Math.sin(time) * 8; // Reduced from 15 to 8
        camera.position.y = 5; // Keep height constant
        camera.lookAt(0, 2, 0); // Look at center of robot arm
        
        renderer.render(scene, camera);
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
        
        // Update position display (end effector position)
        positionValue.textContent = `X: ${state.position.x.toFixed(2)}, Y: ${state.position.y.toFixed(2)}, Z: ${state.position.z.toFixed(2)}`;
        
        // Update joint angles display
        if (state.joint_angles) {
            const jointStr = state.joint_angles.map((angle, i) => `J${i+1}: ${angle.toFixed(1)}°`).join(', ');
            orientationValue.textContent = jointStr;
            
            // Update 3D visualization with smooth animation
            updateRobotArm(state.joint_angles);
        } else {
            // Fallback to orientation display
            orientationValue.textContent = `Roll: ${state.orientation.roll.toFixed(2)}, Pitch: ${state.orientation.pitch.toFixed(2)}, Yaw: ${state.orientation.yaw.toFixed(2)}`;
        }
        
        // Update mode indicator
        modeIndicator.textContent = `Mode: ${state.mode.charAt(0).toUpperCase() + state.mode.slice(1)}`;
        
        // Log state update
        addLogEntry(`State updated: ${state.status}`);
        
        // Log command received if status indicates movement
        if (state.status === 'moving') {
            if (state.joint_angles) {
                addCommandEntry('JOINT MOVE', `Joints: [${state.joint_angles.map(a => a.toFixed(1)).join(', ')}]°`);
            } else {
                addCommandEntry('MOVE', `Position: (${state.position.x.toFixed(2)}, ${state.position.y.toFixed(2)}, ${state.position.z.toFixed(2)})`);
            }
        } else if (state.status === 'rotating') {
            addCommandEntry('ROTATE', `Joint movement in progress`);
        }
    });
    
    socket.on('command_sent', function(data) {
        addLogEntry(`Command sent: ${data.command}`);
        
        // Add to commands log with details
        if (data.command === 'move_joints' && data.joint_angles) {
            addCommandEntry('JOINT COMMAND', `Angles: [${data.joint_angles.map(a => a.toFixed(1)).join(', ')}]°`);
        } else if (data.command === 'move' && data.position) {
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
    console.log("Starting 6-DOF robot arm visualization initialization...");
    initVisualization();
});
