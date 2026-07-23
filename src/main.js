import { SceneManager } from './visualizer/SceneManager.js';
import { DroneVisuals } from './visualizer/DroneVisuals.js';
import { TrajectoryVisuals } from './visualizer/TrajectoryVisuals.js';
import { HUDController } from './ui/HUDController.js';
import { ChartPlotter } from './ui/ChartPlotter.js';

class App {
  constructor() {
    this.sceneManager = null;
    this.droneVisuals = null;
    this.trajectoryVisuals = null;
    this.hud = null;
    this.chart = null;
    
    this.socket = null;
    this.wsUrl = 'ws://127.0.0.1:8765';
    
    // FPS stats tracking
    this.lastFrameTime = performance.now();
    this.fps = 60;
    
    this.init();
  }

  init() {
    try {
      // 1. Initialize Visual Modules
      this.sceneManager = new SceneManager('canvas-container');
      this.droneVisuals = new DroneVisuals(this.sceneManager.scene);
      this.trajectoryVisuals = new TrajectoryVisuals(this.sceneManager.scene);
      
      // 2. Initialize HUD (expose socket sender)
      this.hud = new HUDController(
        this.sendSocketMessage.bind(this),
        this.sceneManager
      );
      
      // 3. Initialize PID Altitude Chart
      this.chart = new ChartPlotter('pid-chart');
      
      // 3. Connect to Web Socket Simulation Server
      this.connectSocket();
      
      // 4. Start Render loop
      this.animate();
    } catch (e) {
      console.error("Initialization error: ", e);
    }
  }

  connectSocket() {
    this.hud.logTerminal(`[SOCKET] Connecting to simulator core at ${this.wsUrl}...`);
    this.socket = new WebSocket(this.wsUrl);

    this.socket.onopen = () => {
      this.hud.updateConnection(true);
    };

    this.socket.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        
        if (data.type === 'config') {
          // Synchronize UI selectors with backend settings
          this.syncUI(data);
        } else if (data.type === 'telemetry') {
          this.handleTelemetry(data);
        }
      } catch (err) {
        console.error("Error processing websocket message:", err);
      }
    };

    this.socket.onclose = () => {
      this.hud.updateConnection(false);
      this.droneVisuals.clearAll();
      this.trajectoryVisuals.clear();
      // Retry connection every 2.5 seconds
      setTimeout(() => this.connectSocket(), 2500);
    };

    this.socket.onerror = () => {
      this.socket.close();
    };
  }

  sendSocketMessage(payload) {
    if (this.socket && this.socket.readyState === WebSocket.OPEN) {
      this.socket.send(jsonStringify(payload));
    }
  }

  syncUI(config) {
    // 1. Sync active step
    const stepBtn = document.querySelector(`.step-btn[data-step="${config.step}"]`);
    if (stepBtn) {
      document.querySelectorAll('.step-btn').forEach(b => b.classList.remove('active'));
      stepBtn.classList.add('active');
    }
    
    // 2. Sync scale buttons
    const scaleBtn = document.querySelector(`[data-scale="${config.num_drones}"]`);
    if (scaleBtn) {
      document.querySelectorAll('[data-scale]').forEach(b => b.classList.remove('active'));
      scaleBtn.classList.add('active');
    }
    
    // 3. Sync path buttons
    const pathBtn = document.querySelector(`[data-path="${config.path_type}"]`);
    if (pathBtn) {
      document.querySelectorAll('[data-path]').forEach(b => b.classList.remove('active'));
      pathBtn.classList.add('active');
    }
    
    this.hud.logTerminal(`[SOCKET] Handshake completed. Configured step: ${config.step}, drones: ${config.num_drones}, path: ${config.path_type}.`);
  }

  handleTelemetry(data) {
    const drones = data.drones;
    
    // 1. Update 3D drone meshes, light glowing, and propeller angles
    this.droneVisuals.update(drones);
    
    // 2. Update planned spline lines (useful in Step 1)
    if (data.paths) {
      this.trajectoryVisuals.update(data.paths, drones);
    }
    
    // 3. Compute telemetry stats
    const droneCount = drones.length;
    let totalBattery = 0;
    let failsafeCount = 0;
    drones.forEach(d => {
      totalBattery += d.battery;
      if (d.state === "FAILSAFE_LAND") {
        failsafeCount++;
      }
    });
    const avgBattery = droneCount > 0 ? (totalBattery / droneCount) : 0;
    
    // Collisions counter (will calculate in Step 4 in python, let's read or default to 0)
    let collisionCount = 0;
    if (data.collisions !== undefined) {
      collisionCount = data.collisions;
    }
    
    // Update stats widgets on GCS Cockpit HUD
    this.hud.updateStats(droneCount, avgBattery, collisionCount, this.fps, failsafeCount);
    
    // 4. Update dynamic PID tracking chart (for Drone 0 in Step 2+)
    if (drones.length > 0 && this.chart && data.step >= 2) {
      const drone0 = drones[0];
      this.chart.push(drone0.tz, drone0.z);
    }
  }

  animate() {
    requestAnimationFrame(this.animate.bind(this));
    
    // Measure FPS
    const time = performance.now();
    const dt = time - this.lastFrameTime;
    this.lastFrameTime = time;
    this.fps = 1000 / dt;
    
    // Update Orbit controls and camera positions
    if (this.sceneManager) {
      this.sceneManager.update();
      this.sceneManager.render();
    }
  }
}

// Utility to safe stringify JSON payload
function jsonStringify(obj) {
  return JSON.stringify(obj);
}

// Instantiate App
window.addEventListener('DOMContentLoaded', () => {
  new App();
});
