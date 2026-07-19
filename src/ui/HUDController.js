export class HUDController {
  constructor(socketSender, cameraController) {
    this.sendSocketMessage = socketSender;
    this.cameraController = cameraController;
    
    // Elements Cache
    this.stepButtons = document.querySelectorAll('.step-btn');
    this.scaleButtons = document.querySelectorAll('[data-scale]');
    this.pathButtons = document.querySelectorAll('[data-path]');
    
    this.droneCountVal = document.getElementById('stat-drone-count');
    this.collisionsVal = document.getElementById('stat-collisions');
    this.batteryVal = document.getElementById('stat-avg-battery');
    this.fpsVal = document.getElementById('stat-fps');
    
    this.terminal = document.getElementById('terminal-log');
    this.statusBadge = document.getElementById('connection-status');
    this.statusText = this.statusBadge.querySelector('.status-text');

    this.initEvents();
  }

  initEvents() {
    // 1. Step Buttons
    this.stepButtons.forEach(btn => {
      btn.addEventListener('click', () => {
        if (btn.classList.contains('locked')) {
          this.logTerminal(`[WARNING] Step ${btn.dataset.step} is currently locked. Complete previous steps to unlock.`, "text-red");
          return;
        }
        
        this.stepButtons.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        const stepNum = parseInt(btn.dataset.step);
        
        this.sendSocketMessage({
          type: "set_step",
          value: stepNum
        });
        
        this.logTerminal(`[SYSTEM] Requesting simulation state change: Transitioning to Step ${stepNum}...`, "text-cyan");
        
        // Show corresponding config panels
        document.querySelectorAll('.step-controls-panel').forEach(panel => panel.classList.remove('active'));
        const activePanel = document.getElementById(`step-${stepNum}-controls`);
        if (activePanel) {
          activePanel.classList.add('active');
        }
      });
    });

    // 2. Swarm Scale Toggle
    this.scaleButtons.forEach(btn => {
      btn.addEventListener('click', () => {
        this.scaleButtons.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        const scaleVal = parseInt(btn.dataset.scale);
        
        this.sendSocketMessage({
          type: "set_drones",
          value: scaleVal
        });
        this.logTerminal(`[SYSTEM] Scaling drone swarm to: ${scaleVal} units.`, "text-cyan");
      });
    });

    // 3. Path Toggle
    this.pathButtons.forEach(btn => {
      btn.addEventListener('click', () => {
        this.pathButtons.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        const pathVal = btn.dataset.path;
        
        this.sendSocketMessage({
          type: "set_path_type",
          value: pathVal
        });
        this.logTerminal(`[SYSTEM] Regenerating splines. Mode: ${pathVal.toUpperCase()}.`, "text-cyan");
      });
    });

    // 4. Camera Buttons
    document.getElementById('btn-cam-orbit').addEventListener('click', () => {
      this.cameraController.setCameraPosition(15, 15, 20, 0, 5, 0);
      this.logTerminal("[GCS] Camera view set to ORBIT (3D Perspective).");
    });
    
    document.getElementById('btn-cam-top').addEventListener('click', () => {
      this.cameraController.setCameraPosition(0, 30, 0.01, 0, 0, 0); // orthographic lookdown
      this.logTerminal("[GCS] Camera view set to TOP-DOWN (2D Projection).");
    });
    
    document.getElementById('btn-cam-front').addEventListener('click', () => {
      this.cameraController.setCameraPosition(0, 5, 25, 0, 5, 0);
      this.logTerminal("[GCS] Camera view set to FRONT PROFILE.");
    });

    // 5. PID Sliders (Step 2 live tuning)
    const pidSliders = ['kp-xy', 'ki-xy', 'kd-xy', 'kp-z', 'ki-z', 'kd-z'];
    pidSliders.forEach(pid => {
      const slider = document.getElementById(`slide-${pid}`);
      const valueSpan = document.getElementById(`val-${pid}`);
      if (slider && valueSpan) {
        slider.addEventListener('input', () => {
          valueSpan.textContent = slider.value;
          this.sendPIDUpdate();
        });
      }
    });
  }

  sendPIDUpdate() {
    const getVal = (id) => {
      const el = document.getElementById(id);
      return el ? parseFloat(el.value) : 0.0;
    };
    this.sendSocketMessage({
      type: "set_pid",
      kp_xy: getVal('slide-kp-xy'),
      ki_xy: getVal('slide-ki-xy'),
      kd_xy: getVal('slide-kd-xy'),
      kp_z: getVal('slide-kp-z'),
      ki_z: getVal('slide-ki-z'),
      kd_z: getVal('slide-kd-z')
    });
  }

  // Update Status HUD elements from telemetry data
  updateStats(numDrones, avgBattery, collisionCount, fps) {
    if (this.droneCountVal) this.droneCountVal.textContent = numDrones;
    if (this.batteryVal) this.batteryVal.textContent = `${Math.round(avgBattery)}%`;
    if (this.collisionsVal) this.collisionsVal.textContent = collisionCount;
    if (this.fpsVal) this.fpsVal.textContent = Math.round(fps);
  }

  updateConnection(isConnected) {
    if (isConnected) {
      this.statusBadge.className = "status-badge connected";
      this.statusText.textContent = "TELEMETRY CONNECTED";
      this.logTerminal("[SYSTEM] Bi-directional WebSocket pipeline established successfully.", "text-green");
    } else {
      this.statusBadge.className = "status-badge disconnected";
      this.statusText.textContent = "OFFLINE";
      this.logTerminal("[SYSTEM] Connection lost. Attempting auto-reconnect to WS server...", "text-red");
    }
  }

  // Log to terminal window
  logTerminal(message, cssClass = "") {
    const time = new Date().toLocaleTimeString();
    const line = document.createElement('div');
    line.className = `log-line ${cssClass}`;
    line.innerHTML = `<span style="color: #5a6b82;">[${time}]</span> ${message}`;
    this.terminal.appendChild(line);
    
    // Auto scroll to bottom
    this.terminal.scrollTop = this.terminal.scrollHeight;
    
    // Cap log lines to 100 to prevent performance degradation
    while (this.terminal.childElementCount > 100) {
      this.terminal.removeChild(this.terminal.firstChild);
    }
  }

  // Helper to dynamically unlock steps as we complete them
  unlockStep(stepNumber) {
    const btn = document.querySelector(`.step-btn[data-step="${stepNumber}"]`);
    if (btn && btn.classList.contains('locked')) {
      btn.classList.remove('locked');
      btn.removeAttribute('disabled');
      this.logTerminal(`[SYSTEM] COMPONENT UNLOCKED: Step ${stepNumber} control loop interface is now online.`, "text-green");
    }
  }
}
