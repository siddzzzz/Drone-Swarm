# CyberSwarm: High-Fidelity 3D Drone Swarm Simulation & GCS Cockpit

A step-by-step, physics-grounded simulator and 3D Ground Control Station (GCS) for large-scale autonomous drone swarm shows. 

Built with a **hybrid architecture**:
- **Robotics & Control Backend (Python)**: 6-DOF quadcopter dynamics, cascaded PID flight controllers, motor lag, aerodynamic drag, harmonic wind gusts, Hungarian matching, and vectorized Artificial Potential Field (APF) collision avoidance.
- **3D Ground Control Station (JavaScript / Three.js / WebGL)**: Hardware-accelerated 60 FPS visualizer with free-angle orbit controls, holographic grid, glowing LED light nodes, dynamic trails, live PID telemetry plotting, and interactive HUD control panels.

---

## Architecture Overview

```
                               ┌────────────────────────────────────────┐
                               │       Python Physics & Robotics Core   │
                               │  - 60Hz Physics Integration Loop       │
                               │  - 6-DOF Cascaded PID Flight Control   │
                               │  - Vectorized APF Collision Avoidance  │
                               │  - Hungarian Task / Shape Matching     │
                               │  - Show Timeline Orchestrator          │
                               └──────────────────┬─────────────────────┘
                                                  │ (WebSocket Telemetry @ 60Hz)
                                                  │ JSON State Stream
                                                  ▼
                               ┌────────────────────────────────────────┐
                               │     WebGL Ground Control Station (GCS) │
                               │  - Three.js 3D Viewport (Free Orbit)   │
                               │  - Real-Time Drone Tilt & Prop Spin    │
                               │  - Glowing LED Trails & Safe Grids     │
                               │  - Live PID Error Charting & Sliders   │
                               └────────────────────────────────────────┘
```

---

## 5-Step Progressive Roadmap

This project is structured step-by-step to build a foundation from simple kinematics to full physics-based swarm coordination:

### Step 1: Trajectory Planning & Splines (`core/path_planner.py`)
- **Catmull-Rom Splines**: Smooth parametric curves interpolated through sparse 3D waypoints.
- **Smootherstep Velocity Profiles**: $S(u) = 10u^3 - 15u^4 + 6u^5$ for smooth jerk-free acceleration and deceleration.
- **Parametric Paths**: Circle, Figure-Eight, 3D Helix, and complex show sequences.

### Step 2: Cascaded PID Flight Control (`core/controller.py`, `core/drone.py`)
- **Position Loop**: Calculates translational force demands $\mathbf{F}_{\text{demand}} = [F_x, F_y, F_z]^T$ with integral anti-windup.
- **Tilt Vectoring**: Translates horizontal force demands into pitch ($\theta$) and roll ($\phi$) target angles:
  $$\text{Pitch}_{\text{cmd}} = \text{clip}\left(\frac{F_x}{T}, -\theta_{\max}, \theta_{\max}\right), \quad \text{Roll}_{\text{cmd}} = \text{clip}\left(-\frac{F_y}{T}, -\phi_{\max}, \phi_{\max}\right)$$
- **Thrust Calculation**: Direct feedforward for gravity and cosine projection tilt compensation:
  $$T = \frac{F_z + mg}{\cos\phi \cos\theta}$$
- **Motor / ESC Lag**: 1st-order differential lag ($\tau = 0.15\text{s}$) modeling physical rotor inertia.

### Step 3: Environmental Physics & Hardware Failsafes (`core/physics.py`)
- **Aerodynamic Drag**: Quadratic relative-velocity air resistance:
  $$\mathbf{F}_{\text{drag}} = -\frac{1}{2} \rho C_d A \|\mathbf{v} - \mathbf{w}\| (\mathbf{v} - \mathbf{w})$$
- **Harmonic Wind & Turbulence**: Multi-frequency wind gusts and vertical turbulence.
- **Feedforward Wind Trim**: Autopilot estimates steady drag and trims control outputs.
- **Non-Linear Battery Depletion & Failsafes**: Drain rate scales with $T^{1.5}$. If battery drops $\le 15\%$, drone initiates emergency Return-to-Home and autonomous landing.

### Step 4: Swarm Formations & Collision Avoidance (`core/swarm_coordinator.py`)
- **Optimal Task Assignment (Hungarian Algorithm)**: Linear sum assignment minimizing $\sum \|\mathbf{p}_i - \mathbf{q}_j\|^2$ to prevent path intersections.
- **Adaptive Spacing Enforcement**: Automatically scales formations up to ensure $100\%$ drone participation with zero collisions.
- **Vectorized Artificial Potential Fields (APF)**: Fast NumPy matrix repulsive forces:
  $$\mathbf{F}_{\text{rep}} = K_{\text{rep}} \left(\frac{1}{d} - \frac{1}{R_{\text{safety}}}\right) \frac{1}{d^2} \hat{\mathbf{r}}$$

### Step 5: Full Show Choreography (`core/show_orchestrator.py`)
- Automated 70-second timeline with synchronized LED color shifts and smooth transitions:
  1. **Takeoff & Pre-show Hover** (Cyan LEDs)
  2. **3D Footballer Kicking Soccer Ball** (Lime Green LEDs)
  3. **3D Winnie-the-Pooh Bear Holding Honey Pot** (Gold LEDs)
  4. **3D Pulsing Heart** (Crimson Pink LEDs)
  5. **3D Pyramid** (Cyan LEDs)
  6. **Synchronized Formation Landing** (Purple LEDs)

---

## Directory Structure

```
Drone-Swarm/
├── backend/
│   ├── main.py                     # WebSocket server & 60Hz simulation runner
│   ├── requirements.txt            # Python dependencies (numpy, scipy, websockets)
│   └── core/
│       ├── drone.py                # 6-DOF Quadcopter physical agent model
│       ├── controller.py           # Discrete-time PID & 3D position controllers
│       ├── physics.py              # Wind, turbulence gusts, and quadratic drag
│       ├── path_planner.py         # Catmull-Rom spline trajectory generation
│       ├── swarm_coordinator.py    # Hungarian solver, 3D shapes, and vectorized APF
│       └── show_orchestrator.py    # 70-second synchronized show timeline
├── src/
│   ├── main.js                     # GCS Application entry point & WS coordinator
│   ├── style.css                   # Glassmorphic futuristic HUD stylesheet
│   ├── visualizer/
│   │   ├── SceneManager.js         # Three.js scene, lighting, fog, and OrbitControls
│   │   ├── DroneVisuals.js         # Quadcopter 3D meshes, spinning props, and trails
│   │   └── TrajectoryVisuals.js    # Spline reference lines and waypoint particles
│   └── ui/
│       ├── HUDController.js        # GCS button handlers, telemetry readouts, and sliders
│       └── ChartPlotter.js         # Real-time HTML5 2D canvas altitude PID chart
├── index.html                      # Main GCS dashboard layout
├── package.json                    # Frontend dependencies (vite, three)
├── run.bat                         # One-click startup script for Windows
└── README.md                       # Documentation
```

---

## Quick Start Guide

### Prerequisites
1. **Python 3.8+** with `pip`
2. **Node.js 16+** with `npm`

### Installation

1. **Install Python backend requirements:**
   ```bash
   pip install -r backend/requirements.txt
   ```

2. **Install frontend packages:**
   ```bash
   npm install
   ```

### Running the Simulator

#### Option A: One-Click Windows Script
Double-click `run.bat` or run:
```cmd
run.bat
```

#### Option B: Manual Terminal Execution
1. **Start the Python Physics Core (Terminal 1):**
   ```bash
   python backend/main.py
   ```
2. **Start the Vite GCS Dashboard (Terminal 2):**
   ```bash
   npm run dev
   ```
3. Open `http://localhost:5173` in your web browser.

---

## Interactive GCS Controls

- **Simulation Step Selector (01 to 05)**: Switch between trajectory planning, PID control, wind physics, APF swarm avoidance, and full show mode.
- **Swarm Scale (1, 10, 30, 60, 100)**: Scale from a single drone up to 100 drones in real-time.
- **Path Modes**: Circle, Figure-8, Helix, and SHOW mode.
- **Camera Views**:
  - **Orbit View**: Click & drag with Left Mouse Button to rotate, scroll wheel to zoom, Right Mouse Button to pan.
  - **Top View (2D)**: True orthographic top-down projection.
  - **Front View**: Horizontal side profile.
- **Live Parameter Tuning**:
  - PID gains ($K_p, K_i, K_d$ for XY translation and Z altitude).
  - Weather controls (Wind speed, Wind direction, Gust turbulence).
  - APF collision avoidance parameters ($K_{\text{rep}}$, $R_{\text{safety}}$).