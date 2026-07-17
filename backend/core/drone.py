import numpy as np

class Drone:
    def __init__(self, drone_id, initial_pos=None):
        self.id = drone_id
        self.position = np.array(initial_pos if initial_pos is not None else [0.0, 0.0, 0.0], dtype=np.float64)
        self.velocity = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.acceleration = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        
        self.target_pos = np.copy(self.position)
        self.color = "#00F2FE"  # Hex string for LED glow
        self.state = "IDLE"  # IDLE, TAKEOFF, FLYING, LANDING, LANDED
        self.battery = 100.0  # Percentage
        
        # Local Trajectory Planning (Independent)
        self.waypoints = []        # List of numpy arrays representing key locations
        self.waypoint_times = []   # List of timestamps (seconds) when each waypoint must be reached
        self.local_time = 0.0      # Current internal clock (seconds)
        self.path_loop = True      # Should the path repeat after completion?
        self.is_kinematic = True   # Kinematic (Step 1) vs Dynamic (Step 2+)
        
        # PID controllers and physical properties
        self.pid_x = None
        self.pid_y = None
        self.pid_z = None
        self.mass = 1.2
        self.max_thrust = 25.0
        self.tilt_angles = [0.0, 0.0]

    def set_mission_waypoints(self, waypoints, times, loop=True):
        """
        Sets the sparse mission waypoints and times.
        Each drone plans its own trajectory solution based on this scheduling list.
        """
        self.waypoints = [np.array(wp, dtype=np.float64) for wp in waypoints]
        self.waypoint_times = list(times)
        self.path_loop = loop
        self.local_time = 0.0
        if len(self.waypoints) > 0:
            self.target_pos = np.copy(self.waypoints[0])
            self.position = np.copy(self.waypoints[0])

    def evaluate_trajectory(self, t):
        """
        Independently calculates the target 3D position at time 't' using Catmull-Rom spline
        interpolation over the drone's sparse waypoint mission.
        """
        n = len(self.waypoints)
        if n == 0:
            return self.position
        if n == 1:
            return self.waypoints[0]
            
        t_start = self.waypoint_times[0]
        t_end = self.waypoint_times[-1]
        
        # Handle looping or clamping
        if t < t_start:
            return self.waypoints[0]
        if t >= t_end:
            if self.path_loop:
                # Wrap time around the duration loop
                duration = t_end - t_start
                t = t_start + ((t - t_start) % duration)
            else:
                return self.waypoints[-1]
                
        # Find active segment (where t is between waypoint times)
        idx = 0
        for i in range(n - 1):
            if self.waypoint_times[i] <= t < self.waypoint_times[i+1]:
                idx = i
                break
                
        t0 = self.waypoint_times[idx]
        t1 = self.waypoint_times[idx+1]
        
        # Normalized segment time (0 to 1)
        u = (t - t0) / (t1 - t0)
        
        # Get 4 control points for Catmull-Rom spline
        p1 = self.waypoints[idx]
        p2 = self.waypoints[idx+1]
        
        if self.path_loop:
            # Wrap around indices
            p0 = self.waypoints[(idx - 1) % n]
            p3 = self.waypoints[(idx + 2) % n]
        else:
            # Clamp indices for open paths
            p0 = self.waypoints[idx - 1] if idx > 0 else p1 - (p2 - p1)
            p3 = self.waypoints[idx + 2] if idx + 2 < n else p2 + (p2 - p1)
            
        # Catmull-Rom formulation:
        # P(u) = 0.5 * ( (2*P1) + (-P0 + P2)*u + (2*P0 - 5*P1 + 4*P2 - P3)*u^2 + (-P0 + 3*P1 - 3*P2 + P3)*u^3 )
        target = 0.5 * (
            2 * p1 +
            (-p0 + p2) * u +
            (2 * p0 - 5 * p1 + 4 * p2 - p3) * (u**2) +
            (-p0 + 3 * p1 - 3 * p2 + p3) * (u**3)
        )
        return target

    def update_led_color(self):
        """Updates LED color dynamically based on mission phase or drone ID."""
        is_show = len(self.waypoint_times) == 14 and self.waypoint_times[-1] == 70.0
        
        if is_show:
            t = self.local_time
            t_end = self.waypoint_times[-1]
            if t >= t_end and self.path_loop:
                t = t % t_end
                
            if t < 4.0:
                self.color = "#00F2FE"  # Cyan (Takeoff)
                self.state = "TAKEOFF"
            elif t < 7.0:
                self.color = "#00F2FE"  # Cyan (Hover Grid Hold)
                self.state = "FLYING"
            elif t < 19.0:
                self.color = "#0055FF"  # Blue (Sphere & Hold)
                self.state = "FLYING"
            elif t < 31.0:
                self.color = "#FFD700"  # Gold/Yellow (Star & Hold)
                self.state = "FLYING"
            elif t < 43.0:
                self.color = "#FF2A5F"  # Crimson/Pink (Heart & Hold)
                self.state = "FLYING"
            elif t < 55.0:
                self.color = "#39FF14"  # Lime Green (Pyramid & Hold)
                self.state = "FLYING"
            elif t < 66.0:
                self.color = "#00F2FE"  # Cyan (Landing Hover Hold)
                self.state = "FLYING"
            else:
                self.color = "#8A2BE2"  # Purple (Landing)
                self.state = "LANDING"
        else:
            # Generate static distinct rainbow hue based on drone ID
            neon_colors = [
                "#00f2fe", "#4facfe", "#39ff14", "#ff2a5f", 
                "#ffdf00", "#ff007f", "#8a2be2", "#ff4500"
            ]
            self.color = neon_colors[self.id % len(neon_colors)]
            self.state = "FLYING"

    def update(self, dt):
        """
        Updates the drone's position and control systems.
        """
        # Increment local mission clock
        self.local_time += dt
        
        # 1. Independent Trajectory Planning:
        # Find where we SHOULD be at this exact timestamp
        self.target_pos = self.evaluate_trajectory(self.local_time)
        
        # Update LED colors & flight state based on current phase
        self.update_led_color()
        
        # 2. Kinematic vs. Dynamic tracking:
        if self.is_kinematic:
            # Step 1: Teleport exactly to planned target (Ideal Path Follower)
            self.position = np.copy(self.target_pos)
        else:
            # Step 2+: Position control via forces and PID loops
            self.update_dynamics(dt)

    def update_dynamics(self, dt):
        """Placeholder for Step 2 Dynamic calculations."""
        pass

    def to_dict(self):
        """Serializes drone telemetry for Ground Control Station transmission."""
        return {
            "id": self.id,
            "x": float(self.position[0]),
            "y": float(self.position[1]),
            "z": float(self.position[2]),
            "tx": float(self.target_pos[0]),
            "ty": float(self.target_pos[1]),
            "tz": float(self.target_pos[2]),
            "roll": float(self.tilt_angles[0]),
            "pitch": float(self.tilt_angles[1]),
            "color": self.color,
            "state": self.state,
            "battery": float(self.battery),
            # Send sparse waypoints for rendering GCS reference nodes
            "waypoints": [wp.tolist() for wp in self.waypoints]
        }
