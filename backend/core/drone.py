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
        self.waypoint_colors = []  # List of HEX colors for each segment/waypoint
        self.local_time = 0.0      # Current internal clock (seconds)
        self.path_loop = True      # Should the path repeat after completion?
        self.is_kinematic = True   # Kinematic (Step 1) vs Dynamic (Step 2+)
        
        # PID controllers and physical properties
        from core.controller import PositionController3D
        self.controller = PositionController3D()
        self.mass = 1.2
        self.max_thrust = 35.0
        self.tilt_angles = [0.0, 0.0]

    def set_mission_waypoints(self, waypoints, times, loop=True, colors=None):
        """
        Sets the sparse mission waypoints, times, and step LED colors.
        Each drone plans its own trajectory solution based on this scheduling list.
        """
        self.waypoints = [np.array(wp, dtype=np.float64) for wp in waypoints]
        self.waypoint_times = list(times)
        self.path_loop = loop
        self.local_time = 0.0
        
        if colors is not None:
            self.waypoint_colors = list(colors)
        else:
            self.waypoint_colors = ["#00F2FE"] * len(self.waypoints)
            
        if len(self.waypoints) > 0:
            self.target_pos = np.copy(self.waypoints[0])
            self.position = np.copy(self.waypoints[0])
            self.color = self.waypoint_colors[0]

    def set_pid_gains(self, kp_xy, ki_xy, kd_xy, kp_z, ki_z, kd_z):
        """Updates internal PositionController3D parameters."""
        self.controller.set_gains(kp_xy, ki_xy, kd_xy, kp_z, ki_z, kd_z)

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
        
        # Normalized segment time (0 to 1) with S-curve ease-in ease-out (smootherstep)
        u_raw = (t - t0) / (t1 - t0)
        u = 10.0 * (u_raw ** 3) - 15.0 * (u_raw ** 4) + 6.0 * (u_raw ** 5)
        
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
        """Updates LED color dynamically based on its active path segment settings."""
        n = len(self.waypoints)
        if n == 0 or len(self.waypoint_colors) == 0:
            return
            
        t = self.local_time
        t_end = self.waypoint_times[-1]
        
        # Handle wrap around for loop
        if t >= t_end:
            if self.path_loop:
                duration = t_end - self.waypoint_times[0]
                t = self.waypoint_times[0] + ((t - self.waypoint_times[0]) % duration)
            else:
                self.color = self.waypoint_colors[-1]
                if self.color == "#151720":
                    self.state = "STANDBY"
                return
                
        # Find active segment
        idx = 0
        for i in range(n - 1):
            if self.waypoint_times[i] <= t < self.waypoint_times[i+1]:
                idx = i
                break
                
        # Match color of active segment
        self.color = self.waypoint_colors[idx]
        
        # Update flight state based on color/height/segment
        if self.color == "#151720":
            self.state = "STANDBY"
        elif idx == 0:
            self.state = "TAKEOFF"
        elif idx == n - 2:
            self.state = "LANDING"
        else:
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
        """
        Step 2+: Physical simulation of translation and tilt.
        Uses a cascade model: horizontal forces calculate tilt target angles,
        which vector the main thrust.
        """
        g = 9.81
        
        # 1. Ask controller for required translation forces F_demand
        # F_demand is [Fx, Fy, Fz]
        f_demand = self.controller.update(self.position, self.target_pos, self.velocity, dt)
        
        # Cosine of actual tilt angles (pitch is index 1, roll is index 0)
        cos_tilt = np.cos(self.tilt_angles[0]) * np.cos(self.tilt_angles[1])
        cos_tilt = max(cos_tilt, 0.5)  # clamp to prevent division by zero / extreme angles
        
        # 2. Total Thrust demand (with gravity feedforward and tilt projection compensation)
        thrust = (f_demand[2] + self.mass * g) / cos_tilt
        # Clamp thrust between 0 and max_thrust
        thrust = np.clip(thrust, 0.0, self.max_thrust)
        
        # 3. Calculate target tilt angles (Roll, Pitch) from horizontal force demands
        # Fx = thrust * sin(pitch) -> pitch_target = arcsin(Fx / thrust)
        # Fy = -thrust * sin(roll) -> roll_target = -arcsin(Fy / thrust)
        # Small angle approximation for stability:
        max_tilt = 0.52  # ~30 degrees
        
        if thrust > 0.1:
            pitch_target = np.clip(f_demand[0] / thrust, -max_tilt, max_tilt)
            roll_target = np.clip(-f_demand[1] / thrust, -max_tilt, max_tilt)
        else:
            pitch_target = 0.0
            roll_target = 0.0
            
        # 4. First-order angular motor lag (attitude response)
        tau = 0.15  # seconds
        self.tilt_angles[0] += (roll_target - self.tilt_angles[0]) / tau * dt
        self.tilt_angles[1] += (pitch_target - self.tilt_angles[1]) / tau * dt
        
        # 5. Vector thrust force using actual tilt angles
        r = self.tilt_angles[0]
        p = self.tilt_angles[1]
        
        # Inertial forces from vectoring
        # (For Step 2, we don't have drag or wind yet, that goes in Step 3!)
        ax = (thrust / self.mass) * np.sin(p) * np.cos(r)
        ay = -(thrust / self.mass) * np.sin(r)
        az = (thrust / self.mass) * np.cos(p) * np.cos(r) - g
        
        self.acceleration = np.array([ax, ay, az], dtype=np.float64)
        
        # 6. Integrate acceleration to velocity and position (Euler-Cromer)
        self.velocity += self.acceleration * dt
        # Clamp maximum speed for numerical stability
        max_speed = 15.0  # m/s
        speed = np.linalg.norm(self.velocity)
        if speed > max_speed:
            self.velocity = (self.velocity / speed) * max_speed
            
        self.position += self.velocity * dt
        
        # State estimation
        self.state = "FLYING"

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
