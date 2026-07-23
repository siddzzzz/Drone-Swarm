import numpy as np

class EnvironmentPhysics:
    """
    Simulates environmental forces acting on the drone swarm, including
    aerodynamic drag, steady wind, dynamic turbulent gusts, and air density.
    """
    def __init__(self, wind_speed=0.0, wind_direction_deg=0.0, gust_intensity=0.0):
        self.wind_speed = float(wind_speed)          # m/s
        self.wind_direction = float(wind_direction_deg)  # degrees (0 = heading East, 90 = North)
        self.gust_intensity = float(gust_intensity)  # m/s peak turbulence
        
        # Physical Constants
        self.air_density = 1.225   # kg/m^3 (sea level air density)
        self.drag_coeff = 0.45     # Cd for typical quadcopter body frame
        self.cross_area = 0.08     # m^2 projected area of quadcopter frame
        
    def set_weather(self, wind_speed, wind_direction_deg, gust_intensity):
        """Updates weather parameters from GCS HUD input controls."""
        self.wind_speed = float(wind_speed)
        self.wind_direction = float(wind_direction_deg)
        self.gust_intensity = float(gust_intensity)

    def get_wind_vector(self, t):
        """
        Calculates the instantaneous 3D wind velocity vector at time 't'.
        Combines steady direction vector with dynamic multi-frequency gust turbulence.
        """
        if self.wind_speed <= 1e-4 and self.gust_intensity <= 1e-4:
            return np.array([0.0, 0.0, 0.0], dtype=np.float64)
            
        rad = np.radians(self.wind_direction)
        base_dir = np.array([np.cos(rad), np.sin(rad), 0.0], dtype=np.float64)
        
        # Base steady wind velocity
        v_base = base_dir * self.wind_speed
        
        # Dynamic turbulence / gust model using harmonic wave superposition
        if self.gust_intensity > 1e-4:
            gust_x = self.gust_intensity * (0.6 * np.sin(1.3 * t) + 0.4 * np.cos(3.7 * t))
            gust_y = self.gust_intensity * (0.6 * np.cos(1.1 * t) + 0.4 * np.sin(2.9 * t))
            gust_z = self.gust_intensity * 0.2 * np.sin(2.1 * t)  # Vertical updraft/downdraft
            gust_vec = np.array([gust_x, gust_y, gust_z], dtype=np.float64)
        else:
            gust_vec = np.array([0.0, 0.0, 0.0], dtype=np.float64)
            
        return v_base + gust_vec

    def compute_drag_force(self, velocity, wind_vector):
        """
        Calculates quadratic aerodynamic drag force: F_d = -0.5 * rho * Cd * A * |v_rel| * v_rel
        velocity: drone current 3D velocity vector [vx, vy, vz]
        wind_vector: instantaneous 3D wind vector [wx, wy, wz]
        """
        # Relative air velocity vector
        v_rel = velocity - wind_vector
        speed_rel = np.linalg.norm(v_rel)
        
        if speed_rel < 1e-6:
            return np.array([0.0, 0.0, 0.0], dtype=np.float64)
            
        # Quadratic drag magnitude
        f_drag_mag = 0.5 * self.air_density * self.drag_coeff * self.cross_area * (speed_rel ** 2)
        
        # Direction opposing relative motion
        f_drag = -f_drag_mag * (v_rel / speed_rel)
        
        return f_drag
