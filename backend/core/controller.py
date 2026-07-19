import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd, max_integral=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.max_integral = max_integral
        
    def update(self, error, velocity, dt):
        """Discrete-time PID calculation using derivative on measurement (velocity)."""
        if dt < 1e-5:
            return 0.0
            
        # 1. Proportional Term
        p_term = self.kp * error
        
        # 2. Integral Term with Windup Clamp
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.max_integral, self.max_integral)
        i_term = self.ki * self.integral
        
        # 3. Derivative Term on measurement (Dampens physical velocity to prevent overshoot)
        d_term = -self.kd * velocity
        
        return p_term + i_term + d_term

    def reset(self):
        """Clears controller history."""
        self.integral = 0.0
        self.prev_error = 0.0

class PositionController3D:
    """
    Combines three PID controllers (X, Y, Z translation) to guide a drone
    to a 3D coordinate target.
    """
    def __init__(self, kp_xy=3.0, ki_xy=0.1, kd_xy=2.0, kp_z=4.0, ki_z=0.2, kd_z=2.5):
        # PID parameters
        self.pid_x = PIDController(kp_xy, ki_xy, kd_xy)
        self.pid_y = PIDController(kp_xy, ki_xy, kd_xy)
        self.pid_z = PIDController(kp_z, ki_z, kd_z)
        
    def set_gains(self, kp_xy, ki_xy, kd_xy, kp_z, ki_z, kd_z):
        """Dynamically tunes PID coefficients from GCS HUD sliders."""
        self.pid_x.kp = kp_xy
        self.pid_x.ki = ki_xy
        self.pid_x.kd = kd_xy
        
        self.pid_y.kp = kp_xy
        self.pid_y.ki = ki_xy
        self.pid_y.kd = kd_xy
        
        self.pid_z.kp = kp_z
        self.pid_z.ki = ki_z
        self.pid_z.kd = kd_z

    def update(self, current_pos, target_pos, current_vel, dt):
        """
        Computes desired translational force vector (Fx, Fy, Fz) to drive
        the drone from current_pos to target_pos.
        """
        error = target_pos - current_pos
        
        # Compute force demands
        fx = self.pid_x.update(error[0], current_vel[0], dt)
        fy = self.pid_y.update(error[1], current_vel[1], dt)
        fz = self.pid_z.update(error[2], current_vel[2], dt)
        
        return np.array([fx, fy, fz], dtype=np.float64)
        
    def reset(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
