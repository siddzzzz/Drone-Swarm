import numpy as np
from core.swarm_coordinator import SwarmCoordinator

class ShowOrchestrator:
    """
    Master Show Choreographer & Timed Sequence Orchestrator.
    Manages keyframe scheduling, color transitions, shape morphing sequences,
    and automatic takeoff/landing routines for the full drone show.
    """
    
    # Pre-defined show timeline phases (in seconds)
    KEYFRAME_TIMINGS = {
        "TAKEOFF": 4.0,
        "HOVER_INIT": 7.0,
        "FOOTBALLER": 15.0,
        "FOOTBALLER_HOLD": 19.0,
        "POOH_BEAR": 27.0,
        "POOH_BEAR_HOLD": 31.0,
        "HEART": 39.0,
        "HEART_HOLD": 43.0,
        "PYRAMID": 51.0,
        "PYRAMID_HOLD": 55.0,
        "HOVER_LAND": 63.0,
        "HOVER_LAND_HOLD": 66.0,
        "TOUCHDOWN": 70.0
    }

    @classmethod
    def get_show_schedule(cls):
        """Returns the ordered keyframe timestamps for the full 70-second drone show."""
        return [0.0, 4.0, 7.0, 15.0, 19.0, 27.0, 31.0, 39.0, 43.0, 51.0, 55.0, 63.0, 66.0, 70.0]

    @classmethod
    def generate_choreography(cls, num_drones):
        """
        Builds complete multi-phase show mission schedules for all drones in the swarm.
        Returns:
            swarm_missions: list of (waypoints, timestamps, loop_flag, color_schedule) for each drone.
        """
        swarm_missions = []
        times = cls.get_show_schedule()
        
        # 1. Base takeoff grid (0.25m altitude on launch pads)
        grid_base = SwarmCoordinator.get_grid_shape(num_drones, spacing=2.5, height=0.25)
        # 2. Pre-show hover grid (3.5m altitude)
        grid_hover = SwarmCoordinator.get_grid_shape(num_drones, spacing=2.5, height=3.5)
        
        # Track waypoints and LED colors for each drone
        drone_wps = [[grid_base[i]] for i in range(num_drones)]
        drone_colors = [["#00F2FE"] for i in range(num_drones)]  # Cyan takeoff LED
        
        # Phase 1: Arm & Takeoff to Hover Grid (t=4s reach, t=7s hold)
        for i in range(num_drones):
            drone_wps[i].append(grid_hover[i])
            drone_wps[i].append(grid_hover[i])
            drone_colors[i].append("#00F2FE")
            drone_colors[i].append("#00F2FE")
            
        # Phase 2: Sequence of complex 3D show shapes with custom LED color palettes
        shapes_schedule = [
            # 1. 3D Footballer Kicking Soccer Ball (Lime Green)
            ("footballer", lambda n: SwarmCoordinator.get_footballer_shape(n, scale=1.3, center_height=14.0), "#39FF14"),
            # 2. Winnie-the-Pooh Bear with Honey Pot (Gold / Yellow)
            ("pooh_bear", lambda n: SwarmCoordinator.get_pooh_bear_shape(n, scale=1.2, center_height=14.0), "#FFD700"),
            # 3. Pulsing 3D Heart (Crimson / Pink)
            ("heart", lambda n: SwarmCoordinator.get_heart_shape(n, scale=0.65, center_height=14.0), "#FF2A5F"),
            # 4. 3D Pyramid (Cyan / Neon Blue)
            ("pyramid", lambda n: SwarmCoordinator.get_pyramid_shape(n, base_width=14.0, height=11.0, base_height=5.0), "#00F2FE")
        ]
        
        for shape_name, shape_gen, shape_color in shapes_schedule:
            prev_positions = [drone_wps[i][-1] for i in range(num_drones)]
            raw_shape_points = shape_gen(num_drones)
            
            # Enforce safety spacing (d_min = 2.2m)
            fitted_points, pruned_indices = SwarmCoordinator.enforce_spacing(
                raw_shape_points, d_min=2.2, max_scale_factor=3.0
            )
            M = len(fitted_points)
            
            # Safety ground shift (keep lowest point >= 3.5m)
            if M > 0:
                min_z = min(pt[2] for pt in fitted_points)
                if min_z < 3.5:
                    z_shift = 3.5 - min_z
                    fitted_points = [np.array([pt[0], pt[1], pt[2] + z_shift]) for pt in fitted_points]
            
            # Compute nearby perimeter standby positions for unassigned drones
            shape_center = np.mean(fitted_points, axis=0) if M > 0 else np.array([0.0, 0.0, 14.0])
            max_r = max([np.linalg.norm(pt[:2] - shape_center[:2]) for pt in fitted_points]) if M > 0 else 8.0
            standby_radius = max_r + 3.0
            
            perim_angles = np.linspace(0, 2 * np.pi, num_drones, endpoint=False)
            standby_points = []
            for k in range(num_drones):
                stby_x = shape_center[0] + standby_radius * np.cos(perim_angles[k])
                stby_y = shape_center[1] + standby_radius * np.sin(perim_angles[k])
                stby_z = shape_center[2]
                standby_points.append(np.array([stby_x, stby_y, stby_z], dtype=np.float64))
            
            # Distance cost matrix matching
            C = np.zeros((num_drones, num_drones))
            for i in range(num_drones):
                for j in range(num_drones):
                    if j < M:
                        diff = prev_positions[i] - fitted_points[j]
                        C[i, j] = np.sum(diff * diff)
                    else:
                        stby_target = standby_points[j - M]
                        diff = prev_positions[i] - stby_target
                        C[i, j] = np.sum(diff * diff)
                            
            cols = SwarmCoordinator.solve_cost_matrix(C)
            
            for i in range(num_drones):
                target_col = cols[i]
                if target_col < M:
                    target_pos = fitted_points[target_col]
                    target_color = shape_color
                else:
                    target_pos = standby_points[target_col - M]
                    target_color = "#151720"  # LED OFF for nearby standby hover
                    
                # Append reach and hold waypoints & colors
                drone_wps[i].append(target_pos)
                drone_wps[i].append(target_pos)
                drone_colors[i].append(target_color)
                drone_colors[i].append(target_color)
                
        # Phase 3: Return to Landing Hover (t=63s reach, t=66s hold)
        for i in range(num_drones):
            drone_wps[i].append(grid_hover[i])
            drone_wps[i].append(grid_hover[i])
            drone_colors[i].append("#00F2FE")
            drone_colors[i].append("#00F2FE")
            
        # Phase 4: Ground Touchdown Landing (t=70s)
        for i in range(num_drones):
            drone_wps[i].append(grid_base[i])
            drone_colors[i].append("#8A2BE2")  # Purple landing LED
            
        for i in range(num_drones):
            swarm_missions.append((drone_wps[i], times, True, drone_colors[i]))
            
        return swarm_missions

    @classmethod
    def get_show_progress(cls, local_time):
        """
        Calculates active show phase name and percentage completion for telemetry transmission.
        """
        t = local_time % 70.0
        
        if t < 4.0:
            phase = "TAKEOFF"
        elif t < 7.0:
            phase = "HOVER_PRE_SHOW"
        elif t < 19.0:
            phase = "3D_FOOTBALLER"
        elif t < 31.0:
            phase = "3D_POOH_BEAR"
        elif t < 43.0:
            phase = "3D_HEART"
        elif t < 55.0:
            phase = "3D_PYRAMID"
        elif t < 66.0:
            phase = "HOVER_POST_SHOW"
        else:
            phase = "LANDING"
            
        progress_pct = round((t / 70.0) * 100.0, 1)
        return phase, progress_pct
