import numpy as np
from core.swarm_coordinator import SwarmCoordinator

class PathPlanner:
    """
    Trajectory planner that generates sparse, scheduled mission waypoints and times
    for the entire drone swarm at once. This avoids redundant calculations and handles
    minimum spacing safety thresholds and unlit standby modes.
    """
    
    @staticmethod
    def get_circle_waypoints(radius=10.0, height=5.0, num_points=8, total_time=20.0):
        """Generates sparse circular waypoints with timestamps."""
        angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
        waypoints = []
        times = []
        dt = total_time / num_points
        
        for i, angle in enumerate(angles):
            waypoints.append([radius * np.cos(angle), radius * np.sin(angle), height])
            times.append(i * dt)
            
        # Add the closing waypoint to make the loop seamless
        waypoints.append([radius * np.cos(0), radius * np.sin(0), height])
        times.append(total_time)
        
        return waypoints, times, True

    @staticmethod
    def get_figure_eight_waypoints(width=15.0, height=10.0, z_height=5.0, num_points=12, total_time=24.0):
        """Generates sparse Figure-Eight waypoints with timestamps."""
        t_vals = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
        waypoints = []
        times = []
        dt = total_time / num_points
        
        for i, t in enumerate(t_vals):
            waypoints.append([
                width * np.sin(t),
                height * np.sin(2 * t) / 2.0,
                z_height
            ])
            times.append(i * dt)
            
        # Add closing waypoint
        waypoints.append([0.0, 0.0, z_height])
        times.append(total_time)
        
        return waypoints, times, True

    @staticmethod
    def get_helix_waypoints(radius=8.0, start_height=2.0, end_height=12.0, num_turns=2, num_points=16, total_time=25.0):
        """Generates sparse spiraling helix waypoints with timestamps (open path)."""
        angles = np.linspace(0, 2 * np.pi * num_turns, num_points)
        z_vals = np.linspace(start_height, end_height, num_points)
        waypoints = []
        times = []
        dt = total_time / (num_points - 1)
        
        for i in range(num_points):
            waypoints.append([
                radius * np.cos(angles[i]),
                radius * np.sin(angles[i]),
                z_vals[i]
            ])
            times.append(i * dt)
            
        return waypoints, times, False

    @classmethod
    def plan_for_swarm(cls, num_drones, path_type="circle", base_height=5.0):
        """
        Plans schedules (waypoints, times, loop, colors) for all drones in the swarm at once.
        Enforces safety margins using SwarmCoordinator.enforce_spacing. Drones that cannot
        fit are automatically switched off (LEDs off) and put in low hover standby.
        """
        swarm_missions = []
        
        # 1. SHOW CHOREOGRAPHY MODE (Shape morphing show with hover holds & spacing validation)
        if path_type == "show":
            # Timestamps including Hold intervals (14 key waypoint entries)
            times = [0.0, 4.0, 7.0, 15.0, 19.0, 27.0, 31.0, 39.0, 43.0, 51.0, 55.0, 63.0, 66.0, 70.0]
            
            # Generate grids
            grid_base = SwarmCoordinator.get_grid_shape(num_drones, spacing=1.8, height=0.25)
            grid_hover = SwarmCoordinator.get_grid_shape(num_drones, spacing=1.8, height=3.5)
            grid_standby = SwarmCoordinator.get_grid_shape(num_drones, spacing=1.8, height=2.0)
            
            # Master records for each drone
            drone_wps = [[grid_base[i]] for i in range(num_drones)]
            drone_colors = [["#00F2FE"] for i in range(num_drones)] # Takeoff LED starts Cyan
            
            # Takeoff hover phase (t=4s, t=7s)
            for i in range(num_drones):
                drone_wps[i].append(grid_hover[i])
                drone_wps[i].append(grid_hover[i])
                drone_colors[i].append("#00F2FE")
                drone_colors[i].append("#00F2FE")
                
            # Sequence of target shapes, their coordinate generators, and active colors
            shapes_schedule = [
                # Sphere (t=15s, hold until 19s)
                ("sphere", lambda n: SwarmCoordinator.get_sphere_shape(n, radius=8.0, center_height=12.0), "#0055FF"),
                # Star (t=27s, hold until 31s)
                ("star", lambda n: SwarmCoordinator.get_star_shape(n, radius=9.0, center_height=12.0), "#FFD700"),
                # Heart (t=39s, hold until 43s)
                ("heart", lambda n: SwarmCoordinator.get_heart_shape(n, scale=0.55, center_height=12.0), "#FF2A5F"),
                # Pyramid (t=51s, hold until 55s)
                ("pyramid", lambda n: SwarmCoordinator.get_pyramid_shape(n, base_width=11.0, height=9.0, base_height=4.5), "#39FF14")
            ]
            
            for shape_name, shape_gen, shape_color in shapes_schedule:
                # 1. Grab current position of each drone (last assigned waypoint)
                prev_positions = [drone_wps[i][-1] for i in range(num_drones)]
                
                # 2. Generate raw shape coordinates
                raw_shape_points = shape_gen(num_drones)
                
                # 3. Enforce spacing and scale up or prune extra drones
                # We enforce d_min safety margin = 1.4 meters, max scale = 2.0x
                fitted_points, pruned_indices = SwarmCoordinator.enforce_spacing(
                    raw_shape_points, d_min=1.4, max_scale_factor=2.0
                )
                
                M = len(fitted_points)
                
                # 4. Safety Ground Shift: Shift shape upwards if it drops below the 3.5m safety margin
                if M > 0:
                    min_z = min(pt[2] for pt in fitted_points)
                    if min_z < 3.5:
                        z_shift = 3.5 - min_z
                        fitted_points = [np.array([pt[0], pt[1], pt[2] + z_shift]) for pt in fitted_points]
                
                # 4. Construct cost matrix to pair drones to shapes or standby targets
                # If a drone doesn't fit, it is assigned to its own grid column at standby height (1m)
                C = np.zeros((num_drones, num_drones))
                for i in range(num_drones):
                    for j in range(num_drones):
                        if j < M:
                            diff = prev_positions[i] - fitted_points[j]
                            C[i, j] = np.sum(diff * diff)
                        else:
                            standby_idx = j - M
                            if standby_idx == i:
                                diff = prev_positions[i] - grid_standby[i]
                                C[i, j] = np.sum(diff * diff)
                            else:
                                C[i, j] = 1e6 # force diagonal assignment for standby
                                
                # 5. Solve linear sum assignment
                cols = SwarmCoordinator.solve_cost_matrix(C)
                
                # 6. Store waypoints and colors
                for i in range(num_drones):
                    target_col = cols[i]
                    if target_col < M:
                        # Fits in shape: active
                        target_pos = fitted_points[target_col]
                        target_color = shape_color
                    else:
                        # Pruned: Standby unlit hover
                        target_pos = grid_standby[i]
                        target_color = "#151720" # LED OFF color
                        
                    # Add reach waypoint and hold waypoint
                    drone_wps[i].append(target_pos)
                    drone_wps[i].append(target_pos)
                    drone_colors[i].append(target_color)
                    drone_colors[i].append(target_color)
                    
            # Return to landing hover (t=63s, hold until 66s)
            for i in range(num_drones):
                drone_wps[i].append(grid_hover[i])
                drone_wps[i].append(grid_hover[i])
                drone_colors[i].append("#00F2FE")
                drone_colors[i].append("#00F2FE")
                
            # Ground touchdown landing (t=70s)
            for i in range(num_drones):
                drone_wps[i].append(grid_base[i])
                drone_colors[i].append("#8A2BE2") # purple land LED
                
            # Package mission tuples
            for i in range(num_drones):
                swarm_missions.append((drone_wps[i], times, True, drone_colors[i]))
                
            return swarm_missions
            
        # 2. PATH MODES (Circle, Figure-8, Helix)
        neon_colors = ["#00f2fe", "#4facfe", "#39ff14", "#ff2a5f", "#ffdf00", "#ff007f", "#8a2be2", "#ff4500"]
        for i in range(num_drones):
            # Single Drone
            if num_drones == 1:
                if path_type == "circle":
                    wps, times, loop = cls.get_circle_waypoints(radius=8.0, height=base_height, total_time=15.0)
                elif path_type == "figure_eight":
                    wps, times, loop = cls.get_figure_eight_waypoints(width=12.0, height=9.0, z_height=base_height, total_time=18.0)
                else: # helix
                    wps, times, loop = cls.get_helix_waypoints(radius=6.0, start_height=2.0, end_height=10.0, num_turns=2, total_time=20.0)
                    
            # Swarm scale (more than 1 drone)
            else:
                if path_type == "circle":
                    ring_idx = i // 10 if num_drones > 10 else i
                    idx_in_ring = i % 10 if num_drones > 10 else 0
                    rings_count = max(1, num_drones // 10)
                    
                    radius = 4.0 + ring_idx * 1.5
                    height = base_height + (ring_idx - (rings_count-1)/2.0) * 1.0 if num_drones > 10 else base_height
                    
                    base_wps, times, loop = cls.get_circle_waypoints(radius=radius, height=height, total_time=25.0)
                    
                    shift_fraction = idx_in_ring / (10.0 if num_drones > 10 else num_drones)
                    shifted_wps = []
                    num_wps = len(base_wps) - 1
                    
                    for w_idx in range(num_wps):
                        angle = (w_idx / num_wps) * 2 * np.pi + (shift_fraction * 2 * np.pi)
                        shifted_wps.append([radius * np.cos(angle), radius * np.sin(angle), height])
                    shifted_wps.append(list(shifted_wps[0]))
                    wps, times, loop = shifted_wps, times, loop
                    
                elif path_type == "figure_eight":
                    layers_count = max(1, num_drones // 20)
                    layer = i // 20 if num_drones > 20 else i
                    idx_in_layer = i % 20 if num_drones > 20 else 0
                    
                    z = base_height + (layer - (layers_count-1)/2.0) * 1.2
                    base_wps, times, loop = cls.get_figure_eight_waypoints(width=14.0, height=10.0, z_height=z, total_time=25.0)
                    
                    shifted_wps = []
                    num_wps = len(base_wps) - 1
                    phase_offset = (idx_in_layer / (20.0 if num_drones > 20 else num_drones)) * 2 * np.pi
                    
                    for w_idx in range(num_wps):
                        t = (w_idx / num_wps) * 2 * np.pi + phase_offset
                        shifted_wps.append([14.0 * np.sin(t), 10.0 * np.sin(2 * t) / 2.0, z])
                    shifted_wps.append(list(shifted_wps[0]))
                    wps, times, loop = shifted_wps, times, loop
                    
                else: # helix
                    helixes_count = 4
                    helix_id = i // (num_drones // helixes_count) if num_drones >= helixes_count else i
                    idx_in_helix = i % (num_drones // helixes_count) if num_drones >= helixes_count else 0
                    drones_per_helix = num_drones // helixes_count if num_drones >= helixes_count else 1
                    
                    radius = 4.0 + helix_id * 1.8
                    angle_offset = (2 * np.pi / helixes_count) * helix_id + (2 * np.pi / drones_per_helix) * idx_in_helix
                    
                    num_points = 16
                    total_time = 24.0
                    dt = total_time / (num_points - 1)
                    
                    angles = np.linspace(0, 5 * np.pi, num_points) + angle_offset
                    z_start = 2.0 + (idx_in_helix / drones_per_helix) * 2.0
                    z_end = 12.0 + (idx_in_helix / drones_per_helix) * 2.0
                    z_vals = np.linspace(z_start, z_end, num_points)
                    
                    wps = [[radius * np.cos(a), radius * np.sin(a), z] for a, z in zip(angles, z_vals)]
                    times = [t_idx * dt for t_idx in range(num_points)]
                    loop = False
                    
            c_hex = neon_colors[i % len(neon_colors)]
            colors = [c_hex] * len(wps)
            swarm_missions.append((wps, times, loop, colors))
            
        return swarm_missions
