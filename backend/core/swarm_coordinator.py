import numpy as np
from scipy.optimize import linear_sum_assignment

class SwarmCoordinator:
    """
    Generates 3D coordinates for shapes and solves the optimal matching assignment
    to minimize total transition distance for the drone swarm.
    """
    
    @staticmethod
    def get_grid_shape(num_drones, spacing=2.0, height=0.0):
        """Generates a flat 2D grid of points in the XY plane at a set height."""
        points = []
        cols = int(np.ceil(np.sqrt(num_drones)))
        rows = int(np.ceil(num_drones / cols))
        
        offset_x = (cols - 1) * spacing / 2.0
        offset_y = (rows - 1) * spacing / 2.0
        
        for i in range(num_drones):
            r = i // cols
            c = i % cols
            x = c * spacing - offset_x
            y = r * spacing - offset_y
            points.append(np.array([x, y, height], dtype=np.float64))
            
        return points

    @staticmethod
    def get_sphere_shape(num_drones, radius=8.0, center_height=7.0):
        """Generates 3D sphere points using Fibonacci sphere lattice distribution."""
        points = []
        phi = np.pi * (3.0 - np.sqrt(5.0))  # Golden angle in radians
        
        for i in range(num_drones):
            # y coordinate from 1 to -1
            y = 1.0 - (i / float(num_drones - 1)) * 2.0 if num_drones > 1 else 0.0
            r_at_y = np.sqrt(1.0 - y * y) if num_drones > 1 else 1.0
            
            theta = phi * i
            
            x = np.cos(theta) * r_at_y * radius
            z = y * radius + center_height
            y_coord = np.sin(theta) * r_at_y * radius
            
            points.append(np.array([x, y_coord, z], dtype=np.float64))
            
        return points

    @staticmethod
    def get_heart_shape(num_drones, scale=0.5, center_height=7.0):
        """Generates a vertical 3D heart shape outline/grid in the XZ plane."""
        points = []
        t_vals = np.linspace(0, 2 * np.pi, num_drones, endpoint=False)
        
        for i, t in enumerate(t_vals):
            # Parametric heart equations
            # x = 16 * sin^3(t)
            # z = 13 * cos(t) - 5*cos(2t) - 2*cos(3t) - cos(4t)
            x = 16 * (np.sin(t) ** 3) * scale
            z = (13 * np.cos(t) - 5 * np.cos(2*t) - 2 * np.cos(3*t) - np.cos(4*t)) * scale + center_height
            
            # Spread slightly in the Y dimension (depth) to give it 3D thickness
            y = np.sin(t * 3) * 0.5
            
            points.append(np.array([x, y, z], dtype=np.float64))
            
        return points

    @staticmethod
    def get_pyramid_shape(num_drones, base_width=12.0, height=8.0, base_height=3.0):
        """Generates points distributed over the faces of a square-base pyramid."""
        points = []
        
        # Divide drones between base and 4 faces
        num_base = num_drones // 5
        num_face = (num_drones - num_base) // 4
        # Remainder points added to base
        num_base += num_drones - (num_base + num_face * 4)
        
        # 1. Base grid points (flat square at base_height)
        cols = int(np.ceil(np.sqrt(num_base)))
        rows = int(np.ceil(num_base / cols))
        bx_space = base_width / (cols - 1) if cols > 1 else 0
        by_space = base_width / (rows - 1) if rows > 1 else 0
        
        for i in range(num_base):
            r = i // cols
            c = i % cols
            x = c * bx_space - base_width / 2.0
            y = r * by_space - base_width / 2.0
            points.append(np.array([x, y, base_height], dtype=np.float64))
            
        # 2. Four faces (triangles sloping to apex)
        apex = np.array([0.0, 0.0, base_height + height])
        
        # Face corners
        corners = [
            np.array([-base_width/2, -base_width/2, base_height]),
            np.array([base_width/2, -base_width/2, base_height]),
            np.array([base_width/2, base_width/2, base_height]),
            np.array([-base_width/2, base_width/2, base_height])
        ]
        
        for face_idx in range(4):
            c1 = corners[face_idx]
            c2 = corners[(face_idx + 1) % 4]
            
            # Distribute points on triangular face
            for k in range(num_face):
                # Random/structured triangle point coordinates using barycentric weights
                # u and v are weights
                if num_face > 1:
                    row = int(np.floor(np.sqrt(k)))
                    col = k - row * row
                    max_row = int(np.ceil(np.sqrt(num_face)))
                    u = row / max_row
                    v = (col / (2.0 * row + 1.0)) * (1.0 - u) if row > 0 else 0.0
                else:
                    u, v = 0.33, 0.33
                
                w = 1.0 - u - v
                pos = u * c1 + v * c2 + w * apex
                points.append(pos)
                
        return points

    @staticmethod
    def get_star_shape(num_drones, radius=10.0, center_height=7.0):
        """Generates a vertical 3D star outline in the XZ plane."""
        points = []
        num_tips = 5
        inner_radius = radius * 0.4
        
        for i in range(num_drones):
            fraction = i / float(num_drones)
            angle = fraction * 2 * np.pi
            
            # Determine if point lies on outer tip or inner valley
            tip_angle = (angle * num_tips) % (2 * np.pi)
            if tip_angle < np.pi:
                r = inner_radius + (radius - inner_radius) * (tip_angle / np.pi)
            else:
                r = radius - (radius - inner_radius) * ((tip_angle - np.pi) / np.pi)
                
            x = r * np.cos(angle)
            z = r * np.sin(angle) + center_height
            y = np.cos(angle * 5) * 0.4 # slight ripple thickness
            points.append(np.array([x, y, z], dtype=np.float64))
            
        return points

    @staticmethod
    def solve_optimal_assignment(current_positions, target_positions):
        """
        Solves the linear sum assignment (Hungarian algorithm) to match
        current drone positions to target shape positions.
        Returns a mapping index array.
        """
        n = len(current_positions)
        if n == 0:
            return []
            
        # Compute cost matrix (squared distance to prioritize short flights)
        cost_matrix = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                diff = current_positions[i] - target_positions[j]
                cost_matrix[i, j] = np.sum(diff * diff)
                
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        return col_ind  # mapping: drone_i goes to target_positions[col_ind[i]]
