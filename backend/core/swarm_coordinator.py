import numpy as np
from scipy.optimize import linear_sum_assignment

class SwarmCoordinator:
    """
    Generates 3D coordinates for shapes (including complex 3D human footballer & Winnie-the-Pooh bear)
    and solves optimal matching assignments to minimize flight distances.
    """
    
    @staticmethod
    def get_grid_shape(num_drones, spacing=2.5, height=0.25):
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
    def get_sphere_shape(num_drones, radius=10.0, center_height=14.0):
        """Generates 3D sphere points using Fibonacci sphere lattice distribution."""
        points = []
        phi = np.pi * (3.0 - np.sqrt(5.0))  # Golden angle in radians
        
        for i in range(num_drones):
            y = 1.0 - (i / float(num_drones - 1)) * 2.0 if num_drones > 1 else 0.0
            r_at_y = np.sqrt(1.0 - y * y) if num_drones > 1 else 1.0
            
            theta = phi * i
            
            x = np.cos(theta) * r_at_y * radius
            z = y * radius + center_height
            y_coord = np.sin(theta) * r_at_y * radius
            
            points.append(np.array([x, y_coord, z], dtype=np.float64))
            
        return points

    @staticmethod
    def get_footballer_shape(num_drones, scale=1.2, center_height=14.0):
        """
        Generates a 3D silhouette of a human footballer kicking a soccer ball!
        Anatomical distribution: Head, Torso, Legs (kicking pose), Arms, and Soccer Ball.
        """
        points = []
        
        # Allocate drones to body parts
        # Ball: 15%, Head: 10%, Torso: 30%, Kicking Leg: 20%, Support Leg: 15%, Arms: 10%
        n_ball = max(1, int(num_drones * 0.15))
        n_head = max(1, int(num_drones * 0.10))
        n_torso = max(2, int(num_drones * 0.30))
        n_kick_leg = max(1, int(num_drones * 0.20))
        n_supp_leg = max(1, int(num_drones * 0.15))
        n_arms = max(1, num_drones - (n_ball + n_head + n_torso + n_kick_leg + n_supp_leg))
        
        # 1. Soccer Ball (Sphere offset at foot)
        ball_center = np.array([4.5 * scale, 0.0, center_height - 3.0 * scale])
        ball_radius = 1.5 * scale
        phi = np.pi * (3.0 - np.sqrt(5.0))
        for i in range(n_ball):
            y = 1.0 - (i / float(n_ball - 1)) * 2.0 if n_ball > 1 else 0.0
            r_y = np.sqrt(max(0.0, 1.0 - y * y))
            theta = phi * i
            pt = ball_center + np.array([np.cos(theta)*r_y*ball_radius, np.sin(theta)*r_y*ball_radius, y*ball_radius])
            points.append(pt)
            
        # 2. Head (Sphere at top)
        head_center = np.array([-1.0 * scale, 0.0, center_height + 5.5 * scale])
        head_radius = 1.3 * scale
        for i in range(n_head):
            y = 1.0 - (i / float(n_head - 1)) * 2.0 if n_head > 1 else 0.0
            r_y = np.sqrt(max(0.0, 1.0 - y * y))
            theta = phi * i
            pt = head_center + np.array([np.cos(theta)*r_y*head_radius, np.sin(theta)*r_y*head_radius, y*head_radius])
            points.append(pt)
            
        # 3. Torso (Leaning forward cylinder/volume)
        for i in range(n_torso):
            t = i / float(n_torso - 1) if n_torso > 1 else 0.5
            # Torso spine line from hip to neck
            spine = np.array([-2.0 + t * 1.0, 0.0, (center_height + 0.5) + t * 3.8]) * scale
            # Add width/depth around spine
            angle = t * 4 * np.pi
            r_torso = (1.2 - 0.3 * t) * scale
            offset = np.array([np.cos(angle)*r_torso*0.5, np.sin(angle)*r_torso, 0.0])
            points.append(spine + offset)
            
        # 4. Kicking Leg (Extended forward high kick)
        hip_r = np.array([-1.5 * scale, 0.5 * scale, center_height + 0.5 * scale])
        foot_r = np.array([3.2 * scale, 0.2 * scale, center_height - 2.5 * scale])
        for i in range(n_kick_leg):
            t = i / float(n_kick_leg - 1) if n_kick_leg > 1 else 0.5
            pos = (1 - t) * hip_r + t * foot_r
            # Joint bend at knee
            knee_bump = np.array([0.5 * scale * np.sin(t * np.pi), 0.0, 0.8 * scale * np.sin(t * np.pi)])
            points.append(pos + knee_bump)
            
        # 5. Support Leg (Planted back on ground)
        hip_l = np.array([-2.2 * scale, -0.5 * scale, center_height + 0.5 * scale])
        foot_l = np.array([-3.5 * scale, -0.2 * scale, center_height - 6.0 * scale])
        for i in range(n_supp_leg):
            t = i / float(n_supp_leg - 1) if n_supp_leg > 1 else 0.5
            pos = (1 - t) * hip_l + t * foot_l
            points.append(pos)
            
        # 6. Balance Arms (Outstretched for balance)
        shoulder_l = np.array([-1.5 * scale, -1.2 * scale, center_height + 4.0 * scale])
        hand_l = np.array([-4.0 * scale, -3.0 * scale, center_height + 3.0 * scale])
        shoulder_r = np.array([-0.5 * scale, 1.2 * scale, center_height + 4.0 * scale])
        hand_r = np.array([1.5 * scale, 3.0 * scale, center_height + 4.5 * scale])
        
        n_arm_each = max(1, n_arms // 2)
        for i in range(n_arm_each):
            t = i / float(n_arm_each - 1) if n_arm_each > 1 else 0.5
            points.append((1 - t) * shoulder_l + t * hand_l)
            points.append((1 - t) * shoulder_r + t * hand_r)
            
        # Ensure exact count matches num_drones
        while len(points) < num_drones:
            points.append(ball_center + np.random.uniform(-0.5, 0.5, 3))
            
        return points[:num_drones]

    @staticmethod
    def get_pooh_bear_shape(num_drones, scale=1.1, center_height=14.0):
        """
        Generates a 3D silhouette of Winnie-the-Pooh Bear holding a honey pot!
        Anatomical distribution: Chubby Head + Round Ears, Chubby Belly Torso, Arms holding pot, Legs.
        """
        points = []
        
        # Allocate drones
        n_head = max(2, int(num_drones * 0.20))
        n_ears = max(2, int(num_drones * 0.10))
        n_belly = max(4, int(num_drones * 0.40))
        n_pot = max(2, int(num_drones * 0.15))
        n_limbs = max(2, num_drones - (n_head + n_ears + n_belly + n_pot))
        
        phi = np.pi * (3.0 - np.sqrt(5.0))
        
        # 1. Chubby Round Head
        head_center = np.array([0.0, 0.0, center_height + 4.0 * scale])
        head_r = 2.2 * scale
        for i in range(n_head):
            y = 1.0 - (i / float(n_head - 1)) * 2.0 if n_head > 1 else 0.0
            r_y = np.sqrt(max(0.0, 1.0 - y * y))
            theta = phi * i
            pt = head_center + np.array([np.cos(theta)*r_y*head_r, np.sin(theta)*r_y*head_r*0.9, y*head_r])
            points.append(pt)
            
        # 2. Bear Ears (2 smaller spheres on top of head)
        ear_l = head_center + np.array([-1.6 * scale, 0.0, 2.0 * scale])
        ear_r = head_center + np.array([1.6 * scale, 0.0, 2.0 * scale])
        n_ear_each = n_ears // 2
        for i in range(n_ear_each):
            t = phi * i
            points.append(ear_l + np.array([np.cos(t)*0.8*scale, 0.0, np.sin(t)*0.8*scale]))
            points.append(ear_r + np.array([np.cos(t)*0.8*scale, 0.0, np.sin(t)*0.8*scale]))
            
        # 3. Big Chubby Belly (Pear-shaped body)
        belly_center = np.array([0.0, 0.0, center_height - 1.0 * scale])
        belly_r = 3.5 * scale
        for i in range(n_belly):
            y = 1.0 - (i / float(n_belly - 1)) * 2.0 if n_belly > 1 else 0.0
            r_y = np.sqrt(max(0.0, 1.0 - y * y))
            theta = phi * i
            # Pear shape multiplier: wider at bottom
            pear_factor = 1.0 + (0.3 * (1.0 - y))
            pt = belly_center + np.array([np.cos(theta)*r_y*belly_r*pear_factor, np.sin(theta)*r_y*belly_r*pear_factor, y*belly_r*1.1])
            points.append(pt)
            
        # 4. Honey Pot ("HUNNY" pot held in front)
        pot_center = np.array([0.0, 2.5 * scale, center_height - 0.5 * scale])
        pot_r = 1.6 * scale
        for i in range(n_pot):
            y = 1.0 - (i / float(n_pot - 1)) * 2.0 if n_pot > 1 else 0.0
            r_y = np.sqrt(max(0.0, 1.0 - y * y))
            theta = phi * i
            pt = pot_center + np.array([np.cos(theta)*r_y*pot_r, np.sin(theta)*r_y*pot_r, y*pot_r])
            points.append(pt)
            
        # 5. Short Chubby Legs & Paws
        leg_l = np.array([-1.8 * scale, 0.0, center_height - 4.5 * scale])
        leg_r = np.array([1.8 * scale, 0.0, center_height - 4.5 * scale])
        n_leg_each = max(1, n_limbs // 2)
        for i in range(n_leg_each):
            t = i / float(n_leg_each - 1) if n_leg_each > 1 else 0.5
            points.append(leg_l + np.array([0.0, t * 1.0 * scale, -t * 1.5 * scale]))
            points.append(leg_r + np.array([0.0, t * 1.0 * scale, -t * 1.5 * scale]))
            
        while len(points) < num_drones:
            points.append(belly_center + np.random.uniform(-0.5, 0.5, 3))
            
        return points[:num_drones]

    @staticmethod
    def get_pyramid_shape(num_drones, base_width=14.0, height=11.0, base_height=5.0):
        """Generates points distributed over the faces of a square-base pyramid."""
        points = []
        num_base = num_drones // 5
        num_face = (num_drones - num_base) // 4
        num_base += num_drones - (num_base + num_face * 4)
        
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
            
        apex = np.array([0.0, 0.0, base_height + height])
        corners = [
            np.array([-base_width/2, -base_width/2, base_height]),
            np.array([base_width/2, -base_width/2, base_height]),
            np.array([base_width/2, base_width/2, base_height]),
            np.array([-base_width/2, base_width/2, base_height])
        ]
        
        for face_idx in range(4):
            c1 = corners[face_idx]
            c2 = corners[(face_idx + 1) % 4]
            for k in range(num_face):
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
    def solve_optimal_assignment(current_positions, target_positions):
        """Solves linear sum assignment (Hungarian algorithm) to match drones to targets."""
        n = len(current_positions)
        if n == 0:
            return []
            
        cost_matrix = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                diff = current_positions[i] - target_positions[j]
                cost_matrix[i, j] = np.sum(diff * diff)
                
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        return col_ind

    @staticmethod
    def enforce_spacing(points, d_min=2.2, max_scale_factor=2.5):
        """
        Validates minimum distance between shape points.
        Increased default minimum safety spacing d_min to 2.2 meters!
        Attempts to scale up the shape to satisfy spacing.
        """
        n = len(points)
        if n <= 1:
            return points, []

        pts = np.array(points)
        center = np.mean(pts, axis=0)
        
        min_d = float('inf')
        for i in range(n):
            for j in range(i + 1, n):
                d = np.linalg.norm(pts[i] - pts[j])
                if d < min_d:
                    min_d = d
                    
        # 1. Attempt scaling up if points are closer than d_min
        if min_d < d_min and min_d > 1e-5:
            required_scale = d_min / min_d
            scale_factor = min(required_scale, max_scale_factor)
            pts = center + (pts - center) * scale_factor
            
        # 2. Prune points that still violate spacing
        fitted_points = []
        pruned_indices = []
        
        for idx, pt in enumerate(pts):
            fits = True
            for f_pt in fitted_points:
                if np.linalg.norm(pt - f_pt) < d_min:
                    fits = False
                    break
            if fits:
                fitted_points.append(pt)
            else:
                pruned_indices.append(idx)
                
        return [np.array(p) for p in fitted_points], pruned_indices

    @staticmethod
    def solve_cost_matrix(cost_matrix):
        """Solves the assignment problem directly on a pre-computed cost matrix."""
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        return col_ind
