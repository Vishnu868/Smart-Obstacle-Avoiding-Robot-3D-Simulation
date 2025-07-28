import pybullet as p
import pybullet_data
import numpy as np
import time
import math
import heapq
from collections import defaultdict

class PathOptimizationRobot:
    def __init__(self, start_pos=(-4, -4, 0.1), target_pos=(4, 4, 0.1), render_mode=True):
        # Simulation parameters
        self.start_pos = start_pos
        self.target_pos = target_pos
        self.obstacles = []
        self.robot_id = None
        self.wheels = []
        self.max_velocity = 500  # SIGNIFICANTLY increased max velocity
        self.max_force = 1000    # SIGNIFICANTLY increased force for more powerful movement
        
        # Path planning parameters
        self.grid_size = 0.5     # Increased for faster planning
        self.grid_resolution = int(10 / self.grid_size)
        self.collision_margin = 0.3
        
        # Forward kinematics parameters - EXTREME OPTIMIZATION
        self.wheel_radius = 0.1
        self.wheel_distance = 0.4
        self.steering_angle_max = 0.8     # Increased for sharper turns
        self.steering_speed = 5.0         # Significantly increased for faster steering response
        self.drive_speed = 200            # DRAMATICALLY increased for much faster movement
        self.waypoint_threshold = 0.5     # Increased for faster waypoint reaching
        self.look_ahead = 2               # Increased to make it more forward-looking
        
        # Initialize PyBullet
        if render_mode:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        # Set Top-down Camera View
        p.resetDebugVisualizerCamera(
            cameraDistance=12,       # Distance from target
            cameraYaw=0,            # Initial yaw angle (rotation around z-axis)
            cameraPitch=-89.9,       # Initial pitch angle (nearly -90 for top-down view)
            cameraTargetPosition=[0, 0, 0]  # Target position (center of the scene)
        )

        # Enable GUI controls for the camera
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 1)
        
        # Add keyboard shortcuts info on screen
        help_text = """
        Camera Controls:
        - Mouse + Left-click: Rotate view
        - Mouse + Right-click: Pan view
        - Mouse wheel: Zoom in/out
        - Ctrl + Mouse: More precise control
        
        SPEED-OPTIMIZED MODE ENABLED
        """
        self.help_id = p.addUserDebugText(help_text, [0, 0, 2], textColorRGB=[1, 1, 0], textSize=1.5)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)
        p.setRealTimeSimulation(0)
        
        # Optimize simulation parameters for speed
        p.setPhysicsEngineParameter(fixedTimeStep=1.0/240.0)
        p.setPhysicsEngineParameter(numSolverIterations=5)   # Further reduced for speed
        
        self.path_visual_ids = []
        # Setup environment
        self.setup_environment()
        self.load_robot()
        
        # Debug mode for visualization - TURN OFF FOR SPEED
        self.debug = False
        
        # Path visualization handles for cleanup
        self.path_visual_ids = []
        
        # Path completion tracking
        self.path_completion_threshold = 0.5  # Increased to consider waypoint reached sooner
        
    def setup_environment(self):
        # Load ground plane
        p.loadURDF("plane.urdf")
        
        # Create target visualization (red sphere)
        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=0.2,
            rgbaColor=[1, 0, 0, 0.7]
        )
        target_id = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=[self.target_pos[0], self.target_pos[1], 0.2]
        )
        self.path_visual_ids.append(target_id)
        
        # Create obstacles
        self.create_obstacles()
        
    def create_obstacles(self):
        # Create several box obstacles - spread out to ensure path existence
        obstacle_positions = [
            (0, 0, 0.5),      # Center
            (2, 2, 0.5),      # Quadrant 1
            (-2, 2, 0.5),     # Quadrant 2
            (-2, -2, 0.5),    # Quadrant 3
            (2, -2, 0.5),     # Quadrant 4
            (0, 3, 0.5),      # North
            (3, 0, 0.5),      # East
            (0, -3, 0.5),     # South
            (-3, 0, 0.5)      # West
        ]
        
        for i, pos in enumerate(obstacle_positions):
            size = 0.4
            height = 1.0
            
            collision_shape_id = p.createCollisionShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[size, size, height/2]
            )
            
            visual_shape_id = p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[size, size, height/2],
                rgbaColor=[0.5, 0.5, 0.5, 1]
            )
            
            obstacle_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=collision_shape_id,
                baseVisualShapeIndex=visual_shape_id,
                basePosition=[pos[0], pos[1], pos[2]]
            )
            
            self.obstacles.append((obstacle_id, pos, size))
    
    def load_robot(self):
        # Create a simple car model
        self.robot_id = p.loadURDF("racecar/racecar.urdf", 
                                  list(self.start_pos), 
                                  p.getQuaternionFromEuler([0, 0, 0]))
        
        # Get wheel joints
        num_joints = p.getNumJoints(self.robot_id)
        self.wheels = []
        
        # Store joint information
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            
            if 'wheel' in joint_name:
                self.wheels.append(i)
        
        # Set the steering and driving links correctly based on the racecar URDF
        self.steering_links = [0, 2]  # Front left and right steering hinges
        self.driving_links = [5, 7]   # Rear wheels
        
        # Make sure vehicle is properly placed
        start_pos_adjusted = (self.start_pos[0], self.start_pos[1], 0.1)
        p.resetBasePositionAndOrientation(
            self.robot_id,
            list(start_pos_adjusted),
            p.getQuaternionFromEuler([0, 0, 0])
        )
        
        # Initialize joint parameters with higher defaults for speed
        for wheel in self.wheels:
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=wheel,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=0,
                force=0
            )
        
        # Set damping for all joints to improve stability but allow faster movement
        for i in range(num_joints):
            p.changeDynamics(self.robot_id, i, linearDamping=0.01, angularDamping=0.01)  # Further reduced damping
            # Reduce friction for faster movement
            p.changeDynamics(self.robot_id, i, lateralFriction=0.3)  # Further lowered friction
    
    def is_collision_free(self, x, y):
        # Check if a grid cell is collision-free
        for obs_id, pos, size in self.obstacles:
            # Simple collision check with margin
            if (abs(x - pos[0]) < size + self.collision_margin and
                abs(y - pos[1]) < size + self.collision_margin):
                return False
        return True
    
    def draw_grid_cell(self, x, y, color):
        # Debug visualization - draw grid cells
        if not self.debug:
            return
            
        size = self.grid_size / 2
        p.addUserDebugLine([x-size, y-size, 0.05], [x+size, y-size, 0.05], color, 1, 0)
        p.addUserDebugLine([x+size, y-size, 0.05], [x+size, y+size, 0.05], color, 1, 0)
        p.addUserDebugLine([x+size, y+size, 0.05], [x-size, y+size, 0.05], color, 1, 0)
        p.addUserDebugLine([x-size, y+size, 0.05], [x-size, y-size, 0.05], color, 1, 0)
    
    def reconstruct_path(self, came_from, current):
        # Reconstruct path from A* search
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]  # Return reversed path
    
    def heuristic(self, a, b):
        # Euclidean distance heuristic
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def a_star_search(self):
        # Discretize start and goal positions to grid
        start = (int(self.start_pos[0] / self.grid_size),
                int(self.start_pos[1] / self.grid_size))
        goal = (int(self.target_pos[0] / self.grid_size),
               int(self.target_pos[1] / self.grid_size))
        
        print(f"Start grid position: {start}")
        print(f"Goal grid position: {goal}")
        
        # A* algorithm implementation
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        came_from = {}
        g_score = defaultdict(lambda: float('inf'))
        g_score[start] = 0
        
        f_score = defaultdict(lambda: float('inf'))
        f_score[start] = self.heuristic(start, goal)
        
        open_set_hash = {start}
        
        # 8-connected grid neighbors
        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0),
                     (1, 1), (1, -1), (-1, 1), (-1, -1)]
        
        # Skip grid visualization for speed
        if self.debug:
            # Just visualize the perimeter for reference
            for i in range(-10, 11, 20):
                for j in range(-10, 10):
                    self.draw_grid_cell(i * self.grid_size, j * self.grid_size, [0, 0.3, 0])
                    self.draw_grid_cell(j * self.grid_size, i * self.grid_size, [0, 0.3, 0])
        
        # Debug info
        tested_cells = 0
        
        while open_set:
            _, current = heapq.heappop(open_set)
            open_set_hash.remove(current)
            tested_cells += 1
            
            # Visualize cells much less frequently for speed
            if self.debug and tested_cells % 50 == 0:
                self.draw_grid_cell(
                    current[0] * self.grid_size, 
                    current[1] * self.grid_size, 
                    [0, 0, 1]
                )
            
            if current == goal:
                path = self.reconstruct_path(came_from, current)
                print(f"Path found after testing {tested_cells} cells!")
                return path
            
            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Check if this neighbor is valid
                grid_x = neighbor[0] * self.grid_size
                grid_y = neighbor[1] * self.grid_size
                
                # Skip if out of bounds (make sure this matches the environment size)
                if (abs(grid_x) > 10 or abs(grid_y) > 10):
                    continue
                
                # Skip if collision
                if not self.is_collision_free(grid_x, grid_y):
                    continue
                
                # Compute movement cost (diagonal moves cost more)
                move_cost = 1
                if dx != 0 and dy != 0:
                    move_cost = 1.414  # sqrt(2)
                
                tentative_g_score = g_score[current] + move_cost
                
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    
                    if neighbor not in open_set_hash:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
                        open_set_hash.add(neighbor)
        
        # No path found
        print(f"No path found after testing {tested_cells} cells.")
        return None
    
    def smooth_path(self, path, smoothing_factor=0.3):  # Increased smoothing for better curves
        """Apply path smoothing to reduce sharp turns"""
        if len(path) <= 2:
            return path
            
        # Apply more aggressive path pruning to remove unnecessary waypoints
        # Only keep points that represent significant direction changes
        pruned_path = [path[0]]  # Keep start point
        
        for i in range(1, len(path) - 1):
            prev = path[i-1]
            current = path[i]
            next_pt = path[i+1]
            
            # Calculate vectors between points
            v1 = (current[0] - prev[0], current[1] - prev[1])
            v2 = (next_pt[0] - current[0], next_pt[1] - current[1])
            
            # Calculate change in direction
            dot_product = v1[0]**v2[0] + v1[1]**v2[1]
            v1_mag = math.sqrt(v1[0]**2 + v1[1]**2)
            v2_mag = math.sqrt(v2[0]**2 + v2[1]**2)
            
            # Skip points where direction doesn't change much
            if v1_mag > 0 and v2_mag > 0:
                cos_angle = dot_product / (v1_mag * v2_mag)
                # Keep point only if angle is significant (less than cos(15°) ≈ 0.97)
                if cos_angle < 0.97:
                    pruned_path.append(current)
            else:
                pruned_path.append(current)
        
        pruned_path.append(path[-1])  # Keep end point
        
        # Apply smoothing to the pruned path
        smoothed_path = [pruned_path[0]]  # Keep the start point
        
        # Apply smoothing to intermediate points
        for i in range(1, len(pruned_path) - 1):
            prev = pruned_path[i-1]
            current = pruned_path[i]
            next_pt = pruned_path[i+1]
            
            # Calculate smoothed point
            smoothed_x = current[0] + smoothing_factor * (prev[0] + next_pt[0] - 2 * current[0])
            smoothed_y = current[1] + smoothing_factor * (prev[1] + next_pt[1] - 2 * current[1])
            
            smoothed_path.append((smoothed_x, smoothed_y))
            
        smoothed_path.append(pruned_path[-1])  # Keep the end point
        
        # Add direct fast path points for long straight segments
        final_path = [smoothed_path[0]]
        
        for i in range(1, len(smoothed_path)):
            prev = smoothed_path[i-1]
            current = smoothed_path[i]
            
            # Calculate distance between points
            dist = math.sqrt((current[0] - prev[0])*2 + (current[1] - prev[1])*2)
            
            # For long segments, add intermediate points to help with path following
            if dist > 2.0:
                # Number of intermediate points based on distance
                num_points = min(2, int(dist / 2.0))
                
                for j in range(1, num_points + 1):
                    ratio = j / (num_points + 1)
                    x = prev[0] + ratio * (current[0] - prev[0])
                    y = prev[1] + ratio * (current[1] - prev[1])
                    final_path.append((x, y))
            
            final_path.append(current)
        
        return final_path
    
    def visualize_path(self, path):
        # Clear existing path visualization
        if self.path_visual_ids:  # Check if list is not empty
            target_id = self.path_visual_ids[0] if self.path_visual_ids else None
            for visual_id in self.path_visual_ids[1:]:  # Skip the first one (target sphere)
                p.removeBody(visual_id)
            self.path_visual_ids = [target_id] if target_id else []  # Keep only the target sphere if it exists
    
        # Visualize the planned path with small spheres and line segments
        for i, (grid_x, grid_y) in enumerate(path):
            x = grid_x * self.grid_size
            y = grid_y * self.grid_size
            
            # Create a small sphere to represent the path (different color for final waypoint)
            color = [0, 1, 0, 0.7]  # Green for normal waypoints
            if i == len(path) - 1:
                color = [1, 0.5, 0, 0.9]  # Orange for final waypoint
               
            # Only visualize every other waypoint for cleaner display
            if i % 2 == 0 or i == len(path) - 1:
                visual_shape_id = p.createVisualShape(
                    shapeType=p.GEOM_SPHERE,
                    radius=0.07 if i % 4 == 0 else 0.05,  # Alternate sizes for visibility
                    rgbaColor=color
                )
            
                body_id = p.createMultiBody(
                    baseMass=0,
                    baseVisualShapeIndex=visual_shape_id,
                    basePosition=[x, y, 0.1]
                )
                self.path_visual_ids.append(body_id)
             
            # Draw line segments connecting waypoints
            if i > 0:
                prev_x = path[i-1][0] * self.grid_size
                prev_y = path[i-1][1] * self.grid_size
                line_id = p.addUserDebugLine(
                    [prev_x, prev_y, 0.1],
                    [x, y, 0.1],
                    [0, 1, 0],
                    lineWidth=3  # Thicker lines for better visibility
                )
    
    def get_robot_state(self):
        """Get current robot position, orientation and velocity"""
        position, orientation = p.getBasePositionAndOrientation(self.robot_id)
        linear_vel, angular_vel = p.getBaseVelocity(self.robot_id)
        
        # Convert quaternion to Euler angles
        euler = p.getEulerFromQuaternion(orientation)
        
        return {
            'position': position,
            'orientation': orientation,
            'euler': euler,  # Roll, pitch, yaw
            'linear_velocity': linear_vel,
            'angular_velocity': angular_vel,
            'yaw': euler[2]  # Direct access to yaw for convenience
        }
    
    def calculate_steering_angle(self, current_pos, current_yaw, target_pos):
        """Calculate desired steering angle to reach target"""
        # Vector to target
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        
        # Angle to target
        target_angle = math.atan2(dy, dx)
        
        # Calculate difference in angles (normalized to [-pi, pi])
        angle_diff = target_angle - current_yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        # Proportional control for steering - IMPROVED GAIN
        steering = angle_diff * 3.0  # Significantly increased gain for more responsive steering
            
        # Clamp to max steering angle
        return max(-self.steering_angle_max, min(self.steering_angle_max, steering))
    
    def get_look_ahead_target(self, world_path, current_waypoint, current_pos):
        """Get a target point ahead on the path for smoother navigation"""
        # If we're very close to the start, ensure we're heading in the right direction
        if current_waypoint <= 1:
            return world_path[1]  # Always target the first actual waypoint initially
        
        # Look ahead more waypoints for smoother path following
        look_ahead_idx = min(current_waypoint + self.look_ahead, len(world_path) - 1)
        
        # We want to make sure we hit every waypoint when close to the end
        if look_ahead_idx >= len(world_path) - 2:
            # When near the end, target precisely
            return world_path[current_waypoint]
            
        # Get the look-ahead waypoint
        target = world_path[look_ahead_idx]
        
        # Draw debug visualization
        if self.debug:
            # Highlight current target waypoint in yellow
            target_visual_shape_id = p.createVisualShape(
                shapeType=p.GEOM_SPHERE,
                radius=0.1,
                rgbaColor=[1, 1, 0, 0.9]
            )
            
            target_body_id = p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=target_visual_shape_id,
                basePosition=[target[0], target[1], 0.15]
            )
            self.path_visual_ids.append(target_body_id)
            
            # Draw a line from robot to target
            p.addUserDebugLine(
                [current_pos[0], current_pos[1], 0.2],
                [target[0], target[1], 0.2],
                [1, 1, 0],
                lineWidth=3,
                lifeTime=0.1
            )
            
        return target
    
    def apply_forward_kinematics(self, target_pos, debug=False):
        """Apply forward kinematics to control the robot to move toward target"""
        # Get current state
        state = self.get_robot_state()
        current_pos = state['position']
        current_yaw = state['yaw']
        
        # Calculate distance to target
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate steering angle
        steering_angle = self.calculate_steering_angle(current_pos, current_yaw, target_pos)
        
        if debug:
            print(f"Current yaw: {current_yaw:.2f}, Target angle: {math.atan2(dy, dx):.2f}")
            print(f"Steering angle: {steering_angle:.2f}")
            print(f"Target position: ({target_pos[0]:.2f}, {target_pos[1]:.2f})")
            print(f"Current position: ({current_pos[0]:.2f}, {current_pos[1]:.2f})")
            print(f"Distance: {distance:.2f}")
        
        # Apply steering to front wheels - faster response
        for steering_link in self.steering_links:
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=steering_link,
                controlMode=p.POSITION_CONTROL,
                targetPosition=steering_angle,
                force=self.max_force,  # Max force for faster steering
                maxVelocity=10.0  # Significantly increased for faster response
            )
        
        # Calculate drive velocity based on distance and steering angle
        # Less speed reduction when turning to maintain momentum
        turn_factor = 1.0 - (abs(steering_angle) / self.steering_angle_max) * 0.3  # Much less reduction
        drive_velocity = self.drive_speed * turn_factor
        
        # Only slow down when extremely close to target
        if distance < 0.3:  # Only slow down when very close
            drive_velocity *= max(0.5, distance * 3)  # Smoother slowing down
        
        # Ensure minimum velocity to prevent getting stuck
        if drive_velocity < 20.0:  # Higher minimum speed
            drive_velocity = 20.0
        
        # Apply velocity to wheels
        for wheel in self.wheels:
            # Skip steering joints
            if wheel in self.steering_links:
                continue
                
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=wheel,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=drive_velocity,
                force=self.max_force
            )
        
        # Debug visualization
        if self.debug:
            # Draw a line showing the current heading
            p.addUserDebugLine(
                [current_pos[0], current_pos[1], 0.2],
                [current_pos[0] + math.cos(current_yaw), current_pos[1] + math.sin(current_yaw), 0.2],
                [1, 0, 0],
                lineWidth=2,
                lifeTime=0.1
            )
            
            # Draw a line showing the desired heading
            target_angle = math.atan2(dy, dx)
            p.addUserDebugLine(
                [current_pos[0], current_pos[1], 0.2],
                [current_pos[0] + math.cos(target_angle), current_pos[1] + math.sin(target_angle), 0.2],
                [0, 0, 1],
                lineWidth=2,
                lifeTime=0.1
            )
        
        return distance
    
    def follow_path(self, world_path):
        """Follow the path using forward kinematics"""
        # Start with the first waypoint
        current_waypoint = 1  # Start from the second point as first is start position
        waypoint_timeout = 0
        max_waypoint_timeout = 50  # Reduce timeout to be more responsive
        
        # Give robot a moment to settle
        for _ in range(5):  # Minimal settling time
            p.stepSimulation()
            time.sleep(0.005)
        
        print("Starting path following with OPTIMIZED forward kinematics...")
        
        # Initial state information
        state = self.get_robot_state()
        print(f"Initial position: ({state['position'][0]:.2f}, {state['position'][1]:.2f})")
        print(f"Initial orientation (yaw): {state['yaw']:.2f}")
        print(f"First waypoint: ({world_path[current_waypoint][0]:.2f}, {world_path[current_waypoint][1]:.2f})")
        print(f"Total waypoints: {len(world_path)}")
        
        simulation_steps = 0
        stabilization_time = 5  # Minimal stabilization time
        
        # Make sure the robot is aligned to the path initially
        if len(world_path) > 1:
            initial_target = world_path[1]
            # Set initial orientation to face the first waypoint
            dx = initial_target[0] - self.start_pos[0]
            dy = initial_target[1] - self.start_pos[1]
            initial_yaw = math.atan2(dy, dx)
            
            # Reset the robot with the correct orientation
            p.resetBasePositionAndOrientation(
                self.robot_id,
                [self.start_pos[0], self.start_pos[1], 0.1],
                p.getQuaternionFromEuler([0, 0, initial_yaw])
            )
            
            # Let physics stabilize
            for _ in range(stabilization_time):
                p.stepSimulation()
                time.sleep(0.001)  # Minimal sleep time
        
        # Track time since last progress to detect and fix getting stuck
        last_progress_time = 0
        last_position = state['position']
        
        # Main loop for path following
        while current_waypoint < len(world_path):
            # Get current waypoint
            waypoint = world_path[current_waypoint]
            
            # Get a look-ahead target
            state = self.get_robot_state()
            current_pos = state['position']
            target_pos = self.get_look_ahead_target(world_path, current_waypoint, current_pos)
            
            # Debug output less frequently for speed
            debug_now = simulation_steps % 50 == 0
            
            # Apply forward kinematics to move towards target
            distance = self.apply_forward_kinematics(target_pos, debug=debug_now)
            
            # Step simulation
            p.stepSimulation()
            
            # Only add delay every several steps for performance
            if simulation_steps % 10 == 0:  # Reduced delay frequency
                time.sleep(0.001)  # Minimal delay
            
            simulation_steps += 1
            
            # Debug output for monitoring
            if distance < self.waypoint_threshold:
                # Waypoint reached, move to next
                current_waypoint += 1
                waypoint_timeout = 0
                
                if current_waypoint < len(world_path):
                    next_waypoint = world_path[current_waypoint]
                    if debug_now:
                        print(f"Reached waypoint! Moving to next: ({next_waypoint[0]:.2f}, {next_waypoint[1]:.2f})")
                else:
                    print("Reached final waypoint!")
                    break
            else:
                waypoint_timeout += 1
                
                # Detect if robot is stuck
                if waypoint_timeout > max_waypoint_timeout:
                    # Calculate if we've made progress since last check
                    current_pos_np = np.array(current_pos[:2])  # Just x, y
                    last_pos_np = np.array(last_position[:2])
                    movement = np.linalg.norm(current_pos_np - last_pos_np)
                    
                    if movement < 0.05:  # Very little movement
                        # We're stuck, try to skip this waypoint if not the last one
                        if current_waypoint < len(world_path) - 1:
                            current_waypoint += 1
                            waypoint_timeout = 0
                            if debug_now:
                                print(f"Robot appears stuck! Skipping to waypoint {current_waypoint}")
                        else:
                            # If we're stuck on the last waypoint, try a different approach
                            # Directly target the final destination
                            if debug_now:
                                print("Targeting final destination directly!")
                            distance = self.apply_forward_kinematics(self.target_pos, debug=debug_now)
                            
                            # If close enough to final target, consider done
                            if distance < self.waypoint_threshold * 1.5:
                                print("Close enough to final target!")
                                break
                    
                    # Reset timeout to give more time
                    waypoint_timeout = 0
            
            # Update last position every 20 steps
            if simulation_steps % 20 == 0:
                last_position = current_pos
                
            # Safety check for maximum time
            if simulation_steps > 5000:  # Increased limit for complex paths
                print("Maximum simulation steps reached, terminating.")
                break
                
        # Final approach to target if not exactly there
        state = self.get_robot_state()
        final_distance = math.sqrt(
            (state['position'][0] - self.target_pos[0])**2 +
            (state['position'][1] - self.target_pos[1])**2
        )
        
        # If we're not quite at the target, do a final approach
        if final_distance > 0.5:
            print(f"Final approach to target, distance: {final_distance:.2f}")
            
            # Direct approach to final target
            for _ in range(100):  # More steps for final approach
                distance = self.apply_forward_kinematics(self.target_pos)
                p.stepSimulation()
                time.sleep(0.001)
                
                # Check if we've reached the target
                if distance < 0.5:
                    print("Target reached on final approach!")
                    break
                    
        # Stop the robot
        for wheel in self.wheels:
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=wheel,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=0,
                force=self.max_force
            )
            
        # Let physics stabilize and robot stop
        for _ in range(20):
            p.stepSimulation()
            time.sleep(0.01)
            
        # Final position
        state = self.get_robot_state()
        final_distance = math.sqrt(
            (state['position'][0] - self.target_pos[0])**2 +
            (state['position'][1] - self.target_pos[1])**2
        )
        
        print(f"Path following complete! Final distance to target: {final_distance:.2f}")
        print(f"Total simulation steps: {simulation_steps}")
        
        return final_distance < 1.0  # Success if within 1.0 units
        
    def plan_and_execute(self):
        """Plan a path and execute it"""
        print("Starting path planning...")
        
        # Plan path with A*
        start_time = time.time()
        path = self.a_star_search()
        planning_time = time.time() - start_time
        
        if not path:
            print("No path found!")
            return False
            
        print(f"Path planning completed in {planning_time:.2f} seconds.")
        print(f"Path length: {len(path)} waypoints")
        
        # Smooth the path
        print("Smoothing path...")
        smoothed_path = self.smooth_path(path)
        print(f"Smoothed path length: {len(smoothed_path)} waypoints")
        
        # Convert grid path to world coordinates
        world_path = [(pt[0] * self.grid_size, pt[1] * self.grid_size) for pt in smoothed_path]
        
        # Visualize path
        print("Visualizing path...")
        self.visualize_path(smoothed_path)
        
        # Follow the path
        print("Following path...")
        start_time = time.time()
        success = self.follow_path(world_path)
        execution_time = time.time() - start_time
        
        print(f"Path execution completed in {execution_time:.2f} seconds.")
        print(f"Success: {success}")
        
        return success
        
    def run(self):
        """Run the full demo"""
        success = self.plan_and_execute()
        
        # Let the simulation run a bit longer
        if success:
            print("Mission accomplished! Robot reached the target.")
            # Add celebratory movement - spin in place
            for _ in range(50):
                p.setJointMotorControl2(
                    bodyUniqueId=self.robot_id,
                    jointIndex=self.wheels[0],
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=100,
                    force=self.max_force
                )
                p.setJointMotorControl2(
                    bodyUniqueId=self.robot_id,
                    jointIndex=self.wheels[1],
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=-100,
                    force=self.max_force
                )
                p.stepSimulation()
                time.sleep(0.01)
        else:
            print("Mission failed! Robot couldn't reach the target.")
            
        # Stop the robot
        for wheel in self.wheels:
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=wheel,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=0,
                force=self.max_force
            )
            
        # Run for a while to allow user to inspect the result
        print("Simulation complete. Press Ctrl+C to exit.")
        try:
            while True:
                p.stepSimulation()
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Exiting...")
            
    def cleanup(self):
        """Clean up the simulation"""
        p.disconnect()

# Fix the class initialization method name
if __name__ == "__main__":
    # Create and run the robot - initialization happens automatically here
    robot = PathOptimizationRobot()
    try:
        robot.run()
    finally:
        robot.cleanup()