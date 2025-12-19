import math
import rclpy
import numpy as np
import heapq
import time
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

# Map grid cell states
FREE = 0
UNKNOWN = -1
OCCUPIED = 100

# ================ CONFIGURATION PARAMETERS ================
# Navigation parameters
SPEED = 0.15                     # Forward speed (m/s)
TURN_SPEED = 0.5                 # Rotation speed (rad/s)
GOAL_TOLERANCE = 0.15            # Distance threshold for goal achievement (m)
SAFETY_DISTANCE = 0.25           # Minimum clearance from obstacles (m)

# Frontier exploration parameters
MIN_FRONTIER_DISTANCE = 0.4      # Minimum distance to consider a frontier (m)
MAX_FRONTIER_DISTANCE = 2.5      # Maximum search radius for frontiers (m)

# Goal adjustment parameters
NUDGE_RADIUS_M = 0.35            # Radius for wall proximity check (m)
NUDGE_GAIN = 0.20                # Strength of wall repulsion force
NUDGE_MAX_STEPS = 5              # Maximum iterations for goal adjustment

# Recovery behavior parameters
STUCK_DIST_M = 0.03              # Movement threshold for stuck detection (m)
STUCK_TIME_S = 2.0               # Time window for stuck detection (s)
BACKUP_TIME_S = 1.2              # Duration of backward movement during recovery (s)
BACKUP_SPEED = -0.08             # Speed during backward movement (m/s)
TURN_TIME_S = 1.4                # Duration of rotation during recovery (s)
TURN_SPEED_RECOVERY = 0.9        # Rotation speed during recovery (rad/s)
RECOVERY_COOLDOWN_S = 1.0        # Minimum time between recovery triggers (s)
MIN_FRONT_CLEAR_M = 0.35         # Required frontal clearance to exit recovery (m)


class FrontierExplorer(Node):
    """
    Autonomous frontier exploration node for mobile robots.
    
    This node implements frontier-based exploration using:
    1. Occupancy grid mapping for environment representation
    2. A* path planning for collision-free navigation
    3. TF-based localization in the map frame
    4. Recovery behaviors for handling stuck situations
    """
    
    def __init__(self):
        """Initialize the frontier exploration node with subscribers and publishers."""
        super().__init__('frontier_explorer')
        
        # Node initialization banner
        self.get_logger().info("=" * 60)
        self.get_logger().info("FRONTIER EXPLORATION NODE INITIALIZATION")
        self.get_logger().info("=" * 60)
        
        # ============ SUBSCRIBERS ============
        # Map subscription for environment representation
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        
        # Odometry subscription (used as fallback localization)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # Laser scan subscription for obstacle detection
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        
        # ============ PUBLISHERS ============
        # Velocity commands for robot control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        
        # Frontier goal visualization
        self.goal_pub = self.create_publisher(PoseStamped, '/frontier_goal', 10)
        
        # Path visualization for debugging
        self.path_pub = self.create_publisher(Path, '/a_star_path', 10)
        
        # ============ LOCALIZATION ============
        # TF buffer for map frame localization
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # ============ EXPLORATION STATE ============
        self.map_data = None                     # Current occupancy grid
        self.robot_x_map = 0.0                   # Robot X position in map frame
        self.robot_y_map = 0.0                   # Robot Y position in map frame
        self.robot_yaw_map = 0.0                 # Robot orientation in map frame
        self.scan_ranges = None                  # Latest laser scan data
        
        # Goal management
        self.current_goal = None                 # Current target position (x, y)
        self.goal_reached = True                 # Goal completion status
        self.avoiding_obstacle = False           # Obstacle avoidance flag
        self.planning_counter = 0                # Planning cycle counter
        
        # Stuck detection and recovery
        self.last_pose_check_time = time.time()  # Last pose check timestamp
        self.last_pose_check = None              # Last recorded position
        self.stuck_start_time = None             # Time when stuck condition began
        
        # Recovery state machine
        self.recovery_state = "NONE"             # Current recovery state
        self.recovery_until = 0.0                # Recovery action end time
        self.recovery_turn_dir = 1               # Rotation direction (+1: left, -1: right)
        self.last_recovery_time = 0.0            # Last recovery activation time
        
        # Path following
        self.path_waypoints = None               # Waypoints from A* path
        self.path_index = 0                      # Current waypoint index
        
        # ============ TIMERS ============
        # Control loop for navigation (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Planning loop for frontier selection (0.5 Hz)
        self.planning_timer = self.create_timer(2.0, self.planning_loop)
        
        self.get_logger().info("Frontier explorer node initialized successfully")
    
    # ============ PATH PLANNING METHODS ============
    
    def astar(self, start, goal, blocked):
        """
        A* pathfinding algorithm for grid-based navigation.
        
        Args:
            start: (x, y) grid coordinates of starting position
            goal: (x, y) grid coordinates of target position
            blocked: 2D boolean array indicating occupied cells
            
        Returns:
            List of grid coordinates forming the path, or None if no path exists
        """
        sx, sy = start
        gx, gy = goal
        height, width = blocked.shape
        
        def in_bounds(x, y):
            """Check if coordinates are within grid boundaries."""
            return 0 <= x < width and 0 <= y < height
        
        def heuristic(x, y):
            """Manhattan distance heuristic for A* algorithm."""
            return abs(x - gx) + abs(y - gy)
        
        # 8-connected movement directions (allows diagonal movement)
        moves = [
            (-1, 0), (1, 0), (0, -1), (0, 1),      # Cardinal directions
            (-1, -1), (-1, 1), (1, -1), (1, 1)     # Diagonal directions
        ]
        
        # Priority queue for open set (f_score, g_score, position)
        openq = []
        heapq.heappush(openq, (heuristic(sx, sy), 0, (sx, sy)))
        
        # Navigation data structures
        came_from = {(sx, sy): None}      # Parent nodes for path reconstruction
        g_score = {(sx, sy): 0}           # Cost from start to each node
        
        while openq:
            # Get node with lowest f_score
            f, g, (x, y) = heapq.heappop(openq)
            
            # Check if goal reached
            if (x, y) == (gx, gy):
                # Reconstruct path from goal to start
                path = []
                current = (x, y)
                while current is not None:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path
            
            # Explore neighbors
            for dx, dy in moves:
                nx, ny = x + dx, y + dy
                
                # Skip if out of bounds or blocked
                if not in_bounds(nx, ny) or blocked[ny, nx]:
                    continue
                
                # Movement cost (1.0 for cardinal, √2 for diagonal)
                step_cost = 1.0 if (dx == 0 or dy == 0) else 1.4142
                tentative_g = g_score[(x, y)] + step_cost
                
                # Update if we found a better path to this neighbor
                if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = tentative_g
                    came_from[(nx, ny)] = (x, y)
                    f_score = tentative_g + heuristic(nx, ny)
                    heapq.heappush(openq, (f_score, tentative_g, (nx, ny)))
        
        # No path found
        return None
    
    def make_inflated_blocked(self, data, inflation_cells):
        """
        Create obstacle inflation for safety margin.
        
        Args:
            data: 2D occupancy grid data
            inflation_cells: Number of cells to inflate obstacles
            
        Returns:
            2D boolean array where True indicates blocked space
        """
        height, width = data.shape
        blocked = np.zeros((height, width), dtype=bool)
        occupied = (data == OCCUPIED)
        
        # Get coordinates of all occupied cells
        occupied_y, occupied_x = np.where(occupied)
        
        # Inflate each occupied cell
        for y, x in zip(occupied_y, occupied_x):
            y_min = max(0, y - inflation_cells)
            y_max = min(height, y + inflation_cells + 1)
            x_min = max(0, x - inflation_cells)
            x_max = min(width, x + inflation_cells + 1)
            blocked[y_min:y_max, x_min:x_max] = True
        
        # Treat unknown cells as blocked for safety in exploration
        # blocked |= (data == UNKNOWN)
        
        return blocked
    
    # ============ LOCALIZATION METHODS ============
    
    def get_robot_pose_in_map(self):
        """
        Retrieve robot pose in map frame using TF transforms.
        
        Returns:
            Tuple (x, y, yaw) representing pose in map frame,
            or fallback to odometry if TF lookup fails
        """
        try:
            now = rclpy.time.Time()
            # Lookup transform from map to robot base frame
            transform = self.tf_buffer.lookup_transform(
                "map", 
                "base_footprint", 
                now,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Extract orientation (quaternion to yaw)
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            # Convert quaternion to yaw angle
            yaw = math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz)
            )
            
            return x, y, yaw
            
        except Exception as error:
            self.get_logger().warning(f"TF transform lookup failed: {error}")
            # Fallback to odometry data (note: may be in different frame)
            return self.robot_x_map, self.robot_y_map, self.robot_yaw_map
    
    # ============ SENSOR CALLBACKS ============
    
    def map_callback(self, msg):
        """Process incoming occupancy grid map updates."""
        self.map_data = msg
        self.get_logger().info(
            f"Map update received: {msg.info.width}×{msg.info.height} cells, "
            f"resolution: {msg.info.resolution:.3f} m/cell, "
            f"origin: ({msg.info.origin.position.x:.2f}, "
            f"{msg.info.origin.position.y:.2f})"
        )
    
    def odom_callback(self, msg):
        """
        Process odometry data (used as fallback localization).
        
        Note: Odometry is typically in odom frame, not map frame.
        This is used only when TF transforms are unavailable.
        """
        self.robot_x_map = msg.pose.pose.position.x
        self.robot_y_map = msg.pose.pose.position.y
        
        # Extract yaw from orientation quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        self.robot_yaw_map = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )
    
    def scan_callback(self, msg):
        """Store latest laser scan data for obstacle detection."""
        self.scan_ranges = msg.ranges
    
    # ============ FRONTIER DETECTION METHODS ============
    
    def find_simple_frontiers(self):
        """
        Identify frontier cells between explored and unexplored areas.
        
        Frontiers are defined as free cells adjacent to unknown cells.
        
        Returns:
            List of tuples (world_x, world_y, distance) for frontier candidates
        """
        if self.map_data is None:
            self.get_logger().debug("No map data available for frontier detection")
            return []
        
        # Get current robot pose in map frame
        robot_x_map, robot_y_map, robot_yaw_map = self.get_robot_pose_in_map()
        
        # Reshape map data to 2D array
        data = np.array(self.map_data.data).reshape(
            self.map_data.info.height,
            self.map_data.info.width
        )
        
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        
        frontiers = []
        
        # Convert robot position to grid coordinates
        robot_grid_x = int((robot_x_map - origin_x) / resolution)
        robot_grid_y = int((robot_y_map - origin_y) / resolution)
        
        self.get_logger().debug(
            f"Robot position - Map frame: ({robot_x_map:.2f}, {robot_y_map:.2f}), "
            f"Grid coordinates: ({robot_grid_x}, {robot_grid_y})"
        )
        
        # Define search radius based on maximum frontier distance
        search_radius = int(MAX_FRONTIER_DISTANCE / resolution)
        
        # Calculate search bounds within map boundaries
        min_x = max(0, robot_grid_x - search_radius)
        max_x = min(self.map_data.info.width, robot_grid_x + search_radius)
        min_y = max(0, robot_grid_y - search_radius)
        max_y = min(self.map_data.info.height, robot_grid_y + search_radius)
        
        frontier_count = 0
        
        # Scan local area for frontier cells
        for y in range(min_y, max_y):
            for x in range(min_x, max_x):
                # Check if cell is free space
                if data[y, x] == FREE:
                    # Determine if cell is adjacent to unknown space
                    is_frontier = False
                    
                    # Check 4-connected neighborhood
                    if y > 0 and data[y - 1, x] == UNKNOWN:
                        is_frontier = True
                    elif y < self.map_data.info.height - 1 and data[y + 1, x] == UNKNOWN:
                        is_frontier = True
                    elif x > 0 and data[y, x - 1] == UNKNOWN:
                        is_frontier = True
                    elif x < self.map_data.info.width - 1 and data[y, x + 1] == UNKNOWN:
                        is_frontier = True
                    
                    if is_frontier:
                        frontier_count += 1
                        
                        # Convert grid coordinates to world coordinates
                        world_x = origin_x + (x + 0.5) * resolution
                        world_y = origin_y + (y + 0.5) * resolution
                        
                        # Calculate distance from robot
                        distance = math.hypot(world_x - robot_x_map, world_y - robot_y_map)
                        
                        # Filter by distance constraints
                        if MIN_FRONTIER_DISTANCE <= distance <= MAX_FRONTIER_DISTANCE:
                            frontiers.append((world_x, world_y, distance))
        
        self.get_logger().info(
            f"Frontier detection complete: {frontier_count} frontier cells identified, "
            f"{len(frontiers)} within distance constraints"
        )
        
        return frontiers
    
    # ============ NAVIGATION SAFETY METHODS ============
    
    def is_position_safe(self, world_x, world_y):
        """
        Verify that a position maintains safe distance from obstacles.
        
        Args:
            world_x, world_y: World coordinates to evaluate
            
        Returns:
            True if position is safe, False if too close to obstacles
        """
        if self.map_data is None:
            return True  # Assume safe if no map data
        
        data = np.array(self.map_data.data).reshape(
            self.map_data.info.height,
            self.map_data.info.width
        )
        
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        
        # Convert world coordinates to grid coordinates
        grid_x = int((world_x - origin_x) / resolution)
        grid_y = int((world_y - origin_y) / resolution)
        
        # Define safety radius in grid cells
        safety_cells = int(SAFETY_DISTANCE / resolution)
        
        # Check surrounding cells for obstacles
        for dy in range(-safety_cells, safety_cells + 1):
            for dx in range(-safety_cells, safety_cells + 1):
                check_x = grid_x + dx
                check_y = grid_y + dy
                
                # Ensure check is within map bounds
                if (0 <= check_x < self.map_data.info.width and
                        0 <= check_y < self.map_data.info.height):
                    
                    if data[check_y, check_x] == OCCUPIED:
                        return False
        
        return True
    
    def nudge_goal_away_from_walls(self, world_x, world_y):
        """
        Adjust goal position away from nearby walls using repulsion forces.
        
        Args:
            world_x, world_y: Original goal coordinates
            
        Returns:
            Adjusted (x, y) coordinates with improved clearance
        """
        if self.map_data is None:
            return world_x, world_y
        
        data = np.array(self.map_data.data).reshape(
            self.map_data.info.height,
            self.map_data.info.width
        )
        
        # Convert to grid coordinates
        grid_x, grid_y = self.world_to_grid(world_x, world_y)
        
        # Define search radius for nearby walls
        radius_cells = int(NUDGE_RADIUS_M / self.map_data.info.resolution)
        if radius_cells <= 0:
            return world_x, world_y
        
        # Initialize repulsion force vector
        force_x, force_y = 0.0, 0.0
        
        # Calculate repulsion from nearby occupied cells
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                check_x = grid_x + dx
                check_y = grid_y + dy
                
                cell_value = self.cell_value(check_x, check_y, data)
                if cell_value != OCCUPIED:
                    continue
                
                # Inverse square law repulsion
                vec_x = grid_x - check_x
                vec_y = grid_y - check_y
                distance_squared = vec_x * vec_x + vec_y * vec_y
                
                if distance_squared == 0:
                    continue
                
                weight = 1.0 / distance_squared
                force_x += vec_x * weight
                force_y += vec_y * weight
        
        # Normalize force vector
        force_magnitude = math.hypot(force_x, force_y)
        if force_magnitude < 1e-6:
            return world_x, world_y
        
        force_x /= force_magnitude
        force_y /= force_magnitude
        
        # Try progressively smaller nudges until safe position found
        for step in range(NUDGE_MAX_STEPS):
            gain = NUDGE_GAIN * (0.5 ** step)  # Exponential decay
            candidate_x = world_x + gain * force_x
            candidate_y = world_y + gain * force_y
            
            self.get_logger().debug(
                f"Goal adjustment attempt {step}: "
                f"gain={gain:.3f}, candidate=({candidate_x:.2f}, {candidate_y:.2f})"
            )
            
            # Convert candidate to grid for validation
            cand_grid_x, cand_grid_y = self.world_to_grid(candidate_x, candidate_y)
            
            # Validation criteria:
            # 1. Candidate must be in free space
            cell_val = self.cell_value(cand_grid_x, cand_grid_y, data)
            if cell_val != FREE:
                continue
            
            # 2. Path from original to candidate must not cross walls
            if self.segment_hits_wall(grid_x, grid_y, cand_grid_x, cand_grid_y, data):
                continue
            
            # 3. Candidate must maintain safety clearance
            if not self.is_position_safe(candidate_x, candidate_y):
                self.get_logger().debug("Adjusted position violates safety clearance")
                continue
            
            self.get_logger().info(
                f"Goal successfully adjusted: "
                f"({world_x:.2f}, {world_y:.2f}) → ({candidate_x:.2f}, {candidate_y:.2f})"
            )
            return candidate_x, candidate_y
        
        # Return original coordinates if no safe adjustment found
        return world_x, world_y
    
    # ============ COORDINATE TRANSFORMATION METHODS ============
    
    def world_to_grid(self, world_x, world_y):
        """
        Convert world coordinates to grid coordinates.
        
        Args:
            world_x, world_y: World coordinates in meters
            
        Returns:
            Grid coordinates (x, y) as integers
        """
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        
        grid_x = int((world_x - origin_x) / resolution)
        grid_y = int((world_y - origin_y) / resolution)
        
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """
        Convert grid coordinates to world coordinates.
        
        Args:
            grid_x, grid_y: Grid coordinates
            
        Returns:
            World coordinates (x, y) in meters
        """
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        
        world_x = origin_x + (grid_x + 0.5) * resolution
        world_y = origin_y + (grid_y + 0.5) * resolution
        
        return world_x, world_y
    
    def cell_value(self, grid_x, grid_y, data):
        """
        Retrieve occupancy value at grid coordinates with bounds checking.
        
        Args:
            grid_x, grid_y: Grid coordinates
            data: 2D occupancy grid array
            
        Returns:
            Cell value or None if coordinates are out of bounds
        """
        if (grid_x < 0 or grid_y < 0 or
                grid_x >= self.map_data.info.width or
                grid_y >= self.map_data.info.height):
            return None
        
        return data[grid_y, grid_x]
    
    def segment_hits_wall(self, start_x, start_y, end_x, end_y, data):
        """
        Check if line segment between grid points intersects obstacles.
        
        Args:
            start_x, start_y: Starting grid coordinates
            end_x, end_y: Ending grid coordinates
            data: 2D occupancy grid array
            
        Returns:
            True if segment intersects occupied cells, False otherwise
        """
        # Determine number of sampling steps
        steps = max(abs(end_x - start_x), abs(end_y - start_y), 1)
        
        # Sample points along the segment
        for k in range(steps + 1):
            t = k / steps
            sample_x = int(round(start_x + t * (end_x - start_x)))
            sample_y = int(round(start_y + t * (end_y - start_y)))
            
            cell_val = self.cell_value(sample_x, sample_y, data)
            if cell_val is None or cell_val == OCCUPIED:
                return True
        
        return False
    
    # ============ GOAL SELECTION METHODS ============
    
    def find_best_goal(self, frontiers):
        """
        Select optimal frontier goal based on multi-criteria scoring.
        
        Scoring considers:
        1. Distance to robot (closer preferred)
        2. Angular alignment with robot heading (forward preferred)
        3. Safety clearance from obstacles
        
        Args:
            frontiers: List of frontier candidates (x, y, distance)
            
        Returns:
            Optimal (x, y) coordinates or None if no suitable goal
        """
        if not frontiers:
            return None
        
        # Get current robot pose for evaluation
        robot_x_map, robot_y_map, robot_yaw_map = self.get_robot_pose_in_map()
        
        best_goal = None
        best_score = -float('inf')
        
        self.get_logger().debug(f"Evaluating {len(frontiers)} frontier candidates")
        
        for world_x, world_y, distance in frontiers:
            # Initial safety check
            if not self.is_position_safe(world_x, world_y):
                self.get_logger().debug(
                    f"Candidate ({world_x:.2f}, {world_y:.2f}) rejected: insufficient clearance"
                )
                continue
            
            # Apply wall avoidance adjustment
            adjusted_x, adjusted_y = self.nudge_goal_away_from_walls(world_x, world_y)
            
            if (adjusted_x, adjusted_y) != (world_x, world_y):
                self.get_logger().debug(
                    f"Candidate adjusted: ({world_x:.2f}, {world_y:.2f}) → "
                    f"({adjusted_x:.2f}, {adjusted_y:.2f})"
                )
            
            # Re-evaluate safety after adjustment
            if not self.is_position_safe(adjusted_x, adjusted_y):
                adjusted_x, adjusted_y = world_x, world_y  # Revert to original
            
            if not self.is_position_safe(adjusted_x, adjusted_y):
                continue
            
            # Calculate angular difference to goal
            goal_angle = math.atan2(adjusted_y - robot_y_map, adjusted_x - robot_x_map)
            angle_diff = abs(robot_yaw_map - goal_angle)
            
            # Normalize angle to [-π, π] range
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            # Reject goals requiring significant sideways movement
            if angle_diff > math.radians(110):
                self.get_logger().debug(
                    f"Candidate ({adjusted_x:.2f}, {adjusted_y:.2f}) rejected: "
                    f"excessive angular deviation ({math.degrees(angle_diff):.1f}°)"
                )
                continue
            
            # Compute scoring components
            distance_score = 1.0 / (distance + 0.1)        # Inverse distance
            alignment_score = 1.0 - (angle_diff / math.pi)  # Forward alignment
            
            # Weighted composite score
            total_score = distance_score * 0.6 + alignment_score * 0.4
            
            self.get_logger().debug(
                f"Candidate ({adjusted_x:.2f}, {adjusted_y:.2f}) scores: "
                f"distance={distance:.2f}m ({distance_score:.3f}), "
                f"alignment={alignment_score:.3f}, total={total_score:.3f}"
            )
            
            # Update best goal if score improves
            if total_score > best_score:
                best_score = total_score
                best_goal = (adjusted_x, adjusted_y)
                self.get_logger().info(
                    f"New optimal goal selected: ({adjusted_x:.2f}, {adjusted_y:.2f}) "
                    f"score={best_score:.3f}"
                )
        
        return best_goal
    
    # ============ OBSTACLE DETECTION METHODS ============
    
    def check_obstacle_ahead(self):
        """
        Detect obstacles in frontal sector using laser scan data.
        
        Returns:
            True if obstacle detected within safety distance, False otherwise
        """
        if self.scan_ranges is None:
            return False
        
        num_ranges = len(self.scan_ranges)
        if num_ranges == 0:
            return False
        
        # Define frontal sector (approximately ±10 degrees)
        half_window = int(num_ranges * (10.0 / 360.0))
        
        def is_obstacle(range_value):
            """Determine if a range reading indicates an obstacle."""
            return np.isfinite(range_value) and range_value > 0.01 and range_value < SAFETY_DISTANCE
        
        # Check left side of frontal sector
        for i in range(0, min(half_window, num_ranges)):
            if is_obstacle(self.scan_ranges[i]):
                return True
        
        # Check right side of frontal sector
        for i in range(max(0, num_ranges - half_window), num_ranges):
            if is_obstacle(self.scan_ranges[i]):
                return True
        
        return False
    
    def front_clearance(self):
        """
        Measure minimum distance in frontal sector for recovery assessment.
        
        Returns:
            Minimum valid range in frontal sector, or infinity if no valid readings
        """
        if self.scan_ranges is None or len(self.scan_ranges) == 0:
            return float("inf")
        
        num_ranges = len(self.scan_ranges)
        half_window = int(num_ranges * (10.0 / 360.0))
        valid_ranges = []
        
        def is_valid(range_value):
            """Filter valid range measurements."""
            return np.isfinite(range_value) and range_value > 0.01
        
        # Collect valid ranges from frontal sector
        for i in range(0, min(half_window, num_ranges)):
            if is_valid(self.scan_ranges[i]):
                valid_ranges.append(self.scan_ranges[i])
        
        for i in range(max(0, num_ranges - half_window), num_ranges):
            if is_valid(self.scan_ranges[i]):
                valid_ranges.append(self.scan_ranges[i])
        
        return min(valid_ranges) if valid_ranges else float("inf")
    
    # ============ RECOVERY BEHAVIOR METHODS ============
    
    def update_stuck_detector(self, robot_x_map, robot_y_map):
        """
        Monitor robot movement to detect stuck conditions.
        
        Args:
            robot_x_map, robot_y_map: Current robot position
            
        Returns:
            True if robot is stuck, False otherwise
        """
        current_time = time.time()
        
        # Initialize reference position
        if self.last_pose_check is None:
            self.last_pose_check = (robot_x_map, robot_y_map)
            self.last_pose_check_time = current_time
            return False
        
        # Calculate movement since last check
        delta_x = robot_x_map - self.last_pose_check[0]
        delta_y = robot_y_map - self.last_pose_check[1]
        distance_moved = math.hypot(delta_x, delta_y)
        
        # Update reference periodically
        time_elapsed = current_time - self.last_pose_check_time
        if time_elapsed >= STUCK_TIME_S:
            self.last_pose_check = (robot_x_map, robot_y_map)
            self.last_pose_check_time = current_time
            
            if distance_moved < STUCK_DIST_M:
                self.get_logger().warning(
                    f"Stuck condition detected: moved only {distance_moved:.3f}m "
                    f"in {time_elapsed:.1f}s"
                )
                return True
        
        return False
    
    # def start_recovery(self):
    #     """Activate recovery behavior sequence for stuck situations."""
    #     current_time = time.time()
        
    #     # Prevent rapid re-triggering of recovery
    #     if current_time - self.last_recovery_time < RECOVERY_COOLDOWN_S:
    #         return
        
    #     self.get_logger().warning(
    #         "Activating recovery behavior: backing up followed by rotation"
    #     )
        
    #     # Reset current navigation goal
    #     self.current_goal = None
    #     self.goal_reached = True
        
    #     # Determine rotation direction based on obstacle proximity
    #     if self.scan_ranges is not None and len(self.scan_ranges) > 0:
    #         num_ranges = len(self.scan_ranges)
    #         right_idx = int(num_ranges * 0.75)
    #         left_idx = int(num_ranges * 0.25)
            
    #         # Get ranges with bounds checking
    #         right_range = (
    #             self.scan_ranges[right_idx]
    #             if right_idx < num_ranges and np.isfinite(self.scan_ranges[right_idx])
    #             else float("inf")
    #         )
    #         left_range = (
    #             self.scan_ranges[left_idx]
    #             if left_idx < num_ranges and np.isfinite(self.scan_ranges[left_idx])
    #             else float("inf")
    #         )
            
    #         # Rotate toward more open space
    #         self.recovery_turn_dir = 1 if (right_range < left_range) else -1
    #     else:
    #         self.recovery_turn_dir = 1  # Default to left turn
        
    #     # Initialize recovery state machine
    #     self.recovery_state = "BACKUP"
    #     self.recovery_until = current_time + BACKUP_TIME_S
    #     self.last_recovery_time = current_time
    
    # ============ PLANNING AND CONTROL METHODS ============
    
    def planning_loop(self):
        """
        Main planning cycle for frontier exploration.
        
        This method runs periodically to:
        1. Detect exploration frontiers
        2. Select optimal goal
        3. Plan collision-free path
        4. Update navigation state
        """
        self.planning_counter += 1
        self.get_logger().info(f"Planning cycle #{self.planning_counter}")
        planning_start = time.time()
        
        # Validate prerequisites
        if self.map_data is None:
            self.get_logger().info("Awaiting initial map data")
            return
        
        if not self.goal_reached:
            self.get_logger().debug("Currently navigating to existing goal")
            return
        
        # Log current robot state for debugging
        robot_x, robot_y, robot_yaw = self.get_robot_pose_in_map()
        self.get_logger().debug(
            f"Planning from position: ({robot_x:.3f}, {robot_y:.3f}), "
            f"orientation: {math.degrees(robot_yaw):.1f}°"
        )
        
        self.get_logger().info("Initiating frontier-based goal selection")
        
        # Step 1: Frontier detection
        frontiers = self.find_simple_frontiers()
        self.get_logger().info(f"Identified {len(frontiers)} frontier candidates")
        
        # Log sample frontiers for debugging
        if frontiers:
            for i, (fx, fy, dist) in enumerate(frontiers[:3]):
                self.get_logger().debug(
                    f"Frontier {i}: ({fx:.2f}, {fy:.2f}), distance: {dist:.2f}m"
                )
        
        # Step 2: Goal selection or fallback behavior
        if not frontiers:
            self.get_logger().info("No frontiers detected, initiating random exploration")
            self.random_exploration()
            return
        
        best_goal = self.find_best_goal(frontiers)
        if best_goal is None:
            self.get_logger().info("No suitable goal found, initiating random exploration")
            self.random_exploration()
            return
        
        goal_x, goal_y = best_goal
        
        # Step 3: Path planning with A*
        data = np.array(self.map_data.data).reshape(
            self.map_data.info.height,
            self.map_data.info.width
        )
        
        # Inflate obstacles for safety margin
        resolution = self.map_data.info.resolution
        inflation_cells = max(1, int(0.12 / resolution))
        blocked_map = self.make_inflated_blocked(data, inflation_cells)
        
        # Convert to grid coordinates for path planning
        start_grid = self.world_to_grid(robot_x, robot_y)
        goal_grid = self.world_to_grid(goal_x, goal_y)
        
        # Generate path using A* algorithm
        grid_path = self.astar(start_grid, goal_grid, blocked_map)
        
        if grid_path is None or len(grid_path) < 2:
            self.get_logger().info(
                "Path planning failed: no valid route to selected goal"
            )
            self.goal_reached = True
            self.current_goal = None
            return
        
        self.get_logger().info(
            f"Path planning successful: {len(grid_path)} waypoints generated"
        )
        
        # Convert path to world coordinates
        world_path = [self.grid_to_world(cx, cy) for (cx, cy) in grid_path]
        self.publish_path(world_path)  # Visualize path
        
        # Step 4: Initialize path following
        self.path_waypoints = world_path
        self.path_index = 1  # Skip current position
        self.current_goal = self.path_waypoints[self.path_index]
        self.goal_reached = False
        
        # Publish goal for visualization
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_msg)
        
        planning_time = time.time() - planning_start
        self.get_logger().info(
            f"New navigation goal: ({goal_x:.2f}, {goal_y:.2f}), "
            f"planning time: {planning_time:.2f}s"
        )
    
    def random_exploration(self):
        """
        Fallback exploration behavior when no frontiers are detected.
        
        Generates a random goal within safety constraints to encourage
        continued exploration and map coverage.
        """
        robot_x_map, robot_y_map, robot_yaw_map = self.get_robot_pose_in_map()
        
        # Generate random goal within forward hemisphere
        angle_offset = np.random.uniform(-math.pi / 2, math.pi / 2)
        distance = np.random.uniform(1.0, 2.0)
        
        goal_x = robot_x_map + distance * math.cos(robot_yaw_map + angle_offset)
        goal_y = robot_y_map + distance * math.sin(robot_yaw_map + angle_offset)
        
        # Validate goal safety
        if self.is_position_safe(goal_x, goal_y):
            self.current_goal = (goal_x, goal_y)
            self.goal_reached = False
            
            # Visualize random exploration goal
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = "map"
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = goal_x
            goal_msg.pose.position.y = goal_y
            goal_msg.pose.orientation.w = 1.0
            self.goal_pub.publish(goal_msg)
            
            self.get_logger().info(
                f"Random exploration goal: ({goal_x:.2f}, {goal_y:.2f})"
            )
        else:
            self.get_logger().info(
                "Random goal generation failed safety check, initiating rotation"
            )
            # Default to in-place rotation if no safe goal found
            self.current_goal = None
            self.goal_reached = False
    
    def publish_path(self, world_points):
        """
        Visualize planned path for debugging and monitoring.
        
        Args:
            world_points: List of (x, y) coordinates representing the path
        """
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in world_points:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = path_msg.header.stamp
            pose_stamped.pose.position.x = float(x)
            pose_stamped.pose.position.y = float(y)
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)
        
        self.path_pub.publish(path_msg)
    
    def control_loop(self):
        """
        Primary control loop for robot navigation.
        
        This method runs at high frequency (10 Hz) to:
        1. Execute recovery behaviors if active
        2. Detect and respond to obstacles
        3. Navigate toward current goal
        4. Monitor for stuck conditions
        """
        command = Twist()
        
        # ===== OBSTACLE AVOIDANCE =====
        if self.check_obstacle_ahead():
            self.get_logger().info("Obstacle detected in frontal sector")
            command.linear.x = 0.0
            command.angular.z = TURN_SPEED
            self.avoiding_obstacle = True
            
        elif self.avoiding_obstacle:
            # Transition from obstacle avoidance
            self.avoiding_obstacle = False
            command.linear.x = 0.0
            command.angular.z = 0.0
        
        # ===== RECOVERY BEHAVIOR EXECUTION =====
        current_time = time.time()
        
        if self.recovery_state != "NONE":
            if self.recovery_state == "BACKUP":
                # Execute backward movement
                command.linear.x = BACKUP_SPEED
                command.angular.z = 0.0
                
                if current_time >= self.recovery_until:
                    self.recovery_state = "TURN"
                    self.recovery_until = current_time + TURN_TIME_S
                    self.get_logger().info("Recovery: transition to rotation phase")
            
            elif self.recovery_state == "TURN":
                # Execute rotation
                command.linear.x = 0.0
                command.angular.z = TURN_SPEED_RECOVERY * self.recovery_turn_dir
                
                if current_time >= self.recovery_until:
                    self.recovery_state = "WAIT_CLEAR"
                    self.get_logger().info("Recovery: transition to clearance check")
            
            elif self.recovery_state == "WAIT_CLEAR":
                # Wait for sufficient frontal clearance
                command.linear.x = 0.0
                command.angular.z = 0.0
                
                if self.front_clearance() > MIN_FRONT_CLEAR_M:
                    self.recovery_state = "NONE"
                    self.get_logger().info("Recovery: sufficient clearance detected")
            
            self.cmd_vel_pub.publish(command)
            return
        
        # ===== STUCK DETECTION =====
        robot_x_map, robot_y_map, robot_yaw_map = self.get_robot_pose_in_map()
        
        # if self.update_stuck_detector(robot_x_map, robot_y_map):
        #     self.start_recovery()
        #     # Execute initial recovery command
        #     command.linear.x = BACKUP_SPEED
        #     command.angular.z = 0.0
        #     self.cmd_vel_pub.publish(command)
        #     return
        
        # ===== GOAL-DIRECTED NAVIGATION =====
        if self.current_goal is not None and not self.goal_reached:
            goal_x, goal_y = self.current_goal
            
            # Calculate navigation errors
            delta_x = goal_x - robot_x_map
            delta_y = goal_y - robot_y_map
            distance_to_goal = math.hypot(delta_x, delta_y)
            target_angle = math.atan2(delta_y, delta_x)
            
            # Check goal achievement
            if distance_to_goal < GOAL_TOLERANCE:
                # Handle waypoint navigation or final goal
                if (self.path_waypoints is not None and
                        self.path_index < len(self.path_waypoints) - 1):
                    self.path_index += 1
                    self.current_goal = self.path_waypoints[self.path_index]
                    self.get_logger().info(
                        f"Waypoint achieved, advancing to {self.path_index}/"
                        f"{len(self.path_waypoints)}"
                    )
                else:
                    # Final goal reached
                    self.goal_reached = True
                    self.path_waypoints = None
                    command.linear.x = 0.0
                    command.angular.z = 0.0
                    self.get_logger().info("Navigation goal successfully achieved")
            
            else:
                # Compute angular error for steering
                angle_error = target_angle - robot_yaw_map
                
                # Normalize angular error to [-π, π]
                if angle_error > math.pi:
                    angle_error -= 2 * math.pi
                elif angle_error < -math.pi:
                    angle_error += 2 * math.pi
                
                self.get_logger().debug(
                    f"Navigation status: distance={distance_to_goal:.2f}m, "
                    f"angular error={math.degrees(angle_error):.1f}°"
                )
                
                # Proportional control for navigation
                if abs(angle_error) > 0.5:  # ~30 degrees threshold
                    # Initial alignment via rotation
                    command.linear.x = 0.0
                    command.angular.z = TURN_SPEED * (1.0 if angle_error > 0 else -1.0)
                else:
                    # Combined translation and rotation
                    command.linear.x = min(SPEED, distance_to_goal * 0.5)
                    command.angular.z = 1.0 * angle_error
        
        else:
            # No active goal - maintain stationary position
            command.linear.x = 0.0
            command.angular.z = 0.0
        
        self.cmd_vel_pub.publish(command)


def main(args=None):
    """
    Main entry point for frontier exploration node.
    
    Initializes ROS 2, creates exploration node, and manages execution lifecycle.
    """
    rclpy.init(args=args)
    
    try:
        explorer = FrontierExplorer()
        rclpy.spin(explorer)
        
    except KeyboardInterrupt:
        explorer.get_logger().info("Frontier exploration node shutdown initiated")
    
    finally:
        # Clean shutdown procedure
        explorer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

    