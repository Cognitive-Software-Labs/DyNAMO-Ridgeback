import numpy as np
from collections import deque
from ridgeback_autonomy.frontier_explorer.params import UNKNOWN, FREE, VIEW_RADIUS, GRID_SIZE, OBSTACLE


def update_robot_map(pos, robot_map, true_map):
    """Sensor Model: Discovers the area around the robot based on its VIEW_RADIUS."""
    cx, cy = pos
    for y in range(max(0, cy - VIEW_RADIUS), min(GRID_SIZE, cy + VIEW_RADIUS)):
        for x in range(max(0, cx - VIEW_RADIUS), min(GRID_SIZE, cx + VIEW_RADIUS)):
            # Calculate Euclidean distance from the robot
            dist = np.sqrt((x - cx)**2 + (y - cy)**2)
            if dist <= VIEW_RADIUS:
                # Reveal the true map cells to the robot
                robot_map[y, x] = true_map[y, x]


def update_robot_awareness_from_costmap(robot_pos, robot_awareness_map, costmap_data):
    """Update robot awareness map from costmap data.
    Used for ROS 2 integration."""
    # This function is a no-op as the costmap already represents the robot's knowledge
    pass


def is_frontier_point(x, y, robot_map):
    """Checks if a point is a valid frontier (A FREE cell next to an UNKNOWN cell)."""
    if y < 0 or y >= robot_map.shape[0] or x < 0 or x >= robot_map.shape[1]:
        return False
    if robot_map[y, x] != FREE:
        return False
    # Check the 4 direct neighbors (Up, Down, Left, Right)
    for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < robot_map.shape[1] and 0 <= ny < robot_map.shape[0]:
            if robot_map[ny, nx] == UNKNOWN:
                return True
    return False

def get_frontier_clusters(robot_map):
    """Uses Flood Fill to group adjacent frontier points into clusters."""
    visited = np.zeros_like(robot_map, dtype=bool)
    clusters = []

    for y in range(1, robot_map.shape[0] - 1):
        for x in range(1, robot_map.shape[1] - 1):
            if not visited[y, x] and is_frontier_point(x, y, robot_map):
                new_cluster = []
                queue = deque([(x, y)])
                visited[y, x] = True
                
                # Start Flood Fill
                while queue:
                    curr_x, curr_y = queue.popleft()
                    new_cluster.append((curr_x, curr_y))
                    
                    # Check 8-way connectivity for adjacent frontier points
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            nx, ny = curr_x + dx, curr_y + dy
                            if (0 <= nx < robot_map.shape[1] and 0 <= ny < robot_map.shape[0] and 
                                    not visited[ny, nx] and 
                                    is_frontier_point(nx, ny, robot_map)):
                                visited[ny, nx] = True
                                queue.append((nx, ny))
                clusters.append(new_cluster)
    return clusters

def get_best_frontier(pos, robot_map, distance_weight=0.5, size_weight=0.5, min_frontier_size=6):
    """Selects frontier cluster based on both distance and size.
    
    Args:
        pos: Robot position (x, y)
        robot_map: Occupancy map (numpy array)
        distance_weight: How much to prioritize closeness (0-1)
        size_weight: How much to prioritize cluster size (0-1)
        min_frontier_size: Minimum frontier cluster size
    
    Returns:
        Target position (x, y) or None if no frontier found
    """
    clusters = get_frontier_clusters(robot_map)
    if not clusters:
        return None

    best_target = None
    best_score = float('-inf')
    
    # Normalize metrics for fair comparison
    max_distance = float('-inf')
    max_size = float('-inf')
    
    # First pass: find max values
    for cluster in clusters:
        if len(cluster) < min_frontier_size:
            continue
        cx = np.mean([p[0] for p in cluster])
        cy = np.mean([p[1] for p in cluster])
        valid_target = min(cluster, key=lambda p: (p[0]-cx)**2 + (p[1]-cy)**2)
        dist = np.sqrt((valid_target[0] - pos[0])**2 + (valid_target[1] - pos[1])**2)
        
        max_distance = max(max_distance, dist)
        max_size = max(max_size, len(cluster))
    
    # Second pass: score each cluster
    for cluster in clusters:
        if len(cluster) < min_frontier_size:
            continue
        
        cx = np.mean([p[0] for p in cluster])
        cy = np.mean([p[1] for p in cluster])
        valid_target = min(cluster, key=lambda p: (p[0]-cx)**2 + (p[1]-cy)**2)
        dist = np.sqrt((valid_target[0] - pos[0])**2 + (valid_target[1] - pos[1])**2)
        
        # Normalize and invert distance (closer = higher score)
        distance_score = 1 - (dist / max_distance) if max_distance > 0 else 0
        # Normalize size (larger = higher score)
        size_score = len(cluster) / max_size if max_size > 0 else 0
        
        # Combined score
        score = (distance_weight * distance_score) + (size_weight * size_score)
        
        if score > best_score:
            best_score = score
            best_target = valid_target
    
    # Fallback
    if best_target is None and clusters:
        cluster = clusters[0]
        if len(cluster) >= min_frontier_size:
            cx = np.mean([p[0] for p in cluster])
            cy = np.mean([p[1] for p in cluster])
            best_target = min(cluster, key=lambda p: (p[0]-cx)**2 + (p[1]-cy)**2)
    
    return best_target