import heapq
import numpy as np
from ridgeback_autonomy.frontier_explorer.params import OBSTACLE, GRID_SIZE

def heuristic(a, b):
    # Euclidean distance
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def a_star(start, goal, robot_map):
    """Optimized A* Algorithm to find the shortest path avoiding obstacles."""
    # 8-way movement (Horizontal, Vertical, Diagonal)
    neighbors = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]
    rows, cols = robot_map.shape
    
    # Priority queue to pick the node with the lowest f_score
    oheap = []
    # Dictionary to retrace the path once the goal is reached
    came_from = {}
    
    # Cost from start along best known path
    g_score = {start: 0}
    # Estimated total cost from start to goal through y
    f_score = {start: heuristic(start, goal)}
    
    heapq.heappush(oheap, (f_score[start], start))
    
    while oheap:
        # Pop the node with the lowest f_score
        current = heapq.heappop(oheap)[1]

        # Goal reached! Retrace the path backward
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1] # Return reversed path (from start to goal)

        for i, j in neighbors:
            neighbor = (current[0] + i, current[1] + j)
            
            # Check map boundaries
            if 0 <= neighbor[0] < cols and 0 <= neighbor[1] < rows:
                # Check if it's a known obstacle on the robot's map
                if robot_map[neighbor[1], neighbor[0]] == OBSTACLE:
                    continue
                
                # Distance to adjacent node is 1
                tentative_g_score = g_score[current] + 1
                
                # If we found a shorter path to this neighbor, or it's unvisited
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    # Record the path
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    
                    # Update f_score and push to heap
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(oheap, (f_score[neighbor], neighbor))
                    
    # Return empty list if no path is found
    return []