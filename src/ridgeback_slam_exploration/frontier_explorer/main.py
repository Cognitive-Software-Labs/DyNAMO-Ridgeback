import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from params import UNKNOWN, START_POS, GRID_SIZE, OBSTACLE
from map_generator import get_true_map
from navigator import update_robot_map, get_best_frontier
from path_finding import a_star

def run_simulation():
    true_map = get_true_map()
    robot_map = np.full((GRID_SIZE, GRID_SIZE), UNKNOWN)
    robot_pos = START_POS
    
    path_history = [START_POS]
    
    cmap_custom = ListedColormap(['gray', 'white', 'black'])
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))

    while True: 
        update_robot_map(robot_pos, robot_map, true_map)
        target = get_best_frontier(robot_pos, robot_map)
        
        if not target:
            break
            
        full_path = a_star(robot_pos, target, robot_map)
        
        # Initialize the future path as empty
        future_path = []
        
        if full_path:
            # Plan the future: extract the segment of the path we will traverse
            move_idx = max(1, len(full_path) // 33)
            future_path = full_path[:move_idx+1] # +1 to include the stopping point
            
            # Execute the movement
            next_step = full_path[min(move_idx, len(full_path)-1)]
            robot_pos = next_step
            
            # !!! NEW: Add the new position to the history !!!
            path_history.append(robot_pos)
            
        else:
            print(f"Path blocked to {target}. Marking as obstacle and rerouting...")
            robot_map[target[1], target[0]] = OBSTACLE
            continue 

        # --- Drawing Section ---
        ax.clear()
        ax.imshow(robot_map, cmap=cmap_custom, vmin=-1, vmax=1, origin='lower')
        
        if len(path_history) > 1:
            hist_x = [p[0] for p in path_history]
            hist_y = [p[1] for p in path_history]
            # Use a solid brown/orange line for the history
            ax.plot(hist_x, hist_y, color="#884410", linewidth=2, linestyle='-', label="History")
            
        if future_path:
            # The future starts from the robot, so we add the current position at the beginning
            fut_path_with_start = [path_history[-1]] + future_path
            fut_x = [p[0] for p in fut_path_with_start]
            fut_y = [p[1] for p in fut_path_with_start]
            ax.plot(fut_x, fut_y, color='green', linewidth=2, linestyle='--', label="Planned Path")
        
        ax.plot(robot_pos[0], robot_pos[1], 'bo', markersize=10, label="Robot")
        ax.scatter(target[0], target[1], color='red', s=20, label="Target")
        
        plt.pause(0.05) 

    plt.ioff()
    plt.show()

if __name__ == "__main__":
    run_simulation()