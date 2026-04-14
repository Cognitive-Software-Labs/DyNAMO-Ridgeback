import numpy as np
from params import GRID_SIZE, OBSTACLE, FREE

def get_true_map():

    m = np.full((GRID_SIZE, GRID_SIZE), FREE)
    
    m[0, :] = OBSTACLE          # Peretele de sus (primul rând)
    m[-1, :] = OBSTACLE         # Peretele de jos (ultimul rând)
    m[:, 0] = OBSTACLE          # Peretele din stânga (prima coloană)
    m[:, -1] = OBSTACLE         # Peretele din dreapta (ultima coloană)
    

    m[15:35, 20:25] = OBSTACLE 
    m[10:15, 10:30] = OBSTACLE
    m[35:45, 5:15] = OBSTACLE
    m[5, 5:45] = OBSTACLE 

    
    return m