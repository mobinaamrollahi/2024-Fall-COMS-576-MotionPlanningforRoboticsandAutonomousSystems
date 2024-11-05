from collections import deque
import math
import json 
from derived_classes import StateSpacefor2D, ActionSpacefor2D, StateTransitionfor2D

def fsearch(X, start, goal):
    queue = deque([(start, [])])
    visited = set()

    while queue:
        (x, y), path = queue.popleft()
        
        if (x, y) == goal:
            return path + [(x, y)]
        
        if (x, y) in visited:
            continue
        
        visited.add((x, y))
        
        # Check all four directions
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            next_x, next_y = x + dx, y + dy
            if (next_x, next_y) in X.safe_grids:
                queue.append(((next_x, next_y), path + [(x, y)]))
    return None

def add_rotations(path, thetaI):
    full_path = []
    current_theta = thetaI
    
    for i in range(len(path)):
        if i == 0:
            full_path.append((path[i][0], path[i][1], current_theta))
        else:
            previous_x, previous_y = path[i-1]
            current_x, current_y = path[i]
            
            # Determine the orientation reuired to end up in the state
            if current_x > previous_x:
                required_theta = 0 
            elif current_x < previous_x:
                required_theta = math.pi  
            elif current_y > previous_y:
                required_theta = math.pi/2  
            else:
                required_theta = 3*math.pi/2  
            
            # Add rotation if needed
            if abs(current_theta - required_theta) > 0:
                full_path.append((previous_x, previous_y, required_theta))
                current_theta = required_theta
            
            # Add movement
            full_path.append((current_x, current_y, current_theta))
    
    return full_path

def discrete_planning(X, start, goal, thetaI):
    # Extract x, y coordinates
    XI = start[:2]
    XG = goal[:2]
    
    # Find path considering only x, y
    xy_path = fsearch(X, XI, XG)
    
    if xy_path is None:
        return None
    
    # Add rotations to the path
    full_path = add_rotations(xy_path, thetaI)
    
    return full_path


if __name__ == "__main__":
    # Load data from input.json
    with open('input.json', 'r') as file:
        data = json.load(file)

    # Assign the values from the JSON to the variables
    X_max = data['X_max']
    Y_max = data['Y_max']
    r = data['r']
    obstacles = data['obstacles']
    # Evaluate the intial state to get the angle
    xI = (data['xI'][0], data['xI'][1], eval(data['xI'][2]))  
    # Goal state doesn't need orientation
    XG = tuple(data['XG'])  

    X = StateSpacefor2D(X_max, Y_max, r, obstacles)
    
    '''
    print("Safe grid points:", X.safe_grids)

    print("\nAll valid states:")
    for state in X.get_all_states():
        print(state)
    '''

    U = ActionSpacefor2D(X)
    f = StateTransitionfor2D()
    thetaI = xI[2]
    
    path = discrete_planning(X, xI, XG, thetaI)
    
    # To print path, theta has to be excluded
    if path:
        state_before = None
        unique_path = []
        
        # Excluding theta resualts in repetitive grid point
        for state in path:
            x, y, theta = state
            current_state = (x, y)
            
            # We will only print the unique ones
            if current_state != state_before:
                unique_path.append(current_state)
                state_before = current_state
        
        # Print the unique path
        for state in unique_path:
            print(f"({state[0]}, {state[1]})")
    else:
        print("No path found")