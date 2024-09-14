import collections
from collections import deque
from queue import Queue
import heapq
from abc import ABC, abstractmethod
import math
import numpy as np
import json 

class GridEnvironment():
    def __init__(self, X_max, Y_max, obstacles, xI, XG):
        """
        Initialize the environment with grid size, obstacles, initial state, and goal set.
        
        :param X_max: The maximum x-coordinate (exclusive).
        :param Y_max: The maximum y-coordinate (exclusive).
        :param obstacles: A set of coordinates representing obstacles.
        :param xI: The initial state (x, y).
        :param XG: A list of goal states.
        """
        self.X_max = X_max
        self.Y_max = Y_max
        self.obstacles = set(obstacles)
        self.xI = xI
        self.XG = XG
        self.W = {(x, y) for x in range(self.X_max) for y in range(self.Y_max)}

class StateSpace:
    # A base class to specify a state space X
    def __init__(self, enviornment):
        self.evn = enviornment
        self.states = self.env.W - self.env.obstacles

    def __contains__(self, x) -> bool:
        if x not in self.states:
            raise NotImplementedError
        # Return whether the given state x is in the state space
        return True

    def get_distance_lower_bound(self, x1, x2) -> float:
    # Return the lower bound on the distance between the given states x1 and x2
        # Convert to numpy arrays
        x1_arr = np.array(x1)
        x2_arr = np.array(x2)
    
        # Use numpy's norm function to calculate the distance
        return np.linalg.norm(x1_arr - x2_arr)

class ActionSpace:
    #A base class to specify an action space
    def __call__(self, x) -> list:
    # Return the list of all the possible actions at the given state x
        actions = [(0, 1), (-1, 0), (1, 0), (0, -1)]
        valid_actions = []

        # Check each action to see if it results in a valid state
        for action in actions:
            new_state = (x[0] + action[0], x[1] + action[1])
            if new_state in self.state_space:  # Check if new_state is within the state space
                valid_actions.append(action)  # Add action if the resulting state is valid

        return valid_actions

class StateTransition:
    # A base class to specify a state transition function
    def __call__(self, x, u):
    # Return the new state obtained by applying action u at state x
        raise NotImplementedError

class Queue(ABC):
    def __init__(self):
        self.elements = deque()
        self.parents = {}

    def insert(self, x, parent):
        pass

    def pop(self):
        pass

    def get_path(self, goal):
        path = []
        current = goal
        while current:
            path.append(current)
            current = self.parents.get(current)
        return path[::-1]

class QueueBFS(Queue):
    def insert(self, x, parent):
        self.elements.append(x)
        self.parents[x] = parent

    def pop(self):
        return self.elements.pop(0)

class QueueAstar(Queue):
    def __init__(self, X, XG):
        super().__init__()
        self.X = X
        self.XG = XG
        self.elements = []
        self.parent = {}
        self.f_scores = {}

    def insert(self, x, parent):
        f_score = self.compute_f_score(x)
        heapq.heappush(self.elements, (f_score, x))
        self.parents[x] = parent
        self.f_scores[x] = f_score

    def pop(self):
        return heapq.heappop(self.elements)[1]

    def compute_f_score(self, x):
        g_score = len(self.get_path(x)) - 1  # Cost from start to x
        h_score = min(self.X.get_distance_lower_bound(x, goal) for goal in self.XG)
        return g_score + h_score

def get_queue(alg, X, XG):
    if alg == "bfs":
        return QueueBFS()
    elif alg == "astar":
        return QueueAstar(X, XG)
    else:
        # This is because 'dfs' has also been considered as one of the algorithms!
        raise ValueError("Invalid algorithm specified. Use 'bfs' or 'astar'")

def fsearch(X, U, f, xI, XG, alg):
    """
    Args:
    X = State Space
    U = Action Space
    f = State Transition Function
    xI = Intial State
    XG = Goal State
    alg = The Planning Algorithm
    """
    # Check if the initial state is valid
    if xI not in X:
        raise ValueError("Initial state is not in the state space")
    
    # For any element x in a non-empty set $X_G$ the statement x in X returns true
    if len(XG) != 0:
        for element in XG:
            if element in X:
                return True
    
    if xI in XG:
        return {"visited": {xI}, "path": [xI]}
    
    if not XG:
        return {"visited": set(), "path": None}


    visited = set()
    Q = get_queue(alg, X, XG)
    parent = {}
    
    Q.insert(xI, None)
    visited = set([xI])

    while Q:
        x = Q.pop()  # For BFS. For A*, this would be Q.get()

        if x in visited:
            continue
        visited.add(x)

        if x in XG:
            # Reconstruct path
            path = []
            while x is not None:
                path.append(x)
                x = parent.get(x)
            return {"visited": visited, "path": path[::-1]}
        
        for u in U(x):
            x_prime = f(x, u)
            if x_prime not in visited:
                visited.add(x_prime)
                parent[x_prime] = x
                Q.append(x_prime)
    
    return {"visited": visited, "path": None}

def main():
    # Load data from input.json
    with open('input.json', 'r') as file:
        data = json.load(file)

    # Assign the values from the JSON to the variables
    X_max = data['X_max']
    Y_max = data['Y_max']
    obstacles = data['obstacles']
    xI = data['xI']
    XG = data['XG']

    # Now X_max, Y_max, obstacles, xI, and XG are ready to be used in your program
    print(X_max, Y_max, obstacles, xI, XG)

    # Create the environment
    env = GridEnvironment(X_max, Y_max, obstacles, xI, XG)
    
    # Create instances of StateSpace and ActionSpace
    X = StateSpace(env)  # State space
    U = ActionSpace(X)   # Action space

# Entry point of the program
if __name__ == "__main__":
    main()

    #result = fsearch(X, U, f, xI, XG, args.alg)
    #print("rs",result)
