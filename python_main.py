import collections
from collections import deque
from queue import Queue
import heapq
import math
import numpy as np
import json 

class StateSpace:
    def __init__(self, X_max, Y_max, obstacles):
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
        self.obstacles = obstacles  # Convert obstacles to a set of tuples for efficient lookup


    def __contains__(self, x) -> bool:
        # Ensure x is a tuple and within bounds
        #x = (x[0], x[1])  # Ensure x is a tuple with two elements

        x, y = x
        in_bounds = (0 <= x <= self.X_max) and (0 <= y <= self.Y_max)
        not_in_obstacle = all([x != O[0] or y != O[1] for O in self.obstacles])
        #print(f"Checking state: {x}")
        #print(f"Bounds: (0 <= {x} < {self.X_max}), (0 <= {y} < {self.Y_max})")
        #print(f"Not in obstacles: {x not in self.obstacles}")

        return in_bounds and not_in_obstacle

    def get_distance_lower_bound(self, x1, x2) -> float:
        # Convert to numpy arrays
        x1_arr = np.array(x1)
        x2_arr = np.array(x2)
        # Use numpy's norm function to calculate the distance
        return np.linalg.norm(x1_arr - x2_arr)

class ActionSpace:
    actions = [(0, 1), (-1, 0), (1, 0), (0, -1)]

    # A base class to specify an action space
    def __init__(self, X, f):
        self.X = X
        self.f = f

    def __call__(self, x):
        valid_actions = []

        # Check each action to see if it results in a valid state
        for action in self.actions:
            new_state = self.f(x, action)
            if self.X.__contains__(new_state):  # Check if new_state is within the state space
                valid_actions.append(action)  # Add action if the resulting state is valid
        #print("Current State", x, "Valid Actions:", valid_actions)
        return valid_actions

class StateTransition:
    # A base class to specify a state transition function
    def __call__(self, x, u):
        # Return the new state obtained by applying action u at state x
        x_prime = (x[0] + u[0], x[1] + u[1])
        return x_prime

def fsearch(X, U, f, xI, XG, alg):
    """
    Args:
    X = State Space
    U = Action Space
    f = State Transition Function
    xI = Initial State
    XG = Goal State
    alg = The Planning Algorithm
    """
    if alg == 'bfs':
        Q = QueueBFS()
    elif alg == 'astar':
        Q = QueueAstar(X, XG)
        
    visited = set()
    Q.insert(xI, None)
    parent = {}

    while Q.queue:
        #if not Q.queue:  # Check if the queue is empty
            #print("Queue is empty in fsearch")
            # break
        x = Q.pop()  

        if x in visited:
            continue
        visited.add(x)

        if x == XG:
            # print("State", x)
            # Reconstruct path
            path = []
            while x is not None:
                path.insert(0, x)
                x = Q.parent[x]
            return {"visited": visited, "path": path}
        for u in U(x):
            x_prime = f(x, u)
            if x_prime not in visited:
                Q.insert(x_prime, x)  

    return {"visited": visited, "path": None}

class Queue:
    def __init__(self):
        self.queue = []
        self.parent = {}  # parents dictionary

    def insert(self, x, parent):
        pass

    def pop(self):
        pass

class QueueBFS(Queue):
    def insert(self, x, parent):
        if x not in self.parent:
            self.queue.append(x)
            self.parent[x] = parent

    def pop(self):
        return self.queue.pop(0)  

class QueueAstar(Queue):
    def __init__(self, X, XG):
        super().__init__()
        self.X = X
        self.XG = XG
        self.queue = []
        self.parent = {}
        self.f_scores = {}

    def insert(self, x, parent):
        self.queue.insert(0, x)        
        self.parent[x] = parent
        if parent is not None:
            self.f_scores[x] = self.f_scores[parent] + 1
        else:
            self.f_scores[x] = 0
        for goal in self.XG:
            self.f_scores[x] += self.X.get_distance_lower_bound(x, goal)

    def pop(self):
        index = self.queue.index(min(self.queue, key=lambda x: self.f_scores[x]))
        x = self.queue.pop(index)
        return x
    
# Entry point of the program
if __name__ == "__main__":
    # Load data from input.json
    with open('input.json', 'r') as file:
        data = json.load(file)

    # Assign the values from the JSON to the variables
    X_max = data['X_max']
    Y_max = data['Y_max']
    obstacles = [tuple(x) for x in data['obstacles']]
    xI = tuple(data['xI'])
    XG = tuple(data['XG'])

    # Print the loaded data
    # print("X_max:", X_max, "Y_max:", Y_max, "obstacles:", obstacles, "Initial State:", xI, "Goal State:", XG)

    X = StateSpace(X_max, Y_max, obstacles)
    f = StateTransition()
    U = ActionSpace(X, f)
    
    # Run fsearch with BFS
    result_bfs = fsearch(X, U, f, xI, XG, 'bfs')
    print("BFS results:")
    print(result_bfs)

    # Run fsearch with A* algorithm
    result_astar = fsearch(X, U, f, xI, XG, 'astar')
    print("AStar results:")
    print(result_astar)
