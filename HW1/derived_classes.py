import numpy as np
from abstract_base_classes import StateSpace, ActionSpace, StateTransition

class StateSpacefor2D(StateSpace): 
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

class ActionSpacefor2D(ActionSpace):
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

class StateTransitionfor2D(StateTransition):
    # A base class to specify a state transition function
    def __call__(self, x, u):
        # Return the new state obtained by applying action u at state x
        x_prime = (x[0] + u[0], x[1] + u[1])
        return x_prime