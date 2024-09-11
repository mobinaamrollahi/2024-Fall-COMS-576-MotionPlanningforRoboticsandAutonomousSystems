from collections import deque
import heapq

class GridEnvironment:
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
        return 0

class ActionSpace:
    #A base class to specify an action space
    def __call__(self, x) -> list:
    # Return the list of all the possible actions at the given state x
        raise NotImplementedError

class StateTransition:
    # A base class to specify a state transition function
    def __call__(self, x, u):
    # Return the new state obtained by applying action u at state x
        raise NotImplementedError


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
    if xI in X:
        return True
    
    # For any element x in a non-empty set $X_G$ the statement x in X returns true
    if len(XG) != 0:
        for element in XG:
            if element in X:
                return True
    
    # Check if the initial state is already a goal state
    if xI in XG:
        return {"visited": {xI}, "path": [xI]}
    
    # Check if the goal set is empty
    if not XG:
        return {"visited": set(), "path": None}