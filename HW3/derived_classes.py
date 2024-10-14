import numpy as np
import math
from abstract_base_classes import StateSpace, ActionSpace, StateTransition

class StateSpacefor2D(StateSpace): 
    def __init__(self, X_max, Y_max, r, obstacles):
        """
        Initialize the environment with grid size, obstacles, initial state, and goal set.
        
        :param X_max: The maximum x-coordinate (exclusive).
        :param Y_max: The maximum y-coordinate (exclusive).
        :param r: The radius of the robot.
        :param obstacles: A list of circular obstacles [x, y, radius].
        """
        self.X_max = X_max
        self.Y_max = Y_max
        self.obstacles = obstacles
        self.robot_radius = r
        self.theta_values = [0, math.pi/2, math.pi, 3*math.pi/2]
        self.safe_grids = self.calculate_safe_grids()

    def calculate_safe_grids(self):
        safe_grids = set()
        for x in range(int(self.robot_radius), int(self.X_max - self.robot_radius) + 1):
            for y in range(int(self.robot_radius), int(self.Y_max - self.robot_radius) + 1):
                if self.safe_position_check(x, y):
                    safe_grids.add((x, y))
        return safe_grids

    def safe_position_check(self, x, y):
        # Check if the position is within bounds considering the robot's radius
        if x - self.robot_radius < 0 or x + self.robot_radius > self.X_max or \
           y - self.robot_radius < 0 or y + self.robot_radius > self.Y_max:
            return False

        # Check for collision with obstacles
        for obs_x, obs_y, obs_radius in self.obstacles:
            distance = math.sqrt((x - obs_x)**2 + (y - obs_y)**2)
            if distance <= self.robot_radius + obs_radius:
                return False

        return True

    def __contains__(self, state):
        x, y, theta = state
        return (x, y) in self.safe_grids and theta in self.theta_values

    def get_all_states(self):
        states = []
        for x, y in self.safe_grids:
            for theta in self.theta_values:
                states.append((x, y, theta))
        return states

    def get_distance_lower_bound(self, x1, x2):
        # Manhattan distance for grid-based movement
        return abs(x1[0] - x2[0]) + abs(x1[1] - x2[1])

    def valid_state_check(self, x, y, theta):
        return (x, y) in self.safe_grids and theta in self.theta_values

class ActionSpacefor2D(ActionSpace):
    def __init__(self, X):
        self.X = X
        self.theta_values = [0, math.pi/2, math.pi, 3*math.pi/2]

    def __call__(self, x):
        return self.get_valid_actions(x)

    def get_valid_actions(self, x):
        valid_actions = []
        current_x, current_y, current_theta = x

        # Check if current_theta is one of the cardinal directions
        is_cardinal = any(abs(current_theta - t) < 0 for t in self.theta_values)

        # Generate rotation actions
        for target_theta in self.theta_values:
            if abs(current_theta - target_theta) >= 0:
                rotation = self.get_shortest_rotation(current_theta, target_theta)
                new_state = (current_x, current_y, target_theta)
                if new_state in self.X:
                    valid_actions.append((0, 0, rotation))

        # If it's a cardinal direction, consider translation
        if is_cardinal:
            dx, dy, _ = self.get_forward_vector(current_theta)
            new_x, new_y = current_x + dx, current_y + dy
            new_state = (new_x, new_y, current_theta)
            if new_state in self.X:
                valid_actions.append((dx, dy, 0))

        return valid_actions

    def get_shortest_rotation(self, current_theta, target_theta):
        diff = (target_theta - current_theta) % (2 * math.pi)
        if diff > math.pi:
            return diff - 2 * math.pi
        return diff

    def get_forward_vector(self, theta):
        if abs(theta - 0) < 0:
            return (1, 0, 0)  # Face right, move right
        elif abs(theta - math.pi/2) < 0:
            return (0, 1, 0)  # Face up, move up
        elif abs(theta - math.pi) < 0:
            return (-1, 0, 0)  # Face left, move left
        elif abs(theta - 3*math.pi/2) < 0:
            return (0, -1, 0)  # Face down, move down
        else:
            return (0, 0, 0)  # No movement for non-cardinal directions
        
class StateTransitionfor2D(StateTransition):
    def __call__(self, x, u):
        current_x, current_y, current_theta = x
        dx, dy, dtheta = u

        # Rotation action
        if dx == 0 and dy == 0 and dtheta != 0:
            new_theta = (current_theta + dtheta) % (2 * math.pi)
            return (current_x, current_y, new_theta)
        
        # Translation 
        elif dtheta == 0:
            new_x = current_x + dx
            new_y = current_y + dy
            return (new_x, new_y, current_theta)

    def __str__(self):
        return "StateTransitionfor2D: Handles rotation OR translation, not both simultaneously"