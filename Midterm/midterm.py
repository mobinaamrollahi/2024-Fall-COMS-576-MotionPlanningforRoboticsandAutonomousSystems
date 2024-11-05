import random
import math
import numpy as np
from graph import Tree
from edge import EdgeStraight
from geometry import get_euclidean_distance


##############################################################################
# Classes for creating an edge
##############################################################################
class EdgeCreator:
    def make_edge(self, s1, s2):
        """Return an Edge object beginning at state s1 and ending at state s2"""
        raise NotImplementedError


class StraightEdgeCreator(EdgeCreator):
    def __init__(self, step_size):
        self.step_size = step_size

    def make_edge(self, s1, s2):
        return EdgeStraight(s1, s2, self.step_size)


##############################################################################
# Classes for computing distance between 2 points
##############################################################################
class DistanceComputator:
    def get_distance(self, s1, s2):
        """Return the distance between s1 and s2"""
        raise NotImplementedError


class EuclideanDistanceComputator(DistanceComputator):
    def get_distance(self, s1, s2):
        """Return the Euclidean distance between s1 and s2"""
        return get_euclidean_distance(s1, s2)


##############################################################################
# Classes for collision checking
##############################################################################
class CollisionChecker:
    def is_in_collision(self, state):
        """Return whether the given state is in collision"""
        raise NotImplementedError
    
    def not_a_valid_config(self, state):
        """Return whether the given state is outside the world boundaries"""
        raise NotImplementedError

    def is_checking_required(self):
        """Return whether collision needs to be checked at all"""
        raise NotImplementedError


class EmptyCollisionChecker(CollisionChecker):
    def is_in_collision(self, state):
        """Return whether the given state is in collision"""
        return False
    
    def not_a_valid_config(self, Xmin, Xmax, Ymin, Ymax, s, r):
        """Return whether the given state puts the robot outside world boundaries"""
        # Check if any part of the robot goes beyond boundaries
        if (s[0] - r < Xmin or  # Left boundary
            s[0] + r > Xmax or  # Right boundary
            s[1] - r < Ymin or  # Bottom boundary
            s[1] + r > Ymax):   # Top boundary
            return True
        return False

    def is_checking_required(self):
        """Return whether collision needs to be checked at all"""
        return True


class ObstacleCollisionChecker(CollisionChecker):
    def __init__(self, obstacles):
        """The constructor
        @type obstacles: a list [obs_1, ..., obs_m] of obstacles, where obs_i is an Obstacle
            object that include a contain(s) function, which returns whether a state s
            is inside the obstacle
        """
        self.obstacles = obstacles

    def is_in_collision(self, s, r):
        """Return whether a circular robot with center s and radius r is in collision with the obstacles"""
        # Calculate distance between centers using Euclidean distance
        for obstacle in self.obstacles:
            dx = s[0] - obstacle.x
            dy = s[1] - obstacle.y
            distance = math.sqrt((dx)**2 + (dy)**2)
            if distance <= r + obstacle.r:
                return True

        return False
    
    def not_a_valid_config(self, Xmin, Xmax, Ymin, Ymax, s, r):
        """Return whether the given state puts the robot outside world boundaries"""
        # Check if any part of the robot goes beyond boundaries
        if (s[0] - r < Xmin or  # Left boundary
            s[0] + r > Xmax or  # Right boundary
            s[1] - r < Ymin or  # Bottom boundary
            s[1] + r > Ymax):   # Top boundary
            return True
        return False
            
    def is_checking_required(self):
        """Return whether collision needs to be checked at all"""
        return True

##############################################################################
# Planning algorithms
##############################################################################   

class PathPlanning:   
    def plan(self, s1, s2):
        """ Plan a path from s1 to s2."""
        pass

    def sample(self):
        """Sample a random configuration from the configuration space. """
        pass

    def stopping_configuration(self, s1, s2, tol):
        """Find the furthest non-colliding configuration along the path from s1 to s2."""
        pass


class CircularMobileRobotPathPlanning(PathPlanning):
    def __init__(self, robot_radius: float, Xmax: float, Ymax: float, circular_obstacles: list):
        """
        The constructor
        Xmin, Xmax, Ymin, Ymax: floats that defines the boundary of the world
        O: a list of tuples (x, y, r) that represent the center (x,y) and radius (r) of the circular obstacle.
        r: the radius of the robot
        """
        self.Xmin = 0
        self.Xmax = Xmax
        self.Ymin = 0
        self.Ymax = Ymax
        self.O = circular_obstacles
        self.r = robot_radius
        self.cspace = [(self.Xmin, self.Xmax), (self.Ymin, self.Ymax)]

        # Create instances to be compatiable with the rest of the code
        self.edge_creator = StraightEdgeCreator(step_size=0.1)
        self.collision_checker = ObstacleCollisionChecker(self.O)
        self.distance_computator = EuclideanDistanceComputator()

    def plan(self, qI: tuple, qG: tuple):
        qIp = qI[:2]
        qGp = qG[:2]
        numIt = 1000
        tol = 1e-5
        p = 0.01
        G = Tree()

        root = G.add_vertex(np.array(qIp))
        for i in range(numIt):
            use_goal = qGp is not None and random.uniform(0, 1) <= p
            if use_goal:
                alpha = np.array(qGp)
            else:
                alpha = self.sample()
            vn = G.get_nearest(alpha, self.distance_computator, tol)
            qn = G.get_vertex_state(vn)
            (qs, edge) = self.stopping_configuration(
                qn, alpha, tol)
            if qs is None or edge is None:
                continue
            dist = get_euclidean_distance(qn, qs)
            if dist > tol:
                vs = G.add_vertex(qs)
                G.add_edge(vn, vs, edge)
                if use_goal and get_euclidean_distance(qs, qGp) < tol:
                    path = []
                    if root is not None:
                        path = G.get_path(root, vs)
                        # print("Path", path)
                    return (G, path)

        return (G, None)
    
    def sample(self):
        """Return a sample configuration of the C-space based on uniform random sampling"""
        sample = [random.uniform(cspace_comp[0], cspace_comp[1]) for cspace_comp in self.cspace]
        return np.array(sample)


    def stopping_configuration(self, s1, s2, tol):
        """Return (s, edge) where s is the point along the edge from s1 to s2 that is closest to s2 and
        is not in collision with the obstacles and edge is the edge from s to s1"""
        
        if np.allclose(s1, s2, rtol=tol, atol=tol):
            return (s1, None)
    
        edge = self.edge_creator.make_edge(s1, s2)
        if not self.collision_checker.is_checking_required():
            return (s2, edge)

        if edge.get_length() < tol:
            return (s1, edge)

        curr_ind = 0
        prev_state = None
        curr_state = edge.get_discretized_state(curr_ind)

        while curr_state is not None:
            if (self.collision_checker.is_in_collision(curr_state, self.r) or 
            self.collision_checker.not_a_valid_config(self.Xmin, self.Xmax, self.Ymin, self.Ymax, curr_state, self.r)):
                if curr_ind == 0:
                    return (None, None)
                elif curr_ind == 1:
                    return (s1, None)
                split_t = (curr_ind - 1) * edge.get_step_size() / edge.get_length()
                (edge1, _) = edge.split(split_t)
                return (prev_state, edge1)
            curr_ind = curr_ind + 1
            prev_state = curr_state
            curr_state = edge.get_discretized_state(curr_ind)

        return (s2, edge)