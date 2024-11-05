import numpy as np
import random
from edge_impl import EdgeStraight
from geometry import get_euclidean_distance
from graph_impl import Tree, ConnectedComponents

# Classes for creating an edge
class EdgeCreator:
    def make_edge(self, s1, s2):
        """Return an Edge object beginning at state s1 and ending at state s2"""
        raise NotImplementedError


class StraightEdgeCreator(EdgeCreator):
    def __init__(self, step_size):
        self.step_size = step_size

    def make_edge(self, s1, s2):
        return EdgeStraight(s1, s2, self.step_size)
    
# Classes for computing distance between two points
class DistanceComputator:
    def get_distance(self, s1, s2):
        """Return the distance between s1 and s2"""
        raise NotImplementedError

class EuclideanDistanceComputator(DistanceComputator):
    def get_distance(self, s1, s2):
        """Return the Euclidean distance between s1 and s2"""
        return get_euclidean_distance(s1, s2)
    
# Classes for collision checking
class CollisionChecker:
    def is_in_collision(self, state):
        """Return whether the given state is in collision"""
        raise NotImplementedError

    def is_checking_required(self):
        """Return whether collision needs to be checked at all"""
        raise NotImplementedError
    
class EmptyCollisionChecker(CollisionChecker):
    def is_in_collision(self, state):
        """Return whether the given state is in collision"""
        return False

    def is_checking_required(self):
        """Return whether collision needs to be checked at all"""
        return False

class ObstacleCollisionChecker(CollisionChecker):
    def __init__(self, obstacles):
        """The constructor

        @type obstacles: a list [obs_1, ..., obs_m] of obstacles, where obs_i is an Obstacle
            object that include a contain(s) function, which returns whether a state s
            is inside the obstacle
        """
        self.obstacles = obstacles

    def is_in_collision(self, s):
        """Return whether the point s is in collision with the obstacles"""
        for obs in self.obstacles:
            if obs.contain(s):
                return True
        return False

    def is_checking_required(self):
        """Return whether collision needs to be checked at all"""
        return True

# Planning Algorithms
def RRT (C, qI, qG, edge_creator, distance_computator, collision_checker, N, tol = 1e-4):
    G = Tree()
    root = G.add_vertex(np.array(qI))
    for i in range(N):
        use_goal = qG is not None and random.uniform(0, 1) <= p
        if use_goal:
            alpha = np.array(qG)
        else:
            alpha = sample(C)
        vn = G.get_nearest_neighbors(alpha, distance_computator, tol)
        qn = G.get_vertex_config(vn)
        (qs, edge) = stopping_configuration(
            qn, alpha, edge_creator, collision_checker, tol
        )
        if qs is None or edge is None:
            continue
        dist = get_euclidean_distance(qn, qs)
        if dist > tol:
            vs = G.add_vertex(qs)
            G.add_edge(vn, vs, edge)
            if use_goal and get_euclidean_distance(qs, qG) < tol:
                return (G, root, vs)

    return (G, root, None)
    
def PRM (C, qI, qG, edge_creator, distance_computator, collision_checker,
         N, k,  tol = 1e-4):

    def build_roadmap(G, alpha):
        """
        BUILD ROADMAP Algorithm by add configuration alpha to the roadmap G
        """
        if collision_checker.is_in_collision(alpha):
            return None
        neighbors = G.get_nearest_vertices(alpha, k, distance_computator)
        vs = G.add_vertex(alpha)
        for vn in neighbors:
            if G.is_same_component(vn, vs):
                continue
            qn = G.get_vertex_state(vn)
            if connect(alpha, qn, edge_creator, collision_checker, tol) and connect(
                qn, alpha, edge_creator, collision_checker, tol
            ):
                G.add_edge(vs, vn, edge_creator.make_edge(alpha, qn))
        return vs
    
    G = ConnectedComponents()
    i = 0

    while i < N:
        alpha = sample(C)
        if build_roadmap(G, alpha) is not None:
            i += 1
    root = None
    if qI is not None:
        root = build_roadmap(G, np.array(qI))
    goal = None
    if qG is not None:
        goal = build_roadmap(G, np.array(qG))
    return (G, root, goal)

def sample(C):
    """Return a sample configuration of the C-space based on uniform random sampling"""
    sample = [random.uniform(cspace_comp[0], cspace_comp[1]) for cspace_comp in C]
    return np.array(sample)


def stopping_configuration(s1, s2, edge_creator, collision_checker, tol):
    """Return (s, edge) where s is the point along the edge from s1 to s2 that is closest to s2 and
    is not in collision with the obstacles and edge is the edge from s to s1"""

    edge = edge_creator.make_edge(s1, s2)
    if not collision_checker.is_checking_required():
        return (s2, edge)

    if edge.get_length() < tol:
        return (s1, edge)

    curr_ind = 0
    prev_state = None
    curr_state = edge.get_discretized_state(curr_ind)

    while curr_state is not None:
        if collision_checker.is_in_collision(curr_state):
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

def connect(s1, s2, edge_creator, collision_checker, tol):
    """Return whether an edge between s1 and s2 is collision-free"""
    if not collision_checker.is_checking_required():
        return True

    edge = edge_creator.make_edge(s1, s2)
    if edge.get_length() < tol:
        return True

    curr_ind = 0
    curr_state = edge.get_discretized_state(curr_ind)
    while curr_state is not None:
        if collision_checker.is_in_collision(curr_state):
            return False
        curr_ind = curr_ind + 1
        curr_state = edge.get_discretized_state(curr_ind)

    return True