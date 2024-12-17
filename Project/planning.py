from graph import *
import networkx as nx
import random
from random import randint
import math 
import matplotlib.pyplot as plt
import numpy as np
from graph import Tree
from edge import *
from geometry import get_euclidean_distance
from collisionshecker import WorkspaceObstacle, WorkspaceBoundary, CSpaceObstacle
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

class ObstacleCollisionChecker(CollisionChecker):
    def __init__(self, r, obstacles, Xmin, Xmax, Ymin, Ymax):
        """The constructor
        @type obstacles: a list [obs_1, ..., obs_m] of obstacles, where obs_i is an Obstacle
            object that include a contain(s) function, which returns whether a state s
            is inside the obstacle
        """
        self.r = r
        self.obstacles = obstacles
        self.Xmin = Xmin
        self.Xmax = Xmax
        self.Ymin = Ymin
        self.Ymax = Ymax

    def is_in_collision(self, s):
        """Return whether a circular robot with radius r and center s is in collision with the obstacles"""
        # Calculate distance between centers using Euclidean distance
        # print("s", s)
        for obstacle in self.obstacles:
            # print("obstacle.y", obstacle.y)
            # print("s[0]", s[0])
            dx = s[0] - obstacle[0]
            dy = s[1] - obstacle[1]
            distance = math.sqrt((dx)**2 + (dy)**2)
            if distance <= obstacle[2] + self.r:
                return True
        return False
    
    def not_a_valid_config(self, s):
        """Return whether the given state puts the circular robot with radius r and center s outside world boundaries"""
        # Check if any part of the robot goes beyond boundaries
        if (s[0] - self.r < self.Xmin or  # Left boundary
            s[0] + self.r > self.Xmax or  # Right boundary
            s[1] - self.r < self.Ymin or  # Bottom boundary
            s[1]+ self.r > self.Ymax):   # Top boundary
            return True
        return False
            
    def is_checking_required(self):
        """Return whether collision needs to be checked at all"""
        return True

##############################################################################
# Planning algorithms
##############################################################################
class CircularMobileRobotPathPlanning:
    def __init__(
        self, robot_radius: float, region_radius:float, Xmin: float, Xmax: float, Ymin: float, Ymax: float, circular_obstacles: list, qI, qG, N, tol
    ):
        """
        @type cspace: a list of tuples (smin, smax) indicating that the C-space
            is given by the product of the tuples.
        @type qI: a tuple (x, y) indicating the initial configuration.
        @type qG: a typle (x, y) indicating the goal configuation
            (can be None if rrt is only used to explore the C-space).
        @type edge_creator: an EdgeCreator object that includes the make_edge(s1, s2) function,
            which returns an Edge object beginning at state s1 and ending at state s2.
        @type distance_computator: a DistanceComputator object that includes the get_distance(s1, s2)
            function, which returns the distance between s1 and s2.
        @type collision_checker: a CollisionChecker object that includes the is_in_collision(s)
            function, which returns whether the state s is in collision.
        @type self.N: an integer indicating the maximum number of iterations.
        @type tol: a float, indicating the tolerance on the euclidean distance when checking whether
            2 states are the same
        """
        
        # Define CSpace
        self.cspace = [(Xmin, Xmax), (Ymin, Ymax)]
        self.edge_creator = StraightEdgeCreator(0.1)
        self.distance_computator = EuclideanDistanceComputator()
        self.Xmin = Xmin
        self.Xmax = Xmax
        self.Ymin = Ymin
        self.Ymax = Ymax
        self.robot_radius = robot_radius
        self.region_radius = region_radius
        self.qI = qI
        self.qG = qG
        self.N = N
        self.obstacles = circular_obstacles
        self.tol = tol

        # Create instances to be compatiable with the rest of the code
        self.edge_creator = StraightEdgeCreator(0.1)
        self.distance_computator = EuclideanDistanceComputator()
        self.collision_checker = ObstacleCollisionChecker(r = self.robot_radius, obstacles = self.obstacles, Xmin = self.Xmin, Xmax = self.Xmax, Ymin = self.Ymin, Ymax = self.Ymax)        

    def rrt(self,
    ):
        """RRT with obstacles
        @type pG: a float indicating the probability of choosing the goal configuration.
        @return (G, root, goal) where G is the tree, root is the id of the root vertex
            and goal is the id of the goal vertex (if one exists in the tree; otherwise goal will be None).
        """
        G = Tree()
        root = G.add_vertex(np.array(self.qI))
        print("root is", root)
        for i in range(self.N):
            # use_goal = self.qG is not None and random.uniform(0, 1) <= pG
            #if use_goal:
            #qrand = np.array(self.qG)
            #else:
            qrand = self.sample(self.cspace)
            vn = G.get_nearest(qrand, self.distance_computator, self.tol)
            qnearest = G.get_vertex_state(vn)
            (qs, edge) = self.stopping_configuration(
                qnearest, qrand, self.edge_creator, self.collision_checker, self.tol
            )
            if qs is None or edge is None:
                continue
            dist = get_euclidean_distance(qnearest, qs)
            if dist > self.tol:
                vs = G.add_vertex(qs)
                G.add_edge(vn, vs, edge)

        # Initialize the minimum cost and path
        min_cost = float('inf')
        best_path = []

        # Iterate over all the vertices in the graph
        for vertex in G.get_vertices():
            distance_to_goal = self.distance_computator.get_distance(G.get_vertex_state(vertex),np.array(self.qG))
            if distance_to_goal <= self.region_radius:
                path = G.get_path(root, vertex) 
                path_cost = 0
                for u, v in zip(path[:-1], path[1:]):
                    path_cost += np.linalg.norm(np.array(v) - np.array(u))

                if path_cost < min_cost:
                    min_cost = path_cost
                    best_path = path

        return (G, best_path)

    def rrtstar(self, 
        d=2, 
        eta=10,
        ):
        """RRTstar with obstacles
        @type eta: a prespecified value for the steering function.
        @type d: an integer, indicating the number dimension of the world
        @return (G, root, goal) where G is the roadmap, root is the id of the root vertex
            and goal is the id of the goal vertex.
            If the root (resp. goal) vertex does not exist in the roadmap, root (resp. goal) will be None.
        """
        G = nx.Graph()
        root = tuple(self.qI)  
        G.add_node(root)
        print(f"added the root: {root}")
        for i in range(1, self.N):
            print("Iteration: ",i)
            qrand = self.samplefree(self.cspace, self.collision_checker)
            qnearest = self.nearest(G, qrand)
            qnew = self.steer(G, qnearest, qrand, eta)
            if self.obstaclefree(np.array(qnew), np.array(qnearest), self.edge_creator, self.collision_checker, self.tol, self.robot_radius):
                gamma = self.calculate_gamma(self.Xmax, self.Ymax, d)
                minr = min(gamma * (math.log(len(G.nodes())) / len(G.nodes())) ** (0.5), eta)
                r = minr/len(G.nodes())
                Qnear = self.rnear(G, qnew, r)
                G.add_node(tuple(qnew)) 

                min_cost = self.cost(G,root, qnearest) + self.distance_computator.get_distance(np.array(qnearest), np.array(qnew))
                qmin = qnearest
                for qnear in Qnear:
                    new_cost = self.cost(G, root, qnear) + self.distance_computator.get_distance(np.array(qnear), np.array(qnew))
                    if self.obstaclefree(np.array(qnew), np.array(qnear), self.edge_creator, self.collision_checker, self.tol, self.robot_radius):
                        if new_cost < min_cost: 
                            min_cost = new_cost
                            qmin = qnear
                            
                G.add_edge(tuple(qmin), tuple(qnew))
            

                for qnear in Qnear:
                    cost_new = self.cost(G, root, tuple(qnew)) + self.distance_computator.get_distance(np.array(qnear), np.array(qnew))
                    if self.obstaclefree(np.array(qnew), np.array(qnear), self.edge_creator, self.collision_checker, self.tol, self.robot_radius) and cost_new < self.cost(G, root, tuple(qnear)):
                        qparent = self.parent(G, qnear)
                        G.remove_edge(qparent, qnear)
                        G.add_edge(qnew, qnear)

        path = self.shortest_path(G, self.qG, self.region_radius, root)
        return (G, path)

    def sample(self, cspace):
        """Return a sample configuration of the C-space based on uniform random sampling"""
        sample = [random.uniform(cspace_comp[0], cspace_comp[1]) for cspace_comp in cspace]
        return np.array(sample)
    
    def samplefree(self, cspace, collision_checker):
        """Return a sample from free configuration space based on uniform random sampling"""
        sample = [random.uniform(cspace_comp[0], cspace_comp[1]) for cspace_comp in cspace]
        while collision_checker.is_in_collision(sample) or collision_checker.not_a_valid_config(sample):
            sample = [random.uniform(cspace_comp[0], cspace_comp[1]) for cspace_comp in cspace]
        return np.array(sample)

    def stopping_configuration(self, s1, s2, edge_creator, collision_checker, tol):
        """Return (s, edge) where s is the point along the edge from s1 to s2 that is closest to s2 and
        is not in collision with the obstacles and edge is the edge from s to s1"""

        edge = edge_creator.make_edge(s1, s2)
        # print(f"In the stopping configuration. The new edge is {edge}.")
        if not collision_checker.is_checking_required():
            return (s2, edge)

        if edge.get_length() < tol:
            return (s1, edge)

        curr_ind = 0
        prev_state = None
        curr_state = edge.get_discretized_state(curr_ind)

        while curr_state is not None:
            if (self.collision_checker.is_in_collision(curr_state) or 
            self.collision_checker.not_a_valid_config(curr_state)):
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
    
    def obstaclefree(self, s1, s2, edge_creator, collision_checker, tol, robot_radius):
        """Return whether an edge between s1 and s2 is collision-free"""
        dx = s2[0] - s1[0]
        dy = s2[1] - s1[1]
        line_length = self.distance_computator.get_distance(s1, s2)

        # Calculate the unit vector of the original line (to get direction)
        unit_vector = np.array([dy, -dx]) / line_length  # Perpendicular vector to the line

        # Compute the offset for the parallel lines
        offset = robot_radius / (np.cos(np.tanh(dy/dx))) #line_length / np.abs(np.linalg.norm(unit_vector))

        # Calculate the new coordinates for the parallel lines
        x1_parallel = s1[0] + unit_vector[0] * offset
        y1_parallel = s1[1] + unit_vector[1] * offset
        x2_parallel = s2[0] + unit_vector[0] * offset
        y2_parallel = s2[1] + unit_vector[1] * offset

        x1_parallel_neg = s1[0] - unit_vector[0] * offset
        y1_parallel_neg = s1[1] - unit_vector[1] * offset
        x2_parallel_neg = s2[0] - unit_vector[0] * offset
        y2_parallel_neg = s2[1] - unit_vector[1] * offset

        if self.edge_circle_intersection(s1, s2, self.obstacles) or self.edge_circle_intersection([x1_parallel, y1_parallel], [x2_parallel, y2_parallel], self.obstacles) or self.edge_circle_intersection([x1_parallel_neg, y1_parallel_neg], [x2_parallel_neg, y2_parallel_neg], self.obstacles):
            return False
        
        return True

        # return ([x1_parallel, y1_parallel], [x2_parallel, y2_parallel]), ([x1_parallel_neg, y1_parallel_neg], [x2_parallel_neg, y2_parallel_neg])
    def edge_circle_intersection(self, s1, s2, obstacles):
        """
        Check if the line from s1 to s2 intersects with the obstacles
        Returns True if there is an intersection, otherwise False.
        """
        # Direction vector of the line
        dx = s2[0] - s1[0]
        dy = s2[1] - s1[1]
        for obstacle in obstacles:
            # print(f"Checking to see whether {s1} and {s2} intersect with {obstacle}.")
            # Quadratic coefficients (a, b, c) for the line-circle intersection
            a = dx**2 + dy**2
            b = 2 * (dx * (s1[0] - obstacle[0]) + dy * (s1[1] - obstacle[1]))
            c = (s1[0] - obstacle[0])**2 + (s1[1] - obstacle[1])**2 - obstacle[2]**2

            # Discriminant of the quadratic equation
            discriminant = b**2 - 4 * a * c

            # If the discriminant is negative, no intersection
            if discriminant >= 0:
                print(f"{s1} and {s2} intersect with {obstacle}.")
                return True
            
        return False

    # Function to calculate gamma_RRT_star
    def calculate_gamma(self, Xmax, Ymax, d):
        """
        Calculate the gamma_RRT* value for 2D space.

        Parameters:
        - Xmax: Max x coordinate of the configuration space.
        - Ymax: Max y coordinate of the configuration space.
        - obstacles: List of obstacles, each represented as (x, y, radius) for circular obstacles.

        Returns:
        - gamma_RRT_star: The value of gamma for asymptotic optimality.
        """
        total_area = Xmax * Ymax  
        zeta_d = math.pi
        gamma_rrt_star = math.pow(2*(1 + 1/d), (1/d)) * math.pow(total_area/zeta_d, (1/d))

        return gamma_rrt_star
    
    def calculate_k(self, d, tol):
        return (2 ** (d + 1)) * math.e * (1 + 1/d) + tol

    def nearest(self, graph, x):
        min_distance = float('inf')
        nearest_vertex = None
        # nearest_edge = None

        for vertex in graph.nodes(): 
            distance = self.distance_computator.get_distance(x, vertex)

            if distance < min_distance:
                min_distance = distance
                nearest_vertex = vertex
        return nearest_vertex

            
    def steer(self, graph, xnearest, xrand, eta):
        """Generate a new point xnew by steering from xnearest towards xrand"""
        distance = self.distance_computator.get_distance(xrand, xnearest)
        # print(f"In the steer funtion. The distance between {xrand} and {xnearest} is {distance}.")
        direction = xnearest - xrand
        
        if distance <= eta:
            return xnearest
        
        if distance > eta:
            direction = (direction / distance) * eta
            # print(f"In the steer funtion. The direction between {xrand} and {xnearest} is {direction}.")
            xnew = xrand + direction        
        
        return tuple(xnew)

    def rnear(self, graph, xnew, max_distance):
        """Find the near neighbors of xnew"""
        near_neighbors = []
        for vertex in graph.nodes(): 
            # Skip the check for xnew itself
            if np.array_equal(np.array(vertex), np.array(xnew)):
                continue  # Don't add xnew to its own near neighbors list

            if self.distance_computator.get_distance(np.array(vertex), np.array(xnew)) < max_distance:
                near_neighbors.append(vertex)

        return near_neighbors
    
    def knear(self, graph, xnew, k):
        """Find the k-nearest neighbors of xnew"""
        near_neighbors = []
        distances = []

        # Calculate the distance from xnew to each vertex in the graph
        for vertex in graph.nodes():
            # Skip the check for xnew itself
            if np.array_equal(np.array(vertex), np.array(xnew)):
                continue  # Don't add xnew to its own near neighbors list

            # Compute the distance from xnew to the current vertex
            dist = self.distance_computator.get_distance(np.array(vertex), np.array(xnew))
            distances.append((vertex, dist))

        # Sort the distances in ascending order and pick the first k neighbors
        distances.sort(key=lambda x: x[1])  # Sort by distance

        # Extract the k nearest neighbors
        for i in range(min(int(k), len(distances))):  # Ensure we don't exceed the number of vertices
            near_neighbors.append(distances[i][0])

        return near_neighbors

    def parent(self, graph, v):
        """Returns the parent of node v in a directed graph."""
        for v1, v2 in graph.edges():  # Iterate through edges connected to v
            if v == v2:  # Check if v is the destination of the edge
                return v1 
            if v == v1:  # Check if v is the destination of the edge
                return v2
        return None 
    
    def chooseparent(self, graph, root, qs, qn, Qnear):
        qmin = qn
        min_cost = self.cost(graph, root, qn) + get_euclidean_distance(np.array(qn), np.array(qs))

        for q in Qnear:
            if not self.collision_checker.is_in_collision(q) and not self.collision_checker.is_in_collision(qs):
                current_cost = self.cost(graph, q, root) + get_euclidean_distance(np.array(q), np.array(qs))
                if current_cost < min_cost:
                    qmin = q
                    min_cost = current_cost

        return qmin

    def cost(self, graph, root, goal):
        """Calculates the total cost of the path from root to goal by iterating through the path vertices."""
        if get_euclidean_distance(np.array(root), np.array(goal)) < self.tol:
            return 0

        path = nx.shortest_path(graph, root, goal)
        
        total_cost = 0
        
        # If the path is None or empty, return 0 
        if path is None or len(path) == 0:
            return 0
        
        # Iterate through the path, calculate Euclidean distance between consecutive nodes
        for i in range(1, len(path)):
            parent = path[i-1]
            child = path[i]
            
            # If parent is None, we stop the calculation
            if parent is None:
                break
            
            total_cost += get_euclidean_distance(np.array(parent), np.array(child))

        return total_cost

    def shortest_path(self, graph, qG, region_radius, root):
        # Initialize the minimum cost and path
        min_cost = float('inf')
        best_path = []

        # Iterate over all the vertices in the graph
        for vertex in graph.nodes():
            # Calculate the distance between the vertex and the goal (qG)
            distance_to_goal = self.distance_computator.get_distance(vertex, np.array(qG))
            
            # Check if the distance is within the robot's radius
            if distance_to_goal <= region_radius:
                # Calculate the shortest path from the root to this vertex
                try:
                    path = nx.shortest_path(graph, source=root, target=vertex, weight='weight')  # Assuming weight is defined
                    # Calculate the total distance (cost) of the path
                    path_cost = 0
                    for u, v in zip(path[:-1], path[1:]):
                        path_cost += np.linalg.norm(np.array(v) - np.array(u))

                    # If this path is shorter, update the best path
                    if path_cost < min_cost:
                        min_cost = path_cost
                        best_path = path

                except nx.NetworkXNoPath:
                    # If there's no path between root and this vertex, skip it
                    continue

        return best_path