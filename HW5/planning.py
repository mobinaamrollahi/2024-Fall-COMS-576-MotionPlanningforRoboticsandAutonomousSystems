import networkx as nx
import dubins
import random
import numpy as np
import math
from obstacle import WorldBoundary2D
from geometry import get_euclidean_distance
from tqdm import tqdm
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


class DubinsDistanceComputator(DistanceComputator):
    def __init__(self, rad):
        self.rad = rad  # Store the turning radius

    def get_distance(self, s1, s2, mode=0):
        """Return the Dubins distance between s1 and s2 using Dubins path"""
        path1 = dubins.shortest_path(s1, s2, self.rad)
        path2 = dubins.shortest_path(s2, s1, self.rad)

        if mode == 0:
            return min(path1.path_length(), path2.path_length())
        elif mode == 1:
            return path2.path_length()
        else:
            return path1.path_length()
        
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
    def is_in_collision(self, state, Xmin, Xmax, Ymin, Ymax):
        """Return whether the given state is in collision"""
        self.Xmin = Xmin
        self.Xmax = Xmax
        self.Ymin = Ymin
        self.Ymax = Ymax
        return False
    
    def not_a_valid_config(self, s):
        """Return whether the given state puts the point outside world boundaries"""
        # Check if any part of the robot goes beyond boundaries
        if (s[0] < self.Xmin or  # Left boundary
            s[0] > self.Xmax or  # Right boundary
            s[1] < self.Ymin or  # Bottom boundary
            s[1] > self.Ymax):   # Top boundary
            return False
        return True

    def is_checking_required(self):
        """Return whether collision needs to be checked at all"""
        return True


class ObstacleCollisionChecker(CollisionChecker):
    def __init__(self, obstacles, Xmin, Xmax, Ymin, Ymax):
        """The constructor
        @type obstacles: a list [obs_1, ..., obs_m] of obstacles, where obs_i is an Obstacle
            object that include a contain(s) function, which returns whether a state s
            is inside the obstacle
        """
        self.obstacles = obstacles
        self.Xmin = Xmin
        self.Xmax = Xmax
        self.Ymin = Ymin
        self.Ymax = Ymax

    def is_in_collision(self, s):
        """Return whether a point s is in collision with the obstacles"""
        # Calculate distance between centers using Euclidean distance
        # print("s", s)
        for obstacle in self.obstacles:
            # print("obstacle.y", obstacle.y)
            # print("s[0]", s[0])
            dx = s[0] - obstacle.x
            dy = s[1] - obstacle.y
            distance = math.sqrt((dx)**2 + (dy)**2)
            if distance <= obstacle.r:
                return True
        return False
    
    def not_a_valid_config(self, s):
        """Return whether the given state puts the car outside world boundaries"""
        # Check if any part of the robot goes beyond boundaries
        if (s[0] < self.Xmin or  # Left boundary
            s[0] > self.Xmax or  # Right boundary
            s[1] < self.Ymin or  # Bottom boundary
            s[1] > self.Ymax):   # Top boundary
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

    def connect(s1, s2, step_size, collision_checker, turning_rad, tol):
        """Return whether a path s1 and s2 is collision-free"""
        pass

class DubinsCarPathPlanning(PathPlanning):
    def __init__(self, cspace, Xmin, Xmax, Ymin, Ymax, qI, qG, turning_rad, obstacles):
        """
        The constructor
        Xmin, Xmax, Ymin, Ymax: floats that defines the boundary of the world
        O: a list of tuples (x, y, r) that represent the center (x,y) and radius (r) of the circular obstacle.
        turning_rad: the turning radius of the robot
        @type cspace: a list of tuples (smin, smax) indicating that the C-space
            is given by the product of the tuples.
        @type qI: a tuple (x, y, theta) indicating the initial configuration.
        @type qG: a typle (x, y, theta) indicating the goal configuation
            (can be None if rrt is only used to explore the C-space).
        """
        self.qI = qI
        self.qG = qG
        self.turning_rad = turning_rad
        self.Xmin = Xmin
        self.Xmax = Xmax
        self.Ymin = Ymin
        self.Ymax = Ymax
        self.cspace = cspace
        self.O = obstacles
        self.G = nx.DiGraph()

         # Create instances to be compatiable with the rest of the code
        self.collision_checker = ObstacleCollisionChecker(self.O, self.Xmin, self.Xmax, self.Ymin, self.Ymax)
        self.distance_computator = DubinsDistanceComputator(self.turning_rad)

    def rrt(self,
        pG,
        numIt,
        tol,
        step_size,
    ):
        """RRT with obstacles
        @type pG: a float indicating the probability of choosing the goal configuration.
        @type numIt: an integer indicating the maximum number of iterations.
        @type tol: a float, indicating the tolerance on the euclidean distance when checking whether
            2 states are the same

        @return (G, root, goal) where G is the tree, root is the id of the root vertex
            and goal is the id of the goal vertex (if one exists in the tree; otherwise goal will be None).
        """
        print("self.qG type:", self.qG)
        print("self.qI type:", self.qI)
        root = self.G.add_node(tuple(self.qI), tipping_point = 0)
        print("root is", root)
        for i in tqdm(range(numIt), desc="Iterations", unit="iteration"):        
            use_goal = tuple(self.qG) is not None and random.uniform(0, 1) <= pG
            if use_goal:
                alpha = self.qG
                print("alpha is equal to goal")
            else:
                alpha = self.sample(self.cspace)
                # alpha = ConfigDubins(alpha)
                # print("alpha", alpha)

            nearest_node = None
            closest_dist = math.inf
            best_path = None
            
            for v in self.G.nodes:
                # print("v", np.array(v))
                s0 = alpha
                s1 = v
                # print("s1:", s1)
                # print("s0:", s0)

                path = dubins.shortest_path(s1, s0, self.turning_rad)
                if closest_dist == None or path.path_length() < closest_dist:
                    # print("path length is:", path.path_length())
                    nearest_node = v
                    closest_dist = path.path_length()
                    best_path = path

            check, vs = self.stopping_configuration(nearest_node = nearest_node, alpha = alpha, path = best_path, 
                                                     tol = tol, step_size = step_size)
            #print("The graph is:", self.G)
            # print("vs, self.qG", vs, tuple(self.qG))
            # print("vs - self.qG", vs - tuple(self.qG))
            # Assuming vs and self.qG are tuples
            if vs is not None:
                vs_array = np.array(vs)
                qG_array = np.array(self.qG)

                # Only subtract the first two entries (x, y) from both vs and self.qG
                vs_sub = vs_array[:-1]  # Take the first two elements (x, y)
                qG_sub = qG_array[:-1]  # Take the first two elements (x, y)
                # print("The distance between vs and tuple(self.qG) for the first two entries is:", get_euclidean_distance(vs_sub, qG_sub))
            # Now calculate the Euclidean distance only for the first two entries (x, y)
            #   distance = get_euclidean_distance(vs_sub, qG_sub)

                # Check if the distance is less than the tolerance
                if get_euclidean_distance(vs_sub, qG_sub) < tol:
                    print(f"The distance between vs and tuple(self.qG) for the first two entries is: {get_euclidean_distance(vs_sub, qG_sub)}")
                if check and use_goal and get_euclidean_distance(vs_sub, qG_sub) < tol :
                    return (self.G, tuple(self.qI), tuple(self.qG))

        return (self.G, tuple(self.qI), None)


    def prm(self,
        k,
        numIt,
        tol,
        step_size,
    ):
        """PRM with obstacles
        @type k: a float, indicating the number of nearest neighbors
        @return (G, root, goal) where G is the roadmap, root is the id of the root vertex
            and goal is the id of the goal vertex.
            If the root (resp. goal) vertex does not exist in the roadmap, root (resp. goal) will be None.
        """
        '''
        def add_to_roadmap(G, alpha):
            """Add configuration alpha to the roadmap G"""
            if self.collision_checker.is_in_collision(alpha):
                return None
            
            nodes = list(self.G.nodes)
            nodes.sort(key=lambda x: self.distance_computator.get_distance(x, alpha, mode=0)) 
            # Extract the k-nearest neighbor of the vertices to alpha
            neighbors = nodes[:k]
        
            self.G.add_node(alpha)
            for vn in neighbors:
                if (not nx.has_path(self.G, alpha, vn)):
                            self.connect(alpha, vn, step_size)
                if (not nx.has_path(self.G, vn, alpha)):
                            self.connect(vn, alpha, step_size)

                return alpha
        '''
        
            
        # with tqdm(total=numIt, desc="Iterations", unit="iteration") as pbar:
        for i in range(numIt):
            alpha = self.sample(self.cspace)
            print("i:", i)
            print("alpha:", tuple(alpha))
            # alpha = ConfigDubins(alpha)
            if not self.collision_checker.is_in_collision(tuple(alpha)):
                nodes = list(self.G.nodes)
                # print("node", nodes)
                nodes.sort(key=lambda x: self.distance_computator.get_distance(x, tuple(alpha), mode=0)) 
                # Extract the k-nearest neighbor of the vertices to alpha
                neighbors = nodes[:k]
                print("neighbors", neighbors)
                # Print the number of neighbors
                print(f"Number of neighbors: {len(neighbors)}")

                self.G.add_node(tuple(alpha))
                for vn in neighbors:
                    if (not nx.has_path(self.G, tuple(alpha), vn)):
                                self.connect(tuple(alpha), vn, step_size)
                    if (not nx.has_path(self.G, vn, tuple(alpha))):
                                self.connect(vn, tuple(alpha), step_size)

        nodes = list(self.G.nodes)
        nodes.sort(key=lambda x: self.distance_computator.get_distance(x, tuple(self.qI), mode=1)) 
        # Extract the k-nearest neighbor of the vertices to alpha
        neighbors = nodes[:k]
        self.G.add_node(tuple(self.qI))

        if tuple(self.qI) is not None and not self.collision_checker.is_in_collision(tuple(self.qI)) and not self.collision_checker.not_a_valid_config(tuple(self.qI)):
            for vn in neighbors:
                if (not nx.has_path(self.G, tuple(self.qI), vn)):
                    #print("no path between qI, and", vn)
                    self.connect(tuple(self.qI), tuple(vn), step_size)


        nodes = list(self.G.nodes)
        nodes.sort(key=lambda x: self.distance_computator.get_distance(x, tuple(self.qG), mode=2)) 
        # Extract the k-nearest neighbor of the vertices to alpha
        neighbors = nodes[:k]
        self.G.add_node(tuple(self.qG))

        if tuple(self.qG) is not None and not self.collision_checker.is_in_collision(tuple(self.qG)) and not self.collision_checker.not_a_valid_config(tuple(self.qG)):
            for vn in neighbors:
                if (not nx.has_path(self.G, tuple(self.qG), vn)):
                    # print("no path between qg, and", vn)
                    self.connect(tuple(vn), tuple(self.qG), step_size)

        return (self.G, tuple(self.qI), tuple(self.qG))

    def sample(self, cspace):
        """Return a sample configuration of the C-space based on uniform random sampling"""
        sample = [random.uniform(cspace_comp[0], cspace_comp[1])
                for cspace_comp in cspace]
        # An appropriate range for sampling theta 
        sample.append(random.uniform(-(math.pi/2), math.pi/2))
        print("sample:", sample)
        return np.array(sample)

    def stopping_configuration(self, nearest_node, alpha, path, tol, step_size):
        """Return (configurations, edges) where configurations is the set of points along part of the Dubins curve that is collision-free.
        Edges represent the sets of edges between these configurations."""
        
        # Generate the Dubins curve path between s1 and s2
        configurations, _ = path.sample_many(step_size)
        configurations.append(tuple(alpha))
        # print("alpha", alpha)
        # print("configurations", configurations)


        curr_node = None
        prev_node = None
        flag = False
        # Iterate over configurations check collisions
        # print(len(configurations))
        for configuration in configurations[1:]:
            # print("i:", i)
            # print("Configuration shape:", np.shape(configuration))
            # print("Configuration:", configuration)

            # Skip collision check if the current configuration is 'alpha'
            # if np.array_equal(configuration, alpha):
                # print("Skipping collision check for alpha:", alpha)
            #    continue

            if flag:
                # tipping_point is when we're trying to draw the arrows
                # not all nodes can have an arrow, only the very last one of a collision-free path
                self.G.add_node(curr_node, parent=prev_node, tipping_point=1)
                # print("tipping_point:", self.G.nodes[curr_node]['tipping_point'])
                self.G.add_edge(prev_node, curr_node)
                # print("Configuration in stopping_configuration", configuration)
            
            # Check for collisions
            if self.collision_checker.is_in_collision(np.array(configuration)) or self.collision_checker.not_a_valid_config(np.array(configuration)):
                # print('np.array(configuration)',np.array(configuration))
                flag = False
                # print("Configuration is in collision")
                break
            
            flag = True
            # Update prev_node and curr_node
            if not prev_node:
                prev_node = nearest_node
            else:
                prev_node = curr_node
            
            curr_node = configuration  # Create a new ConfigDubins node

        if flag:
            # print("current node", curr_node)
            self.G.add_node(curr_node, parent=prev_node, tipping_point = 0)
            self.G.add_edge(prev_node, curr_node)
            # print("tipping_point:", self.G.nodes[curr_node]['tipping_point'])
        elif curr_node:
            # print("current node", curr_node)
            self.G.nodes[curr_node]['tipping_point'] = 0
            # print("tipping_point:", self.G.nodes[curr_node]['tipping_point'])
        
        return flag, curr_node

    def connect(self, s1, s2, step_size):
        """Return whether a dubins curve between s1 and s2 is collision-free"""
        path = dubins.shortest_path(s1, s2, self.turning_rad)
        print("lenght of the path", path.path_length())
        configurations,_ = path.sample_many(step_size)
        check = True
        for configuration in configurations:
            # configuration = ConfigDubins(configuration)
            if self.collision_checker.is_in_collision(tuple(configuration)) or self.collision_checker.not_a_valid_config(tuple(configuration)):
                check = False
                break

        if check:
            self.G.add_edge(s1, s2)

