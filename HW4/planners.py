from typing import List, Tuple, Optional, Dict
import numpy as np
from collections import deque
import random
from abstract_base import AbstractTreeGraph, AbstractRoadmapGraph
from graph_impl import TreeGraph, RoadmapGraph, Edge

class PlannerFunctions:
    """
    Common functions that are used by all planners:
    is_valid_config: it will check whether a certain point lies in the configuration space.
    check_collision: it will check if a certain point is in collision with obstacles. 
    sample_random: generates a random point in the configuration space.
    extend: this function is mainly used by the single-tree search method. It will check periodically if the tree can be connected to qG.
    """
    
    @staticmethod
    def is_valid_config(q: List[float], C: List[List[float]], O: List[List[float]], radius: float) -> bool:
        """Check if configuration is valid (within C-space and not in collision)"""
        # Check C-space bounds
        print(f"\nChecking validity of point {q}")

        in_bounds = (C[0][0] <= q[0] <= C[0][1] and C[1][0] <= q[1] <= C[1][1])
        print(f"In bounds: {in_bounds}")

        if not in_bounds:
            print(f"Point {q} outside bounds")
            return False
        
        print("Point in bounds")

        # For problem 1a, O is empty so skip collision check
        if not O:
            print("No obstacles, point is valid")
            return True

        for obstacle in O:
            # Calculate distance from q to obstacle center
            p1 = np.array(obstacle)
            p2 = np.array(q)
            distance = np.linalg.norm(p1 - p2)

            print(f"Distance to obstacle at {obstacle}: {distance}")
            print(f"Radius: {radius}")
            
            # If point is within radius of obstacle center
            if distance < radius:
                    return False
                
        print("Point is valid (no collisions)")                  
        # No collision found
        return True
    
    @staticmethod
    def sample_random(C: List[List[float]]) -> List[float]:
        """Generate random configuration within C-space bounds"""
        return [
            random.uniform(C[0][0], C[0][1]),
            random.uniform(C[1][0], C[1][1])
        ]
    
    @staticmethod
    def extend(q_near: List[float], qG: List[float], step_size: float) -> List[float]:
        """Extend from q_near towards qG by step_size"""
        direction = np.array(qG) - np.array(q_near)
        distance = np.linalg.norm(direction)
        
        if distance < step_size:
            return qG
        
        direction = direction / distance * step_size
        return (np.array(q_near) + direction).tolist()

class RRT:
    def __init__(self, C: List[List[float]], O: List[List[float]], radius: float,
                 qI: List[float], step_size: float, n_iterations: int):
        self.C = C
        self.O = O
        self.radius = radius
        self.qI = qI  # Initial configuration
        self.step_size = step_size
        self.n_iterations = n_iterations
        self.functions = PlannerFunctions()
        # Initialize tree with start configuration
        self.tree = TreeGraph()
        self.tree.add_vertex(self.qI)

    def nearest(self, q: List[float]) -> int:
        """Find nearest vertex in the tree to configuration q"""
        min_dist = float('inf')
        nearest_id = None
        
        for vid, config in self.tree.vertices.items():
            dist = np.linalg.norm(np.array(config) - np.array(q))
            if dist < min_dist:
                min_dist = dist
                nearest_id = vid
        
        return nearest_id

    def stopping_configuration(self, q_near: List[float], q_rand: List[float]) -> List[float]:
        """Find stopping configuration when extending from q_near toward q_rand"""
        direction = np.array(q_rand) - np.array(q_near)
        distance = np.linalg.norm(direction)
        
        if distance == 0:
            return q_near
            
        if distance > self.step_size:
            direction = direction / distance * self.step_size
            q_new = np.array(q_near) + direction
        else:
            q_new = np.array(q_rand)
            
        if self.functions.is_valid_config(q_new.tolist(), self.C, self.O, self.radius):
            return q_new.tolist()
        return q_near

    def explore(self) -> AbstractTreeGraph:
        """
        RDT algorithm
        """
        for i in range(self.n_iterations):
            # Generate random configuration α(i)
            q_rand = self.functions.sample_random(self.C)
            
            # Find nearest vertex
            nearest_id = self.nearest(q_rand)
            q_near = self.tree.get_vertex_config(nearest_id)
            
            # Find stopping configuration
            q_stop = self.stopping_configuration(q_near, q_rand)
            
            # If made progress, add new vertex and edge
            if q_stop != q_near:
                new_id = self.tree.add_vertex(q_stop)
                self.tree.add_edge(nearest_id, new_id)
                
        return self.tree

    def plan_to_goal(self, qG: List[float], p: float) -> Tuple[Optional[List[List[float]]], AbstractTreeGraph]:
        """
        Single-tree RRT with goal bias:
        Same as explore, but with probability p, sample goal instead of random configuration.
        """
        for i in range(self.n_iterations):
            # With probability p, sample the goal; otherwise sample a random configuration
            q_rand = qG if random.random() < p else self.functions.sample_random(self.C)
            print("q_rand", q_rand)
            
            # Find nearest vertex
            nearest_id = self.nearest(q_rand)
            q_near = self.tree.get_vertex_config(nearest_id)
            
            # Find stopping configuration
            q_stop = self.stopping_configuration(q_near, q_rand)
            print("q_stop", q_stop)
            
            # If made progress
            if q_stop != q_near:
                new_id = self.tree.add_vertex(q_stop)
                self.tree.add_edge(nearest_id, new_id)
                # We nee to set a parent for the new_id
                self.tree._parent[new_id] = nearest_id
                print(f"Added vertex {new_id} with parent {nearest_id}")
                
            # If we sampled the goal and reached it
            if q_stop == qG:
              return self.tree.get_path_to_root(new_id), self.tree

            '''   
            # If we can reach the goal from new vertex
            if np.linalg.norm(np.array(q_stop) - np.array(qG)) < self.step_size:
                q_final = self.stopping_configuration(q_stop, qG)
                if q_final != q_stop and np.allclose(q_final, qG):
                    goal_id = self.tree.add_vertex(qG)
                    self.tree._parent[goal_id] = new_id  # Set parent for goal_id
                    
                    # Ensure goal_id is connected to the tree
                    self.tree.add_edge(new_id, goal_id)
                    print(f"Goal vertex {goal_id} added with parent {new_id}")
                    
                    # Verify goal_id has a parent
                    if goal_id not in self.tree._parent:
                        raise ValueError(f"Goal vertex {goal_id} was not correctly connected to the tree.")
                    
                    return self.tree.get_path_to_root(goal_id), self.tree
            '''
        # Return None if no path found after all iterations
        return None, self.tree
    
class PRM:
    def __init__(self, C: List[List[float]], O: List[List[float]], radius: float,
                 N: int, K: int, step_size: float):
        self.C = C
        self.O = O
        self.radius = radius
        self.N = N  # number of samples
        self.K = K  # number of nearest neighbors
        self.step_size = step_size
        self.functions = PlannerFunctions()

    def build_roadmap(self) -> AbstractRoadmapGraph:
        """
        BUILD ROADMAP Algorithm
        """
        # 1. Initialize roadmap
        roadmap = RoadmapGraph()
        i = 0

        # 2. While we haven't added N vertices
        while i < self.N:
            # Generate random configuration
            alpha_i = self.functions.sample_random(self.C)
            
            # 3. Check if configuration is in Cfree
            if self.functions.is_valid_config(alpha_i, self.C, self.O, self.radius):
                # 4. Add vertex
                current_id = roadmap.add_vertex(alpha_i)
                print(f"Added vertex {i}: {alpha_i}")
                
                # 5. Increment counter
                i += 1
                
                # 6. Find neighborhood
                neighbors = self.find_neighborhood(alpha_i, roadmap)
                print(f"Found {len(neighbors)} neighbors")
                
                # 7. For each neighbor
                for neighbor_id, neighbor_config in neighbors:
                    # Check if not in same component and can connect
                    if (not self.same_component(current_id, neighbor_id, roadmap) and 
                        self.connect(alpha_i, neighbor_config)):
                        # Add edge
                        roadmap.add_edge(current_id, neighbor_id)
                        print(f"Added edge between {current_id} and {neighbor_id}")

        return roadmap

    def find_neighborhood(self, config: List[float], roadmap: RoadmapGraph) -> List[Tuple[int, List[float]]]:
        """Find K nearest neighbors in the roadmap"""
        distances = []
        for vid, other_config in roadmap.vertices.items():
            dist = np.sqrt(sum((a - b) ** 2 for a, b in zip(config, other_config)))
            distances.append((vid, other_config, dist))
        
        # Sort by distance and return K nearest
        distances.sort(key=lambda x: x[2])
        return [(vid, config) for vid, config, _ in distances[:self.K]]

    def same_component(self, v1_id: int, v2_id: int, roadmap: RoadmapGraph) -> bool:
        """Check if two vertices are in the same connected component using BFS"""
        if v1_id == v2_id:
            return True
            
        visited = {v1_id}
        queue = deque([v1_id])
        
        while queue:
            current = queue.popleft()
            for neighbor_id, _ in roadmap.get_neighbors(current):
                if neighbor_id == v2_id:
                    return True
                if neighbor_id not in visited:
                    visited.add(neighbor_id)
                    queue.append(neighbor_id)
        
        return False

    def connect(self, q1: List[float], q2: List[float]) -> bool:
        """Check if two configurations can be connected"""
        direction = np.array(q2) - np.array(q1)
        distance = np.linalg.norm(direction)
        
        # Check intermediate points
        n_steps = max(int(np.ceil(distance / self.step_size)), 1)
        for i in range(n_steps + 1):
            t = i / n_steps
            q = np.array(q1) + t * direction
            if not self.functions.is_valid_config(q.tolist(), self.C, self.O, self.radius):
                return False
        return True

    def plan_path(self, qI: List[float], qG: List[float]) -> Tuple[Optional[List[List[float]]], AbstractRoadmapGraph]:
        """Build roadmap and plan path from qI to qG"""
        # print("Starting to build roadmap")
        roadmap = self.build_roadmap()
        
        # Add start and goal
        start_id = roadmap.add_vertex(qI)
        goal_id = roadmap.add_vertex(qG)
        
        # Find neighbors for start and goal
        for config_id, config in [(start_id, qI), (goal_id, qG)]:
            neighbors = self.find_neighborhood(config, roadmap)
            for neighbor_id, neighbor_config in neighbors:
                if self.connect(config, neighbor_config):
                    roadmap.add_edge(config_id, neighbor_id)
        
        # Find path
        path_ids = roadmap.find_path(start_id, goal_id)
        if path_ids is not None:
            # print("A path is found")
            return roadmap.get_path(path_ids), roadmap
            
        # print("There's not path.")
        return None, roadmap