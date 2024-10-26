from abstract_base import AbstractEdge, AbstractGraph, AbstractTreeGraph, AbstractRoadmapGraph
import numpy as np
from collections import deque
from typing import List, Tuple, Dict, Optional, Set

class Edge(AbstractEdge):
    def __init__(self, start_config: List[float], end_config: List[float]):
        self._start = start_config
        self._end = end_config
        self._cost = self._compute_cost()
    
    def _compute_cost(self) -> float:
        return np.sqrt(sum((a - b) ** 2 for a, b in zip(self._start, self._end)))
    
    def get_cost(self) -> float:
        return self._cost
    
    def get_config_at(self, t: float) -> List[float]:
        return [
            self._start[0] + t * (self._end[0] - self._start[0]),
            self._start[1] + t * (self._end[1] - self._start[1])
        ]
    
    @property
    def start_config(self) -> List[float]:
        return self._start
    
    @property
    def end_config(self) -> List[float]:
        return self._end

class TreeGraph(AbstractTreeGraph):
    def __init__(self):
        """Initialize empty tree"""
        self._vertices = {}  
        self._edges = {}     
        self._parent = {}   
        self._next_vertex_id = 0
        self._root_id = None
    
    def add_vertex(self, config: List[float]) -> int:
        """Add vertex to tree and return its ID"""
        vertex_id = self._next_vertex_id
        self._vertices[vertex_id] = config
        self._edges[vertex_id] = []
        self._next_vertex_id += 1
        
        # The first node, is also the root node without any parent:
        if self._root_id is None:
            self._root_id = vertex_id
            self._parent[vertex_id] = None
            
        return vertex_id
    
    def add_vertex_with_parent(self, config: List[float], parent_id: int) -> int:
        """Add vertex with parent connection"""
        if parent_id not in self._vertices:
            raise ValueError(f"Parent ID {parent_id} does not exist")
            
        vertex_id = self.add_vertex(config)
        self.add_edge(parent_id, vertex_id)
        self._parent[vertex_id] = parent_id

        # Debugging statement to confirm parent assignment
        print(f"Vertex {vertex_id} added with parent {parent_id}")
        print("Current parent structure:", self._parent)
        return vertex_id

    
    def add_edge(self, v1_id: int, v2_id: int) -> None:
        """Add edge between vertices"""
        if v1_id not in self._vertices or v2_id not in self._vertices:
            raise ValueError("Both vertices must exist before adding an edge")
        
        # Create edge object
        edge = Edge(self._vertices[v1_id], self._vertices[v2_id])
        
        # Add to both vertices' edge lists (undirected edge)
        self._edges[v1_id].append((v2_id, edge))
        self._edges[v2_id].append((v1_id, edge))

    def remove_edge(self, v1_id: int, v2_id: int) -> None:
        """Remove edge between two vertices"""
        # Remove from v1's edges
        self._edges[v1_id] = [(vid, edge) for vid, edge in self._edges[v1_id] 
                             if vid != v2_id]
        
        # Remove from v2's edges
        self._edges[v2_id] = [(vid, edge) for vid, edge in self._edges[v2_id] 
                             if vid != v1_id]
        
        # Update parent relationships if needed
        if self._parent.get(v2_id) == v1_id:
            self._parent[v2_id] = None
        if self._parent.get(v1_id) == v2_id:
            self._parent[v1_id] = None
        
    def get_vertex_config(self, vertex_id: int) -> List[float]:
        return self._vertices[vertex_id]
    
    def get_neighbors(self, vertex_id: int) -> List[Tuple[int, Edge]]:
        return self._edges[vertex_id]
    
    def get_path_to_root(self, vertex_id: int) -> List[List[float]]:
        if vertex_id not in self._vertices:
            raise ValueError(f"Vertex ID {vertex_id} does not exist")
    
        path = []
        current_id = vertex_id
        # Prventing infinite loops
        visited = set()
        
        while current_id is not None:
            if current_id in visited:
                raise ValueError("Cycle detected in tree")
            
            path.append(self._vertices[current_id])
            visited.add(current_id)
                
            if current_id not in self._parent:
                # This is the root node
                if len(path) == 1:  
                    break
                else:
                    print(f"No parent found for vertex {current_id}. Current path: {path}")
                    raise ValueError(f"No parent found for vertex {current_id}")
                
            current_id = self._parent[current_id]
            
        return path[::-1]
    
    def get_path(self, vertex_ids: List[int]) -> List[List[float]]:
        """Convert list of vertex IDs to list of configurations"""
        return [self._vertices[vid] for vid in vertex_ids]
    
    @property
    def vertices(self) -> Dict[int, List[float]]:
        return self._vertices
    
    @property
    def edges(self) -> Dict[int, List[Tuple[int, Edge]]]:
        return self._edges
    
    @property
    def root_id(self) -> int:
        return self._root_id
    
    @property
    def parent(self) -> Dict[int, Optional[int]]:
        return self._parent

class RoadmapGraph(AbstractGraph):
    def __init__(self):
        """Initialize empty roadmap"""
        self._vertices = {}  
        self._edges = {}     
        self._next_vertex_id = 0

    def add_vertex(self, config: List[float]) -> int:
        """Add vertex to roadmap and return its ID"""
        vertex_id = self._next_vertex_id
        self._vertices[vertex_id] = config
        self._edges[vertex_id] = []
        self._next_vertex_id += 1
        return vertex_id

    def add_edge(self, v1_id: int, v2_id: int) -> None:
        """Add edge between vertices"""
        # Create edge object
        edge = Edge(self._vertices[v1_id], self._vertices[v2_id])
        
        # Add to both vertices' edge lists (undirected edge)
        self._edges[v1_id].append((v2_id, edge))
        self._edges[v2_id].append((v1_id, edge))

    def get_vertex_config(self, vertex_id: int) -> List[float]:
        """Get configuration of vertex"""
        return self._vertices[vertex_id]

    def get_neighbors(self, vertex_id: int) -> List[Tuple[int, Edge]]:
        """Get list of (neighbor_id, edge) pairs"""
        return self._edges[vertex_id]

    def get_path(self, vertex_ids: List[int]) -> List[List[float]]:
        """Convert list of vertex IDs to list of configurations"""
        return [self._vertices[vid] for vid in vertex_ids]

    @property
    def vertices(self) -> Dict[int, List[float]]:
        """Get dictionary of vertex_id -> configuration"""
        return self._vertices

    @property
    def edges(self) -> Dict[int, List[Tuple[int, Edge]]]:
        """Get dictionary of vertex_id -> list of (neighbor_id, edge) pairs"""
        return self._edges

    def connect_to_neighbors(self, vertex_id: int, k: int, valid_edge: callable) -> None:
        """
        Connect vertex to k nearest neighbors if path is valid
        """
        # Get all vertices except the current one
        other_vertices = [(vid, config) for vid, config in self._vertices.items() 
                         if vid != vertex_id]
        
        if not other_vertices:
            return

        # Calculate distances to all other vertices
        distances = []
        current_config = self._vertices[vertex_id]
        for vid, config in other_vertices:
            dist = np.sqrt(sum((a - b) ** 2 for a, b in zip(current_config, config)))
            distances.append((vid, dist))

        # Sort by distance
        distances.sort(key=lambda x: x[1])

        # Try to connect to k nearest neighbors
        for vid, _ in distances[:k]:
            if valid_edge(current_config, self._vertices[vid]):
                self.add_edge(vertex_id, vid)

    def find_path(self, start_id: int, goal_id: int) -> Optional[List[int]]:
        """Find path between start and goal vertices using BFS"""
        if start_id not in self._vertices or goal_id not in self._vertices:
            return None

        # Use BFS to find path
        queue = deque([(start_id, [start_id])])
        visited = {start_id}

        while queue:
            current_id, path = queue.popleft()
            if current_id == goal_id:
                return path

            for next_id, _ in self._edges[current_id]:
                if next_id not in visited:
                    visited.add(next_id)
                    queue.append((next_id, path + [next_id]))

        return None