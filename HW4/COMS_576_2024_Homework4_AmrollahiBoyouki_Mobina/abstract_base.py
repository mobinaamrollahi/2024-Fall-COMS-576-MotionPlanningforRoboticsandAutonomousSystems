from abc import ABC, abstractmethod
from typing import List, Tuple, Dict, Optional, Set

class AbstractEdge(ABC):
    """Abstract base class for edges in a graph"""
    
    @abstractmethod
    def __init__(self, start_config: List[float], end_config: List[float]):
        """
        Initialize an edge between two configurations
        
        Args:
            start_config: Starting configuration [x, y]
            end_config: Ending configuration [x, y]
        """
        pass
    
    @abstractmethod
    def get_cost(self) -> float:
        """
        Get the cost of traversing this edge
        
        Returns:
            Cost value (typically distance)
        """
        pass
    
    @abstractmethod
    def get_config_at(self, t: float) -> List[float]:
        """
        Get configuration at parameter t along the edge
        
        Args:
            t: Parameter between 0 and 1 (0 = start, 1 = end)
        Returns:
            Configuration at parameter t
        """
        pass
    
    @property
    @abstractmethod
    def start_config(self) -> List[float]:
        """Get start configuration"""
        pass
    
    @property
    @abstractmethod
    def end_config(self) -> List[float]:
        """Get end configuration"""
        pass

class AbstractGraph(ABC):
    """Abstract base class for graphs"""
    
    @abstractmethod
    def __init__(self):
        """Initialize empty graph"""
        pass
    
    @abstractmethod
    def add_vertex(self, config: List[float]) -> int:
        """
        Add vertex to graph
        
        Args:
            config: Configuration [x, y]
        Returns:
            Vertex ID
        """
        pass
    
    @abstractmethod
    def add_edge(self, v1_id: int, v2_id: int) -> None:
        """
        Add edge between vertices
        
        Args:
            v1_id: First vertex ID
            v2_id: Second vertex ID
        """
        pass
    
    @abstractmethod
    def get_vertex_config(self, vertex_id: int) -> List[float]:
        """
        Get configuration of vertex
        
        Args:
            vertex_id: ID of vertex
        Returns:
            Configuration of vertex
        """
        pass
    
    @abstractmethod
    def get_neighbors(self, vertex_id: int) -> List[Tuple[int, 'AbstractEdge']]:
        """
        Get neighbors of vertex
        
        Args:
            vertex_id: ID of vertex
        Returns:
            List of (neighbor_id, edge) pairs
        """
        pass
    
    @abstractmethod
    def get_path(self, vertex_ids: List[int]) -> List[List[float]]:
        """
        Convert list of vertex IDs to list of configurations
        
        Args:
            vertex_ids: List of vertex IDs representing a path
        Returns:
            List of configurations along the path
        """
        pass
    
    @property
    @abstractmethod
    def vertices(self) -> Dict[int, List[float]]:
        """Get dictionary of vertex_id -> configuration"""
        pass
    
    @property
    @abstractmethod
    def edges(self) -> Dict[int, List[Tuple[int, 'AbstractEdge']]]:
        """Get dictionary of vertex_id -> list of (neighbor_id, edge) pairs"""
        pass

class AbstractTreeGraph(AbstractGraph):
    """Abstract base class for tree graphs (used in RRT)"""
    @abstractmethod
    def add_vertex(self, config: List[float]) -> int:
        pass

    @abstractmethod
    def add_vertex_with_parent(self, config: List[float], parent_id: int) -> int:
        """
        Add vertex with parent connection
        
        Args:
            config: Configuration of new vertex
            parent_id: ID of parent vertex
        Returns:
            ID of new vertex
        """
        pass

    @abstractmethod
    def add_edge(self, v1_id: int, v2_id: int) -> None:
        pass

    @abstractmethod
    def remove_edge(self, v1_id: int, v2_id: int) -> None:
        """Remove edge between two vertices"""
        pass

    @abstractmethod
    def get_vertex_config(self, vertex_id: int) -> List[float]:
        pass
    
    @abstractmethod
    def get_neighbors(self, vertex_id: int) -> List[Tuple[int, 'AbstractEdge']]:
        pass

    @abstractmethod
    def get_path_to_root(self, vertex_id: int) -> List[List[float]]:
        """
        Get path from vertex to root
        
        Args:
            vertex_id: ID of vertex
        Returns:
            List of configurations from root to vertex
        """
        pass

    @abstractmethod
    def get_path(self, vertex_ids: List[int]) -> List[List[float]]:
        pass
    
    @property
    @abstractmethod
    def vertices(self) -> Dict[int, List[float]]:
        pass
    
    @property
    @abstractmethod
    def edges(self) -> Dict[int, List[Tuple[int, 'AbstractEdge']]]:
        pass
    
    @property
    @abstractmethod
    def root_id(self) -> int:
        """Get ID of root vertex"""
        pass
    
    @property
    @abstractmethod
    def parent(self) -> Dict[int, Optional[int]]:
        """Get dictionary of vertex_id -> parent_id"""
        pass

class AbstractRoadmapGraph(AbstractGraph):
    """Abstract base class for roadmap graphs (used in PRM)"""
    
    @abstractmethod
    def connect_to_neighbors(self, vertex_id: int, k: int, valid_edge: callable) -> None:
        """
        Connect vertex to k nearest neighbors if path is valid
        
        Args:
            vertex_id: Vertex to connect
            k: Number of nearest neighbors to try
            valid_edge: Function(config1, config2) -> bool to check if edge is valid
        """
        pass
    
    @abstractmethod
    def find_path(self, start_id: int, goal_id: int) -> Optional[List[int]]:
        """
        Find path between start and goal vertices
        
        Args:
            start_id: ID of start vertex
            goal_id: ID of goal vertex
        Returns:
            List of vertex IDs representing path, or None if no path exists
        """
        pass