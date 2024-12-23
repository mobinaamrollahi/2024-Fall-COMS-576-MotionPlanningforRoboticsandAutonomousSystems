from abstract_base import AbstractEdge, AbstractGraph, AbstractTreeGraph, AbstractRoadmapGraph
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import math, copy
from heapq import heappush
from typing import List, Tuple, Dict, Optional, Set
import edge_impl
from derived_queues import QueueAstar

class Graph:
    """ A class for maintaining the graph"""

    def __init__(self):
        """Initialize empty tree"""
        # key = id of the vertex; value = state of the vertex
        self._vertices = {}  

        # key = id of a vertex; value = the list of the ids of the vertex parents
        self._parent = {}   

        # key = (v1, v2); value = (cost, edge)
        # v1 is the id of the origin vertex and v2 is the id of the destination vertex.
        # cost is the cost of the edge.
        # edge is of type Edge and stores information about the edge, e.g.,
        # the origin and destination states and the discretized points along the edge
        self._edges = {}     
    
    def add_vertex(self, state: List[float]) -> int:
        """Add vertex to tree and return its ID"""
        vertex_id = len(self._vertices)
        self._vertices[vertex_id] = state
        self._parent[vertex_id] = None 
        return vertex_id
    
    def get_vertex_config(self, vertex_id) -> List[float]:
        """Get the state of the vertex with id = vertex_id"""
        return self._vertices[vertex_id]
    
    def get_vertices(self):
        return list(self._vertices.keys())

    '''
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
    '''
    
    def add_edge(self, v_id1, v_id2, edge) -> None:
        """Add edge between vertices v_1 and v_2"""
        if v_id1 not in self._vertices or v_id2 not in self._vertices:
            raise ValueError("Both vertices must exist before adding an edge")
        
        self._edges[(v_id1, v_id2)] = (
            edge.get_cost(),
            edge,
        )
        self._parent[v_id2].append(v_id1)

    def remove_edge(self, edge_id) -> None:
        """Remove edge between two vertices"""
        # Remove from v1's edges
        del self._edges[edge_id] 
        v_id1 = edge_id[0]
        v_id2 = edge_id[1]
        self._parent[v_id2].remove[v_id1]
    
    def get_nearest_neighbors(self, state, distance_computator, tol):
        """Return the vertex in the swath of the graph that is closest to the given state"""

        if len(self._edges) == 0:
            return self.get_nearest_vertex(state, distance_computator)
        
        (nearest_edge, nearest_t) = self.get_nearest_edge(state, distance_computator)
        if nearest_t <= tol:
            return nearest_edge[0]
        
        if nearest_t >= 1 - tol:
            return nearest_edge[1]
        
        return self.split_edge(nearest_edge, nearest_t)
    
    def get_nearest_edge(self, state, distance_computator):
        """Return the edge that is nearest to the given state based on the given distance function
        @type distance_computator: a DistanceComputator object that includes the get_distance(s1, s2)
            function, which returns the distance between s1 and s2.
            In this problem, we will be working with euclidean distance.

        @return a tuple (nearest_edge, nearest_t) where
            * nearest_edge is a tuple (vid1, vid2), indicating the id of the origin and the destination vertices
            * nearest_t is a float in [0, 1], such that the nearest point along the edge to the given state is at
              distance nearest_t/length where length is the length of nearest_edge
        """
        nearest_dist = math.inf
        nearest_edge = None
        nearest_t = None

        for edge_id, (cost, edge) in self._edges.items():
            (sstar, tstar) = edge.get_nearest_point(state)
            dist = distance_computator.get_distance(sstar, state)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_edge = edge_id
                nearest_t = tstar

        return (nearest_edge, nearest_t)
    
    def get_nearest_vertex(self, state, distance_computator):
        """Return the id of the nearest vertex to the given state based on the given distance function
        @type distance_computator: a DistanceComputator object that includes the get_distance(s1, s2)
            function, which returns the distance between s1 and s2.
        """
        nearest_dist = math.inf
        nearest_vertex = None
        for vertex, s in self._vertices.items():
            dist = distance_computator.get_distance(s, state)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_vertex = vertex
        return nearest_vertex

    def get_nearest_vertices(self, state, k, distance_computator):
        """Return the ids of k nearest vertices to the given state based on the given distance function
        @type distance_computator: a DistanceComputator object that includes the get_distance(s1, s2)
            function, which returns the distance between s1 and s2.
        """
        dist_vertices = []
        for vertex, s in self._vertices.items():
            dist = distance_computator.get_distance(s, state)
            heappush(dist_vertices, (dist, vertex))

        nearest_vertices = [
            dist_vertices[i][1] for i in range(min(k, len(dist_vertices)))
        ]
        return nearest_vertices
    
    def split_edge(self, edge_id, t):
        """Split the given edge at distance t/length where length is the length of the edge

        @return the id of the new vertex at the splitted point
        """
        edge = self._edges[edge_id][1]
        (edge1, edge2) = edge.split(t)

        self.remove_edge(edge_id)

        s = edge1.get_destination()
        # TODO: Ideally, we should check that edge1.get_destination() == edge2.get_origin()
        v = self.add_vertex(s)
        self.add_edge(edge_id[0], v, edge1)
        self.add_edge(v, edge_id[1], edge2)

        return v
    
    def get_vertex_path(self, root_vertex, goal_vertex):
        """Run Dijkstra's algorithm backward to compute the sequence of vertices from root_vertex to goal_vertex"""

        class ZeroCostToGoEstimator:
            """Cost to go estimator, which always returns 0."""

            def get_lower_bound(self, x):
                return 0

        Q = QueueAstar(ZeroCostToGoEstimator())
        Q.insert(goal_vertex, None, 0)
        while len(Q) > 0:
            v = Q.pop()
            if v == root_vertex:
                vertex_path = Q.get_path(goal_vertex, root_vertex)
                vertex_path.reverse()
                return vertex_path
            for u in self.parent[v]:
                edge_cost = self.edges[(u, v)][0]
                Q.insert(u, v, edge_cost)
        return []  

    def get_path(self, root_vertex, goal_vertex) -> List[List[float]]:
        """Convert list of vertex IDs to a sequence of discretized states from root_vertex to goal_vertex"""
        vertex_path = self.get_vertex_path(root_vertex, goal_vertex)
        return self.get_path_from_vertex_path(vertex_path) 
    
    def get_path_from_vertex_path(self, vertex_path) -> List[List[float]]:
        """Run Dijkstra's algorithm backward to compute the sequence of vertices from root_vertex to goal_vertex"""

        if len(vertex_path) == 0:
            return []   
         
        path = []
        prev_vertex = vertex_path[0]
        for current_id in range(1, len(vertex_path)):
            current_vertex = vertex_path[current_id]
            edge = self.add_edge[(prev_vertex, current_id)][1]
            current_path = edge.get_path()
            path.extend(current_path)
            prev_vertex = current_vertex
            
        return path
    
    def draw(self):
        """Draw the graph on the axis ax"""
        for state in self.vertices.values():
            if (len(state)) == 2:
                plt.plot(state[0], state[1], "k.", linewidth=5)
            elif len(state) == 3:
                plt.plot(
                    state[0],
                    state[1],
                    marker=(3, 0, state[2] * 180 / math.pi - 90),
                    markersize=8,
                    linestyle="None",
                    markerfacecolor="black",
                    markeredgecolor="black",
                )

        for (_, edge) in self.edges.values():
            s2_ind = 1
            s1 = edge.get_discretized_state(s2_ind - 1)
            s2 = edge.get_discretized_state(s2_ind)
            while s2 is not None:
                plt.plot([s1[0], s2[0]], [s1[1], s2[1]], "k-", linewidth=1)
                s2_ind = s2_ind + 1
                s1 = s2
                s2 = edge.get_discretized_state(s2_ind)
    
class Tree(Graph):
    """A graph where each vertex has at most one parent"""
    def add_edge(self, vid1, vid2, edge):
        """Add an edge from vertex with id vid1 to vertex with id vid2"""
        # Ensure that a vertex only has at most one parent (this is a tree).
        assert len(self.parents[vid2]) == 0
        super().add_edge(vid1, vid2, edge)

    def get_vertex_path(self, root_vertex, goal_vertex):
        """Trace back parents to return a path from root_vertex to goal_vertex"""
        vertex_path = [goal_vertex]
        v = goal_vertex
        while v != root_vertex:
            parents = self._parent[v]
            if len(parents) == 0:
                return []
            v = parents[0]
            vertex_path.insert(0, v)
        return vertex_path


class ConnectedComponents(Graph):
    """An undirected graph that maintains connected components and incrementally updates it as an edge/vertex is added"""

    def __init__(self):
        super().__init__()
        self.components = []

    def get_component(self, v):
        """Return the index of the component of vertex v"""
        for ind, component in enumerate(self.components):
            if v in component:
                return ind
        raise ValueError

    def is_same_component(self, v1, v2):
        """Return whether vertices v1 and v2 are in the same connected component"""
        c1 = self.get_component(v1)
        c2 = self.get_component(v2)
        return c1 == c2

    def add_vertex(self, state) -> int:
        """Add vertex to roadmap and return its ID"""
        vertex_id = self.add_vertex(state)
        self.components.append([vertex_id])
        return vertex_id

    def add_edge(self, v_id1, v_id2, edge) -> None:
        """Add edge between vertices and update the connect componenets"""
        reverse_edge = copy.deepcopy(edge)
        reverse_edge.reverse()
        super().add_edge(v_id1, v_id2, edge)
        super().add_edge(v_id2, v_id1, reverse_edge)

        c1 = self.get_component(v_id1)
        c2 = self.get_component(v_id2)

        if c1 == c2:
            return

        self.components[c1].extend(self.components[c2])
        del self.components[c2]
    
    def remove_edge(self, edge):
        """remove_edge is not implemented in this class"""
        raise NotImplementedError
    