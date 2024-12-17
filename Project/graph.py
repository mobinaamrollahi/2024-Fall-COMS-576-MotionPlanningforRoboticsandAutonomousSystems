from math import sqrt
import matplotlib.pyplot as plt
import math, copy
from edge import *
from heapq import heappush
from Queue import QueueAstar
import numpy

X_MAX = 800
Y_MAX= 600
'''
class Graph(object):
    """Graph class"""
    def __init__(self):
        self.vertices = set()
        self.adjacent = {}
        self.edges = []

    def add_vertex(self, vertex):
        """Add vertex to the graph."""
        self.vertices.add(vertex)
        """Add vertex to the graph."""
        if vertex not in self.adjacent:
            self.adjacent[vertex] = []

    def add_edge(self, v1, v2):
        """Add edge to the graph between vertices v1 and v2."""
        if v2 not in self.adjacent[v1]:
            self.adjacent[v1].append(v2)
        if v1 not in self.adjacent[v2]:
            self.adjacent[v2].append(v1)
        self.edges.append([v1, v2])


    def get_vertices(self):
        """Get all vertices of the graph."""
        return list(self.vertices)

    def get_adjacent(self, vertex):
        """Get list of adjacent vertices in the graph for the vertex"""
        if vertex in self.adjacent:
            return self.adjacent[vertex]
        return []

    def if_adjacent(self, v1, v2):
        """If v1 is adjacent to v2 returns True, otherwise returns False"""
        if v1 in self.adjacent[v2]:
            return True
        return False

    def remove_edge(self, v1, v2):
        """if v1 and v2 has edge between them, removes it and returns True, returns False otherwise"""
        if v2 in self.adjacent[v1]:
            #print("adj_bef: ", self.get_adjacent(v2))
            self.adjacent[v1].remove(v2)
            self.adjacent[v2].remove(v1)
            #print("adj_bef: ", self.get_adjacent(v2))
            try:
                self.edges.remove([v1, v2])
            except ValueError:
                self.edges.remove([v2, v1])
            return True
        return False

    def has_edge(self, v1, v2):
        """Returns True if there is an edge between vertices v1 and v2, False otherwise."""
        if v1 not in self.vertices or v2 not in self.vertices:
            return False
        return v2 in self.adjacent[v1]

    def get_edge_weight(self, v1, v2):
        """Returns distance from v1 to v2 op plot if v1 is adjacent to v2, otherwise returns False"""
        if v1 in self.adjacent[v2]:
            return sqrt((v2[0] - v1[0]) ** 2 + (v2[1] - v1[1]) ** 2)
        return False

    def get_edges(self):
        """Get all edges of the graph."""
        return self.edges

    def plot_edges(self, obstacles, path):
        fig, ax = plt.subplots()
        ax.set_xlim([0, X_MAX])
        ax.set_ylim([0, Y_MAX])

        for key, values in self.adjacent.items():
            if key in self.vertices:
                for value in values:
                    if value in self.vertices:
                        if [key, value] in path or [value, key] in path:
                            edge_color = "red"
                            A=1
                            line="-"
                        else:
                            edge_color = "blue"
                            A = 0.4
                            line = "--"

                        plt.plot(
                            [key[0], value[0]],
                            [key[1], value[1]],
                            linestyle=line,
                            color=edge_color,
                            alpha=A,
                        )

        for obstacle in obstacles:
            x_coords = [obstacle.x1, obstacle.x2, obstacle.x3, obstacle.x1]
            y_coords = [obstacle.y1, obstacle.y2, obstacle.y3, obstacle.y1]
            ax.plot(x_coords, y_coords, color='black')

'''

class Graph:
    """A class for maintaining a graph"""
    def __init__(self):
        # a dictionary whose key = id of the vertex and value = state of the vertex
        self.vertices = {}

        # a dictionary whose key = id of the vertex and value is the list of the ids of
        # its parents
        self.parents = {}

        # a dictionary whose key = (v1, v2) and value = (cost, edge).
        # v1 is the id of the origin vertex and v2 is the id of the destination vertex.
        # cost is the cost of the edge.
        # edge is of type Edge and stores information about the edge, e.g.,
        # the origin and destination states and the discretized points along the edge
        self.edges = {}

    def __str__(self):
        return "vertices: " + str(self.vertices) + " edges: " + str(self.edges)

    def add_vertex(self, state):
        """Add a vertex at a given state

        @return the id of the added vertex
        """
        vid = len(self.vertices)
        self.vertices[vid] = state
        self.parents[vid] = []
        return vid
    
    def add_parent(self, v, parent):
        """Set the parent of vertex v"""
        if v in self.parents:
            self.parents[v].append(parent)  # Append parent to the list of parents
        else:
            self.parents[v] = [parent]
    
    def get_parent(self, vertex):
        """Return the parent of the given vertex"""
        # Check if the vertex exists in the parents dictionary
        if vertex in self.parents:
            return self.parents[vertex]
        else:
            return None

    def get_vertex_state(self, vid):
        """Get the state of the vertex with id = vid"""
        return self.vertices[vid]

    def get_vertices(self):
        return list(self.vertices.keys())

    def add_edge(self, vid1, vid2, edge):
        """Add an edge from vertex with id vid1 to vertex with id vid2"""
        # print(f"In the graph.py. The edge that is passes is {edge}.")
        self.edges[(vid1, vid2)] = (
            edge.get_cost(),
            edge,
        )
        self.parents[vid2].append(vid1)
        

    def remove_edge(self, edge_id):
        """Remove a given edge

        @type edge: a tuple (vid1, vid2) indicating the id of the origin and the destination vertices
        """
        del self.edges[edge_id]
        v1 = edge_id[0]
        v2 = edge_id[1]
        self.parents[v2].remove(v1)

    def get_nearest(self, state, distance_computator, tol):
        """Return the vertex in the swath of the graph that is closest to the given state"""

        if len(self.edges) == 0:
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

        @return a tuple (nearest_edge, nearest_t) where
            * nearest_edge is a tuple (vid1, vid2), indicating the id of the origin and the destination vertices
            * nearest_t is a float in [0, 1], such that the nearest point along the edge to the given state is at
              distance nearest_t/length where length is the length of nearest_edge
        """
        nearest_dist = math.inf
        nearest_edge = None
        nearest_t = None

        for edge_id, (cost, edge) in self.edges.items():
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
        for vertex, s in self.vertices.items():
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
        for vertex, s in self.vertices.items():
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
        edge = self.edges[edge_id][1]
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
            for u in self.parents[v]:
                edge_cost = self.edges[(u, v)][0]
                Q.insert(u, v, edge_cost)
        return []

    def get_path(self, root_vertex, goal_vertex):
        """Return a sequence of discretized states from root_vertex to goal_vertex"""
        vertex_path = self.get_vertex_path(root_vertex, goal_vertex)
        return self.get_path_from_vertex_path(vertex_path)

    def get_path_from_vertex_path(self, vertex_path):
        """Return a sequence of discretized states along the given vertex_path"""
        if len(vertex_path) == 0:
            return []

        path = []
        prev_vertex = vertex_path[0]
        for curr_ind in range(1, len(vertex_path)):
            curr_vertex = vertex_path[curr_ind]
            edge = self.edges[(prev_vertex, curr_vertex)][1]
            curr_path = edge.get_path()
            path.extend(curr_path)
            prev_vertex = curr_vertex

        return path

    def draw(self, ax):
        """Draw the graph on the axis ax"""
        for state in self.vertices.values():
            if (len(state)) == 2:
                ax.plot(state[0], state[1], "k.", linewidth=5)
            elif len(state) == 3:
                ax.plot(
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
                ax.plot([s1[0], s2[0]], [s1[1], s2[1]], "k-", linewidth=1)
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
            parents = self.parents[v]
            if len(parents) == 0:
                return []
            v = parents[0]
            vertex_path.insert(0, v)
        return vertex_path


class GraphCC(Graph):
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

    def add_vertex(self, state):
        """Add a vertex at a given state and update the connected component

        @return the id of the added vertex
        """
        vid = super().add_vertex(state)
        self.components.append([vid])
        return vid

    def add_edge(self, vid1, vid2, edge):
        """Add an edge from vertex with id vid1 to vertex with id vid2 and update the connected component"""
        reverse_edge = copy.deepcopy(edge)
        reverse_edge.reverse()
        super().add_edge(vid1, vid2, edge)
        super().add_edge(vid2, vid1, reverse_edge)

        c1 = self.get_component(vid1)
        c2 = self.get_component(vid2)

        if c1 == c2:
            return

        self.components[c1].extend(self.components[c2])
        del self.components[c2]

    def remove_edge(self, edge):
        """remove_edge is not implemented in this class"""
        raise NotImplementedError
