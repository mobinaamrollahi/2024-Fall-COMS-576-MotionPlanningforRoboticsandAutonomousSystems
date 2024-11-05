import math
import numpy as np
from abc import ABC, abstractmethod
from typing import List, Tuple, Dict, Optional, Set
from geometry import get_euclidean_distance, get_nearest_point_on_line

class Edge:
    """
    A base class to store edge information, including the path corresponding to an edge.
    """
    def __init__(self, start_config: List[float], end_config: List[float], step_size: float):
        """
        Intialization
        @type start_config: a float indicating the state at the begining of the edge.
        @type end_config: a float indicating the state at the end of the edge.
        @type step_size: a float indicating the the length of each segment after discretization.
        """
        self._start = start_config
        self._end = end_config
        self.step_size = step_size

    @property
    def start_config(self) -> List[float]:
        return self._start
    
    @property
    def end_config(self) -> List[float]:
        return self._end
    
    @property
    def step_size(self) -> float:
        return self.step_size
    
    def get_length(self) -> float:
        """Return the length of the edge"""
        raise NotImplementedError    

    def get_cost(self) -> float:
        return self.get_length()
    
    def get_path(self):
        """Return the path, representing the geometry of the edge"""
        return [self._start, self._end]
    
    def reverse(self):
        """Reverse the origin/destination of the edge"""
        tmp = self._start
        self._start = self._end
        self._end = tmp

    def get_discretized_state(self, i):
        """Return the i^{th} discretized state"""
        raise NotImplementedError

    def get_nearest_point(self, state):
        """Compute the nearest point on this edge to the given state

        @return (s, t) where s is the point on the edge that is closest to state
        and it is at distance t*length from the beginning of the edge
        """
        raise NotImplementedError

    def split(self):
        """Split the edge at distance t/length where length is the length of this edge

        @return (edge1, edge2) edge1 and edge2 are the result of splitting the original edge
        """
        raise NotImplementedError
    
    def get_config_at(self, t: float) -> List[float]:
        return [
            self._start[0] + t * (self._end[0] - self._start[0]),
            self._start[1] + t * (self._end[1] - self._start[1])
        ]
    
    
    
class EdgeStraight(Edge):
    """Store the information about an edge representing a straight line between 2 points"""

    def __init__(self, start_config: List[float], end_config: List[float], step_size):
        super().__init__(start_config, end_config, step_size)

        # Store useful information so that they do not need to be recomputed
        self.line_segment = end_config - start_config  # The line segment from start_config to end_config
        self.length = get_euclidean_distance(
            self.start_config, self.end_config
        )  # the length of this line segment
        self.tstep = min(step_size / self.length, 1)  # for discretization purpose

        # The number of discretized state
        self.num_discretized_states = math.ceil(self.length / step_size) + 1

    def reverse(self):
        """Reverse the origin/destination of the edge"""
        super().reverse()
        self.line_segment = self.end_config - self.start_config

    def get_discretized_state(self, i):
        """Return the i^{th} discretized state"""
        if i == 0:
            return self.start_config
        if i == self.num_discretized_states - 1:
            return self.end_config
        if i >= self.num_discretized_states:
            return None

        return self.start_config + (i * self.tstep) * self.line_segment

    def get_nearest_point(self, state):
        """Compute the nearest point on this edge to the given state

        @return (s, t) where s is the point on the edge that is closest to state
        and it is at distance t*length from the beginning of the edge
        """
        return get_nearest_point_on_line(self.start_config, self.end_config, state)

    def split(self, t):
        """Split the edge at distance t/length where length is the length of this edge

        @return (edge1, edge2) edge1 and edge2 are the result of splitting the original edge
        """
        s = self.start_config + t * self.line_segment
        return (
            EdgeStraight(self.start_config, s, self.step_size),
            EdgeStraight(s, self.end_config, self.step_size),
        )

    def get_length(self):
        """Return the length of the edge"""
        return self.length
