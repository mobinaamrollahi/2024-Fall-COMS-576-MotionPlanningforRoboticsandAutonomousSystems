import json, sys, os, argparse
import numpy as np
from discrete_search import (
    StateSpace,
    ActionSpace,
    StateTransition,
    fsearch,
    ALG_BFS,
    ALG_ASTAR,
)


class Grid2DStates(StateSpace):
    def __init__(self, Xmin, Xmax, Ymin, Ymax, O):
        """
        Xmin, Xmax, Ymin, Ymax: integers that defines the boundary of the world
        O:    a list of tuples that represent the grid points
              that are occupied by an obstacle.
              A tuple (i, j) in O means that grid point (i,j) is occupied.
        """
        self.Xmin = Xmin
        self.Xmax = Xmax
        self.Ymin = Ymin
        self.Ymax = Ymax
        self.O = O

    def __contains__(self, x):
        return (
            x[0] >= self.Xmin
            and x[0] <= self.Xmax
            and x[1] >= self.Ymin
            and x[1] <= self.Ymax
            and x not in self.O
        )

    def get_distance_lower_bound(self, x1, x2):
        """Return the lower bound on the distance between the given states x1 and x2"""
        return sum([abs(x1[i] - x2[i]) for i in range(len(x1))])


class GridStateTransition(StateTransition):
    def __call__(self, x, u):
        return tuple([x[i] + u[i] for i in range(len(x))])


class Grid2DActions(ActionSpace):
    all_actions = [(0, 1), (0, -1), (-1, 0), (1, 0)]

    def __init__(self, X, f):
        self.X = X
        self.f = f

    def __call__(self, x):
        return [u for u in Grid2DActions.all_actions if self.f(x, u) in self.X]


if __name__ == "__main__":
    O = [(1,3), (2,3), (1,2), (1,1), (2,1), (3,1)]
    X = Grid2DStates(0, 4, 0, 4, O)
    f = GridStateTransition()
    U = Grid2DActions(X, f)
    xI = (0,0)
    XG = [(4,4)]

    result = fsearch(X, U, f, xI, XG, ALG_BFS)

    print("visited: {}".format(result["visited"]))
    print("path:    {}".format(result["path"]))
