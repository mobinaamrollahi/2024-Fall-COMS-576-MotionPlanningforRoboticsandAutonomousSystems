from discrete_search import (
    StateSpace,
    ActionSpace,
    StateTransition,
    fsearch,
    ALG_BFS,
    ALG_ASTAR,
)


class CircularObstacle:
    """A class for representing a circular obstacle"""

    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r


class Grid2DStates(StateSpace):
    def __init__(self, Xmin, Xmax, Ymin, Ymax, O, r):
        """
        Xmin, Xmax, Ymin, Ymax: integers that defines the boundary of the world
        O:    a list of tuples (x, y, r) that represent the center (x,y) and radius (r) of the circular obstacle.
        r:     the radius of the robot
        """
        self.Xmin = Xmin
        self.Xmax = Xmax
        self.Ymin = Ymin
        self.Ymax = Ymax
        self.O = O
        self.r = r

    def __contains__(self, x):
        return self._is_in_workspace(x) and not (
            any(self._is_in_collision(x, o) for o in self.O)
        )

    def _is_in_collision(self, x, o):
        """Return true if and only if the robot collides with obstacle o when its center is at x"""
        return (x[0] - o.x) ** 2 + (x[1] - o.y) ** 2 <= (self.r + o.r) ** 2

    def _is_in_workspace(self, x):
        """Return true if and only if the robot is outside of the workspace"""
        return (
            x[0] >= self.Xmin + self.r
            and x[0] <= self.Xmax - self.r
            and x[1] >= self.Ymin + self.r
            and x[1] <= self.Ymax - self.r
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
    Xmax = 6
    Ymax = 5
    o1 = CircularObstacle(x=3.0, y=1.5, r=1.12)
    o2 = CircularObstacle(x=2.5, y=3.0, r=0.5)
    O = [o1, o2]
    X = Grid2DStates(0, Xmax, 0, Ymax, O, 0.5)
    f = GridStateTransition()
    U = Grid2DActions(X, f)
    xI = (1, 1)
    XG = [(5, 3)]

    accessible_points = [
        (x, y) for x in range(Xmax + 1) for y in range(Ymax + 1) if (x, y) in X
    ]
    path_result = fsearch(X, U, f, xI, XG, ALG_BFS)

    print("accessible grid points: {}".format(accessible_points))

    print("path: {}".format(path_result["path"]))
