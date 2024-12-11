import math
from geometry import is_inside_circle


class Obstacle:
    def contain(self, s):
        return False


class CircularObstacle:
    """A class for representing a circular obstacle"""

    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r

    def get_boundary(self):
        """Return the list of coordinates (x,y) of the boundary of the obstacle"""
        num_theta = 100
        theta_inc = 2 * math.pi / num_theta
        theta_range = [theta_inc * i for i in range(num_theta + 1)]
        return [
            (
                self.r * math.cos(theta) + self.x,
                self.r * math.sin(theta) + self.y,
            )
            for theta in theta_range
        ]

    def contain(self, s):
        """Return whether a point s is inside this obstacle"""
        center = (self.x, self.y)  # Create a tuple for the center (x, y)
        return is_inside_circle(center, self.r, s)


class WorldBoundary2D(Obstacle):
    """A class representing the world"""

    def __init__(self, xlim, ylim):
        self.xmin = xlim[0]
        self.xmax = xlim[1]
        self.ymin = ylim[0]
        self.ymax = ylim[1]

    def contain(self, s):
        """Return True iff the given point is not within the boundary (i.e., the point is
        "in collision" with an obstacle.).
        """
        return (
            s[0] < self.xmin or s[0] > self.xmax or s[1] < self.ymin or s[1] > self.ymax
        )


def construct_circular_obstacles(dt):
    r = 1 - dt  # the radius of the circle
    c = [(0, -1), (0, 1)]  # the center of each circle
    t = [(0, math.pi), (-math.pi, 0)]  # range of theta of each circle
    obstacles = []
    for i in range(len(c)):
        print(CircularObstacle(c[i], r))
        obstacles.append(CircularObstacle(c[i], r, t[i]))
    return obstacles
