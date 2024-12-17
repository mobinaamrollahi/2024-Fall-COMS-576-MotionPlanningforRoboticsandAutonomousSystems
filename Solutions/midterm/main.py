import math
import matplotlib.pyplot as plt
from draw_cspace import draw
from midterm import CircularMobileRobotPathPlanning


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


if __name__ == "__main__":
    # Parameters
    robot_radius = 0.5
    Xmax = 6
    Ymax = 5
    qI = (1, 1)
    qG = (5, 3)

    # obstacles
    o1 = CircularObstacle(x=3.0, y=1.5, r=1.12)
    o2 = CircularObstacle(x=2.5, y=3.0, r=0.5)
    circular_obstacles = [o1, o2]
    setup = CircularMobileRobotPathPlanning(
        robot_radius=robot_radius,
        Xmax=Xmax,
        Ymax=Ymax,
        circular_obstacles=circular_obstacles,
    )

    (G, path) = setup.plan(qI, qG)

    print(path)

    # Plot the result
    fig, ax = plt.subplots()
    draw(
        ax,
        [(0, Xmax), (0, Ymax)],
        [obs.get_boundary() for obs in circular_obstacles],
        qI,
        qG,
        G,
        path,
        "RRT planning for a mobile planar robot",
    )
    ax.set_xlabel(r"$x$", fontsize=20)
    ax.set_ylabel(r"$y$", fontsize=20)
    plt.show()
