import sys, argparse
import networkx as nx
from geometry import *
from planning import CircularMobileRobotPathPlanning
from draw_cspace import draw
import matplotlib.pyplot as plt

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
    
ALG_RRT = "rrt"
ALG_RRTS = "rrts"


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="Run sampling-based motion planning algorithm"
    )
    parser.add_argument(
        "--alg",
        choices=[ALG_RRT, ALG_RRTS],
        required=False,
        default=ALG_RRTS,
        dest="alg",
        help="algorithm, default to rrt",
    )
    args = parser.parse_args(sys.argv[1:])
    return args

if __name__ == "__main__":
    # Parameters
    Xmin = 0
    Xmax = 300
    Ymin = 0
    Ymax = 300 
    cspace = [(Xmin, Xmax), (Ymin, Ymax)]
    qI = [50, 50]
    qG = [260, 260]
    robot_radius = 10
    region_radius = 30
    N = 250
    tol=1e-3

    c = [(0, -1), (0, 1)]  # the center of each circle
    rho = 0.5

    # obstacles
    o1 = (200, 200, 30)
    o2 = (75, 180, 50)
    o3 = (180, 150, 20)

    obstacles = [o1, o2, o3]

    setup = CircularMobileRobotPathPlanning(
        robot_radius=robot_radius, 
        region_radius=region_radius,
        Xmin=Xmin,
        Xmax=Xmax,
        Ymin=Ymin,
        Ymax=Ymax,
        circular_obstacles=obstacles,
        qI=qI,
        qG=qG,
        N=N,
        tol=tol,
    )

    args = parse_args()

    if args.alg == ALG_RRT:
        (G, path) = setup.rrt( )

        # Plot the result
        fig, ax = plt.subplots()
        draw(
            ax,
            [(0, Xmax), (0, Ymax)],
            obstacles,
            region_radius,
            qI,
            qG,
            G,
            path,
            "RRT planning for a mobile planar robot, N = 10000",
            "RRT",
        )
        ax.set_xlabel('X', fontsize=20)
        ax.set_ylabel('Y', fontsize=20)
        plt.show()
        
        

    else:
        (G, path) = setup.rrtstar(
            d=2, 
            eta=5,
        )
        # Plot the result           
        fig, ax = plt.subplots()
        draw(
            ax,
            [(0, Xmax), (0, Ymax)],
            obstacles,
            region_radius,
            qI,
            qG,
            G,
            path,
            "RRT* planning for a mobile planar robot, N = 250, eta = 10",
            "RRTS",
        )
        ax.set_xlabel('X', fontsize=20)
        ax.set_ylabel('Y', fontsize=20)
        plt.show()
        
